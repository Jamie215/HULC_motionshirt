# Firmware Power Optimization

Power work on `firmware.ino` after the decision **not** to use a coordinated
collection trigger. This tracks what has landed and the backlog of ideas still
worth exploring, so nothing gets lost. Pre-optimization baseline (from
`IDLE_WAKE_SOURCE.md`): ~12 mA IDLE (DETECTOR), ~8 mA IDLE (SIGMOTION), ~22 mA
ACTIVE recording (full fusion).

## Measured results (bench, DETECTOR idle wake source)

After the "Landed" changes below:

| Build | IDLE | ACTIVE |
|---|---|---|
| Baseline (pre-optimization) | ~12 mA | ~22 mA |
| Optimized, Rotation Vector (default) | **~9 mA** | **~20–21 mA** |
| Optimized, Game Rotation Vector | **~9 mA** | **~20 mA** |

- **IDLE ~12 → ~9 mA (~25%)** — from the DC/DC regulator and the slower/gated
  advertising. IDLE is identical across the RV and GRV builds, as expected: the
  fusion source only affects the RUNNING states, so the `ACTIVE_FUSION` switch
  cannot move idle (a useful confirmation the switch is scoped correctly).
- **ACTIVE ~22 → ~20–21 mA** — from the DC/DC regulator and the report-rate match.
- **GRV vs RV: no meaningful power difference** (~20 vs ~20–21 mA, within
  measurement noise). Expected in hindsight — on the BNO08x the always-running
  gyro and the fusion MotionEngine dominate; the magnetometer is comparatively
  cheap, so dropping it saves little. **GRV is therefore a data-quality option,
  not a power lever** (see item 4).

## Landed

These are in the firmware now. None change the recorded data format or the state
machine's logic; each is commented at its site.

1. **DC/DC regulator enabled** — `NRF_POWER->DCDCEN = 1` at the top of `setup()`.
   Switches the nRF52840's main supply from the default LDO to the internal buck
   converter (the XIAO populates the REG1 inductor). More efficient in every
   state, most of all with the radio active. Board-wide; a large part of the
   measured idle and active drop (see Measured results).

2. **BLE advertising made cheap and IDLE-only.**
   - Advertising interval slowed to ~1 s (`ADV_INTERVAL_UNITS = 1600`, units of
     0.625 ms) — cuts advertising radio-on time ~10× vs the fast default. Offload
     is occasional and IDLE-only, so ~1 s discovery latency is fine.
   - Advertising is **gated to IDLE**: `applyPendingTransition()` re-advertises on
     IDLE entry and calls `stopAdvertise()` entering the RUNNING states; the
     disconnect handler only re-advertises when in IDLE. Offload is rejected
     outside IDLE anyway, so advertising while recording bought nothing and spent
     radio power at the IMU's peak draw.

3. **Report rate matched to log rate (Tier A).** `BNO_RV_INTERVAL_MS` 65 ms
   (~15 Hz) → 100 ms (10 Hz). The BNO no longer computes and ships ~5 fusion
   samples/sec that ACTIVE's `activeHz` gate discarded (each a wasted I²C read +
   nRF wake). Also drops STATIC_POSTURE (shared config, logs only 0.2 Hz) from
   ~15 Hz to 10 Hz. No change to recorded data.

4. **Game Rotation Vector A/B switch** — `ACTIVE_FUSION_SOURCE` (Section 3b),
   defaulting to `ACTIVE_FUSION_RV` so behavior is unchanged until flipped.
   `ACTIVE_FUSION_GRV` selects the 6-axis Game Rotation Vector (no magnetometer):
   no mag calibration, immune to magnetic interference from batteries / other
   on-body nodes / metal; the cost is slow yaw drift (pitch and roll stay solid).
   The 20-byte record format is identical, so nothing downstream changes.
   **Bench-measured: GRV gives no meaningful power saving vs RV** (see Measured
   results) — the gyro + fusion engine dominate, the magnetometer is cheap. So
   **decide GRV purely on data quality**, not power: keep the default RV unless a
   run of captured orientation data shows magnetometer-driven heading corruption
   (likely once multiple nodes + batteries sit close together on the body), in
   which case GRV trades absolute heading for cleaner, interference-free
   orientation. When building the GRV path, compile-check it against the installed
   SparkFun library version (the `getGameQuat*` getter names follow the current
   API but were not compiled here).

5. **Flash powered down during IDLE — two layers.** `qspiSleep()` runs whenever
   the node sits in unconnected IDLE (which never touches flash); `qspiWake()`
   runs before any access — on BLE connect (offload/erase arrive as control
   commands), at the top of `applyPendingTransition()` for the header write, and
   at end of `setup()`; re-park on IDLE entry and on disconnect back into IDLE.
   - **Layer 1 — external chip deep power-down** (`0xB9` / `0xAB`). Correct but
     tiny: serial-NOR standby is ~tens of µA against the ~9 mA idle, below meter
     resolution. Don't expect the idle figure to move for this alone.
   - **Layer 2 — nRF QSPI *peripheral* deactivation** (`nrfx_qspi_uninit()` on
     sleep, `nrfx_qspi_init()` + WREN on wake), gated by
     `QSPI_DEACTIVATE_PERIPHERAL`. Wake ordering is deliberate: **re-init the
     peripheral first, then release the chip, then re-assert WREN.**
     **❌ Measured WORSE and disabled by default (flag = 0)** — see below.

   **Bench result — layer 2 backfires on this core.** Enabling
   `QSPI_DEACTIVATE_PERIPHERAL` raised idle **~9 mA → ~11 mA** (active unchanged
   within noise, since the peripheral is never disabled while logging). Cause:
   **nRF52840 Errata 122, "QSPI uses current after being disabled"** — a *disabled*
   QSPI peripheral leaks ~1–2 mA unless Nordic's errata workaround (an extra write
   to an undocumented power register) is applied, and the Seeed mbed core's nrfx
   does not apply it. So `nrfx_qspi_uninit()` handed us the leak, not a saving.
   Layer 2 is left in the code (correct logic, gated off) but **off by default**;
   layer 1 stays on (harmless). To ever revisit, add the Errata 122 register
   workaround after the disable and re-measure — but note idle here is dominated
   by the BNO hub (~8 mA), so the ceiling on any QSPI-side win is small.

## Backlog — worth exploring

Ordered roughly by payoff. Each needs bench time or a design decision, so they
were deliberately left out of the safe batch above.

- **Tier B: STATIC-specific slow RV.** STATIC logs only 0.2 Hz but (even after
  Tier A) runs fusion at 10 Hz — still ~50 reports read and discarded per logged
  sample. Dropping STATIC's RV to 1–2 Hz would cut its wakeups ~5–10× more. The
  catch is structural: `configureBNO_Running()` is shared and guarded by
  `bnoInRunningMode`, which skips reconfiguration on ACTIVE↔STATIC re-entry, so
  this needs a STATIC-specific config path (extend the guard to distinguish
  "running-active" vs "running-static", apply once per transition, reuse the
  existing `delay(50)`-guarded enable sequence). Re-issuing `enableReport` on
  every transition is exactly what risked the premature-reset behavior seen
  before (SparkFun issue #2), so this one needs careful validation. Transitions
  are driven by the classifier (500 ms), not the RV, so a slow STATIC RV only
  affects posture-snapshot freshness, not motion detection.

- **Reconfigure BNO rate on watermark throttle.** When flash fills,
  `writeQuaternionSample()` halves `activeHz` (10→5→2), but that only changes the
  software gate — the BNO keeps emitting 10 Hz, so the throttle saves flash but
  not IMU/nRF power. Reconfiguring the RV report rate when `activeHz` changes
  would capture the power saving too. (Shares the reconfigure-safety concern
  above.)

- **QSPI peripheral deactivation — tried, backfired (item 5 layer 2).** Measured
  ~9 → ~11 mA idle from nRF52840 Errata 122; disabled by default. Only worth
  revisiting with the Errata 122 register workaround applied after the disable,
  and even then the BNO-dominated idle caps the win — low priority.

- **Sleep instead of spin while connected-idle.** In `waitForIMUData()`, the
  BLE-connected branch busy-loops on `BLE.poll()` for up to 100 ms with no sleep,
  so the core never idles while a central is connected. A `__WFE()` between polls
  would let it sleep between radio/BNO events. Relatedly, the fast 15–30 ms
  connection interval is held for the whole connection but only needed during
  offload — relaxing it except around a transfer cuts connected-idle radio
  wakeups.

- **Production build: drop USB/serial.** `DEBUG_SERIAL 1` keeps USB CDC (and
  HFCLK) alive. Note the `Serial.print`/`Serial.begin` calls in the QSPI helpers
  and `setup()` are **not** behind the macro, so `DEBUG_SERIAL 0` alone doesn't
  fully shed USB. Gate those too for a deployed build to remove the USB draw.

- **fsync tuning.** `FSYNC_EVERY = 10` does a 4 KB sector erase + write + readback
  every ~1 s at 10 Hz — erases are power-hungry and wear flash. Raising it trades
  a little more data-at-risk on power cut for fewer erases; the readback verify
  could also be made less frequent.

- **LEDs (hardware).** The SparkFun BNO086 breakout's red LED and the XIAO
  nRF52840's own power LED are **power indicators wired to the 3.3 V rail, not
  GPIOs** — not firmware-controllable. Disable by cutting the board's `LED`
  solder jumper (per node, reversible). ~1–3 mA each, on 24/7. Worth doing on
  every deployed node. In firmware, the blue "connected" LED (`PIN_LED_BLUE`) is
  held on for the whole connection — could be dropped or briefly blinked during
  long offloads.

- **Below hub-awake idle needs hardware.** Per `IDLE_WAKE_SOURCE.md`, the BNO's
  own devSleep suppresses the motion wake, so idle floor on this chip is
  hub-awake accel-only. Going lower would need a separate low-power motion
  interrupt waking the nRF, which then powers the BNO — a hardware change, out of
  scope for firmware alone.
