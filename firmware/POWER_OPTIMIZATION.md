# Firmware Power Optimization

Power work on `firmware.ino` after the decision **not** to use a coordinated
collection trigger. This tracks what has landed and the backlog of ideas still
worth exploring, so nothing gets lost. Pre-optimization baseline (from
`IDLE_WAKE_SOURCE.md`): ~12 mA IDLE (DETECTOR), ~22 mA ACTIVE recording (full
fusion).

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

5. **External flash deep power-down during IDLE.** `qspiSleep()` puts the P25Q16H
   into deep power-down (`0xB9`) whenever the node sits in unconnected IDLE (which
   never touches flash); `qspiWake()` releases it (`0xAB`) before any access — on
   BLE connect (offload/erase arrive as control commands), at the top of
   `applyPendingTransition()` for the header write, and at end of `setup()`;
   re-park on IDLE entry and on disconnect back into IDLE. **Saving is small** —
   serial-NOR standby is ~tens of µA against the ~9 mA idle, below meter
   resolution — so don't expect the idle figure to move; this is completeness,
   not a lever.

   **Tried and reverted: nRF QSPI *peripheral* deactivation.** Also deactivating
   the nRF's QSPI peripheral (`nrfx_qspi_uninit()` on sleep, re-init on wake) was
   implemented and bench-tested, and it made idle **WORSE — ~9 mA → ~11 mA**.
   Cause: **nRF52840 Errata 122, "QSPI uses current after being disabled"** — a
   *disabled* QSPI peripheral leaks ~1–2 mA unless Nordic's errata workaround (an
   extra write to an undocumented power register) is applied, and the Seeed mbed
   core's nrfx does not apply it. So `nrfx_qspi_uninit()` handed us the leak, not
   a saving. **That code was removed** for readability; only the harmless chip
   deep-power-down above remains. If ever revisited, the peripheral disable must
   be paired with the Errata 122 register workaround and re-measured — but idle
   here is dominated by the BNO hub (~8 mA), so the ceiling on any QSPI-side win
   is small regardless.

6. **Tier B: STATIC-specific slow RV.** The RUNNING config is no longer shared
   at one rate — `configureBNO_Running()` now takes a `BnoRunningCfg`
   (`BNO_CFG_ACTIVE` / `BNO_CFG_STATIC`) and STATIC arms the fusion vector at
   `BNO_RV_STATIC_INTERVAL_MS` (1000 ms, ~1 Hz) instead of the ACTIVE 100 ms
   (~10 Hz). STATIC logs only 0.2 Hz, so even the Tier A 10 Hz RV shipped ~50
   reports per logged sample — each a wasted I²C read + nRF wake; ~1 Hz cuts
   those RV wakeups ~10× while still giving 5 orientation samples per logged
   snapshot (a logged posture is at most ~1 s stale, negligible for a static
   hold). **The Classifier is untouched — it keeps running at
   `ACTIVE_STABILITY_MS` (500 ms) in both RUNNING states, so it (not the RV)
   still drives every transition and motion detection out of STATIC is
   unchanged.** No change to the recorded data format or the 0.2 Hz log rate.

   Structural notes (this was the "needs careful validation" backlog item):
   - The old `bnoInRunningMode` bool couldn't tell an ACTIVE↔STATIC switch (rate
     must change) from a same-state re-entry (skip). It's replaced by the
     `BnoRunningCfg` enum (`NONE`/`ACTIVE`/`STATIC`), so same-state re-entry is
     still skipped, a fresh entry from IDLE / post-reset (`NONE`) enables *both*
     reports, and an ACTIVE↔STATIC switch re-issues *only* the fusion vector at
     the new rate — the fewest `enableReport`s, to stay off the command-buffer /
     premature-reset path (SparkFun issue #2). The `delay(50)` ack guard is kept.
   - The enum type lives in Section 10 (with `SystemState`), ahead of the first
     function, so the Arduino auto-generated prototype for the now-typed
     `configureBNO_Running(BnoRunningCfg)` sees it defined.
   - The classifier-floor point: because the Classifier still wakes the nRF at
     ~2 Hz in STATIC, the STATIC wake rate drops from ~12 Hz (10 RV + 2 clf) to
     ~3 Hz (1 RV + 2 clf) — a ~4× cut in nRF wakeups, not the full 10× the RV
     alone changes by. Taking the RV below the classifier's 2 Hz would buy little.

   **Bench measurement still pending** (developed without hardware in the loop):
   verify the STATIC current drop, and — per the issue-#2 caution — watch for any
   premature BNO reset across repeated ACTIVE↔STATIC transitions now that the
   fusion vector is re-issued on each. Update the Measured results table with the
   STATIC figure once taken.

7. **Significant Motion (0x12) wake source — retained as A/B option; blocked on
   sensitivity, not power.** SIGMOTION is kept as a third `IDLE_WAKE_SOURCE`.
   **Measured idle ~7.4 mA vs the detector's ~9 mA on the optimized build — a
   real ~1.6 mA (~18%) saving.** (Ignore the ~8 mA figure once recorded in an
   earlier AI-authored docs commit; it was never reproduced. The current numbers
   are the measured ones.)

   Note this *system* win coexists with the datasheet putting the SIGMOTION
   sensor **above** the detector at the chip level (Figure 6-18): Significant
   Motion ~0.48 mA vs Stability Detector ~0.06 mA. Both are true — the node saves
   ~1.6 mA not because the sensor is cheaper (it isn't) but because the one-shot
   event avoids the detector's ~1 Hz heartbeat and ~6.6 s reboot churn waking the
   nRF. The system behavior dominates the tiny sensor delta.

   **The blocker is sensitivity, and it is a signal mismatch, not a threshold.**
   Bench testing: SIGMOTION fires on shaking / dropping the node, but does
   **nothing** when a held arm is slowly stretched — the exact slow, deliberate
   motions this device exists to capture. Significant Motion is a high-pass
   motion-*energy* detector (Android semantics) built to reject gentle handling;
   a slow stretch is low-energy, so it is below threshold by design. Its
   threshold is not exposed, and lowering motion-energy sensitivity would invite
   false wakes (each false IDLE→ACTIVE runs full fusion ~20 mA for ~70 s before
   timing back to IDLE, which erases the 1.6 mA saving and logs junk) without
   reliably catching slow *rotation* — accel energy is simply the wrong axis for
   slow limb movement. The detector (default) catches these because its
   stability/tilt behavior responds to the slow gravity-vector reorientation a
   stretch produces.

   **Path forward (see backlog "tunable low-power wake that keeps slow-stretch
   capture").** The real goal is the detector's sensitivity without its ~1 Hz
   heartbeat. Candidates: an **on-change Accelerometer** (`changeSensitivity`)
   that stays silent when still but wakes on any accel change (tilt or movement
   transient), or the **detector reported on-change** rather than at 1 Hz. Both
   need a bench A/B (power + does-it-catch-a-slow-stretch) before replacing the
   detector default. Until one is proven, DETECTOR stays default and SIGMOTION is
   the lowest-power option only where slow-motion wake latency is acceptable
   (e.g. off-body standby).

## Backlog — worth exploring

Ordered roughly by payoff. Each needs bench time or a design decision, so they
were deliberately left out of the safe batch above.

- **Tunable low-power wake that keeps slow-stretch capture (chase SIGMOTION's
  ~1.6 mA).** SIGMOTION idles ~1.6 mA under the detector purely because it stays
  silent when still — no ~1 Hz heartbeat waking the nRF (see Landed item 7) — but
  it misses slow held-limb stretches (it thresholds motion *energy*, and a slow
  stretch is low-energy). The detector already catches those stretches on
  accelerometer alone, so the real goal is "detector sensitivity, minus the
  heartbeat." Candidates, all needing a bench A/B (idle current AND
  does-it-catch-a-slow-stretch on-body):
  - **On-change Accelerometer** (`sh2_setSensorConfig` `changeSensitivity`): wake
    when the acceleration reading moves by a tuned delta. It watches the *whole*
    accel vector, so it catches both a slow tilt (gravity direction shifting) and
    the start/stop transients of any real limb movement — the same two signals the
    detector uses — while staying silent when still. Gives a real
    power/sensitivity dial the fixed SIGMOTION threshold does not. Needs custom
    wake logic (baseline compare) and likely a drop to the `sh2` driver (not
    exposed by the SparkFun helper).
  - **Detector reported on-change instead of at 1 Hz** — same proven sensor, just
    stop the heartbeat so it only pokes the nRF on a stability change. Lowest
    effort. **Under test now** via `IDLE_DETECTOR_QUIET` (Section 4): the interval
    is stretched to 60 s so the detector is effectively silent-when-still while
    EXITED still fires immediately on motion. The old warnings against this — a
    stretched interval worsening the self-reset toward the ~1 s floor, and dropped
    motion wakes — both date from the removed devSleep path, so they are being
    re-measured hub-awake, not assumed. `handleIdle()` now logs ms-since-last-reset
    so the bench shows immediately whether the reset stays ~6.5 s (good) or falls
    toward ~1 s (abandon); the slow-stretch wake must also still fire.

  Narrow blind spot (largely theoretical for a body-worn sensor): a *pure*,
  constant-speed rotation about the vertical, with the node sitting exactly on the
  rotation axis, moves neither gravity nor produces linear accel — only a gyro
  (classifier) would catch it. On a limb the node is off-axis, so real movement
  always produces some accel (that the accel-only detector already catches
  stretches is the proof), so this rarely bites. The SH-2 Tilt Detector would be a
  clean fit but is **not** in the BNO086 sensor list (datasheet Fig 6-18) — don't
  count on it.

- **Reconfigure BNO rate on watermark throttle.** When flash fills,
  `writeQuaternionSample()` halves `activeHz` (10→5→2), but that only changes the
  software gate — the BNO keeps emitting 10 Hz, so the throttle saves flash but
  not IMU/nRF power. Reconfiguring the RV report rate when `activeHz` changes
  would capture the power saving too. Now cheaper to do safely: Landed item 6
  (Tier B) added the per-transition fusion-only reconfigure path
  (`configureBNO_Running(BnoRunningCfg)`), so this could re-issue just the fusion
  vector at the throttled rate the same way (mind the same SparkFun issue #2
  re-issue caution).

- **QSPI peripheral deactivation — tried, backfired, removed (see item 5).**
  Measured ~9 → ~11 mA idle from nRF52840 Errata 122; the code was removed. Only
  worth revisiting with the Errata 122 register workaround applied after the
  disable, and even then the BNO-dominated idle caps the win — low priority.

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
