# Firmware Power Optimization

Power work on `firmware.ino` after the decision **not** to use a coordinated
collection trigger. This tracks what has landed and the backlog of ideas still
worth exploring, so nothing gets lost. Current baseline draw (from
`IDLE_WAKE_SOURCE.md`): ~12 mA IDLE (DETECTOR), ~8 mA IDLE (SIGMOTION), ~22 mA
ACTIVE recording (full fusion).

## Landed

These are in the firmware now. None change the recorded data format or the state
machine's logic; each is commented at its site.

1. **DC/DC regulator enabled** — `NRF_POWER->DCDCEN = 1` at the top of `setup()`.
   Switches the nRF52840's main supply from the default LDO to the internal buck
   converter (the XIAO populates the REG1 inductor). More efficient in every
   state, most of all with the radio active. *Board-wide; bench-measure the
   delta.*

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
   lower power, no mag calibration, immune to magnetic interference from
   batteries / other on-body nodes / metal; the cost is slow yaw drift (pitch and
   roll stay solid). The 20-byte record format is identical, so nothing
   downstream changes. **This is a data-quality decision — collect a run on each
   build and compare before committing to GRV.** When building the GRV path,
   compile-check it against the installed SparkFun library version (the
   `getGameQuat*` getter names follow the current API but were not compiled here).

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

- **QSPI flash deep power-down during IDLE.** `qspiConfig()` leaves DPM disabled,
  so the P25Q16H sits in standby current continuously — but IDLE never writes
  flash. Issue the chip's deep-power-down on IDLE entry and wake it on ACTIVE
  entry to remove flash standby draw across the whole idle period.

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
