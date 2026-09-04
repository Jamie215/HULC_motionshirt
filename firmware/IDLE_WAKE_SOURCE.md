# IDLE Wake Source — Detector, Classifier, Significant Motion

## The switch

`firmware.ino`, Section 3:

```c
#define IDLE_WAKE_SOURCE   IDLE_WAKE_DETECTOR   // DETECTOR | CLASSIFIER | SIGMOTION
```

To A/B: build each of `IDLE_WAKE_DETECTOR` / `IDLE_WAKE_CLASSIFIER` /
`IDLE_WAKE_SIGMOTION`, exercise on the bench with Serial open, and see which
reliably transitions IDLE → ACTIVE_RECORDING (watch for the
`DETECTOR: 0x1C val=` / `CLASSIFIER: MOTION` / `SIGMOTION:` lines) — test slow
motions, not just shakes.

| | `IDLE_WAKE_DETECTOR` (default) | `IDLE_WAKE_CLASSIFIER` (alt) | `IDLE_WAKE_SIGMOTION` (alt) |
|---|---|---|---|
| Sensor | Stability Detector `0x1C` | Stability Classifier `0x13` | Significant Motion `0x12` |
| Sensing | accelerometer only (on-change) | accel + gyro (MotionEngine) | accelerometer only (one-shot) |
| Idle reset | ~6.7 s (inherent, benign) | **No** — fusion keeps the hub active | **No** |
| Idle current | ~8.8 mA | Higher (gyro running) | **~7.4 mA** (lowest) |
| Slow-motion sensitivity | Good | Good | **Poor** — misses slow held-limb motion |
| Motion trigger | detector `EXITED` report | classifier value `== MOTION` | event fires (one-shot) |

## Measuring idle self-reset

`handleIdle()` logs each self-reset with the interval since the previous one:

```
IDLE: BNO reset — <ms> since last reset — re-arming detector
```

- **Detector build** — a steady stream, ~6.7 s apart, whenever the device sits
  idle.
- **Classifier / SIGMOTION builds** — the line essentially never prints; those
  sources don't self-reset. That absence *is* the result.

## Bench procedure

1. Build with `IDLE_WAKE_SOURCE = IDLE_WAKE_DETECTOR`, flash, leave the device
   still in IDLE for a few minutes, capture serial. Note the reset interval from
   the `IDLE: BNO reset — <ms> since last reset` lines and (with a meter) idle
   current.
2. Rebuild with `IDLE_WAKE_SOURCE = IDLE_WAKE_CLASSIFIER`, repeat.
3. Compare: classifier should show **zero idle resets** at the cost of **higher
   idle current**. Confirm motion still wakes IDLE→ACTIVE promptly (the
   classifier reaches `MOTION` within one `IDLE_STABILITY_MS` interval, default
   1 s) — tune `IDLE_STABILITY_MS` for the latency/power balance you want.

**Net:** low-power idle on this chip means hub-awake accel-only — ~8.8 mA in IDLE
(detector), rising to ~20–21 mA while ACTIVE recording (full fusion). devSleep
would be lower on paper but suppresses the motion wake, so it is not used.
Getting below the detector's idle figure while keeping slow-motion capture would
need a hardware wake (a separate low-power accel/motion interrupt waking the nRF,
which then powers the BNO), not the BNO's own devSleep.

## Recommendation

Use the **detector** (default) for on-body capture: it is the only source that
reliably wakes on slow, deliberate motion, at ~8.8 mA. Use **SIGMOTION** only for
off-body standby, where its ~1.4 mA lower idle is worth losing slow-motion
sensitivity. Use the **classifier** when a reset-free, robust idle matters more
than the current — it removes the periodic reboot (and with it the small risk of
a reset landing mid-I²C transaction and stalling the bus), at higher idle
current. The `i2cBusRecover()` insurance in `setup()` stays valuable in any case.

## Tuning SIGMOTION sensitivity (potential future investigation — NOT done)

Open question: could SIGMOTION be made sensitive enough for on-body slow-motion
wake, giving us its ~7.4 mA (reset-free, ~1.4 mA under the detector) without
losing slow held-limb stretches? Findings so far, from the SH-2 driver headers
and the SH-2 Reference Manual (the manual PDF was not fully read — the exact
record byte layout is still TODO):

- **A threshold DOES exist.** Earlier notes calling it a "fixed, non-exposed"
  threshold were wrong. The sh2 driver defines an FRS configuration record
  `SIG_MOTION_DETECT_CONFIG` (0xC274); the manual describes it as holding an
  **acceleration threshold** to trigger and a **timeout/window**, as 32-bit
  integer / fixed-point words.
- **The SparkFun library can't write it.** `SparkFun_BNO08x_Arduino_Library`
  exposes no public FRS-write method (its `frs*` helpers are commented-out and
  private). `enableReport(sensor, interval_us, sensorSpecific)` exposes a
  `sensorSpecific` config word — reachable at zero cost and worth a bench probe —
  but detector thresholds normally live in the FRS record, not that word, so it
  is likely a no-op for SIGMOTION.
- **Even tuned, it may be the wrong tool.** Significant Motion is an
  energy-over-a-window detector (Android-style). Slow held-limb stretches are
  low-acceleration and may not sustain across the window, so they can be missed
  even at a low threshold; lowering the threshold far enough to catch them risks
  false wakes on tiny vibration/noise, defeating the low-power purpose.

If pursued, the work is: (1) get the exact `0xC274` record layout (word order,
units, defaults) from the SH-2 Reference Manual figure "Significant Motion
Detector Configuration Record"; (2) add an FRS-write path (extend the SparkFun
library, or drive the CEVA `sh2_setFrs()` HAL directly); (3) bench-sweep the
threshold against real slow held-limb motion vs. false-wake rate. Until then,
CLASSIFIER remains the recommended reset-free on-body source.

Sources: sh2 driver `sh2.h` (FRS record IDs); SH-2 Reference Manual (CEVA),
"Significant Motion Detector Configuration Record".
