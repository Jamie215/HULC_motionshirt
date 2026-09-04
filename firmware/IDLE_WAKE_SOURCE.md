# IDLE Wake Source — Detector, Classifier, Significant Motion

## The switch

`firmware.ino`, Section 3:

```c
#define IDLE_WAKE_SOURCE   IDLE_WAKE_DETECTOR   // DETECTOR | CLASSIFIER | SIGMOTION
```

## Third option: Significant Motion (0x12) — lowest power, but misses slow motion

`IDLE_WAKE_SIGMOTION` arms the BNO08x's one-shot Significant Motion sensor
(`0x12`): its mere arrival means motion started (no value to decode), and it
auto-disables after firing, so `enableIdleReports()` re-arms it on each IDLE
entry.

**Measured (optimized build): ~7.4 mA idle vs the detector's ~8.8 mA — ~1.4 mA
lower.** SIGMOTION is lower because, unlike the detector, it does **not** run the
~6.7 s idle self-reset (the SIGMOTION build prints no `IDLE: BNO reset` lines).
That system behavior is the difference, not the sensor itself — the datasheet
(Fig 6-18) puts the SIGMOTION sensor *above* the detector at the chip level
(~0.48 mA vs ~0.06 mA).

**Sensitivity is the blocker.** Bench testing: SIGMOTION fires on shaking or
picking up the node, but does **nothing** when a held arm is slowly stretched.
Significant Motion thresholds motion *energy* (Android semantics, to reject
gentle handling), a slow stretch is low-energy, and the threshold is not exposed
for tuning. So SIGMOTION is suitable only where slow-motion wake latency is
acceptable (e.g. off-body standby), **not** as the default for a device that must
catch slow deliberate motion — which is why DETECTOR is the default.

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

Selecting the classifier also unifies the motion definition: IDLE now wakes on
the **same** `MOTION` classification that `STATIC_POSTURE` and
`ACTIVE_RECORDING` already act on.

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

## Resolution: devSleep was explored and dropped

Bench testing (and comparing against the original working firmware) found the
Stability Detector **stopped waking on motion** once the low-power **devSleep**
path was added: with the hub asleep, the EXITED wake event is coalesced/rarely
delivered (shaking only occasionally printed a `0x1C val=` line, state never
changed). Two regressions compounded it: the report interval was stretched
1 s → 10 s, and the `_US` (microsecond) interval was passed to
`enableReport()`, which takes **milliseconds** (asking for a report every
~2.8 h).

Because devSleep suppressed the motion wake, we **opted out of it entirely** and
run the hub awake. The devSleep path has since been **removed from the firmware**
(the `DETECTOR_DIAG_NO_DEVSLEEP` / `IDLE_USE_DEVSLEEP` switch, the
`sh2_setSensorConfig()` wake/always-on arming, the `modeSleep()`/`modeOn()`
calls, and the `IDLE_DETECTOR_INTERVAL_US` constant are all gone). All wake
sources now arm via `enableReport()` with the hub awake, and the default
`IDLE_WAKE_SOURCE` is `IDLE_WAKE_DETECTOR` reported on-change (~8.8 mA idle,
harmless periodic re-arm). The `getStabilityClassifier()` read was **not** the
problem — the original working code uses the identical read.

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
