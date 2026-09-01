# IDLE Wake Source — Detector vs Classifier

Follow-up to `state_machine_test/FINDINGS.md`. The investigation proved the
periodic ~6.6 s IDLE reset is gated by the BNO086's **fusion engine**
(MotionEngine): accelerometer-only sensors self-reboot in idle; anything that
runs fusion (gyro involved) does not. The **Stability Classifier (0x13)** runs
accel+gyro through the MotionEngine and did **not** reset in the test harness.

This change carries that finding into the production firmware (`firmware.ino`):
the Stability Classifier can now be the IDLE state-changing flag, and the two
wake sources are a compile-time A/B switch with matching instrumentation.

## The switch

`firmware.ino`, Section 3:

```c
#define IDLE_WAKE_SOURCE   IDLE_WAKE_CLASSIFIER   // DETECTOR | CLASSIFIER | SIGMOTION
```

## Third option: Significant Motion (0x12)

Added after bench testing found the **Stability Detector did not wake on a
shake** (state never left IDLE, nothing logged) — while the Classifier worked.
`IDLE_WAKE_SIGMOTION` arms the **Significant Motion** sensor (`0x12`), the
BNO08x's purpose-built, low-power, **one-shot** wake-on-motion event: its mere
arrival means motion started (no value to decode), and it auto-disables after
firing, so `enableIdleReports()` re-arms it on each IDLE entry / reset.

Like the Detector it is **accel-based** and runs hub-awake, so the hub still
self-reboots ~6.6 s in idle and re-arms — SigMotion is a bet that the *wake
event* is delivered more reliably than the Detector's ENTERED/EXITED transition,
not a fix for the reboot. (devSleep was explored and dropped — it suppressed the
motion wake; see below.)

To A/B: build each of `IDLE_WAKE_DETECTOR` / `IDLE_WAKE_SIGMOTION` /
`IDLE_WAKE_CLASSIFIER`, shake on the bench with Serial open, and see which
reliably transitions IDLE → ACTIVE_RECORDING (watch for the
`SIGMOTION:` / `DETECTOR: 0x1C val=` / `CLASSIFIER: MOTION` lines).

| | `IDLE_WAKE_DETECTOR` (original) | `IDLE_WAKE_CLASSIFIER` (new default) |
|---|---|---|
| Sensor | Stability Detector `0x1C` | Stability Classifier `0x13` |
| Sensing | accelerometer only | accel + gyro (MotionEngine) |
| Idle reset | **Yes, ~6.6 s** (inherent) | **No** — fusion keeps the hub active |
| Idle current | ~12 mA (hub awake, accel-only) | Higher (gyro running) |
| Motion trigger | detector `EXITED` report | classifier value `== MOTION` |

Selecting the classifier also unifies the motion definition: IDLE now wakes on
the **same** `MOTION` classification that `STATIC_POSTURE` and
`ACTIVE_RECORDING` already act on.

## Measuring the difference: idle-reset stats

`handleIdle()` records every self-reset and prints a running summary on each one:

```
IDLE reset [<source>]: held <ms> — drained <n> post-reset events ...
IDLE reset stats: n=<count>  mean=<ms>  min=<ms>  max=<ms>
```

- **Detector build** — expect a steady stream of these, `mean` ≈ 6600 ms,
  one every ~6.6 s the device sits idle.
- **Classifier build** — expect the `IDLE reset` line to essentially never
  print while idle (`n` stays 0). That absence *is* the result.

Sub-1 s holds are excluded from the stats as post-arm settle noise, matching the
test-harness A/B convention.

## Bench procedure

1. Build with `IDLE_WAKE_SOURCE = IDLE_WAKE_DETECTOR`, flash, leave the device
   still in IDLE for a few minutes, capture serial. Note the reset `n`/`mean`
   and (with a meter) idle current.
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
sources now arm via `enableReport()` at `IDLE_DETECTOR_INTERVAL_MS = 1000` with
the hub awake (~12 mA idle, harmless periodic re-arm), and the default
`IDLE_WAKE_SOURCE` is `IDLE_WAKE_DETECTOR`. The `getStabilityClassifier()` read
was **not** the problem — the original working code uses the identical read.

**Net:** low-power idle on this chip means hub-awake accel-only — ~12 mA in IDLE,
rising to ~22 mA while ACTIVE recording (full fusion). devSleep would be lower on
paper but suppresses the motion wake, so it is not used. Getting below the
hub-awake idle figure would need a hardware wake (a separate low-power
accel/motion interrupt waking the nRF, which then powers the BNO), not the BNO's
own devSleep.

## Recommendation

Use the classifier when a reset-free, robust idle matters more than the last few
mA — it removes the periodic reboot (and with it the small risk of a
reset landing mid-I²C transaction and stalling the bus). Keep the detector build
for the lowest-power idle where the harmless periodic re-arm is acceptable. The
`i2cBusRecover()` insurance in `setup()` stays valuable either way.
