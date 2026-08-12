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
#define IDLE_WAKE_SOURCE      IDLE_WAKE_CLASSIFIER   // or IDLE_WAKE_DETECTOR
```

| | `IDLE_WAKE_DETECTOR` (original) | `IDLE_WAKE_CLASSIFIER` (new default) |
|---|---|---|
| Sensor | Stability Detector `0x1C` | Stability Classifier `0x13` |
| Sensing | accelerometer only | accel + gyro (MotionEngine) |
| Idle reset | **Yes, ~6.6 s** (inherent) | **No** — fusion keeps the hub active |
| `IDLE_USE_DEVSLEEP` | `1` (auto) — hub sleeps ~7 mA | `0` (auto) — MotionEngine can't hold devSleep |
| Idle current | Lowest | Higher (gyro running, no devSleep) |
| Motion trigger | detector `EXITED` report | classifier value `== MOTION` |

`IDLE_USE_DEVSLEEP` is now derived from the wake source automatically — you only
set `IDLE_WAKE_SOURCE`. Selecting the classifier also unifies the motion
definition: IDLE now wakes on the **same** `MOTION` classification that
`STATIC_POSTURE` and `ACTIVE_RECORDING` already act on.

## What "compare the performance" measures

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

## Recommendation

Use the classifier when a reset-free, robust idle matters more than the last few
mA — it removes the periodic reboot (and with it the small risk of a
reset landing mid-I²C transaction and stalling the bus). Keep the detector build
for the lowest-power idle where the harmless periodic re-arm is acceptable. The
`i2cBusRecover()` insurance in `setup()` stays valuable either way.
