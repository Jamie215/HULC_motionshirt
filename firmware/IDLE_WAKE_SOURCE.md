# IDLE Wake Source — why the Stability Detector

Follow-up to `state_machine_test/FINDINGS.md`. The investigation proved the
periodic ~6.6 s IDLE reset is gated by the BNO086's **fusion engine**
(MotionEngine): accelerometer-only sensors self-reboot in idle; anything that
runs fusion (gyro involved) does not.

This doc records **which wake source the production firmware uses and why**. The
short version: `firmware.ino` uses the **Stability Detector (0x1C)** for IDLE and
accepts the ~6.6 s reset as a benign, understood tradeoff.

## The decision

IDLE arms the **Stability Detector (0x1C)** — accelerometer-only — and puts the
BNO hub in devSleep (`IDLE_USE_DEVSLEEP = 1`). This is the lowest-power idle at
~7 mA. Its cost is the inherent ~6.6 s self-reboot while asleep, which is
harmless: IDLE logs nothing, and the firmware re-arms the detector automatically
on each reset (see `handleIdle()`).

The one real risk of the reset — that it lands mid-I²C transaction and leaves the
bus stuck (SDA held low) — is insured against by `i2cBusRecover()` in `setup()`,
which stays valuable regardless of wake source.

### The alternative we considered and did not adopt

The **Stability Classifier (0x13)** — accel + gyro through the MotionEngine — does
**not** self-reset in idle, because the running fusion engine keeps the hub
active. It was the reset-free alternative. We did not adopt it because:

| | Stability Detector `0x1C` (adopted) | Stability Classifier `0x13` (not adopted) |
|---|---|---|
| Sensing | accelerometer only | accel + gyro (MotionEngine) |
| Idle reset | ~6.6 s (inherent, benign) | none — fusion keeps the hub active |
| devSleep | holds it → ~7 mA idle | running MotionEngine can't hold it |
| Idle current | lowest | higher (gyro on, no devSleep) |
| Motion trigger | detector `EXITED` report | classifier value `== MOTION` |

The detector's lower idle current won out, since the reset it costs is benign.
See `FINDINGS.md` for the full four-sensor comparison (detector, raw
accelerometer, classifier, Rotation Vector) behind this table.

## History: this used to be a compile-time A/B switch

Earlier revisions of `firmware.ino` exposed both sources through an
`IDLE_WAKE_SOURCE` compile-time switch so the two could be A/B compared on the
bench. That switch — along with its diagnostic instrumentation — has been
**removed**: the detector is the settled choice, and keeping a second, rarely
built path (whose `handleIdle()` motion-wake logic had drifted out of sync) was
more liability than flexibility.

## If the reset ever does become a problem

If a future requirement makes the ~6.6 s reset unacceptable (e.g. it starts
causing real bus stalls the recovery routine can't absorb, or the idle-current
budget grows enough to afford the gyro), moving IDLE to the classifier is a
deliberate, well-understood change:

1. In `enableIdleReports()`, arm `imu.enableStabilityClassifier(interval)` instead
   of the `sh2_setSensorConfig(0x1C)` detector path, and set
   `IDLE_USE_DEVSLEEP = 0` (the MotionEngine can't hold devSleep).
2. In `handleIdle()`, wake on the classifier's `MOTION` value (reuse `isMotion()`,
   as `STATIC_POSTURE`/`ACTIVE_RECORDING` already do) instead of the detector's
   `EXITED` report.

That unifies the motion definition across all three states and removes the idle
reset, at the cost of higher idle current.
