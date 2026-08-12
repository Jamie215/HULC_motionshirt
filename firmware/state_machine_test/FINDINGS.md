# BNO086 IDLE Reset Investigation — Findings

Plain-language summary of what we set out to learn, what we tried, and where we
landed. Companion to `state_machine_test.ino` (the stripped-down test harness
used for all experiments below).

## The question

When the motion shirt sits still in IDLE (low-power, waiting for movement), the
BNO086 sensor **restarts itself about every ~6.5 seconds**. We wanted to know
**why**, and whether we could stop it or make it happen less often.

## The short answer (updated — see "Best explanation")

- The restart is **real and repeatable** — the sensor genuinely reboots.
- It is **NOT a fundamental flaw of the chip**, and **NOT a generic I²C bug**
  (an earlier draft of this doc guessed I²C transport — that was wrong).
- **The deciding factor is the FUSION ENGINE.** Sensors that don't run the
  BNO's fusion pipeline (the Stability Detector and the raw Accelerometer)
  restart every ~6.5 s. The **Rotation Vector — a fused output — does NOT
  restart** (streamed 300+ reports with zero resets). Running fusion keeps the
  hub fully active, so it never enters the idle state that self-resets.
- **Why nobody else reports it:** almost everyone streams a fused output (like
  Rotation Vector) continuously, so their hub never idles. We only hit it
  because IDLE deliberately uses a lightweight detector to save power.
- **Practically it's harmless:** IDLE records nothing, the firmware re-arms
  automatically, and ACTIVE recording (which streams Rotation Vector) does
  **not** restart at all. The restart is the *price of the low-power IDLE wake
  sensor* — a power-vs-restart tradeoff, not a bug.

## How we investigated

We built a stripped-down test sketch — just the state machine, no Bluetooth, no
flash storage — so we could study the restart in isolation and print exactly
when it happened. Then we changed one thing at a time.

| What we tried | Result | What it ruled out |
|---|---|---|
| Turned OFF the sensor's deep-sleep (ran it fully awake) | Still restarts ~6.5s | It's not caused by sleep/devSleep |
| Swept how often the sensor reports (50 ms → 5 s) | Restart stayed ~6.5s (when reporting fast) | The restart clock doesn't track the report rate |
| Polled the sensor constantly vs. sleeping between reports | **Identical** ~6.5s | It's not the host being too slow or asleep |
| Used a plain accelerometer instead of the motion detector | Still ~6.5s | It's not about misusing the detector as a wake source |
| Raised I²C speed to 400 kHz | **Much worse** — reset every ~130 ms + corruption | This board's I²C is unstable at 400 kHz (weak pull-ups); 100 kHz is the sane setting |
| Streamed the **Rotation Vector** (fused quaternions) instead of the detector | **NO reset** — 300+ reports, ran clean | THE key result: fused output doesn't restart; the reset is specific to non-fusion sensors |

### The one interesting mechanism we found (the "race")

When the sensor is barely reporting, it restarts in ~1 second. If it manages to
get a report out within that ~1 second window, it survives much longer —
~6.5 seconds. So there are effectively two timers: a short "nothing's happening"
one (~1 s) and a hard ceiling (~6.5 s). Reporting faster reaches the 6.5 s
ceiling but **can never push past it**. This directly answered the original
idea ("slow the reports down to restart less often") — it's backwards: slower
reporting makes it *worse*, and faster reporting caps out at 6.5 s.

## What the datasheet told us

- The only "reset if the host is too slow" watchdog in the datasheet is on the
  **older BNO080**. On our **BNO085/086, being slow causes a retry, NOT a
  reset.** So that popular theory doesn't apply to our chip.
- The datasheet documents **no periodic/self-reset** for the 085/086. Per the
  docs, it shouldn't be restarting on its own — which is another hint that
  something about *our setup* is triggering it.

## A mistake we caught (and the lesson)

We added a probe to read the sensor's "reset cause" fresh on every restart. It
reported **"External Reset"** — briefly exciting. But it turned out **the probe
itself was causing those resets**: the low-level call it used confused the
driver, and the driver recovered by hard-resetting the chip. So that reading was
an **artifact, not the real cause.** We removed the probe.

- **Lesson:** the act of measuring changed the thing being measured. The true
  cause of the *original* 6.5 s restart is still unknown.
- **Silver lining:** it accidentally proved the **driver WILL reset the chip when
  its I²C/SHTP communication gets confused** — which supports the "driver /
  transport reset loop" explanation below.

## Best explanation (revised after the Rotation Vector test)

**The restart is an idle-state behaviour of the BNO086, gated by whether the
fusion engine (MotionEngine) is running.**

- Lightweight sensors that don't run fusion — the **Stability Detector** and the
  **raw Accelerometer** — leave the hub in a low-activity state, and it
  self-restarts every ~6.5 s.
- The **Rotation Vector** is a *fused* output; enabling it runs the fusion
  pipeline continuously, keeping the hub fully active. Streamed 300+ reports
  with **zero restarts**.

Confirmed across all four sensors we could arm in IDLE:

| Sensor | Uses | Runs MotionEngine? | Restarts? |
|---|---|---|---|
| Stability Detector (0x1C) | accelerometer only | No | **Yes, ~6.5 s** |
| Raw Accelerometer (0x01)  | accelerometer only | No | **Yes, ~6.5 s** |
| Stability Classifier (0x13)| accelerometer + gyro | **Yes** | **No** |
| Rotation Vector (0x05)    | accel + gyro + mag | **Yes** | **No** |

Clean rule: **accelerometer-only sensors restart; anything that runs the
MotionEngine (gyro involved) does not.** Datasheet 2.4.1 confirms the detector
is accelerometer-only and "lower power than the stability classifier."

This fits every result with no contradictions:

- Detector and raw accelerometer restart → neither runs the MotionEngine.
- Stability Classifier and Rotation Vector don't restart → both run it.
- It's **not** the report rate — the detector restarted even at 20 Hz, faster
  than the Rotation Vector's 15 Hz. So it's the sensor *type* (MotionEngine vs.
  not), not how often it reports. The classifier confirms this: it's mostly
  quiet on the wire while still, yet doesn't restart — because the *engine* is
  running, not because it's streaming reports.
- Not reported elsewhere → everyone streams a MotionEngine output continuously,
  so their hub never idles.

NOTE — an earlier version of this doc concluded "I²C transport / driver-level
reset." **That was wrong**: a transport-level reset would hit the Rotation
Vector too, and it doesn't. The I²C angle was a dead end; the fusion-engine
state is the real differentiator. (The 400 kHz instability and the probe's
self-inflicted resets were separate, real I²C effects — but not the cause of
the ~6.5 s idle restart.)

## Loose ends (minor)

- **Duration of RV immunity:** the Rotation Vector ran reset-free for ~20 s
  (300+ reports) — enough to break the 6.5 s pattern decisively, but a multi-
  minute run would confirm it's truly indefinite, not just a longer interval.
- **SPI:** no longer relevant. It was proposed to test an I²C-transport theory
  that the Rotation Vector result has since disproved.

## Does it matter, and what to do

- **In normal use: no.** ACTIVE recording streams the Rotation Vector, which does
  **not** restart. IDLE (the detector) restarts, but logs nothing and
  auto-recovers, so no motion data is lost.
- **The restart is a power-vs-restart tradeoff, not a bug.** The IDLE wake source
  is a knob, not a right/wrong choice:

  | IDLE wake option | Restarts? | Idle power | Notes |
  |---|---|---|---|
  | Stability Detector (current) | Yes ~6.5 s | Lowest (accel-only, devSleep-friendly) | Restart is harmless in IDLE |
  | Stability Classifier | **No** | Higher (gyro on, MotionEngine — likely can't devSleep) | Still stability-based; quiet on the wire while stationary |
  | Rotation Vector | **No** | Highest (full fusion) | Overkill for a wake source |

  The detector was chosen for lowest power, and its restart is the price. The
  **Stability Classifier is the reset-free middle ground** if the restart ever
  becomes a problem — but it runs the gyro/MotionEngine, so idle current rises
  and it likely can't be combined with the hub's devSleep.
- **One real risk worth guarding against:** if a restart ever lands mid-I²C
  transaction, it can leave the bus stuck (SDA held low) and **freeze the device
  until power-cycle**. Not observed (restarts kept auto-recovering), but a cheap
  **I²C bus-recovery routine** insures against it.

## Recommendation

Accept the ~6.5 s IDLE restart as a **benign, understood tradeoff** (low-power
detector wake in exchange for a harmless periodic re-arm). Keep the **I²C
bus-recovery** insurance in the production firmware. Only move IDLE to a
fusion-based wake (no restarts, higher power) if the restart ever proves to
cause real problems.
