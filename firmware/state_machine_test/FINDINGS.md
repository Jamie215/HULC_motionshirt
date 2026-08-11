# BNO086 IDLE Reset Investigation — Findings

Plain-language summary of what we set out to learn, what we tried, and where we
landed. Companion to `state_machine_test.ino` (the stripped-down test harness
used for all experiments below).

## The question

When the motion shirt sits still in IDLE (low-power, waiting for movement), the
BNO086 sensor **restarts itself about every ~6.5 seconds**. We wanted to know
**why**, and whether we could stop it or make it happen less often.

## The short answer

- The restart is **real and repeatable** — the sensor genuinely reboots.
- We **could not eliminate it from firmware.** Nothing we changed in software
  stopped it.
- It is **almost certainly NOT a fundamental property of the chip** — if it
  were, every BNO086 project would report it, and they don't.
- Best evidence points to an **I²C communication / driver issue specific to
  this BNO086 + nRF52840 setup**, not the sensor itself. This is **strongly
  suspected but unconfirmed** (see "What we didn't test").
- **Practically it's harmless:** in IDLE nothing is being recorded, the firmware
  re-arms the sensor automatically, and no motion data is lost.

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

## Most likely explanation

An **I²C transport / driver-level reset**, specific to BNO086-on-nRF52840 —
things like I²C clock-stretching quirks, SHTP (the sensor's packet protocol)
framing, or the driver's own reset logic. This fits all the evidence:

- Not reported elsewhere → specific to this combination, not the chip.
- Hit **both** the detector and the plain accelerometer → transport-level, not
  sensor-specific.
- Polling perfectly didn't fix it → not about servicing speed.
- We independently watched the driver hard-reset the chip when communication got
  confused (the probe accident).

## What we didn't test (the honest gap)

- **SPI instead of I²C.** This is the one experiment that would confirm-or-kill
  the transport theory: SPI avoids I²C's clock stretching and framing quirks
  entirely. It was **not pursued** (requires rewiring the board). So the
  transport explanation stays *strongly suspected but unproven*.
- **Library update** is not an option: we're already on the latest SparkFun
  BNO08x library (v1.0.6), and its changelog has no relevant I²C/reset fix.

## Does it matter, and what to do

- **In normal use: no.** IDLE logs nothing, the firmware auto-recovers, no motion
  data is lost. The restart was only ever a concern for idle power and log noise,
  both minor.
- **One real risk worth guarding against:** if a restart ever lands in the middle
  of an I²C transaction, it can leave the bus stuck (SDA held low) and **freeze
  the device until power-cycle** — the worst outcome for an unattended wearable.
  We haven't observed this (restarts kept auto-recovering), but it's a
  probabilistic edge case worth insuring against with an **I²C bus-recovery
  routine** (detect a stuck bus, pulse the clock 9× to free it, re-init).

## Recommendation

Accept the ~6.5 s IDLE restart as a **benign, handled event** (it already is —
the state machine re-arms on reset). Add **I²C deadlock recovery** to the
production firmware as cheap insurance for unattended reliability. Revisit **SPI**
only if the restart ever proves to cause real data loss or if idle power becomes
critical.
