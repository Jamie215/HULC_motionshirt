# Setup & Calibration — Design Plan (pipeline stages 5–7)

Planning doc for the **not-yet-built** stages of the analysis pipeline, and the
record of the design decisions behind them. Stages 1–4 (capture → offload →
reconcile → montage/resolve) exist in `firmware/` and `tools/`; this covers what
comes after, in build order:

- **§2 Configuration & node identity** — how a node knows/records where it is.
- **§3 Time offset vs. mounting calibration** — two different quantities, two
  different lifetimes. The single most important distinction here.
- **§4 Calibration (stage 5)** — the sensor→segment solve, cache-and-verify.
- **§5 Free-body diagram (part of stage 7)** — the natural visual payoff.

See `pipeline_walkthrough.html` for the whole pipeline at a glance, and
`MONTAGE_SCHEMA.md` for the montage/resolver already built.

---

## 1. Anchor facts (from the firmware, not assumptions)

- Nodes log the **full Rotation Vector** (`enableRotationVector()` → report
  `0x05`), which is **magnetometer-referenced**. Every node's orientation is
  therefore already in a **shared world frame** (gravity + magnetic north), not
  the arbitrary per-node heading a Game Rotation Vector would give.
- Each sample is timestamped with the node's free-running **`millis()`**, which
  starts at zero on **power-up**. There is no persistent real-time clock.
- Nodes advertise a unique identity `HULC-IMU-XXXX` (from the BLE MAC); the
  offloaded file is named by it.

These three facts drive everything below.

---

## 2. Configuration & node identity

**Decision: separate "which node is where" (config) from "how the sensor sits on
the bone" (calibration).** They have different lifetimes (config is stable across
wears if sensors live in fixed garment pockets; calibration is per-don), so they
are different artifacts, not one step.

### 2.1 Persist the segment assignment on the node

**Decision: store the segment as a 1-byte enum in a per-log *header*, not per
record, and not as a string.**

- Per-record would waste flash; a single header field per session is negligible
  against a 2 MB flash of 20-byte records. The "abbreviate to save space" worry
  only existed under a per-record assumption.
- A `uint8` segment code (`0=torso, 1=upper_arm_l, 2=upper_arm_r, …`) costs one
  byte. It makes the **offloaded file self-describing** — the file already says
  *which node*; the header adds *which body part* — which removes the
  mislabel-after-offload risk.
- Source-of-truth rule: the node header is the **default**, a host montage may
  **override**, and reconcile **warns on disagreement** (same pattern as the
  landmark check). Better: **auto-populate the montage from the headers** so it
  stops being hand-typed and the user only confirms.

### 2.2 The configuration stage (front end of montage + calibration)

The montage **is** the configuration; the config stage is the setup UX that
produces it. Fold three things into one "don the shirt" flow:

1. **Assign** each node to a location (writes/confirms the montage; may write the
   assignment to the node per §2.1).
2. **Verify placement** live via the free-body diagram (§5) — a mislabeled or
   slipped node is visible immediately.
3. **Calibrate** — strike the neutral pose (§4).

**Optional upgrade — auto-assignment.** Because nodes report absolute
orientation, a scripted setup ("raise your right arm") can detect *which node
moved* and assign it automatically, turning the error-prone manual mapping into
confirmation. Prototype after the manual path works.

---

## 3. Time offset vs. mounting calibration (the key distinction)

Two quantities are refreshed at the start of a wear. They are **independent** and
must not be conflated:

| | Aligns | Invalidated by | Cacheable? | Refreshed how |
|---|---|---|---|---|
| **Time offset** | the nodes' clocks (temporal) | every **power-up** | **No** | automatic BLE clock read at connect, or a shared sync gesture |
| **Mounting calibration** | sensor→segment (spatial) | every **re-don** | **Yes** | a ~2 s neutral pose, or reuse-if-verified |

- **Time offset cannot be cached.** Each node's `millis()` restarts at zero on
  boot, so a previous session's offset is meaningless after a power cycle. It is
  re-anchored every session — but on a well-behaved central this is *automatic*
  (the BLE clock-offset read needs no user action). Within a session it holds
  (drift negligible over minutes).
- **Mounting calibration survives a power cycle** (it is a physical relationship,
  not a clock) and is broken only by physically re-wearing the garment — so it
  *can* be cached and verified.
- A charge-then-rewear trips **both**, for different reasons (the power cycle
  resets clocks; the re-don shifts straps). That coincidence is why bundling them
  into one session-start step is convenient — not extra burden, since the
  session-start moment already exists for timing.

**Correction this supersedes:** the sync gesture is per-*session*, not
once-forever. "Once at the beginning" means the beginning of each wear/session,
because power-up resets the clocks.

---

## 4. Calibration (stage 5) — the sensor→segment solve

**What it computes:** the mounting offset for each node — the rotation from the
sensor's frame to its segment's anatomical frame. The mag-referenced world frame
is *given*, so this is a sensor-to-segment calibration, **not** a "resting
quaternion" capture. See `MONTAGE_SCHEMA.md` §4 for the three jobs the neutral
pose does (solve offset, set zero, heading co-registration fallback).

**Cache-and-verify contract (decided).** The stage-5 routine emits **both**:

1. the per-segment mounting offset (→ `calibration.json`), and
2. a **staleness / consistency check** — enough to decide, on the next don,
   *reuse vs. re-pose*: during the first still moment, test whether the cached
   offsets are still self-consistent in the shared world frame (e.g. calibrated
   segment axes still point where a quiet-standing pose implies). Small residual
   → reuse silently; over threshold → prompt for a fresh ~2 s pose. The same
   check catches **intra-session slippage**, not just re-dons.

**Cheapening ladder** (build 1–2 first; 3–4 later):

1. **Fold into the session-start move** you already do for timing — the pose *is*
   the calibration, not a separate chore.
2. **Cache + verify** (above) — the "feels like skipping" path.
3. **Functional / auto-calibration from motion** — recover the offset from a
   short bout of ordinary movement (gravity gives vertical; a joint's gyro axis
   gives the hinge), removing the deliberate pose. More algorithm; needs each
   joint to move.
4. **Better mounting** (rigid, anatomically-shaped pockets) shrinks don-to-don
   variation, widening how often #2 can reuse before a re-pose.

**Emits:** `calibration.json` — per-segment offset + captured window + the
consistency baseline. Gates every angle/ROM metric; flips the resolver's
`calibrated` flags from relative-only to clinical.

---

## 5. Free-body diagram (part of stage 7) — feasibility

The most feasible visual, because **orientation is exactly what is measured** — a
quaternion per segment per frame directly drives an oriented 3-D body. It maps
one-to-one onto the resolver's tiers:

- **Segment tier (1 node)** → each segment drawn as its own oriented body,
  floating. A literal free-body diagram; needs only the calibrated orientation
  (or raw, if the mounting tilt is acceptable). **Build this first** — it
  validates stages 5–6 visually.
- **Joint tier (2 adjacent nodes)** → connect them at the joint, render the angle
  between them. A linkage, not floating bodies.
- **Chain (torso→arm→forearm→hand)** → a connected skeleton via forward
  kinematics.

**The honest caveat:** orientation is measured; **position/connection is
modeled**. Connecting segments into a skeleton needs the kinematic chain (the
montage), **assumed segment lengths**, and calibration to make them line up.
Root the torso, place each segment's proximal end at its parent's distal end,
orient by the quaternion, step down the chain. Without calibration the segments
still render but won't sit anatomically — which is useful: **the FBD from the
neutral pose is how you *see* whether calibration worked.**

---

## 6. Build order

1. **Stage 5 calibration** with the cache-and-verify contract (§4) — the gate for
   every clinical angle.
2. **Floating-segment FBD** (§5, segment tier) — validates §1 visually, cheap.
3. **Firmware node header** (§2.1) + config-stage UX (§2.2) — self-describing
   logs, montage auto-populated.
4. **Metric plugins** (stage 6) over the calibrated stream — ROM first, then the
   rest already declared by the resolver.
5. **Connected FBD / skeleton + full interface** (stage 7).
