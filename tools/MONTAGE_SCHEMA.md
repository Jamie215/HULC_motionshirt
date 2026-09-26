# Montage Schema & Capability Resolution

Companion to `motion_capabilities.py`. Defines the **montage** — the
declarative record of where each IMU node is placed for a session — and the
**capability resolver** that maps a montage to the set of joints and metrics
that can actually be computed from it.

This is the contract every motion/ROM metric builds against. It sits one layer
above `reconcile_nodes.py`: reconcile puts the nodes on one timeline; the
montage says *what body part each of those aligned streams is*, and the
resolver says *what that placement lets you compute*.

```
   per-node .bin logs
        │  reconcile_nodes.py  (time-align → aligned.csv: t_common_ms, n0_*, n1_* ...)
        ▼
   aligned.csv  ── montage.json ──►  motion_capabilities.py
        │                                    │  (which joints/metrics are valid)
        ▼                                    ▼
   metric plugins (ROM, smoothness, dwell, ...) consume BOTH
```

---

## 1. Why a montage is required (and not optional metadata)

An IMU node measures the **orientation of the segment** it is strapped to. It
does **not** measure a joint. A clinical joint angle — the thing "range of
motion" quantifies — is the **relative orientation of two adjacent segments'
nodes**:

| Joint | Proximal segment | Distal segment |
|---|---|---|
| shoulder | `torso` | `upper_arm` |
| elbow | `upper_arm` | `forearm` |
| wrist | `forearm` | `hand` |

Consequences that the schema exists to make explicit:

- **A joint needs both of its nodes.** Drop the torso node and you can still
  report how the upper arm moved in space, but you **cannot** separate shoulder
  motion from trunk motion — shoulder ROM becomes non-computable, not
  approximate. The resolver returns it as blocked, naming the missing node.
- **The computable metric set is a function of placement.** Bilateral symmetry
  needs both sides; inter-joint coordination needs ≥2 joints on a limb; trunk
  compensation needs the torso node. None of these are "always on".
- **Orientation ≠ anatomical angle without calibration** (see §4).

---

## 2. Schema

A montage is JSON. `motion_capabilities.py --example` prints a fillable one.

```json
{
  "schema_version": "1.0",
  "subject":  { "id": "S01", "notes": "" },
  "session":  { "id": "2026-09-04-A", "aligned_csv": "aligned.csv" },
  "calibration": {
    "neutral_pose": "N-pose",
    "captured": true,
    "t_window_ms": [1000, 4000],
    "functional": []
  },
  "nodes": [
    { "node_id": "HULC-IMU-D067", "column": "n0", "segment": "torso",       "landmark": "sternum",       "calibrated": true },
    { "node_id": "HULC-IMU-A1B2", "column": "n1", "segment": "upper_arm_r", "landmark": "right humerus", "calibrated": true },
    { "node_id": "HULC-IMU-C3D4", "column": "n2", "segment": "forearm_r",   "landmark": "right forearm", "calibrated": false }
  ]
}
```

### Fields

| Field | Meaning |
|---|---|
| `subject.id` | Person identity (drives per-person segment lengths later, if FK is added). |
| `session.aligned_csv` | The reconcile output this montage annotates. |
| `calibration.neutral_pose` | The static zeroing pose captured at session start (e.g. `N-pose`). |
| `calibration.captured` | Whether that pose was actually recorded this session. |
| `calibration.t_window_ms` | Where in the aligned stream the neutral pose sits — the window a downstream step averages to define each segment's anatomical zero. *Optional hint:* calibration uses it only if the data there is actually still; otherwise (a placeholder like the example's `[1000, 4000]`, or a mistimed window) it auto-locates the first still stretch. `--window` overrides both. The closing hold is always found automatically and lands in `calibration.json`'s `closing` block, not here. |
| `calibration.functional` | Optional functional-calibration movements captured (e.g. a known elbow flexion to fix a joint axis). |
| `nodes[].node_id` | The board's advertised id (`HULC-IMU-XXXX`), for traceability. |
| `nodes[].column` | **The bridge to reconcile output** — the per-node prefix in the aligned CSV header (`n0`, `n1`, …). |
| `nodes[].segment` | One of the canonical segments (§3). One node per segment. **Authoritative** for what the node represents. |
| `nodes[].landmark` | *Optional.* Human label of where the node sits, e.g. `right humerus`. Descriptive only — it does not drive resolution, but the resolver checks it against `segment` and warns on a mismatch (§3.1). |
| `nodes[].calibrated` | Whether this node has a valid anatomical calibration this session. |

### Canonical segments

`torso`, `upper_arm_l`, `upper_arm_r`, `forearm_l`, `forearm_r`, `hand_l`,
`hand_r`. The suffix `_l` / `_r` is the body side. `torso` is the root
reference frame for both shoulders.

### 3.1 Landmark vs segment (why labels are only advisory)

A node is placed around an anatomical **landmark**; the model reasons about the
**segment** (bone) the node is on. The two are not the same:

- A landmark that *names a segment* — `humerus`, `forearm`, `torso`, `hand` —
  maps cleanly, and a `landmark` that disagrees with its `segment` is flagged
  as a likely mislabel (e.g. `landmark: "forearm"` on `segment: "upper_arm_l"`).
- A landmark that *names a joint* — `shoulder`, `elbow`, `wrist` — is
  **ambiguous**: a joint spans two segments, but a node sits on one bone. So a
  node "at the wrist" is `forearm_l` (distal) **or** `hand_l`, not both. The
  resolver keeps `segment` authoritative and emits a labeling note rather than
  guessing.

This is why **two nodes labeled "wrist" and "elbow" on one arm yield one joint,
not two** — each joint needs a node on the bone *either side* of it. Two left
nodes cover at most one adjacent segment pair; computing both `elbow_l` and
`wrist_l` needs three left segments (`upper_arm_l` + `forearm_l` + `hand_l`).
Labeling notes appear in the report header and under `montage_warnings` in the
JSON output.

### 3.2 Node header vs montage (source-of-truth rule)

Each node also stores **its own segment** as a 1-byte code in its log header
(`SETUP_AND_CALIBRATION_PLAN.md` §2.1), written at enroll and reported over BLE.
The code enum is `motion_capabilities.SEGMENT_CODES` — the single source of truth,
shared with the firmware; **append new segments only at the end** so a code never
changes meaning for logs already on deployed nodes. The rule when the two sources
disagree:

- The **montage `segment` stays authoritative** — it is what binds a log to a body
  part in the pipeline.
- The node header is the **default/confirmation**: `read-segments` builds the
  montage straight from the headers, and offload writes a `<node_id>.seg.json`
  sidecar next to each `.bin`.
- On a mismatch, `analyze_session` **warns** (same non-fatal spirit as the
  landmark check) so a swapped or re-placed node surfaces instead of silently
  binding wrong — but it never overrides the montage.

### Validation (hard errors)

- unknown `segment` (not in the canonical list)
- two nodes assigned the same `segment`
- two nodes sharing the same `column`
- missing `segment` or `column`

---

## 3. Joints, DOFs, and decomposition conventions

The resolver does **not** compute angles — it declares capability and carries
the intended decomposition convention so the downstream angle step stays
consistent. Conventions follow the ISB recommendations (Wu et al., 2005).

| Joint | DOFs | Decomposition | Note |
|---|---|---|---|
| shoulder | plane of elevation, elevation, axial rotation | `YXY` (plane of elevation, elevation, axial) | Ball joint, large ROM — Euler order matters; gimbal lock near poles. Trunk contaminates without a calibrated torso node. |
| elbow | flex/ext, pronation/supination | `ZXY` | Pro/sup is a radioulnar rotation seen as forearm axial rotation vs the humerus; sensitive to forearm-node roll — calibrate axial zero explicitly. |
| wrist | flex/ext, radial/ulnar deviation | `ZXY` | |

---

## 4. Calibration is what turns orientation into an *anatomical* angle

A raw relative quaternion between two nodes is a valid *relative* orientation,
but its decomposition into flexion/abduction/rotation is meaningless until each
segment's **anatomical frame** is known. The BNO reports the **full Rotation
Vector** (report `0x05`, magnetometer-referenced), so every node's orientation
already lives in a **shared world frame** (gravity + magnetic north) — the
cross-node *spatial* frame is largely given, not something calibration must
build. What calibration solves is the remaining unknown: the **sensor→segment
mounting offset** — how the sensor housing sits rotated/tilted on the bone,
which the world frame says nothing about.

This solve is built: **`calibrate_segments.py calibrate`** produces
`calibration.json` (the per-segment mounting offsets) from the neutral-pose
window, and `--update-montage` flips the `calibration.captured` and per-node
`calibrated` flags this section gates on. `calibrate_segments.py verify` runs the
cached-with-verify check described at the end of this section.

So calibration here is a **sensor-to-segment** calibration, not a "record the
resting quaternion" step. A static **neutral / N-pose** does three jobs at once:

1. **Solves the mounting offset** — comparing the *known* anatomy of the pose
   against the sensor reading recovers sensor→segment for each node.
2. **Sets the anatomical zero** — "this configuration = 0°."
3. **Heading co-registration fallback** — the mag usually ties the nodes'
   headings together for free, but degrades near metal; the pose is a robust
   on-body backup. (Optionally add a **functional** move — a known single-DOF
   motion — to fix axis directions, recorded in `calibration.functional`.)

The pose zeroes each segment, but its axes stay on the world compass. To tie
joint axes to the body (X anterior, Y superior, Z right), calibration also needs
the subject's **facing** — recovered from the torso node, else from the elbow
hinge axis (when the recording has enough elbow flexion), or stated with
`--facing-deg` — and records the result as `anatomical_frame` in
`calibration.json`. Joint angles are clinical only when it is present.

Every calibration-dependent metric (all joint angles/ROM, posture dwell) is
gated on `calibration.captured` **and** the relevant nodes' `calibrated` flag.
When either is false the resolver still lists the metric but attaches a
**relative-only** warning, so a UI can show the trace without claiming a
clinical number. Uncalibrated segments propagate: an uncalibrated `forearm_r`
flags both `elbow_r` and `wrist_r`.

**Validity is per-mounting, and cached-with-verify** — the mounting offset is a
property of *this* mounting, so it must be refreshed each time a node is put
back on (re-mounting shifts the straps — including taking nodes off to charge
between blocks; a power cycle alone does **not** invalidate it). It is cacheable: reuse the last calibration if a quick
still-pose consistency check passes, re-pose only when it drifts. This is
distinct from the **time offset**, a different quantity with a different
lifetime — see `SETUP_AND_CALIBRATION_PLAN.md` §3.

---

## 5. Capability tiers the resolver emits

This section is the *catalog* — which tier needs what. For **how each metric is
computed** (formula, units, calibration gate, output field), see `METRICS.md`.

| Tier | Needs | Examples |
|---|---|---|
| **segment** | 1 node | elevation, angular speed, smoothness, posture dwell |
| **joint** | 2 adjacent nodes | angle series, ROM, joint velocity, rep count |
| **derived** | a set of joints/segments | **L/R activity asymmetry** (segment pair), L/R ROM symmetry (joint pair), inter-joint coordination, trunk compensation |

### 5.1 Bilateral activity asymmetry (the sparse-montage workhorse)

`activity_asymmetry_<base>` fires on any matching **L/R segment pair**
(`upper_arm`, `forearm`, or `hand`) — one node each side, **no joint and no
torso required**. It answers "which arm is used more" (`asymmetry_index`,
`use_ratio`, `active_time_ratio`), which is often the most useful thing a
2-node bilateral montage can produce.

Its metrics are **session aggregates** of activity (integrated angular travel,
active-time fraction), so — unlike joint angles — they **do not need the two
sides time-aligned**. That matters because two independently-moving arms are
exactly the case `reconcile_nodes.py` aligns with low confidence (no shared
motion to cross-correlate); the asymmetry number stays valid anyway. This is
distinct from joint-level **`symmetry_<joint>`**, which compares ROM between two
*computable joints* and therefore needs the full two-node pair on each side.

Every metric also declares its **quality inputs** — the trust gates a UI must
surface alongside the value, never hide:

- `dropout` — gaps in the aligned stream (a gap is not stillness; the firmware's
  ~1 Hz STATIC_POSTURE heartbeat lets `stillness_confirmed` distinguish them).
- `sensor_cal` — the BNO's own calibration status.
- `sync_confidence` — the Pearson `r` from `reconcile_nodes.py` (joint metrics
  combine two nodes, so a weak alignment weakens every joint number).

---

## 6. Usage

```bash
# a fillable example montage
python tools/motion_capabilities.py --example > montage.json

# human-readable capability report
python tools/motion_capabilities.py montage.json

# machine-readable (for a UI)
python tools/motion_capabilities.py montage.json --json

# validate the model + resolver with no montage file
python tools/motion_capabilities.py --selftest
```

---

## 7. What this deliberately does *not* do (yet)

- **No position / endpoint trajectory.** These are orientation-only sensors;
  double-integrating acceleration drifts. Hand position would require forward
  kinematics through per-person segment lengths (a modeled estimate, not a
  measurement) — a separate, opt-in layer if it is ever added.
- **No angle math.** Decomposition, ROM, smoothness, etc. are metric plugins
  that consume the aligned CSV once the resolver says they are valid. Keeping
  them separate is the point: ROM is the first plugin, not a special case.
