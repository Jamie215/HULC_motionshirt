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
    { "node_id": "HULC-IMU-D067", "column": "n0", "segment": "torso",       "calibrated": true },
    { "node_id": "HULC-IMU-A1B2", "column": "n1", "segment": "upper_arm_r", "calibrated": true },
    { "node_id": "HULC-IMU-C3D4", "column": "n2", "segment": "forearm_r",   "calibrated": false }
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
| `calibration.t_window_ms` | Where in the aligned stream the neutral pose sits — the window a downstream step averages to define each segment's anatomical zero. |
| `calibration.functional` | Optional functional-calibration movements captured (e.g. a known elbow flexion to fix a joint axis). |
| `nodes[].node_id` | The board's advertised id (`HULC-IMU-XXXX`), for traceability. |
| `nodes[].column` | **The bridge to reconcile output** — the per-node prefix in the aligned CSV header (`n0`, `n1`, …). |
| `nodes[].segment` | One of the canonical segments (§3). One node per segment. |
| `nodes[].calibrated` | Whether this node has a valid anatomical calibration this session. |

### Canonical segments

`torso`, `upper_arm_l`, `upper_arm_r`, `forearm_l`, `forearm_r`, `hand_l`,
`hand_r`. The suffix `_l` / `_r` is the body side. `torso` is the root
reference frame for both shoulders.

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
| shoulder | flex/ext, abd/add, int/ext rotation | `YXY` (plane of elevation, elevation, axial) | Ball joint, large ROM — Euler order matters; gimbal lock near poles. Trunk contaminates without a calibrated torso node. |
| elbow | flex/ext, pronation/supination | `ZXY` | Pro/sup is a radioulnar rotation seen as forearm axial rotation vs the humerus; sensitive to forearm-node roll — calibrate axial zero explicitly. |
| wrist | flex/ext, radial/ulnar deviation | `ZXY` | |

---

## 4. Calibration is what turns orientation into an *anatomical* angle

A raw relative quaternion between two nodes is a valid *relative* orientation,
but its decomposition into flexion/abduction/rotation is meaningless until each
segment's **anatomical frame** is known — the BNO reports orientation in its
own mounting frame, offset from anatomy by however the strap sat.

So every calibration-dependent metric (all joint angles/ROM, posture dwell)
is gated on `calibration.captured` **and** the relevant nodes'
`calibrated` flag. When either is false the resolver still lists the metric
but attaches a **relative-only** warning, so a UI can show the trace without
claiming a clinical number. Uncalibrated segments propagate: an uncalibrated
`forearm_r` flags both `elbow_r` and `wrist_r`.

Minimum: a static **neutral/N-pose** zeroes each segment. Better: add a
**functional** movement (a known single-DOF motion) to fix axis directions —
recorded in `calibration.functional`.

---

## 5. Capability tiers the resolver emits

| Tier | Needs | Examples |
|---|---|---|
| **segment** | 1 node | elevation, angular speed, smoothness, posture dwell |
| **joint** | 2 adjacent nodes | angle series, ROM, joint velocity, rep count |
| **derived** | a set of joints/segments | L/R symmetry, inter-joint coordination, trunk compensation |

Every metric also declares its **quality inputs** — the trust gates a UI must
surface alongside the value, never hide:

- `dropout` — gaps in the aligned stream (a gap is not stillness; the firmware's
  0.2 Hz STATIC_POSTURE heartbeat lets `stillness_confirmed` distinguish them).
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
