# Technical reference — how the system works, and why

How the HULC Motion Shirt works inside: the system and its data formats, the
nodes' firmware, the design decisions, the math behind every number, the
evidence for the choice of pose solver, and how to change the software safely.
Each section starts with the idea in ordinary words and then gives the precise
version, so it can be read for understanding or used to review, reproduce and
improve a method. Function and constant names from the code are given in
brackets, so you can find the exact implementation. Equations are written in
LaTeX (`$…$` inline, ` ```math ` blocks), which GitHub displays as formatted
math.

The hands-on side (building a node, installing the software, recording a
session, troubleshooting) is in the [SOP](SOP.md); where the project stands is
in [`HANDOFF.md`](../HANDOFF.md).

1. [The system and the pipeline](#1-the-system-and-the-pipeline)
2. [The capture folder (data format)](#2-the-capture-folder-data-format)
3. [The montage and what it lets you compute](#3-the-montage-and-what-it-lets-you-compute)
4. [The nodes and their firmware](#4-the-nodes-and-their-firmware)
5. [Design decisions: identity, time, calibration, and the recording protocol](#5-design-decisions-identity-time-calibration-and-the-recording-protocol)
6. [Clock alignment](#6-clock-alignment)
7. [Metrics](#7-metrics)
8. [Choosing the pose solver](#8-choosing-the-pose-solver)
9. [Changing the software safely](#9-changing-the-software-safely)

**Key terms**

| **Term** | **Meaning** |
|---|---|
| Quaternion | Four numbers $(w, x, y, z)$ that describe an orientation (which way a node points) without the gaps or jumps of three angles. $q$ and $-q$ describe the same orientation |
| World frame | The shared reference every node reports in: "up" from gravity and "north" from the magnetometer |
| Angular velocity / speed | How fast something is turning, and about which axis (velocity); or just how fast, ignoring the axis (speed) |
| Cross-correlation | Sliding one signal along another and scoring how well they match at each shift; the best shift is the delay between them |
| Lag / offset | How far apart two clocks are (ms). Drift is how fast that offset changes, in parts per million (ppm) |
| DOF | Degree of freedom: one direction a joint can move in (e.g. elbow flexion) |
| Euler sequence | A recipe for splitting a 3-D rotation into three angles about named axes in a fixed order. The ISB (International Society of Biomechanics) recommends one per joint |
| ROM | Range of motion: the span between the smallest and largest angle reached |
| RMS | Root-mean-square: a typical size of an error or signal, weighting large values more |
| Neutral pose / freeze | The 5-second still pose at the start of each block (arms down, palms to the thighs) that defines zero |
| IK | Inverse kinematics: finding the body-model pose that best matches the sensors |

---

## 1. The system and the pipeline

### 1.1 The system: today and the target

```mermaid
flowchart LR
    N["Nodes<br/>record to flash"]
    subgraph ACQ["Acquisition"]
        L["Laptop: hulc-ms nodes<br/>(today, interim)"]
        M["Mobile app<br/>enroll, offload, store every block<br/>(target, not built)"]
    end
    C[("Capture folders<br/>the seam")]
    subgraph AN["Analysis (on a computer)"]
        A["hulc-ms analyze<br/>block check + metrics"]
        R["Review page<br/>HULC-MS app"]
    end
    N -- BLE --> L --> C
    N -. BLE .-> M -. "export after the<br/>collection period" .-> C
    C --> A --> R
    classDef todo stroke-dasharray: 5 5
    class M todo
```

Solid lines exist today; dashed lines are the target.

The **capture folder** (chapter 2) is the seam. Left of it is acquisition, right of
it analysis, and neither side knows the other: the mobile app replaces
`hulc-ms nodes` by writing the same folders, and the analysis, the HULC-MS app
and the regression library work unchanged. A whole export (many blocks, many
subjects) is just a tree of capture folders: `hulc-ms validate --recursive`
checks it, and dropping its top folder on the HULC-MS app analyzes every block.

| Layer | Status | Code | Talks to the next layer through |
|---|---|---|---|
| Firmware | built | `firmware/firmware.ino` | BLE GATT ([§4.3](#43-ble-protocol--talking-to-a-node)) |
| Acquisition | interim: laptop | `tools/multinode_test.py` (`hulc-ms nodes`) | capture folders |
| Acquisition | **TODO: mobile app** | — | capture folders, exported to the computer |
| Analysis | built | `tools/analyze_session.py` (`hulc-ms analyze`) and the stage modules | the output files (§2.5) |
| Review | built | `tools/skeleton_viewer.py` (review page), `tools/app.py` (HULC-MS app) | — |

---

### 1.2 The pipeline

Each node records the ORIENTATION of the SEGMENT it is strapped to. A clinical
**joint angle** is the *relative* orientation of two adjacent segments, so what
can be computed depends entirely on where the nodes are placed — the
**montage** (chapter 3). The pipeline makes that dependency explicit and never
fabricates a number a placement can't support.

`hulc-ms analyze` (`tools/analyze_session.py`) runs stages 3–7 in order; the app
runs it for each dropped block. Each stage is a separate module that reads
files and writes files, so it can be run, tested or replaced on its own
(running them by hand: [SOP §7](SOP.md#7-at-the-charger)):

```mermaid
flowchart TD
    CF[("capture folder<br/>.bin + .seg.json + montage.json")]
    RC["3 · reconcile_nodes<br/>align the clocks from the motion"]
    MC["4 · motion_capabilities<br/>which joints the montage allows"]
    CS["5 · calibrate_segments<br/>freeze, mounting offsets, facing, end"]
    ME["6 · metrics<br/>angles, ROM, speed, reps, plausibility"]
    SV["7 · skeleton_viewer<br/>the review page"]
    OS["opensense_ik<br/>(optional OpenSim solve)"]
    CF --> RC -- "aligned.csv<br/>aligned.quality.json" --> CS
    CF --> MC -- "joints / blocked" --> ME
    CS -- calibration.json --> ME
    ME -- metrics.json --> SV
    RC -.-> OS -.-> SV
    SV --> H[/"session.html"/]
    ME --> BC["analyze_session<br/>the block check<br/>(reads every stage's output)"] --> J[/"block_check.json"/]
```

| Stage | Module | What it does |
|-------|------|--------------|
| 1–2 Capture & offload | `firmware.ino`, `multinode_test.py` (`hulc-ms nodes`) | Nodes log autonomously; the laptop offloads each node's `.bin` |
| 3 Reconcile | `reconcile_nodes.py` | Decodes the `.bin` logs and time-aligns them onto one timeline **from the motion itself** (cross-correlating world-frame angular-velocity vectors; angular speed as fallback) → `aligned.csv`, plus `aligned.quality.json` (sync confidence, data gaps, clock restarts). Math: [chapter 6](#6-clock-alignment) |
| 4 Capability | `motion_capabilities.py` | Segments, joints and the montage schema; reads a capture folder (`capture_logs`, `montage_from_capture`); given the montage, resolves which joints/metrics are valid and which are blocked (and why) |
| 5 Calibrate | `calibrate_segments.py` | Finds the sync movement, the freeze after it and the session end; solves each node's **sensor→segment mounting offset**; estimates the subject's facing (torso node, elbow hinge, or `--facing-deg`) → `calibration.json` |
| 6 Metrics | `metrics.py` | Per-DOF joint angles over the analysis window → range of motion, angular velocity, rep bouts, plausibility flags, plus segment and derived tiers → `metrics.json`. Every metric: [chapter 7](#7-metrics) |
| 7 Visualize | `skeleton_viewer.py` | The review page (one self-contained HTML file): the segments as a stickman, Front / Side / Top views, the metrics panel, the Timeline, and a switch to each OpenSense solve |
| (optional) OpenSense | `opensense_ik.py` | The same session solved on a published OpenSim model (Thoracoscapular Shoulder Model, right arm; or Rajagopal 2016), through the same metrics and page; fit-lost frames excluded. Why: [chapter 8](#8-choosing-the-pose-solver) |
| Block check | `analyze_session.py` | Runs the stages; OK / REDO per check (sync, freeze, facing, end, trunk, joint limits, …) → `block_check.json` |

![The review page (stage 7): the metrics panel on the left, the 3-D stickman mid-movement with live angles on the right, and the play / view controls along the bottom](img/review-page.png)

Around the stages:

| Module | Role |
|---|---|
| `app.py` | the HULC-MS app: a local web page around `analyze_session.py` |
| `validate_capture.py` | `hulc-ms validate`: checks a capture folder against chapter 2 |
| `regress.py` | `hulc-ms regress`: the regression library of real recordings (chapter 9) |
| `hulc_paths.py` | where the tools write; how one tool starts another (`tool_cmd`) |
| `hulc-ms.py` (repo root) | the `hulc-ms` command; the standalone build's entry point |
| `tools/evaluation/` | synthetic-truth studies behind the design (`timing_bench`, `compare_paths`, `opensense_feasibility`) |

---

## 2. The capture folder (data format)

The **capture folder** is where acquisition hands over to analysis. Whatever
gets the logs off the nodes — today the laptop (`hulc-ms nodes offload`),
later the mobile app — writes one folder per block in this format, and
everything after it (`hulc-ms analyze`, the HULC-MS app, the regression
library) reads only this. Keep to it and the analysis needs no change when the
acquisition side does.

Check a folder, or a whole export of many blocks, before analyzing:

```bash
hulc-ms validate --capture-dir ./capture/block1
hulc-ms validate --capture-dir ./phone_export --recursive
```

### 2.1 One folder per block

A **block** is one mounting of the nodes, from strapping on to taking off
([SOP §6](SOP.md#6-recording-a-block)). Its folder
holds, for every node:

| File | Required | Content |
|---|---|---|
| `<name>.bin` | yes | the node's log: its 20-byte records, back to back (§2.2) |
| `<name>.seg.json` | yes | the header naming the node and its segment (§2.3) |
| `montage.json` | no | which node is on which segment, plus subject / session ids (§2.4) |

- `<name>` is free (`HULC-IMU-485C`, `export_1`, …), but each `.bin` and its
  `.seg.json` share it. The node is identified by `node_id` in the
  `.seg.json`, not by the file name.
- One file pair per node, at most one node per segment.
- Nothing else in the folder is read. Keep the folder whole: without the
  `.seg.json` files a log does not say what it measured.

**Many blocks** (a collection period's export): any folder tree whose leaf
folders are block folders. `hulc-ms validate --recursive` checks every folder
that holds `.bin` logs; dropping the top folder on the HULC-MS app makes one
session per block folder. A suggested layout:
`<export>/<subject>/<date>_<block>/…`.

### 2.2 The log: `.bin`

Exactly the bytes the node stores after its 4 KB flash header — the log data
region, as offload delivers it — with no header, framing or padding added.

Each record is 20 bytes, little-endian:

| Bytes | Type | Field |
|---|---|---|
| 0–3 | `uint32` | `t_ms`: the node's own `millis()` clock |
| 4–7 | `float32` | quaternion w (real) |
| 8–11 | `float32` | quaternion x |
| 12–15 | `float32` | quaternion y |
| 16–19 | `float32` | quaternion z |

- The quaternion is the BNO086 Rotation Vector: the sensor's orientation in a
  world frame of gravity and magnetic north, the same frame for every node.
- **Clocks are per node.** Each node counts from its own boot; the analysis
  aligns them from the motion itself (the sync movement), so no time sync or
  wall-clock time is needed in the file.
- **Rate:** ~10 records/s while moving, ~1/s while still, none in IDLE.
- **What the loader tolerates** (and `validate` reports): records with
  `t_ms = 0xFFFFFFFF` (erased flash) or a zero / non-finite quaternion are
  dropped, so a transfer hole may be **zero-filled** (keep the offsets, never
  close the gap); trailing bytes that are not a whole record; a constant
  framing shift; a clock restart (a reboot mid-block: only the longest part is
  analyzed).

### 2.3 The header: `.seg.json`

```json
{
  "node_id": "HULC-IMU-485C",
  "segment": "torso",
  "code": 0
}
```

| Field | Required | Meaning |
|---|---|---|
| `node_id` | yes | the node's advertised name, `HULC-IMU-` + the last 4 hex digits of its BLE MAC; stable per board |
| `segment` | yes | one of the segment names below |
| `code` | no | the segment code the node stores (read from the status characteristic, bytes 4–5: [§4.3](#43-ble-protocol--talking-to-a-node)) |

| Code | Segment | Code | Segment |
|---|---|---|---|
| 0 | `torso` | 4 | `forearm_r` |
| 1 | `upper_arm_l` | 5 | `hand_l` |
| 2 | `upper_arm_r` | 6 | `hand_r` |
| 3 | `forearm_l` | 0xFF | unassigned (not allowed in a capture) |

The table is `motion_capabilities.SEGMENT_CODES`, shared with the firmware;
new segments are only ever appended.

### 2.4 The montage: `montage.json` (optional)

If present it is used as is (`--montage` overrides it; without either, the
analysis builds one from the `.seg.json` files). Write it when there is more to
record than the headers carry — the subject id, the session id, strap
positions. The full format is in §3.2; the minimum:

```json
{
  "schema_version": "1.0",
  "subject": {"id": "S01"},
  "session": {"id": "2026-10-21_block1"},
  "calibration": {"neutral_pose": "N-pose", "captured": true, "protocol": "sync-first"},
  "nodes": [
    {"node_id": "HULC-IMU-485C", "column": "n0", "segment": "torso"},
    {"node_id": "HULC-IMU-B059", "column": "n1", "segment": "upper_arm_r"}
  ]
}
```

### 2.5 What the analysis writes

`hulc-ms analyze --capture-dir … --outdir OUT` (the app writes the same into
`~/HULC_sessions/<session>/out/`):

| File | Content |
|---|---|
| `block_check.json` | the verdict (OK / REDO) and one line per check |
| `session.html` | the review page, self-contained |
| `metrics.json` | per-joint, per-DOF angles: range of motion, speed, reps, plausibility; segment and derived metrics ([chapter 7](#7-metrics)) |
| `calibration.json` | the freeze window, mounting offsets, facing, session end |
| `aligned.csv` | all nodes on one timeline: `t_common_ms`, then `n<i>_qw, n<i>_qx, n<i>_qy, n<i>_qz` per node (column order from the montage) |
| `aligned.quality.json` | per node: clock offset, sync confidence, data gaps, clock restarts |
| `montage.json` | only when the montage was built from the headers |
| `opensense/<model>/` | per body-model solve, when OpenSim is installed |

---

## 3. The montage and what it lets you compute

The **montage** is the record of where each node is placed for a session; the
**capability resolver** (`motion_capabilities.py`) maps a montage to the
joints and metrics that can actually be computed from it. Reconcile puts the
nodes on one timeline; the montage says *what body part each aligned stream
is*; the resolver says *what that placement lets you compute*. Every metric
builds against it.

### 3.1 Why a montage is required

A node measures the **orientation of the segment** it is strapped to, not a
joint. A joint angle is the **relative orientation of two adjacent segments'
nodes**:

| Joint | Proximal segment | Distal segment |
|---|---|---|
| shoulder | `torso` | `upper_arm` |
| elbow | `upper_arm` | `forearm` |
| wrist | `forearm` | `hand` |

```mermaid
flowchart LR
    T(["torso"]) ---|shoulder| UA(["upper_arm"]) ---|elbow| FA(["forearm"]) ---|wrist| H(["hand"])
```

Each rounded box is a node on a segment; each labelled link is a joint, and
it can be computed only when the nodes at **both** of its ends are worn. The
same chain exists on the left and right sides (`_l` / `_r`); the torso is
shared.

- **A joint needs both of its nodes.** Drop the torso node and you can still
  report how the upper arm moved in space, but you **cannot** separate shoulder
  motion from trunk motion — shoulder ROM becomes non-computable, not
  approximate. The resolver returns it as blocked, naming the missing node.
- **The computable metric set is a function of placement.** Bilateral symmetry
  needs both sides; inter-joint coordination needs ≥2 joints on a limb; trunk
  compensation needs the torso node.
- **Orientation ≠ anatomical angle without calibration** (§5.4).

### 3.2 Schema

`python tools/motion_capabilities.py --example` prints a fillable montage
(`tools/montage.example.json` is a filled-in one).

```json
{
  "schema_version": "1.0",
  "subject":  { "id": "S01", "notes": "" },
  "session":  { "id": "2026-09-04-A", "aligned_csv": "aligned.csv" },
  "calibration": {
    "neutral_pose": "N-pose",
    "captured": true,
    "protocol": "sync-first",
    "functional": []
  },
  "nodes": [
    { "node_id": "HULC-IMU-D067", "column": "n0", "segment": "torso",       "landmark": "sternum",       "calibrated": true },
    { "node_id": "HULC-IMU-A1B2", "column": "n1", "segment": "upper_arm_r", "landmark": "right humerus", "calibrated": true },
    { "node_id": "HULC-IMU-C3D4", "column": "n2", "segment": "forearm_r",   "landmark": "right forearm", "calibrated": false }
  ]
}
```

| Field | Meaning |
|---|---|
| `subject.id` | Person identity. |
| `session.aligned_csv` | The reconcile output this montage annotates. |
| `calibration.neutral_pose` | The static zeroing pose (e.g. `N-pose`). |
| `calibration.captured` | Whether that pose was actually recorded this session. |
| `calibration.protocol` | *Optional.* `sync-first` (default — sync movement, then the freeze) or `hold-first` (recordings made before 2026-10). `--protocol` overrides. |
| `calibration.t_window_ms` | *Optional, normally absent.* A known neutral window `[t0, t1]`; used only if the data there is actually still. Calibration finds the freeze itself, and `--window` overrides both. `[1000, 4000]` — the placeholder `enroll` used to write — is ignored. |
| `calibration.functional` | Optional functional-calibration movements captured (e.g. a known elbow flexion to fix a joint axis). |
| `nodes[].node_id` | The board's advertised id (`HULC-IMU-XXXX`). |
| `nodes[].column` | **The bridge to reconcile output** — the per-node prefix in `aligned.csv` (`n0`, `n1`, …). |
| `nodes[].segment` | One of the canonical segments: `torso`, `upper_arm_l/r`, `forearm_l/r`, `hand_l/r`. One node per segment. **Authoritative** for what the node represents. |
| `nodes[].landmark` | *Optional.* Human label of where the node sits. Descriptive only; checked against `segment` (§3.3). |
| `nodes[].calibrated` | Whether this node has a valid anatomical calibration this session. |

**Hard errors:** an unknown `segment`; two nodes on the same `segment` or the
same `column`; a missing `segment` or `column`.

### 3.3 Landmark vs segment, and node header vs montage

**Landmarks are advisory.** A landmark that *names a segment* (`humerus`,
`forearm`) is checked, and one that disagrees with its `segment` is flagged
as a likely mislabel. A landmark that *names a joint* (`shoulder`, `elbow`,
`wrist`) is ambiguous — a joint spans two segments but a node sits on one
bone — so the resolver keeps `segment` authoritative and emits a note. This is
why **two nodes labelled "wrist" and "elbow" on one arm yield one joint, not
two**: `elbow_l` and `wrist_l` together need three left segments.

**Which source wins.** Each node stores its own segment in its log header
(§5.2). The node header is the **default**: offload writes a
`<node_id>.seg.json` next to each `.bin`, and the block's `montage.json` from
them. `analyze_session` uses, in order: `--montage` if given; else the capture
folder's `montage.json`; else one built from the `.seg.json` files (saved to
the output folder). A montage that is used **is authoritative**; on a mismatch
with a node's header the analysis **warns**, so a swapped node surfaces
instead of silently binding wrong. Labels never change an angle: where on the
segment the strap sat is measured by the freeze calibration.

### 3.4 Joints, DOFs and conventions

The resolver declares capability and the decomposition convention; the angle
math is in `metrics.py` ([§7.2](#72-the-shared-chain-quaternion--joint-angle)).
Conventions follow the ISB recommendations (Wu et al., 2005).

| Joint | DOFs | Decomposition | Note |
|---|---|---|---|
| shoulder | plane of elevation, elevation, axial rotation | `YXY` | Ball joint, large ROM; gimbal lock near poles. Trunk contaminates without a calibrated torso node. |
| elbow | flex/ext, pronation/supination | `ZXY` | Pro/sup is sensitive to forearm-node roll — calibrate axial zero explicitly. |
| wrist | flex/ext, radial/ulnar deviation | `ZXY` | |

### 3.5 Capability tiers

| Tier | Needs | Examples |
|---|---|---|
| **segment** | 1 node | elevation, angular speed, smoothness, posture dwell |
| **joint** | 2 adjacent nodes | angle series, ROM, joint velocity, rep count |
| **derived** | a set of joints/segments | L/R activity asymmetry (segment pair), L/R ROM symmetry (joint pair), inter-joint coordination, trunk compensation |

**Bilateral activity asymmetry** fires on any L/R segment pair — one node each
side, no joint or torso needed — and answers "which arm is used more". Its
metrics are session aggregates, so they **do not need the two sides
time-aligned**, which matters because two independently moving arms are
exactly what reconcile aligns with low confidence.

Every metric also declares its **quality inputs**, which a UI must show beside
the value: `dropout` (gaps in the aligned stream), `sensor_cal` (the BNO's own
calibration status) and `sync_confidence` (from reconcile).

```bash
python tools/motion_capabilities.py montage.json          # human-readable capability report
python tools/motion_capabilities.py montage.json --json   # machine-readable
python tools/motion_capabilities.py --selftest            # the model + resolver, no montage
```

**Deliberately not done:** no position / endpoint trajectory (orientation-only
sensors; double-integrating acceleration drifts — hand position would need
per-person segment lengths, a modelled estimate).

---

## 4. The nodes and their firmware

What a node is made of, and how to build, flash and bench-test it, is in the [SOP](SOP.md) (chapters 2 and 3, Appendix D). This chapter is the inside view: the firmware switches, how a node behaves, the Bluetooth protocol a central uses, the flash log, and the decisions behind time alignment, the wake source and power. The firmware itself (`firmware/firmware.ino`: its header comment, numbered SECTIONs and `handleControl()`) is the source of truth; the reference central is `tools/multinode_test.py`, behind `hulc-ms nodes`.

In short, each node is a Seeed XIAO nRF52840 (Bluetooth microcontroller)
with a SparkFun BNO086 over I²C at 100 kHz (this board's I²C is unstable at
400 kHz). The BNO086's fused, magnetometer-referenced Rotation Vector gives
every node a quaternion in the same world frame (gravity and magnetic north).
The log goes to the XIAO's external 2 MB QSPI flash (P25Q16H), driven directly
through `nrfx_qspi`, so the bootloader on the internal flash is never at
risk. Each node advertises a unique name, `HULC-IMU-XXXX` (the last four hex
digits of its Bluetooth address), so boards never collide and need no
per-board firmware.

### 4.1 Firmware build switches

Compile-time switches worth knowing (all near the top of `firmware.ino`):

| Switch | Section | Default | What it does |
|---|---|---|---|
| `IDLE_WAKE_SOURCE` | 3 | `IDLE_WAKE_DETECTOR` | which BNO sensor wakes IDLE (§4.6) |
| `ACTIVE_FUSION_SOURCE` | 3b | `ACTIVE_FUSION_RV` | Rotation Vector, or the 6-axis Game Rotation Vector (§4.7) |
| `DEFAULT_ACTIVE_HZ` | 4 | 10 | ACTIVE log rate |
| `NOT_MOTION_TO_STATIC_MS` / `NOT_MOTION_TO_IDLE_MS` | 5 | 10 s / 60 s | state timeouts (§4.2) |
| `DEBUG_SERIAL` | 9 | 1 | serial logging (turn off for a deployed build, §4.7) |

`firmware/state_machine_test/state_machine_test.ino` is a stripped-down
harness (state machine only, no BLE or flash) used for the IDLE-reset
investigation in §4.6; it is not part of the product.

### 4.2 How a node behaves

An adaptive-power state machine that records autonomously, with no central
connected:

- **`ACTIVE_RECORDING`** — moving: logs the quaternion at ~10 Hz (exactly
  `activeHz`, on scheduled slots).
- **`STATIC_POSTURE`** — after 10 s still: logs ~1 Hz.
- **`IDLE`** — after 60 s still, or ~8 s lying flat (ON_TABLE: 3 s → STATIC,
  then 5 s → IDLE): logs nothing, sleeps between IMU interrupts, advertises.
  **Offload works only in IDLE**, so the laptop is disconnected while
  recording and connects afterwards.
- **Flash watermark** — past 80% full the node halves its ACTIVE rate
  (10 → 5 → 2 Hz). At 10 Hz that is after ~2.3 h of ACTIVE recording; it stops
  at ~2.6 h. Battery runs out first: plan blocks by battery, never past ~90 min.

```mermaid
stateDiagram-v2
    direction LR
    [*] --> IDLE: power on
    IDLE --> ACTIVE_RECORDING: motion (wake source, §4.6)
    ACTIVE_RECORDING --> STATIC_POSTURE: 10 s without motion,<br/>or 3 s lying flat
    STATIC_POSTURE --> ACTIVE_RECORDING: motion
    STATIC_POSTURE --> IDLE: 60 s without motion,<br/>or 5 more s lying flat
    ACTIVE_RECORDING --> IDLE: flash full
    note right of ACTIVE_RECORDING: logs ~10 Hz
    note right of STATIC_POSTURE: logs ~1 Hz
    note left of IDLE
        logs nothing, advertises,
        offload / erase allowed
    end note
```

How these timings shape the recording protocol is in
[§5.5.2](#552-timings-and-the-settings-behind-them).

### 4.3 BLE protocol — talking to a node

What a central needs to enroll a node, get its log off, and write it as a
capture folder ([chapter 2](#2-the-capture-folder-data-format)).
Reference functions in `tools/multinode_test.py` are named per step.

#### 4.3.1 Finding a node

- Each node advertises as **`HULC-IMU-XXXX`**, the last 4 hex digits of its BLE
  MAC: unique and stable per board. This is the `node_id` of the capture format.
- It advertises **only in IDLE** (once per second), and stops while a central is
  connected. Recording happens with no central connected.
- Reference: `scan()`, `connect_with_retry()`.

#### 4.3.2 GATT

Service `A0010000-B0CE-4A4A-8F0B-0011223344FF`; the characteristics share its
suffix:

| UUID | Name | Access | Size | Use |
|---|---|---|---|---|
| `A0010001-…` | Quaternion stream | NOTIFY | 20 B | debug only (a live record, when streaming is on) |
| `A0010002-…` | Control | WRITE | ≤ 9 B | commands, below |
| `A0010003-…` | Status | READ | 6 B | state, log size, segment |
| `A0010004-…` | Log offload | NOTIFY | ≤ 200 B | the log, framed (below) |
| `A0010005-…` | Time Info | READ | 16 B | clock-offset measurement (optional) |

**Status (A0010003), 6 bytes**

| Byte | Field |
|---|---|
| 0 | state: 0 IDLE, 1 STATIC_POSTURE, 2 ACTIVE_RECORDING |
| 1 | flags: bit 0 streaming, bit 1 time-synced |
| 2–3 | log size in KB (`uint16` LE) |
| 4 | segment config tag: `0x01` when byte 5 is a valid assignment |
| 5 | segment code (0 torso … 6 hand_r, `0xFF` unassigned; the table in [§2.3](#23-the-header-segjson)) |

Reference: `read_status()`, `_read_segment()`.

**Time Info (A0010005), 16 bytes, little-endian**

| Bytes | Field |
|---|---|
| 0–3 | node `millis()` now |
| 4–11 | sync epoch in ms (`uint64`, 0 if never synced) |
| 12–15 | node `millis()` at the last sync |

A central computes a node's epoch now as
`sync_epoch_ms + (node_millis_now − sync_millis)`, and two nodes' difference
is their clock offset. Not needed for the analysis (it aligns the clocks from
the motion, §4.5); useful to check a link. Reference: `read_syncinfo()`,
`report_offsets()`.

**Control (A0010002)**

| Command | Payload | Does | When |
|---|---|---|---|
| `0x00` / `0x01` | — | stop / start the debug quaternion stream | any time |
| `0x02` | `uint32` LE Unix seconds | time sync (seconds, legacy) | any time |
| `0x05` | `uint64` LE Unix ms | time sync (ms) | any time |
| `0x03` | — | erase the log (whole chip, verified blank; ~30 s) | after a verified offload |
| `0x04` | — | start the offload over A0010004 | **IDLE only** (ignored otherwise) |
| `0x06` | `uint32` LE offset, `uint32` LE length | re-send that byte range of the log | **IDLE only** |
| `0x07` | `uint8` segment code 0–6 | store this node's segment in its flash header (survives power cycles and erases) | enroll |

The command byte comes first, e.g. `[0x06, off0, off1, off2, off3, len0, len1, len2, len3]`.

#### 4.3.3 Getting the log off (offload)

```mermaid
sequenceDiagram
    participant C as Central (laptop / app)
    participant N as Node (in IDLE)
    C->>N: connect, subscribe to A0010004
    C->>N: write 0x04 (start offload)
    N-->>C: header: [0xFFFFFFFF][total length]
    loop every chunk
        N-->>C: [offset][up to 196 bytes]
    end
    Note over C: stream quiet ~3 s:<br/>find missing ranges against the total
    loop until complete (max 4 passes)
        C->>N: write 0x06 [offset][length]
        N-->>C: [offset][the missing bytes]
    end
    Note over C: write <node>.bin (holes zero-filled)<br/>+ <node>.seg.json from status bytes 4–5
    C->>N: write 0x03 (erase), only after the transfer is verified
```

1. Connect to a node in IDLE (status byte 0 = 0). Subscribe to A0010004.
   The node requests a 15–30 ms connection interval (§4.5).
2. Write `0x04`. The node first sends a **header notification**:
   `[0xFFFFFFFF (uint32 LE)][total length (uint32 LE)]` — the exact byte count
   of the log.
3. Then **data notifications**: `[offset (uint32 LE)][up to 196 bytes]`, the
   offset being where the payload sits in the log. Place each payload at its
   offset.
4. Notifications are unacknowledged and some get dropped. When the stream goes
   quiet (~3 s), compute the **missing ranges** against the total and request
   each with `0x06`; repeat until complete (the laptop tries up to 4 passes,
   `OFFLOAD_MAX_ATTEMPTS`). Reference: `measure_offload()`, `_offload_pass()`,
   `_missing_ranges()`.
5. Write the bytes as the node's `.bin`, unchanged; any range still missing
   stays **zero-filled** at its offset (never close the gap). Read the status
   bytes 4–5 and write the `.seg.json` header next to it (`node_id`,
   `segment`, `code`). Together these make the capture folder.
6. Only after the transfer is verified complete (bytes received = total), and
   preferably after the folder has been checked (`hulc-ms validate`), write
   `0x03` to erase. The node keeps the log until then, so a failed transfer can
   simply be repeated.

**Why the framing.** Before it, each 200-byte notification carried 10 records
back to back, and a dropped notification silently collapsed 10 records out of
the middle of the file — showing up downstream as ~1.2 s gaps with clean data
on both sides. With offsets, a drop leaves a locatable hole. Two more pieces
make recovery work: the firmware **backpressures** (when the BLE TX buffer is
full, `writeValue()` fails and the same notification is retried instead of
skipped — without this the sender alone dropped ~45%), and recovery uses
**small range re-requests** rather than a whole re-offload, which recreates the
same burst and tends to drop the same chunk again.

Reading the laptop's output: `pass 1 (full): 3528/5300 bytes, 5 hole(s)` (drops
located) → `pass 2: re-requesting 5 range(s)...` → `COMPLETE — saved N bytes`.
`[!] INCOMPLETE after 4 attempts` saves the file zero-filled and lists the
missing offsets; re-run `offload` to fill them (the log is still on the node).

#### 4.3.4 Enrolling a node (once per board)

Power on one node at a time, connect, write `[0x07, code]` for its segment, and
read the status back to confirm (bytes 4–5). The assignment stays on the node,
so every later offload can write the `.seg.json` from the node itself.
Reference: `enroll()`, `set_segment()`, `read_segments()`. The operator steps
are in [SOP §4](SOP.md#4-setting-up-the-nodes-once-per-rig).

#### 4.3.5 Things a central must not do

- Offload or erase while the node is recording: offload is refused outside
  IDLE; an erase loses whatever was not yet offloaded.
- Erase before the offload is verified.
- Re-number segment codes: they are stored on deployed nodes.

### 4.4 The flash log

```
 0x000000 ┌──────────────────────────── 4 KB header sector ─────────────────────────────┐
          │ bytes 0–3 magic 0xC0DE0001 · 4–7 write pointer · 8 segment tag · 9 segment code │
 0x001000 ├──────────────────────────────────────────────────────────────────────────────┤
          │ record 0: t_ms (uint32) · qw qx qy qz (4 × float32)          = 20 bytes        │
          │ record 1 …                                                                     │
          │ … append-only; past 80 % the ACTIVE rate is halved (watermark)                 │
          │ … logging stops 200 KB before the end                                          │
 0x200000 └──────────────────────────────────────────────────────────────────────────────┘
```

- A 4 KB header sector followed by 20-byte records, append-only; offload
  sends everything after the header (record layout:
  [§2.2](#22-the-log-bin)).
- The header (with the write pointer) is re-saved every `FSYNC_EVERY` (10)
  samples — a 4 KB sector erase + write + readback, ~1 s at 10 Hz — so a power
  cut loses at most ~1 s of samples.
- **Erase between blocks.** The write pointer persists across reboots, so a
  node power-cycled without erasing *appends* the next recording, and since
  `millis()` restarts at 0 the log holds two sessions with a backward time
  jump. The analysis keeps the longest monotonic part and warns
  (`clock restart` / `non-monotonic`), but stale data accumulates. Use
  `hulc-ms nodes erase` before a block, or `offload --erase-after` (each node
  wiped only after its offload verifies `COMPLETE`).
- `eraseLog()` erases the whole chip in 64 KB blocks and **verifies the data
  region reads back blank before resetting the write pointer**, so a silently
  failed erase leaves the log intact at its real size rather than reporting
  `log=0KB`.

### 4.5 Multi-node time alignment

**Requirements:** every node records the movement and the samples can be
aligned afterwards; the logs can be moved off the nodes to free flash.
**Constraints (decided):** no wire between nodes (BLE only); analysis is
**offline** after collection; quaternions at ~10 Hz, so the alignment target
is **25–50 ms** (a fraction of a sample), not single-digit ms.

**The key reframe.** Because analysis is offline, BLE latency **does not touch
the data path** — nodes never stream live. Each sample carries the node's
`millis()`; the clocks are aligned afterwards. The production method is
**cross-correlating the nodes' motion** around the sync movement every block
starts with (`tools/reconcile_nodes.py`; the math, its confidence measure and
its limits are in [chapter 6](#6-clock-alignment)).
It is immune to BLE latency, and its synthetic selftest recovers a known
offset to ~15–18 ms. It needs **shared motion** — two limbs moving
independently share nothing to lock onto — which is why every block starts
with a sync movement that moves every node. The BLE time sync (`0x05`, A005)
remains for link checks.

**The BLE connection interval is the crux** for offload speed. Measured: the
bottleneck was the **central**, not the firmware — a Windows desktop `bleak`
central imposes slow intervals (~850 ms per read; a full flash would take
hours), while an **iPhone offloaded a 48 KB log in 2.46 s (~20 KB/s,
~10 ms/chunk)**, i.e. a full 2 MB flash in ~1.7 min. The firmware requests
**15–30 ms** (`setConnectionInterval(12, 24)`); 15 ms is the floor because
Apple requires Interval Min ≥ 15 ms or iOS falls back to its slow default.
**The product's central should be iOS, Android or Linux (BlueZ), not a Windows
desktop.**

**Coordinated trigger — decided against.** Each node runs its own motion
detector, so a low-motion segment (e.g. the torso) could stay IDLE and miss an
event. A shared "any node moves → everyone records" trigger with a pre-roll
buffer was proposed; after discussion with the PIs it was not built: the
node's own idle behaviour is sufficient for the project, and the trigger would
cost design flexibility, battery and flash. (The STATIC timeout and the
recording protocol's "never still for a minute" rule cover the torso case.)

Open questions here (iOS sync-read latency, offload pacing, whether 10 Hz is
enough) are in [`HANDOFF.md`](../HANDOFF.md#open-work).

### 4.6 IDLE wake source and the ~6.5 s IDLE reset

#### 4.6.1 The switch

`firmware.ino`, Section 3:

```c
#define IDLE_WAKE_SOURCE   IDLE_WAKE_DETECTOR   // DETECTOR | CLASSIFIER | SIGMOTION
```

| | `IDLE_WAKE_DETECTOR` (default) | `IDLE_WAKE_CLASSIFIER` | `IDLE_WAKE_SIGMOTION` |
|---|---|---|---|
| Sensor | Stability Detector `0x1C` | Stability Classifier `0x13` | Significant Motion `0x12` |
| Sensing | accelerometer only (on-change) | accel + gyro (MotionEngine) | accelerometer only (one-shot) |
| Idle reset | ~6.7 s (inherent, benign) | **none** | **none** |
| Idle current | ~8.8 mA | higher (gyro running) | **~7.4 mA** (lowest) |
| Slow-motion sensitivity | good | good | **poor** — misses slow held-limb motion |

**Recommendation (current default):** the **detector** for on-body capture —
the only source that reliably wakes on slow, deliberate motion. **SIGMOTION**
only for off-body standby. The **classifier** if a reset-free idle ever matters
more than current. Keep the `i2cBusRecover()` insurance in `setup()` in every
case.

**devSleep was explored and dropped.** With the hub's own devSleep the
detector's wake event was coalesced or rarely delivered, so shaking failed to
wake IDLE → ACTIVE. Every wake source runs the hub awake; going lower needs a
hardware wake (§4.7.2).

**To A/B them:** build each value, exercise on the bench with Serial open
(test slow motions, not just shakes), and watch for the `DETECTOR: 0x1C val=` /
`CLASSIFIER: MOTION` / `SIGMOTION:` lines and the IDLE → ACTIVE transition.
`handleIdle()` logs each self-reset as
`IDLE: BNO reset — <ms> since last reset`; with the classifier or SIGMOTION
that line never prints, which *is* the result.

#### 4.6.2 The ~6.5 s IDLE reset (known quirk, understood, harmless)

In IDLE with the detector, the BNO086 **reboots itself every ~6.5 s**. Found
with the `state_machine_test` harness, one change at a time:

| Tried | Result | Rules out |
|---|---|---|
| devSleep off (fully awake) | still ~6.5 s | sleep |
| report interval 50 ms → 5 s | still ~6.5 s (worse when slow) | report rate |
| constant polling vs sleeping | identical | a slow host |
| plain accelerometer instead of detector | still ~6.5 s | misuse of the detector |
| I²C at 400 kHz | reset every ~130 ms + corruption | (board unstable at 400 kHz; stay at 100 kHz) |
| **Rotation Vector instead** | **no reset**, 300+ reports | **the cause is non-fusion sensors** |

**Explanation:** accelerometer-only sensors (detector, raw accel) leave the hub
in a low-activity state that self-restarts; anything that runs the fusion
engine (classifier, Rotation Vector) keeps it active and does not. That is why
almost nobody else reports it — most projects stream a fused output
continuously. The datasheet's "host too slow" watchdog applies to the older
BNO080, not the 085/086.

**Why it doesn't matter:** IDLE logs nothing and the firmware re-arms
automatically; ACTIVE streams the Rotation Vector and never resets. It is the
price of the low-power wake sensor. **The one real risk:** a reset landing
mid-I²C transaction could hold SDA low and freeze the node until a power
cycle — never observed, insured against by `i2cBusRecover()`.

*A lesson from the investigation:* a probe reading the "reset cause" on every
restart reported "External Reset", but the probe itself was causing those
resets (the driver hard-resets the chip when its SHTP traffic gets confused).
It was removed — and it showed why the bus-recovery routine is worth keeping.

#### 4.6.3 Not done: tuning SIGMOTION

SIGMOTION has a threshold (FRS record `SIG_MOTION_DETECT_CONFIG`, `0xC274`:
acceleration threshold + window), but the SparkFun library exposes no
FRS-write path. Making it work would need: the exact record layout from the
SH-2 Reference Manual, an FRS-write path (extend the library or use CEVA's
`sh2_setFrs()`), and a bench sweep against real slow held-limb motion vs false
wakes. Even then, an energy-over-a-window detector may still miss slow
stretches.

### 4.7 Power

**Measured now:** ~8.8 mA IDLE (detector), ~20–21 mA ACTIVE recording (full
fusion). Before the optimization work: ~12 mA / ~22 mA. Battery life on the
current firmware has **not** been bench-measured yet (open item).

#### 4.7.1 Landed

None of these change the recorded data format or the state logic; each is
commented at its site in `firmware.ino`.

1. **DC/DC regulator** (`NRF_POWER->DCDCEN = 1`): more efficient than the LDO
   in every state.
2. **Cheap, IDLE-only advertising:** ~1 s interval (`ADV_INTERVAL_UNITS`), and
   no advertising while recording (offload is IDLE-only anyway).
3. **BNO report rate matched to the log rate** (Rotation Vector at 100 ms
   instead of 65 ms): no fused samples computed only to be discarded.
4. **Game Rotation Vector switch** (`ACTIVE_FUSION_SOURCE`, default RV):
   6-axis, no magnetometer, immune to magnetic interference but with slow yaw
   drift. **Measured: no meaningful power saving**, so decide on data quality
   alone — switch only if captures show magnetometer-driven heading
   corruption. The GRV path has not been compile-checked against the installed
   library.
5. **QSPI flash deep power-down in IDLE** (`qspiSleep()`/`qspiWake()`): a tiny
   saving, kept for completeness. *Tried and removed:* also disabling the
   nRF QSPI peripheral made idle **worse** (~9 → ~11 mA) because of nRF52840
   Errata 122 — only revisit with Nordic's register workaround.
6. **STATIC arms the Rotation Vector at ~1 Hz** instead of 10 Hz
   (`BnoRunningCfg`: `BNO_CFG_ACTIVE` / `BNO_CFG_STATIC`); the classifier
   still drives every transition. ~4× fewer nRF wakes in STATIC. Only the
   fusion report is re-issued on an ACTIVE ↔ STATIC switch, to stay off the
   SparkFun premature-reset path (library issue #2).
7. **IDLE wake source** selectable (§4.6).
8. **Detector reported on change** (60 s keepalive instead of a ~1 Hz
   heartbeat): ~0.2 mA.
9. **STATIC logs every RV report (~1 Hz, was 0.2 Hz)** — for data quality: the
   torso is mostly STATIC during arm work and every shoulder angle is measured
   against it. Simulated (`tools/evaluation/timing_bench.py`): shoulder axial
   rotation 6.3° → 1.9° RMS, elevation 5.1° → 2.5°. Cost ~+14% of a torso
   node's log. Write energy not yet bench-measured.
10. **ACTIVE logs at exactly `activeHz`** (scheduled slots with period/4 slack,
    `ACTIVE_SAMPLE_SLACK_DIV`): the old gate logged 6.9–8.3 Hz when report
    timing didn't line up. This logs up to ~20–44% more ACTIVE records than
    older firmware did; if storage was sized on the observed rate, lower
    `DEFAULT_ACTIVE_HZ`. Not yet bench-measured on hardware.

#### 4.7.2 Backlog — worth exploring

Roughly by payoff; each needs bench time or a design decision.

- **Reconfigure the BNO rate on watermark throttle.** When flash fills,
  `writeQuaternionSample()` halves `activeHz` (10 → 5 → 2) but the BNO keeps
  emitting 10 Hz, so it saves flash but not power. Re-issue the fusion vector
  at the throttled rate through the same per-transition path as item 6 (mind
  the SparkFun issue #2 caution).
- **Sleep instead of spin while connected-idle.** In `waitForIMUData()` the
  BLE-connected branch busy-loops on `BLE.poll()` for up to 100 ms; a `__WFE()`
  between polls would let the core sleep. Relatedly, the fast connection
  interval is only needed during offload and could be relaxed otherwise.
- **Production build: drop USB/serial.** `DEBUG_SERIAL 1` keeps USB CDC (and
  HFCLK) alive, and the `Serial.print`/`Serial.begin` calls in the QSPI helpers
  and `setup()` are **not** behind the macro — gate those too.
- **fsync tuning.** `FSYNC_EVERY = 10` costs a sector erase + write + readback
  every ~1 s at 10 Hz; raising it trades a little data-at-risk on power loss
  for fewer power-hungry erases and less wear.
- **LEDs:** cut the jumpers (SOP §2); in firmware, the blue "connected" LED is held
  on for the whole connection and could blink instead.
- **Below ~8.8 mA while keeping slow-motion wake needs hardware:** a separate
  low-power motion sensor whose interrupt wakes the nRF, which then powers the
  BNO. The remaining gap to SIGMOTION is the detector's self-reset, which no
  report-rate change affects.
- **QSPI peripheral disable** (item 5), only with the Errata 122 workaround —
  low priority, since idle is dominated by the BNO hub (~8 mA).

---

## 5. Design decisions: identity, time, calibration, and the recording protocol

The reasoning behind stages 3–5, kept so the next change does not undo it.

### 5.1 Anchor facts (from the firmware)

- Nodes log the **full Rotation Vector** (report `0x05`), which is
  **magnetometer-referenced**: every node's orientation is already in a
  **shared world frame** (gravity + magnetic north), not the arbitrary
  per-node heading a Game Rotation Vector would give.
- Each sample is timestamped with the node's free-running **`millis()`**,
  which starts at zero on **power-up**. There is no real-time clock.
- Nodes advertise a unique identity `HULC-IMU-XXXX` (from the BLE MAC).

### 5.2 Node identity: the segment is stored on the node

"Which node is where" (configuration) is kept separate from "how the sensor
sits on the bone" (calibration): they have different lifetimes.

- The segment is a **1-byte code in the log header** (reserved header bytes,
  plus a config-valid tag), not per record and not a string. It survives power
  cycles and erases, is set over BLE with control `0x07` at enroll, and is
  reported on the status characteristic. The enum is
  `motion_capabilities.SEGMENT_CODES`, the one source of truth.
- This makes every **offloaded log self-describing** and removes the
  mislabel-after-offload risk. Offload writes the montage from the headers,
  so the user only confirms it (§3.3).
- *Not built, worth trying:* **auto-assignment** — a scripted "raise your
  right arm" detects which node moved and assigns it.

### 5.3 Time offset vs mounting calibration (the key distinction)

Two quantities are refreshed at the start of a block. They are
**independent** and must not be conflated:

| | Aligns | Invalidated by | Cacheable? | Refreshed how |
|---|---|---|---|---|
| **Time offset** | the nodes' clocks (temporal) | every **power-up** | **No** | the sync movement (cross-correlation in reconcile) |
| **Mounting calibration** | sensor→segment (spatial) | every **re-mount** | **Yes** | the freeze, or reuse-if-verified |

- **Time offset cannot be cached:** `millis()` restarts at zero on boot. Within
  a block it holds (drift is negligible over minutes; long sessions:
  [§6.11](#611-long-session-drift-tracking-textoffsett-as-a-fitted-curve)).
- **Mounting calibration survives a power cycle** (it is physical) and is
  broken only by re-mounting — so it *can* be cached and verified.
- Charging between blocks trips **both**: the straps come off, and each charge
  is also the offload-and-erase point, so every block starts a fresh log that
  is reconciled and calibrated on its own. That is why the sync movement and
  the freeze are one block-start routine. A node that reboots mid-log is
  caught by reconcile's clock-restart check.

```mermaid
flowchart LR
    P["Node powers up<br/>(millis restarts at 0)"] --> TO["Time offset is lost"]
    RM["Straps re-mounted"] --> MO["Mounting calibration is lost"]
    CH["Charging between blocks"] --> RM
    CH --> E["Offload + erase:<br/>a new log per block"] --> TO
    TO --> S["Fixed by the sync movement<br/>(reconcile)"]
    MO --> F["Fixed by the freeze<br/>(calibrate_segments)"]
    S & F --> B["Every block starts with:<br/>sync movement, then freeze"]
```

### 5.4 Calibration: the sensor→segment solve

**In plain terms.** A sensor is like a compass strapped to the arm at some
unknown angle: it reports honestly in a fixed world frame, but a crooked strap
means its reading is not yet *about the bone*. The one unknown is that
crookedness — the fixed rotation between sensor and bone. To find it, the
subject holds a pose whose answer we already know (the N-pose: arms down,
palms to the thighs); whatever the sensor reads there *is* the correction.
Subtract it from every later reading and the numbers become anatomical. A
joint angle is then one corrected sensor relative to its neighbour
(upper arm vs torso = shoulder), which reads zero in the freeze pose.

The freeze does three jobs: it **solves the mounting offset**, **sets the
anatomical zero**, and is a **heading co-registration fallback** where the
magnetometer degrades near metal. The pose zeroes each segment, but its axes
stay on the world compass; to tie joint axes to the body (X anterior,
Y superior, Z right) calibration also needs the subject's **facing** — from
the torso node › the elbow hinge axis (needs ≥ 30° of flexion) ›
`--facing-deg` › none — stored as `anatomical_frame` in `calibration.json`.
Joint angles are clinical only when it is present.

![The same freeze before and after calibration, side view: Raw (left) shows the sensors' own orientation; Calibrated (right) shows the arm hanging straight, which is what the freeze pose should look like](img/review-neutral-raw-vs-calibrated.png)

What `calibrate_segments.py` implements:

- **Freeze window:** `--window` › the montage's `t_window_ms` *only if the data
  there is still* › auto-detect (the first hold ≥ 3 s, ≤ 0.04 rad/s at its
  quietest, after the sync movement; `hold-first` recordings take the first
  hold of the log).
- **Session end:** no closing pose; the log's final rest in a pose no body
  holds (nodes on the charger) marks the nodes coming off (`find_session_end`;
  `--end` by hand). An earlier version required a closing hold; it could not
  be guaranteed in practice.
- **Gating:** every calibration-dependent metric is gated on
  `calibration.captured` **and** each node's `calibrated` flag; when either is
  false it is still listed but marked **relative-only**, never as a clinical
  number. Uncalibrated segments propagate (an uncalibrated `forearm_r` flags
  both `elbow_r` and `wrist_r`).
- **Cache-and-verify:** `calibration.json` stores the offsets and a
  consistency baseline; `calibrate_segments.py verify` checks a later still
  pose against it — small residual → reuse, over threshold → re-pose. The same
  check catches strap slippage within a session.

**Placement:** the node need not be aligned to anatomy, only placed **the same
way each time** (a fixed tilt *is* the mounting offset and is removed). The
fragile axis is roll *around* the limb, which corrupts pro/supination and
axial rotation — pin it to a consistent flat spot by a bony landmark
([SOP §6.1](SOP.md#61-where-each-node-goes)).

**Cheaper calibration, in order of effort** (1–2 built): (1) fold the pose into
the block-start routine; (2) cache + verify; (3) functional calibration from
ordinary motion (gravity gives vertical, a joint's gyro axis gives the hinge),
removing the deliberate pose; (4) rigid, anatomically shaped pockets in the
garment, so (2) can reuse more often.

---

### 5.5 The recording protocol: what each step gives, and the settings behind it

The [SOP](SOP.md) says what to do in a session; this section says why each step is there and which pipeline or firmware setting each number comes from. If you change one of the named settings, update the SOP too.

#### 5.5.1 What each step gives the pipeline

| Step | What the pipeline gets from it | Without it |
|---|---|---|
| **1 Strap on, switch on** | Nothing yet — and nothing here is used. Nodes may power up one by one and sit still for minutes; the analysis only starts after the sync movement. | — |
| **2 Sync movement** | One strong motion shared by every node, so their independent clocks can be aligned (`reconcile_nodes.py`); the **landmark** calibration uses to find the freeze; and motion that wakes every node and settles its magnetic heading. | Clocks off by milliseconds to tens of seconds; the freeze cannot be told from setup stillness |
| **3 Freeze** | The sensor→segment mounting offsets and the zero of every angle (`calibrate_segments.py`), plus which way the subject faces (torso node). Taken after the movement, so the nodes are awake and any strap shift from the twisting is already in the calibration. | Every angle measured against a wrong zero |
| **4 Task** | The movement of interest; with no torso node, elbow bends also give the facing. | — |
| **5 Nodes off** | Where the analysis ends: taken-off nodes lie still on the charger in a pose no body holds, then go IDLE. | — (detected; see SOP §7) |

---

#### 5.5.2 Timings and the settings behind them

| Step | Min / target / max | Why (the setting) |
|---|---|---|
| Sync movement | 4 / 10 / — s, 3–5 cycles of ~1–1.5 s | Clocks are aligned by matching the nodes' world angular-velocity vectors; one clear shared burst gives a distinct peak (`sync_peak_ratio` ≥ 1.25, `VECTOR_PEAK_RATIO_MIN`). Calibration recognizes it as every node moving (≥ 0.4 rad/s) **and turning together** (coherence of their rotation vectors ≥ 0.8) for ≥ 3 s (`SYNC_MIN_RAD_S`, `SYNC_MIN_COHERENCE`, `SYNC_MIN_MS`). Real twists ran the torso at only 0.4–0.8 rad/s but at coherence 0.84–0.96; strapping a node on moves the nodes independently (coherence 0.2–0.7), so it never counts. Faster than ~2 cycles/s aliases at 10 Hz. 10 s lets a node that wakes late still catch most of it. |
| Freeze | 3 / 5 / 8 s, starting within 20 s of the movement | The freeze is the **first still stretch after the sync movement lasting ≥ 3 s whose quietest 2 s averages ≤ 0.04 rad/s (~2°/s)** (`SYNC_TO_HOLD_MAX_MS`, `NEUTRAL_MIN_HOLD_MS`, `NEUTRAL_QUIET_RAD_S`) — pauses in motion only slow to ~0.06–0.09 rad/s, so **really freeze**. Over 10 s still, a node drops to 1 Hz STATIC (`NOT_MOTION_TO_STATIC_MS`): usable, but 5 s keeps 10 Hz. |
| Rests in the task | 5–8 s | ≥ 5 s separates sets in the rep count (`REP_PAUSE_S`); ≤ 8 s keeps the nodes at 10 Hz. |
| Still in the task | < 60 s | After 60 s without motion a node goes IDLE and stops logging until moved (`NOT_MOTION_TO_IDLE_MS`); the first moments after waking can be lost. After a long break, repeat steps 2–3. |
| Nodes off | lying flat ≥ 10 s | IDLE needs ON_TABLE (3 s → STATIC, then 5 s → IDLE); offload needs IDLE. |

A typical block on one timeline (seconds; the task part varies):

```mermaid
gantt
    dateFormat X
    axisFormat %s s
    section Subject
    Strap on (not analyzed)       :a1, 0, 20
    Sync movement (4–10 s)        :crit, a2, 20, 30
    Freeze (3–8 s, within 20 s)   :crit, a3, 30, 35
    Task set 1                    :a4, 35, 55
    Rest (5–8 s)                  :a5, 55, 61
    Task set 2                    :a6, 61, 81
    Nodes off, flat (≥ 10 s)      :a7, 81, 95
    section Analysis window
    Calibrated and measured       :active, b1, 30, 81
```

##### The freeze pose (N-pose)

- Standing upright, weight even, looking ahead. Seated is fine if the trunk is
  upright and the arms hang freely clear of the chair.
- Arms straight down at the sides, **elbows fully straight**, shoulders relaxed.
- **Palms facing the thighs** (thumbs forward). This is the forearm's zero: it
  is what `metrics.py` and the OpenSense models assume.
- No talking, weight shifting or looking around. Breathing is fine.

##### The sync movement, per montage

| Montage | Movement | Key points |
|---|---|---|
| upper arm + forearm | **Whole-arm swings** forward and back from the shoulder, elbow **locked straight**, ~60–90° each way | Bending the elbow during the swing weakens the match. |
| torso + upper arm | **Trunk twists** left–right, ±30–45°, with the hand **on the hip** | An arm swing leaves the torso still, so it cannot sync the torso node. |
| torso + upper arm + forearm | **Trunk twists** as above, whole arm against the side | One movement moves all three nodes. |

##### Task rules

- **Speed:** at 10 Hz, at most ~1 repetition per second, smooth. Fast snaps
  (>~300°/s) are where angles and model fits are worst.
- **Range:** the full range of interest, but not past what the subject can hold
  steadily. Wobble grows with speed and effort.
- **No torso node:** the front direction comes from the elbow's hinge, which
  needs elbow flexion of ≥ 30° somewhere in the block (`HINGE_MIN_FLEX_DEG`).
  Curls cover it; otherwise add three slow elbow bends.
- **Straps:** if a strap slips or is adjusted, the rest of the block has a
  different mounting. End the block there (nodes off, charger, offload) and
  start a new one.

#### 5.5.3 Why the torso node goes on the front of the chest

The facing is recovered on the assumption that the torso node's face points
out of the chest [`TORSO_FORWARD_IN_SENSOR` in `calibrate_segments.py`]. A node
on the side of the trunk is flagged low-confidence and loses the facing. If
the torso spot is ever standardized elsewhere, that constant has to move with
it. Placement otherwise needs to be repeatable, not aligned: the freeze lines
each node up with its bone, and a repeatable placement also lets
`calibrate_segments.py verify` reuse a cached calibration.

#### 5.5.4 Where the analysis ends

No closing pose is needed. The end of the log is read node by node
[`calibrate_segments.find_session_end`]: a node laid on the charger rests
still, in a pose no body holds (tilted more than 60° from the freeze,
[`OFF_BODY_TILT_DEG`]), until the log ends. Nodes come off one at a time, so
the **first** node found lying off the body marks the take-off, even while
another is still being handled. The analysis ends where taking them off
began: at the last whole-body pause within 10 s before that
[`HANDLING_MAX_MS`], otherwise 5 s before it [`HANDLING_MARGIN_MS`]. If the
log ends in motion, or with the nodes still worn, the analysis runs to the end
and the block check says so. The review page's Timeline lets a person correct
either detection, which re-runs the analysis with `--window t0,t1` /
`--end t` ([SOP §8](SOP.md#8-reviewing-the-analysis)).

#### 5.5.5 The block check: what each line reads

`hulc-ms analyze` (and the app) ends with `BLOCK CHECK: OK` or `REDO` and one
line per check, also saved as `block_check.json`. What each check is computed
from (the operator's version, with what to do, is
[SOP §10.1](SOP.md#101-the-block-check-line-by-line)):

| Check | Looks at | REDO / NOTE when | What to do |
|---|---|---|---|
| Sync | `aligned.quality.json` | REDO: a node not aligned, or vector peak ratio < 1.25 | Bigger sync movement that moves every node |
| Clock | `clock_restarts` | NOTE: a node rebooted mid-block | Charge before the battery runs out |
| Logging rate | each log's median rate | NOTE: under 8 Hz | Firmware version, flash watermark |
| Freeze | calibration `neutral.found_by` | REDO: no clean freeze after the sync movement (calibration fell back to a weaker choice); NOTE: a node was not fully still (pose spread > 3°) | Freeze ~5 s right after the movement, palms to the thighs |
| Facing | calibration `heading` | NOTE: front direction unknown, angles relative only | Torso node flat on the sternum; no torso node: include elbow bends |
| Session end | calibration `end` | NOTE: nodes not seen coming off | `--end <t_ms>` if the tail includes taking them off |
| Trunk | metrics: torso median tilt from the freeze | REDO: over 30° for most of the session — the torso was calibrated in the wrong pose (e.g. its node still on the table) or its strap moved | Strap the torso node on before the sync movement; or `--window` the real freeze |
| Angles | metrics plausibility | REDO: a joint outside physiological limits for > 25% of the session (a calibration / facing error); NOTE: implausible angles | Check the freeze pose, the facing and strap slip |
| Model fit | OpenSense `fit_lost` | NOTE: the model lost the sensors for some seconds | Compare with the Sensors view there |

---

## 6. Clock alignment

Each node keeps its own clock, so their timestamps do not agree. This chapter
explains how `tools/reconcile_nodes.py` finds the differences between the
clocks from the motion itself, and maps every node onto one shared timeline.
The hardware side of the decision (why this is done afterwards, from the
motion, rather than over Bluetooth) is in
[§4.5](#45-multi-node-time-alignment).

```mermaid
flowchart TD
    L["Per-node logs<br/>(t_ms on each node's own clock + quaternion)"] --> V["World-frame angular velocity<br/>per node (6.2.1)"]
    V --> X["Cross-correlate against the reference node<br/>(n0) over all lags (6.3)"]
    X --> P{"Distinct peak?<br/>ρ ≥ 1.25 and c ≥ 0.10"}
    P -- yes --> O["Lag → clock offset,<br/>parabolic sub-sample refinement (6.3.1–6.3.2)"]
    P -- no --> SP["Fall back: angular speed (6.2–6.4)"] --> O
    O --> D["Optional drift fit (6.5),<br/>off by default"]
    D --> M["Map every t_ms onto one timeline (6.6)<br/>→ aligned.csv + aligned.quality.json"]
```

### 6.1 The problem

Every node stamps its samples with its own millisecond counter (`millis()`),
which starts at zero when the node is switched on and ticks on the node's own
crystal. If a real event happens at time $\tau$, node $X$ records it at

```math
t_X(\tau) = \bigl(\tau - \text{boot}_X\bigr)\cdot 1000 \quad[\text{ms since node } X \text{ booted}]
```

where $\text{boot}_X$ is when node $X$ was switched on. Two nodes therefore
disagree in two ways:

1. **Offset.** They were switched on at different times
   ($\text{boot}_A \neq \text{boot}_B$), so their clocks start from different
   points.
2. **Drift (skew).** Their crystals tick at slightly different rates (tens of
   ppm), so the offset slowly changes over time.

"Reconciling" means finding, for each node $B$, how to convert its timestamps
onto the clock of the reference node $A$ (the first node):

```math
t_{\text{on }A} = t_B + \text{offset} + \text{drift}\cdot\bigl(t_B - t_{B,0}\bigr) \quad (1)
```

The offset (ms) and the drift (a rate, reported in ppm) are the two numbers to
estimate. This two-number clock model is the standard one used by network
time protocols such as NTP and PTP.

Because the nodes are wireless and analysis happens afterwards, both numbers
are recovered **from the recorded motion itself**, which makes them
independent of Bluetooth delays ([§4.5](#45-multi-node-time-alignment)).

---

> **The default method is the one in §6.2.1.** Sections 6.2–6.4 first describe
> the original method, which matches the nodes' angular *speed*; it is still
> the fallback (`--sync speed`). The default (`--sync vector`) matches the
> full angular-velocity *vector* instead. Both share the offset and drift
> steps (§6.3.1, §6.3.2, §6.5, §6.6).

### 6.2 The motion feature: angular speed (mounting-invariant)

To compare two nodes' motion, we need a signal that looks the same on both
even though each node is strapped on at a different, unknown angle. Raw
orientations fail this test. **How fast a segment is turning** passes it: a
rigid body turns at one rate, wherever on it a sensor sits and however it is
tilted.

Each node logs unit quaternions $q_i$. The rotation between two consecutive
orientations $q_i$ and $q_{i+1}$ has the angle

```math
\theta_i = 2\,\arccos\!\bigl(\lvert\, q_i \cdot q_{i+1}\,\rvert\bigr) \quad (2)
```

Here $q_i \cdot q_{i+1}$ is the quaternion dot product, and the absolute value
picks the shorter way round (since $q$ and $-q$ are the same orientation).
Dividing by the time between the samples gives the angular speed:

```math
\omega_i = \frac{\theta_i}{\Delta t_i}, \qquad \Delta t_i = \frac{t_{i+1}-t_i}{1000} \quad (3)
```

Angular velocity is a property of the whole rigid body, so its size
$\lvert\omega\rvert$ is the same whichever way each sensor is mounted. Two
sensors that move together therefore produce matching angular-speed signals,
even though their raw quaternions differ [`angular_speed()`].

#### 6.2.1 The default feature: world-frame angular-velocity vectors

Speed throws away the axis of rotation. Because every node reports in the
**same** world frame (gravity and magnetic north), we can keep the axis too:
two nodes that move together then show the same angular-velocity *vector*,
direction included, which makes the match much sharper.

The rotation from one sample to the next is $\Delta q_i$, defined by
$q_{i+1} = \Delta q_i \otimes q_i$ ($\otimes$ is quaternion multiplication and
$q^{*}$ the inverse rotation). Its axis $\hat{\mathbf u}$ and angle give the
angular velocity in the world frame:

```math
\Delta q_i = q_{i+1} \otimes q_i^{*}, \qquad
\boldsymbol\omega_i = \hat{\mathbf u}(\Delta q_i)\,\frac{2\arccos\lvert\Delta q_{i,w}\rvert}{\Delta t_i} \quad (3')
```

[`world_angular_velocity`]. Each of the three components is resampled to a
regular time grid, and the three cross-correlations (§6.3) are added up, which
is the same as correlating the vectors with a dot product,
$\sum_k \boldsymbol\omega_A(k)\cdot\boldsymbol\omega_B(k+\ell)$. The best lag
is found as in §6.3, with the same refinement (§6.3.2).

**When is the match trusted?** Nodes on different segments (say torso and
arm) mostly move on their own, so the overall match score $c$ (the
normalized correlation: the summed dot products divided by the size of both
signals) stays low, even when one shared sync movement pins down the lag
exactly. What matters is whether the best peak **stands out**. The result is
accepted when the best peak is at least 1.25 times the highest peak more
than 0.5 s away, and $c$ is at least 0.10:

```math
\rho = \frac{C(\hat\ell)}{\max_{\lvert \ell - \hat\ell\rvert > 0.5\,\text{s}} C(\ell)} \ \ge\ 1.25
\quad\text{and}\quad c \ge 0.10 \quad (9')
```

[`VECTOR_PEAK_RATIO_MIN`, `VECTOR_PEAK_EXCLUDE_S`, `VECTOR_CONFIDENCE_MIN`]. If
either test fails, the aligner falls back to the speed method of §§6.2–6.4. The
method used and the peak ratio $\rho$ are saved in `aligned.quality.json` as
`sync_method` and `sync_peak_ratio`. On synthetic torso + upper-arm sessions
(`timing_bench.py`), the vector method matched the accuracy of perfectly
synchronized clocks, where the speed method failed.

---

### 6.3 Estimating the offset by cross-correlation

To find how far apart two clocks are, slide one node's motion signal along the
other's and find the shift where they line up best. That shift is the delay
between the clocks.

The two signals are sampled at slightly different, uneven times, so each is
first resampled onto a regular grid at $f_s$ (100 Hz by default), in time
measured from each node's own first sample ($t - t_0$). This gives the
sequences $a[n]$ (node $A$) and $b[n]$ (node $B$), plus each node's first
timestamp, $A_0$ and $B_0$ [`resample_uniform()`].

The match at each shift $k$ is the **cross-correlation**, the standard way to
estimate a time delay: subtract each signal's mean, multiply them sample by
sample at that shift, and add up.

```math
C[k] = \sum_n \bigl(a[n]-\bar a\bigr)\,\bigl(b[n-k]-\bar b\bigr) \quad (4)
```

The best shift $\hat k$ is the one with the largest score, and converting it
to seconds gives the lag $L$:

```math
\hat k = \arg\max_k C[k], \qquad L = \frac{\hat k}{f_s}\ \ [\text{s}] \quad (5)
```

A positive $L$ means $a$ lags behind $b$. The correlation is computed with the
FFT, which keeps it fast on long recordings [`_xcorr_full()`,
`best_lag_seconds()`, `lag_and_confidence()`].

#### 6.3.1 From relative-time lag to an absolute clock offset

The lag $L$ was measured on signals that each start at their own node's first
sample, so it is not yet the clock offset: the difference between the two
start times has to be added back. The two signals match when they sample the
same real motion, that is, for every $s$:

```math
\text{boot}_A + \frac{A_0 + s}{1000} \;=\; \text{boot}_B + \frac{B_0 + s - 1000L}{1000}
\quad \forall s
```

The $s$ terms cancel (the offset is constant when there is no drift), which
leaves

```math
\bigl(\text{boot}_B - \text{boot}_A\bigr)\cdot 1000 \;=\; (A_0 - B_0) + 1000L
```

The left side is exactly the offset that converts $B$'s timestamps onto $A$'s
clock ($t_A = t_B + \text{offset}$). So:

```math
\boxed{\ \text{offset} = (A_0 - B_0) + 1000L \quad[\text{ms}]\ } \quad (6)
```

The first term is bookkeeping (it puts back the different start times); the
second, $L$, is what was actually measured from the shared motion. Working
directly in absolute timestamps would give the same answer; the code uses
relative time only for numerical convenience [`estimate_offset_drift()`].

#### 6.3.2 Sub-sample refinement (parabolic interpolation)

The scores $C[k]$ exist only at whole-sample shifts, 10 ms apart at 100 Hz,
but the true best shift usually falls between two of them. Near its top, any
smooth peak is shaped like a parabola, so we fit one through the best score
and its two neighbours, $y_{-1}, y_0, y_{+1}$
($= C[\hat k -1], C[\hat k], C[\hat k +1]$), and take its highest point:

```math
\delta = \tfrac{1}{2}\,\frac{y_{-1}-y_{+1}}{\,y_{-1}-2y_0+y_{+1}\,}, \qquad \delta\in[-\tfrac12,\tfrac12] \quad (7)
```

```math
L = \frac{\hat k + \delta}{f_s}
```

This standard signal-processing step (also used to find spectral peaks and
pitch) improves the lag resolution from about 10 ms to about 1 ms.

---

### 6.4 Confidence — was there real shared motion?

Taking the largest score always produces *some* answer, even for two signals
that have nothing in common. So the result also gets a confidence: how well
the two signals agree once they are lined up at the best shift. This is the
**Pearson correlation** of the overlapping parts:

```math
r = \mathrm{corr}\bigl(a_{\text{overlap}},\, b_{\text{overlap}}\bigr)\ \text{at lag }\hat k, \qquad r\in[-1,1] \quad (8)
```

Pearson correlation divides out each signal's size, so it scores whether the
two signals have the same *shape*: a node that swings more or less than the
other still scores high if they moved *together*. The offset is accepted
only when

```math
r \ge r_{\min} \quad (r_{\min} = 0.40) \quad (9)
```

Below that, the offset is reported as unreliable: reconcile prints a
low-confidence warning, `aligned.quality.json` records it, and the block check
says REDO with a bigger sync movement [`_pearson_at_lag()`, `CONFIDENCE_MIN`].

This is the method's main limitation. Cross-correlation needs **shared
motion**: two nodes on limbs that move independently (one arm swinging, the
other still) have nothing to lock onto, and $r$ correctly drops towards 0.
That is why every block starts with a sync movement that moves every node
([§4.5](#45-multi-node-time-alignment)).

---

### 6.5 Drift (clock skew) estimation

Over a recording of a few minutes, two crystals drift apart by only a few
milliseconds. That is smaller than the measurement noise and well inside the
25–50 ms target, so estimating it would add noise rather than remove error.
Drift is therefore **off by default** and can be switched on for long
recordings.

When it is on, the lag is measured separately in $K$ windows spread over the
recording, giving pairs $(t_j, L_j)$, and a straight line $L = m\,t + c$ is
fitted through them by least squares. The slope $m$ is the drift. It is used
only if it is both statistically real (more than three times its standard
error) and big enough to matter (it moves the offset by more than
$\varepsilon_{\min}$ = 15 ms across the recording, [`DRIFT_RESOLVE_MS`]):

```math
\lvert m\rvert > 3\,\mathrm{SE}(m) \quad\text{AND}\quad \lvert m\rvert\cdot\bigl(t_{\text{last}}-t_{\text{first}}\bigr) > \varepsilon_{\min}\ (=15\text{ ms}) \quad (10)
```

Otherwise the drift is set to 0. It is reported as `drift_ppm` $= m\times 10^6$
[`_windowed_lags()`, `_lsq_slope()`].

Two safeguards keep the window measurements honest [`_aligned_overlap`,
`_windowed_lags`]:

- The windows are placed over the time **both** nodes cover, after applying
  the overall offset, so window $j$ of $A$ and of $B$ cover the same real
  time.
- A window is used only if the two nodes share motion in it (correlation of at
  least $r_{\min}$). In a window where one node sits still, the "best" lag is
  meaningless. Without this check, a line fitted through such windows once
  reported a false drift of $+15.9$ s on a real recording.

Fewer than 3 usable windows means no drift is estimated.

---

### 6.6 Applying the mapping and emitting aligned streams

With the offset (and the drift, if used), each node's timestamps are converted
onto the reference clock with Eq. (1) [`to_A_clock()`]. A common, regular time
grid is then laid over the stretch that every node covers, and each node's
orientation is estimated at each grid time by blending the two nearest samples
(normalized linear interpolation, "nlerp"):

```math
q(t) = \mathrm{normalize}\!\bigl((1-f)\,q_0 + f\,q_1\bigr), \qquad f = \frac{t - t_0}{t_1 - t_0} \quad (11)
```

If $q_0\cdot q_1 < 0$, one of them is flipped first so the blend takes the
shorter way round. For human movement at 10 Hz this is accurate enough; true
spherical interpolation (SLERP) could be substituted. The result is a CSV with
one `t_common_ms` column and each node's $q_w,q_x,q_y,q_z$, ready for the
next stage [`nlerp()`, `align_and_emit()`].

---

### 6.7 Validation

`reconcile_nodes.py --selftest` builds two synthetic nodes from the same
random motion, with a **known** offset and drift added, different mounting
angles, sensor noise and timing jitter, and checks that:

- the recovered offset is close to the true one (target: within about
  25–50 ms), and
- a **negative control** works: a second node given *independent* motion
  comes back with low confidence and is flagged.

Current result: the offset is recovered to about 15–18 ms; confidence is
$r\approx 0.93$ with shared motion and $r\approx 0.07$ with independent
motion. This shows the *math* works on synthetic data; it is **not** a check
against real hardware timing (§6.9).

---

### 6.8 Known limitations

- **Shared motion is required** (§6.4). Without it, the sync movement at the
  start of the block is what makes alignment possible.
- **The clock model is a straight line** (Eq. 1). That holds over minutes. A
  long session in which temperature changes the crystals could need the offset
  to be re-estimated piece by piece; §6.11 describes how a full-day recording
  would be handled.
- **Gaps in the logs** weaken the correlation and enlarge the leftover error
  (seen on real recordings with intermittent logging).
- **Clock restarts.** A node that reboots mid-log (a flat battery) starts its
  counter again at 0, so one log holds two unrelated clocks. The loader splits
  the log at any backward jump of more than 1 s [`CLOCK_RESET_MS`], keeps the
  longest piece, warns, and records `clock_restarts` in the quality file; the
  other pieces are not analyzed. Offloading at every charge (one block per
  log) avoids this.
- **The speed fallback ignores direction.** Two movements with the same speed
  over time but different axes look identical to it. The default vector
  method (§6.2.1) keeps the axis.

---

### 6.9 Algorithm / Technique Justification

The method combines **standard techniques**:

- **Cross-correlation to estimate a time delay**, the core of the method. The
  classic reference is Knapp & Carter, *"The Generalized Correlation Method
  for Estimation of Time Delay,"* IEEE Trans. ASSP, 1976 (the basis of GCC and
  GCC-PHAT).
- **Aligning unsynchronized sensors through a shared signal**, as is standard
  for synchronizing multiple cameras, motion-capture systems, audio tracks and
  multiple IMUs (by cross-correlating a common motion feature such as rotation
  rate).
- **An offset plus a linear drift** as the clock model, as in NTP, PTP and
  sensor networks.
- **Parabolic peak interpolation**, a textbook way to estimate a peak between
  samples.

**Status:** the implementation has **not** yet been benchmarked against
hardware ground truth.

---

### 6.10 Upgrades for a rigorous / publishable result

1. **Measure the real error.** Wire a shared electrical sync pulse to an
   input pin on every node, timestamp it with `millis()`, and report the
   actual alignment error in milliseconds instead of a confidence score.
2. **GCC-PHAT.** Whiten the cross-spectrum before the inverse FFT. This gives
   a much sharper and more robust delay peak when there is noise or there are
   gaps.
3. **Robust fitting.** Use RANSAC or weighted least squares for the offset and
   drift, so that dropouts cannot pull the estimate off.
4. **Richer feature.** Correlating the full angular-velocity vector rather
   than only its size, to remove the direction ambiguity of §6.8. This is now
   the default (§6.2.1).

---

### 6.11 Long-session drift tracking: $\text{offset}(t)$ as a fitted curve

The single straight line of §6.5 (one offset and one drift for the whole
recording) assumes the clocks drift apart at a **constant** rate. That holds
for a recording of minutes at a stable temperature. It does **not** hold for a
full day of wear, when body and room temperature change the crystals' rates
during the day (§6.8). This section describes the model for that long case,
and clears up two ways the names "offset" and "drift" can mislead.

#### 6.11.1 The model: one curve, sampled by events

There is one underlying curve: the true clock difference between nodes $B$ and
$A$ at every moment, and its slope.

```math
\text{offset}(t) \quad [\text{ms}], \qquad \text{drift}(t) = \frac{d\,\text{offset}}{dt}
```

Every moment of shared motion (the sync movement at the start of a block, or
everyday movement during the day, §6.11.2) gives one **noisy measurement** of
this curve. Cross-correlating the two nodes' motion around the event (vector,
or speed as the fallback) gives one point $\bigl(t_j,\
\widehat{\text{offset}}_j\bigr)$ through Eq. (6), with a confidence $r_j$
(Eq. 8). Reconciliation **rebuilds the curve** by fitting or interpolating
through these points.

The names can hide two things:

- **Each offset measurement stands on its own.** Every event measures
  $\text{offset}(t_j)$ directly. You never add event $j{+}1$'s offset onto
  event $j$'s; that would count it twice. Error only builds up *between*
  measurements, when the curve has to be extended forward
  ($\approx \text{drift}\times\Delta t$): it is the cost of a gap, not part of
  the model.
- **Drift is the slope, not a separate measurement.** It can only be seen once
  measurements are spread out in time. That is why the sync movement (a few
  seconds) gives a good offset but a poor drift: the time span is too short to
  rise above the noise [`DRIFT_RESOLVE_MS`, Eq. 10]. One event's offset is
  noisy too, so the right approach is neither "trust it for $N$ minutes, then
  jump to the next one" (which makes the curve jump and throws away good
  measurements) nor ignoring the earlier estimate: the measurements are
  **combined** (§6.11.2).

#### 6.11.2 Getting the anchors, and combining them over time

**Where the measurements come from.** They do not have to be deliberate. Over
a day of wear the body produces many **shared** movements (standing up,
starting to walk, turning), each giving a new
$\bigl(t_j, \widehat{\text{offset}}_j\bigr)$ through the same steps, with the
confidence test (Eq. 9) deciding which pairs of nodes it is valid for.
Sudden one-off movements (sit-to-stand, turns) are best. Repetitive movement
(steady walking) matches equally well at several shifts (§6.3, §6.4) and pins
the offset poorly even when shared. An event only re-anchors the nodes that
moved together, and different events cover different nodes, so over a day
this evens out. A node that stays still for a long time is not re-anchored,
but its signal is flat then, so a small misalignment does no harm, and its
next movement is itself a new anchor.

**How the measurements are combined.** No fixed "valid for" time is needed.
An old measurement stays useful while the error from extending it stays
within budget. That depends on how fast the drift itself wanders
(temperature changes, the *process noise*) compared with how precisely one
event pins down the offset (the *measurement noise*). Two standard methods
track offset and drift together as a running estimate instead of switching on
a timer:

- **Sliding-window least squares.** Fit a line through the recent
  $\bigl(t_j, \widehat{\text{offset}}_j\bigr)$ points within a window $W$:
  long enough for a reliable slope, short enough that the drift is roughly
  constant inside it.
- **Kalman filter.** Between events, predict
  ($\text{offset}\mathrel{+}= \text{drift}\cdot\Delta t$, with the drift
  allowed to wander slowly and the uncertainty growing); at each event, move
  the estimate towards the measurement by an amount set by their relative
  confidence. Old information fades gradually instead of being dropped, and
  the process-noise setting controls how fast.

Either way, a new event neither replaces the earlier estimate nor adds to it;
it pulls the estimate *partway* towards itself. The drift only changes with
temperature, so events need not be frequent.

**Afterwards, most of this does not matter.** "How long is an old value
good for?" is a real-time question, when only the past is available. This
pipeline works afterwards, with the whole log available at reconcile time, so
almost every moment has measurements **on both sides**. The offset at time
$t$ is then found by **interpolating between the measurements on either
side**, rather than extending an old one forward, and the error is much
smaller. Extending forward is only needed at the very start and end of the
log, or inside a gap with measurements on one side only.

#### 6.11.3 Where the code is, and the upgrade path

`reconcile_nodes.py` currently fits the **single straight line** of §6.5, the
simplest version, which is correct for a recording of minutes at a stable
temperature. Supporting full-day recordings means replacing that line with
the **sliding-window fit or Kalman filter** of §6.11.2, started from the sync
movement and fed by everyday movements, and then reading the offset by
interpolation. The per-event measurement, the confidence test and the
window measurements all exist already [`lag_and_confidence()`,
`_windowed_lags()`, `_pearson_at_lag()`]. What is new is keeping offset and
drift as a **running estimate over the whole log** instead of two fixed
numbers. Robust fitting (§6.10, item 3) and GCC-PHAT (§6.10, item 2) fit in
directly: they sharpen the individual measurements and reduce the weight of
bad ones.

---

## 7. Metrics

This chapter explains every number the analysis reports in `metrics.json`: what it
means, how it is computed, its units, whether it needs the calibration, and
the field it is stored in. The computation is in `metrics.py`; this part is a
readable guide to it.

- **Which** metrics appear for a recording depends on where the nodes were
  placed: you only get what the montage supports. The rules are in
  `motion_capabilities.py`, described in [§3.5](#35-capability-tiers).
- **How** each one is computed is explained here, and in the docstring of the
  named function in `metrics.py`.

### 7.1 The honesty contract (read this first)

A few rules apply to every number, so that a doubtful value is never
presented as a solid one.

- **Missing is reported as missing, never guessed.** A joint needs a node on
  both of its segments. If one is missing, the joint is listed in
  `blocked_joints` with the missing node named, instead of a guessed angle.
- **Uncalibrated means "relative only".** A joint or segment whose nodes were
  not calibrated still gets numbers, but they are marked `clinical: false`.
  Its zero is then "the pose during the freeze", not the anatomical zero. The
  same applies when the subject's facing could not be worked out
  (`anatomical_axes: false` at the top of the file): without it the angle
  names (flexion, abduction…) need not match the actual movement. On the
  review page these rows are dimmed in the raw view. Calibration is what
  turns a sensor's orientation into an *anatomical* angle
  ([§5.4](#54-calibration-the-sensorsegment-solve)).
- **Angles are unwrapped before measuring a range.** Angles wrap around at
  ±180°, so a real sweep through 180° would otherwise look like a jump of
  nearly 360°. The angle series is unwrapped first (`np.unwrap`), so the range
  reflects the real movement, and then shifted by whole turns so its median
  sits within ±180° (unwrapping never moves the zero). Some movements have
  stretches where they are undefined (the plane of elevation, when the arm
  hangs at the side). Those are unwrapped only within each defined stretch,
  each stretch re-centred on its own [`unwrap_runs_deg`]. Inside an undefined
  stretch the value is arbitrary, and unwrapping through it once added fake
  full turns (a real session read a range of 1023°).
- **Implausible values are flagged.** Every movement gets a plausibility check
  (§7.3): a range of more than a full turn, sample-to-sample jumps larger than
  `JUMP_MAX_DEG`, or (when the anatomical axes are known) more than 2% of
  samples outside physiological limits. A failing joint gets a
  `plausibility_warning` naming the likely causes (the calibration pose, the
  facing, a slipped sensor) instead of a silent number.
- **Only the protocol is analyzed.** By default the data is cut to the
  *analysis window*: from the freeze to where the nodes come off, both found in
  stage 5 [`trim_to_analysis`]. Strapping on, the sync movement, and taking the
  nodes off never count towards range of motion, repetitions or activity.
  Nothing is cut unless the calibration's freeze comes from this same
  recording (a calibration reused from another recording cuts nothing). If
  the nodes are not seen coming off, the end of the log stays in and the run
  says so. `--keep-pre-neutral` analyzes the whole recording.

The trust indicators from clock alignment (`sync_confidence`, gaps) are **not**
recalculated here. They travel with these numbers and should be shown
alongside them (`gates_note` in the output says so).

---

### 7.2 The shared chain: quaternion → joint angle

A joint angle describes how one segment is turned relative to its neighbour,
in the body's own directions (forward, up, right), split into the movements
clinicians name (flexion, abduction, rotation). Every joint metric gets there
in the same steps [`compute_joint_metrics`; `metrics.py` module docstring]:

```
q_seg(t) = q_WS(t) ⊗ q_SB                     apply the cached mounting offset (stage 5)
q_rel(t) = conj(q_seg_prox) ⊗ q_seg_dist      distal relative to proximal
q_anat   = conj(q_WA) ⊗ q_rel ⊗ q_WA          re-express in anatomical axes
(α,β,γ)  = euler(q_anat, sequence)            decompose in the joint's ISB/Wu sequence
angle_dof = (α|β|γ)[seq_index]                pick the slot that IS this clinical DOF
```

In words:

1. **Correct each sensor for how it sits on the bone.** `q_WS` is the sensor's
   orientation in the world; `q_SB` is the sensor-to-bone mounting offset from
   `calibration.json`. Without calibration `q_SB` is "no correction", and the
   joint is marked `clinical: false`.
2. **Take the outer segment relative to the inner one** (e.g. forearm relative
   to upper arm for the elbow). This removes the motion they share.
3. **Turn that rotation into the body's own axes.** `q_WA` is the anatomical
   frame at the freeze, from the `anatomical_frame` block of
   `calibration.json`: X = forward, Y = up (along a hanging limb), Z = X × Y =
   the subject's right. It is built from gravity plus the subject's facing,
   taken from the torso node's heading, else from the elbow's hinge axis (the
   facing that makes elbow movement a pure hinge,
   `estimate_facing_from_elbow`), else from `--facing-deg`. This step is
   needed because calibration zeroes each segment but leaves its axes pointing
   along the world compass (Z = up), while the ISB recipes assume body axes.
   Without it, a pure elbow bend would land in whichever angle the facing
   happened to put it. If the facing is not known with confidence, there is no
   `q_WA` and the joint is `clinical: false`.
4. **Split the rotation into three angles** using the joint's ISB Euler
   sequence, and keep the one that corresponds to each clinical movement.

Some details:

- **The wrist uses the hand's own neutral frame.** In the freeze the palms face
  the thighs, which is the hand turned 90° about the forearm's long axis
  [`NEUTRAL_FOREARM_DEG`] from the palms-forward position that the wrist's
  `ZXY` recipe assumes. Wrist rotations are therefore turned by that angle
  about Y before the split [`joint_frame_deg`/`joint_frame_quat`], so wrist
  flexion reads as flexion and not as radial/ulnar deviation. The elbow and
  shoulder are unaffected.
- **Left and right read the same way.** One frame serves both sides, so for a
  **left** joint the rotation is mirrored left-to-right before the split
  (`mirror_left`: `[w,x,y,z] → [w,−x,−y,z]`). A left joint then splits exactly
  like a right one, and every movement has the same clinical sign on both
  sides:

  | Joint | DOF | Positive = |
  |---|---|---|
  | elbow | `flex_ext` | flexion |
  | elbow | `pro_sup` | pronation |
  | wrist | `flex_ext` | flexion |
  | wrist | `rad_uln` | ulnar deviation |
  | shoulder | `plane_elev` | 0° abduction plane, +90° forward flexion, −90° extension |
  | shoulder | `elevation` | raised (always ≥ 0) |
  | shoulder | `axial_rot` | internal rotation |
- **The anatomy is defined once.** The Euler `sequence` and the position
  (`seq_index`) of each clinical movement come from
  `motion_capabilities.JOINTS`, the single body model. This tool never
  redefines anatomy or invents a convention.
- **Zero means the freeze pose.** Calibration makes the relative rotation zero
  in the freeze, so a calibrated angle is measured from anatomical zero.

**Singularity guard.** Every way of splitting a rotation into three angles has
a pose where two of its axes line up and two of the angles stop being
separately defined (the well-known "gimbal lock"). For a sequence like `YXY`
this happens when the middle angle is near 0 or 180° (for the shoulder: arm
hanging at the side); for one like `ZXY`, when the middle angle is near ±90°.
Within 10° of that pose [`SINGULARITY_GUARD_DEG`], the two affected angles are
marked undefined for those samples, so a joint resting there never reports a
fake 180° swing or an infinite speed. The shoulder's axial rotation is the
exception: it is reported as the *sum* of the two outer angles, which stays
well defined with the arm at the side, so only `plane_elev` is masked there
(both are masked with the arm overhead, where the sum is the undefined part).
Affected movements carry `defined_frac` (the share of samples that were well
defined) and a `singularity_note`, and their series hold `NaN` at the masked
samples, so every later step skips them the same way.

What the chain produces for one joint, as the review page graphs it (right
elbow flexion over a block of six bends, the freeze shaded at the start, the
session's max and min marked):

![The review page's movement graph: right elbow flexion / extension over time, six bends up to 124° flexion, minimum −32°](img/review-graph.png)

---

### 7.3 Joint-tier metrics

These are reported for each joint the montage supports, in `joints[]`. Each
joint has `key`, `name`, `clinical`, `decomposition` (the Euler sequence),
`dofs[]` (one entry per movement) and `reps`.

| Metric | What it tells you | Field | How it is computed | Units | Needs calibration? |
|---|---|---|---|---|---|
| **Per-DOF angle** | The joint angle over time, per movement; everything below is built from it | `dofs[].` (series) | Euler split of the relative rotation in the joint's sequence, unwrapped in radians before converting (§7.2) | ° | For an *anatomical* zero, yes; the shape is valid without |
| **Range of motion** | The smallest, largest and middle angle reached, and the span between them | `dofs[].rom` = `{min_deg,max_deg,range_deg,median_deg}` | `range = max − min` over the unwrapped angle [`_rom`/`_stats`]. `null` if the movement is undefined for the whole session | ° | Needed for a *clinical* ROM; relative only when `clinical:false` |
| **Angular velocity** | How fast the joint moved | `dofs[].velocity` = `{peak_deg_s,mean_abs_deg_s,rms_deg_s}` | `v = d(angle)/dt` [`np.gradient`, `velocity_stats`]; peak = max\|v\|, mean = mean\|v\|, RMS = √mean(v²). Computed **only within** continuous defined stretches, so a masked gap never creates a huge fake peak | °/s | No (a shape measure) |
| **Repetitions** | How many back-and-forth movements, grouped into sets | `reps` = `{count,primary_dof,bouts[]}` | Turning points on the joint's main movement [`find_rep_bouts`]: one repetition is one out-and-back swing of at least `rep_threshold` = the larger of 15° [`REP_MIN_AMPLITUDE_DEG`] and 25% of the session's 2nd–98th-percentile range. Each swing is measured from its own local extremes, so a set of small curls still counts next to a big movement elsewhere. Repetitions are grouped into **bouts** (sets): a pause at an extreme longer than 5 s [`REP_PAUSE_S`], or one swing slower than 8 s [`REP_MAX_LEG_S`], ends a bout, so curls, then rotations, then curls show as separate phases. Each bout = `{t_start_ms,t_end_ms,reps,mean_amplitude_deg}`; `count` is their total | count | No (only the shape matters) |
| **Plausibility** | Whether the numbers look physically possible | `dofs[].plausibility` = `{ok,issues[],jumps,outside_limits_frac?,limits_deg?}` | [`check_plausibility`] Flags a range over 360°, jumps over 120° between samples [`JUMP_MAX_DEG`], and (with anatomical axes) more than 2% of samples [`PLAUSIBLE_OUTSIDE_FRAC`] outside the movement's physiological limits (`plausible_deg` in `motion_capabilities`). Any failing movement adds a joint-level `plausibility_warning` | — | The limits check needs anatomical axes |

A `clinical:false` joint also carries a `warning` that spells out that its
angles are relative only.

**The main movement.** Repetitions, left/right symmetry and coordination use
each joint's declared main movement [`Joint.primary`]: elevation for the
shoulder, flexion/extension for the elbow and wrist. If none is declared, or it
is undefined all session, the movement with the largest swing is used.

**Shoulder movements** use the ISB `YXY` names rather than flexion and
abduction:

| Key | Name | Meaning |
|---|---|---|
| `plane_elev` | Plane of elevation | *Which direction* the arm is raised in: 0° = to the side (abduction, frontal plane), +90° = forward (flexion), −90° = backward (extension). Computed as the raw first YXY angle + 180°, ISB's alternative form of the same rotation, so that elevation can be reported as positive. Undefined with the arm at the side or overhead |
| `elevation` | Elevation | *How far* the arm is raised, in whichever direction: raising it 60° forward or 60° to the side both read 60°. The main movement |
| `axial_rot` | Axial rotation (internal / external) | Twist about the upper arm's long axis, internal rotation positive: the raw first + third YXY angles. ISB's own third angle changes with the plane (20° of internal rotation reads +20° with the arm out to the side but −70° with it forward); the sum reads +20° in both, and stays defined with the arm at the side. Undefined only overhead |

---

### 7.4 Segment-tier metrics

These describe each body segment on its own, one entry per node, in
`segments[]` (`segment`, `node_id`, `calibrated`, …). Speed and travel use the
**raw** orientation, which does not depend on how the node is mounted, so they
are valid with or without calibration. Elevation and posture dwell use the
**calibrated** orientation.

| Metric | What it tells you | Field | How it is computed | Units | Needs calibration? |
|---|---|---|---|---|---|
| **Angular speed** | How fast the segment turned | `angular_speed` = `{mean_deg_s,peak_deg_s}` | The angle turned between consecutive samples, `Δ = 2·arccos(\|q_t · q_{t+1}\|)`, divided by `dt` [`segment_angular_speed`]. It measures how much the bone turned, not where it points, so it does not depend on mounting. Also feeds SPARC | °/s | No |
| **Angular travel** | How much the segment moved in total, and for what share of the time | `travel` = `{travel_deg, active_time_frac}` | `travel = ∫\|angular speed\| dt`: all the turning added up, in any direction. `active_time_frac` = share of time faster than 20°/s [`ACTIVE_SPEED_DEG_S`]. These are session totals, so they are valid even when two limbs are not time-aligned [`segment_travel_deg`] | °, fraction | No |
| **Elevation** | How far the segment tilts from vertical | `elevation` = `{min,max,range,median,mean}_deg` | The angle of the bone's long axis from vertical, `arccos(R[2,2])` of the (calibrated) segment orientation per sample [`segment_elevation_series`]. Without calibration it is the board's own tilt: a real tilt, just not tied to the bone (flagged by `calibrated`) | ° | For an anatomical meaning, yes |
| **Smoothness (SPARC)** | How smooth or jerky the movement was | `smoothness_sparc` | Spectral arc length of the speed profile (Balasubramanian et al. 2015) [`sparc`]: take the frequency spectrum of the speed, scale it by its zero-frequency value, and measure the length of that curve up to a cutoff (at most 10 Hz [`SPARC_FC_MAX_HZ`], where the spectrum drops below 0.05 [`SPARC_AMP_THRESH`]). Smoother movement gives a value nearer 0, jerkier more negative (≈ −1.5 smooth … −5 or below jerky). `null` when there is almost no movement | unitless (≤ 0) | No |
| **Posture dwell** | Where the segment spent its time | `posture_dwell` = `{edges_deg, fraction}` | The share of the session spent in each elevation band [`POSTURE_BIN_EDGES_DEG`: 0/30/60/90/120/150/180°], **weighted by time** between samples so uneven sampling stays correct [`posture_dwell`] | fraction per band | **Yes**: reported only when the segment is calibrated |

---

### 7.5 Derived-tier metrics

These compare joints or segments with each other: left with right, one joint
with another, or the trunk with the arm. They appear in `derived[]` **only when
the montage supports them** [`compute_derived`]. Each entry has `target`,
`name`, `requires[]` and a `metrics{}` block, and most carry a `clinical` flag
and/or a `note`.

| Metric | What it tells you | `target` | `metrics{}` | How it is computed |
|---|---|---|---|---|
| **L/R ROM symmetry** | Whether both sides of a joint reach a similar range | `symmetry_<joint>` | `symmetry_index`, `rom_ratio`, `left_rom_deg`, `right_rom_deg`, `dof` | On each side's main movement: `symmetry_index = 100·\|L−R\| / (½(\|L\|+\|R\|))` (0 = identical, up to 200 = completely different); `rom_ratio = min/max`. `clinical` = both sides calibrated |
| **Bilateral activity asymmetry** | Which arm (or segment) was used more | `activity_asymmetry_<segment>` | `asymmetry_index`, `use_ratio`, `active_time_ratio`, `left_travel_deg`, `right_travel_deg` | From each side's segment **travel**: `asymmetry_index = 100·(R−L)/(R+L)`, from −100 to +100 (positive = right used more). A session total, so it is **valid even when the clock alignment is weak** (its `note` says so), which makes it the most useful measure for sparse montages |
| **Inter-joint coordination** | Whether two joints move together, and which leads | `coordination_<a>_<b>` | `pair`, `peak_r`, `lag_s` | The highest normalized cross-correlation between the two joints' main-movement series, and its delay [`cross_correlation`]: both series have their mean removed and are scaled to unit size, so `peak_r` is between −1 and 1; a positive `lag_s` means the second joint follows the first |
| **Trunk compensation** | How much the trunk moved during the task (e.g. leaning instead of lifting the arm) | `compensation_<...>` | `trunk_travel_deg`, `trunk_elevation_range_deg` | The torso segment's travel and elevation range. `clinical` = torso calibrated; when not, the `note` marks the movement as relative |

---

### 7.6 Constants (the tuning knobs)

All are defined at the top of `metrics.py`:

| Constant | Value | What it controls |
|---|---|---|
| `SINGULARITY_GUARD_DEG` | 10° | How close to a gimbal-lock pose an angle is marked undefined |
| `ACTIVE_SPEED_DEG_S` | 20°/s | Speed above which a segment counts as "active" (active time, activity comparison) |
| `REP_MIN_AMPLITUDE_DEG` | 15° | The smallest swing that can count as a repetition (the threshold is the larger of this and 25% of the range) |
| `REP_PAUSE_S` | 5 s | A pause at an extreme longer than this ends a set of repetitions |
| `REP_MAX_LEG_S` | 8 s | A single swing slower than this ends a set |
| `NEUTRAL_FOREARM_DEG` | 90° | How far the hand is turned from palms-forward in the freeze (wrist frame) |
| `JUMP_MAX_DEG` | 120° | A sample-to-sample jump larger than this is flagged as implausible |
| `PLAUSIBLE_OUTSIDE_FRAC` | 0.02 | Share of samples outside physiological limits that fails the plausibility check |
| `POSTURE_BIN_EDGES_DEG` | 0/30/60/90/120/150/180° | Elevation bands for posture dwell |
| `SPARC_FC_MAX_HZ` | 10 Hz | Highest frequency SPARC looks at |
| `SPARC_AMP_THRESH` | 0.05 | Spectrum level that sets SPARC's cutoff |
| `_GIMBAL_SIN` | 1e-6 | Below this \|sin(middle angle)\|, the Euler split falls back to the well-defined sum |

---

### 7.7 Output shape (`metrics.json`)

```jsonc
{
  "schema_version": "1.0",
  "subject": { ... }, "session": { ... },
  "calibration_used": true, "anatomical_axes": true,
  "sample_rate_hz": 50.0, "n_samples": 4000, "duration_s": 79.98,
  "analysis_window_ms": [t0, t1],            // freeze → nodes coming off
  "trimmed_before_neutral_s": 12.4 | null,   // setup dropped before the hold
  "trimmed_after_end_s": 8.1,                // tail dropped (taking the nodes off)
  "end_detected": true, "end_method": "pause_before_takeoff",
  "joints": [ { "key","name","clinical","decomposition",
                "dofs": [ { "key","name","plane",
                            "rom": {min_deg,max_deg,range_deg,median_deg} | null,
                            "velocity": {peak_deg_s,mean_abs_deg_s,rms_deg_s} | null,
                            "plausibility": {ok,issues,jumps,...},
                            "defined_frac"?, "singularity_note"? } ],
                "reps": {count,primary_dof,
                         bouts: [ {t_start_ms,t_end_ms,reps,mean_amplitude_deg} ]},
                "warning"?, "plausibility_warning"? } ],
  "blocked_joints": [ { "key","name","missing": [ ... ] } ],
  "segments": [ { "segment","node_id","calibrated",
                  "angular_speed": {mean_deg_s,peak_deg_s},
                  "travel": {travel_deg,active_time_frac},
                  "elevation": {min_deg,max_deg,range_deg,median_deg,mean_deg},
                  "smoothness_sparc": <float|null>,
                  "posture_dwell"?: {edges_deg,fraction} } ],
  "derived": [ { "target","name","requires": [...],
                 "metrics": { ... }, "clinical"?, "note"? } ],
  "gates_note": "clinical=false ⇒ relative-only ..."
}
```

---

### 7.8 Where the code is, and how to check it

- **The computation:** `tools/metrics.py`, in the function named in each row
  above.
- **Which metrics a montage unlocks:** `tools/motion_capabilities.py`,
  described in [§3.5](#35-capability-tiers).
- **The body model** (segments, joints, movements, Euler sequences):
  `motion_capabilities.JOINTS`.
- **Checking it without hardware:**

  ```
  python tools/metrics.py selftest
  ```

  This round-trips every Euler sequence; recovers a known flexion sweep (and a
  forearm twist) at several facings; checks the calibration and
  anatomical-axis rules, angle wrapping, the wrist's signs, the analysis-window
  cut (including its guard against a reused calibration), the plausibility
  flags and the repetition sets; and tests the SPARC and cross-correlation
  building blocks and the whole derived tier.

---

## 8. Choosing the pose solver

There are two ways to get joint angles from the sensors, and this chapter explains
which one to report, and why.

- **The default chain** (`calibrate_segments.py` → `metrics.py`): correct each
  sensor for its mounting, then compute each joint's angles directly from the
  two sensors on either side of it (chapter 7).
- **OpenSim OpenSense** (`opensense_ik.py`): fit a published musculoskeletal
  body model to all the sensors at once (inverse kinematics), then read the
  angles off the model. Two published models were tried:
  - the **Thoracoscapular Shoulder Model** (TSM; Seth et al. 2019), a model of
    the right arm built for the upper limb;
  - **Rajagopal 2016**, a full-body model built for walking.

The comparison was run on 2026-09-25 with OpenSim 4.6, after the pipeline fixes
made in the same series of changes: automatic freeze detection, facing from
the elbow hinge, the wrist in its palms-in frame, and the plausibility flags.

### 8.1 Verdict

**For a method that can be cited, use OpenSense with the Thoracoscapular
Shoulder Model** for right-arm sessions
(`--opensense-model ThoracoscapularShoulderModel.osim`):

- It is as accurate as the default chain in every condition tested (within
  about 1°), with or without a torso node.
- Its own joint coordinates agree with the angles we report.
- It fits the real recording best (median residual 4.4°, i.e. the typical
  angle between a sensor and the fitted skeleton) and solves in about 10 s.
- The method can then be described as a published model, a published
  inverse-kinematics method and ISB angle conventions (see *Methods text*
  below), instead of "our own chain".

Keep the **default chain** as the path with no extra dependencies and as a
built-in cross-check, and always look at it next to the model. Inverse
kinematics can lose track of the sensors after a fast movement and hold a
wrong pose for seconds (§8.3.1). Such stretches are now detected, shaded on the
review page and left out of that model's numbers. The review page switches
between the direct sensor view and each model's solution on the same timeline
(`analyze_session.py run … --opensense-model A.osim --opensense-model
B.osim`). Use **Rajagopal** only where TSM has no body part: the left arm and
the wrists. Report its own joint coordinates only when there is a torso node
(see finding 3).

### 8.2 Synthetic ground truth (`tools/evaluation/compare_paths.py`)

To measure accuracy you need the true angles, so this test makes up a
movement whose angles are known exactly, creates the sensor readings it would
produce, and checks how close each method gets.

**The true movement.** A 40 s session shaped like the recording protocol is
played on the Rajagopal model:

- a neutral hold (palms facing the thighs)
- a shoulder swing
- 4 curls with the palm up
- pronation/supination at 65° abduction
- a full swing with upper-arm rotation
- rest, with the trunk swaying throughout

**The sensors.** Virtual nodes are strapped on at random angles, and the
subject faces 37° away from the world's +Y direction. The TSM runs solve on a
*different* model from the one that made the true movement, so its joints do
not exactly match the "subject's", as with a real arm.

**Scoring.** Both methods get the same sensor streams and the same
calibration, and every angle goes through the same ISB definitions in
`metrics.py`.

**Conditions:**

- **clean:** 60 Hz, about 1° RMS of slowly varying sensor error
- **realistic:** 8.3 Hz (the firmware's rate when this was run; it now logs
  about 10 Hz), about 3° RMS of slowly varying error per strap (sensor plus
  soft tissue), and a 50 ms clock offset on the outer nodes

Errors are RMS, in degrees (lower is better). Each solver column sits next to
the default chain from its own run; the random draws differ slightly between
runs.

**Full right arm (torso + upper arm + forearm; + hand for Rajagopal):**

| DOF | clean: default | clean: Rajagopal | clean: default | clean: TSM | realistic: default | realistic: Rajagopal | realistic: default | realistic: TSM |
|---|---|---|---|---|---|---|---|---|
| shoulder elevation | 2.7 | 2.1 | 2.7 | **2.0** | 6.0 | 5.6 | 6.2 | **5.6** |
| shoulder axial rot. | 0.8 | 0.8 | 0.8 | 0.8 | 4.7 | 4.7 | 4.6 | 4.6 |
| elbow flexion | 0.4 | 0.3 | 0.4 | 0.3 | 2.4 | 2.4 | 2.3 | 2.3 |
| elbow pro/sup | 1.5 | 1.0 | 1.5 | 1.5 | 2.1 | 3.1 | 2.0 | 2.0 |
| wrist flex / dev | 0.6 / 0.5 | 1.0 / 0.4 | — | — | 2.1 / 5.1 | 2.3 / 5.3 | — | — |

**Torso + upper arm (the 2-node shoulder montage):**

| DOF | clean: default | clean: Rajagopal | clean: TSM | realistic: default | realistic: Rajagopal | realistic: TSM |
|---|---|---|---|---|---|---|
| plane of elevation | — | — | — | 10.7 | 10.8 | **9.8** |
| elevation | 0.9 | 0.9 | 1.0 | 5.8 | 5.8 | 6.0 |
| axial rotation | 0.5 | 0.5 | 0.5 | 4.6 | 4.6 | 5.2 |

(The plane of elevation is undefined with the arm near the side; in the clean
run the arm stayed there too often for it to be scored.)

**Upper arm + forearm (no torso; today's recordings):**

| elbow DOF | clean: default | clean: Rajagopal | clean: TSM | realistic: default | realistic: Rajagopal | realistic: TSM |
|---|---|---|---|---|---|---|
| flexion | 1.2 | 1.1 | 1.1 | 4.2 | 4.7 | 4.3 |
| pro/sup | 8.3 | 8.1 | 8.2 | 9.4 | 9.2 | 9.2 |

All three methods share the roughly 8° pronation/supination error in that
table, and it comes from the test, not the methods: the true angles were
split in the trunk's frame, but the elbow's hinge is 13° off that frame, so
part of the flexion leaks into the "true" pronation/supination. Scored
against the Rajagopal model's own coordinates instead, the default chain's
pronation/supination error is 1.9° (clean) and 4.5° (realistic).

#### 8.2.1 Findings

1. **Both methods give the same reported angles.** With every method's pose
   read out through the same ISB definitions, OpenSense and the default chain
   differ by 0.1–1.3° RMS frame by frame on the no-torso sessions, and by about
   1° or less in the tables above otherwise. Errors roughly double from clean
   to realistic, and that comes from the input (8 Hz, strap wobble, clock
   offset), not the method.
2. **The model's joints help the shoulder slightly.** Elevation is 0.4–0.7°
   better with either model when the elbow is included, because the model's
   joints pull the upper arm into a consistent pose.
3. **The models' own coordinates differ, and that decides which model can be
   cited.** OpenSense users usually report the model's own joint coordinates.
   Their RMS difference from the angles we report:

   | | full arm + torso | no torso (synthetic) | real recording (no torso) |
   |---|---|---|---|
   | TSM `pro_sup` | 0.2° | 0.6–0.7° | 1.7° |
   | TSM `elbow_flexion` | 0.3° | 3.8° | 4.3° |
   | Rajagopal `pro_sup_r` | 1.9° | 6.3–6.4° | 6.2° |
   | Rajagopal `elbow_flex_r` | 0.3° | 0.7–1.3° | 5.2° |

   - Without a torso node, the facing comes from the elbow's hinge. That hinge
     is 13° from the Rajagopal model's trunk axis, so Rajagopal places the
     upper arm 13° rotated and its pronation/supination coordinate absorbs the
     difference.
   - TSM's forearm coordinate is unaffected.
   - Without a torso node, TSM's `elbow_flexion` differs by about 4% of the
     flexion range: its elbow axis is slightly oblique, so it defines flexion
     slightly differently from the hinge frame.
4. **TSM needed three changes before it could be used.** All are listed in
   `opensense_ik.py`:
   - its default pose set to the freeze pose (the published default has the
     trunk tilted and the arm raised to the side);
   - range limits switched on for the elbow and forearm, which ship without
     them (one solve wandered to −368°);
   - the shoulder (glenohumeral) angles left without limits: they need to wrap
     around and have gimbal-lock poses at 0°/180°, where a limited solve got
     stuck with the arm overhead.
5. **Facing recovery:** from the torso node, within 0.8–3.7° (clean) and
   1.4–4.1° (realistic).

### 8.3 Real capture (upper arm + forearm, 2026-09-25)

With real data there are no true angles, so this compares the methods with
each other and checks how well each fits, rather than measuring accuracy.

| | default | Rajagopal | TSM |
|---|---|---|---|
| neutral window | auto 36.7–38.7 s (placeholder rejected) | same | same |
| facing | elbow hinge −45° | same | same |
| elbow flexion ROM | −21…145° | −18…145° | −12…145° |
| pro/sup ROM | −80…100° | −83…92° | −82…99° |
| vs default, frame by frame | — | 5.4° flex, 2.3° pro/sup | **1.8° flex, 1.4° pro/sup** |
| fit residual, median / p95 | — | 5.6° / 20° | **4.4° / 16°** |
| run time | < 1 s | ~35 s | ~10 s |

The residual is large where the sensors disagree with any rigid skeleton. On
this recording that happens during fast movements, where 8 Hz sampling and
timing between the nodes (alignment confidence 0.52) dominate.

#### 8.3.1 Solver stability: the model can lose the sensors (2026-09-27)

Re-running the same recording after the switch to the vector clock alignment
(which changed the node alignment by less than 1 ms) made TSM's fit settle in
a wrong pose for **12 s** (25.7–37.6 s into the analysis window, during the
wide shoulder swing). The elbow stuck at its 168° flexion limit while the
sensors showed an almost straight arm, and every sensor was about 90° from the
fitted skeleton. The earlier run, on the old alignment, tracked that stretch
fine. Rajagopal, on the new alignment, lost the fit for only 1.1 s in short
blips.

Inverse kinematics solves each frame starting from the previous pose. A fast
movement sampled at about 8 Hz can carry the arm past the right solution into
a wrong one, and the fit stays there until the movement brings it back. The
residual shows this clearly (median about 92° inside the stretch, against
2–6° elsewhere), so `opensense_ik.py` now marks frames where any sensor is more
than 30° [`FIT_LOST_DEG`] from the fitted skeleton as **fit lost**:

- they are left out of that model's metrics (TSM's elbow range back to
  −12…145° instead of −12…168°), and listed in `pose_solver.fit_lost`;
- `opensense_fit.csv` records the fit for every frame, and the review page
  shades those stretches on the angle graph, warns in the live panel and the
  header, and blanks them from the solver-comparison overlay.

For reporting: a model's angles are only as good as its fit, frame by frame.
Report the fit-lost time with any model-based number, and use the direct
sensor path for any long fit-lost stretch.

#### 8.3.2 Torso + upper arm: the model adds no constraint (2026-10-01)

The first real torso + upper-arm session: two nodes, new firmware at about
10.6 Hz; three sideways raises to about 125°, forward and sideways raises to
about 70°, then fast small swings. With this montage, TSM has exactly as many
free coordinates as the nodes measure (thorax 3 + shoulder 3 = 2 nodes × 3),
so the fit matches the nodes exactly: median residual **0.0°**, and its
elevation (0…126°) and axial rotation (−76…69°) equal the direct path's. A
model can only add information where the nodes measure more than it needs,
and two nodes on a ball joint never do. Rajagopal, whose shoulder has range
limits, could not follow the large raises and lost the fit for 9.9 s.

So with torso + upper arm, report the direct path (or TSM, which gives the
same) and use the model as a cross-check. The model's constraints start to
matter with a forearm node (the elbow hinge then over-determines the chain),
or with joint limits that the subject's real movement respects.

### 8.4 Methods text (draft)

> Segment orientations were measured with inertial nodes (BNO086, 9-axis
> fusion) strapped to the thorax, upper arm and forearm. Sensor-to-segment
> alignment was obtained from a static neutral pose (arms at the sides,
> palms facing the thighs), detected automatically as the first deliberate
> still hold of each recording. Joint kinematics were estimated with OpenSim 4.6
> OpenSense [Delp 2007; Seth 2018; Al Borno 2022]: IMUPlacer registered the
> nodes to the Thoracoscapular Shoulder Model [Seth 2019; Seth 2016] in the
> neutral pose, and IMU inverse kinematics solved the pose at every sample.
> The model's default pose was set to the neutral pose. Sternoclavicular and
> scapulothoracic coordinates were held at the model's resting posture, as
> the scapula was not instrumented. Ranges were widened to elbow flexion
> −15–160° and pro/supination ±100°. Shoulder, elbow and forearm angles were
> expressed from the solved segment orientations following the ISB
> recommendations [Wu 2005].

References (check volume and pages before use):

- Delp SL et al. 2007, *IEEE Trans Biomed Eng* 54(11)
- Seth A et al. 2018, *PLoS Comput Biol* 14(7)
- Al Borno M et al. 2022, *J NeuroEng Rehabil* 19:22
- Seth A, Dong M, Matias R, Delp SL 2019, *Front Neurorobot* 13:90
- Seth A et al. 2016, *PLoS ONE* 11(1)
- Wu G et al. 2005, *J Biomech* 38(5)

Limits to state with it:

- **Scapula:** held fixed, so the "glenohumeral" (shoulder-joint) motion here
  is really the upper arm relative to the thorax.
- **Validation:** OpenSense's published validation is for walking. This
  device's accuracy still needs its own check against optical motion capture
  or a goniometer.
- **Coverage:** right arm only (TSM). The left arm needs Rajagopal or a
  mirrored model.

### 8.5 Recommendations

1. **Record torso + upper arm** for shoulder work with two nodes. Every method
   gets the shoulder right there (about 6° RMS realistic, 1° clean). Use the
   upper arm + forearm pair for elbow and forearm sessions.
2. **Report** the ISB angles from the TSM-solved pose. TSM's own coordinates
   agree for pronation/supination; use the ISB flexion.
3. **The biggest gain in accuracy is still the input, not the method:** the
   realistic condition roughly doubles every error.
   - Raise the log rate from 10 Hz towards 30–60 Hz.
   - Keep the full rate through the freeze.
   - Tighten the straps.
4. **Check every model fit against the direct sensor view** (the review page's
   solver switch and the graph's "compare solvers" overlay), and report the
   fit-lost time with any model-based number (§8.3.1).
5. **Both arms:** TSM covers the right arm only. The published model could be
   mirrored for the left arm (a modification that must be reported), or
   Rajagopal used for the left, with the caveat about its own coordinates in
   finding 3.

### 8.6 Reproduce

```bash
pip install opensim
git clone --depth 1 --filter=blob:none --sparse https://github.com/opensim-org/opensim-core
(cd opensim-core && git sparse-checkout set --no-cone OpenSim/Tests/shared/ThoracoscapularShoulderModel.osim)
git clone --depth 1 --filter=blob:none --sparse https://github.com/opensim-org/opensim-models
(cd opensim-models && git sparse-checkout set Models/Rajagopal_OpenSense)
RAJ=opensim-models/Models/Rajagopal_OpenSense/Rajagopal2015_opensense.osim
TSM=opensim-core/OpenSim/Tests/shared/ThoracoscapularShoulderModel.osim
python tools/evaluation/compare_paths.py --model $RAJ                    # Rajagopal
python tools/evaluation/compare_paths.py --model $TSM --truth-model $RAJ # Thoracoscapular
```

### 8.7 Appendix — OpenSense feasibility and partial montages (2026-09-25)

The question: could OpenSim's OpenSense (Stanford, Apache-2.0 licence) replace
our own calibration and joint-angle stage, and what happens when only some
of the nodes are worn? This study came before the solver comparison above,
and answered it on synthetic data with known true angles, with no hardware
[`tools/evaluation/opensense_feasibility.py`]. Run on 2026-09-25 with OpenSim
4.6.

#### 8.7.1 Setup

- **Install:** `pip install opensim`, the official Stanford packages for
  Windows / macOS / Linux, Python 3.11–3.13. No conda needed.
- **Model:** `Rajagopal2015_opensense.osim` from `opensim-org/opensim-models`
  (the OpenSense tutorial model). It has a trunk (torso), both upper arms, the
  forearm bones (elbow and pronation/supination) and hands (wrist flexion and
  deviation). The legs, lumbar spine and pelvis position were locked; the
  pelvis rotations stand in for trunk movement.
- **Synthetic session:** 20 s at 60 Hz with a 1 s neutral hold. Every arm
  movement happens at once: shoulder flexion to 140°, abduction to 70°,
  upper-arm rotation ±35°, elbow 5–125°, pronation/supination 7–83°, wrist
  ±45° / −14–22°. The trunk tilts, leans and twists by 8–18°.
- **Virtual sensors:**
  - limb nodes strapped on at random angles; the torso node's +z axis points
    forward (with a 5° tilt);
  - the sensors' world is Z-up, and the subject faces 37° away from magnetic
    north;
  - 1° of orientation noise per sample.
- **Calibration:** the average orientation over the neutral hold (Markley's
  quaternion average), given to IMUPlacer (OpenSense's sensor-registration
  step). Inverse kinematics runs over the whole recording. Error = model
  coordinate − true angle, reported as RMS / maximum in degrees after the
  neutral hold.

#### 8.7.2 Results (wide shoulder ranges; RMS / max °)

"—" means the angle cannot be measured with that montage (no sensor on one
side of the joint), so it was locked and is not reported. UA = upper arm,
FA = forearm.

| Montage (nodes) | Trunk | Shoulder flex / add / rot | Elbow | Pro/sup | Wrist flex / dev |
|---|---|---|---|---|---|
| **A** torso + UA + FA + hand (R) | 0.7 / 0.8 / 0.6 | 4.2 / 2.9 / 5.0 (max 19) | 0.8 | 3.2 | 1.4 / 2.1 |
| **G** all 7 nodes (bilateral) | same as A | R same as A; L 2.5 / 3.0 | 0.8 (L 0.9) | 3.2 (L 2.7) | 1.4 / 2.1 (L 1.2 / 1.9) |
| **B** torso + UA | 0.7 / 0.8 / 0.6 | 4.1 / 2.7 / 5.2 | — | — | — |
| **C** torso + FA (no UA) | ok | **41 / 22 / 28** | **36** | **19** | — |
| **D** UA + FA, no torso, facing known | — (assumed upright) | 16 / 10 / 17 | 1.0 | 1.1 | — |
| **D2** UA + FA, no torso, facing unknown | — | 18 / 35 / 32 | 5.9 | **25** | — |
| **E** FA + hand, shoulder & elbow locked | — | — | wrong (63) | wrong (32) | **28 / 23** |
| **E2** FA + hand, upper chain left free | — | meaningless | meaningless | meaningless | 0.8 / 0.8 |
| **F** both UAs, no torso (facing known) | — | R 16 / 10 / 17, L 12 / 11 | — | — | — |

Speed: about 45–75 s per 1200 frames (about 40–65 ms per frame) on one
processor core, which is fine for analysis after the session.

#### 8.7.3 Findings

1. **The full set of nodes works well.** With the facing handled (point 3) and
   the shoulder's range limits widened (point 4), the elbow and wrist come out
   at about 1–2° RMS, pronation/supination at about 3°, and the trunk under 1°.
2. **The shoulder is the weak joint, because of how the model defines it.**
   Rajagopal's shoulder is a flexion → adduction → rotation sequence directly
   between the torso and the upper arm (no shoulder blade).
   - The 19° spikes all happen with the arm raised far to the side (abduction
     around 60–70°). There, the flexion and rotation errors are equal and
     opposite: the model is near gimbal lock, and one rotation is shared
     between two nearly lined-up axes.
   - The upper arm's orientation relative to the torso is still accurate: 4.5°
     RMS, 8.3° maximum.
   - These are not the ISB Y-X-Y shoulder angles we report. Switching to
     OpenSense would mean either accepting the model's shoulder definition, or
     splitting the upper-arm-to-torso rotation ourselves.
3. **OpenSense's facing correction does not reach the fitting step.**
   IMUPlacer's `base_imu_label` / `base_heading_axis` settings only affect how
   the sensors are placed on the model; the data used for inverse kinematics is
   not rotated.
   - Symptom: a constant 40° error in trunk rotation, plus 4–5° of crosstalk
     into trunk tilt and lean.
   - Fix: before writing the input file (`.sto`), rotate all orientations about
     vertical by the torso node's heading at the freeze. This is what
     `calibrate_segments.compute_heading` already provides.
4. **The model's default range limits must be widened for rehabilitation.** As
   shipped, shoulder flexion is limited to ±90° (and the arms are locked).
   With those limits, a 140° reach fails without any warning: flexion is off
   by up to 50°, and the fit bends the trunk (up to 25°) and the elbow (up to
   25°) to make up for it, which spoils joints that were measured correctly.
   The test widens flexion to −90…180° and adduction to −180…90°. Range limits
   will also hide real hypermobility, so choose limits suited to the patient.
5. **With partial montages, OpenSense cannot recover what is not measured.** A
   joint angle is only valid when **both** of its segments carry a node, which
   is the same rule our `motion_capabilities.JOINTS` already follows.
   - **Missing middle segment (C):** the forearm's orientation (3 numbers) has
     to be shared across the shoulder, elbow and pronation/supination (5
     movements), so there are more unknowns than measurements. The result is a
     plausible-looking but wrong pose (elbow 36° RMS). A torso + forearm
     montage cannot give elbow or shoulder angles.
   - **No torso node (D, F):** the elbow and pronation/supination stay good
     (about 1°) *if the facing is known*. Shoulder angles become "upper arm
     relative to an assumed upright trunk", so any real trunk sway goes into
     them (16° RMS here, with 8–18° of sway). Report them as "arm relative to
     vertical", not as shoulder joint angles.
   - **No torso and unknown facing (D2):** the wrong heading gets built into
     each sensor's calibration, and errors spread to every joint, including
     pronation/supination (25° RMS). Without a torso node the facing has to
     come from somewhere else, e.g. our `--facing-deg`.
   - **Only the outer segments (forearm + hand, E/E2):** the wrist angle is
     only right if the chain *above* the forearm node is left free to absorb
     its orientation (E2: wrist 0.8°). Lock it, and the fit distorts the wrist
     as a compromise (E: 28°). Coordinates above the forearm node come out as
     arbitrary numbers and must be masked, not reported.
   - **Coordinates with no node on either side** are not constrained at all.
     The fit leaves them anywhere, so lock them or drop them from the output.

#### 8.7.4 What adopting OpenSense would take

- **Export:** the aligned stream written as a `DataType=Quaternion` `.sto` file
  with `<body>_imu` columns (w,x,y,z), heading-corrected and rotated from Z-up
  to OpenSim's axes (`sensor_to_opensim_rotations = (−90°, 0, 0)`), plus a
  one-row calibration `.sto` from the neutral window.
- **Montage → model mapping:**
  - torso → `torso`
  - upper_arm_* → `humerus_*`
  - forearm_* → `radius_*` (it carries pronation/supination)
  - hand_* → `hand_*`
- **A prepared model:** legs locked, shoulder ranges widened, and coordinates
  that are not measured locked per session.
- **Output handling:** a mask per montage so that only measurable joints are
  reported. Our metrics (range of motion, speed, repetitions, SPARC) then run
  on the model's coordinates instead of our Euler angles. For the shoulder
  definition, see point 2.
- **Validation:** OpenSense's published validation (Al Borno et al., 2022) is
  for the lower body; the upper-limb accuracy of *this device* still needs a
  check against optical motion capture or a goniometer.

---

## 9. Changing the software safely

Changes are made in the repository and reach the lab machines as a new
standalone build. The standalone `hulc-ms` app is a frozen copy of this code:
editing files inside it does nothing useful.

### 9.1 Rules that keep it easy to change

- **Files between stages.** Every stage's input and output is a file in a
  documented format, never shared memory, so each is testable and replaceable.
- **One way to start a tool.** A tool that runs another goes through
  `hulc_paths.tool_cmd()`, which works from the repository and inside the
  standalone build alike.
- **The capture folder is the only input.** New acquisition writes capture
  folders; the analysis never talks to hardware.
- **Every change is checked twice:** `hulc-ms selftest` (synthetic data, in
  CI) and `hulc-ms regress run` (real recordings, before and after).

### 9.2 Each change

One-time setup: Python 3.11–3.13, then from the repository folder
`pip install -r requirements.txt` (+ `requirements-opensense.txt` for OpenSim).

```mermaid
flowchart LR
    B["Branch +<br/>edit tools/"] --> T["Try it:<br/>python hulc-ms.py"]
    T --> C{"selftest +<br/>regress run<br/>all pass?"}
    C -- no --> B
    C -- yes --> PR["Pull request<br/>CI: selftests +<br/>standalone builds"]
    PR --> M["Merge"] --> TAG["Tag vX.Y.Z"] --> REL["GitHub Release<br/>(builds attached)"] --> LAB["Lab machines<br/>download hulc-ms"]
```

1. **Branch**, then edit `tools/`:

   | To change | Edit |
   |---|---|
   | sync / freeze / session-end detection, facing (thresholds are named constants at the top) | `calibrate_segments.py` |
   | joint angles, ROM, reps, plausibility | `metrics.py`; joint definitions and limits in `motion_capabilities.py` |
   | the block check | `analyze_session.py` |
   | the review page | `skeleton_viewer.py` |
   | the HULC-MS app | `app.py` |
   | reading capture folders | `motion_capabilities.py` (`capture_logs`), `reconcile_nodes.py` (`load_log`) |

2. **Try it** without building anything: `python hulc-ms.py` (the app) or
   `python hulc-ms.py analyze --capture-dir … --outdir …`.
3. **Check nothing broke:**
   ```bash
   python hulc-ms.py selftest                                    # synthetic checks, every tool
   python hulc-ms.py regress run --data <lab drive>/regress_data # real recordings: all must pass
   ```
4. **Push the branch and open a pull request.** CI (`.github/workflows/selftest.yml`)
   runs the selftests; `app.yml` builds the standalone app for Windows, macOS
   and Linux (downloadable from the run's Artifacts) so it can be tried before
   merging.
5. **Merge, then tag a version** (`v0.2.0`, …): the builds are attached to a
   GitHub release, and the lab machines download the new `hulc-ms` there.
   (Building locally: `packaging/build_app.py`, with `requirements-build.txt`.)

**When the app or the review page changes,** refresh the screenshots in
`docs/img/`: `node docs/img/make_screenshots.cjs <capture folder>` (needs
Node with Playwright). Any capture folder works; a regression-library case
is a good choice. Check the images for anything identifying before
committing them.

### 9.3 The regression library

Synthetic selftests check the math; they cannot tell whether a change still
handles **real** recordings. `hulc-ms regress` runs the whole pipeline on a
library of real captures and checks each against what a person confirmed once:
where the freeze is, where the session ends, the block-check verdict, the
facing, and plausible ranges.

```bash
# before AND after any change to the analysis — every case must still pass:
hulc-ms regress run --data /path/to/regress_data

# grow the library (or "Keep as test case…" in the app): expectations are
# pre-filled from today's analysis; CHECK them against what really happened,
# edit, and commit tools/regress_cases/<name>.json
hulc-ms regress add 2026-10-07_torso_upper-arm --capture-dir ./capture/block1 --description "..."
```

The case files are in git (`tools/regress_cases/`); the recordings are
**not** — keep them on the lab drive (`<data>/<case name>/*.bin`) and point
`--data` or `$HULC_REGRESS_DATA` at it. Cases without recordings are skipped,
which is also why the library cannot run in CI. When a recording breaks the
analysis, add it as a case first, then fix the code until every case passes.

### 9.4 Changing what the analysis reads or writes

The capture folder (chapter 2) is the contract with the acquisition side. Keep
changes to it backward compatible (the analysis must still read older
folders), update chapter 2 and `validate_capture.py` with the code, and never
renumber segment codes: they are stored on the nodes.

### 9.5 Dependencies

Edit the pin in `requirements.txt` (or `requirements-opensense.txt` /
`requirements-build.txt`) and re-run the selftests and the regression library
on Python 3.11 and 3.13.

---

### 9.6 Integrating the mobile app

What it has to do to replace `hulc-ms nodes` (protocol:
[§4.3](#43-ble-protocol--talking-to-a-node); format: chapter 2):

1. Enroll each board once (`0x07` segment code), or read the stored segment.
2. At each charging checkpoint: offload every node (framed notifications,
   range re-requests until complete), and store each block as a capture folder:
   `<node>.bin` (the bytes as received) + `<node>.seg.json`; a `montage.json`
   with the subject id if the app knows it.
3. Erase a node only after its transfer is verified.
4. Export the stored folders to the computer as a tree; check it with
   `hulc-ms validate --recursive`, then analyze (HULC-MS app or
   `hulc-ms analyze`).

The open items (M1–M9) are in [`HANDOFF.md`](../HANDOFF.md#open-work).
