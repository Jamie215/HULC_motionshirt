# Technical reference: how the system works, and why

This document describes how the HULC Motion Shirt works inside: the system
and its data formats, the nodes' firmware, the design decisions, the math
behind each number, the evidence for the choice of pose solver, and how to
change the software safely.

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
| World frame | The shared reference that every node reports in: "up" from gravity and "north" from the magnetometer |
| Angular velocity / speed | How fast something is turning and about which axis (velocity), or only how fast, ignoring the axis (speed) |
| Cross-correlation | Sliding one signal along another and scoring how well they match at each shift; the best shift is the delay between them |
| Lag / offset | How far apart two clocks are (ms). Drift is how fast that offset changes, in parts per million (ppm) |
| DOF | Degree of freedom: one direction in which a joint can move (e.g. elbow flexion) |
| Euler sequence | A recipe for splitting a 3D rotation into three angles about named axes in a fixed order. The ISB (International Society of Biomechanics) recommends one per joint |
| ROM | Range of motion: the span between the smallest and largest angle reached |
| RMS | Root-mean-square: a typical size of an error or signal, which weights large values more |
| Neutral pose / freeze | The still pose held for about 5 s at the start of each block (arms down, palms to the thighs), which defines zero |
| IK | Inverse kinematics: finding the body-model pose that best matches the sensors |

---

## 1. The system and the pipeline

### 1.1 The system: today and the target

![The system today and the target: the nodes record to flash, the laptop (today) or the planned mobile app offloads them into capture folders, and the analysis on a computer produces the block check, metrics and review page](img/diagrams/system-target.png)

<!-- diagram: system-target -->
<details><summary>Mermaid source</summary>

```text
flowchart LR
    N["Nodes<br/>record to flash"]
    subgraph ACQ["Acquisition"]
        L["Laptop: hulc-ms nodes<br/>(today, interim)"]
        M["Mobile app<br/>enroll, offload, store every block<br/>(target, not built)"]
    end
    C[("Capture folders<br/>the hand-over")]
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

</details>

Solid lines exist today; dashed lines are the target.

The **capture folder** (chapter 2) is where acquisition hands over to
analysis. The two sides do not depend on each other: the mobile app can
replace `hulc-ms nodes` by writing the same folders, and the analysis, the
HULC-MS app and the regression library then work without change. A whole
export (many blocks, many subjects) is a tree of capture folders.
`hulc-ms validate --recursive` checks it, and dropping its top folder on the
HULC-MS app analyzes every block.

| Layer | Status | Code | Talks to the next layer through |
|---|---|---|---|
| Firmware | built | `firmware/firmware.ino` | BLE GATT ([§4.3](#43-ble-protocol--talking-to-a-node)) |
| Acquisition | interim: laptop | `tools/multinode_test.py` (`hulc-ms nodes`) | capture folders |
| Acquisition | **to do: mobile app** | — | capture folders, exported to the computer |
| Analysis | built | `tools/analyze_session.py` (`hulc-ms analyze`) and the stage modules | the output files (§2.5) |
| Review | built | `tools/skeleton_viewer.py` (review page), `tools/app.py` (HULC-MS app) | — |

---

### 1.2 The pipeline

Each node records the orientation of the segment it is strapped to. A
clinical **joint angle** is the relative orientation of two adjacent
segments, so what can be computed depends on where the nodes are placed: the
**montage** (chapter 3). The pipeline makes this dependency explicit and does
not report a number that the placement cannot support.

`hulc-ms analyze` (`tools/analyze_session.py`) runs steps 3–8 in order, and
the app runs it for each dropped block. Each step is a separate module that
reads files and writes files, so it can be run, tested or replaced on its own
(to run them by hand, see §9.7).

| Step | Stage | Module | What it does |
|---|---|---|---|
| 1–2 | Capture and offload | `firmware.ino`, `multinode_test.py` (`hulc-ms nodes`) | The nodes log on their own; the laptop offloads each node's `.bin` into the capture folder |
| 3 | Reconcile | `reconcile_nodes.py` | Decodes the `.bin` logs and aligns them onto one timeline from the motion itself → `aligned.csv`, plus `aligned.quality.json` (sync confidence, data gaps, clock restarts). Math: [chapter 6](#6-clock-alignment) |
| 4 | Capability | `motion_capabilities.py` | Reads the capture folder and the montage, and works out which joints and metrics are valid and which are blocked, and why (chapter 3) |
| 5 | Calibrate | `calibrate_segments.py` | Finds the sync movement, the freeze after it and the session end; solves each node's **sensor-to-segment mounting offset**; estimates the subject's facing → `calibration.json` (§5.4) |
| 6 | Metrics | `metrics.py` | Joint angles over the analysis window → range of motion, angular velocity, repetitions and plausibility flags, plus the segment and derived metrics → `metrics.json` ([chapter 7](#7-metrics)) |
| 7 | Visualize | `skeleton_viewer.py` | The review page, one self-contained HTML file → `session.html` |
| 8 | Block check | `analyze_session.py` | Reads every step's output and gives OK, NOTE or UNUSABLE for each check (§5.5.5) → `block_check.json` |
| optional | OpenSense | `opensense_ik.py` | The same session solved on a published OpenSim body model, through the same metrics and review page ([chapter 8](#8-choosing-the-pose-solver)) |

---

## 2. The capture folder (data format)

The **capture folder** is where acquisition hands over to analysis. Whatever
gets the logs off the nodes (today the laptop with `hulc-ms nodes offload`,
later the mobile app) writes one folder per block in this format, and
everything after it (`hulc-ms analyze`, the HULC-MS app, the regression
library) reads only this. As long as the format is kept, the analysis needs
no change when the acquisition side changes.

To check a folder, or a whole export of many blocks, before analyzing it:

```bash
hulc-ms validate --capture-dir ./capture/block1
hulc-ms validate --capture-dir ./phone_export --recursive
```

### 2.1 One folder per block

A **block** is one segment of the data collection: mount the nodes, collect,
then offload and erase. It is one mounting and one log per node. Its folder
holds, for every node:

| File | Required | Content |
|---|---|---|
| `<name>.bin` | yes | The node's log: its 20-byte records, back to back (§2.2) |
| `<name>.seg.json` | yes | The header naming the node and its segment (§2.3) |
| `montage.json` | no | Which node is on which segment, plus the subject and session IDs (§2.4) |

- `<name>` can be anything (`HULC-IMU-485C`, `export_1`, …), but each `.bin`
  and its `.seg.json` must share it. The node is identified by `node_id` in
  the `.seg.json`, not by the file name.
- There is one file pair per node, and at most one node per segment.
- Nothing else in the folder is read. Keep the folder whole: without the
  `.seg.json` files, a log does not say what it measured.

**Many blocks** (the export of a collection period): any folder tree whose
leaf folders are block folders. `hulc-ms validate --recursive` checks every
folder that holds `.bin` logs, and dropping the top folder on the HULC-MS app
makes one session per block folder. A suggested layout is
`<export>/<subject>/<date>_<block>/…`.

### 2.2 The log: `.bin`

The `.bin` file holds exactly the bytes that the node stores after its 4 KB
flash header (the log data region, as offload delivers it), with no header,
framing or padding added.

Each record is 20 bytes, little-endian:

| Bytes | Type | Field |
|---|---|---|
| 0–3 | `uint32` | `t_ms`: the node's own `millis()` clock |
| 4–7 | `float32` | quaternion w (real) |
| 8–11 | `float32` | quaternion x |
| 12–15 | `float32` | quaternion y |
| 16–19 | `float32` | quaternion z |

- The quaternion is the BNO086 Rotation Vector: the sensor's orientation in a
  world frame of gravity and magnetic north, which is the same frame for
  every node.
- **Each node has its own clock.** Each node counts from its own power-up.
  The analysis aligns the clocks from the motion itself (the sync movement),
  so no time sync or wall-clock time is needed in the file.
- **Rate:** about 10 records per second while moving, about 1 per second
  while still, and none in IDLE.
- **What the loader tolerates** (and `validate` reports): records with
  `t_ms = 0xFFFFFFFF` (erased flash) or a zero or non-finite quaternion are
  dropped, so a transfer hole can be **zero-filled** (keep the offsets; never
  close the gap). Also tolerated: trailing bytes that are not a whole record,
  a constant framing shift, and a clock restart (a reboot mid-block, in which
  case only the longest part is analyzed).

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
| `node_id` | yes | The node's advertised name, `HULC-IMU-` + the last 4 hex digits of its BLE MAC address; stable for each board |
| `segment` | yes | One of the segment names below |
| `code` | no | The segment code the node stores (read from the status characteristic, bytes 4–5: [§4.3](#43-ble-protocol--talking-to-a-node)) |

| Code | Segment | Code | Segment |
|---|---|---|---|
| 0 | `torso` | 4 | `forearm_r` |
| 1 | `upper_arm_l` | 5 | `hand_l` |
| 2 | `upper_arm_r` | 6 | `hand_r` |
| 3 | `forearm_l` | 0xFF | unassigned (not allowed in a capture) |

The table is `motion_capabilities.SEGMENT_CODES`, which the firmware shares.
New segments are only ever added at the end.

### 2.4 The montage: `montage.json` (optional)

If a `montage.json` is present, it is used as it is (`--montage` overrides
it; without either, the analysis builds one from the `.seg.json` files).
Write one when there is more to record than the headers carry, such as the
subject ID, the session ID or the strap positions. The full format is in
§3.2; the minimum is:

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

`hulc-ms analyze --capture-dir … --outdir OUT` writes the following files
(the app writes the same into `~/HULC_sessions/<session>/out/`):

| File | Content |
|---|---|
| `block_check.json` | The verdict (OK / UNUSABLE) and one line per check |
| `session.html` | The review page, self-contained |
| `metrics.json` | Per-joint, per-DOF angles: range of motion, speed, repetitions and plausibility; segment and derived metrics ([chapter 7](#7-metrics)) |
| `calibration.json` | The freeze window, mounting offsets, facing and session end |
| `aligned.csv` | All nodes on one timeline: `t_common_ms`, then `n<i>_qw, n<i>_qx, n<i>_qy, n<i>_qz` per node (column order from the montage) |
| `aligned.quality.json` | Per node: clock offset, sync confidence, data gaps and clock restarts |
| `montage.json` | Only when the montage was built from the headers |
| `opensense/<model>/` | One folder per body-model solve, when OpenSim is installed |

---

## 3. The montage and what it lets you compute

The **montage** is the record of where each node is placed for a session.
The **capability resolver** (`motion_capabilities.py`) maps a montage to the
joints and metrics that can be computed from it. Reconcile puts the nodes on
one timeline, the montage says which body part each aligned stream belongs
to, and the resolver says what that placement makes it possible to compute.
Every metric is built on this.

### 3.1 Why a montage is required

A node measures the **orientation of the segment** it is strapped to, not a
joint. A joint angle is the **relative orientation of the nodes on two
adjacent segments**:

| Joint | Proximal segment | Distal segment |
|---|---|---|
| shoulder | `torso` | `upper_arm` |
| elbow | `upper_arm` | `forearm` |
| wrist | `forearm` | `hand` |

![The joints between segments: torso and upper arm meet at the shoulder, upper arm and forearm at the elbow, forearm and hand at the wrist](img/diagrams/joints.png)

<!-- diagram: joints -->
<details><summary>Mermaid source</summary>

```text
flowchart LR
    T(["torso"]) ---|shoulder| UA(["upper_arm"]) ---|elbow| FA(["forearm"]) ---|wrist| H(["hand"])
```

</details>

Each rounded box is a node on a segment, and each labelled link is a joint.
A joint can be computed only when the nodes at both of its ends are worn. The
same chain exists on the left and right sides (`_l` / `_r`); the torso is
shared.

- **A joint needs both of its nodes.** Without the torso node, the analysis
  can still report how the upper arm moved in space, but it cannot separate
  shoulder motion from trunk motion. Shoulder ROM is then not computable
  (rather than approximate), and the resolver reports it as blocked and names
  the missing node.
- **The set of computable metrics depends on the placement.** Bilateral
  symmetry needs both sides, inter-joint coordination needs at least two
  joints on a limb, and trunk compensation needs the torso node.
- **An orientation is not an anatomical angle until it is calibrated**
  (§5.4).

### 3.2 Schema

`python tools/motion_capabilities.py --example` prints a montage to fill in
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
| `subject.id` | The participant ID. |
| `session.aligned_csv` | The reconcile output that this montage describes. |
| `calibration.neutral_pose` | The still pose that defines zero (e.g. `N-pose`). |
| `calibration.captured` | Whether that pose was recorded in this session. |
| `calibration.protocol` | *Optional.* `sync-first` (the default: sync movement, then the freeze) or `hold-first` (recordings made before October 2026). `--protocol` overrides it. |
| `calibration.t_window_ms` | *Optional, normally absent.* A known neutral window `[t0, t1]`, used only if the data there is still. Calibration finds the freeze itself, and `--window` overrides both. `[1000, 4000]`, the placeholder that `enroll` used to write, is ignored. |
| `calibration.functional` | Optional functional-calibration movements that were captured (e.g. a known elbow flexion to fix a joint axis). |
| `nodes[].node_id` | The board's advertised ID (`HULC-IMU-XXXX`). |
| `nodes[].column` | **The link to the reconcile output:** the per-node prefix in `aligned.csv` (`n0`, `n1`, …). |
| `nodes[].segment` | One of the segments: `torso`, `upper_arm_l/r`, `forearm_l/r`, `hand_l/r`. One node per segment. This field decides what the node represents. |
| `nodes[].landmark` | *Optional.* A description of where the node sits. It is descriptive only, and is checked against `segment` (§3.3). |
| `nodes[].calibrated` | Whether this node has a valid anatomical calibration in this session. |

**Errors that stop the analysis:** an unknown `segment`; two nodes on the
same `segment` or the same `column`; a missing `segment` or `column`.

### 3.3 Which montage is used

`analyze_session` uses, in order: `--montage` if it is given; otherwise the
capture folder's `montage.json`; otherwise one built from the nodes'
`.seg.json` headers (§5.2). If the montage disagrees with a node's header,
the analysis warns, so that a swapped node is noticed. A node's `landmark` is
descriptive only; a landmark that names a different segment from `segment`
is flagged as a likely mislabel.

### 3.4 Joints, DOFs and Euler sequences

The angle math is in `metrics.py`
([§7.2](#72-the-shared-chain-quaternion--joint-angle)). A joint's relative
rotation is split into three angles about the joint's axes in a fixed order,
its **Euler sequence**. For example, `YXY` means a rotation about Y, then
about the new X, then about the new Y; the clinical movements (DOFs) are read
from those three angles.

| Joint | DOFs | Euler sequence | Note |
|---|---|---|---|
| shoulder | plane of elevation, elevation, axial rotation | `YXY` | Ball joint with a large ROM; gimbal lock near the poles. Trunk movement contaminates it without a calibrated torso node. |
| elbow | flexion/extension, pronation/supination | `ZXY` | Pronation/supination is sensitive to the roll of the forearm node, so its axial zero is calibrated explicitly. |
| wrist | flexion/extension, radial/ulnar deviation | `ZXY` | |

### 3.5 Capability tiers

| Tier | Needs | Examples |
|---|---|---|
| **segment** | 1 node | elevation, angular speed, smoothness, posture dwell |
| **joint** | 2 adjacent nodes | angle series, ROM, joint velocity, repetition count |
| **derived** | a set of joints or segments | left/right activity asymmetry (segment pair), left/right ROM symmetry (joint pair), inter-joint coordination, trunk compensation |

**Bilateral activity asymmetry** is computed for any left/right segment pair
(one node on each side, with no joint or torso node needed) and answers
"which arm is used more". Its metrics are session totals, so they do not need
the two sides to be time-aligned. This matters because two arms moving
independently are exactly the case that reconcile aligns with low
confidence.

Every metric also declares its **quality inputs**, which a user interface
should show next to the value: `dropout` (gaps in the aligned stream),
`sensor_cal` (the BNO086's own calibration status) and `sync_confidence`
(from reconcile).

```bash
python tools/motion_capabilities.py montage.json          # readable capability report
python tools/motion_capabilities.py montage.json --json   # machine-readable
python tools/motion_capabilities.py --selftest            # the model and resolver, no montage
```

**Not done on purpose:** position or endpoint trajectories. The sensors
measure orientation only, and integrating acceleration twice drifts. Hand
position would need per-person segment lengths and would be a modelled
estimate.

---

## 4. The nodes and their firmware

This section describes: the firmware's build switches, how a node behaves,
the Bluetooth protocol that a central uses, the flash log, and the decisions
behind time alignment, the wake source and power. Bench checks for a node
are in §9.8. The firmware itself (`firmware/firmware.ino`: its header
comment, numbered SECTIONs and `handleControl()`) is the source of truth, and
the reference central is `tools/multinode_test.py`, which runs behind
`hulc-ms nodes`.

Each node is a Seeed XIAO nRF52840 (a Bluetooth microcontroller) with a
SparkFun BNO086 connected over I²C at 100 kHz (the I²C on this board is
unstable at 400 kHz). The BNO086's fused, magnetometer-referenced Rotation
Vector gives every node a quaternion in the same world frame (gravity and
magnetic north). The log goes to the XIAO's onboard 2 MB QSPI flash
(P25Q16H), which the firmware drives directly through `nrfx_qspi`, so the
bootloader on the microcontroller's internal flash is never at risk. Each
node advertises a unique name, `HULC-IMU-XXXX` (the last four hex digits of
its Bluetooth address), so no per-board firmware is needed.

### 4.1 Firmware build switches

The compile-time switches worth knowing are near the top of `firmware.ino`:

| Switch | Section | Default | What it does |
|---|---|---|---|
| `IDLE_WAKE_SOURCE` | 3 | `IDLE_WAKE_DETECTOR` | Which BNO086 sensor wakes the node from IDLE (§4.6) |
| `ACTIVE_FUSION_SOURCE` | 3b | `ACTIVE_FUSION_RV` | The Rotation Vector, or the 6-axis Game Rotation Vector (§4.7) |
| `DEFAULT_ACTIVE_HZ` | 4 | 10 | The ACTIVE log rate |
| `NOT_MOTION_TO_STATIC_MS` / `NOT_MOTION_TO_IDLE_MS` | 5 | 10 s / 60 s | The state timeouts (§4.2) |
| `ON_TABLE_DEBOUNCE_ACTIVE_MS` / `ON_TABLE_FAST_PATH_MS` | 5 | 3 s / 5 s | The shorter timeouts when the node lies flat (§4.2) |
| `DEBUG_SERIAL` | 9 | 1 | Serial logging (turn it off for a deployed build, §4.7) |

`firmware/state_machine_test/state_machine_test.ino` is a reduced test
program (the state machine only, with no BLE or flash) that was used to
investigate the IDLE reset in §4.6. It is not part of the product.

### 4.2 How a node behaves

The node runs an adaptive-power state machine and records on its own, with
no central connected:

- **`ACTIVE_RECORDING`** (moving): logs the quaternion at about 10 Hz
  (exactly `activeHz`, on scheduled slots).
- **`STATIC_POSTURE`** (after 10 s still): logs at about 1 Hz.
- **`IDLE`** (after 60 s still): logs nothing, sleeps between sensor
  interrupts, and advertises. **Offload works only in IDLE**, so the laptop
  is disconnected while the node records and connects afterwards.
- **Lying flat is faster.** When the BNO086's stability classifier reports
  `ON_TABLE` (the node lying flat and still, as on a table or charger), the
  firmware uses shorter timeouts: 3 s in ACTIVE moves it to STATIC
  (`ON_TABLE_DEBOUNCE_ACTIVE_MS`), and 5 more seconds moves it to IDLE
  (`ON_TABLE_FAST_PATH_MS`). A node taken off and laid down therefore
  reaches IDLE in about 8 s instead of 70 s.
- **Flash watermark:** once the flash is 80% full, the node halves its ACTIVE
  rate (10 → 5 → 2 Hz). At 10 Hz this happens after about 2.3 h of ACTIVE
  recording, and logging stops at about 2.6 h.

![The node state machine: IDLE wakes to ACTIVE on motion, ACTIVE drops to STATIC after 10 s still or 3 s lying flat, STATIC goes to IDLE after 60 s still or 5 more seconds lying flat, and a full flash goes straight to IDLE](img/diagrams/node-states.png)

<!-- diagram: node-states -->
<details><summary>Mermaid source</summary>

```text
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

</details>


How these timings shape the recording protocol is described in
[§5.5.2](#552-timings-and-the-settings-behind-them).

### 4.3 BLE protocol — talking to a node

This section gives what a central (the laptop today, a phone app later)
needs to find a node, enroll it, and get its log off as a capture folder
([chapter 2](#2-the-capture-folder-data-format)). The reference functions in
`tools/multinode_test.py` are named for each step.

#### 4.3.1 Finding a node, and the GATT layout

A node advertises as **`HULC-IMU-XXXX`** (the `node_id` of the capture
format) **only in IDLE**, once per second, and stops while a central is
connected. Reference: `scan()`, `connect_with_retry()`.

The node's GATT service and its characteristics use 128-bit UUIDs of the
form `A001000N-B0CE-4A4A-8F0B-0011223344FF`, where `N` is 0 for the service
and 1–5 for the characteristics:

| N | Name | Access | Size | Use |
|---|---|---|---|---|
| 1 | Quaternion stream | NOTIFY | 20 B | Debugging only (a live record, when streaming is on) |
| 2 | Control | WRITE | ≤ 9 B | Commands (§4.3.2) |
| 3 | Status | READ | 6 B | State, log size, segment |
| 4 | Log offload | NOTIFY | ≤ 200 B | The log, framed (§4.3.3) |
| 5 | Time Info | READ | 16 B | Clock-offset measurement (optional) |

**Status (N = 3), 6 bytes:**

| Byte | Field |
|---|---|
| 0 | State: 0 IDLE, 1 STATIC_POSTURE, 2 ACTIVE_RECORDING |
| 1 | Flags: bit 0 streaming, bit 1 time-synced |
| 2–3 | Log size in KB (`uint16` LE) |
| 4 | Segment config tag: `0x01` when byte 5 is a valid assignment |
| 5 | Segment code (the table in [§2.3](#23-the-header-segjson); `0xFF` unassigned) |

Reference: `read_status()`, `_read_segment()`.

**Time Info (N = 5), 16 bytes, little-endian:**

| Bytes | Field |
|---|---|
| 0–3 | `node_millis_now`: the node's `millis()` clock at the moment of reading |
| 4–11 | `sync_epoch_ms`: the Unix time in ms that the central sent with the last time-sync command (`uint64`, 0 if never synced) |
| 12–15 | `sync_millis`: the node's `millis()` at the moment it received that command |

A central converts the node's clock to Unix time as:

```math
\text{node time now} = \texttt{sync\_epoch\_ms} + \bigl(\texttt{node\_millis\_now} - \texttt{sync\_millis}\bigr)
```

That is, the time the central gave the node, plus how long the node's own
clock has run since then. Doing this for two nodes and taking the difference
gives their clock offset. The analysis does not need it, because it aligns
the clocks from the motion (§4.5); it is used for link checks. Reference:
`read_syncinfo()`, `report_offsets()`.

#### 4.3.2 Commands (Control, N = 2)

The command byte comes first, followed by its payload, e.g.
`[0x06, off0, off1, off2, off3, len0, len1, len2, len3]`.

| Command | Payload | Does | When |
|---|---|---|---|
| `0x00` / `0x01` | — | Stop / start the debug quaternion stream | Any time |
| `0x02` | `uint32` LE Unix seconds | Time sync (seconds, legacy) | Any time |
| `0x05` | `uint64` LE Unix ms | Time sync (ms) | Any time |
| `0x07` | `uint8` segment code 0–6 | Store this node's segment in its flash header (kept through power cycles and erases) | Enroll |
| `0x04` | — | Start the offload over N = 4 | **IDLE only** (ignored otherwise) |
| `0x06` | `uint32` LE offset, `uint32` LE length | Resend that byte range of the log | **IDLE only** |
| `0x03` | — | Erase the log (whole chip, verified blank; about 30 s) | After a verified offload |

**Enrolling a node** (once per board): power on one node at a time, connect,
write `[0x07, code]` for its segment, and read status bytes 4–5 back to
confirm it. Every later offload can then write the `.seg.json` from the node
itself. Reference: `enroll()`, `set_segment()`, `read_segments()`.

#### 4.3.3 Getting the log off (offload)

![The offload sequence between the central and a node: start, header with the total length, framed chunks, re-requests for missing ranges, then erase only after the transfer is verified](img/diagrams/offload.png)

<!-- diagram: offload -->
<details><summary>Mermaid source</summary>

```text
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

</details>

1. Connect to a node in IDLE (status byte 0 = 0) and subscribe to the log
   offload characteristic (N = 4). The node requests a 15–30 ms connection
   interval (§4.5).
2. Write `0x04`. The node first sends a **header**,
   `[0xFFFFFFFF][total length]` (two `uint32` LE), then **data
   notifications**, `[offset][up to 196 bytes]`, where the offset is the
   payload's position in the log. Place each payload at its offset.
3. Notifications are not acknowledged, so some are dropped. When the stream
   goes quiet (about 3 s), compare what arrived with the total, and request
   each missing range with `0x06`. Repeat until the log is complete (the
   laptop tries up to 4 passes, `OFFLOAD_MAX_ATTEMPTS`). Reference:
   `measure_offload()`, `_offload_pass()`, `_missing_ranges()`.
4. Write the bytes unchanged as the node's `.bin`; any range still missing
   stays **zero-filled** at its offset (never close the gap). Write the
   `.seg.json` from status bytes 4–5.
5. Erase with `0x03` only after the transfer is verified as complete, and
   preferably after `hulc-ms validate`. The node keeps its log until then, so
   a failed transfer can be repeated.

Two firmware details make recovery reliable. The node applies
**backpressure**: when the Bluetooth transmit buffer is full, the same
notification is retried rather than skipped (without this, the node itself
dropped about 45% of them). And recovery uses **small range requests**
rather than a full re-offload, which would repeat the same burst and tend to
drop the same data again.

Reading the laptop's output: `pass 1 (full): 3528/5300 bytes, 5 hole(s)`,
then `pass 2: re-requesting 5 range(s)...`, then `COMPLETE — saved N bytes`.
`[!] INCOMPLETE after 4 attempts` saves the file zero-filled and lists the
missing offsets; run `offload` again to fill them.

#### 4.3.4 Things a central must not do

- Offload or erase while the node is recording (offload is refused outside
  IDLE).
- Erase before the offload is verified.
- Renumber the segment codes; they are stored on deployed nodes.

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

- A 4 KB header sector is followed by 20-byte records, written append-only.
  Offload sends everything after the header (record layout:
  [§2.2](#22-the-log-bin)).
- The header (with the write pointer) is saved every `FSYNC_EVERY` (10)
  samples, so a power cut loses at most about 1 s of samples at 10 Hz.
- **Erase between blocks.** The write pointer is kept across reboots, so a
  node that restarts without being erased adds the next recording to the end
  of the log, and since `millis()` restarts at 0, the log then holds a
  backward jump in time. The analysis keeps the longest monotonic part and
  warns (`clock restart` / `non-monotonic`).
- `eraseLog()` erases the whole chip in 64 KB blocks and **checks that the
  data region reads back blank before it resets the write pointer**, so a
  failed erase leaves the log intact rather than reporting `log=0KB`.

### 4.5 Multi-node time alignment

Each sample carries the node's own `millis()`, and the clocks are aligned
afterwards from the motion: the nodes' motion during the sync movement at
the start of each block is cross-correlated (`tools/reconcile_nodes.py`;
the math is in [chapter 6](#6-clock-alignment)). Because the nodes never
stream live, Bluetooth latency does not affect the result. The target is
**25–50 ms**, a fraction of a sample at 10 Hz, and the synthetic self-test
recovers a known offset to about 15–18 ms. The method needs shared motion,
which is why the sync movement must move every node. The Bluetooth time sync
(`0x05` and Time Info) is kept for link checks only.

**Offload speed depends on the central.** The firmware requests a 15–30 ms
connection interval (`setConnectionInterval(12, 24)`); 15 ms is the shortest
that iOS accepts. A Windows desktop `bleak` central imposed about 850 ms per
read, so a full flash would take hours. In an early test with the nRF Connect
app, an iPhone moved a 48 KB log in 2.46 s (about 20 KB/s), which
corresponds to a full 2 MB flash in about 1.7 min.

**Not built: a coordinated trigger.** A shared "any node moves, every node
records" trigger with a pre-roll buffer was considered, so that a node with
little motion (e.g. the torso) does not stay in IDLE. It was not built: the
STATIC timeout and the protocol's rule of never staying still for a minute
cover this, and the trigger would cost battery and flash.

### 4.6 IDLE wake source and the ~6.5 s IDLE reset

#### 4.6.1 The switch

`firmware.ino`, Section 3:

```c
#define IDLE_WAKE_SOURCE   IDLE_WAKE_DETECTOR   // DETECTOR | CLASSIFIER | SIGMOTION
```

| | `IDLE_WAKE_DETECTOR` (default) | `IDLE_WAKE_CLASSIFIER` | `IDLE_WAKE_SIGMOTION` |
|---|---|---|---|
| Sensor | Stability Detector `0x1C` | Stability Classifier `0x13` | Significant Motion `0x12` |
| Sensing | Accelerometer only (on change) | Accelerometer + gyroscope (runs the fusion engine) | Accelerometer only (one-shot) |
| Idle reset | About every 6.7 s (harmless) | Not triggered, because the fusion engine keeps the sensor active | Not triggered |
| Idle current | About 8.8 mA | Higher (gyroscope running) | About 7.4 mA (lowest) |
| Sensitivity to slow motion | Good | Good | Poor: misses slow, held-limb motion |

The **detector** is the default because it is the only low-power source that
reliably wakes on slow, deliberate movement. SIGMOTION suits off-body
standby only. Keep the `i2cBusRecover()` safeguard in `setup()` with every
source.

The sensor's own sleep mode (devSleep) was tried and dropped: the
detector's wake events were then rarely delivered, so the node did not wake.
The sensor therefore stays awake in IDLE; going lower needs a hardware wake
(§4.7.2).

To compare the sources, build each value and test on the bench with the
serial monitor open, using slow motions as well as shakes. `handleIdle()`
logs each sensor restart as `IDLE: BNO reset — <ms> since last reset`.

#### 4.6.2 The ~6.5 s IDLE reset

In IDLE with the detector, the BNO086 restarts itself about every 6.5 s.
Tests with `state_machine_test`, changing one thing at a time, ruled out the
sensor's sleep mode, the report rate and a slow host. The reset happened
with every accelerometer-only sensor and never with the Rotation Vector.
The likely explanation is that accelerometer-only sensors leave the sensor's
processor in a low-activity state that restarts itself, while anything that
runs the fusion engine keeps it active.

It does not affect the data: IDLE logs nothing, and the firmware sets the
wake sensor up again automatically. The one real risk is a reset during an
I²C transfer, which could hold the SDA line low until the node is
power-cycled. This has never been observed, and `i2cBusRecover()` guards
against it.

#### 4.6.3 Not done: tuning SIGMOTION

SIGMOTION's threshold is an FRS record (`SIG_MOTION_DETECT_CONFIG`,
`0xC274`), which the SparkFun library cannot write. Tuning it would need the
record layout from the SH-2 Reference Manual, a way to write FRS records
(e.g. CEVA's `sh2_setFrs()`) and bench tests against slow held-limb motion,
and it may still miss slow movement.

### 4.7 Power

**Measured:** about 8.8 mA in IDLE (detector) and about 20–21 mA while
recording (full fusion), down from about 12 mA and 22 mA before the
optimization work. Battery life on the current firmware has not been
measured.

#### 4.7.1 Landed

None of these change the data format or the state logic; each is commented
where it is made in `firmware.ino`.

| Change | Effect |
|---|---|
| DC/DC regulator (`NRF_POWER->DCDCEN = 1`) | More efficient than the LDO in every state |
| Advertising only in IDLE, about every 1 s (`ADV_INTERVAL_UNITS`) | No radio use while recording |
| BNO086 report rate matched to the log rate (100 ms instead of 65 ms) | No fused samples computed only to be discarded |
| STATIC sets the Rotation Vector to about 1 Hz (`BNO_CFG_STATIC`) | About 4× fewer microcontroller wakes in STATIC; only the fusion report is changed on a state switch, to avoid SparkFun library issue #2 |
| Detector reported on change (60 s keepalive) | About 0.2 mA |
| QSPI flash deep power-down in IDLE (`qspiSleep()`/`qspiWake()`) | Small; disabling the QSPI peripheral as well made idle worse (about 9 → 11 mA, nRF52840 Errata 122) |
| Game Rotation Vector switch (`ACTIVE_FUSION_SOURCE`) | No meaningful saving; choose it only for data quality (no magnetometer, but slow yaw drift). Not compile-checked against the installed library |
| STATIC logs every report (about 1 Hz, previously 0.2 Hz) | For data quality: in simulation (`tools/evaluation/timing_bench.py`), shoulder axial rotation error fell from 6.3° to 1.9° RMS and elevation from 5.1° to 2.5°, for about 14% more of a torso node's log |
| ACTIVE logs at exactly `activeHz` (`ACTIVE_SAMPLE_SLACK_DIV`) | The old gate logged at 6.9–8.3 Hz; this logs about 20–44% more ACTIVE records. Not yet measured on hardware |

#### 4.7.2 Backlog — worth exploring

Listed roughly by benefit:

- **Lower the BNO086 rate when the watermark halves `activeHz`.** Today this
  saves flash but not power, because the sensor still reports at 10 Hz.
- **Sleep between radio polls while connected** (`__WFE()` in the
  `BLE.poll()` loop of `waitForIMUData()`), and relax the connection interval
  outside offload.
- **Production build without USB serial.** `DEBUG_SERIAL 1` keeps USB running,
  and some `Serial` calls in `setup()` and the QSPI helpers are not behind
  the macro.
- **Fewer header saves** (`FSYNC_EVERY` above 10): fewer erases and less wear,
  at the cost of slightly more data at risk on power loss.
- **LEDs:** cut the LED jumpers on the boards; the blue "connected" LED could
  blink instead of staying on.
- **Below about 8.8 mA with slow-motion wake** needs a separate low-power
  motion sensor that wakes the microcontroller, which then powers the BNO086.

---

## 5. Design decisions: identity, time, calibration, and the recording protocol

This chapter records the reasoning behind steps 3–5 of the pipeline, so that
a later change does not undo it without knowing why it was made.

### 5.1 Starting facts

- The nodes log the **full Rotation Vector** (BNO086 report `0x05`), which is
  referenced to the magnetometer, so every node's orientation is already in
  a **shared world frame**. A Game Rotation Vector would give each node its
  own arbitrary heading instead.
- Each sample is timestamped with the node's free-running **`millis()`**,
  which starts at zero when the battery is plugged in. There is no real-time
  clock.

### 5.2 Node identity: the segment is stored on the node

"Which node is where" (configuration) is kept separate from "how the sensor
sits on the bone" (calibration), because they last for different lengths of
time. The segment is a **1-byte code in the log header** (with a tag that
marks it as valid), set over BLE with `0x07` at enroll, kept through power
cycles and erases, and reported in the status characteristic (§4.3). The
codes are `motion_capabilities.SEGMENT_CODES`, the single source for the
firmware and the analysis. Every offloaded log therefore says what it
measured, and offload builds the block's montage from the nodes (§3.3).

### 5.3 Time offset versus mounting calibration

Two quantities are measured again at the start of each block. They are
independent and should not be confused:

| | Aligns | Changed by | Can be reused? | Measured by |
|---|---|---|---|---|
| **Time offset** | The nodes' clocks (in time) | **Drift** (tens of ppm, a few seconds a day) and any **restart** | **No** | The sync movement (cross-correlation in reconcile) |
| **Mounting calibration** | Sensor to segment (in space) | Every **re-mount** | **Yes** | The freeze, or reuse after a check |

The nodes stay powered while they charge, so their clocks keep running
between blocks, but they never agree: each started at a different moment and
ticks at its own rate, and a node whose battery runs flat starts again at
zero. Each block therefore measures the offset again from its own sync
movement; within a block, drift is negligible (for long recordings, see
[§6.11](#611-long-session-drift-tracking-textoffsett-as-a-fitted-curve)).
The mounting calibration is physical: it survives a power cycle and changes
only when the node is re-mounted, so it can be reused after a check. A node
that restarts in the middle of a log is caught by reconcile's clock-restart
check.

### 5.4 Calibration: the sensor-to-segment solve

A sensor reports its orientation correctly in the world frame, but it sits on
the segment at an unknown angle, the **mounting offset**. During the freeze
the subject holds a pose whose joint angles are known (arms down, palms to the
thighs), so whatever each sensor reads then is its mounting offset. Removing
it from every later reading makes the orientation the bone's, and makes
every joint angle read zero in the freeze pose.

The freeze sets each segment's zero, but its axes still follow the world
compass. To tie the joint axes to the body (X anterior, Y superior, Z right),
calibration also needs the subject's **facing**. It comes from the torso
node, otherwise from the elbow's hinge axis (which needs at least 30° of
flexion), otherwise from `--facing-deg`. It is stored as `anatomical_frame`
in `calibration.json`; without it, angles are relative only.

![The same freeze before and after calibration, side view: Raw (left) shows the sensors' own orientation; Calibrated (right) shows the arm hanging straight, which is what the freeze pose should look like](img/review-neutral-raw-vs-calibrated.png)

What `calibrate_segments.py` implements:

- **Freeze window:** `--window` if given; otherwise the montage's
  `t_window_ms` if the data there is still; otherwise detected automatically
  as the first hold of at least 3 s after the sync movement whose quietest
  2 s average at most 0.04 rad/s (`hold-first` recordings take the first
  hold of the log).
- **Session end:** detected where the nodes come off (§5.5.4), or set with
  `--end`.
- **Gating:** every metric that depends on calibration is gated on
  `calibration.captured` and each node's `calibrated` flag; when either is
  false, the metric is marked relative only. An uncalibrated segment affects
  every joint it is part of (an uncalibrated `forearm_r` flags both
  `elbow_r` and `wrist_r`).
- **Reuse after a check:** `calibration.json` stores the offsets and a
  consistency baseline, and `calibrate_segments.py verify` checks a later
  still pose against it. A small residual means the calibration can be
  reused; the same check catches a strap that slipped.

A node does not need to be aligned with the anatomy, only placed the same way
each time, because a fixed tilt is part of the mounting offset. The most
sensitive direction is roll around the limb, which affects
pronation/supination and axial rotation.

---

### 5.5 The recording protocol: what each step gives, and the settings behind it

This section explains why each step of the recording routine is there and
which pipeline or firmware setting each number comes from.

#### 5.5.1 What each step gives the pipeline

| Step | What the pipeline gets from it | Without it |
|---|---|---|
| **1 Strap on** | Nothing; this part is not analyzed. | — |
| **2 Sync movement** | One strong motion shared by every node, so that their clocks can be aligned (`reconcile_nodes.py`); the landmark that calibration uses to find the freeze; and motion that wakes every node and settles its magnetic heading. | Clocks off by milliseconds to tens of seconds, and the freeze cannot be told apart from stillness during setup |
| **3 Freeze** | The mounting offsets and the zero of every angle (`calibrate_segments.py`), plus the facing (torso node). Coming after the movement, it includes any strap shift from the twisting. | Every angle is measured against a wrong zero |
| **4 Task** | The movement of interest. With no torso node, elbow bends also give the facing. | — |
| **5 Nodes taken off** | Where the analysis ends (§5.5.4). | — |

#### 5.5.2 Timings and the settings behind them

| Step | Duration | Why (the setting) |
|---|---|---|
| Sync movement | At least 4 s, ideally about 10 s: 3–5 cycles of about 1–1.5 s | The clocks are aligned by matching the nodes' world angular-velocity vectors, and one clear shared burst gives a distinct peak (`sync_peak_ratio` ≥ 1.25, `VECTOR_PEAK_RATIO_MIN`). Calibration recognizes the movement as every node moving (≥ 0.4 rad/s) and turning together (coherence of their rotation vectors ≥ 0.8) for at least 3 s (`SYNC_MIN_RAD_S`, `SYNC_MIN_COHERENCE`, `SYNC_MIN_MS`). In real recordings, trunk twists moved the torso at only 0.4–0.8 rad/s but with a coherence of 0.84–0.96, while strapping a node on gives 0.2–0.7, so it never counts. Faster than about 2 cycles per second aliases at 10 Hz. |
| Freeze | At least 3 s, ideally about 5 s, at most 8 s; starting within 20 s of the movement | The first still stretch after the sync movement lasting at least 3 s whose quietest 2 s average at most 0.04 rad/s, about 2°/s (`SYNC_TO_HOLD_MAX_MS`, `NEUTRAL_MIN_HOLD_MS`, `NEUTRAL_QUIET_RAD_S`). Pauses in motion only slow to about 0.06–0.09 rad/s. After 10 s still, a node drops to 1 Hz (`NOT_MOTION_TO_STATIC_MS`). The palms facing the thighs define the forearm's zero, as `metrics.py` and the OpenSense models assume. |
| Rests in the task | 5–8 s | At least 5 s separates sets in the repetition count (`REP_PAUSE_S`); at most 8 s keeps the nodes at 10 Hz. |
| Still in the task | Less than 60 s | After 60 s without motion, a node goes to IDLE and stops logging until it is moved (`NOT_MOTION_TO_IDLE_MS`). |
| Elbow bends (no torso node) | At least 30° of flexion somewhere in the block | The facing then comes from the elbow's hinge (`HINGE_MIN_FLEX_DEG`). |
| Nodes taken off | Lying flat for at least 10 s | IDLE through the ON_TABLE timeouts (3 s, then 5 s, §4.2); offload needs IDLE. |

#### 5.5.3 Why the torso node goes on the front of the chest

The facing is found on the assumption that the torso node's face points out
of the chest [`TORSO_FORWARD_IN_SENSOR` in `calibrate_segments.py`]. A node on
the side of the trunk is flagged as low-confidence and the facing is lost. If
the torso position is ever standardized elsewhere, that constant has to
change with it.

#### 5.5.4 Where the analysis ends

No closing pose is needed. The end of the log is read node by node
[`calibrate_segments.find_session_end`]: a node laid down rests still, in a
pose that no body holds (tilted more than 60° from the freeze,
[`OFF_BODY_TILT_DEG`]), until the log ends. The **first** node found lying
off the body marks the start of taking the nodes off, and the analysis ends
at the last whole-body pause within 10 s before that point
[`HANDLING_MAX_MS`], otherwise 5 s before it [`HANDLING_MARGIN_MS`]. If the
log ends in motion or with the nodes still worn, the analysis runs to the end
and the block check reports it. The review page's Timeline can correct
either detection, which runs the analysis again with `--window t0,t1` or
`--end t`.

#### 5.5.5 The block check: what each line reads

`hulc-ms analyze` (and the app) ends with `BLOCK CHECK: OK` or `UNUSABLE` and
one line per check, also saved as `block_check.json` [`block_check()` in
`analyze_session.py`]:

| Check | Computed from | UNUSABLE / NOTE when |
|---|---|---|
| Sync | `aligned.quality.json` | UNUSABLE: a node is not aligned, or the vector peak ratio is below 1.25 (`PEAK_RATIO_MIN`) |
| Clock | `clock_restarts` | NOTE: a node restarted mid-block |
| Logging rate | Each log's median rate | NOTE: below 8 Hz (`MIN_ACTIVE_HZ`) |
| Freeze | Calibration `neutral.found_by` | UNUSABLE: no clean freeze after the sync movement; NOTE: a node's pose spread above 3° (`NOISY_POSE_DEG`) |
| Facing | Calibration `heading` | NOTE: facing unknown, so angles are relative only |
| Session end | Calibration `end` | NOTE: the nodes were not seen coming off |
| Trunk | Metrics: torso median tilt from the freeze | UNUSABLE: above 30° (`TRUNK_MAX_MEDIAN_DEG`), meaning the torso was calibrated in the wrong pose or its strap moved |
| Angles | Metrics plausibility | UNUSABLE: a joint outside physiological limits for more than 25% of the session (`OUTSIDE_LIMITS_UNUSABLE_FRAC`); NOTE: implausible angles |
| Model fit | OpenSense `fit_lost` | NOTE: the model lost the sensors for some seconds |

Only Sync, Freeze, Trunk and Angles can make a block UNUSABLE, because each
of them makes every angle in the block wrong; the other checks leave the
numbers usable with a caveat.

---

## 6. Clock alignment

Each node keeps its own clock, so their timestamps do not agree. This chapter
explains how `tools/reconcile_nodes.py` finds the differences between the
clocks from the motion itself, and maps every node onto one shared timeline.
The hardware side of the decision (why this is done afterwards, from the
motion, rather than over Bluetooth) is in
[§4.5](#45-multi-node-time-alignment).

![How the clocks are aligned: world-frame angular velocity per node, cross-correlation against the reference node, a peak test with a fall-back to angular speed, then the offset, an optional drift fit and one shared timeline](img/diagrams/clock-alignment.png)

<!-- diagram: clock-alignment -->
<details><summary>Mermaid source</summary>

```text
flowchart TD
    L["Per-node logs<br/>(t_ms on each node's own clock + quaternion)"] --> V["World-frame angular velocity<br/>per node (6.2.1)"]
    V --> X["Cross-correlate against the reference node<br/>(n0) over all lags (6.3)"]
    X --> P{"Distinct peak?<br/>ρ ≥ 1.25 and c ≥ 0.10"}
    P -- yes --> O["Lag → clock offset,<br/>parabolic sub-sample refinement (6.3.1–6.3.2)"]
    P -- no --> SP["Fall back: angular speed (6.2–6.4)"] --> O
    O --> D["Optional drift fit (6.5),<br/>off by default"]
    D --> M["Map every t_ms onto one timeline (6.6)<br/>→ aligned.csv + aligned.quality.json"]
```

</details>

### 6.1 The problem

Every node stamps its samples with its own millisecond counter (`millis()`),
which starts at zero when the node powers on (its battery is plugged in) and ticks on the node's own
crystal. If a real event happens at time $\tau$, node $X$ records it at

```math
t_X(\tau) = \bigl(\tau - \text{boot}_X\bigr)\cdot 1000 \quad[\text{ms since node } X \text{ booted}]
```

where $\text{boot}_X$ is when node $X$ powered on. Two nodes therefore
disagree in two ways:

1. **Offset.** They powered on at different times
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

To compare two nodes' motion, a signal is needed that looks the same on both
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
**same** world frame (gravity and magnetic north), the axis can be kept as well:
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
second, $L$, is what was measured from the shared motion. Working
directly in absolute timestamps would give the same answer; the code uses
relative time only for numerical convenience [`estimate_offset_drift()`].

#### 6.3.2 Sub-sample refinement (parabolic interpolation)

The scores $C[k]$ exist only at whole-sample shifts, 10 ms apart at 100 Hz,
but the true best shift usually falls between two of them. Near its top, any
smooth peak is shaped like a parabola, so a parabola is fitted through the best score
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

### 6.4 Confidence: was there real shared motion?

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
says UNUSABLE and asks for a bigger sync movement next block [`_pearson_at_lag()`, `CONFIDENCE_MIN`].

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

Two safeguards keep the window measurements reliable [`_aligned_overlap`,
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

- **Shared motion is required** (§6.4), which is what the sync movement at
  the start of each block provides.
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

### 6.9 Basis of the method

The method combines standard techniques: cross-correlation to estimate a
time delay (Knapp & Carter, *"The Generalized Correlation Method for
Estimation of Time Delay,"* IEEE Trans. ASSP, 1976), aligning unsynchronized
sensors through a shared signal (as is common for cameras, motion capture and
multiple IMUs), an offset plus a linear drift as the clock model (as in NTP
and PTP), and parabolic peak interpolation. It has not yet been checked
against hardware ground truth.

---

### 6.10 Upgrades for a rigorous / publishable result

1. **Measure the real error.** Wire a shared electrical sync pulse to an
   input pin on every node, timestamp it with `millis()`, and report the
   actual alignment error in milliseconds.
2. **GCC-PHAT.** Whiten the cross-spectrum before the inverse FFT, for a
   sharper and more robust delay peak with noise or gaps.
3. **Robust fitting.** Use RANSAC or weighted least squares for the offset
   and drift, so that dropouts cannot pull the estimate off.

---

### 6.11 Long-session drift tracking: $\text{offset}(t)$ as a fitted curve

The single straight line of §6.5 assumes the clocks drift apart at a
constant rate, which holds for minutes at a stable temperature but not for a
full day of wear, when temperature changes the crystals' rates. For that
case, the offset is a curve, $\text{offset}(t)$, and its slope is the drift.

Every moment of shared motion (the sync movement, or everyday movements such
as standing up or turning) gives one noisy measurement of the curve,
$\bigl(t_j, \widehat{\text{offset}}_j\bigr)$ through Eq. (6), with a
confidence $r_j$ (Eq. 8). Each measurement stands on its own; they are not
added together. Drift can only be seen once measurements are spread out in
time, which is why the short sync movement gives a good offset but a poor
drift. Repetitive movement such as steady walking pins the offset poorly.

The measurements are combined rather than replaced: either a straight line
fitted through the recent points within a sliding window, or a Kalman filter
that predicts the offset between events and moves it towards each new
measurement according to their relative confidence. Because the analysis
runs afterwards on the whole log, the offset at any time can be interpolated
between measurements on both sides, so extending an old value forward is
only needed at the ends of the log or across a gap.

`reconcile_nodes.py` currently fits the single straight line of §6.5. The
pieces for the long case already exist [`lag_and_confidence()`,
`_windowed_lags()`, `_pearson_at_lag()`]; what is new is keeping the offset
and drift as a running estimate over the whole log.

---

## 7. Metrics

This chapter explains every number the analysis reports in `metrics.json`: what it
means, how it is computed, its units, whether it needs the calibration, and
the field it is stored in. The computation is in `metrics.py`; this part is a
readable guide to it.

- **Which** metrics appear for a recording depends on where the nodes were
  placed: only what the montage supports is reported. The rules are in
  `motion_capabilities.py`, described in [§3.5](#35-capability-tiers).
- **How** each one is computed is explained here, and in the docstring of the
  named function in `metrics.py`.

### 7.1 Rules that apply to every number

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
  ([§5.4](#54-calibration-the-sensor-to-segment-solve)).
- **Angles are unwrapped before measuring a range.** Angles wrap around at
  ±180°, so a real sweep through 180° would otherwise look like a jump of
  nearly 360°. The angle series is unwrapped first (`np.unwrap`), so the range
  reflects the real movement, and then shifted by whole turns so its median
  sits within ±180° (unwrapping never moves the zero). Some movements have
  stretches where they are undefined (the plane of elevation, when the arm
  hangs at the side). Those are unwrapped only within each defined stretch,
  each stretch re-centred on its own [`unwrap_runs_deg`]. Inside an undefined
  stretch the value is arbitrary, and unwrapping through it would add false
  full turns.
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
(α,β,γ)  = euler(q_anat, sequence)            decompose in the joint's Euler sequence
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
   along the world compass (Z = up), while the Euler sequences assume body axes.
   Without it, a pure elbow bend would land in whichever angle the facing
   happened to put it. If the facing is not known with confidence, there is no
   `q_WA` and the joint is `clinical: false`.
4. **Split the rotation into three angles** using the joint's Euler
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

**Shoulder movements** use the `YXY` names rather than flexion and
abduction:

| Key | Name | Meaning |
|---|---|---|
| `plane_elev` | Plane of elevation | *Which direction* the arm is raised in: 0° = to the side (abduction, frontal plane), +90° = forward (flexion), −90° = backward (extension). Computed as the raw first YXY angle + 180°, the equivalent alternative form of the same rotation, so that elevation can be reported as positive. Undefined with the arm at the side or overhead |
| `elevation` | Elevation | *How far* the arm is raised, in whichever direction: raising it 60° forward or 60° to the side both read 60°. The main movement |
| `axial_rot` | Axial rotation (internal / external) | Twist about the upper arm's long axis, internal rotation positive: the raw first + third YXY angles. The third angle on its own changes with the plane (20° of internal rotation reads +20° with the arm out to the side but −70° with it forward); the sum reads +20° in both, and stays defined with the arm at the side. Undefined only overhead |

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

Once each node is calibrated, the joint angles still have to be worked out
from the segment orientations. The part of the analysis that does this is the
**pose solver**. Both solvers compared here take the same input, each
segment's calibrated orientation (a quaternion) at every timestamp, and
produce joint angles:

- **The default chain** (`calibrate_segments.py` → `metrics.py`) treats each
  joint separately. For the elbow, it takes the forearm's orientation
  relative to the upper arm's and splits that rotation into angles such as
  flexion and pronation (chapter 7). It is simple, fast and needs no other
  software.
- **OpenSim OpenSense** (`opensense_ik.py`) works on a published
  musculoskeletal body model as a whole. It attaches a virtual sensor to each
  matching segment of the model (IMUPlacer, from the neutral pose), then, at
  every timestamp, searches for the pose whose virtual sensors best match the
  measured orientations (inverse kinematics). The joint angles are read off
  that pose. Two models were tried: the **Thoracoscapular Shoulder Model**
  (TSM; Seth et al. 2019), a right-arm model built for the upper limb, and
  **Rajagopal 2016**, a full-body model built for walking.

Neither solver changes the orientations themselves. If a sensor's reading is
wrong (a slipped strap, magnetic disturbance, a poor freeze), both start from
the same wrong input.

OpenSense was evaluated for three reasons:

- **Physiologically possible poses.** A musculoskeletal model links the
  segments through anatomical joints with limited ranges of motion, so the
  fitted pose is expected to stay physiologically possible, whereas the
  default chain computes each joint on its own.
- **A possible gain in accuracy.** The model fits all the nodes at once, so
  an error in one sensor can be spread across the body instead of passing
  straight into one joint angle. Whether this improves accuracy can only be
  settled against a lab motion-capture system; the tests below show only
  whether the solvers agree, and how each recovers simulated ground truth.
- **An established, citable method.** OpenSense (Al Borno et al. 2022) and
  the TSM (Seth et al. 2019) are peer-reviewed and used by other groups, so
  the computation can be cited and reproduced. This does not replace
  validating the sensors themselves.

The comparison was run on 2026-09-25 with OpenSim 4.6.

### 8.1 Verdict

**For a method that can be cited, use OpenSense with the Thoracoscapular
Shoulder Model** for right-arm sessions
(`--opensense-model ThoracoscapularShoulderModel.osim`):

- It is as accurate as the default chain in every condition tested (within
  about 1°), with or without a torso node.
- Its own joint coordinates agree with the angles the analysis reports.
- It fits the real recording best (median residual 4.4°, the typical angle
  between a sensor and the fitted skeleton) and solves in about 10 s.

Keep the **default chain** as the path with no extra dependencies and as a
cross-check, and look at it next to the model. Inverse kinematics can lose
track of the sensors after a fast movement and hold a wrong pose for several
seconds (§8.3.1); such stretches are detected, shaded on the review page and
left out of that model's numbers. Use **Rajagopal** only where the TSM has no
body part (the left arm and the wrists), and report its own joint
coordinates only when there is a torso node (§8.2.1, finding 3).

### 8.2 Synthetic ground truth (`tools/evaluation/compare_paths.py`)

This test makes up a movement whose angles are known exactly, creates the
sensor readings it would produce, and checks how close each method gets. A
40 s session shaped like the recording protocol (a neutral hold, a shoulder
swing, curls, pronation/supination, a full swing with rotation, rest, with
the trunk swaying throughout) is played on the Rajagopal model. Virtual
nodes are strapped on at random angles, with the subject facing 37° away
from the world's +Y direction. The TSM solves on a different model from the
one that made the movement, as with a real arm. Both methods get the same
sensor streams and calibration, and every angle goes through the same
definitions in `metrics.py`.

- **clean:** 60 Hz, about 1° RMS of slowly varying sensor error
- **realistic:** 8.3 Hz (the firmware's rate at the time; it now logs about
  10 Hz), about 3° RMS of slowly varying error per strap, and a 50 ms clock
  offset on the outer nodes

Errors are RMS in degrees (lower is better). Each solver column sits next to
the default chain from its own run.

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

**Upper arm + forearm (no torso):**

| elbow DOF | clean: default | clean: Rajagopal | clean: TSM | realistic: default | realistic: Rajagopal | realistic: TSM |
|---|---|---|---|---|---|---|
| flexion | 1.2 | 1.1 | 1.1 | 4.2 | 4.7 | 4.3 |
| pro/sup | 8.3 | 8.1 | 8.2 | 9.4 | 9.2 | 9.2 |

The shared pronation/supination error of about 8° in this table comes from
the test, not the methods: the true angles were split in the trunk's frame,
but the elbow's hinge is 13° off that frame. Scored against the Rajagopal
model's own coordinates, the default chain's error is 1.9° (clean) and 4.5°
(realistic).

#### 8.2.1 Findings

1. **Both methods give the same angles.** OpenSense and the default chain
   differ by 0.1–1.3° RMS frame by frame without a torso node, and by about 1°
   or less otherwise. Errors roughly double from clean to realistic, which
   comes from the input (sampling rate, strap wobble, clock offset), not the
   method.
2. **The model's joints help the shoulder slightly:** elevation is 0.4–0.7°
   better with either model when the elbow is included.
3. **The models' own coordinates differ, which decides which model can be
   cited.** OpenSense users usually report the model's own joint coordinates.
   Their RMS difference from the angles the analysis reports:

   | | full arm + torso | no torso (synthetic) | real recording (no torso) |
   |---|---|---|---|
   | TSM `pro_sup` | 0.2° | 0.6–0.7° | 1.7° |
   | TSM `elbow_flexion` | 0.3° | 3.8° | 4.3° |
   | Rajagopal `pro_sup_r` | 1.9° | 6.3–6.4° | 6.2° |
   | Rajagopal `elbow_flex_r` | 0.3° | 0.7–1.3° | 5.2° |

   Without a torso node, the facing comes from the elbow's hinge, which is
   13° from the Rajagopal model's trunk axis, so Rajagopal's
   pronation/supination coordinate absorbs the difference. The TSM's
   `elbow_flexion` differs by about 4% of the range because its elbow axis
   is slightly oblique.
4. **The TSM needed three changes** (listed in `opensense_ik.py`): its
   default pose set to the freeze pose; range limits switched on for the
   elbow and forearm (one solve wandered to −368° without them); and the
   shoulder angles left without limits, because a limited solve got stuck
   with the arm overhead.
5. **Facing recovery** from the torso node: within 0.8–3.7° (clean) and
   1.4–4.1° (realistic).

### 8.3 Real capture (upper arm + forearm, 2026-09-25)

With real data there are no true angles, so this compares the methods with
each other and checks how well each fits.

| | default | Rajagopal | TSM |
|---|---|---|---|
| elbow flexion ROM | −21…145° | −18…145° | −12…145° |
| pro/sup ROM | −80…100° | −83…92° | −82…99° |
| vs default, frame by frame | — | 5.4° flex, 2.3° pro/sup | **1.8° flex, 1.4° pro/sup** |
| fit residual, median / p95 | — | 5.6° / 20° | **4.4° / 16°** |
| run time | < 1 s | ~35 s | ~10 s |

The residual is large where the sensors disagree with any rigid skeleton,
here during fast movements, where the sampling rate and the timing between
the nodes dominate.

#### 8.3.1 Solver stability: the model can lose the sensors

On a re-run of the same recording, the TSM fit settled in a wrong pose for
12 s during the wide shoulder swing: the elbow stuck at its 168° limit while
the sensors showed an almost straight arm. Inverse kinematics starts each
frame from the previous pose, so a fast movement sampled at about 8 Hz can
carry the fit past the right solution. `opensense_ik.py` therefore marks
frames where any sensor is more than 30° [`FIT_LOST_DEG`] from the fitted
skeleton as **fit lost**: they are left out of that model's metrics and
listed in `pose_solver.fit_lost`, `opensense_fit.csv` records the fit for
every frame, and the review page shades those stretches. Report the fit-lost
time with any model-based number.

#### 8.3.2 Torso + upper arm: the model adds no constraint (2026-10-01)

With torso + upper arm, the TSM has exactly as many free coordinates as the
two nodes measure (thorax 3 + shoulder 3), so its fit matches the nodes
exactly (median residual 0.0°) and gives the same angles as the default
chain. A model only adds information where the nodes measure more than it
needs, as with a forearm node, where the elbow hinge constrains the chain.
With this montage, report the default chain and use the model as a
cross-check. Rajagopal, whose shoulder has range limits, could not follow the
large raises and lost the fit for 9.9 s.

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
> computed from the solved segment orientations using the Euler sequences in
> §3.4.

References (check volume and pages before use):

- Delp SL et al. 2007, *IEEE Trans Biomed Eng* 54(11)
- Seth A et al. 2018, *PLoS Comput Biol* 14(7)
- Al Borno M et al. 2022, *J NeuroEng Rehabil* 19:22
- Seth A, Dong M, Matias R, Delp SL 2019, *Front Neurorobot* 13:90
- Seth A et al. 2016, *PLoS ONE* 11(1)

Limits to state with it: the scapula is held fixed, so the shoulder motion
is the upper arm relative to the thorax; OpenSense's published validation is
for walking, so this device's accuracy still needs its own check against
optical motion capture or a goniometer; and the TSM covers the right arm
only.

### 8.5 Recommendations

1. **Record torso + upper arm** for shoulder work with two nodes (about 6°
   RMS realistic, 1° clean, with every method), and upper arm + forearm for
   elbow and forearm sessions.
2. **Report** the joint angles computed from the TSM-solved pose. The TSM's
   own coordinates agree for pronation/supination; use the computed flexion.
3. **The input matters more than the method:** the realistic condition
   roughly doubles every error. Raise the log rate from 10 Hz towards
   30–60 Hz, keep the full rate through the freeze, and tighten the straps.
4. **Check every model fit against the direct sensor view** (the review page's
   solver switch) and report the fit-lost time (§8.3.1).
5. **Both arms:** the TSM could be mirrored for the left arm (a modification
   that must be reported), or Rajagopal used for the left, with the caveat in
   §8.2.1, finding 3.

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

### 8.7 Appendix: OpenSense feasibility and partial montages (2026-09-25)

An earlier study, on synthetic data with known true angles, asked what
OpenSense can recover when only some of the nodes are worn
[`tools/evaluation/opensense_feasibility.py`]. Setup: the Rajagopal OpenSense
model (legs, lumbar spine and pelvis position locked); a 20 s session at
60 Hz with every arm movement at once (shoulder flexion to 140°, abduction
to 70°, elbow 5–125°, pronation/supination 7–83°, wrist ±45°) and trunk sway
of 8–18°; virtual nodes at random angles with 1° of noise, the subject facing
37° from magnetic north; calibration from the neutral hold through IMUPlacer.

Results, RMS / max in degrees ("—" = not measurable with that montage; UA =
upper arm, FA = forearm):

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

Findings:

1. **The full set of nodes works well:** elbow and wrist about 1–2° RMS,
   pronation/supination about 3°, trunk under 1°. The shoulder spikes (up to
   19°) occur near gimbal lock in the model's own shoulder definition, which
   differs from the `YXY` angles the analysis reports.
2. **Facing has to be applied to the data.** IMUPlacer's heading settings
   only affect sensor placement, so all orientations must be rotated about
   vertical by the torso node's heading before fitting (as
   `calibrate_segments.compute_heading` provides); otherwise the trunk
   rotation is off by about 40°.
3. **The model's range limits must be widened for rehabilitation.** As
   shipped, shoulder flexion is limited to ±90°, and a 140° reach then fails
   silently while the fit distorts the trunk and elbow by up to 25°.
4. **A partial montage cannot recover what is not measured.** A joint angle
   is valid only when both of its segments carry a node, which is the rule
   `motion_capabilities.JOINTS` follows. Without the middle segment (C) the
   pose is plausible-looking but wrong; without a torso node (D, F) the
   shoulder angles become "arm relative to vertical" and absorb trunk sway;
   with an unknown facing (D2) errors spread to every joint; and with only
   the outer segments (E/E2), the wrist is right only if the chain above it
   is left free, while the coordinates above it are meaningless and must be
   masked.

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

The modules around the pipeline steps (§1.2), in `tools/` unless noted:

| Module | Role |
|---|---|
| `app.py` | The HULC-MS app: a local web page around `analyze_session.py` |
| `validate_capture.py` | `hulc-ms validate`: checks a capture folder against chapter 2 |
| `regress.py` | `hulc-ms regress`: the regression library of real recordings (chapter 9) |
| `hulc_paths.py` | Where the tools write, and how one tool starts another (`tool_cmd`) |
| `hulc-ms.py` (repository root) | The `hulc-ms` command, and the standalone build's entry point |
| `tools/evaluation/` | Synthetic-truth studies behind the design (`timing_bench`, `compare_paths`, `opensense_feasibility`) |

### 9.2 Each change

One-time setup: Python 3.11–3.13, then from the repository folder
`pip install -r requirements.txt` (+ `requirements-opensense.txt` for OpenSim).

![Changing the software: branch and edit, try it, run the selftests and the regression library, open a pull request, merge, tag a version, and the lab machines download the release](img/diagrams/release.png)

<!-- diagram: release -->
<details><summary>Mermaid source</summary>

```text
flowchart LR
    B["Branch +<br/>edit tools/"] --> T["Try it:<br/>python hulc-ms.py"]
    T --> C{"selftest +<br/>regress run<br/>all pass?"}
    C -- no --> B
    C -- yes --> PR["Pull request<br/>CI: selftests +<br/>standalone builds"]
    PR --> M["Merge"] --> TAG["Tag vX.Y.Z"] --> REL["GitHub Release<br/>(builds attached)"] --> LAB["Lab machines<br/>download hulc-ms"]
```

</details>

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
   python hulc-ms.py regress run --data <regression data>/      # real recordings: all must pass
   ```
4. **Push the branch and open a pull request.** CI (`.github/workflows/selftest.yml`)
   runs the selftests; `app.yml` builds the standalone app for Windows, macOS
   and Linux (downloadable from the run's Artifacts) so it can be tried before
   merging.
5. **Merge, then tag a version** (`v0.2.0`, …): the builds are attached to a
   GitHub release, and the lab machines download the new `hulc-ms` there.
   (Building locally: `packaging/build_app.py`, with `requirements-build.txt`.)

**When a diagram changes,** edit its Mermaid source in the collapsed "Mermaid
source" section just below its image (in this file or the README), then redraw the images with
`node docs/img/make_diagrams.cjs` and commit them. The diagrams are images
because GitHub's own Mermaid rendering cuts off longer labels.

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
# before and after any change to the analysis; every case must still pass:
hulc-ms regress run --data /path/to/regress_data

# grow the library (or "Keep as test case…" in the app): expectations are
# pre-filled from today's analysis; check them against what happened,
# edit, and commit tools/regress_cases/<name>.json
hulc-ms regress add 2026-10-07_torso_upper-arm --capture-dir ./capture/block1 --description "..."
```

The case files are in git (`tools/regress_cases/`); the recordings are
**not**: keep them outside the repository (`<data>/<case name>/*.bin`) and point
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
2. After each block: offload every node (framed notifications,
   range re-requests until complete), and store each block as a capture folder:
   `<node>.bin` (the bytes as received) + `<node>.seg.json`; a `montage.json`
   with the subject id if the app knows it.
3. Erase a node only after its transfer is verified.
4. Export the stored folders to the computer as a tree; check it with
   `hulc-ms validate --recursive`, then analyze (HULC-MS app or
   `hulc-ms analyze`).

### 9.7 Running the stages by hand, and fixing an analysis

**Running the stages by hand.** `hulc-ms analyze` runs these in order (logs
in montage column order):

```bash
python tools/reconcile_nodes.py <n0>.bin <n1>.bin --out aligned.csv
python tools/motion_capabilities.py montage.json
python tools/calibrate_segments.py calibrate aligned.csv montage.json --out calibration.json
python tools/metrics.py compute aligned.csv montage.json --calibration calibration.json --out metrics.json
python tools/skeleton_viewer.py render aligned.csv montage.json --calibration calibration.json --metrics metrics.json --out session.html
```

**Checking and fixing the montage.** `hulc-ms analyze` first prints which
montage it used and which log belongs to which segment
(`n0 upper_arm_r <- HULC-IMU-485C.bin`, …). The montage comes from the
folder's `montage.json`, else from the nodes' `.seg.json` files. If a node
stored the wrong segment, `--montage FILE` gives the right montage instead,
and the run warns where it disagrees with a node. A `montage.json` edited by
hand must stay UTF-8: the tools accept a UTF-8 BOM (Notepad, PowerShell `>`),
but an empty or UTF-16 file stops the analysis with `json … Expecting value`.

**Special cases within a block.**

- A node came off mid-block without an offload: analyze the part after it
  separately, with that part's own freeze (`hulc-ms analyze … --window t0,t1`).
- A battery ran flat mid-block: the node's clock starts again from zero when
  it is recharged. The analysis keeps the larger part and warns; the rest is
  not analyzed.
- Nodes swapped between segments without enrolling them again: the block's
  montage is built from what the nodes store, so it is wrong; fix it
  afterwards with `--montage`.

The files the analysis writes are listed in §2.5.

### 9.8 Bench checks

**Link check** (`hulc-ms nodes check`). The offset is only known to within
the time one Bluetooth read takes, so treat it as an upper bound. A large or
negative drift below that is a measurement artifact (`NOT RESOLVABLE`), not
the clocks; real crystal drift is far smaller. `--count 1` (one node only)
measures a single link's delay on its own, separating it from the scheduling
of several connections.

**Offload speed on any central.** The node prints `[OFFLOAD] Done` on its USB
serial with the speed (KB/s) and the time per chunk, which is about the
Bluetooth connection interval the central chose. The offload output on the
laptop also shows each node's speed.

**Re-mount check (optional).** Every block has its own freeze, so this only
confirms that a previous calibration still holds after a node was taken off
and put back. Record a short still hold, offload it into `./redon`, then:

```bash
python tools/reconcile_nodes.py ./redon/<n0>.bin ./redon/<n1>.bin --out redon.csv
python tools/calibrate_segments.py verify redon.csv montage.json --calibration calibration.json
```

**Without hardware.** `hulc-ms selftest` checks every tool on synthetic data.
For the Bluetooth side alone: `python tools/reconcile_nodes.py --selftest`
(the clock-alignment math) and `python tools/multinode_test.py selftest` (the
enroll logic).
