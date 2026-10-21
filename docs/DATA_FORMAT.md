# Data format — the capture folder

The **capture folder** is where acquisition hands over to analysis. Whatever
gets the logs off the nodes — today the laptop (`hulc-ms nodes offload`),
later the mobile app — writes one folder per block in this format, and
everything after it (`hulc-ms analyze`, the HULC-MS app, the regression
library) reads only this. Keep to it and the analysis needs no change when the
acquisition side does.

Check a folder, or a whole export of many blocks, before analysing:

```bash
hulc-ms validate --capture-dir ./capture/block1
hulc-ms validate --capture-dir ./phone_export --recursive
```

## 1. One folder per block

A **block** is one mounting of the nodes, from strapping on to taking off
(the [recording card](COLLECTION_SOP.md)). Its folder holds, for every node:

| File | Required | Content |
|---|---|---|
| `<name>.bin` | yes | the node's log: its 20-byte records, back to back (§2) |
| `<name>.seg.json` | yes | the header naming the node and its segment (§3) |
| `montage.json` | no | which node is on which segment, plus subject / session ids (§4) |

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

## 2. The log: `.bin`

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
  analysed).

## 3. The header: `.seg.json`

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
| `code` | no | the segment code the node stores (read from the status characteristic, bytes 4–5: [`BLE_PROTOCOL.md`](BLE_PROTOCOL.md)) |

| Code | Segment | Code | Segment |
|---|---|---|---|
| 0 | `torso` | 4 | `forearm_r` |
| 1 | `upper_arm_l` | 5 | `hand_l` |
| 2 | `upper_arm_r` | 6 | `hand_r` |
| 3 | `forearm_l` | 0xFF | unassigned (not allowed in a capture) |

The table is `motion_capabilities.SEGMENT_CODES`, shared with the firmware;
new segments are only ever appended.

## 4. The montage: `montage.json` (optional)

If present it is used as is (`--montage` overrides it; without either, the
analysis builds one from the `.seg.json` files). Write it when there is more to
record than the headers carry — the subject id, the session id, strap
positions. The format is in [`MONTAGE_SCHEMA.md`](MONTAGE_SCHEMA.md); the
minimum:

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

## 5. What the analysis writes

`hulc-ms analyze --capture-dir … --outdir OUT` (the app writes the same into
`~/HULC_sessions/<session>/out/`):

| File | Content |
|---|---|
| `block_check.json` | the verdict (OK / REDO) and one line per check |
| `session.html` | the review page, self-contained |
| `metrics.json` | per-joint, per-DOF angles: range of motion, speed, reps, plausibility; segment and derived metrics |
| `calibration.json` | the freeze window, mounting offsets, facing, session end |
| `aligned.csv` | all nodes on one timeline: `t_common_ms`, then `n<i>_qw, n<i>_qx, n<i>_qy, n<i>_qz` per node (column order from the montage) |
| `aligned.quality.json` | per node: clock offset, sync confidence, data gaps, clock restarts |
| `montage.json` | only when the montage was built from the headers |
| `opensense/<model>/` | per body-model solve, when OpenSim is installed |

The metrics are described in [`METRICS.md`](METRICS.md).
