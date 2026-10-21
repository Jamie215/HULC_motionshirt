# Architecture — today, the target, and where they meet

## The target system

```
 nodes ──BLE──▶ mobile app ──(after the collection period)──▶ computer ──▶ analysis ──▶ review
 (record to     (enroll, offload,        export of capture             (block check,     (review page,
  flash)         store all blocks)        folders                        metrics)          HULC-MS app)
```

## What exists now

```
 nodes ──BLE──▶ laptop: hulc-ms nodes offload ──▶ capture folder ──▶ analysis ──▶ review
                (the interim stand-in for the mobile app)
```

The **capture folder** ([`DATA_FORMAT.md`](DATA_FORMAT.md)) is the seam. Left
of it is acquisition, right of it analysis, and neither side knows the other:
the mobile app replaces `hulc-ms nodes` by writing the same folders, and the
analysis, the HULC-MS app and the regression library work unchanged. A whole
export (many blocks, many subjects) is just a tree of capture folders:
`hulc-ms validate --recursive` checks it, and dropping its top folder on the
HULC-MS app analyses every block.

## Layers and code

| Layer | Status | Code | Talks to the next layer through |
|---|---|---|---|
| Firmware | built | `firmware/firmware.ino` | BLE GATT ([`BLE_PROTOCOL.md`](BLE_PROTOCOL.md)) |
| Acquisition | interim: laptop | `tools/multinode_test.py` (`hulc-ms nodes`) | capture folders |
| Acquisition | **TODO: mobile app** | — | capture folders, exported to the computer |
| Analysis | built | `tools/analyze_session.py` (`hulc-ms analyze`) and the stage modules | the output files ([`DATA_FORMAT.md`](DATA_FORMAT.md) §5) |
| Review | built | `tools/skeleton_viewer.py` (review page), `tools/app.py` (HULC-MS app) | — |

### Inside the analysis

`analyze_session.py` runs the stages in order, each a separate module that
reads files and writes files, so a stage can be run, tested or replaced on its
own:

```
 capture folder ─▶ reconcile_nodes ─▶ aligned.csv ─▶ calibrate_segments ─▶ calibration.json
                   (one timeline)                     (freeze, facing, end)        │
                                                                                    ▼
 session.html ◀─ skeleton_viewer ◀─ metrics.json ◀──────────────────────────── metrics
 block_check.json ◀─ analyze_session (verdict)          (angles, ROM, reps)
```

| Module | Role |
|---|---|
| `motion_capabilities.py` | segments, joints, the montage schema; reading a capture folder (`capture_logs`, `montage_from_capture`) |
| `reconcile_nodes.py` | decode `.bin` logs, align the node clocks from the motion |
| `calibrate_segments.py` | find the sync movement, the freeze and the session end; mounting offsets; facing |
| `metrics.py` | joint angles per DOF, range of motion, reps, plausibility |
| `skeleton_viewer.py` | the review page (one self-contained HTML file) |
| `analyze_session.py` | runs the stages, the block check |
| `opensense_ik.py` | optional: the same session on an OpenSim model |
| `app.py` | the HULC-MS app: a local web page around `analyze_session.py` |
| `validate_capture.py` | checks a capture folder against the format |
| `regress.py` | the regression library of real recordings |
| `hulc_paths.py` | where the tools write; how one tool starts another (`tool_cmd`) |
| `hulc-ms.py` (repo root) | the `hulc-ms` command; the standalone build's entry point |

### Rules that keep it easy to change

- **Files between stages.** Every stage's input and output is a file in a
  documented format, never shared memory, so each is testable and replaceable.
- **One way to start a tool.** A tool that runs another goes through
  `hulc_paths.tool_cmd()`, which works from the repository and inside the
  standalone build alike.
- **The capture folder is the only input.** New acquisition (the mobile app,
  a different radio) writes capture folders; the analysis never talks to
  hardware.
- **Every change is checked twice:** `hulc-ms selftest` (synthetic data, in
  CI) and `hulc-ms regress run` (real recordings, before and after). See
  [`CONTRIBUTING.md`](../CONTRIBUTING.md).

## Integrating the mobile app

What it has to do to replace `hulc-ms nodes` (details:
[`BLE_PROTOCOL.md`](BLE_PROTOCOL.md), [`DATA_FORMAT.md`](DATA_FORMAT.md)):

1. Enroll each board once (`0x07` segment code), or read the stored segment.
2. At each charging checkpoint: offload every node (framed notifications,
   range re-requests until complete), and store each block as a capture folder:
   `<node>.bin` (the bytes as received) + `<node>.seg.json`; a `montage.json`
   with the subject id if the app knows it.
3. Erase a node only after its transfer is verified.
4. Export the stored folders to the computer as a tree; check it with
   `hulc-ms validate --recursive`, then analyse (HULC-MS app or
   `hulc-ms analyze`).

The open items are in [`ROADMAP.md`](ROADMAP.md).
