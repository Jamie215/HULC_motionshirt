# Roadmap — what is done, what is next

The target: nodes → **mobile app** (offload and store every block during the
collection period) → **computer** (import the export, analyse, review). The
analysis and review half is built; the mobile app is the main open piece. How
the parts fit: [`ARCHITECTURE.md`](ARCHITECTURE.md).

## Done

- Firmware: autonomous logging (~10 Hz moving, ~1 Hz still), segment stored on
  the node, framed and recoverable offload.
- Analysis: clock alignment from motion, automatic freeze / session-end
  detection, calibration, joint angles and ROM, block check, optional OpenSim
  solves.
- Review: the review page with Timeline corrections; the HULC-MS app.
- Delivery: the standalone app for Windows, macOS and Linux (CI-built), pinned
  requirements for the Python install, one `hulc-ms` command.
- Safety net: selftests in CI, a regression library of real recordings.
- The hand-over format between acquisition and analysis: the capture folder
  ([`DATA_FORMAT.md`](DATA_FORMAT.md)), with `hulc-ms validate` to check an
  export, and the BLE protocol written out for a phone
  ([`BLE_PROTOCOL.md`](BLE_PROTOCOL.md)).

## TODO — the mobile app (replaces `hulc-ms nodes`)

| # | Item | Notes |
|---|---|---|
| M1 | Choose the platform (iOS / Android / cross-platform) and its BLE library | needs BLE notifications, reads and writes; iOS and Android centrals are both fast enough for offload |
| M2 | Enroll: assign each board a segment (`0x07`) and show what each node stores | one node powered at a time, as the laptop does today |
| M3 | Offload with framing and range re-requests until complete | port `measure_offload()` / `_missing_ranges()`; IDLE only |
| M4 | Store each block as a capture folder (`.bin` + `.seg.json`, `montage.json` with subject / session) | [`DATA_FORMAT.md`](DATA_FORMAT.md); keep zero-filled holes at their offsets |
| M5 | Erase a node only after its transfer is verified | the node keeps its log until `0x03` |
| M6 | Subject / session entry, block list, charge reminders | blocks are planned by battery (~90 min of recording) |
| M7 | Export to the computer at the end of the collection period | a folder tree of capture folders; check with `hulc-ms validate --recursive` |
| M8 | Data protection on the phone (encryption at rest, who can export) | recordings are research data |
| M9 | Test against the regression library | export the library's recordings through the app path and confirm `hulc-ms regress run` still passes |

## TODO — computer side, for the mobile workflow

| # | Item | Notes |
|---|---|---|
| C1 | Import a whole export in one step | works today: drop the top folder on the HULC-MS app (one session per block); a dedicated "import export" view would help at scale |
| C2 | A summary across blocks and subjects (one table: verdict, ROM per joint) | today each block has its own `metrics.json` and review page |
| C3 | Offload / erase buttons in the HULC-MS app | interim convenience until the mobile app exists; today `hulc-ms nodes` in a terminal |

## TODO — validation and open decisions

| # | Item |
|---|---|
| V1 | Motion-capture validation: per-DOF RMSE and Bland–Altman against a lab system |
| V2 | Grow the regression library to 20–30 recordings across people, montages and rooms |
| V3 | Bench-measure battery life on the current firmware |
| V4 | Decide the plane-of-elevation cutoff (10° now, or 20–30°) |
| V5 | Choose the left-arm body model (mirrored shoulder model or Rajagopal) |
| V6 | Check offload writing `montage.json` on hardware, and the standalone app on a Mac |
