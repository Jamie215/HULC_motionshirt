# HULC Motion Shirt

Wearable motion-capture for upper-extremity rehab. A set of small IMU nodes
worn on the body (one per segment) each log their orientation to on-board flash
while the subject moves, then hand the data to a laptop over Bluetooth. An
offline Python pipeline time-aligns the nodes, calibrates them to the body, and
turns the raw quaternions into clinical **joint angles and range of motion** —
plus a self-contained HTML viewer of the movement.

The repository has two halves:

- **`firmware/`** — the node firmware (Arduino / nRF52840) that records motion
  and offloads it over BLE.
- **The analysis** — offloading the nodes and turning their logs into a block
  check and a review page, done **either in the HULC-MS app or with `hulc-ms`
  commands** (see [Analysing sessions](#analysing-sessions)). The code is in
  `tools/`, the documentation in `docs/`.

---

## Analysing sessions

A session is recorded in **blocks** (one mounting, one charge — the
[recording card](docs/COLLECTION_SOP.md)). Each block is offloaded into its own
capture folder, and analysing that folder gives a **block check** (OK / REDO
and what to change) and a **review page** (3-D movement, joint angles, range of
motion). There are two ways to do it; both run the same analysis.

| | **1. The HULC-MS app** | **2. `hulc-ms` commands** |
|---|---|---|
| For | analysing and reviewing blocks without a terminal | offloading the nodes, scripting, batches |
| How | drop a block's folder on a page in the browser | `hulc-ms analyze --capture-dir …` in a terminal |
| Get it | the standalone app (no Python), or the repository + Python | the same two installs |

### Getting hulc-ms

**Standalone app — no Python needed.** One program holding Python and every
tool, built for each system by CI. Signed in to GitHub: **Actions → app → the
latest run → Artifacts** (or the release page, once a version is tagged).

- **Windows:** download **hulc-ms-windows**, unzip it into a normal folder (do not
  run it from inside the zip). First launch: "Windows protected your PC" →
  **More info → Run anyway** (the app is not code-signed).
- **macOS:** download **hulc-ms-macos**, unzip it, then unzip `hulc-ms-macos.zip`
  inside it (the inner zip keeps the app's executable permission); open
  `hulc-ms.app` with right-click → **Open** the first time.
- **Linux:** download **hulc-ms-linux**, unzip it, `tar xzf hulc-ms-linux.tar.gz`.

Body-model solves (OpenSim) are not in the standalone app.

**From the repository, with Python 3.11–3.13.** Install the pinned versions the
tools are tested with, from the repository folder:

```bash
pip install -r requirements.txt              # numpy + bleak (the node commands)
pip install -r requirements-opensense.txt    # optional: + OpenSim body-model solves
```

(`hulc-ms.bat` / `hulc-ms.command` offer to do this the first time they run.)

### 1. The HULC-MS app

Start it: double-click **hulc-ms** (`hulc-ms.exe` / `hulc-ms.app` from the standalone
download, or `hulc-ms.bat` / `hulc-ms.command` in the repository folder). A browser
tab opens at `http://127.0.0.1:8765/`.

1. Type the subject id, then **drop a block's capture folder** on the page (or
   its `.bin` + `.seg.json` files; several block folders become one session
   each).
2. Read the **block check** at the top, then the **review page** below it.
3. Freeze or end in the wrong place? Open **Timeline**, drag it, press
   **Apply**: the block is re-analysed in place.
4. **Keep as test case…** adds the block to the regression library;
   **Quit** stops the app.

The app runs only on this computer (127.0.0.1) and keeps copies of the dropped
files and every analysis in `~/HULC_sessions` (`--workspace` to change); the
original files are never modified, and **Remove** deletes a session's copies.

### 2. `hulc-ms` commands

The same program, with a command after it. What to type for `hulc-ms`:

| You have | Type `hulc-ms` as |
|---|---|
| Standalone app, Windows | `hulc-ms` in a Command Prompt opened in the hulc-ms folder (it runs `hulc-ms.exe`) |
| Standalone app, macOS / Linux | `hulc-ms.app/Contents/MacOS/hulc-ms` / `hulc-ms/hulc-ms` |
| Repository, Windows | `hulc-ms` in a Command Prompt in the repository folder (it runs `hulc-ms.bat`) |
| Repository, macOS / Linux | `python3 hulc-ms.py` (or `./hulc-ms.command`) |

(In PowerShell, type `.\hulc-ms`.)

| Command | What it does |
|---|---|
| `hulc-ms` (no command) | the HULC-MS app |
| `hulc-ms nodes enroll --segments torso,upper_arm_r` | once per rig: power one node at a time; each node stores its body part |
| `hulc-ms nodes offload --count 2 --out-dir ./capture/block1 --subject S01` | at the charger: each node's log + the block's `montage.json` into the folder |
| `hulc-ms analyze --capture-dir ./capture/block1 --outdir ./out/block1` | the analysis: block check printed at the end, review page `out/block1/session.html` |
| `hulc-ms nodes erase --count 2` | wipe the nodes for the next block |
| `hulc-ms regress run --data D:/regress_data` | the regression library (before and after any analysis change) |
| `hulc-ms opensense fetch-models` | once, for body-model solves (Python install with OpenSim) |
| `hulc-ms selftest` | every tool's selftest: checks an install or a build |

`hulc-ms <command> --help` lists the options. The ones you may need for
`analyze`: `--window t0,t1` / `--end t_ms` (set the freeze / end by hand — the
review page's Timeline writes these for you), `--protocol hold-first`
(recordings made before 2026-10), `--montage FILE` (override the folder's
montage), `--facing-deg` (no torso node and too little elbow bending),
`--no-opensense`.

Recording, the charger routine and every threshold: the
[recording card](docs/COLLECTION_SOP.md) and the
[collection reference](docs/COLLECTION_REFERENCE.md).

---

## How the analysis works

Each node records the ORIENTATION of the SEGMENT it is strapped to. A clinical
**joint angle** is the *relative* orientation of two adjacent segments, so what
can be computed depends entirely on where the nodes are placed — the
**montage**. The pipeline makes that dependency explicit and never fabricates a
number a placement can't support. Each node stores its own segment, so a block's
capture folder carries its montage.

```
   capture ──▶ offload ──▶ reconcile ──▶ montage/resolve ──▶ calibrate ──▶ metrics ──▶ visualize
  (firmware)   (BLE)      align to one    which joints are    sensor→bone    ROM etc.    HTML viewer
                          timeline        computable          mounting solve
```

`hulc-ms analyze` (`tools/analyze_session.py`) runs stages 3–7 in order; the app
runs it for each dropped block. The stages are separate modules in `tools/`,
each runnable on its own for development (docs/COLLECTION_REFERENCE.md §5):

| Stage | Module | What it does |
|-------|------|--------------|
| 1–2 Capture & offload | `firmware.ino`, `multinode_test.py` (`hulc-ms nodes`) | Nodes log autonomously; the laptop offloads each node's `.bin` |
| 3 Reconcile | `reconcile_nodes.py` | Time-aligns the per-node logs onto one timeline **from the motion itself** (cross-correlating world-frame angular-velocity vectors; angular speed as fallback) → `aligned.csv`, plus `aligned.quality.json` (sync confidence, data gaps, clock restarts) |
| 4 Capability | `motion_capabilities.py` | Given the montage, resolves which joints/metrics are valid and which are blocked (and why) |
| 5 Calibrate | `calibrate_segments.py` | Finds the freeze after the sync movement and solves each node's **sensor→segment mounting offset**, estimates the subject's facing (torso node, elbow hinge, or `--facing-deg`), and finds where the nodes came off → `calibration.json` |
| 6 Metrics | `metrics.py` | Per-DOF joint angles over the analysis window → range of motion, angular velocity, rep bouts, plausibility flags, plus segment and derived tiers → `metrics.json` |
| 7 Visualize | `skeleton_viewer.py` | The review page: the segments as a stickman, Front / Side / Top views, the metrics panel, the Timeline, and a switch to each OpenSense solve |
| (optional) OpenSense | `opensense_ik.py` | The same session solved on a published OpenSim model (Thoracoscapular Shoulder Model, right arm; or Rajagopal 2016), through the same metrics and page; fit-lost frames excluded. See [`SOLVER_COMPARISON.md`](docs/SOLVER_COMPARISON.md) |
| Block check | `analyze_session.py` | OK / REDO per check (sync, freeze, facing, end, trunk, joint limits, …) → `block_check.json` |
| The app | `app.py` | The local web app around `analyze_session.py` |

### Changing the analysis safely — the regression library

Synthetic selftests (`hulc-ms selftest`; CI runs them on every push) check the
math; they cannot tell whether a change still handles **real** recordings.
`hulc-ms regress` runs the whole pipeline on a library of real captures and checks
each against what a person confirmed once: where the freeze is, where the
session ends, the block-check verdict, the facing, and plausible ranges.

```bash
# before AND after any change to the analysis — every case must still pass:
hulc-ms regress run --data /path/to/regress_data

# grow the library (or "Keep as test case…" in the app): expectations are
# pre-filled from today's analysis; CHECK them against what really happened,
# edit, and commit tools/regress_cases/<name>.json
hulc-ms regress add 2026-10-07_torso_upper-arm --capture-dir ./capture/block1 --description "..."
```

The case files are in git (`tools/regress_cases/`); the recordings are
**not** — keep them on a lab drive (`<data>/<case name>/*.bin`) and point
`--data` or `$HULC_REGRESS_DATA` at it. Cases without recordings are skipped.
When a recording breaks the analysis, add it as a case first, then fix the code
until every case passes.

---

## Hardware

Each node is a:

- **Seeed XIAO nRF52840** (BLE MCU)
- **SparkFun BNO086** 9-DoF IMU over I²C — its fused, magnetometer-referenced
  Rotation Vector gives every node a quaternion in a **shared world frame**
  (gravity + magnetic north)
- **External 2 MB QSPI flash** (P25Q16H) for an append-only log — driven
  directly via `nrfx_qspi`, so the bootloader on internal flash is never at risk

A "motion shirt" is several such nodes, one per body segment (e.g. torso,
upper arm, forearm). Each advertises a unique BLE name `HULC-IMU-XXXX` (last 4
hex of its MAC), so boards never collide and need no per-board reflash.

## Firmware (`firmware/firmware.ino`)

An adaptive-power state machine that records autonomously and
serves data over a BLE GATT service:

- **State machine** — `IDLE` → `STATIC_POSTURE` → `ACTIVE_RECORDING`. It sleeps
  between IMU interrupts when no central is connected, and logs quaternions at
  ~10 Hz while moving and ~1 Hz while still (STATIC). IDLE logs nothing.
- **Adaptive power** — DC/DC regulator, cheap IDLE-only advertising, and
  interrupt-driven sleep. Baseline ≈12 mA IDLE / ≈22 mA active recording; see
  [`firmware/POWER_OPTIMIZATION.md`](firmware/POWER_OPTIMIZATION.md).
- **QSPI flash log** — simple sequential append-only store of 20-byte records.
- **Segment in the header** — each node stores which body part it is on (set
  at enroll, command `0x07`), so every offloaded log says what it measured.
- **BLE offload** — a central connects while the node is `IDLE`, time-syncs it,
  and streams the flash log off in 200-byte chunks, with byte-range re-send for
  gap recovery.
- **Multi-node time sync** — millisecond-resolution sync command plus a
  read-only Time Info characteristic, so a central can measure cross-node clock
  offset. (The analysis does not depend on this being precise — it aligns the
  clocks from the motion; see How the analysis works.)

### BLE GATT layout

| UUID  | Characteristic     | Access  | Notes                              |
|-------|--------------------|---------|------------------------------------|
| A001  | Quaternion stream  | NOTIFY  | 20 B, debug only                   |
| A002  | Control            | WRITE   | 9 B, command byte + payload        |
| A003  | Status             | READ    | 6 B, state / flags / log size / segment |
| A004  | Log offload        | NOTIFY  | 200 B chunks                       |
| A005  | Time Info          | READ    | 16 B, for cross-node clock skew    |

The full command set, characteristic byte layouts, and offload protocol are
documented in the header of `firmware/firmware.ino`.

### Firmware docs

- [`IDLE_WAKE_SOURCE.md`](firmware/IDLE_WAKE_SOURCE.md) — the three selectable
  wake-from-idle sensors (Stability Detector / Classifier / Significant Motion)
  and how to A/B them.
- [`MULTINODE_SYNC_DESIGN.md`](firmware/MULTINODE_SYNC_DESIGN.md) — architecture
  and decisions for keeping nodes aligned and getting their data off.
- [`MULTINODE_SYNC_MATH.md`](firmware/MULTINODE_SYNC_MATH.md) — the clock model,
  cross-correlation, drift, and confidence math behind reconciliation.
- [`MULTINODE_TESTING.md`](firmware/MULTINODE_TESTING.md) — what changed for
  multi-node and how to test the synced connection.
- [`POWER_OPTIMIZATION.md`](firmware/POWER_OPTIMIZATION.md) — power work that has
  landed and the backlog.
- [`state_machine_test/`](firmware/state_machine_test/) — a stripped-down
  harness plus [`FINDINGS.md`](firmware/state_machine_test/FINDINGS.md) on the
  BNO086 ~6.5 s IDLE reset investigation.

### Building & flashing

Arduino IDE (or `arduino-cli`) with the **Seeed nRF52 mbed-enabled** board
package. Libraries: **ArduinoBLE** and the **SparkFun BNO08x Arduino Library**
(`nrfx_qspi` ships with the core — nothing to install). Select the XIAO
nRF52840 board and flash `firmware/firmware.ino`.

---

## Documentation (`docs/`)

- [`COLLECTION_SOP.md`](docs/COLLECTION_SOP.md) — the one-page recording card
  and the charger routine.
- [`COLLECTION_REFERENCE.md`](docs/COLLECTION_REFERENCE.md) — setup and
  enroll, where to strap each node, why each step and the setting behind every
  number, the checkpoint in detail, the block check line by line,
  troubleshooting.
- [`METRICS.md`](docs/METRICS.md) — every metric, its definition and the
  plausibility limits.
- [`MONTAGE_SCHEMA.md`](docs/MONTAGE_SCHEMA.md) — the montage format and the
  capability resolver.
- [`SOLVER_COMPARISON.md`](docs/SOLVER_COMPARISON.md) and
  [`OPENSENSE_FEASIBILITY.md`](docs/OPENSENSE_FEASIBILITY.md) — the direct
  sensor path vs OpenSense, and what OpenSense recovers per montage.
- [`SETUP_AND_CALIBRATION_PLAN.md`](docs/SETUP_AND_CALIBRATION_PLAN.md) — the
  design record of stages 5–7.
- [`pipeline_walkthrough.html`](docs/pipeline_walkthrough.html) — the whole
  pipeline at a glance.

---

## Repository layout

```
hulc-ms.bat / hulc-ms.command      start the HULC-MS app from the repository; `hulc …` commands
hulc-ms.py                      the hulc command (and the standalone app's entry point)
requirements*.txt            pinned dependencies: analysis + nodes, OpenSim, build
docs/                        recording card, reference, metrics, design docs
firmware/
  firmware.ino               node firmware (state machine + BLE + QSPI log)
  *.md                       design / testing / power docs
  state_machine_test/        BNO086 IDLE-reset investigation harness + findings
tools/                       the analysis code
  app.py                     the HULC-MS app (local web page)
  analyze_session.py         hulc-ms analyze: stages 3-7 + the block check
  multinode_test.py          hulc-ms nodes: enroll / offload / erase / check (BLE)
  reconcile_nodes.py         stage 3 - motion-based time alignment
  motion_capabilities.py     stage 4 - montage schema + capability resolver
  calibrate_segments.py      stage 5 - freeze, facing, session end, mounting solve
  metrics.py                 stage 6 - joint angles, ROM, metric tiers
  skeleton_viewer.py         stage 7 - the review page
  opensense_ik.py            optional OpenSense solve (TSM / Rajagopal)
  regress.py                 hulc-ms regress; cases in regress_cases/
  hulc_paths.py              where the tools write (repository vs standalone app)
  montage.example.json       a filled-in montage to copy
  evaluation/                synthetic-truth studies behind the design (timing_bench,
                             compare_paths, opensense_feasibility)
packaging/                   building the standalone app (build_app.py, README.txt)
.github/workflows/           CI: selftest.yml (selftests + library), app.yml (standalone builds)
```

## Data & privacy

Real recordings and per-session generated files (capture folders, `.bin`
logs, `aligned.csv`, every per-session `.json`, generated review pages) are
**git-ignored** by design — see `.gitignore`. Only source, docs, the example
montage and the regression case files (expectations, no recordings) are
tracked; never commit real recording data. The HULC-MS app keeps its copies
outside the repository (`~/HULC_sessions` by default).
