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
- **`tools/`** — the Python analysis pipeline that runs on a laptop after
  collection, from raw logs to range-of-motion numbers and a visual.

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
  offset. (The production alignment path does not depend on this being precise —
  see below.)

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

## Analysis pipeline (`tools/`)

The pipeline is pure Python on top of **NumPy**. Two ways to run it:

- **The standalone app — no Python needed.** Download the build for your
  system (Windows / macOS / Linux) from the repository's GitHub Actions
  **app** workflow (or a release) and double-click **HULC**. It holds Python,
  numpy and every tool; from a terminal, `HULC analyze …` / `HULC nodes …`
  run the tools (see `packaging/README.txt`). Built by
  `packaging/build_app.py`; body-model solves (OpenSim) are not included.
- **With Python 3.11–3.13**, from the repository folder, install the pinned
  (frozen) versions the tools are tested with:

  ```bash
  pip install -r requirements.txt              # numpy + bleak (node commands)
  pip install -r requirements-opensense.txt    # optional: + OpenSim body-model solves
  ```

  Then run the tools as `python tools/<tool>.py …`, as the docs do.

Each node records the ORIENTATION of the SEGMENT it is strapped to. A clinical
**joint angle** is the *relative* orientation of two adjacent segments, so what
can be computed depends entirely on where the nodes are placed — the
**montage**. The pipeline makes that dependency explicit and never fabricates a
number a placement can't support.

### Stages

```
   capture ──▶ offload ──▶ reconcile ──▶ montage/resolve ──▶ calibrate ──▶ metrics ──▶ visualize
  (firmware)   (BLE)      align to one    which joints are    sensor→bone    ROM etc.    HTML viewer
                          timeline        computable          mounting solve
```

| Stage | Tool | What it does |
|-------|------|--------------|
| 1–2 Capture & offload | `firmware.ino`, `multinode_test.py` | Nodes log autonomously; the central offloads each node's `.bin` |
| 3 Reconcile | `reconcile_nodes.py` | Time-aligns the per-node logs onto one timeline **from the motion itself** (cross-correlating world-frame angular-velocity vectors; angular speed as fallback), so alignment doesn't depend on BLE latency → `aligned.csv`, plus `aligned.quality.json` (per-sensor sync confidence and peak distinctness, data gaps, clock restarts — shown in the review page) |
| 4 Capability | `motion_capabilities.py` | Given the montage, resolves which joints/metrics are valid and which are blocked (and why) |
| 5 Calibrate | `calibrate_segments.py` | Solves each node's **sensor→segment mounting offset** from the freeze right after the sync movement (found automatically), estimates the subject's facing (torso node, elbow hinge, or `--facing-deg`), and finds where the nodes came off → `calibration.json` |
| 6 Metrics | `metrics.py` | Per-DOF joint angles over the analysis window (freeze → nodes coming off) → range of motion, angular velocity, rep bouts, plausibility flags, plus segment and derived (L/R symmetry, coordination) tiers → `metrics.json` |
| 7 Visualize | `skeleton_viewer.py` | A self-contained HTML viewer: the segments connected into a stickman by forward kinematics, with the subject's front marked and Front / Side / Top views. With OpenSense solves, a footer switch between the direct sensor view and each model, and a solver-comparison overlay on the angle graph. A **Timeline** strip shows the whole log with the detected sync, freeze and end; drag them to correct a detection and copy the re-run command |
| (optional) OpenSense | `opensense_ik.py` | The same session solved with OpenSim OpenSense on a published model — the Thoracoscapular Shoulder Model (right arm, recommended) or Rajagopal 2016 — reported through the same metrics + viewer, with a per-frame fit residual; frames where the model loses the sensors are flagged and excluded. Needs `pip install opensim` and the model; see [`SOLVER_COMPARISON.md`](tools/SOLVER_COMPARISON.md) |

`analyze_session.py` orchestrates stages 3–7 in one command, binding each log
to its segment automatically from the montage. Each node stores its own
segment, so offload writes the block's `montage.json` into its capture folder
and the folder is all the analysis needs (`--montage` only overrides).

### Quick start

```bash
# 0. one-time, from the repo folder: the pinned dependencies
pip install -r requirements.txt
#    optional, for body-model solves (the review page's Sensors / model switch):
pip install -r requirements-opensense.txt
python tools/opensense_ik.py fetch-models     # -> models/opensense/

# 1. enroll boards (power ONE node on at a time): stores each board's segment
#    on the board, and writes montage.json + nodes_registry.json
python tools/multinode_test.py enroll --segments upper_arm_r,forearm_r

# 2. record one block, laptop DISCONNECTED — the one-page card,
#    tools/COLLECTION_SOP.md: sync movement → freeze → task → nodes off
#    (each node logs to its own flash while it is moving)

# 3. charging checkpoint: nodes off the body onto the charger (they go IDLE and
#    stop logging), then offload this block into its own folder
#    (it also writes ./capture/block1/montage.json from the nodes' segments)
python tools/multinode_test.py offload --count 2 --out-dir ./capture/block1 --subject S01

# 4. analyze the block: double-click "HULC Sessions" (.bat on Windows,
#    .command on macOS / Linux) and drop ./capture/block1 on the page — or
#    from the command line (montage from the folder; --montage FILE overrides):
python tools/analyze_session.py run --capture-dir ./capture/block1 \
    --outdir ./out/block1
#    → aligned.csv, calibration.json, metrics.json, an HTML viewer, and a
#      BLOCK CHECK (OK / REDO + what to change) at the end
#    (with OpenSim + fetched models it also solves on each model by itself;
#    --no-opensense skips, --opensense-model PATH picks other models)
```

Almost every tool has a `selftest` / `--selftest` that validates its logic on
synthetic data with **no hardware**, e.g.:

```bash
python tools/reconcile_nodes.py --selftest
python tools/analyze_session.py selftest
python tools/multinode_test.py selftest
```

GitHub Actions runs every selftest on each push and pull request
(`.github/workflows/selftest.yml`).

### The session app

`tools/app.py` puts the analysis behind a browser page, for anyone who would
rather not use a terminal. Double-click **`HULC Sessions.bat`** (Windows) or
**`HULC Sessions.command`** (macOS; on Linux `bash "HULC Sessions.command"`),
or run `python tools/app.py`. A tab opens at `http://127.0.0.1:8765/`:

- **Drop** a block's capture folder (or its `.bin` + `.seg.json` files) on
  the page; several block folders at once become one session each. Type the
  subject id first; it goes into the block's montage.
- Each block runs through `analyze_session.py` in the background, exactly as
  on the command line (OpenSim solves included when installed), and the app
  shows its **block check** and **review page**.
- On the review page's **Timeline**, drag the freeze or the end and press
  **Apply**: the block is re-analyzed in place with the hand-set values.
- **Keep as test case…** adds the block to the regression library
  (`regress.py add`); check the expectations and commit the case file.

The launchers offer to install the requirements (`pip install -r
requirements.txt`) the first time, if numpy is missing. The standalone app
(above) is the same session app with Python built in. It needs only Python 3 +
numpy, listens on this computer only (127.0.0.1),
and keeps copies of the dropped files and every analysis in a workspace folder
(`~/HULC_sessions`; `--workspace` or `$HULC_WORKSPACE` to change it). The
original files are never modified; **Remove** deletes a session's copies.

### Changing the analysis safely — the regression library

Synthetic selftests check the math; they cannot tell whether a change still
handles **real** recordings. `tools/regress.py` runs the whole pipeline on a
library of real captures and checks each against what a person confirmed once:
where the freeze is, where the session ends, the block-check verdict, the
facing, and plausible ranges.

```bash
# before AND after any change to the analysis — every case must still pass:
python tools/regress.py run --data /path/to/regress_data

# grow the library: add a recording (expectations pre-filled from today's
# analysis), then CHECK them against what really happened, edit, and commit
# tools/regress_cases/<name>.json
python tools/regress.py add 2026-10-07_torso_upper-arm \
    --capture-dir ./capture/block1 --description "..."
```

The case files (montage, description, expectations) are in git
(`tools/regress_cases/`); the recordings are **not** — keep them on a lab drive
(`<data>/<case name>/*.bin`) and point `--data` or `$HULC_REGRESS_DATA` at it.
Cases without recordings are skipped. When a recording breaks the analysis, add
it as a case first, then fix the code until every case passes: each fix then
stays fixed. The more varied the library (people, mountings, rooms, montages),
the safer a threshold change.

### Pipeline docs

- [`COLLECTION_SOP.md`](tools/COLLECTION_SOP.md) — the one-page recording
  card: sync movement, freeze, task, nodes off, and the charger routine.
- [`COLLECTION_REFERENCE.md`](tools/COLLECTION_REFERENCE.md) — laptop and
  node setup (enroll, link check), where to strap each node, why each step,
  the setting behind every number, the charging checkpoint in detail, the
  block check line by line, and troubleshooting.
- [`MONTAGE_SCHEMA.md`](tools/MONTAGE_SCHEMA.md) — the montage format and the
  capability resolver contract.
- [`SETUP_AND_CALIBRATION_PLAN.md`](tools/SETUP_AND_CALIBRATION_PLAN.md) — the
  design of stages 5–7 (calibration, metrics, visual).
- [`pipeline_walkthrough.html`](tools/pipeline_walkthrough.html) — the whole
  pipeline at a glance.
- `timing_bench.py` — synthetic sessions pushed through the real reconcile
  step (own clocks, firmware sampling schedule, STATIC gaps, strap wobble) to
  measure what sync and sampling cost; it motivated the vector clock sync and
  the 1 Hz STATIC logging.
- [`SOLVER_COMPARISON.md`](tools/SOLVER_COMPARISON.md) — default chain vs
  OpenSense (two models) on synthetic ground truth and a real capture, with a
  draft methods paragraph; [`OPENSENSE_FEASIBILITY.md`](tools/OPENSENSE_FEASIBILITY.md)
  — what OpenSense can and cannot recover for each montage.
- [`METRICS.md`](tools/METRICS.md) — every metric, its definition and the
  plausibility limits.
- [`montage.example.json`](tools/montage.example.json) — a filled-in montage to copy.

---

## Repository layout

```
requirements*.txt                the pinned dependencies (analysis + nodes, OpenSim, build)
packaging/                       the standalone app: build_app.py, entry point, README.txt
HULC Sessions.bat / .command     double-click launchers for tools/app.py
.github/workflows/selftest.yml   CI: every selftest + the regression library
firmware/
  firmware.ino               node firmware (state machine + BLE + QSPI log)
  *.md                       design / testing / power docs
  state_machine_test/        BNO086 IDLE-reset investigation harness + findings
tools/
  multinode_test.py          BLE central: enroll / offload / erase / sync check (needs bleak)
  reconcile_nodes.py         stage 3 — motion-based time alignment
  motion_capabilities.py     stage 4 — montage schema + capability resolver
  calibrate_segments.py      stage 5 — freeze, facing, session end, mounting solve
  metrics.py                 stage 6 — joint angles, ROM, and metric tiers
  skeleton_viewer.py         stage 7 — review page (skeleton, Timeline, metrics)
  analyze_session.py         stages 3–7 in one command, plus the block check
  app.py                     the session app: drop a block, get its review page
  hulc_paths.py              where the tools write (repo checkout vs standalone app)
  opensense_ik.py            optional OpenSense solve (TSM / Rajagopal)
  regress.py                 regression library runner; cases in regress_cases/
  timing_bench.py            evaluation: what clock sync and sampling cost
  compare_paths.py           evaluation: default chain vs OpenSense (needs opensim)
  opensense_feasibility.py   evaluation: OpenSense on partial montages (needs opensim)
  montage.example.json       a filled-in montage to copy
  *.md, *.html               pipeline docs
```

## Data & privacy

Real recordings and per-session generated files (capture folders, `.bin`
logs, `aligned.csv`, every per-session `.json`, generated review pages) are
**git-ignored** by design — see `.gitignore`. Only source, docs, the example
montage and the regression case files (expectations, no recordings) are
tracked; never commit real recording data. The session app keeps its copies
outside the repository (`~/HULC_sessions` by default).
