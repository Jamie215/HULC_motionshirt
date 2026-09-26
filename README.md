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

Phase 3e build. An adaptive-power state machine that records autonomously and
serves data over a BLE GATT service:

- **State machine** — `IDLE` → `STATIC_POSTURE` → `ACTIVE_RECORDING`. It sleeps
  between IMU interrupts when no central is connected, and only records
  quaternions while moving. IDLE logs nothing.
- **Adaptive power** — DC/DC regulator, cheap IDLE-only advertising, and
  interrupt-driven sleep. Baseline ≈12 mA IDLE / ≈22 mA active recording; see
  [`firmware/POWER_OPTIMIZATION.md`](firmware/POWER_OPTIMIZATION.md).
- **QSPI flash log** — simple sequential append-only store of 20-byte records.
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
| A003  | Status             | READ    | 4 B, state / flags / log size      |
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

The pipeline is pure Python on top of **NumPy** (`pip install numpy`), which
every offline tool and `selftest` needs. Two optional extras: `bleak`, only for
the live BLE actions in `multinode_test.py` (`pip install bleak`), and OpenSim,
only for the optional OpenSense path and its comparison harness
(`pip install opensim`, see `tools/opensense_ik.py`).

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
| 3 Reconcile | `reconcile_nodes.py` | Time-aligns the per-node logs onto one timeline **from the motion itself** (cross-correlating angular speed), so alignment doesn't depend on BLE latency → `aligned.csv`, plus `aligned.quality.json` (per-sensor sync confidence and data gaps, shown in the review page) |
| 4 Capability | `motion_capabilities.py` | Given the montage, resolves which joints/metrics are valid and which are blocked (and why) |
| 5 Calibrate | `calibrate_segments.py` | Solves each node's **sensor→segment mounting offset** from a short neutral pose, with a cache-and-verify contract so re-donning is cheap → `calibration.json` |
| 6 Metrics | `metrics.py` | Per-DOF joint angles → range of motion, angular velocity, reps, plus segment and derived (L/R symmetry, coordination) tiers → `metrics.json` |
| 7 Visualize | `skeleton_viewer.py` | A self-contained HTML viewer: the segments connected into a stickman by forward kinematics, with the subject's front marked and Front / Side / Top views |
| (optional) OpenSense | `opensense_ik.py` | The same session solved with OpenSim OpenSense on a published model — the Thoracoscapular Shoulder Model (right arm, recommended) or Rajagopal 2016 — reported through the same metrics + viewer, with a per-frame fit residual. Needs `pip install opensim` and the model; see [`SOLVER_COMPARISON.md`](tools/SOLVER_COMPARISON.md) |

`analyze_session.py` orchestrates stages 3–7 in one command, binding each log
to its segment automatically from the montage.

### Quick start

```bash
# 0. one-time: install the BLE dependency (only for live node actions)
pip install bleak

# 1. enroll boards → montage.json (power ONE node on at a time)
python tools/multinode_test.py enroll --segments upper_arm_r,forearm_r

# 2. record: strap on, move with the laptop DISCONNECTED
#    (each node logs to its own flash while in ACTIVE_RECORDING)

# 3. offload each node's log while the subject is at rest (nodes must be IDLE)
python tools/multinode_test.py offload --count 2 --out-dir ./capture

# 4. run the whole analysis chain from the capture dir + montage
python tools/analyze_session.py run --montage montage.json --capture-dir ./capture
#    → aligned.csv, calibration.json, metrics.json, and an HTML viewer
```

Almost every tool has a `selftest` / `--selftest` that validates its logic on
synthetic data with **no hardware**, e.g.:

```bash
python tools/reconcile_nodes.py --selftest
python tools/analyze_session.py selftest
python tools/multinode_test.py selftest
```

### Pipeline docs

- [`COLLECTION_CHECKLIST.md`](tools/COLLECTION_CHECKLIST.md) — the end-to-end
  run-sheet for a session, including the minimal-connect BLE workflow and
  sensor-placement guidance.
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
- `montage.example.json` — a filled-in montage to copy.

---

## Repository layout

```
firmware/
  firmware.ino               Phase 3e node firmware (state machine + BLE + QSPI)
  *.md                       design/testing/power docs
  state_machine_test/        BNO086 IDLE-reset investigation harness + findings
tools/
  multinode_test.py          BLE central: enroll / offload / sync-check (needs bleak)
  reconcile_nodes.py         stage 3 — motion-based time alignment
  motion_capabilities.py     stage 4 — montage schema + capability resolver
  calibrate_segments.py      stage 5 — sensor→segment mounting solve
  metrics.py                 stage 6 — joint angles, ROM, and metric tiers
  skeleton_viewer.py            stage 7 — skeleton HTML viewer
  analyze_session.py         stages 3–7 orchestrated in one command
  *.md, *.html, *.json       pipeline docs + example montage
```

## Data & privacy

Real recordings and per-session generated files (captures, `aligned.csv`,
`montage.json`, `calibration.json`, generated viewers) are **git-ignored** by
design — see `.gitignore`. Only source, docs, and the example montage are
tracked; never commit real recording data.
