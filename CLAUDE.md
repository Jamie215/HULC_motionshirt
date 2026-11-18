# CLAUDE.md

Wearable motion capture for upper-extremity rehab. IMU nodes worn on body
segments log orientation quaternions to flash. Afterwards they are offloaded
over BLE into one **capture folder** per block. An offline Python pipeline aligns
the nodes, calibrates them to the body and produces joint angles, range of
motion, a block check (OK / UNUSABLE) and an HTML review page. See README.md for
the full picture.

## Layout

- `firmware/firmware.ino`: node firmware (state machine, BLE, QSPI flash log)
- `hulc-ms.py`: the `hulc-ms` command and the standalone app's entry point
- `tools/`: the analysis, one module per stage:
  - `reconcile_nodes.py` (align)
  - `motion_capabilities.py` (montage)
  - `calibrate_segments.py`
  - `metrics.py`
  - `skeleton_viewer.py` (review page)
  - `analyze_session.py` (runs them all)
- `tools/multinode_test.py`: `hulc-ms nodes` (BLE enroll / offload / erase / check)
- `tools/app.py`: the HULC-MS local web app
- `tools/validate_capture.py`: checks a capture folder against the data format
- `tools/regress.py` + `tools/regress_cases/`: regression library (expectations only)
- `packaging/build_app.py`: standalone app build (used by `.github/workflows/app.yml`)

## Commands

Use **Python 3.12** (same as CI).

```bash
pip install -r requirements.txt
python hulc-ms.py selftest          # every tool's selftest, no hardware; must pass
python hulc-ms.py regress run --data <folder>
```

The regression recordings are not in the repo. Without `--data` (or
`$HULC_REGRESS_DATA`), `regress run` skips every case and still exits 0. Treat
"0 passed, N skipped" as **not run**, not as a pass.

## Rules

- **Capture-folder format** is the contract between firmware and tools:
  - `.bin` log of 20-byte little-endian records (`uint32 t_ms` + 4 × `float32` quaternion)
  - `.seg.json` header
  - optional `montage.json`

  The record layout appears in `firmware.ino`, `RECORD_SIZE` in
  `tools/reconcile_nodes.py` and `tools/multinode_test.py`, and
  `validate_capture.py`. Change them all together.
- **Analysis changes:** run `regress` with real data before and after any change
  under `tools/` that can move a number, and report the difference.
- **Privacy:** never commit recordings or per-session outputs (`.bin`,
  `aligned.csv`, session `.json`, generated `.html`). They are git-ignored by
  design; don't override that. Subjects are IDs only (`S01`), never names.
- Output locations go through `tools/hulc_paths.py`, because the repo and the
  standalone app write to different places. Don't hard-code paths.
- **Releases:** `app.yml` builds and publishes on a `v*` tag. Don't tag or push
  tags unless asked.
- **Docs** are Markdown only. Don't add `.docx` or other generated copies.
