# Changing the analysis

Changes are made in the repository and reach the lab machines as a new
standalone build. The standalone `hulc-ms` app is a frozen copy of this code:
editing files inside it does nothing useful.

## One-time setup

Python 3.11–3.13, then from the repository folder:

```bash
pip install -r requirements.txt                # + requirements-opensense.txt for OpenSim
```

## Each change

1. **Branch**, then edit `tools/`. Where things live (all in
   [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md)):

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
   python hulc-ms.py selftest                                   # synthetic checks, every tool
   python hulc-ms.py regress run --data <lab drive>/regress_data # real recordings: all must pass
   ```
   If a recording prompted the change, add it as a case first
   (`hulc-ms regress add …`, or **Keep as test case…** in the app), then fix
   the code until every case passes. The regression run needs the recordings,
   so it cannot run in CI; run it before opening the pull request.

4. **Push the branch and open a pull request.** CI runs the selftests and
   builds the standalone app for Windows, macOS and Linux (downloadable from
   the run's Artifacts) so it can be tried before merging.

5. **Merge, then tag a version** (`v0.2.0`, …): the builds are attached to a
   GitHub release, and the lab machines download the new `hulc-ms` there.

## Changing what the analysis reads or writes

The capture folder ([`docs/DATA_FORMAT.md`](docs/DATA_FORMAT.md)) is the
contract with the acquisition side, today the laptop and later the mobile
app. Keep changes to it backward compatible (the analysis must still read
older folders), update the doc and `validate_capture.py` with the code, and
never renumber segment codes: they are stored on the nodes.

## Dependencies

Edit the pin in `requirements.txt` (or `requirements-opensense.txt` /
`requirements-build.txt`) and re-run the selftests and the regression library
on Python 3.11 and 3.13.
