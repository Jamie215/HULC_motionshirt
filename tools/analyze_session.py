"""
HULC Motion Shirt — one-shot post-offload pipeline.

Turns a directory of offloaded node logs + a montage into a viewer + metrics in
ONE command, running the post-offload stages in order and binding logs to
segments automatically (no hand-ordering of .bin files):

    reconcile  -> aligned.csv        (logs passed in montage column order)
    capability -> which joints are computable for this montage
    calibrate  -> calibration.json   (neutral window auto-detected)
    metrics    -> metrics.json       (per-DOF joint angles + range of motion)
    render     -> <out>.html         (stage-7 review: viewer + metrics panel)

The montage records each node's column (n0, n1, ...) and its id. Offload names
each file by node id (e.g. HULC-IMU-485C.bin), so this tool resolves every
node's log from --capture-dir and feeds reconcile the files in the RIGHT order.

Usage
-----
    # validate the orchestration on synthetic logs (no hardware):
    python tools/analyze_session.py selftest

    # run the whole chain from a capture dir + montage:
    python tools/analyze_session.py run --montage montage.json --capture-dir ./capture

    # no torso node? state the subject's facing so joint axes are anatomical:
    python tools/analyze_session.py run --montage montage.json --capture-dir ./capture \
        --facing-deg 90

    # override the neutral window / output name:
    python tools/analyze_session.py run --montage montage.json --capture-dir ./capture \
        --window 1200,4000 --out elbow.html
"""
import argparse
import json
import os
import re
import struct
import subprocess
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from motion_capabilities import validate_montage  # noqa: E402

TOOLS = os.path.dirname(os.path.abspath(__file__))


def _safe_name(node_id):
    """Match the offload file-naming in multinode_test.measure_offload."""
    return re.sub(r"[^A-Za-z0-9._-]", "_", node_id)


def load_montage(path):
    with open(path, encoding="utf-8-sig") as f:      # tolerate a UTF-8 BOM
        montage = json.load(f)
    errors = validate_montage(montage)
    if errors:
        raise SystemExit("[analyze] montage failed validation:\n  - " +
                         "\n  - ".join(errors))
    return montage


def ordered_logs(montage, capture_dir):
    """Return the .bin paths in montage COLUMN order (n0, n1, ...).

    Raises with a clear message if any node's log is missing.
    """
    nodes = sorted(montage["nodes"], key=lambda n: n["column"])
    paths, missing = [], []
    for n in nodes:
        p = os.path.join(capture_dir, f"{_safe_name(n['node_id'])}.bin")
        paths.append(p)
        if not os.path.exists(p):
            missing.append((n["node_id"], p))
    if missing:
        lines = "\n".join(f"    {nid}: expected {p}" for nid, p in missing)
        raise SystemExit(
            f"[analyze] {len(missing)} node log(s) not found in {capture_dir}:\n"
            f"{lines}\n"
            f"    (offload writes <node_id>.bin; check --capture-dir and that "
            f"both nodes offloaded COMPLETE.)")
    return paths, nodes


def warn_segment_disagreements(nodes, capture_dir):
    """Warn when a node's OWN segment header disagrees with the montage.

    Offload writes a `<node_id>.seg.json` sidecar carrying the segment the node
    reported for itself. The montage stays AUTHORITATIVE (source-of-truth rule,
    SETUP_AND_CALIBRATION_PLAN.md §2.1) — this only warns, so a swapped or
    mislabeled node surfaces instead of silently binding to the wrong body part.
    Non-fatal and best-effort: a node captured on older firmware has no sidecar.
    """
    warned = False
    for n in nodes:
        side = os.path.join(capture_dir, f"{_safe_name(n['node_id'])}.seg.json")
        if not os.path.exists(side):
            continue
        try:
            with open(side, encoding="utf-8-sig") as f:
                node_seg = json.load(f).get("segment")
        except (OSError, ValueError):
            continue
        if node_seg and node_seg != n["segment"]:
            print(f"    [!] {n['column']} {n['node_id']}: montage says "
                  f"'{n['segment']}' but the node's header says '{node_seg}'. "
                  f"Using the montage value — fix the montage or re-enroll if the "
                  f"node moved.")
            warned = True
    if not warned:
        print("    (node headers agree with the montage)")


def _run(cmd, step):
    print(f"\n===== {step} =====")
    print("  $ " + " ".join(cmd))
    r = subprocess.run(cmd)
    if r.returncode != 0:
        raise SystemExit(f"[analyze] {step} failed (exit {r.returncode}). "
                         f"Fix the above and re-run.")


def run(montage_path, capture_dir, out_html, outdir, window, fs,
        facing_deg=None):
    montage = load_montage(montage_path)
    logs, nodes = ordered_logs(montage, capture_dir)

    print("[analyze] montage binding (column order = reconcile order):")
    for n, p in zip(nodes, logs):
        print(f"    {n['column']}  {n['segment']:<14} <- {os.path.basename(p)}")
    warn_segment_disagreements(nodes, capture_dir)

    os.makedirs(outdir, exist_ok=True)
    aligned = os.path.join(outdir, "aligned.csv")
    calib = os.path.join(outdir, "calibration.json")
    metrics = os.path.join(outdir, "metrics.json")
    out_html = os.path.join(outdir, out_html)
    py = sys.executable

    # 1. reconcile — logs in montage column order
    cmd = [py, os.path.join(TOOLS, "reconcile_nodes.py"), *logs, "--out", aligned]
    if fs:
        cmd += ["--fs", str(fs)]
    _run(cmd, "1/5 reconcile (align onto one timeline)")

    # 2. capability — which joints this montage supports
    _run([py, os.path.join(TOOLS, "motion_capabilities.py"), montage_path],
         "2/5 capability check")

    # 3. calibrate — auto neutral window unless overridden
    cmd = [py, os.path.join(TOOLS, "calibrate_segments.py"), "calibrate",
           aligned, montage_path, "--out", calib]
    if window:
        cmd += ["--window", window]
    if facing_deg is not None:
        cmd += ["--facing-deg", str(facing_deg)]
    _run(cmd, "3/5 calibrate (sensor->segment offsets)")

    # 4. metrics — per-DOF joint angles + ROM over the calibrated stream. Runs
    #    BEFORE render so the viewer can bake the metrics panel into one page.
    _run([py, os.path.join(TOOLS, "metrics.py"), "compute", aligned,
          montage_path, "--calibration", calib, "--out", metrics],
         "4/5 metrics (per-DOF joint angles + range of motion)")

    # 5. render — the stage-7 review: the 3-D viewer + the metrics panel, one page
    #    (it also picks up reconcile's aligned.quality.json for the sync chips)
    _run([py, os.path.join(TOOLS, "skeleton_viewer.py"), "render", aligned,
          montage_path, "--calibration", calib, "--metrics", metrics,
          "--out", out_html],
         "5/5 render (stage-7 review: viewer + metrics panel)")

    print("\n===== DONE =====")
    print(f"  aligned stream : {aligned}")
    print(f"  calibration    : {calib}")
    print(f"  metrics        : {metrics}")
    print(f"  review (html)  : {out_html}   (3-D viewer + metrics panel)")
    print(f"  open it        : file://{os.path.abspath(out_html)}")


# ---------------------------------------------------------------------------
# Self-test: synthesize two node logs, run the whole chain end-to-end
# ---------------------------------------------------------------------------
def _write_synth_log(path, offset_ms, seed):
    """Write a synthetic 20-byte-record .bin: a still neutral hold, a shared
    whole-arm wiggle, then some motion — enough for reconcile + calibrate."""
    import numpy as np
    rng = np.random.default_rng(seed)
    fs, n = 50.0, 400
    t = (np.arange(n) * (1000.0 / fs)).astype(np.int64) + offset_ms
    q = np.zeros((n, 4))
    q[:, 0] = 1.0                                   # identity (neutral) baseline
    # shared wiggle 100..250: a yaw both nodes see (correlated -> clock lock)
    for i in range(n):
        if 100 <= i < 250:
            ang = 0.6 * np.sin((i - 100) * 0.20)
            q[i] = [np.cos(ang / 2), 0.0, 0.0, np.sin(ang / 2)]
    q += rng.normal(0, 1e-4, q.shape)
    q /= np.linalg.norm(q, axis=1, keepdims=True)
    rec = struct.Struct("<Iffff")
    with open(path, "wb") as f:
        for i in range(n):
            f.write(rec.pack(int(t[i]), *q[i]))


def selftest():
    import tempfile
    ok = True
    tmp = tempfile.mkdtemp(prefix="hulc_analyze_")
    cap = os.path.join(tmp, "capture")
    os.makedirs(cap, exist_ok=True)

    ids = ["HULC-IMU-AAAA", "HULC-IMU-BBBB"]
    _write_synth_log(os.path.join(cap, f"{ids[0]}.bin"), offset_ms=0, seed=1)
    _write_synth_log(os.path.join(cap, f"{ids[1]}.bin"), offset_ms=37, seed=2)

    montage = {
        "schema_version": "1.0",
        "subject": {"id": "S01", "notes": "selftest"},
        "session": {"id": "t", "aligned_csv": "aligned.csv"},
        "calibration": {"neutral_pose": "N-pose", "captured": True,
                        "t_window_ms": [0, 1500], "functional": []},
        "nodes": [
            {"node_id": ids[0], "column": "n0", "segment": "upper_arm_r",
             "landmark": "", "calibrated": True},
            {"node_id": ids[1], "column": "n1", "segment": "forearm_r",
             "landmark": "", "calibrated": True},
        ],
    }
    mpath = os.path.join(tmp, "montage.json")
    with open(mpath, "w") as f:
        json.dump(montage, f)

    # log resolution + ordering
    logs, nodes = ordered_logs(montage, cap)
    check = lambda c, m: print(f"[selftest] {'ok ' if c else 'FAIL'}: {m}")
    r1 = logs[0].endswith("AAAA.bin") and logs[1].endswith("BBBB.bin")
    ok = ok and r1
    check(r1, "logs resolved in montage column order (n0=AAAA, n1=BBBB)")

    # missing-log error path
    bad = dict(montage); bad_nodes = [dict(montage["nodes"][0]),
                                      {**montage["nodes"][1], "node_id": "HULC-IMU-ZZZZ"}]
    bad = {**montage, "nodes": bad_nodes}
    try:
        ordered_logs(bad, cap)
        ok = False; check(False, "missing log should have raised")
    except SystemExit:
        check(True, "missing log raises a clear error")

    # full chain end-to-end
    try:
        run(mpath, cap, "out.html", tmp, window=None, fs=None)
        made = all(os.path.exists(os.path.join(tmp, f)) for f in
                   ("out.html", "calibration.json", "metrics.json"))
        ok = ok and made
        check(made, "end-to-end run produced calibration.json + metrics.json + "
              "out.html")
        # the stage-7 page fuses both halves: the baked 3-D scene AND the metrics
        # panel, in one self-contained (no external script) file.
        with open(os.path.join(tmp, "out.html"), encoding="utf-8") as f:
            html = f.read()
        fused = ('"frames"' in html and '"metrics":' in html
                 and 'id="metrics"' in html and "<script src=" not in html)
        ok = ok and fused
        check(fused, "stage-7 page fuses the 3-D scene + a metrics panel, "
              "self-contained")
        # reconcile's sync / data-loss sidecar reaches the page (sync chips)
        qual = (os.path.exists(os.path.join(tmp, "aligned.quality.json"))
                and '"quality":{"schema_version"' in html)
        ok = ok and qual
        check(qual, "reconcile's sync / data-loss summary is baked into the page")
    except SystemExit as e:
        ok = False; check(False, f"end-to-end run failed: {e}")

    print(f"\n[selftest] {'PASS' if ok else 'FAIL'} — log binding/ordering, the "
          f"missing-log guard, and the full reconcile->render chain "
          f"(artifacts in {tmp}).")
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd")

    pr = sub.add_parser("run", help="run reconcile->capability->calibrate->render")
    pr.add_argument("--montage", required=True, help="montage JSON")
    pr.add_argument("--capture-dir", required=True,
                    help="directory of offloaded <node_id>.bin logs")
    pr.add_argument("--out", default="session.html", help="viewer HTML filename")
    pr.add_argument("--outdir", default=".",
                    help="where aligned.csv/calibration.json/html are written")
    pr.add_argument("--window", metavar="t0,t1",
                    help="neutral window in ms (default: auto-detect)")
    pr.add_argument("--fs", type=float, help="resample rate Hz (default: native)")
    pr.add_argument("--facing-deg", type=float, metavar="DEG",
                    help="subject's facing at neutral, degrees clockwise from "
                         "world +Y; gives anatomical joint axes when the montage "
                         "has no torso node")

    sub.add_parser("selftest", help="validate the pipeline on synthetic logs")

    args = ap.parse_args()
    if args.cmd == "selftest":
        sys.exit(selftest())
    if args.cmd == "run":
        run(args.montage, args.capture_dir, args.out, args.outdir,
            args.window, args.fs, args.facing_deg)
        return
    ap.error("choose a command: run | selftest")


if __name__ == "__main__":
    main()
