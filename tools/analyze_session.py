"""
HULC Motion Shirt — one-shot post-offload pipeline.

Turns a directory of offloaded node logs + a montage into a viewer + metrics in
ONE command, running the post-offload stages in order and binding logs to
segments automatically (no hand-ordering of .bin files):

    reconcile  -> aligned.csv        (logs passed in montage column order;
                                      vector clock sync, clock-restart check)
    capability -> which joints are computable for this montage
    calibrate  -> calibration.json   (the freeze after the sync movement;
                                      facing; where the nodes come off)
    metrics    -> metrics.json       (per-DOF joint angles, ROM, rep bouts from
                                      the freeze to the nodes coming off)
    opensense  -> <outdir>/opensense/<profile>/  (optional, --opensense-model,
                                      once per model)
    render     -> <out>.html         (stage-7 review: viewer + metrics panel;
                                      with OpenSense, a switch between the
                                      direct sensor view and each model solve)

The montage records each node's column (n0, n1, ...) and its id. Offload names
each file by node id (e.g. HULC-IMU-485C.bin), so this tool resolves every
node's log from --capture-dir and feeds reconcile the files in the RIGHT order.
Run it once per BLOCK (one mounting, offloaded at its charging checkpoint into
its own folder — see COLLECTION_SOP.md, the one-page recording card).

Usage
-----
    # validate the orchestration on synthetic logs (no hardware):
    python tools/analyze_session.py selftest

    # run the whole chain for one block:
    python tools/analyze_session.py run --montage montage.json \
        --capture-dir ./capture/block1 --outdir ./out/block1

    # no torso node and too little elbow flexion to infer the facing? state it:
    python tools/analyze_session.py run --montage montage.json \
        --capture-dir ./capture/block1 --outdir ./out/block1 --facing-deg 90

    # also solve on published models; the page gets a Sensors / model switch:
    python tools/analyze_session.py run --montage montage.json \
        --capture-dir ./capture/block1 --outdir ./out/block1 \
        --opensense-model ThoracoscapularShoulderModel.osim \
        --opensense-model Rajagopal2015_opensense.osim

    # a recording made in the old order (neutral hold BEFORE the sync gesture):
    python tools/analyze_session.py run --montage montage.json \
        --capture-dir ./capture/old --protocol hold-first

    # override the neutral window / output name:
    python tools/analyze_session.py run --montage montage.json \
        --capture-dir ./capture/block1 --window 1200,4000 --out elbow.html
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


def resolve_opensense_models(requested):
    """Which OpenSim models to solve on, and a hint when none can run.

    `requested`: --opensense-model values (files or folders); [] = --no-opensense;
    None = automatic: $HULC_OPENSENSE_MODEL (os.pathsep-separated), else the
    folder `opensense_ik.py fetch-models` fills — used only when OpenSim is
    installed, so a plain setup never pays for (or fails on) a model solve."""
    from opensense_ik import DEFAULT_MODELS_DIR, find_models, opensim_available
    how = ("model solves need `pip install opensim` and the models: "
           "`python tools/opensense_ik.py fetch-models`; the review page then "
           "offers a Sensors / model switch")
    if requested == []:
        return [], None
    explicit = requested is not None
    if not explicit:
        env = os.environ.get("HULC_OPENSENSE_MODEL")
        requested = ([p for p in env.split(os.pathsep) if p] if env
                     else [DEFAULT_MODELS_DIR] if os.path.isdir(DEFAULT_MODELS_DIR)
                     else [])
    models = find_models([p for p in requested if os.path.exists(p)])
    missing = [p for p in requested if not os.path.exists(p)]
    if not opensim_available():
        return [], ("OpenSim is not installed, so no model solves this run — "
                    + how) if (models or explicit) else ("no model solves: " + how)
    if not models:
        return [], ((f"no OpenSim model found at {', '.join(missing or requested)} — "
                     if explicit else "no model solves: ") + how)
    return models, (f"OpenSense on {len(models)} model(s): "
                    + ", ".join(os.path.basename(m) for m in models)
                    + ("" if explicit else "  (found automatically; --no-opensense "
                       "skips)"))


# ---------------------------------------------------------------------------
# Block check — accept or redo, read off the run's own outputs
# ---------------------------------------------------------------------------
MIN_ACTIVE_HZ = 8.0          # median logging rate below this = old firmware / throttle
PEAK_RATIO_MIN = 1.25        # reconcile_nodes.VECTOR_PEAK_RATIO_MIN
NOISY_POSE_DEG = 3.0         # calibrate's "noisy pose" flag
# The trunk sits near its freeze pose for most of a session: on real captures
# the torso's median tilt from the freeze was 4–5°; a torso calibrated while its
# node lay on the table read 82° (the figure drawn face-down).
TRUNK_MAX_MEDIAN_DEG = 30.0
# A joint spending more than this share of the session outside physiological
# limits is calibrated wrong, not moving oddly (`plausibility.outside_limits_frac`).
OUTSIDE_LIMITS_REDO_FRAC = 0.25


def _log_rate_hz(path):
    from reconcile_nodes import load_log
    t = load_log(path)[0]
    d = [b - a for a, b in zip(t[:-1], t[1:]) if b > a]
    if not d:
        return None
    d.sort()
    return 1000.0 / d[len(d) // 2]


def block_check(quality, calibration, metrics, solver_reports=(), log_rates=None):
    """The accept/redo verdict for one block, from the run's own outputs.

    Returns {"verdict": "OK" | "REDO", "checks": [{check, status, detail, fix}]};
    status is OK, NOTE (usable, but read the note) or REDO (re-record)."""
    checks = []

    def add(check, status, detail, fix=""):
        checks.append({"check": check, "status": status, "detail": detail,
                       "fix": fix})

    # 1. clocks aligned (the sync movement)
    bad = [n["column"] for n in (quality or {}).get("nodes", [])
           if not n.get("reference")
           and (not n.get("sync_reliable", True)
                or (n.get("sync_method") == "vector"
                    and (n.get("sync_peak_ratio") or 0) < PEAK_RATIO_MIN))]
    if quality is None:
        add("Sync", "NOTE", "no sync summary (aligned.quality.json) to check")
    elif bad:
        add("Sync", "REDO", f"clocks could not be aligned for {', '.join(bad)}",
            "make the sync movement bigger, moving EVERY node: trunk twists with "
            "the hand on the hip (torso node), straight-arm swings (arm only)")
    else:
        add("Sync", "OK", "every node lined up on one clock")
    restarts = [n["column"] for n in (quality or {}).get("nodes", [])
                if n.get("clock_restarts")]
    if restarts:
        add("Clock", "NOTE", f"{', '.join(restarts)} rebooted mid-block; part of "
            "its log was left out", "charge before the battery runs out")
    # 2. logging rate
    for name, hz in (log_rates or {}).items():
        if hz is not None and hz < MIN_ACTIVE_HZ:
            add("Logging rate", "NOTE", f"{name} logged at {hz:.1f} Hz (want ~10)",
                "check the firmware version and the flash watermark")
    # 3. the freeze (neutral hold)
    neu = (calibration or {}).get("neutral", {})
    found = neu.get("found_by", "")
    if "!" in found or not neu.get("still_ok", True):
        add("Freeze", "REDO", found.split("— ")[-1] if found else "not found",
            "right after the sync movement, arms straight down, palms to the "
            "thighs, freeze ~5 s")
    elif "--protocol hold-first" in found:
        add("Freeze", "NOTE", "a still hold ends right as the sync movement "
            "starts — this looks like an old-order recording (hold first)",
            "if it is, re-run with --protocol hold-first")
    else:
        add("Freeze", "OK", found.split("ignoring it; ")[-1] or "found")
        noisy = [seg for seg, v in (calibration or {}).get("segments", {}).items()
                 if v.get("pose_residual_deg", 0) > NOISY_POSE_DEG]
        if noisy:
            add("Freeze", "NOTE", f"{', '.join(noisy)} was not fully still in the "
                "freeze", "freeze completely for the count of five")
    # 4. which way the subject faced
    h = (calibration or {}).get("heading", {})
    if not h.get("confident"):
        add("Facing", "NOTE", "front direction unknown — joint angles are "
            "relative only",
            "torso node flat on the sternum; with no torso node, include a few "
            "elbow bends in the task")
    else:
        add("Facing", "OK", {"torso_auto": "from the torso node",
                             "elbow_hinge": "from the elbow bending",
                             "manual": "given with --facing-deg"}.get(
                                 h.get("source"), str(h.get("source"))))
    # 5. where the session ends
    e = (calibration or {}).get("end") or {}
    if e.get("t_end_ms") is not None:
        add("Session end", "OK", f"analysis ends at {e['t_end_ms'] / 1000:.1f} s "
            + ("(set by hand)" if e.get("method") == "manual"
               else "where the nodes came off"))
    else:
        add("Session end", "NOTE", e.get("note", "nodes not seen coming off"),
            "if the tail includes taking the nodes off, re-run with --end <ms>")
    # 6. physically sensible results
    torso = next((sg for sg in (metrics or {}).get("segments", [])
                  if sg.get("segment") == "torso" and sg.get("calibrated")), None)
    tilt = ((torso or {}).get("elevation") or {}).get("median_deg")
    if tilt is not None:
        if tilt > TRUNK_MAX_MEDIAN_DEG:
            add("Trunk", "REDO", f"the torso reads {tilt:.0f}° tilted for most of the "
                "session — it was calibrated in the wrong pose (node not yet on the "
                "chest?) or its strap moved",
                "strap the torso node on before the sync movement and freeze "
                "upright; or re-run with --window <freeze t0,t1>")
        else:
            add("Trunk", "OK", f"upright (median tilt {tilt:.0f}° from the freeze)")
    for j in (metrics or {}).get("joints", []):
        worst = max((d.get("plausibility") or {}).get("outside_limits_frac") or 0.0
                    for d in j.get("dofs", [{}]))
        name = j.get("name", j.get("key"))
        if worst > OUTSIDE_LIMITS_REDO_FRAC:
            add("Angles", "REDO", f"{name}: outside physiological limits {worst:.0%} "
                "of the session — a calibration or facing error, not real motion",
                "check the freeze pose (palms to the thighs, elbows straight), the "
                "facing line, and strap slip")
        elif j.get("plausibility_warning"):
            add("Angles", "NOTE", f"{name}: implausible angles", "check the freeze "
                "pose (palms, elbows straight) and strap slip")
    # 7. model solves
    for r in solver_reports:
        ps = r.get("pose_solver", {})
        lost = (ps.get("fit_lost") or {}).get("seconds", 0)
        if lost:
            add("Model fit", "NOTE", f"{ps.get('model_title', 'model')}: lost the "
                f"sensors for {lost:.0f} s (left out of its numbers)",
                "compare with the Sensors view there")
    verdict = "REDO" if any(c["status"] == "REDO" for c in checks) else "OK"
    return {"verdict": verdict, "checks": checks}


def print_block_check(bc):
    notes = sum(c["status"] == "NOTE" for c in bc["checks"])
    print(f"\n===== BLOCK CHECK: {bc['verdict']}"
          + (f" ({notes} note{'s' if notes != 1 else ''})" if notes else "")
          + " =====")
    for c in bc["checks"]:
        print(f"  {c['status']:<5} {c['check']:<12} {c['detail']}")
        if c["status"] != "OK" and c["fix"]:
            print(f"        {'':<12} -> {c['fix']}")


def rerun_command(montage_path, capture_dir, out_html, outdir, fs, facing_deg,
                  opensense_models, protocol):
    """The command that re-runs this analysis, WITHOUT --window / --end — the
    review page's Timeline strip appends those when the user drags a marker."""
    q = lambda v: f'"{v}"' if any(c in str(v) for c in ' "\'') else str(v)
    script = os.path.relpath(os.path.join(TOOLS, "analyze_session.py"))
    if script.startswith(".."):
        script = os.path.join(TOOLS, "analyze_session.py")
    # the interpreter that ran this analysis (it has numpy, and opensim if used)
    parts = [q(sys.executable or "python"), q(script), "run", "--montage", q(montage_path),
             "--capture-dir", q(capture_dir), "--outdir", q(outdir)]
    if out_html != "session.html":
        parts += ["--out", q(out_html)]
    if protocol:
        parts += ["--protocol", protocol]
    if facing_deg is not None:
        parts += ["--facing-deg", f"{facing_deg:g}"]
    if fs:
        parts += ["--fs", f"{fs:g}"]
    if opensense_models == []:
        parts.append("--no-opensense")
    for m in opensense_models or []:
        parts += ["--opensense-model", q(m)]
    return " ".join(parts)


def run(montage_path, capture_dir, out_html, outdir, window, fs,
        facing_deg=None, opensense_models=None, protocol=None, end_ms=None):
    rerun = rerun_command(montage_path, capture_dir, out_html, outdir, fs,
                          facing_deg, opensense_models, protocol)
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
    if protocol:
        cmd += ["--protocol", protocol]
    if end_ms is not None:
        cmd += ["--end", str(end_ms)]
    if facing_deg is not None:
        cmd += ["--facing-deg", str(facing_deg)]
    _run(cmd, "3/5 calibrate (sensor->segment offsets)")

    # 4. metrics — per-DOF joint angles + ROM over the calibrated stream. Runs
    #    BEFORE render so the viewer can bake the metrics panel into one page.
    _run([py, os.path.join(TOOLS, "metrics.py"), "compute", aligned,
          montage_path, "--calibration", calib, "--out", metrics],
         "4/5 metrics (per-DOF joint angles + range of motion)")

    # 5. (optional) the OpenSense path — the same session solved on each given
    #    OpenSim model, reported through the same metrics. A failed solve is
    #    reported and skipped: the direct sensor review is still produced.
    solver_dirs = []
    models, os_hint = resolve_opensense_models(opensense_models)
    if os_hint:
        print(f"\n[analyze] {os_hint}")
    if models:
        from opensense_ik import detect_profile
        for k, model in enumerate(models, 1):
            try:
                name = detect_profile(model)
            except (OSError, SystemExit, ValueError, KeyError):
                name = os.path.splitext(os.path.basename(model))[0]
            os_dir = os.path.join(outdir, "opensense", name)
            if os_dir in solver_dirs:
                os_dir += f"_{k}"
            cmd = [py, os.path.join(TOOLS, "opensense_ik.py"), "run", aligned,
                   montage_path, "--calibration", calib, "--model", model,
                   "--outdir", os_dir, "--no-render"]
            step = f"OpenSense {k}/{len(models)}: {os.path.basename(model)}"
            print(f"\n===== {step} =====\n  $ " + " ".join(cmd))
            if subprocess.run(cmd).returncode == 0:
                solver_dirs.append(os_dir)
            else:
                print(f"[analyze] {step} failed — left out of the review page.")

    # 6. render — the stage-7 review: the 3-D viewer + the metrics panel, one page
    #    (it also picks up reconcile's aligned.quality.json for the sync chips).
    #    Each OpenSense solve rides along, switchable against the sensor view.
    cmd = [py, os.path.join(TOOLS, "skeleton_viewer.py"), "render", aligned,
           montage_path, "--calibration", calib, "--metrics", metrics,
           "--out", out_html, "--rerun", rerun]
    for d in solver_dirs:
        cmd += ["--solver-dir", d]
    _run(cmd, "5/5 render (stage-7 review: viewer + metrics panel)")

    print("\n===== DONE =====")
    print(f"  aligned stream : {aligned}")
    print(f"  calibration    : {calib}")
    print(f"  metrics        : {metrics}")
    print(f"  review (html)  : {out_html}   (3-D viewer + metrics panel)")
    print(f"  open it        : file://{os.path.abspath(out_html)}")
    for d in solver_dirs:
        print(f"  opensense      : {os.path.join(d, 'opensense_metrics.json')}"
              f"  (switch to it in the review page)")
    if not solver_dirs:
        print("  model solves   : none" + (f" — {os_hint}" if os_hint and not models
                                           else ""))

    # accept or redo, from the run's own outputs
    def _read(path):
        try:
            with open(path, encoding="utf-8-sig") as f:
                return json.load(f)
        except (OSError, ValueError):
            return None
    rates = {}
    for n, p in zip(nodes, logs):
        try:
            rates[n["segment"]] = _log_rate_hz(p)
        except Exception:                      # a rate is a nicety, never fatal
            rates[n["segment"]] = None
    bc = block_check(_read(os.path.splitext(aligned)[0] + ".quality.json"),
                     _read(calib), _read(metrics),
                     [r for r in (_read(os.path.join(d, "opensense_metrics.json"))
                                  for d in solver_dirs) if r], rates)
    with open(os.path.join(outdir, "block_check.json"), "w") as f:
        json.dump(bc, f, indent=2)
    print_block_check(bc)
    return bc


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
        bcp = os.path.join(tmp, "block_check.json")
        with open(bcp) as f:
            bc_run = json.load(f)
        made_bc = bc_run.get("verdict") in ("OK", "REDO") and bc_run.get("checks")
        ok = ok and bool(made_bc)
        check(bool(made_bc), f"block check written (verdict {bc_run.get('verdict')})")
    except SystemExit as e:
        ok = False; check(False, f"end-to-end run failed: {e}")

    # the verdict logic on its own: a failed sync and a missing freeze are
    # REDO; an unknown facing alone is a NOTE
    good_q = {"nodes": [{"column": "n0", "reference": True},
                        {"column": "n1", "sync_reliable": True,
                         "sync_method": "vector", "sync_peak_ratio": 1.7}]}
    good_c = {"neutral": {"found_by": "freeze found 3 s after the sync movement",
                          "still_ok": True},
              "heading": {"confident": True, "source": "torso_auto"},
              "end": {"t_end_ms": 90000.0, "method": "pause_before_takeoff"},
              "segments": {}}
    bad_q = {"nodes": [{"column": "n0", "reference": True},
                       {"column": "n1", "sync_reliable": True,
                        "sync_method": "vector", "sync_peak_ratio": 1.1}]}
    bad_c = {**good_c, "neutral": {"found_by": "! nothing still enough — using "
                                   "the quietest moment", "still_ok": True},
             "heading": {"confident": False}}
    v_good = block_check(good_q, good_c, {"joints": []})
    v_bad = block_check(bad_q, bad_c, {"joints": []})
    v_face = block_check(good_q, {**good_c, "heading": {"confident": False}},
                         {"joints": []})
    torso_seg = lambda tilt: {"segments": [{"segment": "torso", "calibrated": True,
                                            "elevation": {"median_deg": tilt}}],
                              "joints": []}
    v_flat = block_check(good_q, good_c, torso_seg(82.0))      # the face-down torso
    v_up = block_check(good_q, good_c, torso_seg(5.0))
    v_lim = block_check(good_q, good_c, {"joints": [{"key": "elbow_r", "name": "Elbow",
        "dofs": [{"plausibility": {"outside_limits_frac": 0.6}}]}]})
    vok = (v_good["verdict"] == "OK"
           and all(c["status"] == "OK" for c in v_good["checks"])
           and v_bad["verdict"] == "REDO"
           and {c["check"] for c in v_bad["checks"] if c["status"] == "REDO"}
           == {"Sync", "Freeze"}
           and v_face["verdict"] == "OK"
           and any(c["check"] == "Facing" and c["status"] == "NOTE"
                   for c in v_face["checks"])
           and v_flat["verdict"] == "REDO" and v_up["verdict"] == "OK"
           and v_lim["verdict"] == "REDO")
    ok = ok and vok
    check(vok, "block check: clean block OK; weak sync + no freeze -> REDO on "
          "both; unknown facing alone -> NOTE; torso tilted 82° all session -> "
          "REDO; elbow outside its limits 60% of the time -> REDO")

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
    pr.add_argument("--end", type=float, metavar="T_MS",
                    help="end the analysis here (t_common_ms) instead of where "
                         "the nodes are detected coming off")
    pr.add_argument("--protocol", choices=["sync-first", "hold-first"],
                    help="order of the recording: sync-first (default: sync "
                         "gesture, then the neutral hold) or hold-first "
                         "(recordings made before that order). Default: the "
                         "montage's calibration.protocol, else sync-first")
    pr.add_argument("--facing-deg", type=float, metavar="DEG",
                    help="subject's facing at neutral, degrees clockwise from "
                         "world +Y; gives anatomical joint axes when the montage "
                         "has no torso node")
    pr.add_argument("--no-opensense", action="store_true",
                    help="skip the model solves even when models are installed")
    pr.add_argument("--opensense-model", metavar="OSIM", action="append",
                    help="also solve the session with OpenSim OpenSense on this "
                         "model (ThoracoscapularShoulderModel.osim for the right "
                         "arm, or Rajagopal2015_opensense.osim; needs "
                         "`pip install opensim`) -> <outdir>/opensense/<profile>/; "
                         "a folder adds every model in it; repeat for several. "
                         "Default: $HULC_OPENSENSE_MODEL, else the models "
                         "`opensense_ik.py fetch-models` downloaded. The review "
                         "page then has a switch between the direct sensor view "
                         "and each model")

    sub.add_parser("selftest", help="validate the pipeline on synthetic logs")

    args = ap.parse_args()
    if args.cmd == "selftest":
        sys.exit(selftest())
    if args.cmd == "run":
        run(args.montage, args.capture_dir, args.out, args.outdir,
            args.window, args.fs, args.facing_deg,
            [] if args.no_opensense else args.opensense_model, args.protocol,
            args.end)
        return
    ap.error("choose a command: run | selftest")


if __name__ == "__main__":
    main()
