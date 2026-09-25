#!/usr/bin/env python3
"""
HULC Motion Shirt — the OpenSense path: solve the pose with OpenSim's
validated musculoskeletal model instead of our own per-joint chain.

Where it sits
-------------
The default pipeline turns each node's calibrated orientation straight into
joint angles (calibrate_segments.py -> metrics.py). This tool is the
alternative middle: it hands the same aligned stream to OpenSim OpenSense
(IMUPlacer + IMU inverse kinematics on the Rajagopal 2015 full-body model,
Stanford, Apache-2.0), which fits a jointed skeleton — real joint axes, joint
limits, one kinematic chain — to all nodes at once. The solved pose is then
turned back into per-segment orientations and fed through the SAME
metrics.py and skeleton_viewer.py code, so the two paths differ only in how
the pose is solved, never in how an angle is defined or reported.

    aligned.csv + calibration.json
        │  heading-corrected quaternions (the calibration's anatomical frame)
        ▼
    OpenSense .sto  ──IMUPlacer (neutral hold)──IMU IK (from the hold on)──▶ .mot
        │  forward kinematics of the solved model
        ▼
    opensense_aligned.csv (segment orientation from neutral, world axes)
        └──▶ metrics.py  →  opensense_metrics.json      (same report format)
        └──▶ skeleton_viewer.py → opensense.html         (same viewer)

What it relies on from the default pipeline
-------------------------------------------
* The neutral hold (calibration.json `neutral.t_window_ms`) — the model's
  default pose is matched to it; IK starts there (a solve started on setup
  fidget locks into a wrong branch and never recovers).
* The facing (`anatomical_frame`) — OpenSense's own heading option only moves
  the sensor placement, not the IK input, so the data are rotated into the
  model's frame here. No facing -> this path refuses to run.

Model preparation (why the shipped model can't be used as is)
------------------------------------------------------------
* Forearm neutral: the model's pro/sup zero is palms-FORWARD and its range stops
  at thumb-forward, so it cannot pronate. Our N-pose is palms facing the
  thighs, so the default is set to 90° and the range widened to −10…190°.
* Shoulder flexion is capped at 90°; with it, an overhead reach silently bends
  the trunk and elbow to compensate. Ranges are widened for rehab motion.
* Only what the montage measures is free: trunk rotation only with a torso
  node; each arm's chain down to its deepest node (so a forearm node with no
  upper-arm node still has a chain above it to absorb its orientation);
  everything else is locked. Joint angles are reported only where metrics.py's
  body model says both segments are measured — the model does not recover
  what is not measured.

Setup (one time)
----------------
    pip install opensim                 # official Stanford wheels, Python 3.11–3.13
    git clone --depth 1 --filter=blob:none --sparse \\
        https://github.com/opensim-org/opensim-models
    (cd opensim-models && git sparse-checkout set Models/Rajagopal_OpenSense)
    # model: opensim-models/Models/Rajagopal_OpenSense/Rajagopal2015_opensense.osim

Usage
-----
    python tools/opensense_ik.py run aligned.csv montage.json \\
        --calibration calibration.json --model Rajagopal2015_opensense.osim \\
        --outdir out/opensense
    # or as the last stage of analyze_session.py run ... --opensense-model PATH

    python tools/opensense_ik.py selftest   # model-plan + export logic, no OpenSim
"""

import argparse
import copy
import json
import math
import os
import subprocess
import sys
import time

try:
    import numpy as np
except ImportError:  # pragma: no cover
    raise SystemExit("This tool needs numpy:  pip install numpy")

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from calibrate_segments import (  # noqa: E402
    load_aligned, load_montage, qmul, qconj, qnorm, quat_average,
)
from metrics import (  # noqa: E402
    compute_metrics, print_report, resolve_anatomical_frame, trim_to_analysis,
)
from reconcile_nodes import quality_path  # noqa: E402

TOOLS = os.path.dirname(os.path.abspath(__file__))
IDENTITY = [1.0, 0.0, 0.0, 0.0]

# our segment -> the model body its node is placed on. The forearm node goes on
# the RADIUS (it carries pro/supination), not the ulna.
SEGMENT_BODY = {
    "torso": "torso",
    "upper_arm_r": "humerus_r", "forearm_r": "radius_r", "hand_r": "hand_r",
    "upper_arm_l": "humerus_l", "forearm_l": "radius_l", "hand_l": "hand_l",
}
TRUNK_COORDS = ("pelvis_tilt", "pelvis_list", "pelvis_rotation")
SHOULDER = ("arm_flex", "arm_add", "arm_rot")
ELBOW = ("elbow_flex", "pro_sup")
WRIST = ("wrist_flex", "wrist_dev")
# widened ranges (degrees) — see "Model preparation" above
RANGES = {"arm_flex": (-90, 180), "arm_add": (-180, 90), "arm_rot": (-180, 180),
          "elbow_flex": (-15, 160), "pro_sup": (-10, 190),
          "wrist_flex": (-90, 90), "wrist_dev": (-40, 50)}
# model pro_sup at the neutral pose: 0 = palms forward, 90 = palms to thighs
FOREARM_NEUTRAL_DEG = {"palms-in": 90.0, "palms-forward": 0.0}


# ---------------------------------------------------------------------------
# Model plan — which coordinates the montage can drive (pure logic, testable)
# ---------------------------------------------------------------------------
def plan_free_coords(present_segments):
    """Coordinates left free for IK; every other coordinate is locked.

    Trunk: only with a torso node (otherwise it is held upright). Each arm:
    free from the shoulder down to the DEEPEST node on that side — the chain
    above the most proximal node must stay free to absorb its orientation, and
    nothing below the deepest node is observable."""
    free = set()
    if "torso" in present_segments:
        free.update(TRUNK_COORDS)
    for side in ("r", "l"):
        depth = max((i for i, seg in enumerate(
            (f"upper_arm_{side}", f"forearm_{side}", f"hand_{side}"))
            if seg in present_segments), default=-1)
        groups = (SHOULDER, ELBOW, WRIST)[:depth + 1]
        for g in groups:
            free.update(f"{c}_{side}" for c in g)
    return free


def prepare_model(model_in, model_out, present_segments, forearm_neutral):
    import opensim as osim
    osim.Logger.setLevelString("error")   # the model's meshes are not needed
    m = osim.Model(model_in)
    cs = m.getCoordinateSet()
    free = plan_free_coords(present_segments)
    for i in range(cs.getSize()):
        c = cs.get(i)
        c.set_locked(c.getName() not in free)
        base = c.getName()[:-2] if c.getName()[-2:] in ("_r", "_l") else None
        if base in RANGES:
            lo, hi = RANGES[base]
            c.setRangeMin(math.radians(lo))
            c.setRangeMax(math.radians(hi))
    for side in ("r", "l"):
        cs.get(f"pro_sup_{side}").setDefaultValue(
            math.radians(FOREARM_NEUTRAL_DEG[forearm_neutral]))
    m.printToXML(model_out)
    return free


# ---------------------------------------------------------------------------
# Export — OpenSense orientation files
# ---------------------------------------------------------------------------
def write_sto(path, times_s, cols, rate_hz):
    """OpenSense quaternion table: one `<body>_imu` column of w,x,y,z per node."""
    with open(path, "w") as f:
        f.write(f"DataRate={rate_hz:.6f}\nDataType=Quaternion\nversion=3\n"
                f"OpenSimVersion=4.6\nendheader\n")
        f.write("time\t" + "\t".join(f"{b}_imu" for b in cols) + "\n")
        for k, tt in enumerate(times_s):
            f.write(f"{tt:.6f}\t" + "\t".join(
                ",".join(f"{v:.8f}" for v in cols[b][k]) for b in cols) + "\n")


def to_model_frame(seg_quats, q_wa):
    """World-from-sensor -> model-ground-from-sensor. The model's ground axes
    are the anatomical axes (X anterior, Y up, Z right), so this is conj(q_WA)."""
    qc = qconj(q_wa)
    return {SEGMENT_BODY[s]: qnorm(qmul(qc, q)) for s, q in seg_quats.items()}


def neutral_average(cols, t_ms, t0, t1):
    m = (t_ms >= t0) & (t_ms <= t1)
    return {b: quat_average(q[m])[None, :] for b, q in cols.items()}


# ---------------------------------------------------------------------------
# Solve + forward kinematics
# ---------------------------------------------------------------------------
def solve(model_path, cal_sto, motion_sto, outdir, t_start_s, t_end_s):
    import opensim as osim
    osim.Logger.setLevelString("error")
    R0 = osim.Vec3(0, 0, 0)        # data already in model-ground axes
    placer = osim.IMUPlacer()
    placer.set_model_file(model_path)
    placer.set_orientation_file_for_calibration(cal_sto)
    placer.set_sensor_to_opensim_rotations(R0)
    placer.run(False)
    calibrated = os.path.join(outdir, "calibrated_model.osim")
    placer.getCalibratedModel().printToXML(calibrated)

    ik = osim.IMUInverseKinematicsTool()
    ik.set_model_file(calibrated)
    ik.set_orientations_file(motion_sto)
    ik.set_sensor_to_opensim_rotations(R0)
    ik.set_results_directory(outdir)
    ik.set_time_range(0, float(t_start_s))
    ik.set_time_range(1, float(t_end_s))
    ik.set_report_errors(True)
    t0 = time.time()
    ik.run(False)
    secs = time.time() - t0
    stem = os.path.splitext(os.path.basename(motion_sto))[0]
    mot = os.path.join(outdir, f"ik_{stem}.mot")
    err = next((os.path.join(outdir, f) for f in os.listdir(outdir)
                if f.startswith(f"ik_{stem}") and "rror" in f), None)
    return calibrated, mot, err, secs


def body_orientations(calibrated_model, mot_path, bodies):
    """Forward kinematics: each body's ground orientation as a rotation FROM the
    model's neutral (default) pose — the same form as our calibrated q_seg."""
    import opensim as osim
    m = osim.Model(calibrated_model)
    s = m.initSystem()
    cs, bs = m.getCoordinateSet(), m.getBodySet()

    def q_of(b):
        q = bs.get(b).getTransformInGround(s).R().convertRotationToQuaternion()
        return np.array([q.get(i) for i in range(4)])

    m.realizePosition(s)
    q0 = {b: q_of(b) for b in bodies}           # default pose = neutral hold
    tab = osim.TimeSeriesTable(mot_path)
    t_s = np.asarray(tab.getIndependentColumn(), dtype=float)
    labels = list(tab.getColumnLabels())
    vals = {c: tab.getDependentColumn(c).to_numpy() for c in labels}
    rot = {c: cs.get(c).getMotionType() == 1 for c in labels
           if cs.contains(c)}                     # 1 = Rotational (degrees in .mot)
    out = {b: np.zeros((len(t_s), 4)) for b in bodies}
    for k in range(len(t_s)):
        for c, is_rot in rot.items():
            v = vals[c][k]
            cs.get(c).setValue(s, math.radians(v) if is_rot else v, False)
        m.realizePosition(s)
        for b in bodies:
            out[b][k] = qmul(q_of(b), qconj(q0[b]))
    return t_s, out, vals


def residual_stats(err_path):
    if not err_path:
        return {}
    import opensim as osim
    e = osim.TimeSeriesTable(err_path)
    out = {}
    for lbl in e.getColumnLabels():
        v = np.degrees(e.getDependentColumn(lbl).to_numpy())
        out[lbl] = {"median_deg": round(float(np.median(v)), 2),
                    "p95_deg": round(float(np.percentile(v, 95)), 2),
                    "frac_over_20deg": round(float(np.mean(v > 20.0)), 3)}
    return out


def write_aligned(path, t_ms, seg_q, montage):
    """A reconcile-format CSV of the solved segment orientations, so metrics.py
    and the viewer read OpenSense output exactly like a node stream."""
    cols = [n["column"] for n in montage["nodes"]]
    seg_of = {n["column"]: n["segment"] for n in montage["nodes"]}
    header = ["t_common_ms"] + [f"{c}_q{a}" for c in cols for a in "wxyz"]
    data = np.column_stack([t_ms] + [seg_q[seg_of[c]] for c in cols])
    np.savetxt(path, data, delimiter=",", header=",".join(header), comments="",
               fmt="%.6f")


# ---------------------------------------------------------------------------
# The whole path
# ---------------------------------------------------------------------------
def run(aligned_csv, montage_path, calibration_path, model_path, outdir,
        forearm_neutral="palms-in", render=True):
    try:
        import opensim  # noqa: F401
    except ImportError:
        raise SystemExit("[opensense] needs OpenSim:  pip install opensim  "
                         "(official wheels, Python 3.11–3.13)")
    if not model_path or not os.path.exists(model_path):
        raise SystemExit(f"[opensense] model not found: {model_path!r} — see the "
                         f"setup notes at the top of opensense_ik.py")
    montage = load_montage(montage_path)
    with open(calibration_path, encoding="utf-8-sig") as f:
        calibration = json.load(f)
    q_wa = resolve_anatomical_frame(calibration)
    if q_wa is None:
        raise SystemExit("[opensense] the calibration has no facing (anatomical "
                         "frame), so the data cannot be put in the model's frame. "
                         "Add a torso node, flex the elbow during the session, or "
                         "calibrate with --facing-deg.")
    nw = calibration.get("neutral", {}).get("t_window_ms")
    if not nw:
        raise SystemExit("[opensense] calibration.json has no neutral window")
    os.makedirs(outdir, exist_ok=True)

    t_ms, seg_quats, seg_meta = load_aligned(aligned_csv, montage)
    t_ms, seg_quats, _ = trim_to_analysis(t_ms, seg_quats, calibration)
    present = set(seg_quats)
    cols = to_model_frame(seg_quats, q_wa)
    rate = 1000.0 / float(np.median(np.diff(t_ms)))
    t_s = (t_ms - t_ms[0]) / 1000.0
    cal_sto = os.path.join(outdir, "neutral.sto")
    motion_sto = os.path.join(outdir, "orientations.sto")
    write_sto(cal_sto, [0.0], neutral_average(cols, t_ms, *nw), rate)
    write_sto(motion_sto, t_s, cols, rate)

    model_prep = os.path.join(outdir, "prepared_model.osim")
    free = prepare_model(model_path, model_prep, present, forearm_neutral)
    print(f"[opensense] {len(present)} node(s) on "
          f"{', '.join(SEGMENT_BODY[s] for s in sorted(present))}; "
          f"free coordinates: {', '.join(sorted(free))}")
    t_start = max(0.0, (nw[0] - t_ms[0]) / 1000.0)
    calibrated, mot, err, secs = solve(model_prep, cal_sto, motion_sto, outdir,
                                       t_start, float(t_s[-1]))
    print(f"[opensense] IK solved {len(t_s)} frames in {secs:.0f} s -> {mot}")

    bodies = [SEGMENT_BODY[s] for s in present]
    t_ik, q_body, coords = body_orientations(calibrated, mot, bodies)
    # model ground -> world axes: q_w = q_WA ⊗ q_g ⊗ conj(q_WA)
    seg_q = {s: qnorm(qmul(qmul(q_wa, q_body[SEGMENT_BODY[s]]), qconj(q_wa)))
             for s in present}
    t_ik_ms = t_ms[0] + t_ik * 1000.0
    aligned_out = os.path.join(outdir, "opensense_aligned.csv")
    write_aligned(aligned_out, t_ik_ms, seg_q, montage)
    if os.path.exists(quality_path(aligned_csv)):      # sync chips in the viewer
        with open(quality_path(aligned_csv)) as src, \
                open(quality_path(aligned_out), "w") as dst:
            dst.write(src.read())

    # The solved stream is already "rotation from neutral" per segment, so its
    # calibration is identity offsets with the ORIGINAL facing and window.
    cal_out = copy.deepcopy(calibration)
    for seg in cal_out.get("segments", {}).values():
        seg["mounting_offset_quat"] = list(IDENTITY)
    cal_out["pose_solver"] = "opensense"
    cal_path = os.path.join(outdir, "opensense_calibration.json")
    with open(cal_path, "w") as f:
        json.dump(cal_out, f, indent=2)

    t2, sq2, meta2 = load_aligned(aligned_out, montage)
    rep = compute_metrics(montage, t2, sq2, meta2, cal_out, trim=False)
    rep["pose_solver"] = {
        "name": "OpenSim OpenSense (IMUPlacer + IMU IK)",
        "model": os.path.basename(model_path),
        "forearm_neutral": forearm_neutral,
        "free_coordinates": sorted(free),
        "ik_seconds": round(secs, 1),
        "residual_deg": residual_stats(err),
        "model_coordinates_deg": {
            c: {"p5": round(float(np.percentile(v, 5)), 1),
                "p50": round(float(np.median(v)), 1),
                "p95": round(float(np.percentile(v, 95)), 1)}
            for c, v in coords.items() if c in free},
    }
    met_path = os.path.join(outdir, "opensense_metrics.json")
    with open(met_path, "w") as f:
        json.dump(rep, f, indent=2)
    print(f"[opensense] wrote {met_path}\n")
    print_report(rep)
    print("\nOPENSENSE fit (how far each node sits from the solved skeleton)")
    for lbl, r in rep["pose_solver"]["residual_deg"].items():
        print(f"  {lbl:<16} median {r['median_deg']:5.1f}°  p95 {r['p95_deg']:5.1f}°"
              f"  >20°: {r['frac_over_20deg'] * 100:.0f}% of frames")

    if render:
        html = os.path.join(outdir, "opensense.html")
        subprocess.run([sys.executable, os.path.join(TOOLS, "skeleton_viewer.py"),
                        "render", aligned_out, montage_path, "--calibration",
                        cal_path, "--metrics", met_path, "--out", html],
                       check=False)
    return rep


# ---------------------------------------------------------------------------
# Self-test — the OpenSim-free logic (plan, frames, export)
# ---------------------------------------------------------------------------
def selftest():
    import tempfile
    ok = True

    def check(c, msg):
        nonlocal ok
        ok = ok and c
        print(f"[selftest] {'ok ' if c else 'FAIL'}: {msg}")

    full_r = plan_free_coords({"torso", "upper_arm_r", "forearm_r", "hand_r"})
    check(full_r == set(TRUNK_COORDS) | {f"{c}_r" for c in SHOULDER + ELBOW + WRIST},
          "full right arm frees trunk + the whole right chain, nothing on the left")
    no_torso = plan_free_coords({"upper_arm_r", "forearm_r"})
    check(not (no_torso & set(TRUNK_COORDS)) and "elbow_flex_r" in no_torso
          and "wrist_flex_r" not in no_torso,
          "no torso node -> trunk locked upright; nothing below the deepest node")
    distal = plan_free_coords({"forearm_r", "hand_r"})
    check({"arm_flex_r", "elbow_flex_r", "wrist_flex_r"} <= distal,
          "forearm+hand only -> chain above the forearm stays free to absorb it")
    check(plan_free_coords(set()) == set(), "no nodes -> everything locked")

    # frames: a sensor aligned with the anatomical axes lands on model identity
    from calibrate_segments import anatomical_frame_quat
    q_wa = anatomical_frame_quat(-45.0)
    cols = to_model_frame({"forearm_r": q_wa[None, :]}, q_wa)
    check(np.allclose(np.abs(cols["radius_r"][0]), IDENTITY, atol=1e-9),
          "world->model frame: an anatomically aligned sensor reads identity")

    tmp = tempfile.mkdtemp(prefix="hulc_osense_")
    p = os.path.join(tmp, "x.sto")
    write_sto(p, [0.0, 0.1], {"radius_r": np.array([IDENTITY, IDENTITY])}, 10.0)
    with open(p) as f:
        txt = f.read()
    check("DataType=Quaternion" in txt and "radius_r_imu" in txt
          and "1.00000000,0.00000000" in txt, "OpenSense .sto layout")

    print(f"\n[selftest] {'PASS' if ok else 'FAIL'} — model plan (what each montage "
          f"frees), world->model frame, .sto export. The solve itself needs "
          f"OpenSim + the model: see tools/OPENSENSE_FEASIBILITY.md.")
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd")
    pr = sub.add_parser("run", help="solve the session with OpenSense")
    pr.add_argument("aligned_csv")
    pr.add_argument("montage")
    pr.add_argument("--calibration", default="calibration.json")
    pr.add_argument("--model", default=os.environ.get("HULC_OPENSENSE_MODEL"),
                    help="Rajagopal2015_opensense.osim (or $HULC_OPENSENSE_MODEL)")
    pr.add_argument("--outdir", default="opensense")
    pr.add_argument("--forearm-neutral", choices=sorted(FOREARM_NEUTRAL_DEG),
                    default="palms-in",
                    help="forearm in the neutral hold (default: palms-in, "
                         "palms facing the thighs — the N-pose)")
    pr.add_argument("--no-render", action="store_true",
                    help="skip the opensense.html viewer")
    sub.add_parser("selftest", help="validate the OpenSim-free logic")
    args = ap.parse_args()
    if args.cmd == "selftest":
        sys.exit(selftest())
    if args.cmd == "run":
        run(args.aligned_csv, args.montage, args.calibration, args.model,
            args.outdir, args.forearm_neutral, render=not args.no_render)
        return
    ap.error("choose a command: run | selftest")


if __name__ == "__main__":
    main()
