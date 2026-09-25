#!/usr/bin/env python3
"""
HULC Motion Shirt — the OpenSense path: solve the pose with a published
OpenSim musculoskeletal model instead of our own per-joint chain.

Models (profiles, detected from the model file)
-----------------------------------------------
* thoracoscapular — Thoracoscapular Shoulder Model, Seth, Dong, Matias & Delp
  2019 (Front Neurorobot 13:90); scapulothoracic joint from Seth et al. 2016
  (PLoS ONE 11(1)). An upper-limb model: RIGHT arm, thorax + clavicle +
  scapula + humerus + ulna/radius + hand (wrist welded), ISB-style
  glenohumeral coordinates. Recommended for arm/shoulder work.
* rajagopal — Rajagopal et al. 2016 (IEEE TBME 63(10)), OpenSense variant.
  Full body, BOTH arms and wrists, but built for gait: its arm is a simplified
  chain (ball-joint shoulder, no scapula). Use it for the left arm or wrists.

Where it sits
-------------
The default pipeline turns each node's calibrated orientation straight into
joint angles (calibrate_segments.py -> metrics.py). This tool is the
alternative middle: it hands the same aligned stream to OpenSim OpenSense
(IMUPlacer + IMU inverse kinematics; Al Borno et al. 2022, J NeuroEng Rehabil
19:22), which fits the model's jointed skeleton — real joint axes, joint
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

Model preparation (what is changed from the published model, and why)
----------------------------------------------------------------------
* Default pose = our N-pose (arms at the sides, palms facing the thighs),
  because IMUPlacer assumes the neutral hold IS the default pose.
  thoracoscapular: thorax upright; humerus vertical and facing forward (the
  resting scapula tilts the glenoid, so this is 6° of glenohumeral elevation,
  solved numerically); forearm zero is already thumb-forward.
  rajagopal: forearm pro/sup set to 90° (its zero is palms-forward).
* Ranges for rehab motion: rajagopal caps shoulder flexion at 90° and stops
  pro/sup at thumb-forward (so it cannot pronate); both widened.
  thoracoscapular: elevation 0–180°, elbow −15–160°, pro/sup ±100°.
* Ranges are enforced during IK (clamping on), except angles that must wrap:
  the thoracoscapular plane of elevation and axial rotation (only their sum
  is defined with the arm at the side; clamped, the solve sticks at ±180°).
* thoracoscapular: clavicle and scapula are held at the model's resting
  posture — no node measures them — so the glenohumeral joint carries all
  humerothoracic motion.
* Only what the montage measures is free: trunk rotation only with a torso
  node; each arm's chain down to its deepest node (so a forearm node with no
  upper-arm node still has a chain above it to absorb its orientation);
  everything else is locked. Joint angles are reported only where metrics.py's
  body model says both segments are measured — the model does not recover
  what is not measured.

Setup (one time)
----------------
    pip install opensim                 # official Stanford wheels, Python 3.11–3.13
    # thoracoscapular (ships in the OpenSim source repo, Apache-2.0):
    git clone --depth 1 --filter=blob:none --sparse \\
        https://github.com/opensim-org/opensim-core
    (cd opensim-core && git sparse-checkout set --no-cone \\
        OpenSim/Tests/shared/ThoracoscapularShoulderModel.osim)
    # rajagopal:
    git clone --depth 1 --filter=blob:none --sparse \\
        https://github.com/opensim-org/opensim-models
    (cd opensim-models && git sparse-checkout set Models/Rajagopal_OpenSense)

Usage
-----
    python tools/opensense_ik.py run aligned.csv montage.json \\
        --calibration calibration.json --model ThoracoscapularShoulderModel.osim \\
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

# ---------------------------------------------------------------------------
# Model profiles — how each supported OpenSim model maps onto our montage
# ---------------------------------------------------------------------------
# segment_body : our segment -> the model body its node is placed on. The
#                forearm node goes on the RADIUS (it carries pro/supination).
# trunk        : coordinates the torso node drives (locked upright without it)
# chains       : per arm, (segment, coordinates between it and the segment
#                above) from the shoulder down
# ranges       : widened / adjusted coordinate ranges, degrees
# neutral      : coordinate values (degrees) of our N-pose per forearm option
#                — the model's default pose is set to it, because IMUPlacer
#                assumes the neutral hold IS the default pose
PROFILES = {
    # Rajagopal et al. 2016 (IEEE TBME 63(10)) — full body, both arms. A gait
    # model: its arm is a simplified chain (ball-joint shoulder, no scapula).
    "rajagopal": {
        "title": "Rajagopal 2016 full-body model (OpenSense variant)",
        "segment_body": {
            "torso": "torso",
            "upper_arm_r": "humerus_r", "forearm_r": "radius_r", "hand_r": "hand_r",
            "upper_arm_l": "humerus_l", "forearm_l": "radius_l", "hand_l": "hand_l"},
        "trunk": ("pelvis_tilt", "pelvis_list", "pelvis_rotation"),
        "chains": {side: [(f"upper_arm_{side}",
                           tuple(f"{c}_{side}" for c in ("arm_flex", "arm_add", "arm_rot"))),
                          (f"forearm_{side}",
                           (f"elbow_flex_{side}", f"pro_sup_{side}")),
                          (f"hand_{side}",
                           (f"wrist_flex_{side}", f"wrist_dev_{side}"))]
                   for side in ("r", "l")},
        # shoulder flexion is capped at 90° and pro/sup stops at thumb-forward
        # in the shipped model; both are widened for rehab motion
        "ranges": {f"{c}_{side}": r for side in ("r", "l") for c, r in {
            "arm_flex": (-90, 180), "arm_add": (-180, 90), "arm_rot": (-180, 180),
            "elbow_flex": (-15, 160), "pro_sup": (-10, 190),
            "wrist_flex": (-90, 90), "wrist_dev": (-40, 50)}.items()},
        # pro_sup 0 = palms forward (full supination); 90 = palms to thighs
        "neutral": {"palms-in": {"pro_sup_r": 90.0, "pro_sup_l": 90.0},
                    "palms-forward": {"pro_sup_r": 0.0, "pro_sup_l": 0.0}},
        "elbow_coords": {"r": ("elbow_flex_r", "pro_sup_r"),
                         "l": ("elbow_flex_l", "pro_sup_l")},
    },
    # Thoracoscapular Shoulder Model — Seth, Dong, Matias & Delp 2019 (Front
    # Neurorobot 13:90), scapulothoracic joint from Seth et al. 2016 (PLoS ONE
    # 11(1)). Upper-limb model, RIGHT arm only; ISB-style glenohumeral
    # coordinates (plane of elevation, elevation, axial rotation); wrist welded.
    # No node measures the clavicle or scapula, so they are held at the
    # model's resting posture and the glenohumeral joint carries all
    # humerothoracic motion (elevation widened to 180° to allow for that).
    "thoracoscapular": {
        "title": "Thoracoscapular Shoulder Model (Seth et al. 2019)",
        "segment_body": {"torso": "thorax", "upper_arm_r": "humerus",
                         "forearm_r": "radius", "hand_r": "hand"},
        "trunk": ("ground_thorax_rot_x", "ground_thorax_rot_y",
                  "ground_thorax_rot_z"),
        "chains": {"r": [("upper_arm_r", ("plane_elv", "shoulder_elv", "axial_rot")),
                         ("forearm_r", ("elbow_flexion", "pro_sup")),
                         ("hand_r", ())]},           # wrist welded: not solvable
        # Plane of elevation and axial rotation are angles about axes that
        # turn with the arm: near the arm-at-side pole only their sum is
        # defined, and a real movement can carry either past ±180°. Clamped,
        # the solve cannot wrap and sticks at the bound — so they are left
        # UNCLAMPED (the orientation is what downstream uses). Elevation starts
        # at 0°: a negative elevation is the same pose as its mirror branch
        # (plane+180°, -elevation, axial+180°) and lets the solve flip branches.
        "ranges": {"shoulder_elv": (0, 180), "elbow_flexion": (-15, 160),
                   "pro_sup": (-100, 100)},
        "unclamped": ("plane_elv", "axial_rot"),
        # upright thorax; humerus hanging vertical and facing forward (the
        # resting scapula tilts the glenoid, so that takes 6° of glenohumeral
        # elevation — solved numerically, 0.3° residual); elbow straight. The
        # model's forearm
        # zero is already the neutral (thumb-forward) forearm: checked from the
        # bone geometry (the distal radius is anterior at 0, lateral at -90).
        "neutral": {
            "palms-in": {"ground_thorax_rot_x": 0.0, "ground_thorax_rot_y": 0.0,
                         "ground_thorax_rot_z": 0.0, "plane_elv": -69.5,
                         "shoulder_elv": 6.2, "axial_rot": 53.7,
                         "elbow_flexion": 0.0, "pro_sup": 0.0},
            "palms-forward": {"ground_thorax_rot_x": 0.0, "ground_thorax_rot_y": 0.0,
                              "ground_thorax_rot_z": 0.0, "plane_elv": -69.5,
                              "shoulder_elv": 6.2, "axial_rot": 53.7,
                              "elbow_flexion": 0.0, "pro_sup": -90.0}},
        "elbow_coords": {"r": ("elbow_flexion", "pro_sup")},
    },
}
DEFAULT_PROFILE = "rajagopal"


def detect_profile(model_path):
    """Pick the profile from the model's bodies (no OpenSim needed: the body
    names are plain XML)."""
    with open(model_path, encoding="utf-8", errors="ignore") as f:
        txt = f.read()
    if '<Body name="scapula"' in txt and '<Body name="thorax"' in txt:
        return "thoracoscapular"
    if '<Body name="humerus_r"' in txt:
        return "rajagopal"
    raise SystemExit(f"[opensense] {os.path.basename(model_path)}: not a supported "
                     f"model (supported: {', '.join(PROFILES)})")


def check_montage(profile, present_segments):
    """The model must have a body for every node the montage places."""
    missing = sorted(set(present_segments) - set(profile["segment_body"]))
    if missing:
        raise SystemExit(f"[opensense] the {profile['title']} has no body for "
                         f"{', '.join(missing)} — use a model that covers them "
                         f"(e.g. rajagopal for the left arm)")


# ---------------------------------------------------------------------------
# Model plan — which coordinates the montage can drive (pure logic, testable)
# ---------------------------------------------------------------------------
def plan_free_coords(present_segments, profile=None):
    """Coordinates left free for IK; every other coordinate is locked.

    Trunk: only with a torso node (otherwise it is held upright). Each arm:
    free from the shoulder down to the DEEPEST node on that side — the chain
    above the most proximal node must stay free to absorb its orientation, and
    nothing below the deepest node is observable."""
    profile = profile or PROFILES[DEFAULT_PROFILE]
    free = set()
    if "torso" in present_segments:
        free.update(profile["trunk"])
    for chain in profile["chains"].values():
        depth = max((i for i, (seg, _) in enumerate(chain)
                     if seg in present_segments), default=-1)
        for _, coords in chain[:depth + 1]:
            free.update(coords)
    return free


def prepare_model(model_in, model_out, present_segments, forearm_neutral,
                  profile):
    import opensim as osim
    osim.Logger.setLevelString("error")   # the model's meshes are not needed
    m = osim.Model(model_in)
    cs = m.getCoordinateSet()
    free = plan_free_coords(present_segments, profile)
    for name, val in profile["neutral"][forearm_neutral].items():
        cs.get(name).setDefaultValue(math.radians(val))
    for i in range(cs.getSize()):
        c = cs.get(i)
        c.set_locked(c.getName() not in free)
        # enforce the ranges during IK (some models ship with clamping off,
        # letting a solve wander past 360° or into a hyperextended branch)
        c.set_clamped(c.getName() in free and
                      c.getName() not in profile.get("unclamped", ()))
        if c.getName() in profile["ranges"]:
            lo, hi = profile["ranges"][c.getName()]
            c.setRangeMin(math.radians(lo))
            c.setRangeMax(math.radians(hi))
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


def to_model_frame(seg_quats, q_wa, segment_body):
    """World-from-sensor -> model-ground-from-sensor. The model's ground axes
    are the anatomical axes (X anterior, Y up, Z right), so this is conj(q_WA)."""
    qc = qconj(q_wa)
    return {segment_body[s]: qnorm(qmul(qc, q)) for s, q in seg_quats.items()}


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
        forearm_neutral="palms-in", render=True, profile_name=None):
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

    profile_name = profile_name or detect_profile(model_path)
    profile = PROFILES[profile_name]
    t_ms, seg_quats, seg_meta = load_aligned(aligned_csv, montage)
    t_ms, seg_quats, _ = trim_to_analysis(t_ms, seg_quats, calibration)
    present = set(seg_quats)
    check_montage(profile, present)
    body = profile["segment_body"]
    cols = to_model_frame(seg_quats, q_wa, body)
    rate = 1000.0 / float(np.median(np.diff(t_ms)))
    t_s = (t_ms - t_ms[0]) / 1000.0
    cal_sto = os.path.join(outdir, "neutral.sto")
    motion_sto = os.path.join(outdir, "orientations.sto")
    write_sto(cal_sto, [0.0], neutral_average(cols, t_ms, *nw), rate)
    write_sto(motion_sto, t_s, cols, rate)

    model_prep = os.path.join(outdir, "prepared_model.osim")
    free = prepare_model(model_path, model_prep, present, forearm_neutral,
                         profile)
    print(f"[opensense] {profile['title']}: {len(present)} node(s) on "
          f"{', '.join(body[s] for s in sorted(present))}; "
          f"free coordinates: {', '.join(sorted(free))}")
    t_start = max(0.0, (nw[0] - t_ms[0]) / 1000.0)
    calibrated, mot, err, secs = solve(model_prep, cal_sto, motion_sto, outdir,
                                       t_start, float(t_s[-1]))
    print(f"[opensense] IK solved {len(t_s)} frames in {secs:.0f} s -> {mot}")

    bodies = [body[s] for s in present]
    t_ik, q_body, coords = body_orientations(calibrated, mot, bodies)
    # model ground -> world axes: q_w = q_WA ⊗ q_g ⊗ conj(q_WA)
    seg_q = {s: qnorm(qmul(qmul(q_wa, q_body[body[s]]), qconj(q_wa)))
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
        "profile": profile_name,
        "model_title": profile["title"],
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

    raj, tsm = PROFILES["rajagopal"], PROFILES["thoracoscapular"]
    full_r = plan_free_coords({"torso", "upper_arm_r", "forearm_r", "hand_r"}, raj)
    want = set(raj["trunk"]) | {c for _, g in raj["chains"]["r"] for c in g}
    check(full_r == want,
          "rajagopal: full right arm frees trunk + the whole right chain only")
    no_torso = plan_free_coords({"upper_arm_r", "forearm_r"}, raj)
    check(not (no_torso & set(raj["trunk"])) and "elbow_flex_r" in no_torso
          and "wrist_flex_r" not in no_torso,
          "no torso node -> trunk locked upright; nothing below the deepest node")
    distal = plan_free_coords({"forearm_r", "hand_r"}, raj)
    check({"arm_flex_r", "elbow_flex_r", "wrist_flex_r"} <= distal,
          "forearm+hand only -> chain above the forearm stays free to absorb it")
    check(plan_free_coords(set(), raj) == set(), "no nodes -> everything locked")
    t_ua = plan_free_coords({"torso", "upper_arm_r"}, tsm)
    check(t_ua == set(tsm["trunk"]) | {"plane_elv", "shoulder_elv", "axial_rot"}
          and not any(c.startswith(("clav", "scapula")) for c in t_ua),
          "thoracoscapular: torso+upper arm frees thorax + glenohumeral only "
          "(clavicle/scapula held)")
    try:
        check_montage(tsm, {"upper_arm_l"})
        check(False, "thoracoscapular must reject a left-arm node")
    except SystemExit:
        check(True, "thoracoscapular (right arm only) rejects a left-arm node")

    # frames: a sensor aligned with the anatomical axes lands on model identity
    from calibrate_segments import anatomical_frame_quat
    q_wa = anatomical_frame_quat(-45.0)
    cols = to_model_frame({"forearm_r": q_wa[None, :]}, q_wa, raj["segment_body"])
    check(np.allclose(np.abs(cols["radius_r"][0]), IDENTITY, atol=1e-9),
          "world->model frame: an anatomically aligned sensor reads identity")

    tmp = tempfile.mkdtemp(prefix="hulc_osense_")
    for body, want_p in (('<Body name="thorax">\n<Body name="scapula">',
                          "thoracoscapular"),
                         ('<Body name="pelvis">\n<Body name="humerus_r">',
                          "rajagopal")):
        mp = os.path.join(tmp, f"{want_p}.osim")
        with open(mp, "w") as f:
            f.write(body)
        check(detect_profile(mp) == want_p, f"model detected as {want_p}")
    p = os.path.join(tmp, "x.sto")
    write_sto(p, [0.0, 0.1], {"radius_r": np.array([IDENTITY, IDENTITY])}, 10.0)
    with open(p) as f:
        txt = f.read()
    check("DataType=Quaternion" in txt and "radius_r_imu" in txt
          and "1.00000000,0.00000000" in txt, "OpenSense .sto layout")

    print(f"\n[selftest] {'PASS' if ok else 'FAIL'} — model plans for both profiles "
          f"(what each montage frees), profile detection, world->model frame, .sto "
          f"export. The solve itself needs "
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
    pr.add_argument("--profile", choices=sorted(PROFILES),
                    help="model profile (default: detected from the model)")
    pr.add_argument("--forearm-neutral", choices=["palms-forward", "palms-in"],
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
            args.outdir, args.forearm_neutral, render=not args.no_render,
            profile_name=args.profile)
        return
    ap.error("choose a command: run | selftest")


if __name__ == "__main__":
    main()
