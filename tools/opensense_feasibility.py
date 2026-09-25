#!/usr/bin/env python3
"""
HULC Motion Shirt — OpenSense feasibility + partial-montage evaluation.

Synthetic ground truth, no hardware: drive the Rajagopal2015 OpenSense model
through a known upper-body motion, read off the world orientation of virtual
IMUs (random strap mounting, Z-up sensor world, subject facing 37° off north,
~1° noise), write OpenSense .sto files, then run IMUPlacer +
IMUInverseKinematicsTool for several node subsets and compare the recovered
joint coordinates against truth. Findings: OPENSENSE_FEASIBILITY.md.

This is an evaluation harness, not part of the pipeline. Needs:
    pip install opensim numpy        # official Stanford wheels, Python 3.11-3.13
    git clone --depth 1 https://github.com/opensim-org/opensim-models
Usage:
    python tools/opensense_feasibility.py --model \
        opensim-models/Models/Rajagopal_OpenSense/Rajagopal2015_opensense.osim \
        [--out os_eval] [--default-ranges] [CASE_PREFIX ...]
"""
import argparse
import json
import math
import os
import sys
import time
import numpy as np
import opensim as osim

osim.Logger.setLevelString("error")
_ap = argparse.ArgumentParser(description=__doc__.split("\n")[2])
_ap.add_argument("--model", required=True, help="Rajagopal2015_opensense.osim")
_ap.add_argument("--out", default="os_eval", help="results directory")
_ap.add_argument("--default-ranges", action="store_true",
                 help="keep the model's shipped shoulder ranges (flexion capped at 90°)")
_ap.add_argument("cases", nargs="*", help="only run cases whose name starts with these")
ARGS = _ap.parse_args()
MODEL = ARGS.model
OUT = ARGS.out; os.makedirs(OUT, exist_ok=True)
FS, DUR, T_NEUTRAL = 60.0, 20.0, 1.0
YAW_DEG = 37.0            # subject's facing vs magnetic north (unknown to the solver)
NOISE_DEG = 1.0
WIDE = not ARGS.default_ranges
rng = np.random.default_rng(7)

# ---------------- quaternion helpers (w,x,y,z) ----------------
def qmul(a, b):
    w1, x1, y1, z1 = a; w2, x2, y2, z2 = b
    return np.array([w1*w2-x1*x2-y1*y2-z1*z2, w1*x2+x1*w2+y1*z2-z1*y2,
                     w1*y2-x1*z2+y1*w2+z1*x2, w1*z2+x1*y2-y1*x2+z1*w2])
def qaxis(ax, deg):
    a = math.radians(deg) / 2; v = np.asarray(ax, float); v = v / np.linalg.norm(v)
    return np.r_[math.cos(a), math.sin(a) * v]
def rand_q():
    q = rng.normal(size=4); return q / np.linalg.norm(q)

# ---------------- model prep ----------------
LEG = ["hip_flexion", "hip_adduction", "hip_rotation", "knee_angle", "ankle_angle",
       "subtalar_angle", "mtp_angle"]
ARM = ["arm_flex", "arm_add", "arm_rot", "elbow_flex", "pro_sup", "wrist_flex", "wrist_dev"]
TRUNK = ["pelvis_tilt", "pelvis_list", "pelvis_rotation"]

def prep_model(locked=(), wide=WIDE):
    m = osim.Model(MODEL)
    cs = m.getCoordinateSet()
    for i in range(cs.getSize()):
        c = cs.get(i); n = c.getName()
        if any(n.startswith(p) for p in LEG) or n in ("pelvis_tx", "pelvis_ty", "pelvis_tz") \
                or n.startswith("lumbar") or n.startswith("knee_angle"):
            c.set_locked(True)
        elif any(n.startswith(p) for p in ARM) or n in TRUNK:
            c.set_locked(n in locked)
    if wide:
        for side in "rl":
            for n, lo, hi in [("arm_flex", -90, 180), ("arm_add", -180, 90), ("arm_rot", -90, 90)]:
                c = cs.get(f"{n}_{side}"); c.setRangeMin(math.radians(lo)); c.setRangeMax(math.radians(hi))
    return m

# ---------------- ground-truth motion (degrees) ----------------
t = np.arange(0, DUR, 1 / FS)
ramp = np.clip((t - T_NEUTRAL) / 1.0, 0, 1)          # hold neutral, then ease in
def osc(amp, per, off=0.0, ph=0.0):
    return ramp * (off + amp * np.sin(2 * np.pi * t / per + ph))
truth = {
    "pelvis_tilt": osc(12, 7.1), "pelvis_list": osc(8, 5.3, ph=1), "pelvis_rotation": osc(18, 9.7),
    "arm_flex_r": osc(70, 6.0, off=70), "arm_add_r": osc(35, 4.3, off=-35), "arm_rot_r": osc(35, 3.7),
    "elbow_flex_r": osc(60, 3.1, off=65), "pro_sup_r": osc(38, 2.6, off=45),
    "wrist_flex_r": osc(45, 2.2), "wrist_dev_r": osc(18, 1.9, off=4),
    "arm_flex_l": osc(55, 6.7, off=55, ph=2), "arm_add_l": osc(30, 5.1, off=-30, ph=1), "arm_rot_l": osc(30, 4.1, ph=2),
    "elbow_flex_l": osc(55, 3.5, off=60, ph=1), "pro_sup_l": osc(35, 2.9, off=45, ph=2),
    "wrist_flex_l": osc(35, 2.4, ph=1), "wrist_dev_l": osc(15, 2.1, off=3, ph=1),
}

# ---------------- virtual IMUs on a truth model ----------------
IMU_BODIES = ["torso", "humerus_r", "radius_r", "hand_r", "humerus_l", "radius_l", "hand_l"]
TO_ZUP = qaxis([1, 0, 0], 90)                          # OpenSim Y-up ground -> sensor Z-up world
YAW = qaxis([0, 0, 1], YAW_DEG)

def mount(body):
    if body == "torso":   # sternum node: sensor +z points forward (model +X), small tilt
        return qmul(qaxis([0, 1, 0], 90), qmul(qaxis(rng.normal(size=3), 5), [1, 0, 0, 0]))
    return rand_q()       # limb straps: arbitrary mounting
MOUNT = {b: mount(b) for b in IMU_BODIES}

def body_quats():
    m = prep_model(wide=True); s = m.initSystem()
    cs = m.getCoordinateSet(); bs = m.getBodySet()
    Q = {b: np.zeros((len(t), 4)) for b in IMU_BODIES}
    for k in range(len(t)):
        for n, v in truth.items():
            cs.get(n).setValue(s, math.radians(v[k]), False)
        m.realizePosition(s)
        for b in IMU_BODIES:
            q = bs.get(b).getTransformInGround(s).R().convertRotationToQuaternion()
            Q[b][k] = [q.get(i) for i in range(4)]
    return Q

def sensor_quats(QB, yaw=YAW, noise=NOISE_DEG):
    out = {}
    for b, arr in QB.items():
        rows = []
        for q in arr:
            qs = qmul(yaw, qmul(TO_ZUP, qmul(q, MOUNT[b])))
            if noise:
                qs = qmul(qs, qaxis(rng.normal(size=3), abs(rng.normal(0, noise))))
            rows.append(qs / np.linalg.norm(qs))
        out[b] = np.array(rows)
    return out

def write_sto(path, cols, times):
    with open(path, "w") as f:
        f.write(f"DataRate={FS:.6f}\nDataType=Quaternion\nversion=3\nOpenSimVersion=4.6\nendheader\n")
        f.write("time\t" + "\t".join(f"{b}_imu" for b in cols) + "\n")
        for k, tt in enumerate(times):
            f.write(f"{tt:.6f}\t" + "\t".join(",".join(f"{v:.8f}" for v in cols[b][k]) for b in cols) + "\n")

# ---------------- run one montage ----------------
def run(name, bodies, SQ, base=True, lock=(), wide=WIDE):
    d = os.path.join(OUT, name); os.makedirs(d, exist_ok=True)
    cols = {b: SQ[b] for b in bodies}
    neutral = t < T_NEUTRAL * 0.9
    cal = {b: None for b in bodies}
    for b in bodies:                                    # Markley average over the neutral hold
        A = cols[b][neutral].T @ cols[b][neutral]; q = np.linalg.eigh(A)[1][:, -1]
        cal[b] = (q if q[0] >= 0 else -q)[None, :]
    if base:   # heading from the torso node at neutral (its +z = forward); OpenSense's
               # IMUPlacer heading only affects placement, not the IK input, so
               # de-rotate the data ourselves (as calibrate_segments.compute_heading would)
        qt = cal["torso"][0]
        f = qmul(qmul(qt, [0, 0, 0, 1]), qt * [1, -1, -1, -1])[1:]
        corr = qaxis([0, 0, 1], -math.degrees(math.atan2(f[1], f[0])))
        cal = {b: qmul(corr, q[0])[None, :] for b, q in cal.items()}
        cols = {b: np.array([qmul(corr, q) for q in arr]) for b, arr in cols.items()}
    write_sto(os.path.join(d, "cal.sto"), cal, [0.0])
    write_sto(os.path.join(d, "motion.sto"), cols, t)
    m = prep_model(locked=lock, wide=wide); mp = os.path.join(d, "model.osim"); m.printToXML(mp)
    R = osim.Vec3(-math.pi / 2, 0, 0)
    p = osim.IMUPlacer(); p.set_model_file(mp); p.set_orientation_file_for_calibration(os.path.join(d, "cal.sto"))
    p.set_sensor_to_opensim_rotations(R)
    p.run(False); cm = os.path.join(d, "calibrated.osim"); p.getCalibratedModel().printToXML(cm)
    ik = osim.IMUInverseKinematicsTool(); ik.set_model_file(cm)
    ik.set_orientations_file(os.path.join(d, "motion.sto")); ik.set_sensor_to_opensim_rotations(R)
    ik.set_results_directory(d); ik.set_time_range(0, 0.0); ik.set_time_range(1, float(t[-1]))
    t0 = time.time(); ik.run(False); secs = time.time() - t0
    res = osim.TimeSeriesTable(os.path.join(d, "ik_motion.mot"))
    labels = list(res.getColumnLabels())
    err = {}
    live = t >= T_NEUTRAL
    for n in truth:
        if n not in labels: continue
        est = res.getDependentColumn(n).to_numpy()
        e = (est - truth[n])[live]
        err[n] = (float(np.sqrt(np.mean(e ** 2))), float(np.max(np.abs(e))))
    return err, secs

def fmt(err, keys):
    return "  ".join(f"{k}:{err[k][0]:5.1f}/{err[k][1]:5.1f}" for k in keys if k in err)

if __name__ == "__main__":
    QB = body_quats()
    SQ = sensor_quats(QB)
    SQ_faced = sensor_quats(QB, yaw=np.array([1, 0, 0, 0.]))   # facing known (pre-corrected)
    R_ARM = ["arm_flex_r", "arm_add_r", "arm_rot_r", "elbow_flex_r", "pro_sup_r", "wrist_flex_r", "wrist_dev_r"]
    L_ARM = [k[:-2] + "_l" for k in R_ARM]
    DIST_R = ["elbow_flex_r", "pro_sup_r", "wrist_flex_r", "wrist_dev_r"]
    cases = [
        # name, bodies, SQ, base, locked coords
        ("A_full_right", ["torso", "humerus_r", "radius_r", "hand_r"], SQ, True, L_ARM),
        ("B_torso_upperarm", ["torso", "humerus_r"], SQ, True, L_ARM + DIST_R),
        ("C_torso_forearm_noUA", ["torso", "radius_r"], SQ, True, L_ARM + ["wrist_flex_r", "wrist_dev_r"]),
        ("D_upperarm_forearm_noTorso_facingKnown", ["humerus_r", "radius_r"], SQ_faced, False, L_ARM + TRUNK + ["wrist_flex_r", "wrist_dev_r"]),
        ("D2_upperarm_forearm_noTorso_facingUnknown", ["humerus_r", "radius_r"], SQ, False, L_ARM + TRUNK + ["wrist_flex_r", "wrist_dev_r"]),
        ("E_forearm_hand_only", ["radius_r", "hand_r"], SQ_faced, False, L_ARM + TRUNK + ["arm_flex_r", "arm_add_r", "arm_rot_r"]),
        ("E2_forearm_hand_shoulderFree", ["radius_r", "hand_r"], SQ_faced, False, L_ARM + TRUNK),
        ("F_bilateral_upperarms_noTorso", ["humerus_r", "humerus_l"], SQ_faced, False, TRUNK + DIST_R + [k for k in L_ARM if k not in ("arm_flex_l", "arm_add_l", "arm_rot_l")]),
        ("G_full_bilateral", IMU_BODIES, SQ, True, ()),
    ]
    only = ARGS.cases
    results = {}
    for name, bodies, sq, base, lock in cases:
        if only and not any(name.startswith(o) for o in only): continue
        err, secs = run(name, bodies, sq, base, lock)
        results[name] = {"imus": bodies, "ik_seconds": round(secs, 1), "err_deg_rmse_max": err}
        print(f"\n== {name}  imus={bodies}  IK {secs:.1f}s for {len(t)} frames")
        print("  trunk:", fmt(err, TRUNK))
        print("  R arm:", fmt(err, R_ARM))
        if name.startswith(("F", "G")): print("  L arm:", fmt(err, L_ARM))
    tag = "wide" if WIDE else "default"
    json.dump(results, open(os.path.join(OUT, f"results_{tag}.json"), "w"), indent=1)
