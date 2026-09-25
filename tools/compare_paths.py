#!/usr/bin/env python3
"""
HULC Motion Shirt — which pose solver is more accurate? Default chain vs OpenSense.

Synthetic ground truth, no hardware. A protocol-shaped session (neutral hold,
shoulder swing, curls, pro/supination with abduction, full swing, rest) is
played on the Rajagopal OpenSim model; virtual nodes read its body
orientations. Both paths then process the SAME node streams:

  default    calibrate_segments (auto neutral window + facing from torso or
             elbow hinge) -> per-joint chain (metrics.py)
  opensense  the same calibration -> opensense_ik.py (IMUPlacer + IMU IK) ->
             metrics.py on the solved skeleton

Truth goes through the same metrics.py angle definitions (ISB sequences, from
neutral), so every error is a solver error, not a convention mismatch.

Conditions
  clean      60 Hz, ~1° RMS slow sensor error per node
  realistic  8.3 Hz (today's firmware), ~3° RMS slow error per strap (sensor
             + soft-tissue wobble), 50 ms clock offset on the distal nodes
Montages
  full_right  torso + upper arm + forearm + hand (facing from the torso)
  ua_fa       upper arm + forearm, no torso (facing from the elbow hinge)

Note the bias: truth comes from the same model OpenSense fits, so its joint
axes match the model exactly — a real arm will not. The soft-tissue wobble
is the stand-in for that mismatch in the realistic condition.

Needs OpenSim + the model (see opensense_ik.py):
    python tools/compare_paths.py --model Rajagopal2015_opensense.osim [--out cmp]
"""

import argparse
import json
import math
import os
import sys
import tempfile

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from calibrate_segments import (  # noqa: E402
    qmul, qconj, qnorm, build_calibration, choose_neutral_window,
)
from metrics import (  # noqa: E402
    apply_calibration, compute_joint_metrics, resolve_anatomical_frame,
    trim_to_analysis,
)
from motion_capabilities import JOINTS  # noqa: E402
import opensense_ik  # noqa: E402

IDENT = np.array([1.0, 0.0, 0.0, 0.0])
FINE_HZ = 240.0
DUR_S = 40.0
FACING_YAW_DEG = 37.0          # subject's forward, CCW from world +X


def qaxis(axis, deg):
    a = np.radians(deg) / 2.0
    v = np.asarray(axis, float); v = v / np.linalg.norm(v)
    return np.r_[math.cos(a), math.sin(a) * v]


def rotvec_q(v):
    """Quaternions from rotation vectors (N,3) in radians."""
    ang = np.linalg.norm(v, axis=1)
    ax = v / np.where(ang[:, None] > 1e-12, ang[:, None], 1.0)
    return np.column_stack([np.cos(ang / 2), np.sin(ang / 2)[:, None] * ax])


# ---------------------------------------------------------------------------
# Ground-truth motion (model coordinates, degrees) — protocol shaped
# ---------------------------------------------------------------------------
def env(t, a, b, ramp=1.0):
    """Smooth 0→1→0 envelope over [a, b]."""
    up = np.clip((t - a) / ramp, 0, 1); dn = np.clip((b - t) / ramp, 0, 1)
    e = np.minimum(up, dn)
    return 0.5 - 0.5 * np.cos(np.pi * e)


def truth_coords(t):
    c = {k: np.zeros_like(t) for k in (
        "pelvis_tilt", "pelvis_list", "pelvis_rotation", "arm_flex_r",
        "arm_add_r", "arm_rot_r", "elbow_flex_r", "wrist_flex_r", "wrist_dev_r")}
    c["pro_sup_r"] = np.full_like(t, 90.0)                  # palms-in neutral
    sway = env(t, 3, 38)
    c["pelvis_tilt"] = sway * 6 * np.sin(2 * np.pi * t / 7.3)
    c["pelvis_list"] = sway * 4 * np.sin(2 * np.pi * t / 5.1 + 1)
    c["pelvis_rotation"] = sway * 8 * np.sin(2 * np.pi * t / 9.7)
    s1 = env(t, 3, 11)                                       # shoulder swing
    c["arm_flex_r"] += s1 * 60 * (1 - np.cos(2 * np.pi * (t - 3) / 4.0))
    c["arm_add_r"] += s1 * -20 * (1 - np.cos(2 * np.pi * (t - 3) / 4.0))
    s2 = env(t, 11, 21)                                      # curls, supinated
    c["elbow_flex_r"] += s2 * 65 * (1 - np.cos(2 * np.pi * (t - 11) / 2.5))
    c["pro_sup_r"] -= s2 * 60
    s3 = env(t, 21, 29)                                      # pro/sup + abduction
    c["arm_add_r"] += s3 * -65
    c["elbow_flex_r"] += s3 * 80
    c["pro_sup_r"] += s3 * 70 * np.sin(2 * np.pi * (t - 21) / 2.7)
    s4 = env(t, 29, 37)                                      # full swing
    c["arm_flex_r"] += s4 * (55 + 90 * np.sin(2 * np.pi * (t - 29) / 4.0 - 1.2))
    c["arm_rot_r"] += s4 * 35 * np.sin(2 * np.pi * (t - 29) / 3.1)
    w = env(t, 3, 37)
    c["wrist_flex_r"] = w * 30 * np.sin(2 * np.pi * t / 3.3)
    c["wrist_dev_r"] = w * (5 + 12 * np.sin(2 * np.pi * t / 2.9 + 0.5))
    return c


def body_truth(model_path, t, coords, bodies):
    """Ground-frame body orientations: absolute, and as rotation from neutral."""
    import opensim as osim
    osim.Logger.setLevelString("error")
    m = osim.Model(model_path)
    cs = m.getCoordinateSet()
    for i in range(cs.getSize()):
        cs.get(i).set_locked(False)
        cs.get(i).set_clamped(False)
    s = m.initSystem()
    bs = m.getBodySet()

    def q_of(b):
        q = bs.get(b).getTransformInGround(s).R().convertRotationToQuaternion()
        return np.array([q.get(i) for i in range(4)])

    def pose(k):
        for n, v in coords.items():
            cs.get(n).setValue(s, math.radians(v[k]), False)
        m.realizePosition(s)

    absq = {b: np.zeros((len(t), 4)) for b in bodies}
    for k in range(len(t)):
        pose(k)
        for b in bodies:
            absq[b][k] = q_of(b)
    rel = {b: qnorm(qmul(absq[b], qconj(absq[b][0]))) for b in bodies}
    return absq, rel


# ---------------------------------------------------------------------------
# Virtual nodes
# ---------------------------------------------------------------------------
def node_streams(absq, fine_t, grid_t, cond, rng):
    """World-from-sensor quaternions on the logging grid for each body."""
    to_zup = qaxis([1, 0, 0], 90)                 # model Y-up ground -> Z-up world
    yaw = qaxis([0, 0, 1], FACING_YAW_DEG)
    world = qmul(yaw, to_zup)
    out = {}
    for b, q in absq.items():
        if b == "torso":        # sternum node: +z out of the chest, slight tilt
            mount = qmul(qaxis([0, 1, 0], 90), qaxis(rng.normal(size=3), 5))
        else:
            mount = qnorm(rng.normal(size=4))
        delay = 0.05 if (cond == "realistic" and b not in ("torso", "humerus_r")) else 0.0
        idx = np.clip(np.round((grid_t - delay) * FINE_HZ).astype(int), 0, len(fine_t) - 1)
        qs = qmul(world, qmul(q[idx], mount))
        n = len(grid_t)
        # fused-sensor error is SLOW (drift, strap wobble), not white: a smooth
        # random rotation of ~1° RMS (clean) or ~3° RMS (realistic: + soft
        # tissue), plus a 0.1° per-sample jitter
        rms = np.radians(3.0 if cond == "realistic" else 1.0)
        v = np.zeros((n, 3))
        for _ in range(3):
            f = rng.uniform(0.15, 0.8)
            v += np.outer(np.sin(2 * np.pi * f * grid_t + rng.uniform(0, 6.3)),
                          rng.normal(0, rms / math.sqrt(1.5), 3))
        qs = qmul(qs, rotvec_q(v))
        qs = qmul(qs, rotvec_q(rng.normal(0, np.radians(0.1) / math.sqrt(3), (n, 3))))
        out[b] = qnorm(qs)
    return out


# ---------------------------------------------------------------------------
# Angle series for a set of segment streams (metrics.py's joint chain)
# ---------------------------------------------------------------------------
def joint_series(t_ms, q_seg, q_wa, present):
    out = {}
    for jk, j in JOINTS.items():
        if j.proximal in present and j.distal in present:
            _, ser = compute_joint_metrics(jk, j, q_seg[j.proximal], q_seg[j.distal],
                                           t_ms, True, q_wa=q_wa)
            out[jk] = ser
    return out


def wrapped_err(a, b):
    e = (a - b + 180.0) % 360.0 - 180.0
    return e[np.isfinite(e)]


TRUTH_BODY = opensense_ik.PROFILES["rajagopal"]["segment_body"]


def run_case(model, truth_model, montage_name, segs, cond, outdir, seed):
    """Truth is always played on the Rajagopal model; `model` is the solver's
    model (the same one, or another — then the solver's joints differ from the
    'subject's', as a real arm's would)."""
    rng = np.random.default_rng(seed)
    fine_t = np.arange(0, DUR_S, 1 / FINE_HZ)
    coords = truth_coords(fine_t)
    bodies = [TRUTH_BODY[s] for s in segs]
    absq, rel = body_truth(truth_model, fine_t, coords, bodies)
    fs = 60.0 if cond == "clean" else 8.33
    grid = np.arange(0, DUR_S, 1 / fs)
    nodes = node_streams(absq, fine_t, grid, cond, rng)
    t_ms = grid * 1000.0

    seg_quats = {s: nodes[TRUTH_BODY[s]] for s in segs}
    montage = {"schema_version": "1.0", "subject": {"id": "SYN"},
               "session": {"id": f"{montage_name}_{cond}"},
               "calibration": {"neutral_pose": "N-pose", "captured": True},
               "nodes": [{"node_id": f"N{i}", "column": f"n{i}", "segment": s,
                          "calibrated": True} for i, s in enumerate(segs)]}
    meta = {s: {"column": f"n{i}", "node_id": f"N{i}"} for i, s in enumerate(segs)}
    d = os.path.join(outdir, f"{montage_name}_{cond}")
    os.makedirs(d, exist_ok=True)
    aligned = os.path.join(d, "aligned.csv")
    opensense_ik.write_aligned(aligned, t_ms, seg_quats, montage)
    mpath = os.path.join(d, "montage.json")
    with open(mpath, "w") as f:
        json.dump(montage, f)

    # ---- default path ----
    t0, t1, msg = choose_neutral_window(montage, t_ms, seg_quats)
    cal = build_calibration(montage, t_ms, seg_quats, meta, t0, t1, aligned)
    cpath = os.path.join(d, "calibration.json")
    with open(cpath, "w") as f:
        json.dump(cal, f, indent=2)
    q_wa = resolve_anatomical_frame(cal)
    qs, _ = apply_calibration(seg_quats, cal)
    tt, qs, _ = trim_to_analysis(t_ms, qs, cal)
    ours = joint_series(tt, qs, q_wa, set(segs))

    # ---- OpenSense path (same calibration) ----
    osd = os.path.join(d, "opensense")
    opensense_ik.run(aligned, mpath, cpath, model, osd, render=False)
    from calibrate_segments import load_aligned
    to, qo, _ = load_aligned(os.path.join(osd, "opensense_aligned.csv"), montage)
    with open(os.path.join(osd, "opensense_calibration.json")) as f:
        cal_o = json.load(f)
    qo, _ = apply_calibration(qo, cal_o)
    osr = joint_series(to, qo, resolve_anatomical_frame(cal_o), set(segs))

    # ---- truth, on each path's own time grid (ground axes = anatomical) ----
    def truth_on(times_ms):
        idx = np.clip(np.round(times_ms / 1000.0 * FINE_HZ).astype(int), 0,
                      len(fine_t) - 1)
        return joint_series(times_ms, {s: rel[TRUTH_BODY[s]][idx]
                                       for s in segs}, IDENT, set(segs))
    tr_ours, tr_os = truth_on(tt), truth_on(to)

    h = cal["heading"]
    want_facing = 90.0 - FACING_YAW_DEG
    res = {"montage": montage_name, "condition": cond, "neutral_window": [t0, t1],
           "facing": {"source": h["source"], "confident": h.get("confident"),
                      "error_deg": (round(abs((h["facing_deg"] - want_facing + 180)
                                              % 360 - 180), 2)
                                    if h.get("facing_deg") is not None else None)},
           "dofs": {}}
    for jk in ours:
        for dk in ours[jk]:
            e1 = wrapped_err(ours[jk][dk], tr_ours[jk][dk])
            e2 = wrapped_err(osr[jk][dk], tr_os[jk][dk])
            if not (e1.size and e2.size):
                continue                        # undefined all session (a pole)
            res["dofs"][f"{jk}.{dk}"] = {
                "default": {"rms": round(float(np.sqrt(np.mean(e1 ** 2))), 2),
                            "p95": round(float(np.percentile(np.abs(e1), 95)), 2)},
                "opensense": {"rms": round(float(np.sqrt(np.mean(e2 ** 2))), 2),
                              "p95": round(float(np.percentile(np.abs(e2), 95)), 2)}}
    # Elbow also scored against the model's own joint COORDINATES — independent
    # of any frame choice (the model's elbow axis is ~13° off the trunk's
    # left-right axis, so a trunk-frame "truth" split leaks flexion into pro/sup).
    if "elbow_r" in ours:
        import opensim as osim
        prof = opensense_ik.PROFILES[opensense_ik.detect_profile(model)]
        c_flex, c_ps = prof["elbow_coords"]["r"]
        ps0 = prof["neutral"]["palms-in"].get(c_ps, 0.0)       # solver's neutral
        mot = osim.TimeSeriesTable(os.path.join(osd, "ik_orientations.mot"))
        t_os = np.asarray(mot.getIndependentColumn()) + tt[0] / 1000.0
        tc, tco = truth_coords(tt / 1000.0), truth_coords(t_os)
        neutral_ps = 90.0                   # truth (Rajagopal) palms-in pro_sup

        def rms(e):
            e = e[np.isfinite(e)]
            return round(float(np.sqrt(np.mean(e ** 2))), 2)
        res["elbow_vs_model_coords"] = {
            "default": {"flex_ext": rms(ours["elbow_r"]["flex_ext"] - tc["elbow_flex_r"]),
                        "pro_sup": rms(ours["elbow_r"]["pro_sup"]
                                       - (tc["pro_sup_r"] - neutral_ps))},
            "opensense": {"flex_ext": rms(mot.getDependentColumn(c_flex)
                                          .to_numpy() - tco["elbow_flex_r"]),
                          "pro_sup": rms(mot.getDependentColumn(c_ps).to_numpy()
                                         - ps0 - (tco["pro_sup_r"] - neutral_ps))}}
    return res


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[2])
    ap.add_argument("--model", required=True,
                    help="the solver's model (Rajagopal or Thoracoscapular)")
    ap.add_argument("--truth-model",
                    help="Rajagopal2015_opensense.osim to play the truth on "
                         "(default: --model, which must then be Rajagopal)")
    ap.add_argument("--out", default=os.path.join(tempfile.gettempdir(), "hulc_cmp"))
    ap.add_argument("--only", nargs="*", help="case prefixes, e.g. ua_fa_realistic")
    args = ap.parse_args()
    truth_model = args.truth_model or args.model
    prof = opensense_ik.PROFILES[opensense_ik.detect_profile(args.model)]
    full = [s for s in ("torso", "upper_arm_r", "forearm_r", "hand_r")
            if s in prof["segment_body"] and (s != "hand_r" or prof["chains"]["r"][2][1])]
    cases = [("full_right", full),
             ("ua_fa", ["upper_arm_r", "forearm_r"]),
             ("torso_ua", ["torso", "upper_arm_r"])]
    results = []
    for seed, (name, segs) in enumerate(cases):
        for cond in ("clean", "realistic"):
            tag = f"{name}_{cond}"
            if args.only and not any(tag.startswith(o) for o in args.only):
                continue
            r = run_case(args.model, truth_model, name, segs, cond, args.out, seed)
            results.append(r)
            print(f"\n=== {tag}: neutral {r['neutral_window'][0]:.0f}–"
                  f"{r['neutral_window'][1]:.0f} ms, facing {r['facing']}")
            print(f"  {'DOF':<22} {'default rms/p95':>16} {'opensense rms/p95':>18}")
            for k, v in r["dofs"].items():
                print(f"  {k:<22} {v['default']['rms']:7.1f} /{v['default']['p95']:6.1f}"
                      f"   {v['opensense']['rms']:7.1f} /{v['opensense']['p95']:6.1f}")
            ec = r.get("elbow_vs_model_coords")
            if ec:
                print(f"  elbow vs model coordinates (RMS): default flex "
                      f"{ec['default']['flex_ext']:.1f} pro/sup {ec['default']['pro_sup']:.1f}"
                      f" | opensense flex {ec['opensense']['flex_ext']:.1f} pro/sup "
                      f"{ec['opensense']['pro_sup']:.1f}")
    with open(os.path.join(args.out, "compare_results.json"), "w") as f:
        json.dump(results, f, indent=1)
    print(f"\n[compare] wrote {os.path.join(args.out, 'compare_results.json')}")


if __name__ == "__main__":
    main()
