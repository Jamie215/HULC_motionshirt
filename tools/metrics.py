#!/usr/bin/env python3
"""
HULC Motion Shirt — stage-6 metrics: per-DOF joint angles + range of motion.

The payoff stage. Everything upstream (capture → offload → reconcile → montage →
calibrate) exists to turn raw sensor quaternions into an anatomically-referenced
stream; this tool reads that stream and produces the clinical numbers a therapist
actually wants: **how far each joint moved** (range of motion) and the **per-DOF
angle time series** it moved through. "ROM first" (SETUP_AND_CALIBRATION_PLAN.md
§6, build step 4) — the joint-angle read-out rides along on the same
decomposition.

How a joint angle is computed (the whole chain in four lines)
-------------------------------------------------------------
    q_seg(t) = q_WS(t) ⊗ q_SB          apply the cached mounting offset (stage 5)
    q_rel(t) = conj(q_seg_prox) ⊗ q_seg_dist   distal-relative-to-proximal
    (α,β,γ)  = euler(q_rel, sequence)   decompose in the joint's ISB/Wu sequence
    angle_dof = (α|β|γ)[seq_index]      pick the slot that IS this clinical DOF

The Euler sequence and the slot each clinical DOF occupies both come from
motion_capabilities.JOINTS (the one body model) — this tool never re-declares
anatomy or invents a convention. Because calibration zeroes q_rel at the neutral
pose, every angle here is measured from anatomical zero, so ROM is clinical, not
"relative to whatever the arm happened to be doing at t=0".

Honesty (the same contract the resolver prints)
-----------------------------------------------
* It only computes a joint the montage can actually resolve — both adjacent nodes
  present — reusing motion_capabilities.resolve(). A blocked joint is reported
  blocked, with the missing node named, never a fabricated number.
* A joint whose two nodes are not BOTH anatomically calibrated is flagged
  `clinical: false` and its angles are RELATIVE-only (the mounting offset is
  identity, so zero is "pose at the neutral window" at best, not the anatomical
  landmark). Same wording as the resolver.
* Wrap-around is unwrapped before ROM so a sweep through ±180° doesn't fake a
  360° range.

Usage
-----
    # compute metrics from the aligned stream + montage + calibration:
    python tools/metrics.py compute aligned.csv montage.json \
        --calibration calibration.json --out metrics.json

    # no calibration file -> everything reported RELATIVE-only, honestly flagged:
    python tools/metrics.py compute aligned.csv montage.json

    # validate the math end-to-end (no hardware): round-trips every Euler
    # sequence and recovers a known injected ROM sweep.
    python tools/metrics.py selftest

Anatomy, the montage loader, the aligned-CSV binder, and the quaternion algebra
are all imported from the existing tools so there is exactly one source of truth.
"""

import argparse
import json
import os
import sys

try:
    import numpy as np
except ImportError:  # pragma: no cover
    raise SystemExit("This tool needs numpy:  pip install numpy")

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
# One source of truth: body model + resolver, and the quaternion/loader helpers
# already written and tested in calibrate_segments.py.
from motion_capabilities import JOINTS, resolve  # noqa: E402
from calibrate_segments import (  # noqa: E402
    qmul, qconj, qnorm, load_aligned, load_montage,
)

SCHEMA_VERSION = "1.0"

IDENTITY = np.array([1.0, 0.0, 0.0, 0.0])

# Below this |sin(middle angle)| the Euler split is gimbal-degenerate (the first
# and third axes align); we collapse them onto the still-well-defined sum so the
# recovered orientation stays exact even though the individual angles don't.
_GIMBAL_SIN = 1e-6


# ---------------------------------------------------------------------------
# Quaternion -> rotation matrix (vectorized over (...,4) -> (...,3,3))
# ---------------------------------------------------------------------------
def rotmat_from_quat(q):
    """Active rotation matrix for unit quaternion(s) q=[w,x,y,z].

    Matches the qrotate convention in calibrate_segments (v' = q ⊗ v ⊗ q*), so a
    decomposition here is consistent with how the FBD viewer rotates a segment.
    """
    q = qnorm(np.asarray(q, dtype=float))
    w, x, y, z = q[..., 0], q[..., 1], q[..., 2], q[..., 3]
    R = np.empty(q.shape[:-1] + (3, 3), dtype=float)
    R[..., 0, 0] = 1 - 2 * (y * y + z * z)
    R[..., 0, 1] = 2 * (x * y - w * z)
    R[..., 0, 2] = 2 * (x * z + w * y)
    R[..., 1, 0] = 2 * (x * y + w * z)
    R[..., 1, 1] = 1 - 2 * (x * x + z * z)
    R[..., 1, 2] = 2 * (y * z - w * x)
    R[..., 2, 0] = 2 * (x * z - w * y)
    R[..., 2, 1] = 2 * (y * z + w * x)
    R[..., 2, 2] = 1 - 2 * (x * x + y * y)
    return R


# ---------------------------------------------------------------------------
# Intrinsic Euler decomposition
#
# Angles are returned as (..., 3) in the sequence's own slot order [a1,a2,a3] for
# an intrinsic rotation R = R_{a1}(θ1) · R_{a2}(θ2) · R_{a3}(θ3). Only the two
# sequences the body model actually declares are implemented; each is derived in
# closed form from rotmat_from_quat above and round-trip-verified in selftest().
# Add a sequence here (with its closed form) when a new joint declares one.
# ---------------------------------------------------------------------------
def _euler_ZXY(R):
    """Intrinsic Z-X-Y (elbow/wrist): θ1 about Z, θ2 about X, θ3 about Y."""
    beta = np.arcsin(np.clip(R[..., 2, 1], -1.0, 1.0))          # X (middle)
    alpha = np.arctan2(-R[..., 0, 1], R[..., 1, 1])             # Z (first)
    gamma = np.arctan2(-R[..., 2, 0], R[..., 2, 2])             # Y (third)
    # Gimbal lock: cos(beta) ~ 0 -> Z and Y axes align; fold onto their sum.
    lock = np.abs(np.cos(beta)) < _GIMBAL_SIN
    if np.any(lock):
        s = np.sign(R[..., 2, 1])
        alpha = np.where(lock, s * np.arctan2(R[..., 1, 0], R[..., 0, 0]), alpha)
        gamma = np.where(lock, 0.0, gamma)
    return np.stack([alpha, beta, gamma], axis=-1)


def _euler_YXY(R):
    """Intrinsic Y-X-Y (shoulder, proper Euler): θ1,θ3 about Y, θ2 about X."""
    beta = np.arccos(np.clip(R[..., 1, 1], -1.0, 1.0))          # X (middle)
    alpha = np.arctan2(R[..., 0, 1], R[..., 2, 1])              # Y (first)
    gamma = np.arctan2(R[..., 1, 0], -R[..., 1, 2])             # Y (third)
    # Gimbal lock: sin(beta) ~ 0 -> the two Y axes align; fold onto their sum.
    lock = np.abs(np.sin(beta)) < _GIMBAL_SIN
    if np.any(lock):
        alpha = np.where(lock, np.arctan2(-R[..., 2, 0], R[..., 0, 0]), alpha)
        gamma = np.where(lock, 0.0, gamma)
    return np.stack([alpha, beta, gamma], axis=-1)


_EULER = {"ZXY": _euler_ZXY, "YXY": _euler_YXY}


def euler_from_quat(q, sequence):
    """Decompose quaternion(s) into intrinsic Euler angles (radians, (...,3))."""
    fn = _EULER.get(sequence)
    if fn is None:
        raise SystemExit(
            f"metrics: no Euler decomposition for sequence '{sequence}'. "
            f"Implemented: {', '.join(sorted(_EULER))}. Add its closed form in "
            f"metrics.py (and round-trip it in selftest) before a joint uses it.")
    return fn(rotmat_from_quat(q))


# ---------------------------------------------------------------------------
# Calibration handling — apply the cached mounting offsets
# ---------------------------------------------------------------------------
def apply_calibration(seg_quats, calibration):
    """Return ({seg: q_seg(t)}, {seg: bool calibrated}).

    q_seg = q_WS ⊗ q_SB using the cached per-segment offset. A segment with no
    cached offset (no calibration file, or absent from it) passes through with an
    identity offset and is flagged uncalibrated, so its downstream angles read
    RELATIVE-only rather than silently pretending to be anatomical.
    """
    cal_segs = (calibration or {}).get("segments", {})
    out, calibrated = {}, {}
    for seg, q in seg_quats.items():
        base = cal_segs.get(seg)
        if base and "mounting_offset_quat" in base:
            offset = qnorm(np.asarray(base["mounting_offset_quat"], dtype=float))
            out[seg] = qmul(q, offset)
            calibrated[seg] = True
        else:
            out[seg] = q
            calibrated[seg] = False
    return out, calibrated


# ---------------------------------------------------------------------------
# Metric computation
# ---------------------------------------------------------------------------
def _rom(angle_deg):
    """min / max / range / median of an unwrapped angle series (degrees)."""
    a = np.asarray(angle_deg, dtype=float)
    lo, hi = float(np.min(a)), float(np.max(a))
    return {"min_deg": round(lo, 2), "max_deg": round(hi, 2),
            "range_deg": round(hi - lo, 2),
            "median_deg": round(float(np.median(a)), 2)}


def _peak_velocity_deg_s(angle_deg, t_ms):
    """Peak |angular velocity| (deg/s) of a DOF angle over the session."""
    t_s = np.asarray(t_ms, dtype=float) / 1000.0
    if len(t_s) < 2:
        return 0.0
    # np.gradient handles the (near-)uniform reconcile grid; guard flat spans.
    with np.errstate(invalid="ignore", divide="ignore"):
        v = np.gradient(np.asarray(angle_deg, dtype=float), t_s)
    v = v[np.isfinite(v)]
    return round(float(np.max(np.abs(v))) if v.size else 0.0, 1)


def compute_joint_metrics(jkey, joint, q_prox, q_dist, t_ms, clinical):
    """Per-DOF angle series + ROM + peak velocity for one computable joint."""
    q_rel = qnorm(qmul(qconj(q_prox), q_dist))
    sequence = joint.decomposition.split()[0]
    euler = euler_from_quat(q_rel, sequence)          # (N,3) radians, slot order

    dofs = []
    for d in joint.dofs:
        # Unwrap in radians (removes ±π sawtooth) THEN convert — so a real sweep
        # past the wrap point is continuous and ROM is the true excursion.
        ang = np.degrees(np.unwrap(euler[:, d.seq_index]))
        dofs.append({
            "key": d.key, "name": d.name, "plane": d.plane,
            "rom": _rom(ang),
            "peak_velocity_deg_s": _peak_velocity_deg_s(ang, t_ms),
        })
    return {
        "key": jkey, "name": joint.name,
        "clinical": clinical,
        "decomposition": joint.decomposition,
        "dofs": dofs,
    }


def segment_angular_speed(q, t_ms):
    """Mean & peak angular speed (deg/s) of a segment — calibration-free.

    Frame-independent (a geodesic step between consecutive orientations), so it is
    honest with or without a mounting offset: it measures how much the bone turned,
    not where it points.
    """
    t_s = np.asarray(t_ms, dtype=float) / 1000.0
    dt = np.diff(t_s)
    dt = np.where(dt <= 0, np.nan, dt)
    dot = np.clip(np.abs(np.sum(q[:-1] * q[1:], axis=1)), 0.0, 1.0)
    speed = np.degrees(2.0 * np.arccos(dot)) / dt
    speed = speed[np.isfinite(speed)]
    if speed.size == 0:
        return {"mean_deg_s": 0.0, "peak_deg_s": 0.0}
    return {"mean_deg_s": round(float(np.mean(speed)), 1),
            "peak_deg_s": round(float(np.max(speed)), 1)}


def compute_metrics(montage, t_ms, seg_quats, seg_meta, calibration):
    """Build the full metrics report from a loaded aligned stream + calibration."""
    q_seg, calibrated = apply_calibration(seg_quats, calibration)
    caps = resolve(montage)

    joints_out, blocked_out = [], []
    for cap in caps:
        if cap.kind != "joint":
            continue
        joint = JOINTS[cap.target]
        if not cap.computable:
            blocked_out.append({"key": cap.target, "name": cap.name,
                                "missing": cap.missing})
            continue
        both_cal = calibrated.get(joint.proximal, False) and \
            calibrated.get(joint.distal, False)
        jm = compute_joint_metrics(
            cap.target, joint, q_seg[joint.proximal], q_seg[joint.distal],
            t_ms, clinical=both_cal)
        if not both_cal:
            jm["warning"] = ("angles/ROM are RELATIVE only — one or both nodes "
                             "lack anatomical calibration; capture a neutral pose "
                             "for clinical angles")
        joints_out.append(jm)

    segments_out = []
    for seg in seg_quats:
        segments_out.append({
            "segment": seg,
            "node_id": seg_meta[seg]["node_id"],
            "calibrated": calibrated.get(seg, False),
            "angular_speed": segment_angular_speed(seg_quats[seg], t_ms),
        })

    return {
        "schema_version": SCHEMA_VERSION,
        "subject": montage.get("subject", {}),
        "session": montage.get("session", {}),
        "calibration_used": bool(calibration),
        "n_samples": int(len(t_ms)),
        "duration_s": round(float((t_ms[-1] - t_ms[0]) / 1000.0), 2),
        "joints": joints_out,
        "blocked_joints": blocked_out,
        "segments": segments_out,
        "gates_note": ("clinical=false ⇒ relative-only (uncalibrated). Sync "
                       "confidence + dropout are session-level trust inputs from "
                       "reconcile — surface them alongside these numbers."),
    }


# ---------------------------------------------------------------------------
# Reporting
# ---------------------------------------------------------------------------
def print_report(rep):
    subj = rep["subject"].get("id", "?")
    sess = rep["session"].get("id", "?")
    cal_label = "applied" if rep["calibration_used"] else "NONE (relative-only)"
    print(f"Metrics — subject {subj}, session {sess}")
    print(f"  {rep['n_samples']} samples over {rep['duration_s']} s   "
          f"calibration: {cal_label}")

    print(f"\nJOINT range of motion ({len(rep['joints'])} computable)")
    for j in rep["joints"]:
        tag = "" if j["clinical"] else "   [RELATIVE — uncalibrated]"
        print(f"  {j['key']:<12} {j['name']}{tag}")
        print(f"       decomposition: {j['decomposition']}")
        for d in j["dofs"]:
            r = d["rom"]
            print(f"       {d['name']:<28} ROM {r['range_deg']:6.1f}°  "
                  f"[{r['min_deg']:+.0f}…{r['max_deg']:+.0f}]  "
                  f"peak {d['peak_velocity_deg_s']:.0f}°/s")
    for b in rep["blocked_joints"]:
        print(f"  {b['key']:<12} {b['name']}  ✗ blocked: missing "
              f"{', '.join(b['missing'])}")

    print(f"\nSEGMENT activity ({len(rep['segments'])})")
    for s in rep["segments"]:
        cal = "cal" if s["calibrated"] else "raw"
        sp = s["angular_speed"]
        print(f"  {s['segment']:<12} [{cal}]  mean {sp['mean_deg_s']:6.1f}°/s  "
              f"peak {sp['peak_deg_s']:6.1f}°/s")


# ---------------------------------------------------------------------------
# Commands
# ---------------------------------------------------------------------------
def cmd_compute(args):
    montage = load_montage(args.montage)
    t_ms, seg_quats, seg_meta = load_aligned(args.aligned_csv, montage)
    calibration = None
    if args.calibration and os.path.exists(args.calibration):
        with open(args.calibration, encoding="utf-8-sig") as f:
            calibration = json.load(f)
    elif args.calibration:
        print(f"[metrics] no calibration at {args.calibration} — reporting "
              f"RELATIVE-only.", file=sys.stderr)

    rep = compute_metrics(montage, t_ms, seg_quats, seg_meta, calibration)
    if args.out:
        with open(args.out, "w") as f:
            json.dump(rep, f, indent=2)
        print(f"[metrics] wrote {args.out}\n")
    if args.json:
        print(json.dumps(rep, indent=2))
    else:
        print_report(rep)


# ---------------------------------------------------------------------------
# Self-test — decomposition round-trips + a known injected ROM sweep
# ---------------------------------------------------------------------------
def _q_axis(axis, rad):
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    h = rad / 2.0
    return np.array([np.cos(h), *(np.sin(h) * axis)])


_AXIS_VEC = {"X": [1, 0, 0], "Y": [0, 1, 0], "Z": [0, 0, 1]}


def _quat_from_euler(sequence, angles):
    """Intrinsic-sequence quaternion: q = q_a1(θ1) ⊗ q_a2(θ2) ⊗ q_a3(θ3)."""
    q = IDENTITY.copy()
    for ax, th in zip(sequence, angles):
        q = qmul(q, _q_axis(_AXIS_VEC[ax], th))
    return qnorm(q)


def _geodesic_deg(a, b):
    d = np.clip(abs(float(np.dot(qnorm(a), qnorm(b)))), 0.0, 1.0)
    return np.degrees(2.0 * np.arccos(d))


def selftest():
    rng = np.random.default_rng(0)
    ok = True

    # (1) Every implemented sequence must round-trip: build a quaternion from
    #     random angles, decompose, rebuild — the ORIENTATION must match (angle
    #     triples are non-unique, the rotation they encode is not).
    print("[selftest] Euler decomposition round-trip (orientation recovery):")
    for seq in sorted(_EULER):
        worst = 0.0
        for _ in range(2000):
            ang = rng.uniform(-np.pi, np.pi, 3)
            # keep the middle angle away from gimbal lock for a clean angle test
            ang[1] = rng.uniform(0.2, np.pi - 0.2) if seq[0] == seq[2] \
                else rng.uniform(-np.pi / 2 + 0.2, np.pi / 2 - 0.2)
            q = _quat_from_euler(seq, ang)
            rec = euler_from_quat(q[None, :], seq)[0]
            q2 = _quat_from_euler(seq, rec)
            worst = max(worst, _geodesic_deg(q, q2))
        # Round-trip through arccos/atan2 leaves float64 noise ~1e-6°; anything
        # under 1e-4° is orders below any measurement relevance.
        good = worst < 1e-4
        ok = ok and good
        print(f"           {seq}: max orientation error {worst:.2e}° "
              f"{'OK' if good else 'FAIL'}")

    # (2) Physical check: a pure right-elbow FLEXION sweep 0->90° must show up as
    #     ~90° range on flex_ext and ~0 on pro_sup, using the real body model +
    #     the same apply-calibration/compute path the CLI uses.
    print("[selftest] injected right-elbow flexion sweep 0->90°:")
    n = 200
    t_ms = np.arange(n) * 20.0
    sweep = np.radians(np.linspace(0.0, 90.0, n))
    # Flexion is the Z (first) slot of the elbow's ZXY sequence. Put the upper arm
    # at identity and rotate the forearm about Z — so q_rel = Rz(sweep).
    ua = np.tile(IDENTITY, (n, 1))
    fa = np.stack([_q_axis([0, 0, 1], s) for s in sweep])
    montage = {
        "schema_version": "1.0", "subject": {"id": "S"}, "session": {"id": "t"},
        "calibration": {"captured": True},
        "nodes": [
            {"node_id": "UA", "column": "n0", "segment": "upper_arm_r",
             "calibrated": True},
            {"node_id": "FA", "column": "n1", "segment": "forearm_r",
             "calibrated": True}],
    }
    seg_quats = {"upper_arm_r": qnorm(ua), "forearm_r": qnorm(fa)}
    seg_meta = {"upper_arm_r": {"column": "n0", "node_id": "UA"},
                "forearm_r": {"column": "n1", "node_id": "FA"}}
    # Calibration with identity offsets => clinical=true, angles unchanged.
    cal = {"segments": {s: {"mounting_offset_quat": list(IDENTITY)}
                        for s in seg_quats}}
    rep = compute_metrics(montage, t_ms, seg_quats, seg_meta, cal)
    elbow = next(j for j in rep["joints"] if j["key"] == "elbow_r")
    flex = next(d for d in elbow["dofs"] if d["key"] == "flex_ext")
    pro = next(d for d in elbow["dofs"] if d["key"] == "pro_sup")
    flex_ok = abs(flex["rom"]["range_deg"] - 90.0) < 0.5
    pro_ok = abs(pro["rom"]["range_deg"]) < 0.5
    clin_ok = elbow["clinical"] is True
    ok = ok and flex_ok and pro_ok and clin_ok
    print(f"           flex_ext range {flex['rom']['range_deg']:.2f}° (want 90) "
          f"{'OK' if flex_ok else 'FAIL'}; pro_sup range "
          f"{pro['rom']['range_deg']:.2f}° (want 0) {'OK' if pro_ok else 'FAIL'}")

    # (3) Same joint, NO calibration -> must be flagged relative-only, and the
    #     shoulder must be BLOCKED (no torso node) rather than fabricated.
    rep2 = compute_metrics(montage, t_ms, seg_quats, seg_meta, None)
    elbow2 = next(j for j in rep2["joints"] if j["key"] == "elbow_r")
    rel_ok = (elbow2["clinical"] is False and "warning" in elbow2)
    blocked = {b["key"] for b in rep2["blocked_joints"]}
    block_ok = "shoulder_r" in blocked
    ok = ok and rel_ok and block_ok
    print(f"[selftest] no-cal elbow flagged relative-only: "
          f"{'OK' if rel_ok else 'FAIL'}; shoulder blocked (no torso): "
          f"{'OK' if block_ok else 'FAIL'}")

    # (4) Unwrap: a flexion sweep crossing 180° must report its true range, not a
    #     spurious ~360° jump from the atan2 branch cut.
    big = np.radians(np.linspace(150.0, 210.0, n))       # 60° sweep across ±180
    fa_big = np.stack([_q_axis([0, 0, 1], s) for s in big])
    sq = {"upper_arm_r": qnorm(ua), "forearm_r": qnorm(fa_big)}
    rep3 = compute_metrics(montage, t_ms, sq, seg_meta, cal)
    fb = next(d for d in next(j for j in rep3["joints"] if j["key"] == "elbow_r")
              ["dofs"] if d["key"] == "flex_ext")
    unwrap_ok = abs(fb["rom"]["range_deg"] - 60.0) < 0.5
    ok = ok and unwrap_ok
    print(f"[selftest] sweep across ±180° unwrapped: range "
          f"{fb['rom']['range_deg']:.2f}° (want 60) "
          f"{'OK' if unwrap_ok else 'FAIL'}")

    print(f"\n[selftest] {'PASS' if ok else 'FAIL'} — Euler round-trip (all "
          f"sequences), injected ROM recovery, calibration gating, wrap handling.")
    return 0 if ok else 1


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd")

    pc = sub.add_parser("compute", help="per-DOF joint angles + ROM from the "
                        "aligned stream")
    pc.add_argument("aligned_csv", help="reconcile_nodes.py output CSV")
    pc.add_argument("montage", help="montage JSON (column<->segment mapping)")
    pc.add_argument("--calibration", default="calibration.json",
                    help="calibration JSON from calibrate_segments.py "
                         "(default: calibration.json; absent -> relative-only)")
    pc.add_argument("--out", help="write the metrics report as JSON here")
    pc.add_argument("--json", action="store_true",
                    help="also print the report as JSON to stdout")
    pc.set_defaults(func=cmd_compute)

    ps = sub.add_parser("selftest", help="validate the math (no hardware)")
    ps.set_defaults(func=lambda a: sys.exit(selftest()))

    args = ap.parse_args()
    if not getattr(args, "cmd", None):
        ap.error("choose a command: compute | selftest")
    args.func(args)


if __name__ == "__main__":
    main()
