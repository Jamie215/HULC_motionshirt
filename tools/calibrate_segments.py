#!/usr/bin/env python3
"""
HULC Motion Shirt — stage-5 calibration: the sensor→segment solve (cache-and-verify).

Turns the aligned multi-node stream (reconcile_nodes.py output) into an
anatomically-referenced one, by solving the ONE thing the shared world frame does
not give us: how each sensor housing sits rotated/tilted on its bone. That
per-segment **mounting offset** is what flips every joint angle / ROM number from
"relative only" to clinical (see MONTAGE_SCHEMA.md §4, SETUP_AND_CALIBRATION_PLAN.md §4).

Why this is a mounting solve, not a "resting quaternion" capture
---------------------------------------------------------------
The BNO reports the full Rotation Vector (report 0x05, magnetometer-referenced),
so every node's orientation already lives in a shared world frame (gravity +
magnetic north). The cross-node *spatial* frame is therefore given. What is left
unknown is the sensor→segment rotation. A short static **neutral / N-pose** solves
it: in that pose we know each bone's intended orientation (the anatomical target,
identity by default = "this configuration is 0°"), and comparing that against the
sensor reading recovers the offset for each node.

    frames:  W = world (gravity + mag north)   S = sensor body   B = bone/segment
    measured:  q_WS(t)          (world-from-sensor, what the node logs)
    mounting:  q_SB   (bone-from... constant this wear)   q_WB = q_WS ⊗ q_SB
    at neutral, target q_WB = q_target (default identity):
        q_SB = conj(q_WS_neutral) ⊗ q_target          <-- what we solve & cache
    runtime:   q_seg(t) = q_WS(t) ⊗ q_SB              (anatomical orientation)
    joint:     q_rel = conj(q_seg_prox) ⊗ q_seg_dist  (0° at neutral)

Cache-and-verify (the "feels like skipping" path)
-------------------------------------------------
The offset is a property of THIS wear — a power cycle for charging does NOT break
it, re-donning does. So it is cacheable. Alongside each offset we cache a
HEADING-INDEPENDENT consistency baseline so the next don can decide reuse vs.
re-pose without a fresh deliberate pose every time:

  * per-segment  `gravity_in_segment` — where "up" points in the calibrated bone
    frame. Invariant to which way the subject faces (yaw about gravity), so it
    isolates a single node's slip even with no neighbour.
  * per-pair     `neutral_rel` — the calibrated relative orientation of each
    adjacent segment pair at neutral. Fully world-frame-independent; catches
    relative slippage / a swapped node.

`verify` recomputes both from a new still window (using the CACHED offsets) and
compares to the baseline: all deviations small -> REUSE silently; any over
threshold -> prompt for a fresh ~2 s pose. The same check run on a later still
window within one session catches intra-session slippage.

Usage
-----
    # solve offsets + baseline from the neutral-pose window in a montage:
    python tools/calibrate_segments.py calibrate aligned.csv montage.json \
        --out calibration.json

    # next don (or later in the session): decide reuse vs re-pose:
    python tools/calibrate_segments.py verify aligned_today.csv montage.json \
        --calibration calibration.json

    # validate the math end-to-end (no hardware):
    python tools/calibrate_segments.py selftest

Anatomy (segments + joint adjacency) is imported from motion_capabilities.py so
there is one source of truth; this tool never redefines the body model.
"""

import argparse
import json
import os
import sys

try:
    import numpy as np
except ImportError:  # pragma: no cover
    raise SystemExit("This tool needs numpy:  pip install numpy")

# Single source of truth for the body model + montage validation.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from motion_capabilities import JOINTS, SEGMENTS, validate_montage  # noqa: E402

SCHEMA_VERSION = "1.0"

# World "up" axis of the BNO rotation-vector reference frame (gravity-aligned).
# Only its direction matters, and only for the per-segment gravity check; the
# per-pair check is independent of it. If a firmware convention change moves the
# gravity axis, change this one constant.
WORLD_UP = np.array([0.0, 0.0, 1.0])

# Reuse-vs-re-pose thresholds (degrees). A quiet-standing pose repeats to within
# a few degrees; past these the mounting has moved enough to matter for angles.
DEFAULT_SEG_GRAVITY_DEG = 8.0    # per-segment tilt drift
DEFAULT_PAIR_ANGLE_DEG = 10.0    # per-pair relative-orientation drift

# A window whose mean angular speed exceeds this was not actually still — the
# "neutral pose" is contaminated by motion and the offset will be biased.
STILL_MAX_RAD_S = 0.30

DEFAULT_WIN_MS = 2000.0          # auto-detected still-window length


# ---------------------------------------------------------------------------
# Quaternion helpers (Hamilton convention, [w, x, y, z], vectorized over (...,4))
# ---------------------------------------------------------------------------
def qmul(a, b):
    aw, ax, ay, az = a[..., 0], a[..., 1], a[..., 2], a[..., 3]
    bw, bx, by, bz = b[..., 0], b[..., 1], b[..., 2], b[..., 3]
    return np.stack([
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ], axis=-1)


def qconj(q):
    return q * np.array([1.0, -1.0, -1.0, -1.0])


def qnorm(q):
    n = np.linalg.norm(q, axis=-1, keepdims=True)
    n = np.where(n == 0.0, 1.0, n)
    return q / n


def qrotate(q, v):
    """Rotate vector(s) v by quaternion(s) q: v' = q ⊗ (0,v) ⊗ q*."""
    v = np.broadcast_to(v, q.shape[:-1] + (3,))
    qv = np.concatenate([np.zeros(q.shape[:-1] + (1,)), v], axis=-1)
    return qmul(qmul(q, qv), qconj(q))[..., 1:]


def quat_angle(q):
    """Rotation magnitude (radians) of a unit quaternion."""
    w = np.clip(np.abs(q[..., 0]), 0.0, 1.0)
    return 2.0 * np.arccos(w)


def angle_between_quats(a, b):
    """Geodesic angle (radians) between two orientations."""
    return float(quat_angle(qmul(qconj(a), b)))


def quat_average(Q):
    """Markley average of unit quaternions Q (N,4) — eigenvector of Σqqᵀ.

    Handles the q/-q sign ambiguity correctly, unlike a naive componentwise mean.
    """
    Q = qnorm(np.asarray(Q, dtype=float))
    A = Q.T @ Q
    _, vecs = np.linalg.eigh(A)      # ascending; last col = largest eigenvalue
    q = vecs[:, -1]
    if q[0] < 0:
        q = -q
    return qnorm(q)


# ---------------------------------------------------------------------------
# Aligned-CSV loading (binds reconcile output columns to montage segments)
# ---------------------------------------------------------------------------
def load_aligned(csv_path, montage):
    """Return (t_ms[N], {segment: quats[N,4]}, {segment: node_meta}).

    Reads the reconcile_nodes.py CSV (header: t_common_ms, n0_qw, n0_qx, ...) and
    picks out the four columns for each montage node by its `column` prefix.
    """
    with open(csv_path) as f:
        header = f.readline().strip().split(",")
    idx = {name: i for i, name in enumerate(header)}
    if "t_common_ms" not in idx:
        raise SystemExit(f"{csv_path}: no 't_common_ms' column — is this a "
                         f"reconcile_nodes.py output?")
    data = np.loadtxt(csv_path, delimiter=",", skiprows=1, ndmin=2)
    if data.shape[0] < 2:
        raise SystemExit(f"{csv_path}: need >=2 aligned samples, got "
                         f"{data.shape[0]}.")
    t_ms = data[:, idx["t_common_ms"]].astype(float)

    seg_quats, seg_meta = {}, {}
    for n in montage["nodes"]:
        col, seg = n.get("column"), n.get("segment")
        cols = [f"{col}_q{c}" for c in ("w", "x", "y", "z")]
        missing = [c for c in cols if c not in idx]
        if missing:
            raise SystemExit(f"{csv_path}: montage node segment '{seg}' maps to "
                             f"column '{col}', but the CSV has no {missing}. "
                             f"Does this aligned file match this montage?")
        q = data[:, [idx[c] for c in cols]].astype(float)
        seg_quats[seg] = qnorm(q)
        seg_meta[seg] = {"column": col, "node_id": n.get("node_id")}
    return t_ms, seg_quats, seg_meta


# ---------------------------------------------------------------------------
# Stillness — find/measure a low-motion window
# ---------------------------------------------------------------------------
def combined_speed(t_ms, seg_quats):
    """Mean angular speed (rad/s) across all segments, per inter-sample gap.

    Returns (t_mid[N-1], speed[N-1]). This is the mounting-invariant motion the
    reconcile tool also keys on: near-zero => the body is still.
    """
    dt = np.diff(t_ms) / 1000.0
    dt = np.where(dt <= 0, np.nan, dt)
    per = []
    for q in seg_quats.values():
        dot = np.clip(np.abs(np.sum(q[:-1] * q[1:], axis=1)), 0.0, 1.0)
        per.append(2.0 * np.arccos(dot) / dt)
    speed = np.nanmean(np.vstack(per), axis=0)
    t_mid = (t_ms[:-1] + t_ms[1:]) / 2.0
    return t_mid, speed


def window_mask(t_ms, t0, t1):
    m = (t_ms >= t0) & (t_ms <= t1)
    return m


def auto_still_window(t_ms, seg_quats, win_ms=DEFAULT_WIN_MS):
    """Find the quietest window of ~win_ms: the lowest mean angular speed.

    Returns (t0, t1). Falls back to the whole record if it is shorter than the
    requested window.
    """
    t_mid, speed = combined_speed(t_ms, seg_quats)
    span = t_ms[-1] - t_ms[0]
    if span <= win_ms or len(speed) < 3:
        return float(t_ms[0]), float(t_ms[-1])
    best_t0, best_mean = float(t_ms[0]), np.inf
    # Slide the window start over sample times; each candidate is [t0, t0+win].
    for t0 in t_ms[:-1]:
        t1 = t0 + win_ms
        if t1 > t_ms[-1]:
            break
        m = (t_mid >= t0) & (t_mid <= t1)
        if m.sum() < 2:
            continue
        mean = float(np.nanmean(speed[m]))
        if mean < best_mean:
            best_mean, best_t0 = mean, float(t0)
    return best_t0, best_t0 + win_ms


def window_stillness(t_ms, seg_quats, t0, t1):
    """Mean angular speed (rad/s) inside [t0, t1] — a pose-quality number."""
    t_mid, speed = combined_speed(t_ms, seg_quats)
    m = (t_mid >= t0) & (t_mid <= t1)
    return float(np.nanmean(speed[m])) if m.any() else float("nan")


# ---------------------------------------------------------------------------
# Calibration core
# ---------------------------------------------------------------------------
def _adjacent_pairs(present_segments):
    """Joint keys whose proximal AND distal segments are both present."""
    pairs = {}
    for jkey, j in JOINTS.items():
        if j.proximal in present_segments and j.distal in present_segments:
            pairs[jkey] = (j.proximal, j.distal)
    return pairs


def solve_calibration(t_ms, seg_quats, seg_meta, t0, t1, targets=None):
    """Solve per-segment mounting offsets + the consistency baseline over [t0,t1].

    targets: optional {segment: q_target[4]} anatomical orientation at neutral;
             defaults to identity (neutral pose defines each segment's zero).
    """
    targets = targets or {}
    m = window_mask(t_ms, t0, t1)
    if m.sum() < 2:
        raise SystemExit(f"neutral window [{t0:.0f},{t1:.0f}] ms holds "
                         f"{int(m.sum())} sample(s) — widen it or pick another.")

    ident = np.array([1.0, 0.0, 0.0, 0.0])
    segments = {}
    calibrated_q = {}     # segment -> calibrated orientation at neutral (mean)
    for seg, q in seg_quats.items():
        qwin = q[m]
        q_bar = quat_average(qwin)                    # mean world-from-sensor
        target = np.asarray(targets.get(seg, ident), dtype=float)
        offset = qmul(qconj(q_bar), target)           # q_SB (cached)
        q_seg = qmul(q_bar, offset)                    # == target at neutral
        calibrated_q[seg] = q_seg
        # Spread of the pose about its mean = how still/consistent the pose was.
        resid_deg = float(np.degrees(np.mean(
            [angle_between_quats(q_bar, qi) for qi in qwin])))
        # Heading-independent per-segment baseline: gravity in the bone frame.
        grav_seg = qrotate(qconj(q_seg), WORLD_UP)
        segments[seg] = {
            "column": seg_meta[seg]["column"],
            "node_id": seg_meta[seg]["node_id"],
            "mounting_offset_quat": _q_list(offset),
            "target_quat": _q_list(target),
            "neutral_mean_quat": _q_list(q_bar),
            "gravity_in_segment": [round(float(v), 8) for v in grav_seg],
            "pose_residual_deg": round(resid_deg, 3),
        }

    pairs = {}
    for jkey, (prox, dist) in _adjacent_pairs(seg_quats).items():
        q_rel = qmul(qconj(calibrated_q[prox]), calibrated_q[dist])
        pairs[jkey] = {
            "proximal": prox,
            "distal": dist,
            "neutral_rel_quat": _q_list(q_rel),
            "neutral_rel_angle_deg": round(float(np.degrees(quat_angle(q_rel))), 3),
        }
    return segments, pairs


def _q_list(q):
    return [round(float(v), 8) for v in qnorm(np.asarray(q, dtype=float))]


def build_calibration(montage, t_ms, seg_quats, seg_meta, t0, t1,
                      csv_path, targets=None):
    stillness = window_stillness(t_ms, seg_quats, t0, t1)
    segments, pairs = solve_calibration(t_ms, seg_quats, seg_meta, t0, t1, targets)
    return {
        "schema_version": SCHEMA_VERSION,
        "subject": montage.get("subject", {}),
        "session": montage.get("session", {}),
        "created_from": os.path.basename(csv_path),
        "world_up_axis": [float(v) for v in WORLD_UP],
        "neutral": {
            "pose": montage.get("calibration", {}).get("neutral_pose", "N-pose"),
            "t_window_ms": [round(float(t0), 1), round(float(t1), 1)],
            "n_samples": int(window_mask(t_ms, t0, t1).sum()),
            "stillness_rad_s": round(stillness, 4),
            "still_ok": bool(stillness <= STILL_MAX_RAD_S),
        },
        "thresholds": {
            "segment_gravity_deg": DEFAULT_SEG_GRAVITY_DEG,
            "pair_angle_deg": DEFAULT_PAIR_ANGLE_DEG,
        },
        "segments": segments,
        "pairs": pairs,
    }


# ---------------------------------------------------------------------------
# Verify — reuse vs re-pose against a cached calibration
# ---------------------------------------------------------------------------
def verify_calibration(calibration, t_ms, seg_quats, t0, t1):
    """Apply cached offsets over a new still window; measure drift vs baseline.

    Returns a report dict with per-segment / per-pair deviations and an overall
    decision: 'reuse' (mounting held) or 're-pose' (something moved).
    """
    thr = calibration.get("thresholds", {})
    seg_thr = thr.get("segment_gravity_deg", DEFAULT_SEG_GRAVITY_DEG)
    pair_thr = thr.get("pair_angle_deg", DEFAULT_PAIR_ANGLE_DEG)
    up = np.asarray(calibration.get("world_up_axis", WORLD_UP), dtype=float)

    m = window_mask(t_ms, t0, t1)
    if m.sum() < 2:
        raise SystemExit(f"verify window [{t0:.0f},{t1:.0f}] ms holds "
                         f"{int(m.sum())} sample(s) — widen it or pick another.")

    calibrated_q = {}
    seg_reports = []
    for seg, base in calibration.get("segments", {}).items():
        if seg not in seg_quats:
            seg_reports.append({"segment": seg, "status": "absent",
                                "note": "in calibration but not in this montage/CSV"})
            continue
        offset = np.asarray(base["mounting_offset_quat"], dtype=float)
        q_bar = quat_average(seg_quats[seg][m])
        q_seg = qmul(q_bar, offset)
        calibrated_q[seg] = q_seg
        grav_now = qrotate(qconj(q_seg), up)
        grav_base = np.asarray(base["gravity_in_segment"], dtype=float)
        dev = float(np.degrees(np.arccos(np.clip(
            np.dot(qnorm3(grav_now), qnorm3(grav_base)), -1.0, 1.0))))
        seg_reports.append({
            "segment": seg, "status": "ok",
            "gravity_dev_deg": round(dev, 2),
            "over_threshold": bool(dev > seg_thr),
        })

    pair_reports = []
    for jkey, base in calibration.get("pairs", {}).items():
        prox, dist = base["proximal"], base["distal"]
        if prox not in calibrated_q or dist not in calibrated_q:
            continue
        q_rel = qmul(qconj(calibrated_q[prox]), calibrated_q[dist])
        q_rel_base = np.asarray(base["neutral_rel_quat"], dtype=float)
        dev = float(np.degrees(angle_between_quats(q_rel_base, q_rel)))
        pair_reports.append({
            "pair": jkey, "proximal": prox, "distal": dist,
            "rel_dev_deg": round(dev, 2),
            "over_threshold": bool(dev > pair_thr),
        })

    offenders = ([r["segment"] for r in seg_reports if r.get("over_threshold")]
                 + [r["pair"] for r in pair_reports if r.get("over_threshold")])
    stillness = window_stillness(t_ms, seg_quats, t0, t1)
    return {
        "decision": "re-pose" if offenders else "reuse",
        "offenders": offenders,
        "window_ms": [round(float(t0), 1), round(float(t1), 1)],
        "stillness_rad_s": round(stillness, 4),
        "still_ok": bool(stillness <= STILL_MAX_RAD_S),
        "segments": seg_reports,
        "pairs": pair_reports,
    }


def qnorm3(v):
    n = np.linalg.norm(v)
    return v / n if n else v


# ---------------------------------------------------------------------------
# Montage write-back
# ---------------------------------------------------------------------------
def apply_to_montage(montage, calibration):
    """Flip the montage's calibration flags to reflect a fresh calibrate.

    Sets calibration.captured, records the neutral window, and marks every node
    whose segment got an offset `calibrated: true` — so motion_capabilities.py
    resolves clinical (not relative-only) angles.
    """
    cal = montage.setdefault("calibration", {})
    cal["captured"] = True
    cal["t_window_ms"] = calibration["neutral"]["t_window_ms"]
    calibrated_segs = set(calibration.get("segments", {}))
    for n in montage.get("nodes", []):
        if n.get("segment") in calibrated_segs:
            n["calibrated"] = True
    return montage


# ---------------------------------------------------------------------------
# Reporting
# ---------------------------------------------------------------------------
def print_calibrate_report(cal):
    n = cal["neutral"]
    print(f"Calibration — subject {cal['subject'].get('id', '?')}, "
          f"session {cal['session'].get('id', '?')}")
    still = "still ✓" if n["still_ok"] else f"NOT still ✗ (> {STILL_MAX_RAD_S} rad/s)"
    print(f"  neutral window: {n['t_window_ms'][0]:.0f}–{n['t_window_ms'][1]:.0f} ms "
          f"({n['n_samples']} samples), {n['stillness_rad_s']:.3f} rad/s [{still}]")
    if not n["still_ok"]:
        print("  ! the pose window was not still — offsets may be biased; "
              "re-capture a quiet ~2 s neutral pose.")
    print(f"\nSEGMENT mounting offsets ({len(cal['segments'])})")
    for seg, s in cal["segments"].items():
        flag = "" if s["pose_residual_deg"] < 3.0 else "  ! noisy pose"
        print(f"  ✓ {seg:<12} offset {_fmt_q(s['mounting_offset_quat'])}  "
              f"pose spread {s['pose_residual_deg']:.2f}°{flag}")
    if cal["pairs"]:
        print(f"\nADJACENT-PAIR neutral baselines ({len(cal['pairs'])})")
        for jkey, p in cal["pairs"].items():
            print(f"  {jkey:<12} {p['proximal']}→{p['distal']}: "
                  f"neutral relative {p['neutral_rel_angle_deg']:.2f}°")
    print("\nWrote per-segment offsets + a heading-independent consistency "
          "baseline. On the next don, run `verify` to decide reuse vs re-pose.")


def print_verify_report(rep):
    print(f"Verify — window {rep['window_ms'][0]:.0f}–{rep['window_ms'][1]:.0f} ms, "
          f"{rep['stillness_rad_s']:.3f} rad/s "
          f"[{'still ✓' if rep['still_ok'] else 'NOT still ✗'}]")
    if not rep["still_ok"]:
        print("  ! verify window was not still — decision is unreliable; "
              "hold a quiet pose.")
    print("\nPER-SEGMENT tilt drift (gravity in bone frame, heading-independent)")
    for r in rep["segments"]:
        if r["status"] == "absent":
            print(f"  · {r['segment']:<12} {r['note']}")
            continue
        mark = "✗" if r["over_threshold"] else "✓"
        print(f"  {mark} {r['segment']:<12} {r['gravity_dev_deg']:.2f}°")
    if rep["pairs"]:
        print("\nPER-PAIR relative drift (world-frame-independent)")
        for r in rep["pairs"]:
            mark = "✗" if r["over_threshold"] else "✓"
            print(f"  {mark} {r['pair']:<12} {r['proximal']}→{r['distal']}: "
                  f"{r['rel_dev_deg']:.2f}°")
    print()
    if rep["decision"] == "reuse":
        print("DECISION: REUSE — cached calibration still holds; no re-pose needed.")
    else:
        print(f"DECISION: RE-POSE — mounting moved on: {', '.join(rep['offenders'])}. "
              f"Capture a fresh ~2 s neutral pose.")


def _fmt_q(q):
    return "[" + ", ".join(f"{v:+.3f}" for v in q) + "]"


# ---------------------------------------------------------------------------
# Commands
# ---------------------------------------------------------------------------
def load_montage(path):
    with open(path) as f:
        montage = json.load(f)
    errors = validate_montage(montage)
    if errors:
        print("montage INVALID:", file=sys.stderr)
        for e in errors:
            print(f"  - {e}", file=sys.stderr)
        raise SystemExit(2)
    return montage


def cmd_calibrate(args):
    montage = load_montage(args.montage)
    t_ms, seg_quats, seg_meta = load_aligned(args.aligned_csv, montage)

    if args.window:
        t0, t1 = args.window
    else:
        cal_win = montage.get("calibration", {}).get("t_window_ms")
        if cal_win and len(cal_win) == 2:
            t0, t1 = float(cal_win[0]), float(cal_win[1])
            print(f"[calibrate] using montage neutral window "
                  f"{t0:.0f}–{t1:.0f} ms")
        else:
            t0, t1 = auto_still_window(t_ms, seg_quats, args.win_ms)
            print(f"[calibrate] auto-detected quietest window "
                  f"{t0:.0f}–{t1:.0f} ms (no montage t_window_ms given)")

    cal = build_calibration(montage, t_ms, seg_quats, seg_meta, t0, t1,
                            args.aligned_csv)
    with open(args.out, "w") as f:
        json.dump(cal, f, indent=2)
    print(f"[calibrate] wrote {args.out}\n")
    print_calibrate_report(cal)

    if args.update_montage:
        apply_to_montage(montage, cal)
        with open(args.montage, "w") as f:
            json.dump(montage, f, indent=2)
        print(f"\n[calibrate] updated {args.montage}: calibration.captured=true, "
              f"calibrated flags set for {len(cal['segments'])} node(s).")


def cmd_verify(args):
    montage = load_montage(args.montage)
    t_ms, seg_quats, _ = load_aligned(args.aligned_csv, montage)
    with open(args.calibration) as f:
        calibration = json.load(f)

    if args.window:
        t0, t1 = args.window
    else:
        t0, t1 = auto_still_window(t_ms, seg_quats, args.win_ms)
        # Diagnostic → stderr so --json stdout stays parseable.
        print(f"[verify] auto-detected quietest window "
              f"{t0:.0f}–{t1:.0f} ms", file=sys.stderr)
    rep = verify_calibration(calibration, t_ms, seg_quats, t0, t1)
    if args.json:
        print(json.dumps(rep, indent=2))
    else:
        print_verify_report(rep)
    raise SystemExit(0 if rep["decision"] == "reuse" else 1)


# ---------------------------------------------------------------------------
# Self-test — synthesize sensors with known mounting, verify recovery + gating
# ---------------------------------------------------------------------------
def _axis_angle(axis, deg):
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    h = np.radians(deg) / 2.0
    return np.array([np.cos(h), *(np.sin(h) * axis)])


def _synth_session(true_bone, mounting, t_ms, extra_yaw=None, noise=0.004,
                   seed=0):
    """Build measured world-from-sensor streams for a still pose.

    true_bone[seg] = q_WB (bone orientation in world), mounting[seg] = q_SB.
    q_WS = q_WB ⊗ conj(q_SB). extra_yaw rotates all bones about gravity (a
    different facing, same room) to test heading-independence.
    """
    rng = np.random.default_rng(seed)
    n = len(t_ms)
    out = {}
    for seg, q_wb in true_bone.items():
        if extra_yaw is not None:
            q_wb = qmul(extra_yaw, q_wb)
        q_ws = qmul(q_wb, qconj(mounting[seg]))
        arr = np.tile(q_ws, (n, 1)) + noise * rng.standard_normal((n, 4))
        out[seg] = qnorm(arr)
    return out


def selftest():
    print("[selftest] synthesizing a right-arm chain (torso + upper_arm_r + "
          "forearm_r) ...")
    # Known true bone orientations at neutral (arbitrary but fixed) and known,
    # distinct mountings per node.
    true_bone = {
        "torso":       _axis_angle([0, 0, 1], 5.0),
        "upper_arm_r": _axis_angle([1, 0, 0], 12.0),
        "forearm_r":   _axis_angle([0, 1, 0], 20.0),
    }
    mounting = {
        "torso":       _axis_angle([0, 1, 0], 30.0),
        "upper_arm_r": _axis_angle([1, 1, 0], 55.0),
        "forearm_r":   _axis_angle([0, 1, 1], 80.0),
    }
    meta = {s: {"column": f"n{i}", "node_id": f"HULC-IMU-{i:04d}"}
            for i, s in enumerate(true_bone)}
    t_ms = np.arange(0, 3000, 20.0)      # 3 s @ 50 Hz still window

    q0 = _synth_session(true_bone, mounting, t_ms, seed=1)
    segs, pairs = solve_calibration(t_ms, q0, meta, t_ms[0], t_ms[-1])

    # (1) After calibration, each segment reads its target (identity) at neutral,
    #     i.e. mounting is removed to sub-degree.
    ident = np.array([1.0, 0.0, 0.0, 0.0])
    max_resid = 0.0
    for seg, q in q0.items():
        offset = np.asarray(segs[seg]["mounting_offset_quat"], dtype=float)
        q_seg = quat_average(qmul(q, offset))
        max_resid = max(max_resid, np.degrees(angle_between_quats(ident, q_seg)))
    print(f"[selftest] mounting removed: max neutral residual {max_resid:.3f}° "
          f"(want < 0.5°)")

    # (2) Pairwise neutral relatives are ~0° (both segments zeroed).
    max_pair = max(p["neutral_rel_angle_deg"] for p in pairs.values())
    print(f"[selftest] neutral pairwise relatives: max {max_pair:.3f}° "
          f"(want < 0.5°)")

    calibration = {
        "schema_version": SCHEMA_VERSION, "world_up_axis": list(WORLD_UP),
        "thresholds": {"segment_gravity_deg": DEFAULT_SEG_GRAVITY_DEG,
                       "pair_angle_deg": DEFAULT_PAIR_ANGLE_DEG},
        "segments": segs, "pairs": pairs,
    }

    # (3) Un-shifted re-don, SAME mounting but the subject faces a new direction
    #     (90° yaw about gravity) and clocks differ -> must REUSE (baseline is
    #     heading-independent).
    yaw = _axis_angle([0, 0, 1], 90.0)
    q_redon = _synth_session(true_bone, mounting, t_ms, extra_yaw=yaw, seed=2)
    rep_reuse = verify_calibration(calibration, t_ms, q_redon, t_ms[0], t_ms[-1])
    print(f"[selftest] re-don, new facing (+90° yaw), same straps: "
          f"decision={rep_reuse['decision']} "
          f"(max seg {max(r.get('gravity_dev_deg', 0) for r in rep_reuse['segments']):.2f}°, "
          f"max pair {max((r['rel_dev_deg'] for r in rep_reuse['pairs']), default=0):.2f}°)")

    # (4) Slipped node: the forearm strap rotated 20° about a horizontal axis
    #     since calibration -> must RE-POSE and name forearm_r.
    slipped = dict(mounting)
    slipped["forearm_r"] = qmul(_axis_angle([1, 0, 0], 20.0), mounting["forearm_r"])
    q_slip = _synth_session(true_bone, slipped, t_ms, seed=3)
    rep_repose = verify_calibration(calibration, t_ms, q_slip, t_ms[0], t_ms[-1])
    print(f"[selftest] forearm strap slipped 20°: decision="
          f"{rep_repose['decision']}, offenders={rep_repose['offenders']}")

    # (5) Auto still-window finds a quiet stretch inside a mostly-moving record.
    t_long = np.arange(0, 6000, 20.0)
    rng = np.random.default_rng(7)
    moving = {}
    for seg, q_ws in _synth_session(true_bone, mounting, t_long, seed=4).items():
        drift = np.cumsum(0.03 * rng.standard_normal((len(t_long), 3)), axis=0)
        # inject motion everywhere EXCEPT a still window at 2000-4000 ms
        still = (t_long >= 2000) & (t_long <= 4000)
        drift[still] = drift[still][0]
        dq = np.column_stack([np.ones(len(t_long)), drift])
        moving[seg] = qnorm(qmul(q_ws, qnorm(dq)))
    aw0, aw1 = auto_still_window(t_long, moving, DEFAULT_WIN_MS)
    print(f"[selftest] auto still-window on a moving record: "
          f"{aw0:.0f}–{aw1:.0f} ms (injected still 2000–4000 ms)")

    ok = (max_resid < 0.5 and max_pair < 0.5
          and rep_reuse["decision"] == "reuse"
          and rep_repose["decision"] == "re-pose"
          and "forearm_r" in rep_repose["offenders"]
          and 1500 <= aw0 <= 2500)
    print(f"\n[selftest] {'PASS' if ok else 'FAIL'} "
          f"(offset recovery, heading-independent reuse, slip detection, "
          f"still-window search)")
    return 0 if ok else 1


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
def _window_arg(value):
    parts = value.split(",")
    if len(parts) != 2:
        raise argparse.ArgumentTypeError("window must be 't0,t1' in ms")
    return float(parts[0]), float(parts[1])


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd")

    pc = sub.add_parser("calibrate", help="solve mounting offsets + baseline "
                        "from a neutral-pose window")
    pc.add_argument("aligned_csv", help="reconcile_nodes.py output CSV")
    pc.add_argument("montage", help="montage JSON (column<->segment mapping)")
    pc.add_argument("--out", default="calibration.json",
                    help="output calibration JSON (default: calibration.json)")
    pc.add_argument("--window", type=_window_arg, metavar="t0,t1",
                    help="neutral-pose window in ms (overrides montage "
                         "t_window_ms / auto-detect)")
    pc.add_argument("--win-ms", type=float, default=DEFAULT_WIN_MS,
                    help="auto-detected window length in ms (default 2000)")
    pc.add_argument("--update-montage", action="store_true",
                    help="write calibration.captured + calibrated flags back "
                         "into the montage file")
    pc.set_defaults(func=cmd_calibrate)

    pv = sub.add_parser("verify", help="reuse-vs-re-pose check against a cached "
                        "calibration")
    pv.add_argument("aligned_csv", help="a NEW reconcile output CSV")
    pv.add_argument("montage", help="montage JSON")
    pv.add_argument("--calibration", default="calibration.json",
                    help="cached calibration JSON (default: calibration.json)")
    pv.add_argument("--window", type=_window_arg, metavar="t0,t1",
                    help="still window in ms (default: auto-detect quietest)")
    pv.add_argument("--win-ms", type=float, default=DEFAULT_WIN_MS,
                    help="auto-detected window length in ms (default 2000)")
    pv.add_argument("--json", action="store_true",
                    help="emit the verify report as JSON")
    pv.set_defaults(func=cmd_verify)

    ps = sub.add_parser("selftest", help="validate the math (no hardware)")
    ps.set_defaults(func=lambda a: sys.exit(selftest()))

    args = ap.parse_args()
    if not getattr(args, "cmd", None):
        ap.error("choose a command: calibrate | verify | selftest")
    args.func(args)


if __name__ == "__main__":
    main()
