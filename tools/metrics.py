#!/usr/bin/env python3
"""
HULC Motion Shirt — stage-6 metrics: per-DOF joint angles + range of motion.

The payoff stage. Everything upstream (capture → offload → reconcile → montage →
calibrate) exists to turn raw sensor quaternions into an anatomically-referenced
stream; this tool reads that stream and produces the clinical numbers a therapist
actually wants. It computes every metric tier the resolver (motion_capabilities)
declares for the montage, so what appears is exactly what the placement supports:

  * joint   — per-DOF angle series → range of motion (min/max/range/median),
              angular velocity (peak/mean/RMS), and repetition count.
  * segment — angular speed, angular travel + active-time fraction, elevation
              from vertical, movement smoothness (SPARC), and a time-in-posture
              (posture-dwell) histogram once the segment is calibrated.
  * derived — L/R ROM symmetry, bilateral activity asymmetry, inter-joint
              coordination (cross-correlation + lag), and trunk compensation.

ROM and the per-DOF read-out came first (SETUP_AND_CALIBRATION_PLAN.md §6, build
step 4); the rest ride on the same decomposition and the same honesty contract.

Every metric's meaning, formula, units, and calibration gate are catalogued in
tools/METRICS.md (this module is the implementation it indexes).

How a joint angle is computed (the whole chain in five lines)
-------------------------------------------------------------
    q_seg(t) = q_WS(t) ⊗ q_SB          apply the cached mounting offset (stage 5)
    q_rel(t) = conj(q_seg_prox) ⊗ q_seg_dist   distal-relative-to-proximal
    q_anat   = conj(q_WA) ⊗ q_rel ⊗ q_WA        re-express in anatomical axes
    (α,β,γ)  = euler(q_anat, sequence)  decompose in the joint's ISB/Wu sequence
    angle_dof = (α|β|γ)[seq_index]      pick the slot that IS this clinical DOF

q_WA is the anatomical frame at neutral (X anterior, Y superior, Z right) from
calibration.json's `anatomical_frame` block. It matters: the mounting offset
zeroes each segment but leaves its axes on the WORLD compass (Z = up), and the
ISB sequences assume anatomical axes — without q_WA a pure elbow flexion lands
in whichever slot the subject's facing happens to put it.

The Euler sequence and the slot each clinical DOF occupies both come from
motion_capabilities.JOINTS (the one body model) — this tool never re-declares
anatomy or invents a convention. Because calibration zeroes q_rel at the neutral
pose and q_WA ties the axes to the body, every angle here is measured from
anatomical zero about anatomical axes, so ROM is clinical, not "relative to
whatever the arm happened to be doing at t=0".

Honesty (the same contract the resolver prints)
-----------------------------------------------
* It only computes a joint the montage can actually resolve — both adjacent nodes
  present — reusing motion_capabilities.resolve(). A blocked joint is reported
  blocked, with the missing node named, never a fabricated number.
* A joint whose two nodes are not BOTH anatomically calibrated, or whose
  anatomical axes are unknown (no confident facing -> no q_WA), is flagged
  `clinical: false` and its angles are RELATIVE-only (zero is "pose at the
  neutral window" at best, and without q_WA the DOF split uses world axes, so
  its labels need not match the anatomy). Same wording as the resolver.
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
    qmul, qconj, qnorm, load_aligned, load_montage, anatomical_frame_quat,
    build_anatomical_frame,
)

SCHEMA_VERSION = "1.0"

IDENTITY = np.array([1.0, 0.0, 0.0, 0.0])

# Below this |sin(middle angle)| the Euler split is gimbal-degenerate (the first
# and third axes align); we collapse them onto the still-well-defined sum so the
# recovered orientation stays exact even though the individual angles don't.
_GIMBAL_SIN = 1e-6

# ---- tuning for the derived / activity metrics ----------------------------
# A segment turning faster than this counts as "active" (used for active-time
# fraction and the bilateral activity comparison). Quiet standing sits well
# under it; deliberate limb motion is well over.
ACTIVE_SPEED_DEG_S = 20.0
# A repetition only counts if the primary DOF actually swings at least this far —
# below it the "cycles" are noise/tremor, not reps.
REP_MIN_AMPLITUDE_DEG = 15.0
# Posture-dwell histogram edges (segment elevation from vertical, degrees).
POSTURE_BIN_EDGES_DEG = [0, 30, 60, 90, 120, 150, 180]
# SPARC (spectral arc length) smoothness — Balasubramanian et al. 2015 defaults.
SPARC_FC_MAX_HZ = 10.0     # ignore spectral content above this
SPARC_AMP_THRESH = 0.05    # normalized-magnitude floor that sets the cutoff band

# Every Euler split has a singularity where the outer two axes align and their
# angles become ill-defined (a proper sequence like YXY at middle angle ≈ 0/π —
# the shoulder's classic "arm at the side, plane-of-elevation undefined" pole; a
# Tait-Bryan sequence like ZXY at middle angle ≈ ±90°). Within this guard band of
# the singular value the outer-slot DOFs are treated as undefined, so a joint
# resting at the pole doesn't emit a spurious 180° swing or an infinite velocity.
SINGULARITY_GUARD_DEG = 10.0


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


def resolve_anatomical_frame(calibration):
    """q_WA (world-from-anatomical) from a calibration, or None if unknown.

    Reads the `anatomical_frame` block; a calibration written before that block
    existed falls back to its confident `heading`, so older files still work.
    """
    cal = calibration or {}
    af = cal.get("anatomical_frame")
    if af is not None:
        return (qnorm(np.asarray(af["quat"], dtype=float))
                if af.get("quat") else None)
    h = cal.get("heading", {})
    if h.get("confident") and "facing_deg" in h:
        return anatomical_frame_quat(h["facing_deg"])
    return None


# ---------------------------------------------------------------------------
# Small shared primitives
# ---------------------------------------------------------------------------
def _fs_hz(t_ms):
    """Sample rate of the (uniform) reconcile grid, from the median gap."""
    d = np.diff(np.asarray(t_ms, dtype=float))
    d = d[d > 0]
    return 1000.0 / float(np.median(d)) if d.size else 0.0


def _stats(a):
    """min / max / range / median / mean of a 1-D series, rounded."""
    a = np.asarray(a, dtype=float)
    lo, hi = float(np.min(a)), float(np.max(a))
    return {"min_deg": round(lo, 2), "max_deg": round(hi, 2),
            "range_deg": round(hi - lo, 2),
            "median_deg": round(float(np.median(a)), 2),
            "mean_deg": round(float(np.mean(a)), 2)}


def _rom(angle_deg):
    """ROM summary of an unwrapped angle series (min/max/range/median)."""
    s = _stats(angle_deg)
    return {k: s[k] for k in ("min_deg", "max_deg", "range_deg", "median_deg")}


def _velocity_deg_s(angle_deg, t_ms):
    """Signed angular-velocity series (deg/s) of a DOF/segment angle."""
    t_s = np.asarray(t_ms, dtype=float) / 1000.0
    if len(t_s) < 2:
        return np.zeros_like(np.asarray(angle_deg, dtype=float))
    with np.errstate(invalid="ignore", divide="ignore"):
        v = np.gradient(np.asarray(angle_deg, dtype=float), t_s)
    return np.where(np.isfinite(v), v, 0.0)


def _contiguous_runs(mask):
    """Yield (start, stop) index slices of each maximal True run in a bool mask."""
    m = np.asarray(mask, dtype=bool)
    if not m.any():
        return
    edges = np.diff(m.astype(np.int8))
    starts = list(np.where(edges == 1)[0] + 1)
    stops = list(np.where(edges == -1)[0] + 1)
    if m[0]:
        starts = [0] + starts
    if m[-1]:
        stops = stops + [len(m)]
    yield from zip(starts, stops)


def velocity_stats(angle_deg, t_ms, mask=None):
    """Peak / mean-|.| / RMS angular velocity (deg/s) of an angle series.

    With `mask`, velocity is differentiated only WITHIN contiguous runs of valid
    samples — so a masked-out singular passage never contributes a cross-gap jump
    (which would otherwise read as an impossibly high peak velocity).
    """
    ang = np.asarray(angle_deg, dtype=float)
    t_ms = np.asarray(t_ms, dtype=float)
    if mask is None:
        v = np.abs(_velocity_deg_s(ang, t_ms))
    else:
        parts = [np.abs(_velocity_deg_s(ang[a:b], t_ms[a:b]))
                 for a, b in _contiguous_runs(mask) if b - a >= 2]
        if not parts:
            return {"peak_deg_s": 0.0, "mean_abs_deg_s": 0.0, "rms_deg_s": 0.0}
        v = np.concatenate(parts)
    return {"peak_deg_s": round(float(np.max(v)), 1),
            "mean_abs_deg_s": round(float(np.mean(v)), 1),
            "rms_deg_s": round(float(np.sqrt(np.mean(v * v))), 1)}


def count_reps(angle_deg, min_amplitude_deg=REP_MIN_AMPLITUDE_DEG):
    """Count movement repetitions in an angle series (calibration-free).

    A rep = one full oscillation. We use hysteretic midline crossings: the signal
    must dip below (mid − h) and rise back above (mid + h) to score one cycle,
    where the band h is 25% of the swing. That rejects tremor/noise wiggles while
    counting genuine flex→extend→flex cycles, and it works on RELATIVE angles too
    (only the shape matters, not the anatomical zero) — hence needs_calibration
    is False for this metric in the resolver.
    """
    a = np.asarray(angle_deg, dtype=float)
    amp = float(np.max(a) - np.min(a))
    if amp < min_amplitude_deg or a.size < 3:
        return 0
    mid = 0.5 * (np.max(a) + np.min(a))
    h = 0.25 * amp
    reps, armed = 0, False           # armed = we've seen the low half since last rep
    for x in a:
        if x < mid - h:
            armed = True
        elif x > mid + h and armed:
            reps += 1
            armed = False
    return reps


# ---------------------------------------------------------------------------
# Segment-level metrics (one node; calibration-free unless noted)
# ---------------------------------------------------------------------------
def segment_angular_speed(q, t_ms):
    """Signed angular-speed series → mean & peak (deg/s). Frame-independent.

    A geodesic step between consecutive orientations, so it is honest with or
    without a mounting offset: it measures how much the bone turned, not where it
    points. Returns (summary_dict, speed_series) — the series feeds SPARC.
    """
    t_s = np.asarray(t_ms, dtype=float) / 1000.0
    dt = np.diff(t_s)
    dt = np.where(dt <= 0, np.nan, dt)
    dot = np.clip(np.abs(np.sum(q[:-1] * q[1:], axis=1)), 0.0, 1.0)
    speed = np.degrees(2.0 * np.arccos(dot)) / dt
    speed = np.where(np.isfinite(speed), speed, 0.0)
    if speed.size == 0:
        return {"mean_deg_s": 0.0, "peak_deg_s": 0.0}, speed
    return ({"mean_deg_s": round(float(np.mean(speed)), 1),
             "peak_deg_s": round(float(np.max(speed)), 1)}, speed)


def segment_elevation_series(q_seg):
    """Elevation-from-vertical (deg) of a segment's long axis, per sample.

    The calibrated segment frame is identity at the neutral pose, so its local +Z
    is the bone direction that pointed to world-up at neutral; the elevation is the
    angle that material axis has tilted from vertical = arccos(R[2,2]). Uncalibrated,
    it is the sensor board's own tilt — still a real inclination, just not anchored
    to the bone (flagged by the segment's `calibrated`).
    """
    R22 = rotmat_from_quat(q_seg)[..., 2, 2]
    return np.degrees(np.arccos(np.clip(R22, -1.0, 1.0)))


def segment_travel_deg(q, t_ms):
    """Total angular path length (deg) + active-time fraction of a segment.

    Travel is ∫|angular speed| dt (cumulative rotation, direction-agnostic); the
    active fraction is the share of time spent above ACTIVE_SPEED_DEG_S. Both are
    session aggregates — robust even when two limbs aren't time-aligned — which is
    why the bilateral activity comparison is valid at low sync confidence.
    """
    t_s = np.asarray(t_ms, dtype=float) / 1000.0
    dt = np.diff(t_s)
    good = dt > 0
    dot = np.clip(np.abs(np.sum(q[:-1] * q[1:], axis=1)), 0.0, 1.0)
    step_deg = np.degrees(2.0 * np.arccos(dot))
    travel = float(np.sum(step_deg[good]))
    speed = np.where(good, step_deg / np.where(good, dt, 1.0), 0.0)
    active_frac = (float(np.sum(dt[good & (speed > ACTIVE_SPEED_DEG_S)]))
                   / float(np.sum(dt[good]))) if np.any(good) else 0.0
    return {"travel_deg": round(travel, 1),
            "active_time_frac": round(active_frac, 3)}


def posture_dwell(elevation_deg, t_ms, edges=POSTURE_BIN_EDGES_DEG):
    """Time-in-posture histogram: fraction of session in each elevation band.

    Time-weighted (by inter-sample dt), so it is correct even on a non-uniform
    grid. Anatomically meaningful only once the segment is calibrated (elevation
    is then a real bone inclination) — the caller gates it on `calibrated`.
    """
    t_s = np.asarray(t_ms, dtype=float) / 1000.0
    a = np.asarray(elevation_deg, dtype=float)
    # per-sample dt weight (midpoint rule); guard the single-sample case
    dt = np.gradient(t_s) if t_s.size > 1 else np.array([1.0])
    total = float(np.sum(dt)) or 1.0
    fracs = []
    for lo, hi in zip(edges[:-1], edges[1:]):
        m = (a >= lo) & (a < hi if hi != edges[-1] else a <= hi)
        fracs.append(round(float(np.sum(dt[m])) / total, 3))
    return {"edges_deg": list(edges), "fraction": fracs}


def sparc(speed, fs, fc_max=SPARC_FC_MAX_HZ, amp_thresh=SPARC_AMP_THRESH):
    """Spectral arc length smoothness of a speed profile (Balasubramanian 2015).

    Smoother movement → simpler, more compact magnitude spectrum → shorter arc
    length → value nearer 0; jerky movement → more negative (typ. −1.5 smooth to
    −5+ jerky). Returns None when there is essentially no movement (spectrum has no
    DC to normalize against), so a still segment reports "no movement" rather than
    a meaningless number.
    """
    v = np.asarray(speed, dtype=float)
    if v.size < 4 or fs <= 0 or np.allclose(v, v[0]):
        return None
    # zero-pad for frequency resolution (padlevel 4, per the reference impl)
    nfft = int(2 ** (np.ceil(np.log2(v.size)) + 4))
    Mf = np.abs(np.fft.rfft(v, n=nfft))
    freq = np.fft.rfftfreq(nfft, d=1.0 / fs)
    if Mf[0] <= 0:
        return None
    Mf = Mf / Mf[0]                              # normalize by DC
    # cutoff band: up to fc_max AND up to the last freq whose magnitude ≥ threshold
    in_band = freq <= fc_max
    inx = np.where(in_band)[0]
    above = np.where(Mf[inx] >= amp_thresh)[0]
    fc_i = inx[above[-1]] if above.size else inx[-1]
    f_sel = freq[: fc_i + 1]
    M_sel = Mf[: fc_i + 1]
    if f_sel[-1] <= 0 or f_sel.size < 2:
        return None
    f_norm = f_sel / f_sel[-1]                   # normalize freq axis to [0, 1]
    arc = -np.sum(np.sqrt(np.diff(f_norm) ** 2 + np.diff(M_sel) ** 2))
    return round(float(arc), 3)


def cross_correlation(a, b, fs):
    """Peak normalized cross-correlation of two angle series and its lag (s).

    Both series are mean-removed and unit-normalized, so the peak is in [−1, 1]
    (1 = move identically in phase). The lag is where distal LAGS proximal: a
    positive lag means `b` follows `a`. Used for inter-joint coordination.
    """
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)
    # Keep only samples valid in BOTH (outer-slot DOFs carry NaN at singularities).
    keep = np.isfinite(a) & np.isfinite(b)
    if keep.sum() < 4:
        return {"peak_r": 0.0, "lag_s": 0.0}
    a, b = a[keep] - np.mean(a[keep]), b[keep] - np.mean(b[keep])
    na, nb = np.linalg.norm(a), np.linalg.norm(b)
    if na == 0 or nb == 0:
        return {"peak_r": 0.0, "lag_s": 0.0}
    xc = np.correlate(a / na, b / nb, mode="full")
    lags = np.arange(-len(a) + 1, len(a))
    k = int(np.argmax(np.abs(xc)))
    # np.correlate peaks at lags=-D when b is delayed by D vs a; negate so a
    # positive lag reads as "b follows a by lag_s seconds".
    return {"peak_r": round(float(xc[k]), 3),
            "lag_s": round(float(-lags[k] / fs), 3) if fs > 0 else 0.0}


# ---------------------------------------------------------------------------
# Joint-level metrics (two adjacent nodes)
# ---------------------------------------------------------------------------
def compute_joint_metrics(jkey, joint, q_prox, q_dist, t_ms, clinical,
                          q_wa=None):
    """Per-DOF angle series → ROM, velocity, and reps for one computable joint.

    `q_wa` is the anatomical frame at neutral; the relative rotation is
    re-expressed in it before decomposition. None -> decomposed in world axes
    (only ever reported as relative-only).

    Returns (report_dict, {dof_key: angle_series_deg}); the series are handed back
    so the derived tier (symmetry / coordination) can reuse them without
    re-decomposing. Outer-slot DOFs carry NaN at samples inside the decomposition
    singularity band, so downstream consumers skip those the same way ROM does.
    """
    q_rel = qnorm(qmul(qconj(q_prox), q_dist))
    if q_wa is not None:
        q_rel = qnorm(qmul(qmul(qconj(q_wa), q_rel), q_wa))
    sequence = joint.decomposition.split()[0]
    euler = euler_from_quat(q_rel, sequence)          # (N,3) radians, slot order

    # Middle angle governs the singularity: proper Euler (e.g. YXY) is singular at
    # middle≈0/π (|sin|→0); Tait-Bryan (e.g. ZXY) at middle≈±90° (|cos|→0).
    proper = sequence[0] == sequence[2]
    mid = euler[:, 1]
    sing = np.abs(np.sin(mid)) if proper else np.abs(np.cos(mid))
    well_defined = sing >= np.sin(np.radians(SINGULARITY_GUARD_DEG))

    dofs, series = [], {}
    for d in joint.dofs:
        # Unwrap in radians (removes the ±π sawtooth) THEN convert, so a real sweep
        # past the wrap point stays continuous and ROM is the true excursion.
        ang = np.degrees(np.unwrap(euler[:, d.seq_index]))
        outer = d.seq_index != 1        # the middle slot is never singular
        entry = {"key": d.key, "name": d.name, "plane": d.plane}
        if outer and not well_defined.all():
            # Only trust this DOF where the split is well-conditioned.
            defined_frac = float(well_defined.mean())
            if well_defined.sum() >= 2:
                good = ang[well_defined]
                entry["rom"] = _rom(good)
                entry["velocity"] = velocity_stats(ang, t_ms, mask=well_defined)
            else:
                entry["rom"] = None
                entry["velocity"] = None
            entry["defined_frac"] = round(defined_frac, 3)
            entry["singularity_note"] = (
                f"undefined within {SINGULARITY_GUARD_DEG:.0f}° of the {sequence} "
                f"singularity ({'neutral/overhead' if proper else '±90°'}); "
                f"reported over the {defined_frac * 100:.0f}% of samples where it "
                f"is well-conditioned")
            series[d.key] = np.where(well_defined, ang, np.nan)
        else:
            entry["rom"] = _rom(ang)
            entry["velocity"] = velocity_stats(ang, t_ms)
            series[d.key] = ang
        dofs.append(entry)

    # Reps (and the derived tier) use the joint's declared primary DOF — for the
    # shoulder that is elevation, since plane of elevation / axial rotation can
    # swing widely without the arm doing much. Without one, or if it is
    # singular all session, fall back to the DOF that swung the most.
    rom_dofs = [d for d in dofs if d["rom"] is not None]
    declared = [d for d in rom_dofs if d["key"] == joint.primary]
    primary = (declared[0] if declared
               else max(rom_dofs, key=lambda d: d["rom"]["range_deg"])
               if rom_dofs else dofs[0])
    reps = {"count": 0, "primary_dof": primary["key"]}
    if primary["rom"] is not None:
        reps["count"] = count_reps(series[primary["key"]][
            np.isfinite(series[primary["key"]])])
    report = {
        "key": jkey, "name": joint.name,
        "clinical": clinical,
        "decomposition": joint.decomposition,
        "dofs": dofs,
        "reps": reps,
    }
    return report, series


# ---------------------------------------------------------------------------
# Derived metrics (a set of joints/segments) — driven by the resolver
# ---------------------------------------------------------------------------
def _primary_dof(joint_report):
    """The primary DOF entry (with a valid ROM) of a joint report, or None."""
    dofs = [d for d in (joint_report or {}).get("dofs", []) if d["rom"]]
    if not dofs:
        return None
    key = joint_report.get("reps", {}).get("primary_dof")
    return next((d for d in dofs if d["key"] == key),
                max(dofs, key=lambda d: d["rom"]["range_deg"]))


def _finite_range(a):
    """Peak-to-peak of the finite samples of a series (0 if none)."""
    f = a[np.isfinite(a)]
    return float(np.max(f) - np.min(f)) if f.size else 0.0


def _primary_series(joint_series, jrep, jkey):
    """The primary DOF's angle series for a computed joint (or None)."""
    s = joint_series.get(jkey)
    if not s:
        return None
    key = jrep.get(jkey, {}).get("reps", {}).get("primary_dof")
    return s[key] if key in s else max(s.values(), key=_finite_range)


def _symmetry_index(left, right):
    """Symmetric %-difference of two magnitudes (0 = identical, →200 opposite)."""
    denom = 0.5 * (abs(left) + abs(right))
    return round(100.0 * abs(left - right) / denom, 1) if denom else 0.0


def compute_derived(caps, joint_reports, joint_series, seg_activity, seg_series,
                    calibrated, t_ms):
    """Compute each derived capability the resolver unlocked for this montage."""
    fs = _fs_hz(t_ms)
    jrep = {j["key"]: j for j in joint_reports}
    out = []
    for cap in caps:
        if cap.kind != "derived" or not cap.computable:
            continue
        tgt, entry = cap.target, {"target": cap.target, "name": cap.name,
                                  "requires": cap.requires, "metrics": {}}

        if tgt.startswith("symmetry_"):
            base = tgt[len("symmetry_"):]
            lj, rj = jrep.get(f"{base}_l"), jrep.get(f"{base}_r")
            lp, rp = _primary_dof(lj), _primary_dof(rj)
            if lp and rp:
                lrom, rrom = lp["rom"]["range_deg"], rp["rom"]["range_deg"]
                entry["metrics"] = {
                    "symmetry_index": _symmetry_index(lrom, rrom),
                    "rom_ratio": round(min(lrom, rrom) / max(lrom, rrom), 3)
                    if max(lrom, rrom) else 0.0,
                    "left_rom_deg": lrom, "right_rom_deg": rrom,
                    "dof": lp["key"]}
                entry["clinical"] = lj["clinical"] and rj["clinical"]

        elif tgt.startswith("activity_asymmetry_"):
            base = tgt[len("activity_asymmetry_"):]
            la, ra = seg_activity.get(f"{base}_l"), seg_activity.get(f"{base}_r")
            if la and ra:
                lt, rt = la["travel_deg"], ra["travel_deg"]
                lact, ract = la["active_time_frac"], ra["active_time_frac"]
                tot = lt + rt
                entry["metrics"] = {
                    # signed laterality: + = right used more, in [−100, +100]
                    "asymmetry_index": round(100.0 * (rt - lt) / tot, 1)
                    if tot else 0.0,
                    "use_ratio": round(rt / lt, 3) if lt else None,
                    "active_time_ratio": round(ract / lact, 3) if lact else None,
                    "left_travel_deg": lt, "right_travel_deg": rt}
                entry["note"] = ("session aggregate — no time-alignment needed, so "
                                 "valid even at low sync confidence")

        elif tgt.startswith("coordination_"):
            js = [j for j in cap.requires if j in joint_series]
            if len(js) >= 2:
                a, b = _primary_series(joint_series, jrep, js[0]), \
                    _primary_series(joint_series, jrep, js[1])
                if a is not None and b is not None:
                    entry["metrics"] = {"pair": [js[0], js[1]],
                                        **cross_correlation(a, b, fs)}

        elif tgt.startswith("compensation_"):
            act = seg_activity.get("torso")
            elev = seg_series.get("torso")
            if act is not None:
                entry["metrics"] = {
                    "trunk_travel_deg": act["travel_deg"],
                    "trunk_elevation_range_deg":
                        round(float(np.max(elev) - np.min(elev)), 1)
                        if elev is not None else None}
                entry["clinical"] = calibrated.get("torso", False)
                if not entry["clinical"]:
                    entry["note"] = ("trunk excursion is RELATIVE — torso node "
                                     "uncalibrated")

        if entry["metrics"]:
            out.append(entry)
    return out


# ---------------------------------------------------------------------------
# Top-level assembly
# ---------------------------------------------------------------------------
def compute_metrics(montage, t_ms, seg_quats, seg_meta, calibration):
    """Build the full metrics report from a loaded aligned stream + calibration."""
    q_seg, calibrated = apply_calibration(seg_quats, calibration)
    q_wa = resolve_anatomical_frame(calibration)
    caps = resolve(montage)
    fs = _fs_hz(t_ms)

    # --- joints ---
    joints_out, blocked_out, joint_series = [], [], {}
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
        jm, series = compute_joint_metrics(
            cap.target, joint, q_seg[joint.proximal], q_seg[joint.distal],
            t_ms, clinical=both_cal and q_wa is not None, q_wa=q_wa)
        if not both_cal:
            jm["warning"] = ("angles/ROM are RELATIVE only — one or both nodes "
                             "lack anatomical calibration; capture a neutral pose "
                             "for clinical angles")
        elif q_wa is None:
            jm["warning"] = ("angles/ROM are RELATIVE only — anatomical axes are "
                             "unknown (no confident facing), so the DOF split uses "
                             "world axes and its labels may not match the anatomy; "
                             "add a torso node or calibrate with --facing-deg")
        joints_out.append(jm)
        joint_series[cap.target] = series

    # --- segments (raw quaternion for speed/travel; calibrated for elevation) ---
    segments_out, seg_activity, seg_elev = [], {}, {}
    for seg in seg_quats:
        speed_sum, speed_series = segment_angular_speed(seg_quats[seg], t_ms)
        activity = segment_travel_deg(seg_quats[seg], t_ms)
        elev = segment_elevation_series(q_seg[seg])
        seg_activity[seg] = activity
        seg_elev[seg] = elev
        s = {
            "segment": seg,
            "node_id": seg_meta[seg]["node_id"],
            "calibrated": calibrated.get(seg, False),
            "angular_speed": speed_sum,
            "travel": activity,
            "elevation": _stats(elev),
            "smoothness_sparc": sparc(speed_series, fs),
        }
        if calibrated.get(seg, False):
            s["posture_dwell"] = posture_dwell(elev, t_ms)
        segments_out.append(s)

    # --- derived (symmetry / asymmetry / coordination / compensation) ---
    derived_out = compute_derived(caps, joints_out, joint_series, seg_activity,
                                  seg_elev, calibrated, t_ms)

    return {
        "schema_version": SCHEMA_VERSION,
        "subject": montage.get("subject", {}),
        "session": montage.get("session", {}),
        "calibration_used": bool(calibration),
        "anatomical_axes": q_wa is not None,
        "sample_rate_hz": round(fs, 2),
        "n_samples": int(len(t_ms)),
        "duration_s": round(float((t_ms[-1] - t_ms[0]) / 1000.0), 2),
        "joints": joints_out,
        "blocked_joints": blocked_out,
        "segments": segments_out,
        "derived": derived_out,
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
    if rep["calibration_used"] and not rep.get("anatomical_axes"):
        print("  ! anatomical axes unknown (no confident facing) — joint angles "
              "are RELATIVE-only")

    print(f"\nJOINT range of motion ({len(rep['joints'])} computable)")
    for j in rep["joints"]:
        tag = "" if j["clinical"] else "   [RELATIVE — uncalibrated]"
        reps = j.get("reps", {})
        rep_s = (f"   reps {reps['count']} ({reps['primary_dof']})"
                 if reps.get("count") else "")
        print(f"  {j['key']:<12} {j['name']}{tag}{rep_s}")
        print(f"       decomposition: {j['decomposition']}")
        for d in j["dofs"]:
            r, v = d["rom"], d["velocity"]
            if r is None:
                print(f"       {d['name']:<28} ROM      —   "
                      f"(undefined at singularity for the whole session)")
                continue
            frac = (f"  [{d['defined_frac']*100:.0f}% defined]"
                    if "defined_frac" in d else "")
            print(f"       {d['name']:<28} ROM {r['range_deg']:6.1f}°  "
                  f"[{r['min_deg']:+.0f}…{r['max_deg']:+.0f}]  "
                  f"peak {v['peak_deg_s']:.0f}°/s{frac}")
    for b in rep["blocked_joints"]:
        print(f"  {b['key']:<12} {b['name']}  ✗ blocked: missing "
              f"{', '.join(b['missing'])}")

    print(f"\nSEGMENT activity ({len(rep['segments'])})")
    for s in rep["segments"]:
        cal = "cal" if s["calibrated"] else "raw"
        sp, el = s["angular_speed"], s["elevation"]
        sm = s["smoothness_sparc"]
        sm_s = f"SPARC {sm:+.2f}" if sm is not None else "SPARC  — (still)"
        print(f"  {s['segment']:<12} [{cal}]  travel {s['travel']['travel_deg']:7.0f}°"
              f"  active {s['travel']['active_time_frac']*100:4.0f}%  "
              f"elev {el['range_deg']:5.0f}°  peak {sp['peak_deg_s']:6.0f}°/s  {sm_s}")

    if rep.get("derived"):
        print(f"\nDERIVED ({len(rep['derived'])})")
        for d in rep["derived"]:
            kv = ", ".join(f"{k}={v}" for k, v in d["metrics"].items()
                           if not isinstance(v, list))
            print(f"  {d['target']:<26} {d['name']}")
            print(f"       {kv}")
            if d.get("note"):
                print(f"       ! {d['note']}")


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


def _to_world(q_anat, q_wa):
    """World quaternion(s) of a calibrated segment whose anatomical rotation from
    neutral is q_anat — the inverse of the q_WA re-expression in the joint chain."""
    return qnorm(qmul(qmul(q_wa, q_anat), qconj(q_wa)))


def _frame_cal(segments, facing_deg):
    """Identity-offset calibration with a stated facing (-> anatomical axes)."""
    heading = {"source": "manual", "confident": True, "facing_deg": facing_deg}
    return {"segments": {s: {"mounting_offset_quat": list(IDENTITY)}
                         for s in segments},
            "heading": heading,
            "anatomical_frame": build_anatomical_frame(heading)}


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

    # (2) Physical check: a pure right-elbow FLEXION sweep 0->90° — a rotation of
    #     the forearm about the subject's left-right axis (anatomical Z) — must
    #     show up as ~90° on flex_ext and ~0 on pro_sup WHATEVER direction the
    #     subject faces. Motion is built in anatomical axes and mapped into the
    #     world through the stated facing, so this catches a decomposition that
    #     silently uses world axes.
    print("[selftest] injected right-elbow flexion sweep 0->90°, any facing:")
    n = 200
    t_ms = np.arange(n) * 20.0
    sweep = np.radians(np.linspace(0.0, 90.0, n))
    ua_anat = np.tile(IDENTITY, (n, 1))
    fa_anat = np.stack([_q_axis([0, 0, 1], x) for x in sweep])
    montage = {
        "schema_version": "1.0", "subject": {"id": "S"}, "session": {"id": "t"},
        "calibration": {"captured": True},
        "nodes": [
            {"node_id": "UA", "column": "n0", "segment": "upper_arm_r",
             "calibrated": True},
            {"node_id": "FA", "column": "n1", "segment": "forearm_r",
             "calibrated": True}],
    }
    seg_meta = {"upper_arm_r": {"column": "n0", "node_id": "UA"},
                "forearm_r": {"column": "n1", "node_id": "FA"}}
    flex_ok = pro_ok = clin_ok = True
    for facing in (0.0, 45.0, 90.0, 200.0):
        q_wa = anatomical_frame_quat(facing)
        seg_quats = {"upper_arm_r": _to_world(ua_anat, q_wa),
                     "forearm_r": _to_world(fa_anat, q_wa)}
        cal = _frame_cal(seg_quats, facing)
        rep = compute_metrics(montage, t_ms, seg_quats, seg_meta, cal)
        elbow = next(j for j in rep["joints"] if j["key"] == "elbow_r")
        flex = next(d for d in elbow["dofs"] if d["key"] == "flex_ext")
        pro = next(d for d in elbow["dofs"] if d["key"] == "pro_sup")
        f_ok = (abs(flex["rom"]["range_deg"] - 90.0) < 0.5
                and flex["rom"]["max_deg"] > 89.5)       # flexion reads positive
        p_ok = abs(pro["rom"]["range_deg"]) < 0.5
        flex_ok, pro_ok = flex_ok and f_ok, pro_ok and p_ok
        clin_ok = clin_ok and elbow["clinical"] is True
        print(f"           facing {facing:5.0f}°: flex_ext range "
              f"{flex['rom']['range_deg']:.2f}° (want 90) "
              f"{'OK' if f_ok else 'FAIL'}; pro_sup range "
              f"{pro['rom']['range_deg']:.2f}° (want 0) {'OK' if p_ok else 'FAIL'}")
    ok = ok and flex_ok and pro_ok and clin_ok
    # A pure forearm TWIST (about the long axis, anatomical Y) must land on
    # pro_sup, not flex_ext — the exact swap the world-axis bug produced.
    twist = np.stack([_q_axis([0, 1, 0], x) for x in sweep])
    q_wa = anatomical_frame_quat(30.0)
    tq = {"upper_arm_r": _to_world(ua_anat, q_wa), "forearm_r": _to_world(twist, q_wa)}
    trep = compute_metrics(montage, t_ms, tq, seg_meta, _frame_cal(tq, 30.0))
    tel = next(j for j in trep["joints"] if j["key"] == "elbow_r")
    t_flex = next(d for d in tel["dofs"] if d["key"] == "flex_ext")["rom"]
    t_pro = next(d for d in tel["dofs"] if d["key"] == "pro_sup")["rom"]
    twist_ok = abs(t_pro["range_deg"] - 90.0) < 0.5 and t_flex["range_deg"] < 0.5
    ok = ok and twist_ok
    print(f"           forearm twist 0->90°: pro_sup {t_pro['range_deg']:.2f}° "
          f"(want 90), flex_ext {t_flex['range_deg']:.2f}° (want 0) "
          f"{'OK' if twist_ok else 'FAIL'}")
    q_wa = anatomical_frame_quat(0.0)
    seg_quats = {"upper_arm_r": _to_world(ua_anat, q_wa),
                 "forearm_r": _to_world(fa_anat, q_wa)}
    cal = _frame_cal(seg_quats, 0.0)

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
    #     Calibrated offsets but NO facing -> axes unknown -> still relative-only.
    no_axes = {"segments": cal["segments"],
               "heading": {"source": "none", "confident": False},
               "anatomical_frame": build_anatomical_frame(
                   {"source": "none", "confident": False})}
    rep_na = compute_metrics(montage, t_ms, seg_quats, seg_meta, no_axes)
    el_na = next(j for j in rep_na["joints"] if j["key"] == "elbow_r")
    axes_ok = (el_na["clinical"] is False and "axes" in el_na.get("warning", "")
               and rep_na["anatomical_axes"] is False)
    ok = ok and axes_ok
    print(f"[selftest] calibrated but no facing -> relative-only (axes unknown): "
          f"{'OK' if axes_ok else 'FAIL'}")

    # (4) Unwrap: a flexion sweep crossing 180° must report its true range, not a
    #     spurious ~360° jump from the atan2 branch cut.
    big = np.radians(np.linspace(150.0, 210.0, n))       # 60° sweep across ±180
    fa_big = _to_world(np.stack([_q_axis([0, 0, 1], x) for x in big]), q_wa)
    sq = {"upper_arm_r": seg_quats["upper_arm_r"], "forearm_r": fa_big}
    rep3 = compute_metrics(montage, t_ms, sq, seg_meta, cal)
    fb = next(d for d in next(j for j in rep3["joints"] if j["key"] == "elbow_r")
              ["dofs"] if d["key"] == "flex_ext")
    unwrap_ok = abs(fb["rom"]["range_deg"] - 60.0) < 0.5
    ok = ok and unwrap_ok
    print(f"[selftest] sweep across ±180° unwrapped: range "
          f"{fb['rom']['range_deg']:.2f}° (want 60) "
          f"{'OK' if unwrap_ok else 'FAIL'}")

    # (5) Metric primitives with known answers.
    t8 = np.arange(400) * 20.0                    # 8 s @ 50 Hz
    fs8 = _fs_hz(t8)
    #   reps: start extended (a trough, −cos) and flex 5 times -> 5 reps.
    five = -30.0 * np.cos(2 * np.pi * (5 / 8.0) * (t8 / 1000.0))
    reps5 = count_reps(five)
    tremor = count_reps(3.0 * np.sin(2 * np.pi * 5 * (t8 / 1000.0)))   # <15° -> 0
    reps_ok = (reps5 == 5 and tremor == 0)
    #   SPARC: a single smooth speed bump is smoother (nearer 0) than a noisy one.
    smooth = np.exp(-((t8 - 4000) / 900.0) ** 2)
    jerky = smooth + 0.5 * np.abs(np.sin(2 * np.pi * 3 * (t8 / 1000.0))) * \
        rng.random(len(t8))
    sp_smooth, sp_jerky = sparc(smooth, fs8), sparc(jerky, fs8)
    sparc_ok = sp_smooth > sp_jerky
    #   cross-correlation: b lags a by 10 samples (0.2 s) -> peak ~1, lag ~+0.2 s.
    xc = cross_correlation(five, np.roll(five, 10), fs8)
    # peak_r < 1 because a 10-sample lag drops edge samples from the overlap.
    xc_ok = xc["peak_r"] > 0.9 and abs(xc["lag_s"] - 0.2) < 0.04
    ok = ok and reps_ok and sparc_ok and xc_ok
    print(f"[selftest] primitives: reps(5-cycle)={reps5} tremor={tremor} "
          f"{'OK' if reps_ok else 'FAIL'}; SPARC smooth {sp_smooth} > jerky "
          f"{sp_jerky} {'OK' if sparc_ok else 'FAIL'}; xcorr peak {xc['peak_r']} "
          f"lag {xc['lag_s']}s {'OK' if xc_ok else 'FAIL'}")

    # (6) Full bilateral integration: right side moves MORE than left, both do
    #     clean cyclic elbow flexion. Exercises reps + every derived tier through
    #     the real compute_metrics path.
    T = 8.0
    tb = np.arange(400) * 20.0
    tsec = tb / 1000.0

    def _cyc(axis, amp_deg, cycles, phase=0.0):
        th = np.radians(amp_deg) * np.sin(2 * np.pi * (cycles / T) * tsec + phase)
        return np.stack([_q_axis(axis, x) for x in th])

    # Shoulders raised in the frontal plane (about anatomical X) around 50° of
    # elevation, oscillating in elevation — the shoulder's primary DOF — so the
    # YXY split stays well away from its neutral/overhead singularity.
    elev = _q_axis([1, 0, 0], np.radians(50.0))
    torso = qnorm(np.tile(IDENTITY, (len(tb), 1)))          # trunk still
    ua_r = qmul(elev, _cyc([1, 0, 0], 20.0, 5))             # R shoulder, 5 cycles
    ua_l = qmul(elev, _cyc([1, 0, 0], 12.0, 3))             # L shoulder, 3 cycles
    # elbow flex phased to start at full extension (a trough) -> clean rep counts
    fa_r = qmul(ua_r, _cyc([0, 0, 1], 30.0, 5, -np.pi / 2))  # R elbow (range 60)
    fa_l = qmul(ua_l, _cyc([0, 0, 1], 20.0, 3, -np.pi / 2))  # L elbow (range 40)
    # Built in anatomical axes, then placed in the world for a subject facing 30°.
    q_wa6 = anatomical_frame_quat(30.0)
    segq = {s: _to_world(q, q_wa6) for s, q in (
        ("torso", torso), ("upper_arm_r", ua_r), ("forearm_r", fa_r),
        ("upper_arm_l", ua_l), ("forearm_l", fa_l))}
    cols = {"torso": "n0", "upper_arm_r": "n1", "forearm_r": "n2",
            "upper_arm_l": "n3", "forearm_l": "n4"}
    bmont = {"schema_version": "1.0", "subject": {"id": "S"},
             "session": {"id": "bilat"}, "calibration": {"captured": True},
             "nodes": [{"node_id": s, "column": cols[s], "segment": s,
                        "calibrated": True} for s in segq]}
    bmeta = {s: {"column": cols[s], "node_id": s} for s in segq}
    bcal = _frame_cal(segq, 30.0)
    brep = compute_metrics(bmont, tb, segq, bmeta, bcal)

    er = next(j for j in brep["joints"] if j["key"] == "elbow_r")
    el = next(j for j in brep["joints"] if j["key"] == "elbow_l")
    reps_int_ok = er["reps"]["count"] == 5 and el["reps"]["count"] == 3
    d = {x["target"]: x for x in brep["derived"]}
    sym = d.get("symmetry_elbow", {}).get("metrics", {})
    sym_ok = 30 <= sym.get("symmetry_index", 0) <= 50      # 60 vs 40 -> 40%
    asym = d.get("activity_asymmetry_forearm", {}).get("metrics", {})
    asym_ok = asym.get("asymmetry_index", 0) > 0 and (asym.get("use_ratio") or 0) > 1
    coord = d.get("coordination_r", {}).get("metrics", {})
    coord_ok = abs(coord.get("peak_r", 0)) > 0.8           # shoulder∥elbow in phase
    comp = d.get("compensation_shoulder_r", {}).get("metrics", {})
    comp_ok = comp.get("trunk_travel_deg", 99) < 5.0       # trunk was still
    integ_ok = reps_int_ok and sym_ok and asym_ok and coord_ok and comp_ok
    ok = ok and integ_ok
    print(f"[selftest] bilateral derived: reps R/L {er['reps']['count']}/"
          f"{el['reps']['count']} {'OK' if reps_int_ok else 'FAIL'}; "
          f"symmetry_elbow {sym.get('symmetry_index')} "
          f"{'OK' if sym_ok else 'FAIL'}; forearm asym idx "
          f"{asym.get('asymmetry_index')} {'OK' if asym_ok else 'FAIL'}; "
          f"coord_r r={coord.get('peak_r')} {'OK' if coord_ok else 'FAIL'}; "
          f"trunk travel {comp.get('trunk_travel_deg')}° "
          f"{'OK' if comp_ok else 'FAIL'}")

    #   posture_dwell only appears for a calibrated segment.
    ua_r_seg = next(s for s in brep["segments"] if s["segment"] == "upper_arm_r")
    dwell_ok = ("posture_dwell" in ua_r_seg
                and abs(sum(ua_r_seg["posture_dwell"]["fraction"]) - 1.0) < 1e-6)
    ok = ok and dwell_ok
    print(f"[selftest] posture_dwell present & normalized for calibrated segment: "
          f"{'OK' if dwell_ok else 'FAIL'}")

    # (6b) Shoulder labels: a forward raise (flexion, about anatomical Z) and a
    #      sideways raise (abduction, about anatomical X) from 30° to 60° must both
    #      read as ELEVATION 30->60, told apart by plane of elevation (±90° vs
    #      0/180°) — and elevation is the primary DOF that reps/symmetry use.
    nl = 100
    tl = np.arange(nl) * 20.0
    raise_ = np.radians(np.linspace(30.0, 60.0, nl))
    lmont = {"schema_version": "1.0", "subject": {"id": "S"},
             "session": {"id": "labels"}, "calibration": {"captured": True},
             "nodes": [{"node_id": s, "column": c, "segment": s, "calibrated": True}
                       for s, c in (("torso", "n0"), ("upper_arm_r", "n1"))]}
    lmeta = {s: {"column": c, "node_id": s}
             for s, c in (("torso", "n0"), ("upper_arm_r", "n1"))}
    label_ok, label_msg = True, []
    for move, axis, plane_ok in (
            ("flexion", [0, 0, 1], lambda p: abs(abs(p) - 90.0) < 0.5),
            ("abduction", [-1, 0, 0],
             lambda p: min(abs(p), abs(abs(p) - 180.0)) < 0.5)):
        q_wa_l = anatomical_frame_quat(120.0)
        lq = {"torso": _to_world(np.tile(IDENTITY, (nl, 1)), q_wa_l),
              "upper_arm_r": _to_world(
                  np.stack([_q_axis(axis, x) for x in raise_]), q_wa_l)}
        lrep = compute_metrics(lmont, tl, lq, lmeta, _frame_cal(lq, 120.0))
        lsh = next(j for j in lrep["joints"] if j["key"] == "shoulder_r")
        lel = next(d for d in lsh["dofs"] if d["key"] == "elevation")["rom"]
        lpl = next(d for d in lsh["dofs"] if d["key"] == "plane_elev")["rom"]
        m_ok = (abs(lel["min_deg"] - 30.0) < 0.5 and abs(lel["max_deg"] - 60.0) < 0.5
                and plane_ok(lpl["median_deg"])
                and lsh["reps"]["primary_dof"] == "elevation")
        label_ok = label_ok and m_ok
        label_msg.append(f"{move}: elevation {lel['min_deg']:.0f}->"
                         f"{lel['max_deg']:.0f}, plane {lpl['median_deg']:+.0f}°")
    ok = ok and label_ok
    print(f"[selftest] shoulder labels: {'; '.join(label_msg)}; primary=elevation "
          f"{'OK' if label_ok else 'FAIL'}")

    # (7) Singularity guard: a shoulder oscillating in the frontal plane THROUGH
    #     the neutral pole would, unguarded, report a spurious ~180° plane-of-
    #     elevation swing and an absurd peak velocity. The guard must flag the
    #     outer DOFs (defined_frac < 1) and keep peak velocity physical, while the
    #     middle DOF (elevation itself) stays fully defined.
    ns = 300
    tns = np.arange(ns) * 20.0
    thn = np.radians(35.0) * np.sin(2 * np.pi * (2 / 6.0) * (tns / 1000.0))
    ua_sing = np.stack([_q_axis([1, 0, 0], x) for x in thn])   # frontal, through 0
    smont = {"schema_version": "1.0", "subject": {"id": "S"},
             "session": {"id": "sing"}, "calibration": {"captured": True},
             "nodes": [
                 {"node_id": "T", "column": "n0", "segment": "torso",
                  "calibrated": True},
                 {"node_id": "U", "column": "n1", "segment": "upper_arm_r",
                  "calibrated": True}]}
    q_wa7 = anatomical_frame_quat(-60.0)
    ssegq = {"torso": _to_world(np.tile(IDENTITY, (ns, 1)), q_wa7),
             "upper_arm_r": _to_world(ua_sing, q_wa7)}
    smeta = {s: {"column": c, "node_id": s}
             for s, c in (("torso", "n0"), ("upper_arm_r", "n1"))}
    scal = _frame_cal(ssegq, -60.0)
    srep = compute_metrics(smont, tns, ssegq, smeta, scal)
    sh = next(j for j in srep["joints"] if j["key"] == "shoulder_r")
    flex_sh = next(d for d in sh["dofs"] if d["key"] == "plane_elev")  # outer slot
    elev_sh = next(d for d in sh["dofs"] if d["key"] == "elevation")   # middle slot
    guard_ok = (flex_sh.get("defined_frac", 1.0) < 1.0            # outer flagged
                and "defined_frac" not in elev_sh                # middle untouched
                and (flex_sh["rom"] is None
                     or flex_sh["velocity"]["peak_deg_s"] < 2000))  # no ∞ velocity
    ok = ok and guard_ok
    print(f"[selftest] shoulder singularity guard: plane_elev defined "
          f"{flex_sh.get('defined_frac')} (outer, flagged), elevation full "
          f"(middle), peak vel physical {'OK' if guard_ok else 'FAIL'}")

    print(f"\n[selftest] {'PASS' if ok else 'FAIL'} — Euler round-trip, injected "
          f"ROM recovery at any facing, anatomical-axis + calibration gating, wrap handling, metric primitives "
          f"(reps/SPARC/xcorr), the full derived tier, and the singularity guard.")
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
