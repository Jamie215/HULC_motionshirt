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
The offset is a property of THIS mounting — a power cycle does NOT break it,
taking a node off (e.g. to charge it between blocks) and re-mounting it does.
Within one mounting it is cacheable. Alongside each offset we cache a
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

Which window is the neutral hold
--------------------------------
`choose_neutral_window`: an explicit `--window` wins; otherwise the montage's
`calibration.t_window_ms` is used only if the data there is actually still
(<= STILL_MAX_RAD_S) — a placeholder or mistimed window is ignored — and
otherwise `find_neutral_window` takes a deliberate hold — a still stretch
lasting at least NEUTRAL_MIN_HOLD_MS whose quietest window averages <=
NEUTRAL_QUIET_RAD_S — and that quietest window. A warm-up pause (short, and
only slowed to ~0.06–0.09 rad/s) does not qualify; it was taken for the hold on
a real capture and skewed every angle by 20–30°. WHICH hold depends on the
recording's order (`--protocol`, or the montage's `calibration.protocol`):
sync-first (default, the SOP) takes the first hold right after the sync
gesture — every node moving fast together — so setup stillness before it, of
any length, is never taken; hold-first (older recordings) takes the first hold
of the log. With no such hold, the first window under NEUTRAL_MAX_RAD_S, then
the quietest.

The end of the session — the `end` block
----------------------------------------
No closing pose is asked of the subject. `find_session_end` reads the end of
the log instead: taken-off nodes lie still on the charger in a pose no body
holds (some node tilted more than OFF_BODY_TILT_DEG from its neutral), then go
IDLE and stop logging. The analysis ends where the handling began — the last
still pause within HANDLING_MAX_MS before that rest, else HANDLING_MARGIN_MS
before it. A log ending in motion, or at rest in a body pose, is kept to its
end (the note says why). `--end` sets the end by hand. Calibrations written
before this carry a `closing` hold, still honoured by analysis_end_ms.

Heading (facing) recovery — the `heading` block
-----------------------------------------------
The shared world frame gives absolute orientation but not how the subject's
FORWARD lines up with world north, so a downstream skeleton could draw a forward
reach sideways. `compute_heading` recovers the facing from the torso node under
ONE coarse assumption (`TORSO_FORWARD_IN_SENSOR`: which torso-sensor axis points
out of the chest), and self-checks it — the axis must land ~horizontal at the
upright neutral pose, and the pose must be still — marking the result
low-confidence rather than confidently wrong. No torso -> not recovered. The
skeleton viewer applies a confident heading as a fixed yaw; nothing here asks the subject
to do or remember anything extra. Without a torso node the facing comes from
the elbow when the recording has enough elbow flexion
(`estimate_facing_from_elbow`: the facing that makes the upper-arm → forearm
motion a pure hinge, i.e. minimises varus/valgus; the sign of flexion resolves
the 180° ambiguity), or is given by hand (`--facing-deg`).

Anatomical axes — the `anatomical_frame` block
----------------------------------------------
The mounting offset zeroes each segment at neutral, but it leaves the segment's
AXES aligned with the world (X/Y = compass directions, Z = up), not with the body.
Joint angles are decomposed in ISB/Wu (2005) sequences that assume anatomical
axes, so once the facing is known we also emit the anatomical frame at neutral:

    X = anterior (the subject's forward), Y = superior (up, along a hanging
    limb), Z = X × Y = the subject's right  (right-handed; one frame for both
    sides — metrics.py mirrors left joints so their signs match the right)

as `q_WA` (world-from-anatomical). Because every calibrated segment is identity
at neutral, each segment's anatomical frame is q_seg ⊗ q_WA, and a joint's
anatomical relative rotation is conj(q_WA) ⊗ q_rel ⊗ q_WA — what metrics.py
decomposes. The offsets themselves are unchanged (the viewer and `verify` still
use the world-aligned zero). No confident facing -> no anatomical frame, and
metrics reports the joints RELATIVE-only.

Usage
-----
    # solve offsets + baseline from the neutral hold (auto-located when the
    # montage window isn't still), plus facing and the session end:
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
from motion_capabilities import JOINTS, validate_montage  # noqa: E402

SCHEMA_VERSION = "1.0"

# World "up" axis of the BNO rotation-vector reference frame (gravity-aligned).
# Only its direction matters, and only for the per-segment gravity check; the
# per-pair check is independent of it. If a firmware convention change moves the
# gravity axis, change this one constant.
WORLD_UP = np.array([0.0, 0.0, 1.0])

# ---- torso facing (heading) recovery — the ONE mounting assumption ----------
# The mag-referenced world gives absolute orientation, but not how the subject's
# FORWARD lines up with world "north" (+Y). We recover that heading from the
# torso node's neutral orientation, given ONE coarse assumption: which axis of
# the torso SENSOR points out of the chest (anterior). Default: the board normal,
# +Z in the sensor frame — a sensor lying flat on the sternum has its normal
# pointing forward. Change this one vector for a different torso mounting; the
# horizontality self-test below flags a badly-wrong guess rather than trusting it.
# Only the horizontal direction of this axis is used, so a board PITCHED up/down
# (a sloped chest) or ROLLED in its own plane still gives the right facing (pitch
# past HEADING_HORIZONTALITY_MAX_DEG is flagged low-confidence). A board YAWED
# toward one side — e.g. sitting off the sternum on a curved chest — shifts the
# facing by that same angle, undetected; keep the torso node on the sternum
# midline, or on the upper back between the shoulder blades (forward = -Z).
TORSO_FORWARD_IN_SENSOR = np.array([0.0, 0.0, 1.0])
# If the assumed forward axis lands more than this far from horizontal at the
# (upright) neutral pose, the mounting assumption is likely violated -> we mark
# the heading low-confidence and DON'T apply it, instead of facing the figure
# confidently wrong.
HEADING_HORIZONTALITY_MAX_DEG = 35.0

# Reuse-vs-re-pose thresholds (degrees). A quiet-standing pose repeats to within
# a few degrees; past these the mounting has moved enough to matter for angles.
DEFAULT_SEG_GRAVITY_DEG = 8.0    # per-segment tilt drift
DEFAULT_PAIR_ANGLE_DEG = 10.0    # per-pair relative-orientation drift

# A window whose mean angular speed exceeds this was not actually still — the
# "neutral pose" is contaminated by motion and the offset will be biased.
STILL_MAX_RAD_S = 0.30

DEFAULT_WIN_MS = 2000.0          # auto-detected still-window length
# Auto-detection looks for the protocol's neutral HOLD, which is much quieter
# than STILL_MAX_RAD_S (a deliberate hold sits around 0.01–0.05 rad/s); the
# stricter bar keeps a slow setup fidget from passing for it.
NEUTRAL_MAX_RAD_S = 0.10
# A deliberate hold is far stiller and longer than a pause in the warm-up: on
# real captures the hold's quietest 2 s sat at 0.004–0.017 rad/s for 9–42 s,
# while warm-up pauses sat at 0.06–0.09 rad/s for 2–4 s. So the first stretch
# that lasts the SOP's minimum hold AND reaches this quiet is the neutral hold;
# the 0.10 rule above is the fallback when no stretch does.
NEUTRAL_QUIET_RAD_S = 0.04
NEUTRAL_MIN_HOLD_MS = 3000.0

# ---- protocol order: sync gesture, THEN the neutral hold (the default) ------
# Nodes power up and wake one by one, so the start of a log is setup of any
# length, possibly with still stretches as long as a hold. The SOP therefore
# runs wake-up -> sync gesture -> neutral hold, and the hold is the first
# deliberate hold that starts within SYNC_TO_HOLD_MAX_MS after a stretch of
# shared fast motion: every node above SYNC_MIN_RAD_S (2 s mean) for at least
# SYNC_MIN_MS. On real captures the sync gesture held every node at 1.8–2.3
# rad/s; a warm-up is not followed by a hold, so it is passed over.
# "hold-first" (recordings made before this order) takes the first deliberate
# hold of the log instead.
PROTOCOLS = ("sync-first", "hold-first")
DEFAULT_PROTOCOL = "sync-first"
SYNC_MIN_RAD_S = 1.0
SYNC_MIN_MS = 3000.0
SYNC_TO_HOLD_MAX_MS = 20000.0
# A deliberate hold ending within this of the gesture's start is how a
# hold-first recording looks (on real ones: 0.2 s apart) -> calibrate hints at
# --protocol hold-first. A wait before a warm-up ends further off.
HOLD_FIRST_HINT_GAP_MS = 3000.0

# ---- facing from elbow motion (montages without a torso node) --------------
# The elbow must flex at least this far (95th percentile) for its hinge axis to
# be observable, and the best facing's varus/valgus RMS must be at most this
# fraction of the median over all facings (a clear minimum, not a flat cost).
# ---- end of the session: the nodes come off ---------------------------------
# No closing pose is asked for. When the nodes are taken off they end up lying
# on the charger, still, in a pose no body holds; they then go IDLE and stop
# logging, so the log ENDS with that rest. If some node is tilted more than
# OFF_BODY_TILT_DEG from its neutral there, the nodes are off the body, and the
# analysis ends where the handling began: at the last still pause within
# HANDLING_MAX_MS before the rest, else HANDLING_MARGIN_MS before it. Taking a
# node off is quick (real captures: 6.5 s after a pause; ~2.5 s straight from
# the arm hanging), and a longer look-back reached a slow moment mid-task and
# cut the last 21 s of a real session.
OFF_BODY_TILT_DEG = 60.0
HANDLING_MAX_MS = 10000.0
HANDLING_MARGIN_MS = 5000.0

HINGE_MIN_FLEX_DEG = 30.0
HINGE_MAX_COST_RATIO = 0.75


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


def _window_means(t_ms, seg_quats, win_ms):
    """(t0[k], mean speed over [t0, t0+win]) for every sample start that fits.

    Cumulative sums keep this O(N log N), so a long session stays cheap."""
    t_mid, speed = combined_speed(t_ms, seg_quats)
    ok = np.isfinite(speed)
    cs = np.concatenate([[0.0], np.cumsum(np.where(ok, speed, 0.0))])
    cn = np.concatenate([[0], np.cumsum(ok)])
    starts = t_ms[:-1][t_ms[:-1] + win_ms <= t_ms[-1]]
    lo = np.searchsorted(t_mid, starts, side="left")
    hi = np.searchsorted(t_mid, starts + win_ms, side="right")
    n = cn[hi] - cn[lo]
    with np.errstate(invalid="ignore", divide="ignore"):
        mean = np.where(n >= 2, (cs[hi] - cs[lo]) / np.maximum(n, 1), np.inf)
    return starts, mean


def find_sync_bursts(t_ms, seg_quats, win_ms=DEFAULT_WIN_MS):
    """Stretches of shared fast motion — the sync gesture and anything like it:
    every node's 2 s mean angular speed >= SYNC_MIN_RAD_S for >= SYNC_MIN_MS.
    Returns [(t0_ms, t1_ms), ...] in time order."""
    if len(t_ms) < 3 or t_ms[-1] - t_ms[0] <= win_ms:
        return []
    per = []
    for seg, q in seg_quats.items():
        starts, mean = _window_means(t_ms, {seg: q}, win_ms)
        per.append(mean)
    low = np.min(per, axis=0)
    out = []
    for i, j in _runs(low >= SYNC_MIN_RAD_S):
        b0, b1 = float(starts[i]), float(starts[j] + win_ms)
        if b1 - b0 >= SYNC_MIN_MS:
            out.append((b0, b1))
    return out


def _runs(mask):
    """(first, last) index of each run of True in `mask`."""
    idx = np.flatnonzero(mask)
    if idx.size == 0:
        return []
    return [(int(r[0]), int(r[-1]))
            for r in np.split(idx, np.flatnonzero(np.diff(idx) > 1) + 1)]


def _deliberate_holds(starts, mean, win_ms):
    """Still stretches that are deliberate holds: windows under
    NEUTRAL_MAX_RAD_S lasting >= NEUTRAL_MIN_HOLD_MS, the quietest one under
    NEUTRAL_QUIET_RAD_S. Returns [(stretch_start_ms, quietest_window_start_ms,
    stretch_end_ms)]."""
    out = []
    for i, j in _runs(mean <= NEUTRAL_MAX_RAD_S):
        k = i + int(np.argmin(mean[i:j + 1]))
        if (starts[j] + win_ms - starts[i] >= NEUTRAL_MIN_HOLD_MS
                and mean[k] <= NEUTRAL_QUIET_RAD_S):
            out.append((float(starts[i]), float(starts[k]),
                        float(starts[j] + win_ms)))
    return out


def find_neutral_window(t_ms, seg_quats, win_ms=DEFAULT_WIN_MS,
                        protocol=DEFAULT_PROTOCOL):
    """The protocol's neutral hold. Returns (t0, t1, source).

    A log starts with setup of any length (nodes powering up one by one,
    strapping on, a warm-up with pauses), so the hold is found by what it is —
    a deliberate hold: a still stretch lasting NEUTRAL_MIN_HOLD_MS whose
    quietest window is under NEUTRAL_QUIET_RAD_S (warm-up pauses are short and
    only slowed down) — and, by default, by WHERE it is:
      sync-first (default): the first deliberate hold starting within
        SYNC_TO_HOLD_MAX_MS after a burst of shared fast motion (the sync
        gesture, see find_sync_bursts). Still stretches before the gesture —
        waiting for nodes to start — never qualify.
      hold-first: the first deliberate hold of the log (the order used before).
    Falls back to the first deliberate hold, then the first window under
    NEUTRAL_MAX_RAD_S, then the quietest window — each named in `source`.
    The window returned is the stretch's quietest one (its edges still carry
    the settle-in)."""
    if len(t_ms) < 3 or t_ms[-1] - t_ms[0] <= win_ms:
        t0, t1 = auto_still_window(t_ms, seg_quats, win_ms)
        return t0, t1, "whole_record"
    starts, mean = _window_means(t_ms, seg_quats, win_ms)
    holds = _deliberate_holds(starts, mean, win_ms)
    if protocol == "sync-first":
        for b0, b1 in find_sync_bursts(t_ms, seg_quats, win_ms):
            after = [k for h0, k, _ in holds
                     if b1 - win_ms <= h0 <= b1 + SYNC_TO_HOLD_MAX_MS]
            if after:
                return after[0], after[0] + win_ms, "after_sync"
    if holds:
        k = holds[0][1]
        return k, k + win_ms, ("first_hold" if protocol == "hold-first"
                               else "first_hold_no_sync")
    still = mean <= NEUTRAL_MAX_RAD_S
    if still.any():
        # the first still stretch, then its quietest window (not its edge, which
        # still carries the settle-in)
        a = int(np.argmax(still))
        b = a + int(np.argmin(still[a:])) if not still[a:].all() else len(still)
        t0 = float(starts[a + int(np.argmin(mean[a:b]))])
        return t0, t0 + win_ms, "first_still"
    t0, t1 = auto_still_window(t_ms, seg_quats, win_ms)
    return t0, t1, "quietest"


def choose_neutral_window(montage, t_ms, seg_quats, win_ms=DEFAULT_WIN_MS,
                          window=None, protocol=None):
    """Resolve the calibration window: --window > a STILL montage window > auto.

    A montage window is only trusted if it was actually still — enrollment
    writes a placeholder, and calibrating on motion biases every offset.
    Returns (t0, t1, message)."""
    if window:
        t0, t1 = window
        return float(t0), float(t1), f"using --window {t0:.0f}–{t1:.0f} ms"
    cal_win = montage.get("calibration", {}).get("t_window_ms")
    note = ""
    if cal_win and len(cal_win) == 2:
        t0, t1 = float(cal_win[0]), float(cal_win[1])
        s = window_stillness(t_ms, seg_quats, t0, t1)
        if np.isfinite(s) and s <= STILL_MAX_RAD_S:
            return t0, t1, f"using montage neutral window {t0:.0f}–{t1:.0f} ms"
        note = (f"montage window {t0:.0f}–{t1:.0f} ms was not still "
                f"({s:.2f} rad/s) — ignoring it; ")
    # protocol order: --protocol > the montage's calibration.protocol > default
    protocol = (protocol or montage.get("calibration", {}).get("protocol")
                or DEFAULT_PROTOCOL)
    if protocol not in PROTOCOLS:
        raise SystemExit(f"[calibrate] unknown protocol {protocol!r} "
                         f"(choose: {', '.join(PROTOCOLS)})")
    t0, t1, src = find_neutral_window(t_ms, seg_quats, win_ms, protocol)
    bursts = find_sync_bursts(t_ms, seg_quats, win_ms) if src == "after_sync" else []
    before = [b for b in bursts if b[1] - win_ms <= t0]   # the gesture it follows
    sync = f"{before[-1][0]:.0f}–{before[-1][1]:.0f} ms" if before else ""
    if before:
        # a deliberate hold that ENDS right at the gesture is what a hold-first
        # recording looks like: say so instead of silently taking a later rest
        starts, mean = _window_means(t_ms, seg_quats, win_ms)
        prior = [(h0, h1) for h0, _, h1 in _deliberate_holds(starts, mean, win_ms)
                 for g0, _ in before
                 if h1 <= t0 and g0 - HOLD_FIRST_HINT_GAP_MS <= h1 <= g0 + win_ms]
        if prior:
            note += (f"a still hold ({prior[-1][0]:.0f}–{prior[-1][1]:.0f} ms) ends "
                     f"right before the sync gesture — if this recording held the "
                     f"neutral pose FIRST, re-run with --protocol hold-first. ")
    gap = (t0 - before[-1][1]) / 1000.0 if before else 0.0
    how = {"after_sync": f"freeze found {max(gap, 0):.0f} s after the sync movement "
                         f"({sync})",
           "first_hold": "freeze found (first of the log; protocol hold-first)",
           "first_hold_no_sync": "! no freeze right after a sync movement (every "
                                 f"node ≥ {SYNC_MIN_RAD_S} rad/s together) — "
                                 "using the first freeze of the log",
           "first_still": "! no clean freeze (≥ "
                          f"{NEUTRAL_MIN_HOLD_MS / 1000:.0f} s, ≤ {NEUTRAL_QUIET_RAD_S} "
                          "rad/s) — using the first still moment",
           "quietest": "! nothing still enough — using the quietest moment",
           "whole_record": "! record too short — using the whole record"}[src]
    return t0, t1, f"{note}{how}: {t0:.0f}–{t1:.0f} ms"


def analysis_start_ms(calibration, t_ms=None, seg_quats=None):
    """Where analysis should begin: the start of the neutral hold, or None.

    Everything before the neutral pose is setup (strapping on, fidgeting) and is
    not part of the protocol, so metrics and the viewer drop it by default.

    With the stream (t_ms, seg_quats), the window must also belong to THIS
    recording: inside its time span and still in its data. A calibration reused
    from an earlier session (verify -> reuse) carries that session's window
    times, which say nothing about where this recording's protocol starts —
    trimming on them would silently drop real data, so None is returned."""
    nw = (calibration or {}).get("neutral", {}).get("t_window_ms")
    if not nw or len(nw) != 2:
        return None
    t0, t1 = float(nw[0]), float(nw[1])
    if t_ms is not None and seg_quats is not None:
        if t0 < t_ms[0] or t1 > t_ms[-1]:
            return None
        s = window_stillness(t_ms, seg_quats, t0, t1)
        if not (np.isfinite(s) and s <= STILL_MAX_RAD_S):
            return None
    return t0


def _pose_deviation(t_ms, seg_quats, segments, t0, t1):
    """How far the pose in [t0, t1] is from the calibration's neutral pose.

    Heading-independent, like `verify`: per segment, the angle between where
    gravity points in the SENSOR frame now vs at neutral; per adjacent pair, the
    angle of their relative rotation now vs at neutral. Returns
    (max_segment_deg, max_pair_deg), or None if the window holds < 2 samples."""
    m = window_mask(t_ms, t0, t1)
    if m.sum() < 2:
        return None
    now = {seg: quat_average(q[m]) for seg, q in seg_quats.items()}
    seg_dev = 0.0
    for seg, qn in now.items():
        ref = segments.get(seg, {}).get("neutral_mean_quat")
        if ref is None:
            continue
        g_ref = qrotate(qconj(np.asarray(ref, float)), WORLD_UP)
        g_now = qrotate(qconj(qn), WORLD_UP)
        c = np.clip(np.dot(g_ref, g_now) / (np.linalg.norm(g_ref) * np.linalg.norm(g_now)),
                    -1.0, 1.0)
        seg_dev = max(seg_dev, float(np.degrees(np.arccos(c))))
    pair_dev = 0.0
    for prox, dist in _adjacent_pairs(now).values():
        ra, rb = segments.get(prox), segments.get(dist)
        if not ra or not rb:
            continue
        rel_ref = qmul(qconj(np.asarray(ra["neutral_mean_quat"], float)),
                       np.asarray(rb["neutral_mean_quat"], float))
        rel_now = qmul(qconj(now[prox]), now[dist])
        pair_dev = max(pair_dev, float(np.degrees(angle_between_quats(rel_ref, rel_now))))
    return seg_dev, pair_dev


def find_session_end(t_ms, seg_quats, segments, after_ms, win_ms=DEFAULT_WIN_MS):
    """Where the session stops being body motion: the nodes are taken off.

    The log ends when the nodes, laid on the charger, go IDLE — so its LAST
    still stretch is that rest. If some node is tilted more than
    OFF_BODY_TILT_DEG from its neutral there (gravity in the sensor frame, as
    in `verify`), the nodes are off the body: the analysis ends at the last
    still pause (one quiet window) within HANDLING_MAX_MS before the rest —
    the subject stopping before reaching for the straps — or, with none,
    HANDLING_MARGIN_MS before the rest. Never before `after_ms`.
    Returns the `end` block: t_end_ms (None = keep to the end of the log), how
    it was found, and the off-body rest."""
    if len(t_ms) < 3 or t_ms[-1] - after_ms < 2 * win_ms:
        return {"t_end_ms": None, "note": "log too short after the neutral hold"}
    starts, mean = _window_means(t_ms, seg_quats, win_ms)
    still = mean <= NEUTRAL_MAX_RAD_S
    if not still[-1]:
        return {"t_end_ms": None,
                "note": "the log ends in motion (nodes not seen coming off) — "
                        "analysis runs to the end of the log"}
    i = len(still) - 1
    while i > 0 and still[i - 1]:
        i -= 1
    rest0 = float(starts[i])
    if rest0 <= after_ms:
        return {"t_end_ms": None, "note": "no motion after the neutral hold"}
    k = i + int(np.argmin(mean[i:]))
    dev = _pose_deviation(t_ms, seg_quats, segments, float(starts[k]),
                          float(starts[k]) + win_ms)
    tilt = dev[0] if dev else 0.0
    rest = [round(rest0, 1), round(float(t_ms[-1]), 1)]
    if tilt <= OFF_BODY_TILT_DEG:
        return {"t_end_ms": None, "final_rest_ms": rest,
                "tilt_dev_deg": round(tilt, 1),
                "note": "the log ends at rest in a body pose (nodes still worn?) — "
                        "analysis runs to the end of the log"}
    # walk back over the handling to the last pause before it
    pause = [j for j in np.flatnonzero(still[:i])
             if rest0 - HANDLING_MAX_MS <= starts[j] + win_ms <= rest0
             and starts[j] >= after_ms]
    if pause:
        t_end, how = float(starts[pause[-1]] + win_ms), "pause_before_takeoff"
    else:
        t_end, how = max(after_ms + win_ms, rest0 - HANDLING_MARGIN_MS), \
            "margin_before_takeoff"
    return {"t_end_ms": round(t_end, 1), "method": how,
            "off_body_rest_ms": rest, "tilt_dev_deg": round(tilt, 1),
            "handling_s": round((rest0 - t_end) / 1000.0, 1)}


def analysis_end_ms(calibration, t_ms=None, seg_quats=None):
    """Where analysis should end — the calibration's `end` block (the nodes
    coming off, or a manual --end) — or None (keep to the end of the log).
    Like analysis_start_ms it only applies to THIS recording: with data given,
    the calibration's neutral hold must be this recording's own and the end
    must lie inside it. Calibrations written before the `end` block carry a
    `closing` hold instead; its end is honoured the same way."""
    cal = calibration or {}
    t1 = (cal.get("end") or {}).get("t_end_ms")
    if t1 is None:
        cw = (cal.get("closing") or {}).get("t_window_ms")
        t1 = cw[1] if cw and len(cw) == 2 else None
    if t1 is None:
        return None
    t1 = float(t1)
    if t_ms is not None and seg_quats is not None:
        if not (t_ms[0] < t1 <= t_ms[-1]):
            return None
        if analysis_start_ms(cal, t_ms, seg_quats) is None:
            return None
    return t1


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


def compute_heading(segments, still_ok, facing_deg=None):
    """Recover the subject's FACING (heading yaw) from the torso node.

    The skeleton view otherwise assumes the subject faced world +Y; if they
    faced elsewhere, a forward reach is drawn sideways. We fix that automatically
    (no protocol, no user input) from the torso, using the single mounting
    assumption `TORSO_FORWARD_IN_SENSOR`:

      forward_world = q_torso_neutral (x) forward_in_sensor
      facing_deg    = azimuth of forward_world about gravity, measured from +Y
      correction    = the yaw the viewer applies to the figure so facing -> +Y

    Self-checks (so it never faces the figure confidently wrong):
      * needs the torso node present;
      * the assumed forward axis must land ~horizontal at the upright neutral
        pose — if it tilts more than HEADING_HORIZONTALITY_MAX_DEG, the mounting
        assumption is suspect;
      * the neutral pose must have been still.
    Any failure -> confident=False and the viewer leaves facing uncorrected.

    `facing_deg` (from --facing-deg) states the facing by hand — the only way to
    get anatomical axes on a montage without a torso node. It overrides the torso
    estimate and is trusted as given.
    """
    if facing_deg is not None:
        f = round(float(facing_deg), 2)
        return {"source": "manual", "confident": True,
                "facing_deg": f, "correction_yaw_deg": f,
                "note": "facing stated by hand (--facing-deg)"}
    torso = segments.get("torso")
    if torso is None:
        return {"source": "none", "confident": False,
                "note": "no torso node — facing cannot be recovered"}

    q_bar = np.asarray(torso["neutral_mean_quat"], dtype=float)
    fwd = qrotate(q_bar, TORSO_FORWARD_IN_SENSOR.astype(float))
    fwd = fwd / (np.linalg.norm(fwd) or 1.0)
    # elevation above the horizontal plane (0 = perfectly horizontal / good)
    horiz_dev = float(np.degrees(np.arcsin(np.clip(abs(fwd[2]), 0.0, 1.0))))
    # azimuth of the horizontal component, measured clockwise from +Y (north).
    facing = float(np.degrees(np.arctan2(fwd[0], fwd[1])))
    horiz_ok = horiz_dev <= HEADING_HORIZONTALITY_MAX_DEG
    confident = bool(horiz_ok and still_ok)
    note = "ok"
    if not horiz_ok:
        note = (f"assumed chest-forward axis is {horiz_dev:.0f}° off horizontal "
                f"at neutral — torso mounting likely differs from the default; "
                f"facing left uncorrected")
    elif not still_ok:
        note = "neutral pose was not still — facing left uncorrected"
    # `facing` is the azimuth atan2(x, y); a figure rotation about +Z by that same
    # angle drives the azimuth to zero (new_az = az - angle), so the correction
    # the viewer applies EQUALS facing (not its negation).
    return {
        "source": "torso_auto",
        "confident": confident,
        "facing_deg": round(facing, 2),          # subject's forward azimuth from +Y
        "correction_yaw_deg": round(facing, 2),   # viewer rotates figure by this
        "horizontality_dev_deg": round(horiz_dev, 2),
        "forward_in_sensor": [float(v) for v in TORSO_FORWARD_IN_SENSOR],
        "note": note,
    }


def _elbow_splits(pairs, facing_deg):
    """ZXY split of each calibrated elbow's relative rotation, expressed in the
    anatomical frame for `facing_deg` (left mirrored) — metrics.py's chain."""
    from metrics import euler_from_quat, mirror_left   # local: metrics imports us
    q_wa = anatomical_frame_quat(facing_deg)
    out = []
    for side, q_rel in pairs:
        r = qnorm(qmul(qmul(qconj(q_wa), q_rel), q_wa))
        if side == "l":
            r = mirror_left(r)
        out.append(np.degrees(euler_from_quat(r, "ZXY")))
    return out


def estimate_facing_from_elbow(t_ms, seg_quats, segments, t_start, still_ok):
    """Recover the subject's facing from elbow motion (no torso node needed).

    The elbow is a hinge: in the right anatomical frame its relative rotation
    splits into flexion (about Z) and pro/supination (about the forearm, Y) with
    almost no varus/valgus (about X). With the wrong facing the hinge axis is
    misread, and flexion leaks into that middle slot. So the facing that
    minimizes the RMS varus/valgus over the session is the subject's facing.

    That criterion cannot tell f from f+180° (both put the hinge on the same
    line); the elbow's one-sided range settles it — flexion must be the positive
    direction. Confidence needs real flexion in the record and a clear minimum;
    a still or barely-flexing elbow leaves the facing unknown rather than guessed.
    Returns a heading dict (source "elbow_hinge") or None if no elbow pair.
    """
    pairs = []
    for side in ("r", "l"):
        ua, fa = f"upper_arm_{side}", f"forearm_{side}"
        if ua in seg_quats and fa in seg_quats and ua in segments and fa in segments:
            keep = t_ms >= t_start
            q_ua = qmul(seg_quats[ua][keep],
                        np.asarray(segments[ua]["mounting_offset_quat"], float))
            q_fa = qmul(seg_quats[fa][keep],
                        np.asarray(segments[fa]["mounting_offset_quat"], float))
            pairs.append((side, qnorm(qmul(qconj(q_ua), q_fa))))
    if not pairs or min(len(q) for _, q in pairs) < 10:
        return None

    def cost(f):
        e = np.concatenate([s[:, 1] for s in _elbow_splits(pairs, f)])
        return float(np.sqrt(np.mean(e * e)))

    grid = np.arange(0.0, 360.0, 2.0)
    costs = np.array([cost(f) for f in grid])
    f0 = grid[int(np.argmin(costs))]
    fine = np.arange(f0 - 2.0, f0 + 2.0001, 0.25)
    fine_costs = [cost(f) for f in fine]
    best = float(fine[int(np.argmin(fine_costs))]) % 360.0
    best_cost = float(min(fine_costs))
    contrast = best_cost / float(np.median(costs) or 1.0)

    def flex_pct(f):
        fl = np.concatenate([s[:, 0] for s in _elbow_splits(pairs, f)])
        fl = (fl + 180.0) % 360.0 - 180.0
        return float(np.percentile(fl, 5)), float(np.percentile(fl, 95))

    p5, p95 = flex_pct(best)
    if -p5 > p95:                      # flexion reads negative: other branch
        best = (best + 180.0) % 360.0
        p5, p95 = flex_pct(best)
    flex_ok = p95 >= HINGE_MIN_FLEX_DEG and p95 >= 2.0 * max(0.0, -p5)
    sharp = contrast <= HINGE_MAX_COST_RATIO
    confident = bool(flex_ok and sharp and still_ok)
    if confident:
        note = "ok"
    elif not flex_ok:
        note = (f"elbow flexion too small or two-sided (5–95%: {p5:.0f}…{p95:.0f}°)"
                f" — facing left unknown; flex the elbow, or pass --facing-deg")
    elif not sharp:
        note = (f"no clear hinge axis (varus/valgus {best_cost:.0f}° vs median "
                f"{np.median(costs):.0f}°) — facing left unknown")
    else:
        note = "neutral pose was not still — facing left unknown"
    f = round(best if best <= 180.0 else best - 360.0, 2)
    return {
        "source": "elbow_hinge",
        "confident": confident,
        "facing_deg": f,
        "correction_yaw_deg": f,
        "varus_valgus_rms_deg": round(best_cost, 2),
        "cost_contrast": round(contrast, 3),
        "flexion_p5_p95_deg": [round(p5, 1), round(p95, 1)],
        "elbows": [side for side, _ in pairs],
        "note": note,
    }


def _quat_from_rotmat(R):
    """Unit quaternion [w,x,y,z] of a proper rotation matrix (Shepperd's method)."""
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = 2.0 * np.sqrt(tr + 1.0)
        q = [0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s,
             (R[1, 0] - R[0, 1]) / s]
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        q = [(R[2, 1] - R[1, 2]) / s, 0.25 * s, (R[0, 1] + R[1, 0]) / s,
             (R[0, 2] + R[2, 0]) / s]
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        q = [(R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, 0.25 * s,
             (R[1, 2] + R[2, 1]) / s]
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        q = [(R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s,
             (R[1, 2] + R[2, 1]) / s, 0.25 * s]
    q = qnorm(np.asarray(q, dtype=float))
    return q if q[0] >= 0 else -q


def anatomical_frame_quat(facing_deg):
    """q_WA: world-from-anatomical frame for a subject facing `facing_deg`.

    Anatomical axes (ISB/Wu 2005): X = anterior, Y = superior, Z = right. The
    facing is the azimuth of the subject's forward, clockwise from world +Y (the
    same convention compute_heading reports), so forward = (sin f, cos f, 0).
    """
    f = np.radians(float(facing_deg))
    up = WORLD_UP / np.linalg.norm(WORLD_UP)
    fwd = np.array([np.sin(f), np.cos(f), 0.0])
    fwd = fwd - np.dot(fwd, up) * up
    fwd = fwd / np.linalg.norm(fwd)
    right = np.cross(fwd, up)
    return _quat_from_rotmat(np.column_stack([fwd, up, right]))


def build_anatomical_frame(heading):
    """The `anatomical_frame` block: q_WA when the facing is known, else why not."""
    if heading.get("confident") and "facing_deg" in heading:
        return {"axes": "X anterior, Y superior, Z right (ISB/Wu 2005)",
                "source": heading["source"],
                "facing_deg": heading["facing_deg"],
                "quat": _q_list(anatomical_frame_quat(heading["facing_deg"]))}
    return {"axes": "X anterior, Y superior, Z right (ISB/Wu 2005)",
            "source": heading.get("source", "none"), "quat": None,
            "note": "no confident facing — joint axes cannot be tied to anatomy; "
                    "add a torso node or pass --facing-deg"}


def build_calibration(montage, t_ms, seg_quats, seg_meta, t0, t1,
                      csv_path, targets=None, facing_deg=None, end_ms=None):
    stillness = window_stillness(t_ms, seg_quats, t0, t1)
    segments, pairs = solve_calibration(t_ms, seg_quats, seg_meta, t0, t1, targets)
    still_ok = bool(stillness <= STILL_MAX_RAD_S)
    end = ({"t_end_ms": round(float(end_ms), 1), "method": "manual"}
           if end_ms is not None else
           find_session_end(t_ms, seg_quats, segments, t1))
    heading = compute_heading(segments, still_ok, facing_deg)
    if heading["source"] == "none":
        # no torso, no stated facing: read it off the elbow's hinge motion
        hinge = estimate_facing_from_elbow(t_ms, seg_quats, segments, t0, still_ok)
        if hinge is not None:
            heading = hinge
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
            "still_ok": still_ok,
        },
        "thresholds": {
            "segment_gravity_deg": DEFAULT_SEG_GRAVITY_DEG,
            "pair_angle_deg": DEFAULT_PAIR_ANGLE_DEG,
        },
        # where the session ends: the nodes come off (see find_session_end)
        "end": end,
        "heading": heading,
        "anatomical_frame": build_anatomical_frame(heading),
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
    e = cal.get("end") or {}
    if e.get("t_end_ms") is not None:
        if e.get("method") == "manual":
            print(f"  session end:    {e['t_end_ms']:.0f} ms (--end) — analysis ends here")
        else:
            how = ("at the last pause before it" if e["method"] == "pause_before_takeoff"
                   else f"{HANDLING_MARGIN_MS / 1000:.0f} s before it")
            print(f"  session end:    {e['t_end_ms']:.0f} ms — nodes off the body from "
                  f"{e['off_body_rest_ms'][0]:.0f} ms (tilted {e['tilt_dev_deg']:.0f}°); "
                  f"analysis ends {how}")
    else:
        print(f"  session end:    end of the log — {e.get('note', 'not detected')}")
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
    h = cal.get("heading", {})
    if h.get("source") == "torso_auto":
        if h.get("confident"):
            print(f"\nFACING (auto from torso): subject faced "
                  f"{h['facing_deg']:+.0f}° from +Y; the FBD skeleton will rotate "
                  f"{h['correction_yaw_deg']:+.0f}° to face forward.")
        else:
            print(f"\nFACING (auto from torso): LOW CONFIDENCE — {h.get('note')}.")
    elif h.get("source") == "manual":
        print(f"\nFACING (stated by hand): subject faced {h['facing_deg']:+.0f}° "
              f"from +Y.")
    elif h.get("source") == "elbow_hinge":
        if h.get("confident"):
            print(f"\nFACING (from elbow hinge motion, {'+'.join(h['elbows'])}): "
                  f"subject faced {h['facing_deg']:+.0f}° from +Y "
                  f"(varus/valgus {h['varus_valgus_rms_deg']:.0f}° RMS, "
                  f"contrast {h['cost_contrast']:.2f}).")
        else:
            print(f"\nFACING (from elbow hinge motion): NOT confident — "
                  f"{h.get('note')}.")
    elif h.get("source") == "none":
        print("\nFACING: not recovered (no torso node) — the FBD skeleton's "
              "forward/side plane stays nominal.")
    af = cal.get("anatomical_frame", {})
    if af.get("quat"):
        print("ANATOMICAL AXES: set (X anterior, Y superior, Z right) — joint "
              "angles will be anatomical.")
    else:
        print("ANATOMICAL AXES: NOT set — joint angles will be RELATIVE-only "
              "(add a torso node or pass --facing-deg).")

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
    # utf-8-sig tolerates a UTF-8 BOM (Windows Notepad / PowerShell '>' add one),
    # which would otherwise crash json.load with "Expecting value: ... char 0".
    with open(path, encoding="utf-8-sig") as f:
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

    t0, t1, msg = choose_neutral_window(montage, t_ms, seg_quats, args.win_ms,
                                        args.window, args.protocol)
    print(f"[calibrate] {msg}")

    cal = build_calibration(montage, t_ms, seg_quats, seg_meta, t0, t1,
                            args.aligned_csv, facing_deg=args.facing_deg,
                            end_ms=args.end)
    cal["neutral"]["found_by"] = msg
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
    with open(args.calibration, encoding="utf-8-sig") as f:
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

    # (6) Facing recovery: build a torso whose assumed forward axis (sensor +Z)
    #     points horizontally at a KNOWN facing, and check compute_heading
    #     recovers it (applying the correction lands forward back on +Y). Also
    #     the no-torso and non-horizontal-mounting guards.
    def _q_from_to(u, v):                      # shortest-arc quaternion u -> v
        u = u / np.linalg.norm(u); v = v / np.linalg.norm(v)
        w = float(np.dot(u, v)) + 1.0
        xyz = np.cross(u, v)
        return qnorm(np.array([w, *xyz]))
    face_deg = 40.0
    q_face = _axis_angle([0, 0, 1], face_deg)  # subject yawed 40° about gravity
    q_flat = _q_from_to(TORSO_FORWARD_IN_SENSOR, np.array([0.0, 1.0, 0.0]))
    q_torso_neutral = qmul(q_face, q_flat)     # forward axis horizontal, faced 40°
    seg_h = {"torso": {"neutral_mean_quat": _q_list(q_torso_neutral)}}
    head = compute_heading(seg_h, still_ok=True)
    # apply the viewer's correction and confirm forward returns to ~+Y
    fwd = qrotate(q_torso_neutral, TORSO_FORWARD_IN_SENSOR.astype(float))
    fwd_corr = qrotate(_axis_angle([0, 0, 1], head["correction_yaw_deg"]), fwd)
    fwd_corr = fwd_corr / np.linalg.norm(fwd_corr)
    heading_ok = (head["source"] == "torso_auto" and head["confident"]
                  and abs(fwd_corr[0]) < 0.02 and fwd_corr[1] > 0.98)
    print(f"[selftest] facing recovery: faced {face_deg}°, recovered "
          f"{head['facing_deg']:.1f}° (horiz dev {head['horizontality_dev_deg']:.1f}°), "
          f"corrected forward -> [{fwd_corr[0]:+.2f},{fwd_corr[1]:+.2f}] (want ~[0,+1])")
    # no torso -> not recovered; forward axis tilted 90° -> low confidence
    no_torso = compute_heading({}, still_ok=True)
    tilted = compute_heading(
        {"torso": {"neutral_mean_quat": _q_list(np.array([1., 0, 0, 0]))}},
        still_ok=True)  # forward=+Z stays vertical -> not horizontal
    guards_ok = (no_torso["source"] == "none"
                 and tilted["source"] == "torso_auto" and not tilted["confident"])
    print(f"[selftest] facing guards: no-torso source='{no_torso['source']}', "
          f"vertical-forward confident={tilted['confident']} (want False)")

    # (7) Anatomical frame: q_WA maps X->forward, Y->up, Z->right for the
    #     stated facing, and a manual facing overrides a missing torso.
    frame_ok = True
    for f_deg, fwd_w, right_w in ((0.0, [0, 1, 0], [1, 0, 0]),
                                  (90.0, [1, 0, 0], [0, -1, 0]),
                                  (40.0, [np.sin(np.radians(40)),
                                          np.cos(np.radians(40)), 0],
                                   [np.cos(np.radians(40)),
                                    -np.sin(np.radians(40)), 0])):
        q_wa = anatomical_frame_quat(f_deg)
        got = [qrotate(q_wa, np.array(v, dtype=float))
               for v in ([1, 0, 0], [0, 1, 0], [0, 0, 1])]
        frame_ok = frame_ok and all(
            np.allclose(g, w, atol=1e-9)
            for g, w in zip(got, (fwd_w, WORLD_UP, right_w)))
    manual = compute_heading({}, still_ok=True, facing_deg=40.0)
    af_manual = build_anatomical_frame(manual)
    af_none = build_anatomical_frame(no_torso)
    frame_ok = (frame_ok and manual["source"] == "manual" and manual["confident"]
                and af_manual["quat"] is not None and af_none["quat"] is None)
    print(f"[selftest] anatomical frame: X->forward, Y->up, Z->right at 0/40/90° "
          f"facing, manual facing without torso, none when unknown: "
          f"{'OK' if frame_ok else 'FAIL'}")

    # (8) Protocol-shaped elbow session with NO torso: setup fidget, neutral hold,
    #     elbow curls with some pro/sup, a quieter rest at the end. The neutral
    #     finder must take the hold (not the quieter end, not a placeholder
    #     montage window over the fidget), and the elbow hinge must give back
    #     the subject's facing.
    face = 70.0
    q_wa = anatomical_frame_quat(face)
    to_world = lambda q: qnorm(qmul(qmul(q_wa, q), qconj(q_wa)))
    t_e = np.arange(0, 30000, 50.0)
    n_e = len(t_e)
    rng = np.random.default_rng(11)
    flex = np.zeros(n_e); ps = np.zeros(n_e); fidget = np.zeros((n_e, 3))
    setup = t_e < 4000
    fidget[setup] = np.cumsum(0.02 * rng.standard_normal((setup.sum(), 3)), axis=0)
    curls = (t_e >= 7000) & (t_e < 25000)
    ph = (t_e[curls] - 7000) / 3000.0 * 2 * np.pi
    flex[curls] = np.radians(60) * (1 - np.cos(ph))          # 0..120°
    ps[curls] = np.radians(30) * np.sin(0.5 * ph)
    ua_anat = qnorm(np.column_stack([np.ones(n_e), fidget]))
    fa_rel = np.stack([qmul(_axis_angle([0, 0, 1], np.degrees(a)),
                            _axis_angle([0, 1, 0], np.degrees(b)))
                       for a, b in zip(flex, ps)])
    fa_anat = qmul(ua_anat, fa_rel)
    mnt = {"upper_arm_r": _axis_angle([1, 2, 3], 70.0),
           "forearm_r": _axis_angle([3, -1, 2], 110.0)}
    elbow_q = {}
    for seg, qa in (("upper_arm_r", ua_anat), ("forearm_r", fa_anat)):
        q_ws = qmul(to_world(qa), qconj(mnt[seg]))
        noise = qnorm(np.column_stack([np.ones(n_e),
                                       0.0003 * rng.standard_normal((n_e, 3))]))
        elbow_q[seg] = qnorm(qmul(q_ws, noise))
    # rest at the end: dead still (quieter than the neutral hold's noise)
    rest = t_e >= 27000
    for seg in elbow_q:
        elbow_q[seg][rest] = elbow_q[seg][rest][0]
    placeholder = {"calibration": {"t_window_ms": [1000, 3000]}}
    n0, n1, nmsg = choose_neutral_window(placeholder, t_e, elbow_q)
    window_ok = 4000 <= n0 and n1 <= 7000 and "not still" in nmsg
    print(f"[selftest] neutral finder: {n0:.0f}–{n1:.0f} ms (hold 4000–7000, "
          f"placeholder over the fidget ignored: {'not still' in nmsg}) "
          f"{'OK' if window_ok else 'FAIL'}")
    meta_e = {s: {"column": f"n{i}", "node_id": f"E{i}"}
              for i, s in enumerate(elbow_q)}
    segs_e, _ = solve_calibration(t_e, elbow_q, meta_e, n0, n1)
    hinge = estimate_facing_from_elbow(t_e, elbow_q, segs_e, n0, True)
    dface = abs((hinge["facing_deg"] - face + 180) % 360 - 180)
    # a session with no flexion must NOT claim a facing
    flat_q = {s: q.copy() for s, q in elbow_q.items()}
    flat_q["forearm_r"] = qnorm(qmul(to_world(ua_anat), qconj(mnt["forearm_r"])))
    hinge_flat = estimate_facing_from_elbow(t_e, flat_q, segs_e, n0, True)
    hinge_ok = hinge["confident"] and dface < 3.0 and not hinge_flat["confident"]
    print(f"[selftest] facing from elbow hinge: faced {face:.0f}°, recovered "
          f"{hinge['facing_deg']:.1f}° (contrast {hinge['cost_contrast']:.2f}); "
          f"no-flexion session confident={hinge_flat['confident']} (want False) "
          f"{'OK' if hinge_ok else 'FAIL'}")

    # (9) Session end — the nodes come off. After the curls (end 25 s) the
    #     subject pauses 2 s, takes the nodes off (2 s of handling), and they
    #     lie on the charger tilted 70–80° from neutral until the log ends.
    #     The end is the pause (27 s). Without the pause, the end falls
    #     HANDLING_MARGIN_MS before the rest. A log that ends at rest in the
    #     body pose (the original elbow session) keeps to its end, and a
    #     calibration from another recording cuts nothing.
    def take_off(pause):
        keep = t_e < 25000
        t_a = t_e[keep]
        t_p = np.arange(25000, 25000 + pause, 50.0)
        t_h = np.arange(25000 + pause, 27000 + pause, 50.0)
        t_r = np.arange(27000 + pause, 34000 + pause, 50.0)
        out = {}
        for seg, q in elbow_q.items():
            last = q[keep][-1]
            frac = (t_h - t_h[0]) / (t_h[-1] - t_h[0])
            axis = [1, 0, 0] if seg == "forearm_r" else [0, 1, 0]
            full = 80.0 if seg == "forearm_r" else 70.0
            wig = 25.0 * np.sin(2 * np.pi * frac * 3)
            hand = np.stack([qmul(last, _axis_angle(axis, full * f + w))
                             for f, w in zip(frac, wig)])
            off = qmul(last, _axis_angle(axis, full))
            out[seg] = qnorm(np.vstack([q[keep], np.tile(last, (len(t_p), 1)), hand,
                                        np.tile(off, (len(t_r), 1))]))
        return np.concatenate([t_a, t_p, t_h, t_r]), out
    t_p2, q_p2 = take_off(2000)
    end_p = find_session_end(t_p2, q_p2, segs_e, n1)
    t_p0, q_p0 = take_off(0)
    end_m = find_session_end(t_p0, q_p0, segs_e, n1)
    end_worn = find_session_end(t_e, elbow_q, segs_e, n1)
    cal_p = {"neutral": {"t_window_ms": [n0, n1]}, "end": end_p}
    foreign = {"neutral": {"t_window_ms": [n0 + 200000, n1 + 200000]}, "end": end_p}
    end_ok = (end_p.get("method") == "pause_before_takeoff"
              and 26500 <= end_p["t_end_ms"] <= 27500
              and end_p["tilt_dev_deg"] > OFF_BODY_TILT_DEG
              and end_m.get("method") == "margin_before_takeoff"
              and end_m["t_end_ms"] < end_m["off_body_rest_ms"][0]
              and end_worn["t_end_ms"] is None
              and analysis_end_ms(cal_p, t_p2, q_p2) == end_p["t_end_ms"]
              and analysis_end_ms(foreign, t_p2, q_p2) is None
              and analysis_end_ms({"neutral": {"t_window_ms": [n0, n1]},
                                   "closing": {"t_window_ms": [27000, 29000]}})
              == 29000)
    print(f"[selftest] session end: take-off after a pause -> {end_p.get('t_end_ms')} "
          f"({end_p.get('method')}, want ~27000), straight from motion -> "
          f"{end_m.get('t_end_ms')} ({end_m.get('method')}), still worn at the end -> "
          f"{end_worn['t_end_ms']} (want None), foreign calibration cuts nothing, "
          f"old closing block honoured {'OK' if end_ok else 'FAIL'}")

    # (10) A warm-up pause is not the neutral hold: a 2.5 s pause at 0.07 rad/s
    #      (the speed real warm-up pauses showed) precedes the real 6 s hold at
    #      0.003 rad/s. The finder must skip the pause.
    t_w = np.arange(0, 20000, 100.0)
    rate = np.full(len(t_w), 1.0)
    rate[(t_w >= 3000) & (t_w < 5500)] = 0.07
    rate[(t_w >= 8000) & (t_w < 14000)] = 0.003
    ang = np.concatenate([[0.0], np.cumsum(rate[:-1] * 0.1)])
    q_w = np.column_stack([np.cos(ang / 2), np.zeros(len(t_w)), np.zeros(len(t_w)),
                           np.sin(ang / 2)])
    w0, w1, wsrc = find_neutral_window(t_w, {"torso": q_w, "upper_arm_r": q_w},
                                       protocol="hold-first")
    pause_ok = 8000 <= w0 and w1 <= 14000 and wsrc == "first_hold"
    print(f"[selftest] warm-up pause skipped: neutral {w0:.0f}–{w1:.0f} ms "
          f"(hold 8000–14000, pause at 3000–5500) "
          f"{'OK' if pause_ok else 'FAIL'}")

    # (11) Sync-first order: a 10 s still wait while nodes start (as still as a
    #      hold), a warm-up, the sync gesture (both nodes ~2 rad/s), THEN the
    #      neutral hold. sync-first must skip the wait and take the hold after
    #      the gesture; hold-first takes the wait (the old order's answer).
    t_s = np.arange(0, 40000, 100.0)
    rate = np.full(len(t_s), 0.6)                         # warm-up / task
    rate[t_s < 10000] = 0.003                             # waiting for nodes
    rate[(t_s >= 16000) & (t_s < 22000)] = 2.0            # sync gesture
    rate[(t_s >= 24000) & (t_s < 30000)] = 0.003          # neutral hold
    ang = np.concatenate([[0.0], np.cumsum(rate[:-1] * 0.1)])
    q_s = np.column_stack([np.cos(ang / 2), np.zeros(len(t_s)), np.zeros(len(t_s)),
                           np.sin(ang / 2)])
    both = {"torso": q_s, "upper_arm_r": q_s}
    s0, s1, ssrc = find_neutral_window(t_s, both)
    h0, h1, hsrc = find_neutral_window(t_s, both, protocol="hold-first")
    _, _, smsg = choose_neutral_window({}, t_s, both)
    sync_ok = (24000 <= s0 and s1 <= 30000 and ssrc == "after_sync"
               and h1 <= 10000 and hsrc == "first_hold"
               and "after the sync movement" in smsg and "hold-first" not in smsg)
    print(f"[selftest] sync-first: hold {s0:.0f}–{s1:.0f} ms after the gesture "
          f"(want 24000–30000; the still wait before it skipped), hold-first "
          f"-> {h0:.0f}–{h1:.0f} ms (want the wait) "
          f"{'OK' if sync_ok else 'FAIL'}")

    ok = (max_resid < 0.5 and max_pair < 0.5
          and window_ok and hinge_ok and end_ok and pause_ok and sync_ok
          and rep_reuse["decision"] == "reuse"
          and rep_repose["decision"] == "re-pose"
          and "forearm_r" in rep_repose["offenders"]
          and 1500 <= aw0 <= 2500
          and heading_ok and guards_ok and frame_ok)
    print(f"\n[selftest] {'PASS' if ok else 'FAIL'} "
          f"(offset recovery, heading-independent reuse, slip detection, "
          f"still-window search, neutral hold + session end finders, facing recovery + guards "
          f"(torso and elbow hinge), anatomical frame)")
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
    pc.add_argument("--end", type=float, metavar="T_MS",
                    help="end the analysis here (t_common_ms) instead of where "
                         "the nodes are detected coming off")
    pc.add_argument("--protocol", choices=PROTOCOLS,
                    help="order of the recording: sync-first (default; sync "
                         "gesture, then the neutral hold) or hold-first "
                         "(recordings made before; hold, then the gesture). "
                         "Default: the montage's calibration.protocol, else "
                         f"{DEFAULT_PROTOCOL}")
    pc.add_argument("--win-ms", type=float, default=DEFAULT_WIN_MS,
                    help="auto-detected window length in ms (default 2000)")
    pc.add_argument("--facing-deg", type=float, metavar="DEG",
                    help="subject's facing during the neutral pose, degrees "
                         "clockwise from world +Y (north); overrides the torso "
                         "estimate — needed for anatomical axes without a torso")
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
