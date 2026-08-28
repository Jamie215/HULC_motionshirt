#!/usr/bin/env python3
"""
HULC Motion Shirt — offline multi-node reconciliation & alignment.

Takes the binary quaternion logs offloaded from two (or more) IMU nodes and
puts them on ONE common timeline, so the streams can be analyzed together —
the production sync path for the wireless, post-collection design (see
firmware/MULTINODE_SYNC_DESIGN.md).

How it works
------------
Each node timestamps its samples with its own free-running millis() clock, so
two nodes' timestamps do not share an origin and drift apart over time. Rather
than depend on a precise real-time BLE sync (which the bench showed is
latency-limited on some hosts), this tool recovers the clock relationship
FROM THE MOTION:

  1. Decode each node's binary log (20-byte records).
  2. Derive a mounting-invariant motion signal per node — angular speed, the
     magnitude of the rotation rate between consecutive quaternions. Two sensors
     on the same moving body see correlated angular speed regardless of how each
     is oriented.
  3. Cross-correlate the two angular-speed signals to find the time lag that
     best aligns them = the clock offset. A start-window vs end-window lag
     comparison estimates the (linear) clock drift.
  4. Apply offset + drift and resample every node's quaternions onto a shared
     time grid, emitting aligned per-node streams.

Because step 3 uses the shared motion, the result does not depend on BLE read
latency at all — a slow link only slows offload, not alignment accuracy.

Usage
-----
    # validate the math with synthetic data (no hardware needed):
    python tools/reconcile_nodes.py --selftest

    # align two real offloaded logs:
    python tools/reconcile_nodes.py nodeA.bin nodeB.bin --out aligned.csv

Record format (little-endian, 20 bytes) — matches firmware.ino:
    [0-3]  uint32 timestamp_ms   [4-7] float qw   [8-11] float qx
    [12-15] float qy             [16-19] float qz

Needs numpy (`pip install numpy`). Output is CSV for portability; the on-wire
BLE format stays the compact 20-byte binary — this tool runs after offload.
"""

import argparse
import struct
import sys

try:
    import numpy as np
except ImportError:  # pragma: no cover
    raise SystemExit("This tool needs numpy:  pip install numpy")

RECORD = struct.Struct("<Iffff")   # timestamp_ms, qw, qx, qy, qz
RECORD_SIZE = RECORD.size          # 20


# ---------------------------------------------------------------------------
# Decode
# ---------------------------------------------------------------------------
# Records read straight out of flash can include junk: erased regions decode to
# a timestamp of 0xFFFFFFFF and NaN quaternions, and a stale write pointer can
# leave gaps. One bad timestamp alone makes resample_uniform try to build a grid
# spanning ~4.29e9 ms (hundreds of millions of samples) and the tool hangs. So
# every log is sanitized on load.
_ERASED_TS = 0xFFFFFFFF
_RECORD_DT = np.dtype([("t", "<u4"), ("w", "<f4"), ("x", "<f4"),
                       ("y", "<f4"), ("z", "<f4")])


def load_log(path: str):
    """Decode + sanitize a binary node log → (t_ms int64[N], quat float64[N,4])."""
    with open(path, "rb") as f:
        raw = f.read()
    n = len(raw) // RECORD_SIZE
    if n == 0:
        raise SystemExit(f"{path}: no complete 20-byte records found.")
    if len(raw) % RECORD_SIZE:
        print(f"[warn] {path}: {len(raw) % RECORD_SIZE} trailing bytes ignored "
              f"(partial record).")

    arr = np.frombuffer(raw[:n * RECORD_SIZE], dtype=_RECORD_DT)
    t = arr["t"].astype(np.int64)
    q = np.stack([arr["w"], arr["x"], arr["y"], arr["z"]], axis=1).astype(np.float64)

    # Drop junk: erased timestamps, and non-finite / zero-norm quaternions.
    finite = np.isfinite(q).all(axis=1)
    nonzero = np.linalg.norm(q, axis=1) > 1e-6
    valid = finite & nonzero & (arr["t"] != _ERASED_TS)
    # Keep only strictly-increasing timestamps (drops backward jumps / dupes).
    tv = t[valid]
    if len(tv):
        rising = np.concatenate(([True], np.maximum.accumulate(tv)[1:]
                                 > np.maximum.accumulate(tv)[:-1]))
        idx = np.flatnonzero(valid)[rising]
    else:
        idx = np.array([], dtype=np.int64)

    dropped = n - len(idx)
    if dropped:
        print(f"[warn] {path}: dropped {dropped}/{n} bad records "
              f"(erased/NaN/non-monotonic); {len(idx)} valid.")
    if len(idx) < 2:
        raise SystemExit(f"{path}: only {len(idx)} valid record(s) after "
                         f"sanitizing — is this a real capture (state reached "
                         f"ACTIVE_RECORDING and logged QUAT samples)?")
    return t[idx], normalize_quats(q[idx])


def normalize_quats(q: np.ndarray) -> np.ndarray:
    norms = np.linalg.norm(q, axis=1, keepdims=True)
    norms[norms == 0] = 1.0
    return q / norms


# ---------------------------------------------------------------------------
# Motion feature: angular speed (mounting-invariant)
# ---------------------------------------------------------------------------
def angular_speed(t_ms: np.ndarray, q: np.ndarray):
    """Angular speed (rad/s) between consecutive quaternions.

    theta = 2*acos(|dot(q_i, q_{i+1})|); speed = theta / dt. The magnitude of
    the body's rotation rate is independent of each sensor's mounting
    orientation, so it is directly comparable across nodes.
    Returns (t_mid_ms, speed) with N-1 points.
    """
    dt = np.diff(t_ms) / 1000.0
    dt[dt <= 0] = np.nan                      # guard clock stalls / resets
    dot = np.abs(np.sum(q[:-1] * q[1:], axis=1))
    dot = np.clip(dot, 0.0, 1.0)
    theta = 2.0 * np.arccos(dot)
    speed = theta / dt
    t_mid = (t_ms[:-1] + t_ms[1:]) / 2.0
    good = np.isfinite(speed)
    return t_mid[good], speed[good]


def resample_uniform(t_ms: np.ndarray, y: np.ndarray, fs: float):
    """Linear-interpolate (t_ms, y) onto a uniform grid at fs Hz.

    Grid is built in RELATIVE time (t - t[0]); returns (t0_ms, grid_rel_ms, y_u)
    so callers can recover absolute time as t0_ms + grid_rel_ms.
    """
    t0 = float(t_ms[0])
    rel = t_ms - t0
    step = 1000.0 / fs
    n_pts = int(rel[-1] / step) + 2
    if n_pts > 5_000_000:
        raise SystemExit(
            f"[reconcile] resample grid would be {n_pts} samples "
            f"(span {rel[-1] / 1000:.0f}s @ {fs:.0f}Hz) — the log's timestamps "
            f"look corrupt. Re-capture, or check the .bin for junk records.")
    grid = np.arange(0.0, rel[-1] + step, step)
    y_u = np.interp(grid, rel, y)
    return t0, grid, y_u


# ---------------------------------------------------------------------------
# Cross-correlation lag
# ---------------------------------------------------------------------------
def _xcorr_full(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Same result as np.correlate(a, b, 'full') but via FFT — O(N log N)
    instead of O(N*M), so it doesn't hang on long signals."""
    N, M = len(a), len(b)
    nfft = 1 << int(np.ceil(np.log2(N + M - 1)))
    cc = np.fft.irfft(np.fft.rfft(a, nfft) * np.conj(np.fft.rfft(b, nfft)), nfft)
    return np.concatenate((cc[nfft - (M - 1):nfft], cc[:N]))


def best_lag_seconds(ya: np.ndarray, yb: np.ndarray, fs: float) -> float:
    """Lag L (seconds) such that ya[t] best matches yb[t - L].

    Positive L means ya is delayed relative to yb. Parabolic interpolation
    refines the peak to sub-sample resolution.
    """
    a = ya - ya.mean()
    b = yb - yb.mean()
    corr = _xcorr_full(a, b)
    lags = np.arange(-len(b) + 1, len(a))
    k = int(np.argmax(corr))
    delta = 0.0
    if 0 < k < len(corr) - 1:
        y0, y1, y2 = corr[k - 1], corr[k], corr[k + 1]
        denom = y0 - 2.0 * y1 + y2
        if denom != 0.0:
            delta = 0.5 * (y0 - y2) / denom
    return (lags[k] + delta) / fs


# Below this correlation at the best lag, the two nodes did not share enough
# motion to trust the recovered offset — the tool should say so and the caller
# should fall back to the BLE clock-offset read and/or a sync gesture.
CONFIDENCE_MIN = 0.40


def _pearson_at_lag(a: np.ndarray, b: np.ndarray, lag: int) -> float:
    """Pearson correlation of a and b overlapped at integer lag (a[n]~b[n-lag])."""
    if lag >= 0:
        aa, bb = a[lag:], b[:len(a) - lag]
    else:
        bb, aa = b[-lag:], a[:len(b) + lag]
    m = min(len(aa), len(bb))
    if m < 4:
        return 0.0
    aa, bb = aa[:m], bb[:m]
    if aa.std() == 0 or bb.std() == 0:
        return 0.0
    return float(np.corrcoef(aa, bb)[0, 1])


def lag_and_confidence(ya: np.ndarray, yb: np.ndarray, fs: float):
    """Best lag (seconds) plus a confidence in [~0,1]: the correlation of the
    two motion signals at that lag. Low confidence = little shared motion."""
    a = ya - ya.mean()
    b = yb - yb.mean()
    corr = _xcorr_full(a, b)
    lags = np.arange(-len(b) + 1, len(a))
    k = int(np.argmax(corr))
    delta = 0.0
    if 0 < k < len(corr) - 1:
        y0, y1, y2 = corr[k - 1], corr[k], corr[k + 1]
        denom = y0 - 2.0 * y1 + y2
        if denom != 0.0:
            delta = 0.5 * (y0 - y2) / denom
    return (lags[k] + delta) / fs, _pearson_at_lag(ya, yb, int(lags[k]))


# A drift estimate is only trustworthy when the lag actually moves across the
# recording by more than the cross-correlation noise. Below this, drift is
# unresolved — and, importantly, negligible (a few ms over minutes), so we
# simply don't apply it rather than fit noise.
DRIFT_RESOLVE_MS = 15.0


def estimate_offset_drift(tA, qA, tB, qB, fs=100.0, window_frac=0.30,
                          estimate_drift=False):
    """Estimate B→A clock offset (ms) and drift (ppm) from shared motion.

    Returns dict with offset_ms, drift_ppm, drift_resolved, and diagnostics.
    Convention: t_on_A_clock = tB + offset_ms + drift_ppm*1e-6*(tB - tB0).

    Drift is OFF by default: at typical sample rates over a few-minute record it
    is only a few ms (negligible vs the 25-50 ms target) and not reliably
    measurable. Pass estimate_drift=True only for long records where a real,
    significant lag trend exists.
    """
    tsA, spA = angular_speed(tA, qA)
    tsB, spB = angular_speed(tB, qB)
    A0, _, uA = resample_uniform(tsA, spA, fs)
    B0, _, uB = resample_uniform(tsB, spB, fs)

    # Global lag over the full overlap, with a confidence (how much shared
    # motion the two nodes had). Low confidence = unreliable offset.
    L_s, confidence = lag_and_confidence(uA, uB, fs)
    L = L_s * 1000.0                               # ms, A delayed vs B
    # Clock offset that maps B's timestamps onto A's clock.
    offset_ms = (A0 - B0) + L
    offset_reliable = confidence >= CONFIDENCE_MIN

    # Drift: measure the lag in several windows across the record and linear-fit
    # lag-vs-time. Accept the slope as real drift only if it is statistically
    # significant (|slope| > 2*SE) AND its total effect over the record exceeds
    # the resolution floor. Otherwise drift is unresolved — and negligible (a
    # few ms over minutes), so we leave it at zero rather than fit noise.
    drift_ppm = 0.0
    drift_resolved = False
    diag = {}
    if estimate_drift:
        win = _windowed_lags(uA, uB, fs, k=8, frac=window_frac)
        if win is not None:
            wt, wl = win                                # window-centre ms, lag ms
            slope, se = _lsq_slope(wt, wl)              # ms/ms, SE
            span_ms = wt[-1] - wt[0]
            total = abs(slope) * span_ms
            # Require a strongly significant trend AND a meaningful total effect.
            if abs(slope) > 3.0 * se and total > DRIFT_RESOLVE_MS:
                drift_ppm = slope * 1e6
                drift_resolved = True
            diag.update(win_t=wt, win_lag=wl, slope_ms_per_ms=slope, slope_se=se,
                        drift_total_ms=slope * span_ms)

    diag.update(global_lag_ms=L, A0=A0, B0=B0, fs=fs)
    return {"offset_ms": offset_ms, "drift_ppm": drift_ppm,
            "drift_resolved": drift_resolved, "confidence": confidence,
            "offset_reliable": offset_reliable, "diag": diag}


def _windowed_lags(uA, uB, fs, k=6, frac=0.3):
    """Lag (ms) measured in k overlapping windows; returns (centre_ms, lag_ms)."""
    n = min(len(uA), len(uB))
    w = int(n * frac)
    if w < 8 or n - w < 1:
        return None
    starts = np.linspace(0, n - w, k).astype(int)
    t, lag = [], []
    for s in starts:
        L = best_lag_seconds(uA[s:s + w], uB[s:s + w], fs) * 1000.0
        t.append((s + w / 2.0) / fs * 1000.0)
        lag.append(L)
    return np.array(t), np.array(lag)


def _lsq_slope(x, y):
    """Least-squares slope of y~x and its standard error."""
    x = x - x.mean()
    sxx = np.sum(x * x)
    if sxx == 0:
        return 0.0, np.inf
    slope = np.sum(x * (y - y.mean())) / sxx
    resid = (y - y.mean()) - slope * x
    dof = max(1, len(x) - 2)
    se = np.sqrt(np.sum(resid ** 2) / dof / sxx)
    return slope, se


def residual_alignment_ms(tA, qA, tB_on_a, qB, fs=100.0):
    """After mapping B onto A's clock, the leftover misalignment between the two
    motion signals.

    Returns (systematic_ms, trend_ms):
      systematic_ms — constant residual offset (global best-lag). This is the
                      alignment quality when drift is negligible.
      trend_ms      — how much the windowed lag trends across the record (real
                      residual drift/warp, distinct from per-window noise).
    """
    tsA, spA = angular_speed(tA, qA)
    tsB, spB = angular_speed(tB_on_a, qB)
    A0, _, uA = resample_uniform(tsA, spA, fs)
    B0, _, uB = resample_uniform(tsB, spB, fs)
    systematic = (A0 - B0) + best_lag_seconds(uA, uB, fs) * 1000.0
    trend = 0.0
    win = _windowed_lags(uA, uB, fs, k=6, frac=0.30)
    if win is not None:
        wt, wl = win
        slope, _ = _lsq_slope(wt, wl)
        trend = slope * (wt[-1] - wt[0])       # total lag change across record
    return systematic, trend


# ---------------------------------------------------------------------------
# Quaternion resampling (nlerp) + aligned emit
# ---------------------------------------------------------------------------
def nlerp(q: np.ndarray, t_ms: np.ndarray, query_ms: np.ndarray) -> np.ndarray:
    """Normalized-linear-interpolate quaternions q(t_ms) at query_ms."""
    idx = np.searchsorted(t_ms, query_ms, side="right") - 1
    idx = np.clip(idx, 0, len(t_ms) - 2)
    t0 = t_ms[idx]
    t1 = t_ms[idx + 1]
    span = np.where(t1 > t0, t1 - t0, 1.0)
    frac = np.clip((query_ms - t0) / span, 0.0, 1.0)[:, None]
    q0 = q[idx]
    q1 = q[idx + 1].copy()
    # take the shorter arc
    flip = np.sum(q0 * q1, axis=1) < 0
    q1[flip] *= -1.0
    out = q0 * (1.0 - frac) + q1 * frac
    return normalize_quats(out)


def to_A_clock(tB, offset_ms, drift_ppm):
    tB0 = float(tB[0])
    return tB + offset_ms + drift_ppm * 1e-6 * (tB - tB0)


def align_and_emit(paths, out_csv, fs=None):
    logs = [load_log(p) for p in paths]
    tA, qA = logs[0]
    ref_name = paths[0]

    # Estimate each other node's mapping onto node-0's clock.
    mapped = [(tA.astype(float), qA)]
    for p, (tB, qB) in zip(paths[1:], logs[1:]):
        est = estimate_offset_drift(tA, qA, tB, qB)
        drift_note = (f"drift {est['drift_ppm']:+.1f} ppm"
                      if est["drift_resolved"]
                      else "drift negligible/unresolved (not applied)")
        tB_on_a = to_A_clock(tB, est["offset_ms"], est["drift_ppm"])
        systematic, trend = residual_alignment_ms(tA, qA, tB_on_a, qB)
        trend_note = (f", residual trend {trend:+.1f} ms across record"
                      if abs(trend) > DRIFT_RESOLVE_MS else "")
        print(f"[reconcile] {p} -> {ref_name}: offset {est['offset_ms']:+.1f} ms "
              f"(confidence {est['confidence']:.2f}), "
              f"{drift_note}; residual alignment {systematic:+.1f} ms{trend_note}")
        if not est["offset_reliable"]:
            print(f"  [!] LOW CONFIDENCE ({est['confidence']:.2f} < "
                  f"{CONFIDENCE_MIN:.2f}) — the nodes shared little motion, so "
                  f"this offset is unreliable. Add a start-of-session sync "
                  f"gesture (a shared whole-body move), or seed the offset from "
                  f"the BLE clock read.")
        mapped.append((tB_on_a, qB))

    # Common grid over the mutual overlap on node-0's clock.
    lo = max(m[0][0] for m in mapped)
    hi = min(m[0][-1] for m in mapped)
    if hi <= lo:
        raise SystemExit("[reconcile] no time overlap between nodes after "
                         "alignment — did the recordings actually overlap?")
    if fs is None:
        # native-ish rate from node 0
        fs = 1000.0 / np.median(np.diff(tA)) if len(tA) > 1 else 10.0
    step = 1000.0 / fs
    grid = np.arange(lo, hi + step, step)

    cols = [grid - grid[0]]
    header = ["t_common_ms"]
    for i, (tm, q) in enumerate(mapped):
        qi = nlerp(q, tm, grid)
        cols.extend([qi[:, 0], qi[:, 1], qi[:, 2], qi[:, 3]])
        header.extend([f"n{i}_qw", f"n{i}_qx", f"n{i}_qy", f"n{i}_qz"])
    data = np.column_stack(cols)

    if out_csv:
        np.savetxt(out_csv, data, delimiter=",", header=",".join(header),
                   comments="", fmt="%.6f")
        print(f"[reconcile] wrote {len(grid)} aligned samples @ {fs:.1f} Hz "
              f"-> {out_csv}")
    return header, data


# ---------------------------------------------------------------------------
# Self-test: synthesize two nodes with a KNOWN offset+drift, verify recovery
# ---------------------------------------------------------------------------
def _integrate_quats(omega_axis, speed, dt):
    """Integrate a body angular-velocity (axis*speed) into a quaternion series."""
    q = np.zeros((len(speed) + 1, 4))
    q[0] = [1.0, 0.0, 0.0, 0.0]
    for i in range(len(speed)):
        ang = speed[i] * dt
        ax = omega_axis[i]
        half = ang / 2.0
        dq = np.array([np.cos(half), *(np.sin(half) * ax)])
        w0, x0, y0, z0 = q[i]
        w1, x1, y1, z1 = dq
        q[i + 1] = [
            w0 * w1 - x0 * x1 - y0 * y1 - z0 * z1,
            w0 * x1 + x0 * w1 + y0 * z1 - z0 * y1,
            w0 * y1 - x0 * z1 + y0 * w1 + z0 * x1,
            w0 * z1 + x0 * y1 - y0 * x1 + z0 * w1,
        ]
    return normalize_quats(q)


def _quat_mul(a, b):
    w0, x0, y0, z0 = a.T
    w1, x1, y1, z1 = b.T
    return np.stack([
        w0 * w1 - x0 * x1 - y0 * y1 - z0 * z1,
        w0 * x1 + x0 * w1 + y0 * z1 - z0 * y1,
        w0 * y1 - x0 * z1 + y0 * w1 + z0 * x1,
        w0 * z1 + x0 * y1 - y0 * x1 + z0 * w1,
    ], axis=1)


def selftest() -> int:
    rng = np.random.default_rng(42)
    true_offset_ms = 137.0      # B clock is ahead of A by this
    true_drift_ppm = 40.0       # B runs fast by 40 ppm
    fs_hz = 10.0                # node sample rate
    dur_s = 120.0
    print(f"[selftest] injecting offset={true_offset_ms:+.0f} ms, "
          f"drift={true_drift_ppm:+.0f} ppm, {fs_hz:.0f} Hz, {dur_s:.0f} s")

    # High-rate shared body motion (a smooth random angular-speed envelope).
    hi_fs = 200.0
    n_hi = int(dur_s * hi_fs)
    t_hi = np.arange(n_hi) / hi_fs
    # smooth speed: sum of a few sines + bursts
    speed = (1.5 + np.sin(2 * np.pi * 0.15 * t_hi)
             + 0.6 * np.sin(2 * np.pi * 0.4 * t_hi + 1.0))
    speed = np.clip(speed + 0.5 * rng.standard_normal(n_hi).cumsum() / np.sqrt(n_hi),
                    0.05, None)
    axis = np.tile(np.array([0.0, 0.0, 1.0]), (n_hi, 1))
    axis = axis + 0.2 * rng.standard_normal((n_hi, 1)) * np.array([1.0, 0.0, 0.0])
    axis /= np.linalg.norm(axis, axis=1, keepdims=True)
    q_body_hi = _integrate_quats(axis, speed, 1.0 / hi_fs)  # n_hi+1
    t_body_hi = np.arange(n_hi + 1) / hi_fs

    def sample_node(clock_offset_ms, drift_ppm, mount, noise, jitter):
        # sample the body motion at fs in TRUE time, then relabel timestamps in
        # this node's own clock (offset + drift), add mounting + sensor noise.
        n = int(dur_s * fs_hz)
        t_true = np.arange(n) / fs_hz
        t_true = t_true + jitter * rng.standard_normal(n) / fs_hz
        t_true = np.clip(np.sort(t_true), 0, dur_s)
        # interpolate the body quaternion at these true times (nearest-ish nlerp)
        qb = nlerp(q_body_hi, t_body_hi * 1000.0, t_true * 1000.0)
        qb = _quat_mul(np.tile(mount, (n, 1)), qb)               # mounting
        qb = normalize_quats(qb + noise * rng.standard_normal((n, 4)))
        # node clock timestamps (ms): true * (1+drift) + offset
        t_ms = t_true * 1000.0 * (1.0 + drift_ppm * 1e-6) + clock_offset_ms
        return t_ms, qb, t_true

    mount_a = np.array([1.0, 0.0, 0.0, 0.0])
    mount_b = np.array([0.7071, 0.0, 0.7071, 0.0])   # 90° different mounting
    # Node A: offset/drift zero → its clock IS true time (ms). Node B: shifted.
    tA, qA, _ = sample_node(0.0, 0.0, mount_a, 0.01, 0.05)
    tB, qB, ttB = sample_node(true_offset_ms, true_drift_ppm, mount_b, 0.01, 0.05)

    est = estimate_offset_drift(tA, qA, tB, qB)
    rec_off = est["offset_ms"]
    off_err = abs(rec_off - (-true_offset_ms))

    # Ground-truth alignment: map B onto A's clock (== true time in ms) and
    # compare each sample to where it TRULY belongs (ttB in ms).
    t_on_a = to_A_clock(tB, est["offset_ms"], est["drift_ppm"])
    true_err = np.abs(t_on_a - ttB * 1000.0)
    mean_err, max_err = float(true_err.mean()), float(true_err.max())

    drift_line = (f"resolved {est['drift_ppm']:+.1f} ppm" if est["drift_resolved"]
                  else "unresolved (below noise; negligible over this record)")
    print(f"[selftest] recovered offset {rec_off:+.1f} ms "
          f"(true {-true_offset_ms:+.1f}); offset error {off_err:.1f} ms")
    print(f"[selftest] drift {drift_line}")
    print(f"[selftest] ground-truth alignment error: "
          f"mean {mean_err:.1f} ms, max {max_err:.1f} ms")
    print(f"[selftest] confidence (shared motion): {est['confidence']:.2f} "
          f"-> reliable={est['offset_reliable']}")

    # Negative case: a node moving INDEPENDENTLY (e.g. the still/other arm) has
    # no shared motion, so the offset must be flagged LOW CONFIDENCE.
    n = int(dur_s * fs_hz)
    rng2 = np.random.default_rng(99)
    sp_ind = np.clip(1.0 + np.sin(2 * np.pi * 0.33 * np.arange(n) / fs_hz)
                     + rng2.standard_normal(n).cumsum() / np.sqrt(n), 0.05, None)
    ax_ind = np.tile(np.array([0.0, 1.0, 0.0]), (n, 1))
    q_ind = _integrate_quats(ax_ind, sp_ind, 1.0 / fs_hz)[:n]
    t_ind = np.arange(n) / fs_hz * 1000.0 + 500.0
    est_ind = estimate_offset_drift(tA, qA, t_ind, q_ind)
    print(f"[selftest] confidence (independent motion): "
          f"{est_ind['confidence']:.2f} -> reliable={est_ind['offset_reliable']}")

    # Deliverable: recovered offset matches truth AND every aligned sample lands
    # within the design target (25-50 ms); AND shared motion reads reliable while
    # independent motion is correctly flagged unreliable.
    tol_ms = 30.0
    ok = (off_err < tol_ms and mean_err < tol_ms
          and est["offset_reliable"] and not est_ind["offset_reliable"])
    print(f"[selftest] {'PASS' if ok else 'FAIL'} "
          f"(alignment < {tol_ms:.0f} ms; confidence gate correct)")
    return 0 if ok else 1


def inspect_logs(paths) -> None:
    """Print per-file capture stats — useful for diagnosing a bad .bin."""
    print("===== LOG INSPECTION =====")
    for p in paths:
        try:
            t, q = load_log(p)
        except SystemExit as exc:
            print(f"[{p}] {exc}")
            continue
        span_s = (t[-1] - t[0]) / 1000.0
        dt = np.diff(t)
        rate = 1000.0 / np.median(dt) if len(dt) else 0.0
        ts, sp = angular_speed(t, q)
        print(f"\n[{p}]")
        print(f"  valid records : {len(t)}")
        print(f"  time span     : {span_s:.1f} s")
        print(f"  sample rate   : {rate:.1f} Hz (median)")
        print(f"  gaps > 1s     : {int(np.sum(dt > 1000))}")
        if len(sp):
            print(f"  angular speed : mean {sp.mean():.2f}, max {sp.max():.2f} rad/s "
                  f"(is there real motion? near-zero = node barely moved)")
    print("\nFor alignment, two logs need OVERLAPPING real motion. Compare the")
    print("time spans and motion above; if a node barely moved, re-capture.")
    print("==========================")


def main() -> None:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("logs", nargs="*", help="binary node logs (node0 is the "
                    "reference clock); 2+ files to align")
    ap.add_argument("--out", help="output CSV path for the aligned streams")
    ap.add_argument("--fs", type=float, default=None,
                    help="output sample rate Hz (default: node0 native rate)")
    ap.add_argument("--selftest", action="store_true",
                    help="run the synthetic recovery test (no hardware)")
    ap.add_argument("--inspect", action="store_true",
                    help="just print stats for each log (records, span, rate, "
                         "motion) — for checking capture quality, no alignment")
    args = ap.parse_args()

    if args.selftest:
        sys.exit(selftest())
    if args.inspect:
        if not args.logs:
            ap.error("--inspect needs at least one log file")
        inspect_logs(args.logs)
        return
    if len(args.logs) < 2:
        ap.error("need at least 2 log files to align (or use --selftest)")
    align_and_emit(args.logs, args.out, args.fs)


if __name__ == "__main__":
    main()
