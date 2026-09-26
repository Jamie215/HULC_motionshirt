#!/usr/bin/env python3
"""
HULC Motion Shirt — timing bench: how much do alignment and resampling cost?

Synthetic ground truth that goes through the REAL reconcile step (no OpenSim
needed). A protocol-shaped session (setup fidget, neutral hold, shoulder swing,
curls, pro/supination with abduction, full swing, rest) is built in anatomical
axes; each node then samples it the way the firmware does:

  * a sync gesture (trunk twist, arm held) right after the neutral hold
  * its own clock: random offset (±3 s) and drift (±30 ppm), 1 ms timestamps
  * ~8.3 Hz (120 ms ± 4 ms jitter), independent sample phase per node
  * STATIC mode as in firmware.ino: after 10 s without motion, one sample per
    STATIC interval (1 s since 2026-09-26, 5 s before; --static-interval)
    until motion resumes
    (woken within 0.5 s)
  * strap/sensor error: a slow wobble that scales with how fast the segment
    moves (soft tissue; RMS at full motion set per run) + 0.5° slow drift +
    0.1° jitter

The logs are written as real .bin files, aligned by reconcile_nodes.py under
each variant, calibrated (auto neutral window; facing from the torso node or
the elbow hinge), split into ISB angles by metrics.py, and scored against the
truth at the true time of every output sample.

Variants
  speed      clock sync from cross-correlated angular-SPEED magnitude (original)
  vector     clock sync from world angular-velocity VECTORS (current default)
  oracle     true clock offsets (what perfect sync would give)

Findings that shaped reconcile_nodes.py (2026-09-26, 4 seeds, 3° wobble):
  * vector sync fixes torso montages outright (speed sync paired the trunk's
    sync twist with an arm swing: 35 s errors); on upper arm + forearm it cuts
    the sync error from ~6 to ~2 ms. Both land within ~0.1° of the oracle.
  * a relative-motion offset refinement made sync slightly worse (dropped);
  * a continuous-time (RTS + cubic Hermite) trajectory instead of nlerp, and
    temporal smoothing of the joint angles, changed nothing measurable: at
    ~8 Hz the remaining error is SLOW (strap/soft-tissue wobble, calibration,
    STATIC-mode gaps), not frame-to-frame jitter (dropped).
  * the torso node spends most of an arm session in STATIC; at the old 5 s
    STATIC interval its slow drift was lost. 1 s (the rate the RV already runs
    at in STATIC): torso + upper arm shoulder axial rotation 6.3 -> 1.9° RMS,
    elevation 5.1 -> 2.5°, plane of elevation 9.5 -> 5.9°, for +3.5% samples.

Usage:  python tools/timing_bench.py [--seeds 3] [--wobble 3] [--out DIR]
"""

import argparse
import contextlib
import io
import json
import os
import struct
import sys
import tempfile

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from calibrate_segments import (  # noqa: E402
    qmul, qconj, qnorm, anatomical_frame_quat, build_calibration,
    choose_neutral_window, load_aligned,
)
from metrics import (  # noqa: E402
    apply_calibration, compute_joint_metrics, resolve_anatomical_frame,
    trim_to_analysis,
)
from motion_capabilities import JOINTS  # noqa: E402
import reconcile_nodes as rn  # noqa: E402

FINE_HZ = 240.0
DUR_S = 60.0
IDENT = np.array([1.0, 0.0, 0.0, 0.0])
REC = struct.Struct("<Iffff")
# firmware schedule model (firmware.ino): no MOTION for NOT_MOTION_TO_STATIC_MS
# (10 s) -> STATIC, logging one sample per STATIC_SAMPLE_INTERVAL_MS; MOTION
# returns to ACTIVE within one classifier report (500 ms). The classifier's
# MOTION threshold is not published: STILL/WAKE_RAD_S stand in for it.
STILL_RAD_S = 0.20
WAKE_RAD_S = 0.30
STATIC_ENTRY_S = 10.0
STATIC_INTERVAL_S = 1.0          # firmware STATIC_SAMPLE_INTERVAL_MS (was 5 s)


def axq(axis, deg):
    """Quaternions for rotations of deg (array) about a fixed axis."""
    deg = np.atleast_1d(np.asarray(deg, dtype=float))
    a = np.radians(deg) / 2.0
    v = np.asarray(axis, float) / np.linalg.norm(axis)
    return np.column_stack([np.cos(a), np.outer(np.sin(a), v)])


def rotvec_q(v):
    """Quaternions from rotation vectors (N,3), radians."""
    v = np.atleast_2d(v)
    ang = np.linalg.norm(v, axis=1)
    ax = v / np.where(ang[:, None] > 1e-12, ang[:, None], 1.0)
    return np.column_stack([np.cos(ang / 2), np.sin(ang / 2)[:, None] * ax])


def env(t, a, b, ramp=1.0):
    up = np.clip((t - a) / ramp, 0, 1); dn = np.clip((b - t) / ramp, 0, 1)
    return 0.5 - 0.5 * np.cos(np.pi * np.minimum(up, dn))


# ---------------------------------------------------------------------------
# Truth: anatomical rotations from neutral (X anterior, Y up, Z right)
# ---------------------------------------------------------------------------
def truth_session(t, rng):
    fidget = np.zeros((len(t), 3))                    # setup fidget, 0-5 s,
    fid = t < 5.0                                     # settling onto neutral
    walk = np.cumsum(rng.normal(0, 0.004, (fid.sum(), 3)), axis=0)
    fidget[fid] = (walk - walk[-1]) * env(t[fid], 0, 5.0, ramp=0.8)[:, None]
    # 10-13 s: SYNC GESTURE — trunk twists with the arm held to the side, so
    # every node (torso included) shares one motion for clock alignment
    sync = env(t, 10, 13, ramp=0.5) * 25 * np.sin(2 * np.pi * (t - 10) / 1.5)
    sway = env(t, 13, 58)
    tilt = sway * 6 * np.sin(2 * np.pi * t / 7.3)
    lst = sway * 4 * np.sin(2 * np.pi * t / 5.1 + 1)
    rot = sway * 8 * np.sin(2 * np.pi * t / 9.7) + sync
    s1 = env(t, 13, 21)                               # shoulder swing
    flex = s1 * 60 * (1 - np.cos(2 * np.pi * (t - 13) / 4.0))
    abd = s1 * 20 * (1 - np.cos(2 * np.pi * (t - 13) / 4.0))
    s2 = env(t, 21, 33)                               # curls, supinated
    eflex = s2 * 65 * (1 - np.cos(2 * np.pi * (t - 21) / 2.8))
    ps = -s2 * 50
    s3 = env(t, 33, 41)                               # pro/sup at abduction
    abd = abd + s3 * 60
    eflex = eflex + s3 * 80
    ps = ps + s3 * 70 * np.sin(2 * np.pi * (t - 33) / 2.7)
    s4 = env(t, 41, 51)                               # full swing
    flex = flex + s4 * (55 + 90 * np.sin(2 * np.pi * (t - 41) / 4.0 - 1.2))
    axial = s4 * 35 * np.sin(2 * np.pi * (t - 41) / 3.1)
    q_t = qnorm(qmul(qmul(axq([0, 0, 1], tilt), axq([1, 0, 0], lst)),
                     axq([0, 1, 0], rot)))
    q_t = qnorm(qmul(q_t, rotvec_q(fidget)))
    q_sh = qnorm(qmul(qmul(axq([0, 0, 1], flex), axq([-1, 0, 0], abd)),
                      axq([0, 1, 0], axial)))
    q_el = qnorm(qmul(axq([0, 0, 1], eflex), axq([0, 1, 0], ps)))
    q_ua = qnorm(qmul(qmul(q_t, q_sh), rotvec_q(1.5 * fidget)))
    q_fa = qnorm(qmul(q_ua, q_el))
    return {"torso": q_t, "upper_arm_r": q_ua, "forearm_r": q_fa}


def speed(q, t):
    d = np.clip(np.abs(np.sum(q[1:] * q[:-1], axis=1)), 0, 1)
    return np.r_[0.0, 2 * np.arccos(d) / np.diff(t)]


# ---------------------------------------------------------------------------
# Node sampling (the firmware's schedule) and .bin logs
# ---------------------------------------------------------------------------
def sample_times(t, spd, rng, static_interval_s=None):
    """True times at which one node logs a sample, following firmware.ino:
    ACTIVE logs every ~120 ms; after STATIC_ENTRY_S with no MOTION from the
    stability classifier it drops to STATIC (one sample per STATIC interval);
    MOTION wakes it back to ACTIVE within one classifier report (<= 0.5 s)."""
    interval = STATIC_INTERVAL_S if static_interval_s is None else static_interval_s
    out, now, still_since, static = [], float(rng.uniform(0, 0.12)), None, False
    end = t[-1]
    while now < end:
        k = min(int(now * FINE_HZ), len(spd) - 1)
        moving = spd[k] > STILL_RAD_S
        if static:
            nxt = now + interval
            j = np.nonzero(spd[k:min(len(spd), int(nxt * FINE_HZ))] > WAKE_RAD_S)[0]
            if j.size:
                now = t[k + j[0]] + rng.uniform(0.0, 0.5)
                static, still_since = False, None
                continue
            out.append(now); now = nxt
            continue
        out.append(now)
        if not moving:
            still_since = now if still_since is None else still_since
            if now - still_since > STATIC_ENTRY_S:
                static = True
        else:
            still_since = None
        now += 0.120 + rng.normal(0, 0.004)
    return np.array(out)


def write_node_log(path, t_true, q_sensor, off_ms, drift_ppm):
    clock = 10000.0 + off_ms + t_true * 1000.0 * (1 + drift_ppm * 1e-6)
    with open(path, "wb") as f:
        for c, q in zip(np.round(clock), q_sensor):
            f.write(REC.pack(int(c), *[float(v) for v in q]))


def slow_wobble(t, rms_deg, rng, envelope=None):
    """Smooth random rotation (~rms_deg RMS, 0.15–0.8 Hz). With `envelope`
    (0..1 per sample) its size follows it: soft tissue and straps shift when
    the segment moves, not while it is held still."""
    v = np.zeros((len(t), 3))
    for _ in range(3):
        f = rng.uniform(0.15, 0.8)
        v += np.outer(np.sin(2 * np.pi * f * t + rng.uniform(0, 6.3)),
                      rng.normal(0, np.radians(rms_deg) / np.sqrt(1.5), 3))
    if envelope is not None:
        v = v * np.asarray(envelope)[:, None]
    return rotvec_q(v)


def motion_envelope(t, spd, ts, full_rad_s=1.5, smooth_s=0.5):
    """0..1 motion level at times ts: the segment's angular speed relative to
    full_rad_s, smoothed over smooth_s so the artifact builds and decays."""
    k = max(1, int(smooth_s * FINE_HZ))
    lvl = np.convolve(np.clip(spd / full_rad_s, 0, 1), np.ones(k) / k, mode="same")
    return lvl[np.clip(np.round(ts * FINE_HZ).astype(int), 0, len(t) - 1)]


# ---------------------------------------------------------------------------
# One montage x one seed: logs -> variants -> errors
# ---------------------------------------------------------------------------
SEG_KEY = {"torso": 1, "upper_arm_r": 2, "forearm_r": 3}
DRIFT_DEG = 0.5            # slow fusion drift present even when still

VARIANTS = {
    "speed": dict(sync_method="speed"),      # the original sync cue
    "vector": dict(sync_method="vector"),    # the current default
    "oracle": dict(oracle=True),             # true clock offsets
}


def joint_series(t_ms, q_seg, q_wa, present):
    out = {}
    for jk, j in JOINTS.items():
        if j.proximal in present and j.distal in present:
            _, ser = compute_joint_metrics(jk, j, q_seg[j.proximal],
                                           q_seg[j.distal], t_ms, True, q_wa=q_wa)
            out[jk] = ser
    return out


def run_case(segs, seed, wobble, outdir, variants):
    rng = np.random.default_rng(seed)
    t = np.arange(0, DUR_S, 1 / FINE_HZ)
    truth = truth_session(t, rng)
    facing = float(rng.uniform(-180, 180))
    q_wa = anatomical_frame_quat(facing)
    d = os.path.join(outdir, f"{'_'.join(s.split('_')[0] for s in segs)}_s{seed}")
    os.makedirs(d, exist_ok=True)
    clocks, paths = [], []
    for i, seg in enumerate(segs):
        # independent random streams per segment and purpose, so changing one
        # setting (e.g. the STATIC interval, which changes how many samples are
        # drawn) leaves every other random draw — mountings, wobble, clocks — as is
        key = [seed, SEG_KEY[seg]]
        r_mount, r_sched, r_wob, r_jit, r_clk = (np.random.default_rng(key + [k])
                                                 for k in range(5))
        q_world = qnorm(qmul(q_wa, truth[seg]))        # bone = anatomical at neutral
        if seg == "torso":                              # sternum: sensor +z forward
            mount = qmul(axq([0, 1, 0], 90)[0], axq(r_mount.normal(size=3), 5)[0])
        else:
            mount = qnorm(r_mount.normal(size=4))
        ts = sample_times(t, speed(truth[seg], t), r_sched)
        n_samples = getattr(run_case, "n_samples", {})
        n_samples[seg] = n_samples.get(seg, 0) + len(ts)
        run_case.n_samples = n_samples
        idx = np.clip(np.round(ts * FINE_HZ).astype(int), 0, len(t) - 1)
        # soft-tissue / strap wobble that follows the motion, plus a small
        # constant slow drift (fusion error)
        env = motion_envelope(t, speed(truth[seg], t), ts)
        qs = qmul(qmul(q_world[idx], mount), slow_wobble(ts, wobble, r_wob, env))
        qs = qmul(qs, slow_wobble(ts, DRIFT_DEG, r_wob))
        qs = qnorm(qmul(qs, rotvec_q(r_jit.normal(0, np.radians(0.1) / np.sqrt(3),
                                                  (len(ts), 3)))))
        off, dr = ((0.0, 0.0) if i == 0 else
                   (r_clk.uniform(-3000, 3000), r_clk.uniform(-30, 30)))
        clocks.append((off, dr))
        p = os.path.join(d, f"N{i}.bin")
        write_node_log(p, ts, qs, off, dr)
        paths.append(p)
    montage = {"schema_version": "1.0", "subject": {"id": "BENCH"},
               "session": {"id": d}, "calibration": {"captured": True},
               "nodes": [{"node_id": f"N{i}", "column": f"n{i}", "segment": s,
                          "calibrated": True} for i, s in enumerate(segs)]}

    res = {}
    for vname, opts in variants.items():
        out_csv = os.path.join(d, f"{vname.strip('+')}.csv")
        kw = {k: v for k, v in opts.items() if k != "oracle"}
        if opts.get("oracle"):
            # true B->A mapping: A_clock = B_clock + (offA - offB) (drift ~0 here
            # to first order; pass the true drift difference as well)
            kw["offsets"] = [((clocks[0][0] - c[0]), (clocks[0][1] - c[1]))
                             for c in clocks[1:]]
        with contextlib.redirect_stdout(io.StringIO()):
            rn.align_and_emit(paths, out_csv, **kw)
        with open(rn.quality_path(out_csv)) as f:
            qual = json.load(f)
        t_ms, sq, meta = load_aligned(out_csv, montage)
        t0, t1, _ = choose_neutral_window(montage, t_ms, sq)
        with contextlib.redirect_stdout(io.StringIO()):
            cal = build_calibration(montage, t_ms, sq, meta, t0, t1, out_csv)
        qw = resolve_anatomical_frame(cal)
        qc, _ = apply_calibration(sq, cal)
        tt, qc, _ = trim_to_analysis(t_ms, qc, cal)
        est = joint_series(tt, qc, qw, set(segs))
        # truth at the TRUE time of each output sample (node-0 clock -> true)
        true_s = (tt + qual["grid_origin_ms"] - 10000.0) / 1000.0
        idx = np.clip(np.round(true_s * FINE_HZ).astype(int), 0, len(t) - 1)
        tru = joint_series(tt, {s: truth[s][idx] for s in segs}, IDENT, set(segs))
        errs = {}
        for jk in est:
            for dk in est[jk]:
                e = (est[jk][dk] - tru[jk][dk] + 180) % 360 - 180
                e = e[np.isfinite(e)]
                if e.size:
                    errs[f"{jk}.{dk}"] = float(np.sqrt(np.mean(e ** 2)))
        sync = []
        for n, c in zip(qual["nodes"][1:], clocks[1:]):
            sync.append(abs(n["offset_ms"] - (clocks[0][0] - c[0])))
        fd = cal["heading"].get("facing_deg")
        res[vname] = {"errs": errs, "sync_err_ms": sync,
                      "facing_src": cal["heading"].get("source"),
                      "facing_err_deg": (None if fd is None else
                                         abs((fd - facing + 180) % 360 - 180))}
    return res


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[2])
    ap.add_argument("--seeds", type=int, default=3)
    ap.add_argument("--wobble", type=float, default=3.0,
                    help="slow strap/sensor error per node, deg RMS")
    ap.add_argument("--out", default=os.path.join(tempfile.gettempdir(), "hulc_timing"))
    ap.add_argument("--variants", nargs="*", default=list(VARIANTS))
    ap.add_argument("--per-seed", action="store_true", help="print every seed")
    ap.add_argument("--static-interval", type=float, default=STATIC_INTERVAL_S,
                    help="firmware STATIC_SAMPLE_INTERVAL_MS to model, seconds")
    args = ap.parse_args()
    globals()["STATIC_INTERVAL_S"] = args.static_interval
    variants = {k: VARIANTS[k] for k in args.variants}
    montages = [["upper_arm_r", "forearm_r"], ["torso", "upper_arm_r"],
                ["torso", "upper_arm_r", "forearm_r"]]
    table = {}
    for segs in montages:
        name = "+".join(s.split("_")[0] for s in segs)
        runs = [run_case(segs, seed, args.wobble, args.out, variants)
                for seed in range(args.seeds)]
        table[name] = runs
        dofs = sorted({k for r in runs for v in r.values() for k in v["errs"]})
        print(f"\n=== {name}  (wobble {args.wobble}°, {args.seeds} seeds; RMS error °, "
              f"mean over seeds)")
        print("  " + f"{'variant':<10}" + "".join(f"{k.split('.')[-1] + '(' + k.split('_')[0] + ')':>18}" for k in dofs)
              + f"{'sync err ms':>14}")
        if args.per_seed:
            for si, r in enumerate(runs):
                for v in variants:
                    print(f"    seed {si} {v:<7} " + " ".join(
                        f"{k.split('.')[-1]}={r[v]['errs'].get(k, np.nan):.1f}" for k in dofs)
                        + f"  sync={np.mean(r[v]['sync_err_ms']):.0f}ms facing="
                        f"{r[v]['facing_src']}/{r[v]['facing_err_deg']}")
        for v in variants:
            row = [np.mean([r[v]["errs"].get(k, np.nan) for r in runs]) for k in dofs]
            se = np.mean([np.mean(r[v]["sync_err_ms"]) for r in runs])
            print("  " + f"{v:<10}" + "".join(f"{x:18.2f}" for x in row) + f"{se:14.1f}")
    print(f"\nlogged samples per node (all seeds): "
          f"{getattr(run_case, 'n_samples', {})}")
    with open(os.path.join(args.out, "timing_results.json"), "w") as f:
        json.dump(table, f, indent=1)


if __name__ == "__main__":
    main()
