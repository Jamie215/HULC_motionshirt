#!/usr/bin/env python3
"""
HULC Motion Shirt — stage-7 visual: the skeleton review viewer.

Orientation is exactly what the shirt measures — one magnetometer-referenced
quaternion per segment per frame. This tool bakes the reconcile → (optional)
calibrate stream into a single self-contained HTML viewer that connects the
segments into a 3-D stickman by forward kinematics (SETUP_AND_CALIBRATION_PLAN.md
§5, joint/chain tier): root the torso, place each segment's proximal end at its
parent's joint, orient the bone by its quaternion, step down the chain. The
CONNECTIVITY is real (the montage's kinematic chain, from
motion_capabilities.JOINTS); the bone LENGTHS and joint offsets are ASSUMED
anatomy (the `ANAT` table in the viewer). Missing nodes never break the figure
and never masquerade as measured:
  - a MISSING MIDDLE segment (e.g. torso + forearm, no upper arm) is drawn as a
    dashed "ghost" bone at rest, hung from the nearest measured ancestor's joint,
    and the measured descendant attaches to its end;
  - with NO torso but both arms placed (the bilateral asymmetry montage) each arm
    roots at a nominal shoulder and a fixed dashed girdle labeled "torso — not
    measured" bridges them;
  - an UNCALIBRATED bone (present but no cached offset) draws with an amber dashed
    overlay + "· raw" label, so a kink there reads as strap tilt, not motion.

Where the front is
------------------
The figure faces the subject's forward: the shoulders are placed on the
subject's left/right (from the calibration facing), a ground arrow marks FRONT
(with BACK / L / R around it), the chest face is lighter and the head carries a
nose. Front / Side / Top buttons snap the camera to those views. When the facing
is unknown the arrow reads "front?" and the forward direction is nominal.

Facing (heading) auto-correction
--------------------------------
The mag-referenced world gives orientation but not how the subject's forward
lines up with world "north", so a forward reach could otherwise draw sideways.
When calibration has a confident facing (recovered from the torso by
calibrate_segments.compute_heading, or stated with --facing-deg), the skeleton is
rotated by that fixed yaw about vertical so a forward reach draws forward. It is
captured once at neutral (so it never eats trunk motion), applied only when
confident, and always labeled. Otherwise facing stays nominal, honestly labeled.

What it shows — and why it validates stages 5-6
-----------------------------------------------
Feeding it a calibration.json makes the raw↔calibrated toggle the whole point:

  * RAW        each segment carries its unknown mounting tilt, so at the
               neutral pose the limbs sit crooked.
  * CALIBRATED the cached per-segment mounting offset q_SB is applied
               (q_seg = q_WS ⊗ q_SB), so at the neutral pose every calibrated
               limb hangs straight.

Jump to the neutral window and flip the toggle: if calibration worked, the
crooked figure straightens into a clean N-pose. That is stage 5 (the solve) and stage 6
(the calibrated stream) verified with your eyes, before any angle is computed.

The offsets are applied IN THE VIEWER (baked raw quats + one offset per
segment), so the toggle is instant and the file stays small. The quaternion
convention, CSV binding, and montage/body model are all imported from the
existing tools — this file never redefines them.

Usage
-----
    # bake a viewer from an aligned stream + montage (raw only):
    python tools/floating_fbd.py render aligned.csv montage.json --out fbd.html

    # with calibration -> raw<->calibrated toggle + neutral-window jump:
    python tools/floating_fbd.py render aligned.csv montage.json \
        --calibration calibration.json --out fbd.html

    # + metrics.json -> the stage-7 review panel (ROM/velocity/reps/derived)
    # reads out beside the body, sharing the raw<->calibrated toggle:
    python tools/floating_fbd.py render aligned.csv montage.json \
        --calibration calibration.json --metrics metrics.json --out review.html

    # validate end-to-end with no hardware (synth a session, bake, check):
    python tools/floating_fbd.py selftest
"""

import argparse
import json
import os
import sys

try:
    import numpy as np
except ImportError:  # pragma: no cover
    raise SystemExit("This tool needs numpy:  pip install numpy")

# One source of truth: reuse the quaternion math, CSV binding, and montage
# loader from the calibration stage; reuse the body model for segment names.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from calibrate_segments import (  # noqa: E402
    load_aligned, load_montage, qmul, qnorm, _q_list,
)
from motion_capabilities import SEGMENTS, JOINTS  # noqa: E402

SCHEMA_VERSION = "1.0"
IDENTITY_Q = [1.0, 0.0, 0.0, 0.0]
DEFAULT_MAX_FRAMES = 1500     # baked-frame cap (stride the record to fit)


# ---------------------------------------------------------------------------
# Baking — reconcile CSV (+ optional calibration) -> a compact frame table
# ---------------------------------------------------------------------------
def _stride_for(n, max_frames):
    """Sub-sampling stride so a length-n record bakes to <= max_frames frames."""
    if max_frames <= 0 or n <= max_frames:
        return 1
    return int(np.ceil(n / float(max_frames)))


def _parent_map(present):
    """Kinematic parent per present segment (the connectivity for the skeleton).

    The chain is REAL — it is the joint adjacency from the body model
    (motion_capabilities.JOINTS: proximal->distal), not a viewer guess. A
    segment's parent is the proximal side of a joint whose BOTH ends are placed
    this session; a segment with no present proximal (the torso, or an orphan
    whose parent node is missing) maps to None and becomes a root the viewer
    anchors nominally. Only the bone LENGTHS and joint offsets are assumed
    anatomy — those live in the viewer, clearly labeled as modeled.
    """
    parents = {s: None for s in present}
    for j in JOINTS.values():
        if j.distal in present and j.proximal in present:
            parents[j.distal] = j.proximal
    return parents


def _anat_chain():
    """The FULL anatomical parent per segment, independent of what's placed.

    Unlike `_parent_map` (which gates on presence), this is the complete
    body-model chain (torso←upper_arm←forearm←hand per side). The viewer walks
    it to bridge a MISSING middle segment: e.g. torso + forearm but no upper
    arm — the forearm still knows it descends from the torso through the (absent)
    upper arm, so the viewer can hang it off the shoulder with a dashed ghost
    upper arm rather than dropping it into space.
    """
    chain = {s: None for s in SEGMENTS}
    for j in JOINTS.values():
        chain[j.distal] = j.proximal
    return chain


def build_scene(csv_path, montage, calibration=None, max_frames=DEFAULT_MAX_FRAMES,
                metrics=None):
    """Bake a viewer-ready scene dict from the aligned stream.

    Bakes RAW world-from-sensor quaternions per segment plus, per segment, the
    single cached mounting offset (identity when uncalibrated). The viewer forms
    the calibrated orientation as q_seg = q_WS ⊗ q_SB on the fly, so both views
    come from one small payload.

    `metrics` (an optional metrics.py report dict) rides along in the scene so the
    stage-7 review panel reads out beside the 3-D body — one page, one payload. It
    holds only session SUMMARY stats (no per-frame arrays), so it stays compact.
    """
    t_ms, seg_quats, seg_meta = load_aligned(csv_path, montage)
    n = len(t_ms)
    stride = _stride_for(n, max_frames)
    keep = slice(0, n, stride)
    t_keep = t_ms[keep]

    cal_segments = (calibration or {}).get("segments", {})
    segments, frames = [], {}
    for seg, q in seg_quats.items():
        entry = cal_segments.get(seg)
        offset = entry["mounting_offset_quat"] if entry else list(IDENTITY_Q)
        segments.append({
            "segment": seg,
            "label": SEGMENTS.get(seg, seg),
            "node_id": seg_meta[seg].get("node_id"),
            "column": seg_meta[seg].get("column"),
            "calibrated": bool(entry),
            "offset": [round(float(v), 6) for v in _q_list(offset)],
        })
        # Round quats to keep the embedded JSON compact but visually exact.
        frames[seg] = [[round(float(v), 5) for v in qi]
                       for qi in qnorm(q[keep])]

    neutral_window = None
    if calibration:
        nw = calibration.get("neutral", {}).get("t_window_ms")
        if nw and len(nw) == 2:
            neutral_window = [float(nw[0]), float(nw[1])]

    has_cal = any(s["calibrated"] for s in segments)
    parents = _parent_map(set(seg_quats))
    present = set(seg_quats)
    # With no torso but both upper arms, the viewer bridges them with a fixed
    # assumed shoulder girdle so the arms read as one body.
    both_arms = {"upper_arm_l", "upper_arm_r"} <= present

    # Facing: the calibration step recovers the subject's heading from the torso
    # (calibrate_segments.compute_heading) or takes it from --facing-deg. The
    # viewer places the shoulders on the subject's left/right from it and rotates
    # the skeleton by `correction_yaw_deg` about vertical so the front faces the
    # FRONT marker — only when confident; otherwise it stays nominal.
    h = (calibration or {}).get("heading", {})
    heading = {
        "source": h.get("source", "none"),
        "confident": bool(h.get("confident")),
        "correction_yaw_deg": float(h.get("correction_yaw_deg", 0.0)),
        "facing_deg": h.get("facing_deg"),
        "note": h.get("note", ""),
    }

    return {
        "schema_version": SCHEMA_VERSION,
        "meta": {
            "subject": montage.get("subject", {}).get("id", "?"),
            "session": montage.get("session", {}).get("id", "?"),
            "source_csv": os.path.basename(csv_path),
            "calibration_source": (os.path.basename(calibration["_path"])
                                   if calibration and "_path" in calibration
                                   else None),
            "has_calibration": has_cal,
            "n_frames": int(len(t_keep)),
            "n_samples_total": int(n),
            "stride": int(stride),
            "t0_ms": float(t_keep[0]),
            "t1_ms": float(t_keep[-1]),
            "neutral_window_ms": neutral_window,
            "has_root": "torso" in seg_quats,
            "both_arms": both_arms,
            "heading": heading,
        },
        "parents": parents,
        "anat_chain": _anat_chain(),
        # joint -> [proximal, distal] segment, so a metrics-panel joint row can
        # highlight the two bones it spans in the 3-D view. Real body-model
        # adjacency (JOINTS), not a viewer guess.
        "joint_segments": {k: [j.proximal, j.distal] for k, j in JOINTS.items()},
        "segments": segments,
        "t_ms": [round(float(t), 1) for t in t_keep],
        "frames": frames,
        # stage-7 review panel: the metrics.py summary report (or null).
        "metrics": metrics,
    }


# ---------------------------------------------------------------------------
# HTML emit
# ---------------------------------------------------------------------------
def render_html(scene):
    data_json = json.dumps(scene, separators=(",", ":"))
    title = (f"Skeleton review — {scene['meta']['subject']} / "
             f"{scene['meta']['session']}")
    return (_HTML_TEMPLATE
            .replace("__TITLE__", _html_escape(title))
            .replace("/*__FBD_DATA__*/null", data_json))


def _html_escape(s):
    return (s.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;"))


# ---------------------------------------------------------------------------
# Commands
# ---------------------------------------------------------------------------
def _load_calibration(path):
    with open(path, encoding="utf-8-sig") as f:   # tolerate a UTF-8 BOM
        cal = json.load(f)
    cal["_path"] = path
    return cal


def _load_json(path):
    with open(path, encoding="utf-8-sig") as f:   # tolerate a UTF-8 BOM
        return json.load(f)


def cmd_render(args):
    montage = load_montage(args.montage)
    calibration = _load_calibration(args.calibration) if args.calibration else None
    metrics = _load_json(args.metrics) if args.metrics else None
    scene = build_scene(args.aligned_csv, montage, calibration, args.max_frames,
                        metrics=metrics)
    html = render_html(scene)
    # UTF-8 always: the page is <meta charset="utf-8"> and carries non-ASCII glyphs
    # (↔, °, ·, —). Without this, Python on Windows defaults to cp1252 and the
    # write dies with a UnicodeEncodeError.
    with open(args.out, "w", encoding="utf-8") as f:
        f.write(html)

    m = scene["meta"]
    print(f"[fbd] wrote {args.out}")
    print(f"[fbd] {len(scene['segments'])} segment(s), {m['n_frames']} frames "
          f"(stride {m['stride']} of {m['n_samples_total']}), "
          f"{m['t0_ms']:.0f}-{m['t1_ms']:.0f} ms")
    ncal = sum(1 for s in scene["segments"] if s["calibrated"])
    if calibration:
        print(f"[fbd] calibration applied to {ncal}/{len(scene['segments'])} "
              f"segment(s); raw<->calibrated toggle enabled.")
        if m["neutral_window_ms"]:
            print(f"[fbd] neutral window {m['neutral_window_ms'][0]:.0f}-"
                  f"{m['neutral_window_ms'][1]:.0f} ms — jump there and flip the "
                  f"toggle to see the mounting tilt straighten out.")
        h = m["heading"]
        print("[fbd] front: " + (f"known (subject faced {h['facing_deg']:.0f}°, "
              f"{h['source']})" if h["confident"] else
              "UNKNOWN — the FRONT marker is nominal (add a torso node or "
              "calibrate with --facing-deg)"))
    else:
        print("[fbd] no calibration given — RAW orientation only (each limb keeps "
              "its mounting tilt). Pass --calibration to enable the toggle.")
    if metrics:
        nj = len(metrics.get("joints", []))
        nd = len(metrics.get("derived", []))
        print(f"[fbd] metrics panel attached: {nj} joint(s), "
              f"{len(metrics.get('segments', []))} segment(s), {nd} derived — "
              f"the stage-7 review reads out beside the body.")
    else:
        print("[fbd] no metrics given — 3-D view only. Pass --metrics metrics.json "
              "for the stage-7 review panel.")
    print(f"[fbd] open it in a browser: file://{os.path.abspath(args.out)}")


# ---------------------------------------------------------------------------
# Self-test — synth a session, bake headless, assert structure
# ---------------------------------------------------------------------------
def _axis_angle(axis, deg):
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    h = np.radians(deg) / 2.0
    return np.array([np.cos(h), *(np.sin(h) * axis)])


def selftest():
    import tempfile
    from calibrate_segments import (qconj, build_calibration)

    print("[selftest] synthesizing a right-arm session (torso + upper_arm_r + "
          "forearm_r) ...")
    true_bone = {
        "torso":       _axis_angle([0, 0, 1], 4.0),
        "upper_arm_r": _axis_angle([1, 0, 0], 10.0),
        "forearm_r":   _axis_angle([0, 1, 0], 15.0),
    }
    mounting = {
        "torso":       _axis_angle([0, 1, 0], 25.0),
        "upper_arm_r": _axis_angle([1, 1, 0], 50.0),
        "forearm_r":   _axis_angle([0, 1, 1], 70.0),
    }
    t_ms = np.arange(0, 4000, 20.0)          # 4 s @ 50 Hz
    rng = np.random.default_rng(3)
    seg_quats, seg_meta = {}, {}
    for i, (seg, q_wb) in enumerate(true_bone.items()):
        q_ws = qmul(q_wb, qconj(mounting[seg]))
        arr = np.tile(q_ws, (len(t_ms), 1)) + 0.003 * rng.standard_normal(
            (len(t_ms), 4))
        seg_quats[seg] = qnorm(arr)
        seg_meta[seg] = {"column": f"n{i}", "node_id": f"HULC-IMU-{i:04d}"}

    # A throwaway montage + calibration built from the synth (via the real
    # calibration builder, so the offsets are the ones the viewer will apply).
    montage = {
        "schema_version": "1.0", "subject": {"id": "SELFTEST"},
        "session": {"id": "synth"},
        "calibration": {"neutral_pose": "N-pose", "captured": True},
        "nodes": [
            {"node_id": seg_meta[s]["node_id"], "column": seg_meta[s]["column"],
             "segment": s, "calibrated": True} for s in true_bone
        ],
    }
    cal = build_calibration(montage, t_ms, seg_quats, seg_meta,
                            float(t_ms[0]), float(t_ms[-1]), "synth.csv")
    cal["neutral"]["t_window_ms"] = [0.0, 4000.0]

    # Write an aligned CSV the real loader can read, then bake through the CLI path.
    with tempfile.TemporaryDirectory() as d:
        csv = os.path.join(d, "aligned.csv")
        header = ["t_common_ms"]
        cols = [t_ms]
        for i, s in enumerate(true_bone):
            header += [f"n{i}_q{c}" for c in ("w", "x", "y", "z")]
            cols += [seg_quats[s][:, k] for k in range(4)]
        np.savetxt(csv, np.column_stack(cols), delimiter=",",
                   header=",".join(header), comments="", fmt="%.6f")

        # a real metrics report over the same synth stream, so the bake carries
        # the stage-7 panel exactly as analyze_session produces it.
        from metrics import compute_metrics
        metrics_report = compute_metrics(montage, t_ms, seg_quats, seg_meta, cal)

        scene = build_scene(csv, montage, cal, max_frames=200,
                            metrics=metrics_report)
        html = render_html(scene)
        out = os.path.join(d, "fbd.html")
        with open(out, "w", encoding="utf-8") as f:
            f.write(html)
        html_size = os.path.getsize(out)

        # A no-torso BILATERAL montage over the same columns (relabel the two
        # non-torso nodes as the left/right upper arms) — the sparse case that
        # roots each arm at a nominal shoulder and bridges them with the assumed
        # girdle. We only need the baked meta/parents here.
        bl_montage = {
            "schema_version": "1.0", "subject": {"id": "BL"},
            "session": {"id": "bilat"}, "calibration": {"captured": False},
            "nodes": [
                {"node_id": "L", "column": "n1", "segment": "upper_arm_l"},
                {"node_id": "R", "column": "n2", "segment": "upper_arm_r"}],
        }
        scene_bl = build_scene(csv, bl_montage, None, max_frames=50)

        # A middle-gap montage: torso + forearm_r but NO upper arm. The forearm
        # still descends from the torso through the (absent) upper arm, which the
        # viewer bridges with a dashed ghost. We assert the baked chain that
        # drives that.
        gap_montage = {
            "schema_version": "1.0", "subject": {"id": "GAP"},
            "session": {"id": "gap"}, "calibration": {"captured": False},
            "nodes": [
                {"node_id": "T", "column": "n0", "segment": "torso"},
                {"node_id": "F", "column": "n2", "segment": "forearm_r"}],
        }
        scene_gap = build_scene(csv, gap_montage, None, max_frames=50)

    ok = True

    def check(cond, msg):
        nonlocal ok
        ok = ok and cond
        print(f"[selftest] {'ok ' if cond else 'FAIL'}: {msg}")

    # (1) Frame table shape: capped, all segments present, quats per frame.
    nf = scene["meta"]["n_frames"]
    check(nf <= 200 and nf > 0, f"frames capped to <=200 (got {nf})")
    check(set(scene["frames"]) == set(true_bone),
          "all placed segments baked")
    check(all(len(scene["frames"][s]) == nf for s in true_bone),
          "every segment has one quat per frame")
    check(len(scene["t_ms"]) == nf, "time axis length matches frames")

    # (2) Each baked segment carries its cached mounting offset (not identity).
    off = {s["segment"]: np.asarray(s["offset"]) for s in scene["segments"]}
    moved = all(float(np.degrees(2 * np.arccos(min(1.0, abs(off[s][0]))))) > 1.0
                for s in true_bone)
    check(moved, "each segment carries a non-identity mounting offset")

    # (3) The core promise: applying the baked offset to the baked RAW quats
    #     yields the calibrated orientation, which at neutral is ~identity
    #     (upright). This is exactly what the viewer's CALIBRATED toggle does.
    max_resid = 0.0
    for s in true_bone:
        raw = np.asarray(scene["frames"][s])           # (nf,4) world-from-sensor
        q_seg = qmul(raw, off[s][None, :])             # q_WS (x) q_SB
        q_mean = qnorm(q_seg.mean(axis=0))
        resid = float(np.degrees(2 * np.arccos(min(1.0, abs(q_mean[0])))))
        max_resid = max(max_resid, resid)
    check(max_resid < 2.0,
          f"offset(raw) collapses to upright at neutral: max {max_resid:.2f}° "
          f"(< 2°)")

    # (4) RAW at neutral is visibly NOT upright (that's what calibration fixes).
    max_raw_tilt = 0.0
    for s in true_bone:
        raw_mean = qnorm(np.asarray(scene["frames"][s]).mean(axis=0))
        max_raw_tilt = max(max_raw_tilt,
                           float(np.degrees(2 * np.arccos(min(1.0, abs(raw_mean[0]))))))
    check(max_raw_tilt > 10.0,
          f"raw neutral is tilted by the mounting: max {max_raw_tilt:.1f}° "
          f"(> 10°)")

    # (5) HTML is self-contained (no external script/CDN) and carries payload.
    check(html.count("/*__FBD_DATA__*/") == 0
          and '"frames"' in html and "getContext('2d')" in html
          and "<script src=" not in html,
          f"HTML has data injected + self-contained viewer ({html_size} bytes)")
    check('data-view="front"' in html and 'data-view="side"' in html
          and "data-layout" not in html and "renderFloating" not in html
          and "compass()" in html and "RIGHT_W" in html,
          "skeleton-only viewer with FRONT compass, Front/Side/Top views and "
          "facing-placed shoulders")
    check(scene["meta"]["neutral_window_ms"] == [0.0, 4000.0],
          "neutral window carried through for the jump button")

    # (5b) Stage-7 metrics panel: the baked page carries the metrics payload +
    #      the panel markup, and (this fully-calibrated synth) reads CLINICAL.
    jkeys = [j["key"] for j in metrics_report["joints"]]
    panel = ('id="metrics"' in html and '"metrics":' in html
             and "Session metrics" in html and 'class="flag clin"' in html
             and scene["metrics"] is not None
             and scene["joint_segments"].get("elbow_r") == ["upper_arm_r", "forearm_r"])
    check(panel, f"stage-7 metrics panel baked in ({len(jkeys)} joint(s): "
          f"{', '.join(jkeys)}); joint→segments map present")

    # (6) Kinematic chain baked for the skeleton layout: the parent map is the
    #     real joint adjacency — forearm hangs off upper arm, upper arm off
    #     torso, and the torso roots (no parent).
    par = scene["parents"]
    check(par.get("forearm_r") == "upper_arm_r"
          and par.get("upper_arm_r") == "torso"
          and par.get("torso") is None
          and scene["meta"]["has_root"] is True,
          f"skeleton chain: {par}")

    # (7) No-torso bilateral: no root, both arms present -> the arms root at
    #     nominal shoulders, bridged by the assumed girdle, and neither arm has a
    #     placed parent.
    mbl = scene_bl["meta"]
    check(mbl["has_root"] is False and mbl["both_arms"] is True
          and scene_bl["parents"].get("upper_arm_l") is None
          and scene_bl["parents"].get("upper_arm_r") is None,
          f"no-torso bilateral roots both arms at the girdle: {mbl}")

    # (7b) Facing: a torso session bakes a torso_auto heading block; a no-torso
    #      session bakes source 'none' (viewer leaves facing nominal).
    check(scene["meta"]["heading"]["source"] == "torso_auto"
          and "correction_yaw_deg" in scene["meta"]["heading"]
          and scene_bl["meta"]["heading"]["source"] == "none",
          f"heading baked: torso={scene['meta']['heading']['source']}, "
          f"no-torso={scene_bl['meta']['heading']['source']}")

    # (8) Middle gap (torso + forearm, no upper arm): the present-gated parent
    #     of the forearm is None (its node's parent isn't placed), but the FULL
    #     anatomical chain still routes forearm -> upper_arm -> torso, which is
    #     what lets the viewer bridge the missing upper arm with a ghost.
    ac = scene_gap["anat_chain"]
    check(scene_gap["parents"].get("forearm_r") is None
          and ac.get("forearm_r") == "upper_arm_r"
          and ac.get("upper_arm_r") == "torso"
          and ac.get("torso") is None,
          f"middle-gap chain: parents={scene_gap['parents']} anat={ac}")

    print(f"\n[selftest] {'PASS' if ok else 'FAIL'} — bake pipeline, the "
          f"raw↔calibrated promise, the stage-7 metrics panel, the skeleton "
          f"chain, the no-torso bilateral fallback, and the missing-middle ghost "
          f"chain.")
    return 0 if ok else 1


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd")

    pr = sub.add_parser("render", help="bake an aligned stream (+ calibration) "
                        "into a standalone HTML skeleton viewer")
    pr.add_argument("aligned_csv", help="reconcile_nodes.py output CSV")
    pr.add_argument("montage", help="montage JSON (column<->segment mapping)")
    pr.add_argument("--calibration", help="calibration.json from "
                    "calibrate_segments.py (enables raw<->calibrated toggle)")
    pr.add_argument("--metrics", help="metrics.json from metrics.py (adds the "
                    "stage-7 review panel: ROM / velocity / reps / derived)")
    pr.add_argument("--out", default="fbd.html",
                    help="output HTML file (default: fbd.html)")
    pr.add_argument("--max-frames", type=int, default=DEFAULT_MAX_FRAMES,
                    help=f"cap baked frames by striding (default "
                         f"{DEFAULT_MAX_FRAMES})")
    pr.set_defaults(func=cmd_render)

    ps = sub.add_parser("selftest", help="validate the bake pipeline (no hardware)")
    ps.set_defaults(func=lambda a: sys.exit(selftest()))

    args = ap.parse_args()
    if not getattr(args, "cmd", None):
        ap.error("choose a command: render | selftest")
    args.func(args)

# ---------------------------------------------------------------------------
# Viewer template — a fully self-contained standalone HTML page: NO external
# scripts, fonts, or CDNs (so it works offline and straight from file://). The
# 3-D is a small hand-rolled Canvas-2D renderer (oriented boxes, painter's-
# algorithm depth sort). The Python side injects the scene JSON at
# /*__FBD_DATA__*/null.
# ---------------------------------------------------------------------------
_HTML_TEMPLATE = r"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>__TITLE__</title>
<style>
  :root{
    --paper:#eef0f3; --surface:#ffffff; --ink:#16202b; --muted:#5a6672;
    --faint:#8a95a1; --line:#d7dde4; --accent:#0e83a0; --accent-ink:#0a6076;
    --built:#1a7f54; --planned:#a76500;
    --shadow:0 1px 2px rgba(22,32,43,.06),0 8px 24px rgba(22,32,43,.06);
    --sky1:#f3f6fa; --sky2:#e4e9f0; --grid:#c2cad3;
  }
  @media (prefers-color-scheme:dark){:root:not([data-theme="light"]){
    --paper:#0d131a; --surface:#151d26; --ink:#e7edf3; --muted:#9aa7b4;
    --faint:#6c7885; --line:#26313b; --accent:#35c1de; --accent-ink:#8fe0f0;
    --built:#3ecf8e; --planned:#e0a44a;
    --shadow:0 1px 2px rgba(0,0,0,.4),0 10px 28px rgba(0,0,0,.4);
    --sky1:#121a23; --sky2:#0c131b; --grid:#2b3743;
  }}
  *{box-sizing:border-box}
  html,body{margin:0;height:100%}
  body{background:var(--paper);color:var(--ink);
    font-family:ui-sans-serif,system-ui,-apple-system,"Segoe UI",Roboto,sans-serif;
    font-size:14px;line-height:1.5;-webkit-font-smoothing:antialiased;
    display:flex;flex-direction:column;height:100vh;overflow:hidden}
  .mono{font-family:ui-monospace,"SFMono-Regular",Menlo,Consolas,monospace}
  header{padding:14px 20px 12px;border-bottom:1px solid var(--line);
    background:var(--surface)}
  .eyebrow{font:600 11px/1 ui-monospace,monospace;letter-spacing:.14em;
    text-transform:uppercase;color:var(--accent-ink);margin-bottom:6px}
  h1{margin:0;font-size:17px;font-weight:600}
  .sub{color:var(--muted);font-size:12.5px;margin-top:3px}
  .stage{max-width:820px;margin-top:8px;color:var(--muted);font-size:12.5px}
  main{flex:1;position:relative;min-height:0}
  #view{position:absolute;inset:0;display:block;width:100%;height:100%;
    touch-action:none;cursor:grab}
  #view:active{cursor:grabbing}
  .hint{max-width:820px;margin-top:4px;color:var(--faint);font-size:11.5px}
  .legend{position:absolute;right:12px;top:12px;background:var(--surface);
    border:1px solid var(--line);border-radius:10px;padding:10px 12px;
    box-shadow:var(--shadow);max-width:250px}
  .legend h2{margin:0 0 8px;font-size:11px;font-weight:600;letter-spacing:.08em;
    text-transform:uppercase;color:var(--muted)}
  .legrow{display:flex;align-items:center;gap:8px;padding:2px 0;font-size:12px}
  .sw{width:12px;height:12px;border-radius:3px;flex:none}
  .badge{margin-left:auto;font:600 9.5px/1.4 ui-monospace,monospace;
    padding:2px 6px;border-radius:5px;letter-spacing:.04em}
  .badge.cal{background:color-mix(in srgb,var(--built) 20%,transparent);
    color:var(--built)}
  .badge.raw{background:color-mix(in srgb,var(--planned) 22%,transparent);
    color:var(--planned)}
  footer{border-top:1px solid var(--line);background:var(--surface);
    padding:11px 20px;display:flex;align-items:center;gap:14px;flex-wrap:wrap}
  button{font:inherit;cursor:pointer;border:1px solid var(--line);
    background:var(--surface);color:var(--ink);border-radius:8px;
    padding:7px 13px;font-weight:500}
  button:hover{border-color:var(--accent)}
  button.primary{background:var(--accent);border-color:var(--accent);color:#fff}
  button:disabled{opacity:.45;cursor:not-allowed}
  .toggle{display:inline-flex;border:1px solid var(--line);border-radius:8px;
    overflow:hidden}
  .toggle button{border:none;border-radius:0;padding:7px 12px}
  .toggle button.on{background:var(--accent);color:#fff}
  input[type=range]{flex:1;min-width:160px;accent-color:var(--accent)}
  .tlabel{font-variant-numeric:tabular-nums;color:var(--muted);font-size:12px;
    min-width:150px;text-align:right}

  /* ---- stage-7 metrics review panel (left drawer over the canvas) ---- */
  #metrics{position:absolute;left:0;top:0;bottom:0;width:352px;max-width:86vw;
    background:var(--surface);border-right:1px solid var(--line);
    box-shadow:var(--shadow);overflow-y:auto;padding:14px 16px 22px;z-index:5;
    transition:transform .18s ease}
  #metrics.hidden{transform:translateX(-102%)}
  #metrics h2{margin:0 0 2px;font-size:14px;font-weight:600}
  #metrics .msub{color:var(--muted);font-size:11.5px;margin-bottom:10px}
  #metrics .rawbanner{display:none;margin:0 0 12px;padding:7px 10px;
    border-radius:8px;font-size:11.5px;
    background:color-mix(in srgb,var(--planned) 16%,transparent);
    color:var(--planned);border:1px solid color-mix(in srgb,var(--planned) 34%,transparent)}
  body[data-mode="raw"] #metrics .rawbanner{display:block}
  #metrics section{margin:0 0 14px}
  #metrics section>h3{margin:0 0 7px;font:600 10.5px/1.3 ui-monospace,monospace;
    letter-spacing:.1em;text-transform:uppercase;color:var(--muted);
    border-bottom:1px solid var(--line);padding-bottom:4px}
  .mcard{border:1px solid var(--line);border-radius:9px;padding:8px 10px;
    margin-bottom:7px;cursor:default}
  .mcard.hl{border-color:var(--accent);
    box-shadow:0 0 0 1px var(--accent) inset}
  .mcard .mhead{display:flex;align-items:baseline;gap:7px;flex-wrap:wrap}
  .mcard .mname{font-weight:600;font-size:12.5px}
  .mcard .mmeta{color:var(--faint);font-size:11px;margin-left:auto;
    font-family:ui-monospace,monospace}
  .flag{font:600 9px/1.4 ui-monospace,monospace;padding:2px 6px;border-radius:5px;
    letter-spacing:.03em;text-transform:uppercase}
  .flag.rel{background:color-mix(in srgb,var(--planned) 20%,transparent);
    color:var(--planned)}
  .flag.clin{background:color-mix(in srgb,var(--built) 18%,transparent);
    color:var(--built)}
  .flag.blk{background:color-mix(in srgb,#c0392b 20%,transparent);color:#c0392b}
  .dof{display:grid;grid-template-columns:1fr auto;gap:2px 10px;
    font-size:11.5px;padding:4px 0 2px;border-top:1px dashed var(--line);
    margin-top:5px}
  .dof:first-of-type{border-top:none;margin-top:3px}
  .dof .dname{color:var(--muted)}
  .dof .drom{font-family:ui-monospace,monospace;font-variant-numeric:tabular-nums;
    text-align:right;white-space:nowrap}
  .dof .dvel{grid-column:1/-1;color:var(--faint);font-size:10.5px;
    font-family:ui-monospace,monospace}
  .dof .dnote{grid-column:1/-1;color:var(--planned);font-size:10.5px}
  /* clinical-gated numbers read dim while the view is showing RAW, mirroring
     the scene's amber "· raw" overlay: the anatomical zero isn't applied. */
  body[data-mode="raw"] .clin-gated{opacity:.5}
  .mkv{font-size:11.5px;color:var(--muted);margin-top:2px;
    font-family:ui-monospace,monospace;word-break:break-word}
  .mnote{font-size:10.5px;color:var(--planned);margin-top:3px}
  #metrics .empty{color:var(--faint);font-size:11.5px;font-style:italic}
  @media (max-width:640px){.legend{display:none}
    #metrics{width:100%;max-width:100%;top:auto;height:58%}
    #metrics.hidden{transform:translateY(102%)}}
</style>
</head>
<body>
<header>
  <div class="eyebrow" id="eyebrow">Stage 7 &middot; skeleton</div>
  <h1>Skeleton review</h1>
  <div class="sub" id="sub">&mdash;</div>
  <div class="stage" id="stage">Each bone is oriented by its measured
    quaternion and hung from its parent's joint. Orientation is measured; the
    bone lengths and joint spots are <b>assumed anatomy</b>. The ground arrow
    marks the subject's <b>front</b>; the chest face is lighter and the head has
    a nose. Drag to orbit &middot; scroll to zoom.</div>
  <div class="hint" id="hint"></div>
</header>
<main>
  <canvas id="view"></canvas>
  <aside id="metrics" class="hidden" aria-label="Session metrics"></aside>
  <div class="legend"><h2>Segments</h2><div id="legend"></div></div>
</main>
<footer>
  <button id="play" class="primary">&#9654; Play</button>
  <button id="mstoggle" hidden>&#9776; Metrics</button>
  <div class="toggle" id="view3d" title="Snap the camera">
    <button data-view="front">Front</button>
    <button data-view="side">Side</button>
    <button data-view="top">Top</button>
  </div>
  <div class="toggle" id="mode">
    <button data-mode="raw">Raw</button>
    <button data-mode="cal">Calibrated</button>
  </div>
  <button id="neutral">Jump to neutral</button>
  <input type="range" id="scrub" min="0" max="0" value="0" step="1">
  <div class="tlabel mono" id="tlabel">0 ms</div>
</footer>

<script>
"use strict";
const DATA = /*__FBD_DATA__*/null;

// ---- anthropometry: one body, sized from a standard ----
// Every segment's length AND breadth is a fraction of the subject's stature,
// from Winter, "Biomechanics and Motor Control of Human Movement" (segment
// length / stature; torso breadth = biacromial, i.e. shoulder, width). So the
// whole figure is one consistent body instead of eyeballed bars, and the
// shoulders come out at the real shoulder width. Tune STAT to resize the figure.
const STAT = 1.60;                         // nominal stature (arbitrary draw units)
const RATIO = {                            // [ length/stature , breadth/stature ]
  torso:     [0.288, 0.245],               // hip->shoulder ; biacromial (shoulders)
  upper_arm: [0.186, 0.057],
  forearm:   [0.146, 0.047],
  hand:      [0.108, 0.040],
};
const segLen = k => RATIO[k][0]*STAT;
const segW   = k => RATIO[k][1]*STAT;
const SHOULDER_W = segW('torso');          // shoulder-to-shoulder span (biacromial)
const HIP_W      = 0.190*STAT;             // hip breadth (bi-iliac) — trapezoid base
const TORSO_LEN  = segLen('torso');
const TRUNK_W    = 0.150*STAT;             // trunk depth (front-back), for the head gap
const HEAD_R     = 0.130*STAT/2;           // head height 0.130 of stature
const NECK       = 0.052*STAT;

// Per-segment colour. The skeleton's lengths / thicknesses live in ANAT below.
const SEG = {
  torso:       {color:[59,130,196]},
  upper_arm_r: {color:[228,87,46]},
  upper_arm_l: {color:[242,165,65]},
  forearm_r:   {color:[23,163,152]},
  forearm_l:   {color:[124,181,24]},
  hand_r:      {color:[111,75,216]},
  hand_l:      {color:[214,84,155]},
};
const rgb = c => `rgb(${c[0]|0},${c[1]|0},${c[2]|0})`;
const shade = (c,f) => [c[0]*f,c[1]*f,c[2]*f];
// segments the metrics panel is hovering — brightened in the scene so a
// joint/segment row visibly points at the bone(s) it measures.
let HILITE = new Set();
const hlBoost = (seg,c) => HILITE.has(seg)
  ? [Math.min(255,c[0]*1.28+34),Math.min(255,c[1]*1.28+34),Math.min(255,c[2]*1.28+34)]
  : c;

// ---- tiny vec3 + quaternion (Hamilton w,x,y,z, same as the Python tools) ----
const sub=(a,b)=>[a[0]-b[0],a[1]-b[1],a[2]-b[2]];
const add=(a,b)=>[a[0]+b[0],a[1]+b[1],a[2]+b[2]];
const dot=(a,b)=>a[0]*b[0]+a[1]*b[1]+a[2]*b[2];
const cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
const scl=(a,s)=>[a[0]*s,a[1]*s,a[2]*s];
function norm(a){const n=Math.hypot(a[0],a[1],a[2])||1;return[a[0]/n,a[1]/n,a[2]/n];}
function qmul(a,b){
  const[aw,ax,ay,az]=a,[bw,bx,by,bz]=b;
  return[aw*bw-ax*bx-ay*by-az*bz, aw*bx+ax*bw+ay*bz-az*by,
         aw*by-ax*bz+ay*bw+az*bx, aw*bz+ax*by-ay*bx+az*bw];
}
// rotate vec v by unit quaternion q (w,x,y,z): v + 2w(u×v) + 2(u×(u×v))
function qrot(q,v){
  const u=[q[1],q[2],q[3]], w=q[0];
  const t=scl(cross(u,v),2);
  return add(add(v,scl(t,w)), cross(u,t));
}

// ---- facing: where the subject's front is ----
// The calibrated torso frame is world-aligned at neutral (Z up), so the subject's
// left/right comes from the calibration facing (azimuth of forward, clockwise from
// world +Y). The shoulders go on the subject's actual right/left, and after the
// yaw correction below the figure's forward lands on +Y (the FRONT marker) and
// its right on +X. Unknown facing -> nominal 0°, labeled "front?".
const HEADING=DATA.meta.heading||{source:'none',confident:false,correction_yaw_deg:0};
const FRONT_KNOWN=!!HEADING.confident;
const FACE_DEG=FRONT_KNOWN?(HEADING.facing_deg??HEADING.correction_yaw_deg??0):0;
const _fr=FACE_DEG*Math.PI/180;
const RIGHT_W=[Math.cos(_fr),-Math.sin(_fr),0];      // subject's right, world, at neutral

// ---- ASSUMED anatomy for the skeleton ----
// The connectivity (the anatomical chain from the body model) is real; the
// numbers here are MODELED: nominal bone length, the bone's direction in the
// calibrated frame at neutral (torso runs up +Z; a hanging arm runs down -Z),
// and where a parent hands off to its child (`sockets` — only the torso has a
// lateral one: the shoulders sit near its top corners, on the subject's side). Change these to fit a
// subject; they never touch the measured orientation, only where a bar is drawn.
//
// Lengths, breadths, the shoulder sockets and the head all come from the one
// anthropometry table defined above (STAT / RATIO / SHOULDER_W ...), so the
// figure is a single consistent body. The shoulders sit at ± half the biacromial
// width at the top of the trunk; the trunk itself is drawn as a moderate tube
// and a separate shoulder bar (in renderSkeleton) spans the full shoulder width.
const ANAT = {
  torso:       {len:TORSO_LEN, dir:[0,0,1], thick:TRUNK_W, sockets:{
                  upper_arm_r:add(scl(RIGHT_W, SHOULDER_W/2),[0,0,TORSO_LEN*0.88]),
                  upper_arm_l:add(scl(RIGHT_W,-SHOULDER_W/2),[0,0,TORSO_LEN*0.88])}},
  upper_arm_r: {len:segLen('upper_arm'), dir:[0,0,-1], thick:segW('upper_arm')},
  upper_arm_l: {len:segLen('upper_arm'), dir:[0,0,-1], thick:segW('upper_arm')},
  forearm_r:   {len:segLen('forearm'),   dir:[0,0,-1], thick:segW('forearm')},
  forearm_l:   {len:segLen('forearm'),   dir:[0,0,-1], thick:segW('forearm')},
  hand_r:      {len:segLen('hand'),      dir:[0,0,-1], thick:segW('hand')},
  hand_l:      {len:segLen('hand'),      dir:[0,0,-1], thick:segW('hand')},
};
// full anatomical chain (presence-independent) — lets us bridge a missing
// middle segment with a dashed ghost instead of dropping its descendants.
const ANAT_CHAIN = DATA.anat_chain || {};
const ROOT_BASE = [0,0,-0.34];               // torso proximal (pelvis) in world
const IDENT = [1,0,0,0];
// where an anatomical parent P hands off to its child C (in P's local frame):
// the torso has lateral shoulder sockets; every other parent hands off at its
// own distal tip.
function socket(P,C){
  const pa=ANAT[P]||{dir:[0,0,-1],len:.2};
  return (pa.sockets&&pa.sockets[C])||scl(pa.dir,pa.len);
}

const cvs=document.getElementById('view'), ctx=cvs.getContext('2d');
const UP=[0,0,1], FOV=45*Math.PI/180, LIGHT=norm([0.45,0.55,1.0]);
const TARGET=[0,0,0.06], GROUND=-0.62;
let DPR=1, W=0, H=0;

// A box's 6 faces as corner-index quads (corners from boxBetween / the torso).
const BOX_FACES=[[0,1,3,2],[4,6,7,5],[0,4,5,1],[2,3,7,6],[0,2,6,4],[1,5,7,3]];

// ---- per-body state ----
const bodies=[];
for(const s of DATA.segments){
  const g=SEG[s.segment]||{color:[136,136,136]};
  bodies.push({seg:s.segment, calibrated:s.calibrated, offset:s.offset,
    color:g.color, frames:DATA.frames[s.segment]});
}

// ---- camera (Z-up spherical orbit) ----
// The figure's front faces +Y, so az=π/2 looks at it from the front and az=0
// from its right side. Default: front, a little to the subject's right, above.
const VIEWS={front:[Math.PI/2,Math.PI*0.46], side:[0,Math.PI*0.46],
             top:[Math.PI/2,0.12]};
let az=Math.PI/2-0.7, el=Math.PI*0.36, rad=3.15;
let cam, fwd, right, tup, focal, ccx, ccy;
function updateCamera(){
  const T=TARGET;
  cam=[T[0]+rad*Math.sin(el)*Math.cos(az), T[1]+rad*Math.sin(el)*Math.sin(az),
       T[2]+rad*Math.cos(el)];
  fwd=norm(sub(T,cam)); right=norm(cross(fwd,UP)); tup=cross(right,fwd);
  focal=(H/2)/Math.tan(FOV/2); ccx=W/2; ccy=H/2;
}
function project(P){
  const v=sub(P,cam), z=dot(v,fwd);
  if(z<=0.02) return null;
  return {x:ccx+focal*dot(v,right)/z, y:ccy-focal*dot(v,tup)/z, z};
}

// ---- playback / view state ----
const N=DATA.meta.n_frames;
let frame=0, mode=DATA.meta.has_calibration?'cal':'raw', playing=false;
function segQuat(b,i){
  const q=b.frames[i];
  return mode==='cal' ? qmul(q,b.offset) : q;
}

// ---- forward kinematics ----
// Walk the full anatomical chain from the (nominal) torso root down. Each link
// is placed at its parent's hand-off socket and oriented by the parent's
// MEASURED quaternion if that node is placed, or by identity (a "ghost" at rest)
// if the node is MISSING. So a present bone always uses real orientation; a
// missing middle bone becomes a dashed placeholder its descendants still hang
// off of. Position is modeled; orientation is real wherever a node exists.
const present=new Set(DATA.segments.map(s=>s.segment));
// facing correction: a fixed yaw about world up (Z) so the subject's forward
// draws toward the FRONT marker (+Y). Captured once at neutral, applied only
// when the facing is confident — so it never eats trunk motion during the clip.
const YAW_DEG=FRONT_KNOWN?(HEADING.correction_yaw_deg||0):0;
const _yr=YAW_DEG*Math.PI/180/2, FACE_Q=[Math.cos(_yr),0,0,Math.sin(_yr)]; // about +Z
const faced=p=>YAW_DEG?qrot(FACE_Q,p):p;            // rotate a world point into facing
function fkPose(){
  const q={}; for(const b of bodies) q[b.seg]=segQuat(b,frame);
  const quatOf=seg=>present.has(seg)?q[seg]:IDENT;   // ghosts sit at rest
  const memo={};
  function prox(seg){                                // world proximal of `seg`
    if(seg in memo) return memo[seg];
    const P=ANAT_CHAIN[seg];
    const r = P ? add(prox(P), qrot(quatOf(P), socket(P,seg)))
                : ROOT_BASE.slice();                 // torso = anatomical root
    return memo[seg]=r;
  }
  const dist=(seg,quat)=>{
    const a=ANAT[seg]||{dir:[0,0,-1],len:.2};
    return add(prox(seg), qrot(quat, scl(a.dir,a.len)));
  };
  // present bones (solid, real orientation), positions rotated into facing
  const pos={};
  for(const b of bodies)
    pos[b.seg]={prox:faced(prox(b.seg)), dist:faced(dist(b.seg,q[b.seg]))};
  // ghost bones: absent ancestors (not the torso root) on some present lineage
  const gset=new Set();
  for(const b of bodies){ let s=ANAT_CHAIN[b.seg];
    while(s){ if(!present.has(s) && s!=='torso') gset.add(s); s=ANAT_CHAIN[s]; } }
  const ghosts=[];
  for(const s of gset) ghosts.push({seg:s, prox:faced(prox(s)), dist:faced(dist(s,IDENT))});
  const shoulder=s=>faced(prox(s));                  // for the girdle
  return {pos, ghosts, shoulder};
}

// ---- the renderer ----
function render(){
  ctx.setTransform(DPR,0,0,DPR,0,0);
  // sky gradient backdrop
  const bg=ctx.createLinearGradient(0,0,0,H);
  bg.addColorStop(0,cssVar('--sky1')); bg.addColorStop(1,cssVar('--sky2'));
  ctx.fillStyle=bg; ctx.fillRect(0,0,W,H);
  updateCamera();

  // ground grid (drawn first, underneath)
  ctx.lineWidth=1; ctx.strokeStyle=cssVar('--grid'); ctx.globalAlpha=0.6;
  const R=2.0, step=0.4;
  for(let a=-R;a<=R+1e-6;a+=step){
    line([a,-R,GROUND],[a,R,GROUND]); line([-R,a,GROUND],[R,a,GROUND]);
  }
  ctx.globalAlpha=1;
  compass();
  renderSkeleton();
}

// ground compass: an arrow toward the subject's FRONT (+Y after the facing
// correction) with BACK and the subject's L / R around it. Faint and "front?"
// when the facing is unknown, so a nominal direction never reads as measured.
function compass(){
  const col=FRONT_KNOWN?cssVar('--accent'):cssVar('--faint');
  const z=GROUND, tip=[0,0.95,z], base=[0,0.28,z], C=1.12;
  ctx.save(); ctx.lineCap='round'; ctx.strokeStyle=col; ctx.fillStyle=col;
  ctx.lineWidth=3; if(!FRONT_KNOWN) ctx.setLineDash([8,6]);
  line(base,tip); ctx.setLineDash([]);
  const a=project(tip), l=project([-0.11,0.76,z]), r=project([0.11,0.76,z]);
  if(a&&l&&r){ctx.beginPath(); ctx.moveTo(a.x,a.y); ctx.lineTo(l.x,l.y);
    ctx.lineTo(r.x,r.y); ctx.closePath(); ctx.fill();}
  ctx.restore();
  label([0,C,z], FRONT_KNOWN?'FRONT':'front? (facing unknown)', col);
  label([0,-C,z],'BACK',cssVar('--faint'));
  label([C,0,z],'R',cssVar('--faint'));
  label([-C,0,z],'L',cssVar('--faint'));
}

// ---- helpers for the SOLID 3-D skeleton (shaded boxes + balls) ----
// 8 corners of an oriented box spanning A->B with a w×d cross-section.
function boxBetween(A,B,w,d){
  let ax=sub(B,A); const Ln=Math.hypot(ax[0],ax[1],ax[2])||1; ax=scl(ax,1/Ln);
  const ref=Math.abs(dot(ax,[0,0,1]))<0.9?[0,0,1]:[0,1,0];
  const u=norm(cross(ax,ref)), v=cross(ax,u), c=[];
  for(const P of [A,B]) for(const sv of [-1,1]) for(const su of [-1,1])
    c.push(add(P, add(scl(u,su*w*0.5), scl(v,sv*d*0.5))));
  return c;                                    // order matches BOX_FACES
}
// one box face -> a shaded, depth-keyed polygon (or null if off-screen)
function faceOf(corners, face, baseColor){
  const wp=face.map(i=>corners[i]), pp=wp.map(project);
  if(pp.some(p=>!p)) return null;
  const nrm=norm(cross(sub(wp[1],wp[0]),sub(wp[2],wp[0])));
  const lit=0.5+0.5*Math.max(0,Math.abs(dot(nrm,LIGHT)));   // simple diffuse
  return {pp, color:shade(baseColor,lit), depth:(pp[0].z+pp[1].z+pp[2].z+pp[3].z)/4};
}
// a shaded ball (fakes a lit sphere) — fills the seams at joints + the head
function ball(P, worldR, baseColor){
  const p=project(P); if(!p) return;
  const r=Math.max(2, focal*worldR/p.z);
  const g=ctx.createRadialGradient(p.x-r*0.35,p.y-r*0.4,r*0.1, p.x,p.y,r);
  g.addColorStop(0, rgb(shade(baseColor,1.35)));
  g.addColorStop(1, rgb(shade(baseColor,0.72)));
  ctx.beginPath(); ctx.arc(p.x,p.y,r,0,7); ctx.fillStyle=g; ctx.fill();
  ctx.lineWidth=1.1; ctx.strokeStyle='rgba(0,0,0,.18)'; ctx.stroke();
}

// the connected stickman: solid shaded 3-D volumes via forward kinematics.
function renderSkeleton(){
  const {pos, ghosts, shoulder}=fkPose();
  // assumed shoulder girdle: with no torso node we can't measure the trunk, so
  // the two arms root at nominal shoulders. Bridge them with a static dashed
  // line (clearly "assumed, not measured") so the arms read as one body.
  if(!DATA.meta.has_root && present.has('upper_arm_l') && present.has('upper_arm_r')){
    const Lp=shoulder('upper_arm_l'), Rp=shoulder('upper_arm_r');
    const a=project(Lp), c=project(Rp);
    if(a&&c){
      ctx.save();
      ctx.setLineDash([7,6]); ctx.lineCap='butt';
      ctx.lineWidth=Math.max(2, focal*0.02/((a.z+c.z)/2));
      ctx.strokeStyle=cssVar('--faint'); ctx.globalAlpha=.85; seg2d(a,c);
      ctx.restore();
      const mid=scl(add(Lp,Rp),0.5);
      label([mid[0],mid[1],mid[2]+0.09],'torso — not measured',cssVar('--faint'));
    }
  }
  // ---- solid 3-D figure: torso block + limb boxes in one depth-sorted pass ----
  const polys=[];
  // torso as a tapered 3-D block: wide at the shoulders, narrow at the hips, with
  // a front-back depth; it tilts/twists with the trunk (corners use its pose).
  if(present.has('torso')){
    const hip=pos['torso'].prox, sL=shoulder('upper_arm_l'), sR=shoulder('upper_arm_r');
    const sMid=scl(add(sL,sR),0.5);
    const xax=norm(sub(sR,sL)), upax=norm(sub(pos['torso'].dist,pos['torso'].prox));
    const dax=norm(cross(upax,xax));                   // front-back
    const tc=[];
    for(const hh of [[hip,HIP_W*0.5],[sMid,SHOULDER_W*0.5]])
      for(const sv of [-1,1]) for(const su of [-1,1])
        tc.push(add(hh[0], add(scl(xax,su*hh[1]), scl(dax,sv*TRUNK_W*0.5))));
    // BOX_FACES[3] is the +dax (front) face: tinted lighter so the chest reads.
    const tcol=hlBoost('torso',SEG.torso.color);
    BOX_FACES.forEach((f,k)=>{
      const p=faceOf(tc,f,k===3?[tcol[0]*0.55+115,tcol[1]*0.55+115,tcol[2]*0.55+115]:tcol);
      if(p) polys.push(p);
    });
  }
  // each limb as a solid shaded box between its two joints
  const raws=[];
  for(const b of bodies){
    if(b.seg==='torso') continue;
    const f=pos[b.seg], t=(ANAT[b.seg]&&ANAT[b.seg].thick)||.05;
    const box=boxBetween(f.prox, f.dist, t, t);
    for(const fc of BOX_FACES){ const p=faceOf(box,fc,hlBoost(b.seg,SEG[b.seg].color)); if(p) polys.push(p); }
    if(!b.calibrated) raws.push([project(f.prox), project(f.dist)]);
  }
  // paint every face far -> near (painter's algorithm) for correct occlusion
  polys.sort((x,y)=>y.depth-x.depth);
  for(const p of polys){
    ctx.beginPath(); ctx.moveTo(p.pp[0].x,p.pp[0].y);
    for(let i=1;i<4;i++) ctx.lineTo(p.pp[i].x,p.pp[i].y);
    ctx.closePath();
    ctx.fillStyle=rgb(p.color); ctx.fill();
    ctx.lineWidth=0.8; ctx.strokeStyle='rgba(0,0,0,.16)'; ctx.stroke();
  }
  // rounded joints: shaded balls fill the seams where boxes meet, and read as 3-D.
  // Each joint links two segments, so its ball blends their two colors (shoulder =
  // torso+arm, elbow = upper_arm+forearm, wrist = forearm+hand). A leaf tip
  // (fingertip) has no child, so it keeps its own segment colour.
  const mix=(a,b)=>[(a[0]+b[0])/2,(a[1]+b[1])/2,(a[2]+b[2])/2];
  const drawn=new Set(bodies.filter(b=>b.seg!=='torso').map(b=>b.seg));
  for(const g of ghosts) drawn.add(g.seg);
  const hasChild=new Set();
  for(const s of drawn){ const p=ANAT_CHAIN[s]; if(p) hasChild.add(p); }
  const parentColor=seg=>{
    const p=ANAT_CHAIN[seg]; if(!p) return null;
    if(p==='torso') return present.has('torso')?SEG.torso.color:null;
    if(drawn.has(p)&&SEG[p]) return SEG[p].color;
    return null;
  };
  for(const b of bodies){
    if(b.seg==='torso') continue;
    const t=(ANAT[b.seg]&&ANAT[b.seg].thick)||.05, col=SEG[b.seg].color;
    const pc=parentColor(b.seg);
    ball(pos[b.seg].prox, t*0.62, pc?mix(col,pc):col);   // joint with parent -> blend
    if(!hasChild.has(b.seg)) ball(pos[b.seg].dist, t*0.62, col); // leaf tip -> own colour
  }
  // ghost (missing-middle) bones: dashed & flat, clearly "not measured"
  ctx.lineCap='round';
  for(const g of ghosts){
    const a=project(g.prox), c=project(g.dist); if(!a||!c) continue;
    const w=Math.max(3, focal*((ANAT[g.seg]&&ANAT[g.seg].thick)||.05)/((a.z+c.z)/2));
    ctx.save(); ctx.setLineDash([6,5]); ctx.globalAlpha=.5;
    ctx.lineWidth=w;
    ctx.strokeStyle=rgb(shade((SEG[g.seg]||{color:[136,136,136]}).color,.9));
    seg2d(a,c); ctx.restore();
  }
  // uncalibrated present bones: amber dashed overlay so a kink reads as strap tilt
  ctx.save(); ctx.setLineDash([5,4]); ctx.lineCap='round'; ctx.lineWidth=3;
  ctx.strokeStyle='rgb(204,120,20)';
  for(const r of raws){ if(r[0]&&r[1]) seg2d(r[0],r[1]); }
  ctx.restore();
  // head as a shaded ball above the shoulders (only when the torso is measured)
  if(present.has('torso')){
    const sMid=scl(add(shoulder('upper_arm_l'),shoulder('upper_arm_r')),0.5);
    const up=norm(sub(pos['torso'].dist, pos['torso'].prox));
    const hc=add(sMid, scl(up, NECK+HEAD_R));
    ball(hc, HEAD_R, SEG.torso.color);
    // nose on the front of the head (forward = up × subject's right), drawn only
    // when it faces the camera so it never shows through the back of the head.
    const rt=norm(sub(shoulder('upper_arm_r'),shoulder('upper_arm_l')));
    const nose=add(hc, scl(norm(cross(up,rt)), HEAD_R*0.95));
    const pn=project(nose), ph=project(hc);
    if(pn&&ph&&pn.z<ph.z) ball(nose, HEAD_R*0.28, shade(SEG.torso.color,0.6));
  }
  // labels: present bones at their distal end (uncalibrated flagged amber
  // "· raw"); ghosts at their midpoint, faint and flagged "no node". The torso
  // trapezoid is self-evident, so it goes unlabelled.
  for(const b of bodies){
    if(b.seg==='torso') continue;
    const raw=!b.calibrated;
    label(add(pos[b.seg].dist, scl((ANAT[b.seg]||{dir:[0,0,-1]}).dir,-0.02)),
          raw?b.seg+' · raw':b.seg, raw?rgb([204,120,20]):undefined);
  }
  for(const g of ghosts){
    const mid=scl(add(g.prox,g.dist),0.5);
    label([mid[0],mid[1],mid[2]+0.05], g.seg+' · no node', cssVar('--faint'));
  }
}

function seg2d(a,b){ctx.beginPath();ctx.moveTo(a.x,a.y);ctx.lineTo(b.x,b.y);ctx.stroke();}
function label(P,text,color){
  const lp=project(P); if(!lp) return;
  ctx.font='600 12px ui-monospace,monospace';
  ctx.textAlign='center'; ctx.textBaseline='bottom';
  ctx.fillStyle=color||cssVar('--ink'); ctx.globalAlpha=0.9;
  ctx.fillText(text,lp.x,lp.y); ctx.globalAlpha=1;
}

function line(A,B){
  const a=project(A), b=project(B); if(!a||!b) return;
  ctx.beginPath(); ctx.moveTo(a.x,a.y); ctx.lineTo(b.x,b.y); ctx.stroke();
}
const _vc={};
function cssVar(name){
  if(!(name in _vc))
    _vc[name]=getComputedStyle(document.documentElement).getPropertyValue(name).trim();
  return _vc[name];
}

// ---- interaction: orbit + zoom ----
let drag=false, px=0, py=0;
cvs.addEventListener('pointerdown',e=>{drag=true;px=e.clientX;py=e.clientY;
  cvs.setPointerCapture(e.pointerId);});
cvs.addEventListener('pointerup',()=>{drag=false;});
cvs.addEventListener('pointermove',e=>{
  if(!drag) return;
  az-=(e.clientX-px)*0.008; el-=(e.clientY-py)*0.008;
  el=Math.max(0.12,Math.min(Math.PI-0.12,el)); px=e.clientX; py=e.clientY;
});
cvs.addEventListener('wheel',e=>{
  e.preventDefault();
  rad=Math.max(1.3,Math.min(9,rad*(1+Math.sign(e.deltaY)*0.08)));
},{passive:false});

// ---- resize ----
function resize(){
  DPR=Math.min(window.devicePixelRatio||1,2);
  W=cvs.clientWidth; H=cvs.clientHeight;
  cvs.width=W*DPR; cvs.height=H*DPR;
}
window.addEventListener('resize',resize);
// theme flips recolor the cached CSS vars
if(window.matchMedia)
  window.matchMedia('(prefers-color-scheme:dark)').addEventListener('change',
    ()=>{for(const k in _vc) delete _vc[k];});

// ---- UI wiring ----
const scrub=document.getElementById('scrub'), tlabel=document.getElementById('tlabel'),
  playBtn=document.getElementById('play'), neutralBtn=document.getElementById('neutral'),
  modeBox=document.getElementById('mode'), viewBox=document.getElementById('view3d'),
  hintEl=document.getElementById('hint'), subEl=document.getElementById('sub');
scrub.max=Math.max(0,N-1);
const fmtS=ms=>(ms/1000).toFixed(2)+' s';
function setFrame(i){
  frame=Math.max(0,Math.min(N-1,i|0)); scrub.value=frame;
  tlabel.textContent=`${fmtS(DATA.t_ms[frame])} · f${frame+1}/${N}`;
}
scrub.addEventListener('input',()=>{pause(); setFrame(+scrub.value);});
function pause(){playing=false; playBtn.innerHTML='&#9654; Play';
  playBtn.classList.add('primary');}
playBtn.addEventListener('click',()=>{
  playing=!playing;
  playBtn.innerHTML=playing?'&#10074;&#10074; Pause':'&#9654; Play';
  playBtn.classList.toggle('primary',!playing);
  if(playing && frame>=N-1) setFrame(0);
});
function setMode(m){
  if(m==='cal' && !DATA.meta.has_calibration) return;
  mode=m;
  document.body.dataset.mode=m;          // CSS dims clinical-gated metric rows in raw
  for(const b of modeBox.querySelectorAll('button'))
    b.classList.toggle('on',b.dataset.mode===m);
}
modeBox.addEventListener('click',e=>{
  const b=e.target.closest('button'); if(b) setMode(b.dataset.mode);
});
if(!DATA.meta.has_calibration){
  modeBox.querySelector('[data-mode="cal"]').disabled=true;
  neutralBtn.disabled=true;
}
// facing status line for the hint
const FACING=FRONT_KNOWN
  ? (HEADING.source==='manual'
      ? `front: stated by hand (faced ${FACE_DEG.toFixed(0)}°)`
      : `front: auto from torso (faced ${FACE_DEG.toFixed(0)}°)`)
  : HEADING.source==='torso_auto'
    ? 'front: unknown (low-confidence torso facing) — FRONT marker is nominal'
    : 'front: unknown (no torso node) — FRONT marker is nominal; calibrate with --facing-deg';
hintEl.innerHTML=FACING+' · a dashed bone has no node; an amber-dashed bone is '+
  'uncalibrated (a kink there may be strap tilt, not motion)';
function setView(v){
  [az,el]=VIEWS[v];
  for(const b of viewBox.querySelectorAll('button'))
    b.classList.toggle('on',b.dataset.view===v);
}
viewBox.addEventListener('click',e=>{
  const b=e.target.closest('button'); if(b) setView(b.dataset.view);
});
// orbiting by hand leaves the preset, so clear its highlight
cvs.addEventListener('pointermove',()=>{ if(drag)
  for(const b of viewBox.querySelectorAll('button')) b.classList.remove('on'); });
neutralBtn.addEventListener('click',()=>{
  const nw=DATA.meta.neutral_window_ms; if(!nw) return;
  const mid=(nw[0]+nw[1])/2; let best=0,bd=Infinity;
  for(let i=0;i<N;i++){const d=Math.abs(DATA.t_ms[i]-mid); if(d<bd){bd=d;best=i;}}
  pause(); setMode('cal'); setFrame(best);
});

// legend
const leg=document.getElementById('legend');
for(const s of DATA.segments){
  const g=SEG[s.segment]||{color:[136,136,136]};
  const row=document.createElement('div'); row.className='legrow';
  row.innerHTML=`<span class="sw" style="background:${rgb(g.color)}"></span>`+
    `<span>${s.segment}</span>`+
    `<span class="badge ${s.calibrated?'cal':'raw'}">`+
    `${s.calibrated?'CAL':'RAW'}</span>`;
  leg.appendChild(row);
}
const m=DATA.meta;
subEl.innerHTML=`subject <b>${m.subject}</b> · session <b>${m.session}</b> · `+
  `${DATA.segments.length} segment(s) · ${N} frames `+
  `(${(m.t1_ms-m.t0_ms)/1000|0}s${m.stride>1?`, 1/${m.stride} sampled`:''}) · `+
  `<span class="mono">${m.source_csv}</span>`+
  (m.calibration_source?` + <span class="mono">${m.calibration_source}</span>`:
   ` · <b style="color:var(--planned)">no calibration — raw only</b>`);

// ---- stage-7 metrics review panel (built from the baked metrics.json) ----
// The panel reads out the session SUMMARY (ROM / velocity / reps / derived)
// beside the 3-D body, sharing this page's raw↔calibrated toggle. Honesty flags
// carry straight through: a joint whose two nodes aren't both calibrated is
// RELATIVE (dimmed while the view shows raw); a blocked joint names its missing
// node; a derived metric shows its own note. Hovering a row highlights the
// bone(s) it measures in the scene.
const esc=s=>String(s).replace(/[&<>]/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;'}[c]));
const n1=(x,u='')=>x==null?'—':(Math.round(x*10)/10)+u;
function segsFor(row){
  // segments a metrics row points at, for the hover highlight.
  if(row.kind==='segment') return [row.data.segment];
  if(row.kind==='joint'||row.kind==='blocked'){
    const js=DATA.joint_segments||{}; return js[row.data.key]||[];
  }
  if(row.kind==='derived'){
    const out=new Set();
    for(const jk of (row.data.requires||[])){
      for(const s of (DATA.joint_segments||{})[jk]||[]) out.add(s);
    }
    return [...out];
  }
  return [];
}
function romLine(d){
  if(!d.rom) return `<span class="dname">${esc(d.name)}</span>`+
    `<span class="drom">— <span style="color:var(--faint)">singular</span></span>`+
    (d.singularity_note?`<span class="dnote">${esc(d.singularity_note)}</span>`:'');
  const v=d.velocity, frac=('defined_frac' in d)
    ? `  ·  ${Math.round(d.defined_frac*100)}% defined` : '';
  return `<span class="dname">${esc(d.name)}</span>`+
    `<span class="drom">${n1(d.rom.range_deg,'°')} `+
    `<span style="color:var(--faint)">[${n1(d.rom.min_deg)}…${n1(d.rom.max_deg)}]</span></span>`+
    (v?`<span class="dvel">peak ${n1(v.peak_deg_s,'°/s')} · mean ${n1(v.mean_abs_deg_s,'°/s')}${frac}</span>`:'');
}
function jointCard(j){
  const rel=!j.clinical;
  const flag=rel?`<span class="flag rel" title="one or both nodes uncalibrated">relative</span>`
                :`<span class="flag clin">clinical</span>`;
  const reps=(j.reps&&j.reps.count)?`<span class="mmeta">${j.reps.count} reps · ${esc(j.reps.primary_dof)}</span>`:
    `<span class="mmeta">${esc(j.decomposition)}</span>`;
  const dofs=j.dofs.map(d=>`<div class="dof${rel?' clin-gated':''}">${romLine(d)}</div>`).join('');
  const warn=rel&&j.warning?`<div class="mnote">${esc(j.warning)}</div>`:'';
  return `<div class="mcard" data-i="${ROWS.push({kind:'joint',data:j})-1}">`+
    `<div class="mhead"><span class="mname">${esc(j.name)}</span>${flag}${reps}</div>`+
    dofs+warn+`</div>`;
}
function segCard(s){
  const cal=s.calibrated?`<span class="flag clin">cal</span>`
                        :`<span class="flag rel">raw</span>`;
  const t=s.travel, sp=s.angular_speed, el=s.elevation;
  const sm=(s.smoothness_sparc==null)?'still':`SPARC ${s.smoothness_sparc>0?'+':''}${n1(s.smoothness_sparc)}`;
  const elev=`<span class="${s.calibrated?'':'clin-gated'}">elev ${n1(el.range_deg,'°')}</span>`;
  return `<div class="mcard" data-i="${ROWS.push({kind:'segment',data:s})-1}">`+
    `<div class="mhead"><span class="mname">${esc(s.segment)}</span>${cal}`+
    `<span class="mmeta">${esc(s.node_id||'')}</span></div>`+
    `<div class="mkv">travel ${n1(t.travel_deg,'°')} · active ${Math.round(t.active_time_frac*100)}%`+
    ` · ${elev} · peak ${n1(sp.peak_deg_s,'°/s')} · ${sm}</div></div>`;
}
function derivedCard(d){
  const gated=(d.clinical===false);
  const kv=Object.entries(d.metrics||{}).filter(([,v])=>!Array.isArray(v)&&v!=null)
    .map(([k,v])=>`${esc(k)}=${esc(v)}`).join(' · ');
  const flag=('clinical' in d)?(d.clinical?`<span class="flag clin">clinical</span>`
             :`<span class="flag rel">relative</span>`):'';
  const note=d.note?`<div class="mnote">${esc(d.note)}</div>`:'';
  return `<div class="mcard${gated?' clin-gated':''}" data-i="${ROWS.push({kind:'derived',data:d})-1}">`+
    `<div class="mhead"><span class="mname">${esc(d.name)}</span>${flag}`+
    `<span class="mmeta">${esc(d.target)}</span></div>`+
    `<div class="mkv">${kv||'—'}</div>${note}</div>`;
}
function blockedCard(b){
  return `<div class="mcard" data-i="${ROWS.push({kind:'blocked',data:b})-1}">`+
    `<div class="mhead"><span class="mname">${esc(b.name)}</span>`+
    `<span class="flag blk">blocked</span>`+
    `<span class="mmeta">needs ${esc((b.missing||[]).join(', '))}</span></div></div>`;
}
const ROWS=[];         // index -> {kind,data}, so a hovered card knows its segments
const MET=DATA.metrics, metricsEl=document.getElementById('metrics'),
  msToggle=document.getElementById('mstoggle');
if(MET){
  document.getElementById('eyebrow').innerHTML='Stage 7 &middot; review';
  const calLbl=MET.calibration_used?'calibration applied'
    :'<b style="color:var(--planned)">no calibration — relative only</b>';
  const sec=(title,inner,empty)=>`<section><h3>${title}</h3>`+
    (inner||`<div class="empty">${empty}</div>`)+`</section>`;
  const joints=(MET.joints||[]).map(jointCard).join('')+
    (MET.blocked_joints||[]).map(blockedCard).join('');
  const segs=(MET.segments||[]).map(segCard).join('');
  const der=(MET.derived||[]).map(derivedCard).join('');
  metricsEl.innerHTML=
    `<h2>Session metrics</h2>`+
    `<div class="msub">${MET.n_samples} samples · ${MET.duration_s}s · `+
      `${MET.sample_rate_hz} Hz · ${calLbl}</div>`+
    `<div class="rawbanner">Showing <b>raw</b> — angles below assume the mounting `+
      `offset; the dimmed <b>relative</b> rows aren't anatomically anchored until `+
      `you flip to <b>Calibrated</b>.</div>`+
    sec('Joints &amp; range of motion',joints,'no computable joints for this montage')+
    sec('Segments',segs,'no segments')+
    sec('Derived',der,'none unlocked by this montage');

  // hover a card -> highlight its bone(s) in the 3-D scene
  metricsEl.addEventListener('mouseover',e=>{
    const card=e.target.closest('.mcard'); if(!card) return;
    HILITE=new Set(segsFor(ROWS[+card.dataset.i]||{}));
    for(const c of metricsEl.querySelectorAll('.mcard')) c.classList.remove('hl');
    card.classList.add('hl');
  });
  metricsEl.addEventListener('mouseleave',()=>{
    HILITE=new Set();
    for(const c of metricsEl.querySelectorAll('.mcard')) c.classList.remove('hl');
  });

  // toggle button (shown only when metrics are present); open by default on a
  // wide screen so the review reads as one page, closed on a phone.
  msToggle.hidden=false;
  const openMetrics=o=>{metricsEl.classList.toggle('hidden',!o);
    msToggle.classList.toggle('primary',o);};
  msToggle.addEventListener('click',()=>openMetrics(metricsEl.classList.contains('hidden')));
  openMetrics(window.innerWidth>820);
}

// ---- animation loop (real-time playback keyed on baked t_ms) ----
let last=performance.now(), acc=0;
function tick(now){
  const dt=now-last; last=now;
  if(playing && N>1){
    acc+=dt;
    while(frame<N-1 && acc>=(DATA.t_ms[frame+1]-DATA.t_ms[frame])){
      acc-=(DATA.t_ms[frame+1]-DATA.t_ms[frame]); setFrame(frame+1);
    }
    if(frame>=N-1) pause();
  }
  render();
  requestAnimationFrame(tick);
}

resize(); setMode(mode); setFrame(0);
requestAnimationFrame(tick);
</script>
</body>
</html>
"""


if __name__ == "__main__":
    main()
