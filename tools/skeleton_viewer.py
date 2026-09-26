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

Forearm rotation
----------------
Each limb box rolls with its bone's measured twist (its cross-section follows
the bone's own anterior axis, not a fixed world reference). The forearm and hand
are flat slabs with a lighter FRONT (palm-side) face and a thumb nub on the
lateral side, so palm-forward / thumb-out reads as supinated and palm-back /
thumb-in as pronated. A corner panel shows the live pronation/supination angle
per forearm, computed exactly as metrics.py does (q_rel in the baked anatomical
frame, left mirrored, ZXY slot 2) — calibrated view with a known front only.
Playback runs at 1× / 2× / 5×.

Angle over time
---------------
Clicking a movement in the metrics panel opens a graph strip under the 3-D view:
that angle across the session (the same movement on the other side dashed), the
playhead synced to playback, click / drag to seek, the neutral window and the
undefined stretches shaded, min / max marked. The series is computed in the page
from the baked quaternions with metrics.py's chain (unwrapped, then shifted by a
multiple of 360° so its middle reads within ±180°, as the panel does).

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
    python tools/skeleton_viewer.py render aligned.csv montage.json --out skeleton.html

    # with calibration -> raw<->calibrated toggle + neutral-window jump:
    python tools/skeleton_viewer.py render aligned.csv montage.json \
        --calibration calibration.json --out skeleton.html

    # + metrics.json -> the stage-7 review panel (ROM/velocity/reps/derived)
    # reads out beside the body, sharing the raw<->calibrated toggle:
    python tools/skeleton_viewer.py render aligned.csv montage.json \
        --calibration calibration.json --metrics metrics.json --out review.html

    # validate end-to-end with no hardware (synth a session, bake, check):
    python tools/skeleton_viewer.py selftest
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
from metrics import (  # noqa: E402
    resolve_anatomical_frame, trim_to_analysis, joint_frame_deg,
)
from reconcile_nodes import quality_path  # noqa: E402

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
                metrics=None, quality=None, trim=True):
    """Bake a viewer-ready scene dict from the aligned stream.

    Bakes RAW world-from-sensor quaternions per segment plus, per segment, the
    single cached mounting offset (identity when uncalibrated). The viewer forms
    the calibrated orientation as q_seg = q_WS ⊗ q_SB on the fly, so both views
    come from one small payload.

    `metrics` (an optional metrics.py report dict) rides along in the scene so the
    stage-7 review panel reads out beside the 3-D body — one page, one payload. It
    holds only session SUMMARY stats (no per-frame arrays), so it stays compact.

    `quality` (the reconcile sidecar, aligned.quality.json) carries per-node sync
    confidence, shown as a header chip and per-card warnings.

    With `trim` (default) playback starts at the neutral hold — the setup before
    it is not protocol, and metrics.py drops it the same way.
    """
    t_ms, seg_quats, seg_meta = load_aligned(csv_path, montage)
    if trim:
        t_ms, seg_quats, _ = trim_to_analysis(t_ms, seg_quats, calibration)
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

    # Anatomical frame at neutral (X anterior, Y up, Z right) — the same q_WA
    # metrics.py decomposes in — so the viewer can read the live forearm
    # pronation/supination angle with identical conventions. None if unknown.
    q_wa = resolve_anatomical_frame(calibration) if calibration else None

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
            "anatomical_frame_quat": (None if q_wa is None
                                      else [round(float(v), 8) for v in q_wa]),
        },
        "parents": parents,
        "anat_chain": _anat_chain(),
        # joint -> [proximal, distal] segment, so a metrics-panel joint row can
        # highlight the two bones it spans in the 3-D view. Real body-model
        # adjacency (JOINTS), not a viewer guess.
        "joint_segments": {k: [j.proximal, j.distal] for k, j in JOINTS.items()},
        # joint -> Euler sequence + DOF slots, so the viewer computes live angles
        # with the same body model (and conventions) as metrics.py
        "joint_defs": {k: {"proximal": j.proximal, "distal": j.distal,
                           "seq": j.decomposition.split()[0],
                           "frame_y_deg": joint_frame_deg(j),
                           "dofs": [{"key": d.key, "seq_index": d.seq_index}
                                    for d in j.dofs]}
                       for k, j in JOINTS.items()},
        # reconcile's sync / data-loss summary (or null for an older session)
        "quality": quality,
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
    # the reconcile quality sidecar: given explicitly, or found next to the CSV
    qpath = args.quality or quality_path(args.aligned_csv)
    quality = _load_json(qpath) if os.path.exists(qpath) else None
    scene = build_scene(args.aligned_csv, montage, calibration, args.max_frames,
                        metrics=metrics, quality=quality)
    html = render_html(scene)
    # UTF-8 always: the page is <meta charset="utf-8"> and carries non-ASCII glyphs
    # (↔, °, ·, —). Without this, Python on Windows defaults to cp1252 and the
    # write dies with a UnicodeEncodeError.
    with open(args.out, "w", encoding="utf-8") as f:
        f.write(html)

    m = scene["meta"]
    print(f"[viewer] wrote {args.out}")
    print(f"[viewer] {len(scene['segments'])} segment(s), {m['n_frames']} frames "
          f"(stride {m['stride']} of {m['n_samples_total']}), "
          f"{m['t0_ms']:.0f}-{m['t1_ms']:.0f} ms")
    ncal = sum(1 for s in scene["segments"] if s["calibrated"])
    if calibration:
        print(f"[viewer] calibration applied to {ncal}/{len(scene['segments'])} "
              f"segment(s); raw<->calibrated toggle enabled.")
        if m["neutral_window_ms"]:
            print(f"[viewer] neutral window {m['neutral_window_ms'][0]:.0f}-"
                  f"{m['neutral_window_ms'][1]:.0f} ms — jump there and flip the "
                  f"toggle to see the mounting tilt straighten out.")
        h = m["heading"]
        print("[viewer] front: " + (f"known (subject faced {h['facing_deg']:.0f}°, "
              f"{h['source']})" if h["confident"] else
              "UNKNOWN — the FRONT marker is nominal (add a torso node or "
              "calibrate with --facing-deg)"))
    else:
        print("[viewer] no calibration given — RAW orientation only (each limb keeps "
              "its mounting tilt). Pass --calibration to enable the toggle.")
    if metrics:
        nj = len(metrics.get("joints", []))
        nd = len(metrics.get("derived", []))
        print(f"[viewer] metrics panel attached: {nj} joint(s), "
              f"{len(metrics.get('segments', []))} segment(s), {nd} derived — "
              f"the stage-7 review reads out beside the body.")
    else:
        print("[viewer] no metrics given — 3-D view only. Pass --metrics metrics.json "
              "for the stage-7 review panel.")
    if quality:
        low = [n["column"] for n in quality.get("nodes", [])
               if not n.get("reference") and not n.get("sync_reliable", True)]
        print(f"[viewer] sync / data-loss summary attached ({os.path.basename(qpath)})"
              + (f" — LOW sync confidence: {', '.join(low)}" if low else ""))
    else:
        print("[viewer] no sync summary found (re-run reconcile_nodes.py to write "
              "aligned.quality.json) — the page shows 'Sync not recorded'.")
    print(f"[viewer] open it in a browser: file://{os.path.abspath(args.out)}")


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
        out = os.path.join(d, "skeleton.html")
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

        # Same session calibrated with a stated facing: the anatomical frame is
        # known, so it is baked for the live forearm-rotation readout.
        cal_face = build_calibration(montage, t_ms, seg_quats, seg_meta,
                                     float(t_ms[0]), float(t_ms[-1]),
                                     "synth.csv", facing_deg=30.0)
        scene_face = build_scene(csv, montage, cal_face, max_frames=50)
        qual = {"schema_version": "1.0", "confidence_min": 0.4, "nodes": [
            {"column": "n0", "reference": True, "sync_reliable": True},
            {"column": "n1", "reference": False, "sync_confidence": 0.2,
             "sync_reliable": False, "gap_frac": 0.0, "longest_gap_ms": 0.0}]}
        scene_q = build_scene(csv, montage, cal, max_frames=50, quality=qual)

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
    check('data-speed="2"' in html and 'data-speed="5"' in html
          and "acc+=dt*SPEED" in html and "function liveAngles" in html
          and len(scene_face["meta"]["anatomical_frame_quat"] or []) == 4
          # no confident facing -> no frame baked (readout stays blank)
          and (scene["meta"]["anatomical_frame_quat"] is None)
          == (not scene["meta"]["heading"]["confident"]),
          "1×/2×/5× playback + live joint angles (anatomical frame baked)")
    jd = scene["joint_defs"]
    check(jd["elbow_r"]["seq"] == "ZXY" and jd["shoulder_r"]["seq"] == "YXY"
          and {d["key"] for d in jd["shoulder_r"]["dofs"]}
          == {"plane_elev", "elevation", "axial_rot"}
          and scene["quality"] is None
          and scene_q["quality"]["nodes"][1]["sync_reliable"] is False
          and 'id="labels"' in html and "function syncChips" in html,
          "joint defs + sync/data-loss summary baked; labels toggle present")
    check('id="graph"' in html and "function angleSeries" in html
          and "function drawGraph" in html and 'class="row plot' in html,
          "angle-over-time graph wired to the metrics rows")
    check(scene["meta"]["neutral_window_ms"] == [0.0, 4000.0],
          "neutral window carried through for the jump button")

    # (5b) Stage-7 metrics panel: the baked page carries the metrics payload +
    #      the panel markup, and (this fully-calibrated synth) reads CLINICAL.
    jkeys = [j["key"] for j in metrics_report["joints"]]
    panel = ('id="metrics"' in html and '"metrics":' in html
             and "Session metrics" in html and 'class="tag rel"' in html
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
    pr.add_argument("--quality", help="sync / data-loss summary from "
                    "reconcile_nodes.py (default: <aligned>.quality.json next to "
                    "the CSV, if present)")
    pr.add_argument("--metrics", help="metrics.json from metrics.py (adds the "
                    "stage-7 review panel: ROM / velocity / reps / derived)")
    pr.add_argument("--out", default="skeleton.html",
                    help="output HTML file (default: skeleton.html)")
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
  header{padding:14px 22px 12px;border-bottom:1px solid var(--line);
    background:var(--surface)}
  .titlebar{display:flex;align-items:center;gap:12px;flex-wrap:wrap}
  h1{margin:0;font-size:20px;font-weight:650;letter-spacing:-.01em}
  .chips{display:flex;gap:6px;flex-wrap:wrap}
  .chip{font-size:12px;font-weight:600;padding:3px 9px;border-radius:999px;
    cursor:default}
  .chip.good{background:color-mix(in srgb,var(--built) 16%,transparent);color:var(--built)}
  .chip.warn{background:color-mix(in srgb,var(--planned) 18%,transparent);color:var(--planned)}
  .chip.muted{background:color-mix(in srgb,var(--faint) 16%,transparent);color:var(--muted)}
  .sub{color:var(--muted);font-size:13.5px;margin-top:4px}
  main{flex:1;position:relative;min-height:0}
  #view{position:absolute;inset:0;display:block;width:100%;height:100%;
    touch-action:none;cursor:grab}
  #view:active{cursor:grabbing}
  .legend{position:absolute;right:12px;top:12px;background:var(--surface);
    border:1px solid var(--line);border-radius:10px;padding:11px 13px;
    box-shadow:var(--shadow);width:250px;font-size:12.5px;
    max-height:calc(100% - 24px);overflow-y:auto}
  .legend h2{margin:0 0 7px;font-size:11.5px;font-weight:600;letter-spacing:.06em;
    text-transform:uppercase;color:var(--muted)}
  .legend h2+div{margin-bottom:10px}
  .legrow{display:flex;align-items:center;gap:8px;padding:2px 0}
  .sw{width:12px;height:12px;border-radius:3px;flex:none}
  .legrow .st{margin-left:auto;font-size:11px;color:var(--built)}
  .legrow .st.no{color:var(--planned)}
  .legrow{cursor:default;border-radius:5px;margin:0 -4px;padding:2px 4px}
  .legrow:hover{background:color-mix(in srgb,var(--accent) 10%,transparent)}
  .key{color:var(--muted);line-height:1.45}
  .key div{padding:1px 0}
  .key .dim{color:var(--faint);margin-top:4px}
  .live{margin:0 0 10px;padding-bottom:10px;border-bottom:1px solid var(--line);
    font-size:13px}
  .lv-j{padding:3px 0}
  .lv-j .n{color:var(--muted);font-size:12px}
  .lv-j .v{font-variant-numeric:tabular-nums;font-weight:600;color:var(--accent-ink)}
  .lv-na{color:var(--faint)}
  .lv-why{color:var(--faint);font-size:11px;margin-top:3px}
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
  #labels.on{background:var(--accent);border-color:var(--accent);color:#fff}
  input[type=range]{flex:1;min-width:160px;accent-color:var(--accent)}
  .tlabel{font-variant-numeric:tabular-nums;color:var(--muted);font-size:12px;
    min-width:150px;text-align:right}

  /* ---- session metrics panel (left drawer over the canvas) ---- */
  #metrics{position:absolute;left:0;top:0;bottom:0;width:440px;max-width:92vw;
    background:var(--surface);border-right:1px solid var(--line);
    box-shadow:var(--shadow);overflow-y:auto;padding:18px 20px 26px;z-index:5;
    transition:transform .18s ease;font-size:14px}
  #metrics.hidden{transform:translateX(-102%)}
  #metrics h2{margin:0 0 3px;font-size:18px;font-weight:650}
  #metrics .msub{color:var(--muted);font-size:13px;margin-bottom:14px}
  #metrics .rawbanner{display:none;margin:0 0 14px;padding:9px 12px;
    border-radius:8px;font-size:13px;
    background:color-mix(in srgb,var(--planned) 14%,transparent);
    color:var(--planned);border:1px solid color-mix(in srgb,var(--planned) 30%,transparent)}
  body[data-mode="raw"] #metrics .rawbanner{display:block}
  #metrics section{margin:0 0 18px}
  #metrics section>h3{margin:0 0 9px;font-size:12px;font-weight:650;
    letter-spacing:.07em;text-transform:uppercase;color:var(--muted);
    border-bottom:1px solid var(--line);padding-bottom:5px}
  .mcard{border:1px solid var(--line);border-radius:10px;padding:11px 14px;
    margin-bottom:9px;cursor:default}
  .mcard.hl{border-color:var(--accent);box-shadow:0 0 0 1px var(--accent) inset}
  .mhead{display:flex;align-items:center;gap:8px;flex-wrap:wrap;margin-bottom:4px}
  .mname{font-weight:650;font-size:15.5px}
  .tag{font-size:11.5px;font-weight:600;padding:2px 8px;border-radius:999px}
  .tag.rel{background:color-mix(in srgb,var(--planned) 18%,transparent);color:var(--planned)}
  .tag.info{background:color-mix(in srgb,var(--accent) 14%,transparent);color:var(--accent-ink);
    margin-left:auto}
  .row{display:grid;grid-template-columns:1fr auto;gap:0 12px;align-items:baseline;
    padding:7px 0 6px;border-top:1px solid var(--line)}
  .mhead+.row{border-top:none}
  .row .k{color:var(--ink);font-size:14px}
  .row .v{font-size:16px;font-weight:650;font-variant-numeric:tabular-nums;
    white-space:nowrap}
  .row .d{grid-column:1/-1;color:var(--muted);font-size:12.5px;margin-top:1px}
  .row .d.warn{color:var(--planned)}
  .mnote{font-size:12.5px;color:var(--planned);margin-top:6px}
  .row.plot{cursor:pointer;margin:0 -8px;padding-left:8px;padding-right:8px;border-radius:6px}
  .row.plot:hover{background:color-mix(in srgb,var(--accent) 7%,transparent)}
  .row.plot.sel{background:color-mix(in srgb,var(--accent) 13%,transparent)}
  .row.plot .k::after{content:"  graph";font-size:11px;color:var(--faint);opacity:0}
  .row.plot:hover .k::after{opacity:1}
  #graph{position:absolute;left:0;right:0;bottom:0;height:210px;z-index:4;
    background:var(--surface);border-top:1px solid var(--line);
    box-shadow:0 -6px 18px rgba(22,32,43,.06);display:flex;flex-direction:column}
  #graph[hidden]{display:none}
  .ghead{display:flex;align-items:center;gap:14px;padding:7px 14px 0;font-size:13px;
    flex-wrap:wrap}
  #gtitle{font-weight:650;font-size:14px}
  .gleg{display:flex;gap:12px;color:var(--muted);font-size:12px}
  .gleg i{display:inline-block;width:16px;height:0;border-top:2.5px solid;vertical-align:middle;
    margin-right:5px}
  .gleg i.dash{border-top-style:dashed}
  .gopt{color:var(--muted);font-size:12px;display:flex;align-items:center;gap:4px}
  #gclose{margin-left:auto;padding:3px 11px;font-size:12.5px}
  #gcv{flex:1;width:100%;min-height:0;display:block;cursor:crosshair;touch-action:none}
  /* relative-only numbers dim while the view shows RAW (anatomical zero off) */
  body[data-mode="raw"] .clin-gated{opacity:.5}
  @media (max-width:640px){.legend{display:none}
    #metrics{width:100%;max-width:100%;top:auto;height:60%}
    #metrics.hidden{transform:translateY(102%)}}
</style>
</head>
<body>
<header>
  <div class="titlebar">
    <h1>Session review</h1>
    <div class="chips" id="chips"></div>
  </div>
  <div class="sub" id="sub">&mdash;</div>
</header>
<main>
  <canvas id="view"></canvas>
  <aside id="metrics" class="hidden" aria-label="Session metrics"></aside>
  <div id="graph" hidden aria-label="Angle over time">
    <div class="ghead">
      <span id="gtitle"></span>
      <span class="gleg" id="gleg"></span>
      <label class="gopt" id="gotherwrap"><input type="checkbox" id="gother" checked>
        compare other side</label>
      <button id="gclose" title="Close the graph">Close</button>
    </div>
    <canvas id="gcv"></canvas>
  </div>
  <div class="legend">
    <div class="live" id="live" hidden></div>
    <h2>Sensors</h2><div id="legend"></div>
    <h2>How to read</h2>
    <div class="key">
      <div>Lighter face = front of that body part (palm side on the forearm and hand)</div>
      <div>Dark dot on the hand = thumb</div>
      <div>Dashed limb = no sensor there</div>
      <div>Orange dashed = sensor not calibrated</div>
      <div>Hover a sensor or a metric to highlight it</div>
      <div class="dim">Drag to rotate &middot; scroll to zoom</div>
    </div>
  </div>
</main>
<footer>
  <button id="play" class="primary">&#9654; Play</button>
  <div class="toggle" id="speed" title="Playback speed">
    <button data-speed="1" class="on">1&times;</button>
    <button data-speed="2">2&times;</button>
    <button data-speed="5">5&times;</button>
  </div>
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
  <button id="labels" title="Show the name of every body part">Labels</button>
  <button id="neutral">Go to neutral pose</button>
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
// Right side = warm (reds / oranges), left side = cool (blues), darkest at the
// shoulder and lightest at the hand, trunk a neutral slate — so a glance tells
// the sides apart and the order along each arm.
const SEG = {
  torso:       {color:[112,124,140]},
  upper_arm_r: {color:[196,52,40]},
  forearm_r:   {color:[232,112,40]},
  hand_r:      {color:[240,170,50]},
  upper_arm_l: {color:[30,82,170]},
  forearm_l:   {color:[48,138,214]},
  hand_l:      {color:[110,190,236]},
};
// Plain-language names for sensors / joints / movements (display only).
const NAMES={torso:'Trunk', upper_arm_r:'Right upper arm', upper_arm_l:'Left upper arm',
  forearm_r:'Right forearm', forearm_l:'Left forearm', hand_r:'Right hand', hand_l:'Left hand',
  shoulder_r:'Right shoulder', shoulder_l:'Left shoulder', elbow_r:'Right elbow',
  elbow_l:'Left elbow', wrist_r:'Right wrist', wrist_l:'Left wrist'};
const nameOf=k=>NAMES[k]||String(k).replace(/_/g,' ');
const shortOf=k=>nameOf(k).replace(/^Right /,'R ').replace(/^Left /,'L ').toLowerCase()
  .replace(/^(r|l) /,m=>m.toUpperCase());
const rgb = c => `rgb(${c[0]|0},${c[1]|0},${c[2]|0})`;
const shade = (c,f) => [c[0]*f,c[1]*f,c[2]*f];
// segments the metrics panel is hovering — brightened in the scene so a
// joint/segment row visibly points at the bone(s) it measures.
let HILITE = new Set(), SHOW_LABELS = false, GRAPH = null;
const graphEl = document.getElementById('graph');
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
const FWD_W=[Math.sin(_fr),Math.cos(_fr),0];         // subject's forward, world, at neutral
// A limb's lateral side (thumb side in the anatomical position) at neutral.
const LATERAL_W=seg=>seg.endsWith('_l')?scl(RIGHT_W,-1):RIGHT_W;

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
const TARGET=[0,0,-0.1], GROUND=-0.62;
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
let az=Math.PI/2-0.7, el=Math.PI*0.36, rad=2.7;
let cam, fwd, right, tup, focal, ccx, ccy;
function updateCamera(){
  const T=TARGET;
  cam=[T[0]+rad*Math.sin(el)*Math.cos(az), T[1]+rad*Math.sin(el)*Math.sin(az),
       T[2]+rad*Math.cos(el)];
  fwd=norm(sub(T,cam)); right=norm(cross(fwd,UP)); tup=cross(right,fwd);
  // centre the figure in the part of the canvas the metrics drawer leaves free
  const mEl=document.getElementById('metrics');
  const cover=(mEl&&!mEl.classList.contains('hidden')&&W>640)?mEl.offsetWidth:0;
  const gh=(GRAPH&&!graphEl.hidden)?graphEl.offsetHeight:0;
  focal=((H-gh)/2)/Math.tan(FOV/2); ccx=(W+cover)/2; ccy=(H-gh)/2;
}
function project(P){
  const v=sub(P,cam), z=dot(v,fwd);
  if(z<=0.02) return null;
  return {x:ccx+focal*dot(v,right)/z, y:ccy-focal*dot(v,tup)/z, z};
}

// ---- playback / view state ----
const N=DATA.meta.n_frames;
let frame=0, mode=DATA.meta.has_calibration?'cal':'raw', playing=false, SPEED=1;
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
    pos[b.seg]={prox:faced(prox(b.seg)), dist:faced(dist(b.seg,q[b.seg])),
      // the bone's own front and lateral directions, so its box rolls with the
      // measured twist (pronation / supination, humeral rotation)
      ant:faced(qrot(q[b.seg],FWD_W)), lat:faced(qrot(q[b.seg],LATERAL_W(b.seg)))};
  // ghost bones: absent ancestors (not the torso root) on some present lineage
  const gset=new Set();
  for(const b of bodies){ let s=ANAT_CHAIN[b.seg];
    while(s){ if(!present.has(s) && s!=='torso') gset.add(s); s=ANAT_CHAIN[s]; } }
  const ghosts=[];
  for(const s of gset) ghosts.push({seg:s, prox:faced(prox(s)), dist:faced(dist(s,IDENT))});
  const shoulder=s=>faced(prox(s));                  // for the girdle
  return {pos, ghosts, shoulder};
}

// ---- live joint angles (degrees), the same chain as metrics.py ----
// q_rel = conj(q_prox) ⊗ q_dist re-expressed in the baked anatomical frame,
// mirrored for the left side, split in the joint's sequence (ZXY elbow/wrist,
// YXY shoulder read clinically: plane+180°, elevation, plane+axial). A slot near
// its singularity reads null. Calibrated view with a known front only.
const QWA=DATA.meta.anatomical_frame_quat, JDEF=DATA.joint_defs||{};
const qconj=q=>[q[0],-q[1],-q[2],-q[3]];
const D2R=Math.PI/180, GUARD=Math.sin(10*D2R);
const wrapPi=a=>Math.PI-((((Math.PI-a)%(2*Math.PI))+2*Math.PI)%(2*Math.PI));
const bodyOf=seg=>bodies.find(b=>b.seg===seg);
function rotm(q){
  const n=Math.hypot(q[0],q[1],q[2],q[3])||1, [w,x,y,z]=q.map(v=>v/n);
  return [[1-2*(y*y+z*z),2*(x*y-w*z),2*(x*z+w*y)],
          [2*(x*y+w*z),1-2*(x*x+z*z),2*(y*z-w*x)],
          [2*(x*z-w*y),2*(y*z+w*x),1-2*(x*x+y*y)]];
}
function liveAngles(jk){
  const J=JDEF[jk]; if(!J||mode!=='cal'||!QWA) return null;
  const bp=bodyOf(J.proximal), bd=bodyOf(J.distal);
  if(!bp||!bd||!bp.calibrated||!bd.calibrated) return null;
  return jointAngles(J, segQuat(bp,frame), segQuat(bd,frame), QWA);
}
// One joint's angles from its two calibrated segment quaternions. With `qwa`
// the rotation is re-expressed in the anatomical frame (and the left side
// mirrored); without it, it is split in world axes — exactly what metrics.py
// does for a relative-only joint.
function jointAngles(J, qp, qd, qwa){
  let q=qmul(qconj(qp),qd);
  if(qwa){
    // the joint's own neutral frame (the wrist turns with the palms-in forearm)
    const h=(J.frame_y_deg||0)*D2R/2, f=qmul(qwa,[Math.cos(h),0,Math.sin(h),0]);
    q=qmul(qmul(qconj(f),q),f);
    if(J.distal.endsWith('_l')) q=[q[0],-q[1],-q[2],q[3]];
  }
  const R=rotm(q), cl=v=>Math.max(-1,Math.min(1,v));
  let a, ok;
  if(J.seq==='ZXY'){
    const b=Math.asin(cl(R[2][1]));
    a=[Math.atan2(-R[0][1],R[1][1]), b, Math.atan2(-R[2][0],R[2][2])];
    const w=Math.abs(Math.cos(b))>=GUARD; ok=[w,true,w];
  } else if(J.seq==='YXY'){
    const b=Math.acos(cl(R[1][1]));
    let al=Math.atan2(R[0][1],R[2][1]), ga=Math.atan2(R[1][0],-R[1][2]);
    if(Math.abs(Math.sin(b))<1e-6){ al=Math.atan2(-R[2][0],R[0][0]); ga=0; }
    a=[wrapPi(al+Math.PI), b, wrapPi(al+ga)];
    ok=[Math.abs(Math.sin(b))>=GUARD, true, b<=Math.PI-10*D2R];
  } else return null;
  const out={};
  for(const d of J.dofs) out[d.key]=ok[d.seq_index]?a[d.seq_index]/D2R:null;
  return out;
}
// A whole-session angle series for one movement, matching metrics.py: the
// calibrated stream (whatever the view toggle shows), unwrapped so a sweep past
// ±180° stays continuous, then shifted by a multiple of 360° so its middle
// reads within -180…180° (the same shift the panel applies). Undefined samples
// (near the decomposition's singularity) are null. Cached per movement.
const _series={};
function angleSeries(jk, dk){
  const key=jk+'.'+dk; if(key in _series) return _series[key];
  const J=JDEF[jk], bp=J&&bodyOf(J.proximal), bd=J&&bodyOf(J.distal);
  if(!bp||!bd) return (_series[key]=null);
  const out=new Array(N); let prev=null;
  for(let i=0;i<N;i++){
    const a=jointAngles(J, qmul(bp.frames[i],bp.offset), qmul(bd.frames[i],bd.offset), QWA);
    let v=a?a[dk]:null;
    if(v!=null&&prev!=null) v+=360*Math.round((prev-v)/360);   // unwrap
    out[i]=v; if(v!=null) prev=v;
  }
  const def=out.filter(v=>v!=null).sort((x,y)=>x-y);
  if(def.length){
    const k=360*Math.round(def[def.length>>1]/360);
    if(k) for(let i=0;i<N;i++) if(out[i]!=null) out[i]-=k;
  }
  return (_series[key]=out);
}
// kept for existing callers: the forearm's pronation (+) / supination (-)
function proSup(side){ const a=liveAngles('elbow_'+side); return a?a.pro_sup:null; }

// live readout box: every joint whose two sensors are placed, right side first
const liveEl=document.getElementById('live');
const LIVE_ORDER=['shoulder_r','elbow_r','wrist_r','shoulder_l','elbow_l','wrist_l'];
const LIVE_FMT={elevation:v=>`${Math.round(v)}° raise`,
                plane_elev:v=>`direction ${Math.round(v)}°`};
function fmtLive(key,v){
  if(v==null) return null;
  if(LIVE_FMT[key]) return LIVE_FMT[key](v);
  const info=DOF_INFO[key]; if(!info||!info.pos) return `${Math.round(v)}°`;
  return Math.abs(v)<0.5?`0° ${info.pos}`:`${Math.round(Math.abs(v))}° ${v>0?info.pos:info.neg}`;
}
function updateLive(){
  const joints=LIVE_ORDER.filter(k=>JDEF[k]&&present.has(JDEF[k].proximal)&&present.has(JDEF[k].distal));
  liveEl.hidden=!joints.length; if(!joints.length) return;
  const why=mode!=='cal'?'Switch to Calibrated to read the angles.'
          :!QWA?'Needs the front direction (see the header).':'';
  const rows=why?'':joints.map(k=>{
    const a=liveAngles(k);
    const vals=a?JDEF[k].dofs.map(d=>fmtLive(d.key,a[d.key])).filter(Boolean):[];
    return `<div class="lv-j"><div class="n">${nameOf(k)}</div>`+
      `<div class="v">${vals.length?vals.join(' · '):'<span class="lv-na">—</span>'}</div></div>`;
  }).join('');
  liveEl.innerHTML=`<h2>Live angles</h2>${rows}`+(why?`<div class="lv-why">${why}</div>`:'');
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
// 8 corners of an oriented box spanning A->B with a w×d cross-section. With
// `front` (the bone's own anterior direction) the depth axis v follows it, so
// the box ROLLS with the bone's twist and face BOX_FACES[3] (+v) is its front;
// without it a fixed world reference is used (roll not shown — the ghosts).
function boxBetween(A,B,w,d,front){
  let ax=sub(B,A); const Ln=Math.hypot(ax[0],ax[1],ax[2])||1; ax=scl(ax,1/Ln);
  let v;
  if(front){ v=sub(front,scl(ax,dot(front,ax))); }
  if(!v||Math.hypot(v[0],v[1],v[2])<1e-6){
    const ref=Math.abs(dot(ax,[0,0,1]))<0.9?[0,0,1]:[0,1,0];
    v=cross(ax,norm(cross(ax,ref)));
  }
  v=norm(v); const u=cross(v,ax), c=[];
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
      label([mid[0],mid[1],mid[2]+0.09],'no trunk sensor',cssVar('--faint'));
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
  // each limb as a solid shaded box between its two joints, rolled with the
  // bone's measured twist. Forearm and hand are FLAT (wide side-to-side, thin
  // front-to-back, like the real limb) so a twist visibly turns the slab; the
  // FRONT face (palm side in the anatomical position) is tinted lighter, the
  // same cue as the chest — palm-forward = supinated, palm-back = pronated.
  const raws=[], thumbs=[];
  const FLAT={forearm:[1.45,0.72], hand:[1.7,0.42]};      // [width, depth] × thick
  const lighten=c=>[c[0]*0.5+128,c[1]*0.5+128,c[2]*0.5+128];
  for(const b of bodies){
    if(b.seg==='torso') continue;
    const f=pos[b.seg], t=(ANAT[b.seg]&&ANAT[b.seg].thick)||.05;
    const k=FLAT[b.seg.replace(/_[lr]$/,'')]||[1,1];
    const box=boxBetween(f.prox, f.dist, t*k[0], t*k[1], f.ant);
    const col=hlBoost(b.seg,SEG[b.seg].color);
    BOX_FACES.forEach((fc,i)=>{ const p=faceOf(box,fc,i===3?lighten(col):col); if(p) polys.push(p); });
    if(!b.calibrated) raws.push([project(f.prox), project(f.dist)]);
    // thumb: on the hand if placed, else at the forearm's wrist end
    const side=b.seg.slice(-2), isHand=b.seg.startsWith('hand');
    if(isHand || (b.seg.startsWith('forearm') && !present.has('hand'+side)))
      thumbs.push({at:add(isHand?scl(add(f.prox,f.dist),0.5):f.dist, scl(f.lat,t*k[0]*0.62)),
                   r:t*(isHand?0.34:0.3), col:shade(SEG[b.seg].color,0.55)});
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
  // thumb nubs (lateral side in the anatomical position): with the palm forward
  // the thumb points out; pronate and it swings in toward the body.
  for(const th of thumbs) ball(th.at, th.r, th.col);
  updateLive();
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
    if(b.seg==='torso' || !(SHOW_LABELS || HILITE.has(b.seg))) continue;
    const raw=!b.calibrated;
    label(add(pos[b.seg].dist, scl((ANAT[b.seg]||{dir:[0,0,-1]}).dir,-0.02)),
          raw?shortOf(b.seg)+' · not calibrated':shortOf(b.seg),
          raw?rgb([204,120,20]):undefined);
  }
  for(const g of ghosts){
    if(!SHOW_LABELS) continue;
    const mid=scl(add(g.prox,g.dist),0.5);
    label([mid[0],mid[1],mid[2]+0.05], shortOf(g.seg)+' · no sensor', cssVar('--faint'));
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
  subEl=document.getElementById('sub');
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
// front-direction status (header chip tooltip)
const FACING=FRONT_KNOWN
  ? (HEADING.source==='manual'
      ? `Front direction entered by hand (${FACE_DEG.toFixed(0)}° from north).`
      : HEADING.source==='elbow_hinge'
        ? `Front direction found from how the elbow bends (${FACE_DEG.toFixed(0)}° from north).`
        : `Front direction found from the chest sensor (${FACE_DEG.toFixed(0)}° from north).`)
  : HEADING.source==='torso_auto'
    ? 'The chest sensor could not tell which way the person faced, so the FRONT arrow is a guess.'
    : HEADING.source==='elbow_hinge'
    ? 'The elbow did not bend enough to tell which way the person faced, so the FRONT arrow is a guess.'
    : 'No chest sensor, so the FRONT arrow is a guess. Enter the facing at calibration to fix this.';
const labelsBtn=document.getElementById('labels');
labelsBtn.addEventListener('click',()=>{
  SHOW_LABELS=!SHOW_LABELS; labelsBtn.classList.toggle('on',SHOW_LABELS);
});
const speedBox=document.getElementById('speed');
speedBox.addEventListener('click',e=>{
  const b=e.target.closest('button'); if(!b) return;
  SPEED=+b.dataset.speed;
  for(const x of speedBox.querySelectorAll('button')) x.classList.toggle('on',x===b);
});
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

// legend: one row per sensor, with its calibration state
const leg=document.getElementById('legend');
// ---- sync / data-loss quality (reconcile sidecar), per sensor ----
const QUAL=DATA.quality;
const qualOf=seg=>{
  if(!QUAL) return null;
  const s=DATA.segments.find(x=>x.segment===seg); if(!s) return null;
  return (QUAL.nodes||[]).find(n=>n.column===s.column)||null;
};
const lowSync=seg=>{const q=qualOf(seg); return !!(q&&!q.reference&&q.sync_reliable===false);};
for(const s of DATA.segments){
  const g=SEG[s.segment]||{color:[136,136,136]};
  const row=document.createElement('div'); row.className='legrow';
  const issues=lowSync(s.segment)?['low sync']:[];
  const bad=!s.calibrated||issues.length;
  const st=[s.calibrated?'calibrated':'not calibrated',...issues].join(' · ');
  row.innerHTML=`<span class="sw" style="background:${rgb(g.color)}"></span>`+
    `<span>${nameOf(s.segment)}</span><span class="st${bad?' no':''}">${st}</span>`;
  row.addEventListener('mouseenter',()=>{HILITE=new Set([s.segment]);});
  row.addEventListener('mouseleave',()=>{HILITE=new Set();});
  leg.appendChild(row);
}
// header: who / what / how long, plus two status chips
const m=DATA.meta;
const secs=(m.t1_ms-m.t0_ms)/1000;
const dur=secs>=90?`${Math.floor(secs/60)} min ${Math.round(secs%60)} s`:`${secs.toFixed(0)} s`;
subEl.innerHTML=`Subject <b>${esc(m.subject)}</b> · Session <b>${esc(m.session)}</b> · `+
  `${dur} recording · ${DATA.segments.length} sensor${DATA.segments.length===1?'':'s'}`;
subEl.title=`Data: ${m.source_csv}`+(m.calibration_source?` + ${m.calibration_source}`:'');
const ncal=DATA.segments.filter(s=>s.calibrated).length;
const chip=(cls,txt,tip)=>`<span class="chip ${cls}" title="${esc(tip)}">${txt}</span>`;
document.getElementById('chips').innerHTML=
  (ncal===DATA.segments.length
    ? chip('good','Calibrated','Every sensor was calibrated from a neutral pose.')
    : ncal
      ? chip('warn',`${ncal} of ${DATA.segments.length} calibrated`,'Some sensors were not calibrated; their angles are relative only.')
      : chip('warn','Not calibrated','No calibration: angles are relative only.'))+
  chip(FRONT_KNOWN?'good':'warn', FRONT_KNOWN?'Front known':'Front unknown', FACING)+
  syncChips();
// Sync: did every sensor's clock line up with the first one (enough shared
// motion)?
function syncChips(){
  if(DATA.segments.length<2) return '';
  if(!QUAL) return chip('muted','Sync not recorded',
    'This session was reconciled before sync quality was saved. Re-run the pipeline to record it.');
  const per=DATA.segments.map(s=>({s, q:qualOf(s.segment)})).filter(x=>x.q);
  const conf=per.filter(x=>!x.q.reference).map(x=>
    `${nameOf(x.s.segment)}: ${x.q.sync_confidence==null?'—':x.q.sync_confidence.toFixed(2)}`).join('\n');
  const low=per.filter(x=>lowSync(x.s.segment)).map(x=>nameOf(x.s.segment));
  return low.length
    ? chip('warn',`Sync low: ${low.join(', ')}`,
        `These sensors shared too little motion with the first sensor to line up their clocks reliably, so timing-based results (joint timing, fast-movement angles) may be off. Add a shared movement at the start of the session.\n\nSync confidence (need ${QUAL.confidence_min}):\n${conf}`)
    : chip('good','Sensors in sync',`Every sensor's clock lined up with the first one.\n\nSync confidence:\n${conf}`);
}

// ---- session metrics panel (built from the baked metrics.json) ----
// Plain-language session summary beside the 3-D body. Only what was actually
// computed is shown: blocked joints, movements undefined for the whole session,
// and empty values are left out. Relative-only joints keep a visible tag.
// Hovering a card highlights the bone(s) it measures.
function esc(s){return String(s).replace(/[&<>"]/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;'}[c]));}
const r0=x=>Math.round(x), r1=x=>Math.round(x*10)/10;
function segsFor(row){
  if(row.kind==='segment') return [row.data.segment];
  if(row.kind==='joint') return (DATA.joint_segments||{})[row.data.key]||[];
  if(row.kind==='derived'){
    const out=new Set();
    for(const jk of (row.data.requires||[]))
      for(const s of (DATA.joint_segments||{})[jk]||[jk]) out.add(s);
    return [...out];
  }
  return [];
}
// movement names + what a positive / negative angle means (clinical sign convention)
const DOF_INFO={
  flex_ext:{name:'Flexion / extension', pos:'flexion', neg:'extension'},
  pro_sup:{name:'Pronation / supination', pos:'pronation', neg:'supination'},
  rad_uln:{name:'Radial / ulnar deviation', pos:'ulnar', neg:'radial'},
  axial_rot:{name:'Internal / external rotation', pos:'internal', neg:'external'},
  elevation:{name:'Arm raise (elevation)'},
  plane_elev:{name:'Raise direction', note:'0° = out to the side, 90° = straight forward'},
};
function signed(v,info){
  if(!info||!info.pos) return `${r0(v)}°`;
  if(Math.abs(v)<0.5) return '0°';
  return `${r0(Math.abs(v))}° ${v>0?info.pos:info.neg}`;
}
const row=(k,v,d,dcls)=>`<div class="row"><span class="k">${k}</span><span class="v">${v}</span>`+
  (d?`<span class="d${dcls?' '+dcls:''}">${d}</span>`:'')+`</div>`;
function relReason(){
  if(!MET.calibration_used) return 'no calibration';
  if(!MET.anatomical_axes) return 'front direction unknown';
  return 'a sensor on this joint is not calibrated';
}
function jointCard(j){
  const dofs=j.dofs.filter(d=>d.rom);                 // skip undefined-all-session
  if(!dofs.length) return '';
  const rel=!j.clinical;
  const rows=dofs.map(d=>{
    const info=DOF_INFO[d.key]||{name:d.name};
    const range=`${r0(d.rom.range_deg)}°`;
    // metrics unwraps angles (so a sweep past ±180° stays continuous), which can
    // leave the whole span offset by a multiple of 360°; shift it back so its
    // middle reads within -180…180° (the range itself is unchanged)
    const k=Math.round(d.rom.median_deg/360)*360, lo=d.rom.min_deg-k, hi=d.rom.max_deg-k;
    const span=rel ? `from ${r0(lo)}° to ${r0(hi)}°`
                   : `from ${signed(lo,info)} to ${signed(hi,info)}`;
    const bits=[span];
    if(d.velocity&&d.velocity.peak_deg_s) bits.push(`fastest ${r0(d.velocity.peak_deg_s)}°/s`);
    if(info.note) bits.push(info.note);
    const part=('defined_frac' in d)&&d.defined_frac<0.995
      ? `measurable for ${r0(d.defined_frac*100)}% of the session (undefined with the arm at the side or overhead)` : '';
    return `<div class="row plot${rel?' clin-gated':''}" data-j="${j.key}" data-dof="${d.key}" `+
      `title="Show this movement over time"><span class="k">${esc(info.name)}</span>`+
      `<span class="v">${range}</span><span class="d">${bits.join(' · ')}</span>`+
      (part?`<span class="d warn">${part}</span>`:'')+`</div>`;
  }).join('');
  const reps=(j.reps&&j.reps.count)?`<span class="tag info">${j.reps.count} rep${j.reps.count===1?'':'s'}</span>`:'';
  const tag=rel?`<span class="tag rel" title="Angles are relative to the start pose, not the body: ${relReason()}.">relative only</span>`:'';
  const segs=(DATA.joint_segments||{})[j.key]||[];
  const syncNote=segs.some(lowSync)
    ?`<div class="mnote">Sensor timing uncertain (low sync) — angles during fast movement may be off.</div>`:'';
  return `<div class="mcard" data-i="${ROWS.push({kind:'joint',data:j})-1}">`+
    `<div class="mhead"><span class="mname">${esc(nameOf(j.key))}</span>${tag}${reps}</div>`+
    rows+(rel?`<div class="mnote">Relative only (${relReason()}): ranges are right, but zero is the start pose rather than the anatomical position.</div>`:'')+
    syncNote+`</div>`;
}
function segCard(sg){
  const rows=[];
  if(sg.travel){
    rows.push(row('Total rotation',`${r0(sg.travel.travel_deg)}°`,'all the turning this body part did'));
    rows.push(row('Time moving',`${r0(sg.travel.active_time_frac*100)}%`,'share of the session faster than 20°/s'));
  }
  if(sg.angular_speed&&sg.angular_speed.peak_deg_s)
    rows.push(row('Fastest turn',`${r0(sg.angular_speed.peak_deg_s)}°/s`,`average ${r0(sg.angular_speed.mean_deg_s)}°/s`));
  if(sg.calibrated&&sg.elevation)
    rows.push(row('Tilt from vertical',`${r0(sg.elevation.range_deg)}°`,`from ${r0(sg.elevation.min_deg)}° to ${r0(sg.elevation.max_deg)}°`));
  if(sg.smoothness_sparc!=null)
    rows.push(row('Smoothness',`${r1(sg.smoothness_sparc)}`,'closer to 0 is smoother (typically −1.5 smooth to −5 jerky)'));
  if(!rows.length) return '';
  return `<div class="mcard" data-i="${ROWS.push({kind:'segment',data:sg})-1}">`+
    `<div class="mhead"><span class="mname">${esc(nameOf(sg.segment))}</span></div>${rows.join('')}</div>`;
}
function derivedCard(dv){
  const M=dv.metrics||{}, t=dv.target||'', rel=dv.clinical===false;
  let title=dv.name, rows=[];
  const has=k=>M[k]!=null;
  if(t.startsWith('symmetry_')){
    title=`Left vs right range — ${nameOf(t.slice(9)+'_r').replace(/^Right /,'')}`;
    if(has('left_rom_deg')) rows.push(row('Left',`${r0(M.left_rom_deg)}°`));
    if(has('right_rom_deg')) rows.push(row('Right',`${r0(M.right_rom_deg)}°`));
    if(has('symmetry_index')) rows.push(row('Difference',`${r0(M.symmetry_index)}%`,'0% = both sides moved the same amount'));
  } else if(t.startsWith('activity_asymmetry_')){
    title=`Which side moved more — ${nameOf(t.slice(19)+'_r').replace(/^Right /,'')}`;
    if(has('left_travel_deg')) rows.push(row('Left total rotation',`${r0(M.left_travel_deg)}°`));
    if(has('right_travel_deg')) rows.push(row('Right total rotation',`${r0(M.right_travel_deg)}°`));
    if(has('asymmetry_index')){
      const a=M.asymmetry_index;
      rows.push(row('Balance',Math.abs(a)<5?'even':`${a>0?'right':'left'} +${r0(Math.abs(a))}`,
        '−100 = only the left moved · 0 = even · +100 = only the right'));
    }
  } else if(t.startsWith('coordination_')){
    const pr=M.pair||dv.requires||[];
    title=`Timing — ${pr.map(nameOf).join(' & ')}`;
    if(has('peak_r')) rows.push(row('Move together',`${r1(M.peak_r)}`,'correlation: 1 = in step, 0 = unrelated, −1 = opposite'));
    if(has('lag_s')&&pr.length===2) rows.push(row('Delay',`${Math.abs(M.lag_s).toFixed(2)} s`,
      Math.abs(M.lag_s)<0.01?'no delay':`${nameOf(M.lag_s>0?pr[1]:pr[0])} follows ${nameOf(M.lag_s>0?pr[0]:pr[1])}`));
  } else if(t.startsWith('compensation_')){
    title='Trunk movement during the task';
    if(has('trunk_travel_deg')) rows.push(row('Trunk total rotation',`${r0(M.trunk_travel_deg)}°`));
    if(has('trunk_elevation_range_deg')) rows.push(row('Trunk lean range',`${r0(M.trunk_elevation_range_deg)}°`));
  } else {
    for(const [k,v] of Object.entries(M)) if(v!=null&&!Array.isArray(v))
      rows.push(row(esc(k.replace(/_/g,' ')),esc(typeof v==='number'?r1(v):v)));
  }
  if(!rows.length) return '';
  const tag=rel?`<span class="tag rel">relative only</span>`:'';
  // timing comparisons depend on the sensors' clocks lining up
  let note='';
  if(t.startsWith('coordination_')){
    const segs=new Set(); for(const jk of (M.pair||dv.requires||[]))
      for(const sg of (DATA.joint_segments||{})[jk]||[]) segs.add(sg);
    const low=[...segs].filter(lowSync).map(nameOf);
    if(low.length) note=`<div class="mnote">Timing may be off: the clock of ${low.join(', ')} didn't line up reliably (low sync).</div>`;
  }
  return `<div class="mcard${rel?' clin-gated':''}" data-i="${ROWS.push({kind:'derived',data:dv})-1}">`+
    `<div class="mhead"><span class="mname">${esc(title)}</span>${tag}</div>${rows.join('')}${note}</div>`;
}
const ROWS=[];         // index -> {kind,data}, so a hovered card knows its segments
const MET=DATA.metrics, metricsEl=document.getElementById('metrics'),
  msToggle=document.getElementById('mstoggle');
if(MET){
  const sec=(title,inner)=>inner?`<section><h3>${title}</h3>${inner}</section>`:'';
  const joints=(MET.joints||[]).map(jointCard).join('');
  const segs=(MET.segments||[]).map(segCard).join('');
  // the trunk-compensation entry repeats per arm task with identical numbers;
  // show each distinct result once
  const seenDer=new Set();
  const der=(MET.derived||[]).filter(dv=>{
    if(!(dv.target||'').startsWith('compensation_')) return true;
    const k=JSON.stringify(dv.metrics); if(seenDer.has(k)) return false;
    seenDer.add(k); return true;
  }).map(derivedCard).join('');
  metricsEl.innerHTML=
    `<h2>Session metrics</h2>`+
    `<div class="msub">${dur} · ${r0(MET.sample_rate_hz)} samples per second · `+
      `click a movement to graph it</div>`+
    `<div class="rawbanner">You're viewing <b>raw</b> sensor orientation. The joint `+
      `angles below assume calibration — switch to <b>Calibrated</b> to see them `+
      `on the figure.</div>`+
    sec('Joints',joints)+
    sec('Comparisons',der)+
    sec('Body parts',segs)+
    ((joints||segs||der)?'':`<div class="msub">Nothing could be computed for this sensor setup.</div>`);

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
    msToggle.classList.toggle('primary',o); layoutOverlays();};
  msToggle.addEventListener('click',()=>openMetrics(metricsEl.classList.contains('hidden')));
  openMetrics(window.innerWidth>820);
}

// ---- angle-over-time graph ----
// Opened by clicking a movement in the metrics panel. Plots that movement for
// the whole session (plus the same movement on the other side, dashed), with
// the playhead synced to playback; click or drag on it to seek. Shaded: the
// neutral-pose window (green) and stretches where the angle is undefined.
const gcv=document.getElementById('gcv'), gctx=gcv.getContext('2d');
const gOther=document.getElementById('gother');
const otherSide=jk=>jk.endsWith('_r')?jk.slice(0,-2)+'_l':jk.slice(0,-2)+'_r';
function layoutOverlays(){
  const mEl=document.getElementById('metrics');
  const cover=(mEl&&!mEl.classList.contains('hidden')&&W>640)?mEl.offsetWidth:0;
  graphEl.style.left=cover+'px';
  const gh=(GRAPH&&!graphEl.hidden)?graphEl.offsetHeight:0;
  const lg=document.querySelector('.legend');
  if(lg) lg.style.maxHeight=`calc(100% - ${24+gh}px)`;
}
function openGraph(jk,dk){
  GRAPH={jk,dk}; graphEl.hidden=false;
  const J=(MET&&MET.joints||[]).find(j=>j.key===jk);
  const info=DOF_INFO[dk]||{name:dk};
  const ok=JDEF[otherSide(jk)]&&present.has(JDEF[otherSide(jk)].proximal)
           &&present.has(JDEF[otherSide(jk)].distal);
  document.getElementById('gotherwrap').style.display=ok?'':'none';
  document.getElementById('gtitle').innerHTML=`${esc(nameOf(jk))} — ${esc(info.name)}`+
    (J&&!J.clinical?' <span class="tag rel">relative only</span>':'');
  for(const r of document.querySelectorAll('#metrics .row.plot'))
    r.classList.toggle('sel',r.dataset.j===jk&&r.dataset.dof===dk);
  layoutOverlays(); resizeGraph();
}
function closeGraph(){
  GRAPH=null; graphEl.hidden=true;
  for(const r of document.querySelectorAll('#metrics .row.plot.sel')) r.classList.remove('sel');
  layoutOverlays();
}
document.getElementById('gclose').addEventListener('click',closeGraph);
document.addEventListener('keydown',e=>{ if(e.key==='Escape'&&GRAPH) closeGraph(); });
metricsEl.addEventListener('click',e=>{
  const r=e.target.closest('.row.plot'); if(!r) return;
  if(GRAPH&&GRAPH.jk===r.dataset.j&&GRAPH.dk===r.dataset.dof) closeGraph();
  else openGraph(r.dataset.j,r.dataset.dof);
});
let GW=0, GH=0;
function resizeGraph(){
  GW=gcv.clientWidth; GH=gcv.clientHeight;
  gcv.width=GW*DPR; gcv.height=GH*DPR;
}
window.addEventListener('resize',()=>{layoutOverlays(); if(GRAPH) resizeGraph();});
const G_PAD={l:58,r:18,t:12,b:26};
function niceStep(span,target){
  const raw=span/Math.max(1,target), p=Math.pow(10,Math.floor(Math.log10(raw)));
  for(const m of [1,2,2.5,5,10]) if(m*p>=raw) return m*p;
  return 10*p;
}
function drawGraph(){
  if(!GRAPH||graphEl.hidden||!GW) return;
  const g=gctx, {jk,dk}=GRAPH, info=DOF_INFO[dk]||{};
  g.setTransform(DPR,0,0,DPR,0,0); g.clearRect(0,0,GW,GH);
  const main=angleSeries(jk,dk), osk=otherSide(jk);
  const showOther=gOther.checked&&document.getElementById('gotherwrap').style.display!=='none';
  const other=showOther?angleSeries(osk,dk):null;
  const t0=DATA.t_ms[0], tEnd=DATA.t_ms[N-1], tspan=Math.max(1,tEnd-t0);
  const X=t=>G_PAD.l+(t-t0)/tspan*(GW-G_PAD.l-G_PAD.r);
  const vals=[...(main||[]),...(other||[])].filter(v=>v!=null);
  const leg=document.getElementById('gleg');
  const colOf=k=>rgb((SEG[(JDEF[k]||{}).distal]||{color:[120,120,120]}).color);
  leg.innerHTML=`<span><i style="border-color:${colOf(jk)}"></i>${esc(nameOf(jk))}</span>`+
    (other?`<span><i class="dash" style="border-color:${colOf(osk)}"></i>${esc(nameOf(osk))}</span>`:'');
  if(!vals.length){
    g.fillStyle=cssVar('--faint'); g.font='13px system-ui,sans-serif'; g.textAlign='center';
    g.fillText('This angle is undefined for the whole session.',GW/2,GH/2); return;
  }
  let lo=Math.min(...vals), hi=Math.max(...vals);
  if(hi-lo<20){const m=(hi+lo)/2; lo=m-10; hi=m+10;}
  const padv=(hi-lo)*0.08; lo-=padv; hi+=padv;
  const Y=v=>G_PAD.t+(hi-v)/(hi-lo)*(GH-G_PAD.t-G_PAD.b);
  const plotL=G_PAD.l, plotR=GW-G_PAD.r, plotT=G_PAD.t, plotB=GH-G_PAD.b;
  const ink=cssVar('--ink'), faint=cssVar('--faint'), grid=cssVar('--line');
  g.font='11px system-ui,sans-serif';
  // neutral-pose window
  const nw=DATA.meta.neutral_window_ms;
  if(nw){
    g.fillStyle=cssVar('--built'); g.globalAlpha=.10;
    g.fillRect(X(Math.max(t0,nw[0])),plotT,X(Math.min(tEnd,nw[1]))-X(Math.max(t0,nw[0])),plotB-plotT);
    g.globalAlpha=.8; g.textAlign='left'; g.fillText('neutral pose',X(Math.max(t0,nw[0]))+4,plotT+11);
    g.globalAlpha=1;
  }
  // undefined stretches of the main series
  if(main){
    g.fillStyle=faint; g.globalAlpha=.14;
    for(let i=0;i<N;){ if(main[i]!=null){i++;continue;}
      let j=i; while(j<N&&main[j]==null) j++;
      const a=X(DATA.t_ms[Math.max(0,i-1)]), b=X(DATA.t_ms[Math.min(N-1,j)]);
      g.fillRect(a,plotT,b-a,plotB-plotT); i=j; }
    g.globalAlpha=1;
  }
  // gridlines + y labels
  const ys=niceStep(hi-lo,4);
  g.strokeStyle=grid; g.lineWidth=1; g.fillStyle=faint; g.textAlign='right'; g.textBaseline='middle';
  for(let v=Math.ceil(lo/ys)*ys; v<=hi; v+=ys){
    const y=Y(v); g.globalAlpha=Math.abs(v)<1e-9?1:.7;
    g.beginPath(); g.moveTo(plotL,y); g.lineTo(plotR,y); g.stroke();
    g.globalAlpha=1; g.fillText(`${Math.round(v)}°`,plotL-8,y);
  }
  // time axis
  const xs=niceStep(tspan/1000,8)*1000;
  g.textAlign='center'; g.textBaseline='top';
  for(let t=0; t<=tspan+1; t+=xs) g.fillText(`${(t/1000).toFixed(xs<1000?1:0)} s`,X(t0+t),plotB+6);
  // what up / down mean
  if(info.pos){
    g.textAlign='left'; g.textBaseline='top'; g.fillStyle=faint;
    g.fillText(`↑ ${info.pos}`,4,plotT); g.textBaseline='bottom'; g.fillText(`↓ ${info.neg}`,4,plotB);
  }
  // series
  const line=(ser,color,dash,w)=>{
    if(!ser) return; g.strokeStyle=color; g.lineWidth=w; g.setLineDash(dash); g.beginPath();
    let pen=false;
    for(let i=0;i<N;i++){ const v=ser[i];
      if(v==null){pen=false;continue;}
      const x=X(DATA.t_ms[i]), y=Y(v);
      if(pen) g.lineTo(x,y); else {g.moveTo(x,y); pen=true;} }
    g.stroke(); g.setLineDash([]);
  };
  line(other,colOf(osk),[6,4],1.6);
  line(main,colOf(jk),[],2.4);
  // min / max of the main series
  if(main){
    const d=main.filter(v=>v!=null), mn=Math.min(...d), mx=Math.max(...d);
    g.strokeStyle=colOf(jk); g.globalAlpha=.45; g.setLineDash([3,4]); g.lineWidth=1;
    for(const v of [mn,mx]){ g.beginPath(); g.moveTo(plotL,Y(v)); g.lineTo(plotR,Y(v)); g.stroke(); }
    g.setLineDash([]); g.globalAlpha=1; g.fillStyle=colOf(jk); g.textAlign='right';
    g.textBaseline='bottom'; g.fillText(`max ${Math.round(mx)}°`,plotR-2,Y(mx)-2);
    g.textBaseline='top'; g.fillText(`min ${Math.round(mn)}°`,plotR-2,Y(mn)+2);
  }
  // playhead
  const px=X(DATA.t_ms[frame]);
  g.strokeStyle=cssVar('--accent'); g.lineWidth=1.5;
  g.beginPath(); g.moveTo(px,plotT); g.lineTo(px,plotB); g.stroke();
  for(const [ser,k] of [[other,osk],[main,jk]]){
    const v=ser&&ser[frame]; if(v==null) continue;
    g.fillStyle=colOf(k); g.beginPath(); g.arc(px,Y(v),4.2,0,7); g.fill();
    g.strokeStyle=cssVar('--surface'); g.lineWidth=1.5; g.stroke();
  }
  const cur=main&&main[frame];
  const txt=cur==null?'undefined here':fmtLive(dk,cur);
  g.font='600 12px system-ui,sans-serif'; g.fillStyle=ink; g.textBaseline='top';
  g.textAlign=px>GW-150?'right':'left';
  g.fillText(`${(( DATA.t_ms[frame]-t0)/1000).toFixed(2)} s · ${txt}`,px+(px>GW-150?-7:7),plotT+2);
}
// click / drag on the graph to seek
let gdrag=false;
function gseek(e){
  const r=gcv.getBoundingClientRect(), x=e.clientX-r.left;
  const f=(x-G_PAD.l)/(GW-G_PAD.l-G_PAD.r); if(!isFinite(f)) return;
  const tt=DATA.t_ms[0]+Math.max(0,Math.min(1,f))*(DATA.t_ms[N-1]-DATA.t_ms[0]);
  let lo=0,hi=N-1; while(hi-lo>1){const m=(lo+hi)>>1; if(DATA.t_ms[m]<tt) lo=m; else hi=m;}
  pause(); setFrame(Math.abs(DATA.t_ms[lo]-tt)<Math.abs(DATA.t_ms[hi]-tt)?lo:hi);
}
gcv.addEventListener('pointerdown',e=>{gdrag=true; gcv.setPointerCapture(e.pointerId); gseek(e);});
gcv.addEventListener('pointermove',e=>{if(gdrag) gseek(e);});
gcv.addEventListener('pointerup',()=>{gdrag=false;});

// ---- animation loop (real-time playback keyed on baked t_ms) ----
let last=performance.now(), acc=0;
function tick(now){
  const dt=now-last; last=now;
  if(playing && N>1){
    acc+=dt*SPEED;
    while(frame<N-1 && acc>=(DATA.t_ms[frame+1]-DATA.t_ms[frame])){
      acc-=(DATA.t_ms[frame+1]-DATA.t_ms[frame]); setFrame(frame+1);
    }
    if(frame>=N-1) pause();
  }
  render(); drawGraph();
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
