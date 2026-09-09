#!/usr/bin/env python3
"""
HULC Motion Shirt — stage-7 visual (segment tier): the floating-segment
free-body diagram.

The cheapest, most honest visual in the pipeline, because **orientation is
exactly what the shirt measures**: one magnetometer-referenced quaternion per
segment per frame drives one oriented body directly. This tool bakes the
reconcile → (optional) calibrate stream into a single self-contained HTML
viewer that draws each placed segment as its own oriented bar, floating at a
fixed slot, tilting/rolling as the subject moves.

Why "floating" (the honest caveat, made visual)
-----------------------------------------------
Orientation is MEASURED; position/connection is MODELED. A node tells you which
way its bone points, not where the bone is. So this first-tier FBD deliberately
does NOT connect segments into a skeleton — each body floats at a fixed anchor
and only its ORIENTATION is real. (The joint/chain tiers that add modeled
connection come later; see SETUP_AND_CALIBRATION_PLAN.md §5.)

What it shows — and why it validates stages 5-6
-----------------------------------------------
Feeding it a calibration.json makes the raw↔calibrated toggle the whole point:

  * RAW        each segment carries its unknown mounting tilt, so at the
               neutral pose the bars sit scattered / crooked.
  * CALIBRATED the cached per-segment mounting offset q_SB is applied
               (q_seg = q_WS ⊗ q_SB), so at the neutral pose every calibrated
               bar snaps upright and aligned.

Jump to the neutral window and flip the toggle: if calibration worked, the
scatter collapses to a clean N-pose. That is stage 5 (the solve) and stage 6
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
from motion_capabilities import SEGMENTS  # noqa: E402

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


def build_scene(csv_path, montage, calibration=None, max_frames=DEFAULT_MAX_FRAMES):
    """Bake a viewer-ready scene dict from the aligned stream.

    Bakes RAW world-from-sensor quaternions per segment plus, per segment, the
    single cached mounting offset (identity when uncalibrated). The viewer forms
    the calibrated orientation as q_seg = q_WS ⊗ q_SB on the fly, so both views
    come from one small payload.
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
        },
        "segments": segments,
        "t_ms": [round(float(t), 1) for t in t_keep],
        "frames": frames,
    }


# ---------------------------------------------------------------------------
# HTML emit
# ---------------------------------------------------------------------------
def render_html(scene):
    data_json = json.dumps(scene, separators=(",", ":"))
    title = (f"Floating-segment FBD — {scene['meta']['subject']} / "
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
    with open(path) as f:
        cal = json.load(f)
    cal["_path"] = path
    return cal


def cmd_render(args):
    montage = load_montage(args.montage)
    calibration = _load_calibration(args.calibration) if args.calibration else None
    scene = build_scene(args.aligned_csv, montage, calibration, args.max_frames)
    html = render_html(scene)
    with open(args.out, "w") as f:
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
                  f"toggle to see the mounting scatter collapse.")
    else:
        print("[fbd] no calibration given — RAW orientation only (each bar keeps "
              "its mounting tilt). Pass --calibration to enable the toggle.")
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
    from calibrate_segments import (qconj, build_calibration, WORLD_UP)

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

        scene = build_scene(csv, montage, cal, max_frames=200)
        html = render_html(scene)
        out = os.path.join(d, "fbd.html")
        with open(out, "w") as f:
            f.write(html)
        html_size = os.path.getsize(out)

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
    check(scene["meta"]["neutral_window_ms"] == [0.0, 4000.0],
          "neutral window carried through for the jump button")

    print(f"\n[selftest] {'PASS' if ok else 'FAIL'} — bake pipeline + the "
          f"raw↔calibrated promise the viewer renders.")
    return 0 if ok else 1


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd")

    pr = sub.add_parser("render", help="bake an aligned stream (+ calibration) "
                        "into a standalone HTML free-body viewer")
    pr.add_argument("aligned_csv", help="reconcile_nodes.py output CSV")
    pr.add_argument("montage", help="montage JSON (column<->segment mapping)")
    pr.add_argument("--calibration", help="calibration.json from "
                    "calibrate_segments.py (enables raw<->calibrated toggle)")
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
  .hint{position:absolute;left:12px;bottom:10px;color:var(--faint);
    font-size:11px;pointer-events:none}
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
  @media (max-width:640px){.legend{display:none}}
</style>
</head>
<body>
<header>
  <div class="eyebrow">Stage 7 &middot; segment tier</div>
  <h1>Floating-segment free-body diagram</h1>
  <div class="sub" id="sub">&mdash;</div>
  <div class="stage">Each bar is one segment, oriented by its measured
    quaternion and floating at a fixed slot. <b>Orientation is measured;
    position is not</b> &mdash; the bodies are drawn apart on purpose, because a
    node reports which way a bone points, not where it is. Drag to orbit &middot;
    scroll to zoom.</div>
</header>
<main>
  <canvas id="view"></canvas>
  <div class="legend"><h2>Segments</h2><div id="legend"></div></div>
  <div class="hint">world up = gravity (Z, blue axis) &middot; north = Y (green)
    &middot; each bar carries a small local triad so roll is visible</div>
</main>
<footer>
  <button id="play" class="primary">&#9654; Play</button>
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

// Fixed per-segment geometry + floating anchor slot. Data frame: X = subject
// L/R, Y = front/back, Z = up (gravity). Length runs along local +Z, so a
// calibrated neutral pose (offset applied, orientation ~identity) points every
// bar straight up.
const SEG = {
  torso:       {len:.52, cross:.24, color:[59,130,196],  anchor:[0,0,.0]},
  upper_arm_r: {len:.30, cross:.10, color:[228,87,46],   anchor:[-.58,0,-.02]},
  upper_arm_l: {len:.30, cross:.10, color:[242,165,65],  anchor:[ .58,0,-.02]},
  forearm_r:   {len:.27, cross:.08, color:[23,163,152],  anchor:[-.58,0,-.74]},
  forearm_l:   {len:.27, cross:.08, color:[124,181,24],  anchor:[ .58,0,-.74]},
  hand_r:      {len:.17, cross:.065,color:[111,75,216],  anchor:[-.58,0,-1.36]},
  hand_l:      {len:.17, cross:.065,color:[214,84,155],  anchor:[ .58,0,-1.36]},
};
const rgb = c => `rgb(${c[0]|0},${c[1]|0},${c[2]|0})`;
const shade = (c,f) => [c[0]*f,c[1]*f,c[2]*f];

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

const cvs=document.getElementById('view'), ctx=cvs.getContext('2d');
const UP=[0,0,1], FOV=45*Math.PI/180, LIGHT=norm([0.45,0.55,1.0]);
const GROUND=-1.72;
let DPR=1, W=0, H=0;

// A unit box's 6 faces as vertex-index quads (verts built per body below).
const BOX_FACES=[[0,1,3,2],[4,6,7,5],[0,4,5,1],[2,3,7,6],[0,2,6,4],[1,5,7,3]];

// ---- build per-body static geometry (local frame) ----
const bodies=[];
for(const s of DATA.segments){
  const g=SEG[s.segment]||{len:.25,cross:.08,color:[136,136,136],anchor:[0,0,0]};
  const cx=g.cross/2, cy=g.cross*0.62/2, L=g.len;
  // 8 corners: cross-section in X/Y, length 0..L along +Z (base at anchor).
  const verts=[
    [-cx,-cy,0],[cx,-cy,0],[-cx,cy,0],[cx,cy,0],
    [-cx,-cy,L],[cx,-cy,L],[-cx,cy,L],[cx,cy,L]];
  bodies.push({seg:s.segment, calibrated:s.calibrated, offset:s.offset,
    anchor:g.anchor, color:g.color, len:L, cross:g.cross, verts,
    frames:DATA.frames[s.segment]});
}

// ---- camera (Z-up spherical orbit) ----
const T=[0,0,-0.62]; let az=Math.PI*0.14, el=Math.PI*0.36, rad=3.15;
let cam, fwd, right, tup, focal, ccx, ccy;
function updateCamera(){
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

  // collect every box face as a polygon with a camera-depth key
  const polys=[];
  for(const b of bodies){
    const q=segQuat(b,frame);
    const world=b.verts.map(v=>add(b.anchor,qrot(q,v)));
    for(const face of BOX_FACES){
      const wp=face.map(i=>world[i]);
      const pp=wp.map(project);
      if(pp.some(p=>p===null)) continue;
      const nrm=norm(cross(sub(wp[1],wp[0]),sub(wp[2],wp[0])));
      // outward-normal light; use abs so inner faces aren't black
      const lit=0.55+0.45*Math.max(0,Math.abs(dot(nrm,LIGHT)));
      const depth=(pp[0].z+pp[1].z+pp[2].z+pp[3].z)/4;
      polys.push({pp, color:shade(b.color,lit), depth});
    }
  }
  // painter's algorithm: far first
  polys.sort((a,b)=>b.depth-a.depth);
  for(const p of polys){
    ctx.beginPath(); ctx.moveTo(p.pp[0].x,p.pp[0].y);
    for(let i=1;i<4;i++) ctx.lineTo(p.pp[i].x,p.pp[i].y);
    ctx.closePath();
    ctx.fillStyle=rgb(p.color); ctx.fill();
    ctx.lineWidth=1; ctx.strokeStyle='rgba(0,0,0,0.18)'; ctx.stroke();
  }

  // per-body overlays: local triad (roll cue) + distal cap + label
  ctx.lineWidth=2.5; ctx.lineCap='round';
  for(const b of bodies){
    const q=segQuat(b,frame), O=b.anchor, aL=b.cross*1.7;
    triad(O,q,aL);
    // distal end marker (bright dot) at local +Z tip
    const tip=project(add(O,qrot(q,[0,0,b.len])));
    if(tip){
      ctx.beginPath(); ctx.arc(tip.x,tip.y,4.5,0,7); ctx.fillStyle='#fff';
      ctx.fill(); ctx.lineWidth=2; ctx.strokeStyle=rgb(b.color); ctx.stroke();
    }
    // label above the tip
    const lp=project(add(O,qrot(q,[0,0,b.len+0.14])));
    if(lp){
      ctx.font='600 12px ui-monospace,monospace';
      ctx.textAlign='center'; ctx.textBaseline='bottom';
      ctx.fillStyle=cssVar('--ink'); ctx.globalAlpha=0.9;
      ctx.fillText(b.seg,lp.x,lp.y); ctx.globalAlpha=1;
    }
  }

  // world axis triad at origin (thin), so "up" is unambiguous
  ctx.lineWidth=2; triad([0,0,0],[1,0,0,0],0.34,true);
}

function line(A,B){
  const a=project(A), b=project(B); if(!a||!b) return;
  ctx.beginPath(); ctx.moveTo(a.x,a.y); ctx.lineTo(b.x,b.y); ctx.stroke();
}
const AXC=['rgba(224,64,64,.95)','rgba(40,170,90,.95)','rgba(60,120,230,.95)'];
function triad(O,q,L,world){
  const axes=[[L,0,0],[0,L,0],[0,0,L]];
  for(let k=0;k<3;k++){
    const a=project(O), c=project(add(O,world?axes[k]:qrot(q,axes[k])));
    if(!a||!c) continue;
    ctx.strokeStyle=AXC[k]; ctx.beginPath();
    ctx.moveTo(a.x,a.y); ctx.lineTo(c.x,c.y); ctx.stroke();
  }
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
  modeBox=document.getElementById('mode'), subEl=document.getElementById('sub');
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
