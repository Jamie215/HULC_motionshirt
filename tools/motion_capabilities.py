#!/usr/bin/env python3
"""
HULC Motion Shirt — montage schema + capability resolver.

Bridges the aligned multi-node stream (the CSV emitted by reconcile_nodes.py)
to upper-extremity *biomechanics*. It answers one question:

    "Given WHERE the nodes are placed this session, WHICH joints and metrics
     can we actually compute — and which are blocked, and why?"

Why this exists
---------------
An IMU node measures the ORIENTATION of the body SEGMENT it is strapped to,
not a joint. A clinical joint angle (the thing "range of motion" is about) is
the RELATIVE orientation between two ADJACENT segments' nodes:

    shoulder = torso        -> upper_arm      (needs BOTH nodes)
    elbow    = upper_arm    -> forearm
    wrist    = forearm      -> hand

So the set of computable joints/metrics is a function of the node placement —
the "montage". A montage missing the torso node can still report how the upper
arm moved in space, but it CANNOT separate shoulder motion from trunk motion.
This tool makes that dependency explicit instead of silently reporting a wrong
number.

What it is NOT
--------------
It does not compute angles. It resolves *capability* — the contract every
later metric plugin builds against. Angle decomposition, ROM, smoothness, etc.
are separate steps that consume the aligned CSV once this says they're valid.
The intended decomposition convention for each joint is carried here as
advisory metadata (ISB / Wu et al. 2005 sequences) so those steps stay
consistent.

Usage
-----
    # print a blank example montage you can fill in:
    python tools/motion_capabilities.py --example > montage.json

    # resolve a montage and print the capability report:
    python tools/motion_capabilities.py montage.json

    # same, as machine-readable JSON (for a UI to consume):
    python tools/motion_capabilities.py montage.json --json

    # validate the model + resolver logic with no montage file:
    python tools/motion_capabilities.py --selftest

Montage schema (see tools/MONTAGE_SCHEMA.md for the full write-up)
-----------------------------------------------------------------
    {
      "schema_version": "1.0",
      "subject":  {"id": "S01"},
      "session":  {"id": "2026-09-04-A", "aligned_csv": "aligned.csv"},
      "calibration": {"neutral_pose": "N-pose", "captured": true,
                      "t_window_ms": [1000, 4000], "functional": []},
      "nodes": [
        {"node_id": "HULC-IMU-D067", "column": "n0", "segment": "torso",       "calibrated": true},
        {"node_id": "HULC-IMU-A1B2", "column": "n1", "segment": "upper_arm_r", "calibrated": true},
        {"node_id": "HULC-IMU-C3D4", "column": "n2", "segment": "forearm_r",   "calibrated": false}
      ]
    }

`column` is the per-node prefix in the reconcile_nodes.py output header
(t_common_ms, n0_qw, n0_qx, ...), so a montage binds anatomy to that CSV
directly.
"""

import argparse
import json
import sys
from dataclasses import dataclass, field
from typing import Optional

SCHEMA_VERSION = "1.0"

# ─────────────────────────────────────────────────────────────────────────────
# 1. Canonical anatomical model (fixed reference data — NOT user-supplied)
#
# The upper extremity is modeled as a rigid kinematic chain rooted at the torso.
# Sides are suffixed _l / _r. Each JOINT names its proximal/distal SEGMENT and
# the degrees of freedom (DOF) that a two-node relative orientation can resolve,
# with the clinically-named angle and the intended decomposition convention.
# ─────────────────────────────────────────────────────────────────────────────

# Canonical segments (the strap-on locations a node can occupy).
SEGMENTS = {
    "torso":        "Trunk — root reference frame for the shoulders.",
    "upper_arm_l":  "Left upper arm (humerus).",
    "upper_arm_r":  "Right upper arm (humerus).",
    "forearm_l":    "Left forearm.",
    "forearm_r":    "Right forearm.",
    "hand_l":       "Left hand / dorsum.",
    "hand_r":       "Right hand / dorsum.",
}


def _seg_base(seg: str) -> str:
    """Segment key without its side suffix ('upper_arm_l' -> 'upper_arm')."""
    return seg[:-2] if seg.endswith(("_l", "_r")) else seg


def _seg_side(seg: str) -> Optional[str]:
    """'l' / 'r' side of a segment key, or None for a midline segment."""
    return seg[-1] if seg.endswith(("_l", "_r")) else None


# ─────────────────────────────────────────────────────────────────────────────
# 1a. Landmark labels — how a user names where a node sits vs the SEGMENT it
#     resolves to. Nodes are placed around anatomical landmarks; a landmark that
#     names a SEGMENT (humerus, forearm, torso, hand) maps cleanly, but a
#     landmark that names a JOINT (shoulder, elbow, wrist) is AMBIGUOUS — it
#     spans two segments — so `segment` stays authoritative and we only warn.
# ─────────────────────────────────────────────────────────────────────────────

# Substring -> segment base for the unambiguous, segment-naming landmarks.
LANDMARK_SEGMENT_HINT = {
    "humerus": "upper_arm", "upper arm": "upper_arm", "upperarm": "upper_arm",
    "forearm": "forearm", "radius": "forearm", "ulna": "forearm",
    "torso": "torso", "trunk": "torso", "sternum": "torso", "chest": "torso",
    "hand": "hand", "dorsum": "hand", "metacarpal": "hand",
}

# Landmarks that name a JOINT, not a segment — inherently ambiguous.
AMBIGUOUS_LANDMARKS = ("shoulder", "elbow", "wrist")


def landmark_hint(landmark: str):
    """Parse a free-text landmark label.

    Returns (segment_base | None, side | None, is_joint). segment_base is None
    when the label names a joint or nothing recognizable; is_joint flags a
    joint-named (ambiguous) landmark.
    """
    s = landmark.lower()
    side = "l" if "left" in s else ("r" if "right" in s else None)
    is_joint = any(a in s for a in AMBIGUOUS_LANDMARKS)
    base = None
    for kw, seg in LANDMARK_SEGMENT_HINT.items():
        if kw in s:
            base = seg
            break
    return base, side, is_joint


@dataclass(frozen=True)
class DOF:
    """One resolvable degree of freedom of a joint (a clinical angle)."""
    key: str
    name: str                 # clinical name
    plane: str                # anatomical plane / axis it lives in
    needs_calibration: bool = True   # anatomical-frame cal needed for a valid number


@dataclass(frozen=True)
class Joint:
    """A joint = relative orientation of `distal` w.r.t. `proximal` segment."""
    key: str
    name: str
    proximal: str
    distal: str
    dofs: tuple                # tuple[DOF]
    decomposition: str        # advisory: intended Euler sequence (ISB/Wu 2005)
    caveat: str = ""


# ISB / Wu et al. (2005) recommended rotation sequences are carried as advisory
# strings — the resolver declares capability; a downstream step does the math.
JOINTS = {
    "shoulder_r": Joint(
        key="shoulder_r", name="Right shoulder (glenohumeral+scapular)",
        proximal="torso", distal="upper_arm_r",
        dofs=(
            DOF("flex_ext",  "Flexion / extension",            "sagittal"),
            DOF("abd_add",   "Abduction / adduction",          "frontal"),
            DOF("int_ext_rot","Internal / external rotation",  "transverse"),
        ),
        decomposition="YXY (plane of elevation, elevation, axial rotation)",
        caveat="Trunk motion contaminates this unless the torso node is present "
               "and calibrated; without torso, report upper-arm elevation only.",
    ),
    "shoulder_l": Joint(
        key="shoulder_l", name="Left shoulder (glenohumeral+scapular)",
        proximal="torso", distal="upper_arm_l",
        dofs=(
            DOF("flex_ext",  "Flexion / extension",            "sagittal"),
            DOF("abd_add",   "Abduction / adduction",          "frontal"),
            DOF("int_ext_rot","Internal / external rotation",  "transverse"),
        ),
        decomposition="YXY (plane of elevation, elevation, axial rotation)",
        caveat="Trunk motion contaminates this unless the torso node is present "
               "and calibrated; without torso, report upper-arm elevation only.",
    ),
    "elbow_r": Joint(
        key="elbow_r", name="Right elbow + forearm",
        proximal="upper_arm_r", distal="forearm_r",
        dofs=(
            DOF("flex_ext",  "Flexion / extension",            "sagittal"),
            DOF("pro_sup",   "Pronation / supination",         "transverse"),
        ),
        decomposition="ZXY (flexion primary; axial = pro/supination)",
        caveat="Pronation/supination is a radioulnar rotation seen here as "
               "forearm axial rotation vs the humerus; sensitive to forearm "
               "node roll placement — calibrate axial zero explicitly.",
    ),
    "elbow_l": Joint(
        key="elbow_l", name="Left elbow + forearm",
        proximal="upper_arm_l", distal="forearm_l",
        dofs=(
            DOF("flex_ext",  "Flexion / extension",            "sagittal"),
            DOF("pro_sup",   "Pronation / supination",         "transverse"),
        ),
        decomposition="ZXY (flexion primary; axial = pro/supination)",
        caveat="Pronation/supination is a radioulnar rotation seen here as "
               "forearm axial rotation vs the humerus; sensitive to forearm "
               "node roll placement — calibrate axial zero explicitly.",
    ),
    "wrist_r": Joint(
        key="wrist_r", name="Right wrist",
        proximal="forearm_r", distal="hand_r",
        dofs=(
            DOF("flex_ext",  "Flexion / extension",            "sagittal"),
            DOF("rad_uln",   "Radial / ulnar deviation",       "frontal"),
        ),
        decomposition="ZXY (flex/ext, deviation)",
    ),
    "wrist_l": Joint(
        key="wrist_l", name="Left wrist",
        proximal="forearm_l", distal="hand_l",
        dofs=(
            DOF("flex_ext",  "Flexion / extension",            "sagittal"),
            DOF("rad_uln",   "Radial / ulnar deviation",       "frontal"),
        ),
        decomposition="ZXY (flex/ext, deviation)",
    ),
}

# ─────────────────────────────────────────────────────────────────────────────
# 2. Metric catalog — what each capability tier can produce.
#
#   segment-level  : needs 1 node   (orientation is enough)
#   joint-level    : needs 2 adjacent nodes (a relative orientation)
#   derived        : needs a set of joints/segments (symmetry, coordination...)
#
# Each metric flags whether it depends on anatomical calibration and which
# quality inputs gate its trust (so a UI never shows a number without them).
# ─────────────────────────────────────────────────────────────────────────────

@dataclass(frozen=True)
class Metric:
    key: str
    name: str
    tier: str                  # "segment" | "joint" | "derived"
    needs_calibration: bool
    quality_inputs: tuple      # trust gates a UI must surface alongside the value


SEGMENT_METRICS = (
    Metric("elevation",     "Segment elevation from vertical", "segment",
           needs_calibration=False, quality_inputs=("dropout", "sensor_cal")),
    Metric("angular_speed", "Angular speed magnitude",         "segment",
           needs_calibration=False, quality_inputs=("dropout", "sensor_cal")),
    Metric("smoothness",    "Movement smoothness (SPARC/jerk)","segment",
           needs_calibration=False, quality_inputs=("dropout", "sensor_cal")),
    Metric("posture_dwell", "Time-in-posture histogram",       "segment",
           needs_calibration=True,  quality_inputs=("dropout", "sensor_cal",
                                                     "stillness_confirmed")),
)

JOINT_METRICS = (
    Metric("angle_series",  "Per-DOF joint angle time series",  "joint",
           needs_calibration=True,  quality_inputs=("dropout", "sensor_cal",
                                                     "sync_confidence")),
    Metric("rom",           "Range of motion (min/max/range)",  "joint",
           needs_calibration=True,  quality_inputs=("dropout", "sensor_cal",
                                                     "sync_confidence")),
    Metric("joint_velocity","Joint angular velocity",           "joint",
           needs_calibration=True,  quality_inputs=("dropout", "sync_confidence")),
    Metric("rep_count",     "Repetition / bout segmentation",   "joint",
           needs_calibration=False, quality_inputs=("dropout", "sync_confidence")),
)


@dataclass
class Capability:
    """One resolved thing you can compute this session (or a reason you can't)."""
    kind: str                     # "segment" | "joint" | "derived"
    target: str                   # segment/joint key
    name: str
    computable: bool
    metrics: list = field(default_factory=list)
    dofs: list = field(default_factory=list)
    requires: list = field(default_factory=list)   # segment keys this needs
    missing: list = field(default_factory=list)     # of `requires`, what's absent
    warnings: list = field(default_factory=list)
    decomposition: Optional[str] = None


# ─────────────────────────────────────────────────────────────────────────────
# 3. Montage validation + resolution
# ─────────────────────────────────────────────────────────────────────────────

def validate_montage(montage: dict) -> list:
    """Return a list of hard errors (empty = valid). Warnings come later."""
    errors = []
    nodes = montage.get("nodes")
    if not isinstance(nodes, list) or not nodes:
        return ["montage has no 'nodes' list"]

    seen_seg, seen_col = {}, {}
    for i, n in enumerate(nodes):
        where = f"nodes[{i}]"
        seg = n.get("segment")
        col = n.get("column")
        if seg is None:
            errors.append(f"{where}: missing 'segment'")
        elif seg not in SEGMENTS:
            errors.append(f"{where}: unknown segment '{seg}' "
                          f"(known: {', '.join(sorted(SEGMENTS))})")
        elif seg in seen_seg:
            errors.append(f"{where}: segment '{seg}' already assigned to "
                          f"nodes[{seen_seg[seg]}]")
        else:
            seen_seg[seg] = i

        if col is None:
            errors.append(f"{where}: missing 'column' (the reconcile CSV prefix, "
                          f"e.g. 'n0')")
        elif col in seen_col:
            errors.append(f"{where}: column '{col}' already used by "
                          f"nodes[{seen_col[col]}]")
        else:
            seen_col[col] = i
    return errors


def montage_warnings(montage: dict) -> list:
    """Advisory (non-fatal) notes about node landmark labels vs their segment.

    A `landmark` is the user's human description of where a node sits. It does
    not drive resolution — `segment` does — but a landmark that names a JOINT is
    ambiguous, and one that disagrees with its segment is likely a mislabel.
    """
    warns = []
    for i, n in enumerate(montage.get("nodes", [])):
        lm = n.get("landmark")
        seg = n.get("segment")
        if not lm:
            continue
        base, side, is_joint = landmark_hint(lm)
        if is_joint and base is None:
            warns.append(
                f"nodes[{i}] landmark '{lm}' names a JOINT, which spans two "
                f"segments — a node sits on one bone, so '{seg}' is what it "
                f"actually represents.")
            continue
        if base and seg and _seg_base(seg) != base:
            warns.append(
                f"nodes[{i}] landmark '{lm}' suggests segment '{base}' but is "
                f"mapped to '{seg}' — check the placement/mapping.")
        if side and seg and _seg_side(seg) and side != _seg_side(seg):
            said = "left" if side == "l" else "right"
            warns.append(
                f"nodes[{i}] landmark '{lm}' says {said}, but segment '{seg}' "
                f"is the other side.")
    return warns


def _calibrated(montage: dict, segment: str) -> bool:
    """Is the node on `segment` anatomically calibrated (and cal captured)?"""
    cal = montage.get("calibration", {})
    if not cal.get("captured"):
        return False
    for n in montage.get("nodes", []):
        if n.get("segment") == segment:
            return bool(n.get("calibrated"))
    return False


def resolve(montage: dict) -> list:
    """Resolve a validated montage into a list[Capability]."""
    present = {n["segment"]: n for n in montage["nodes"] if n.get("segment")}
    caps: list = []

    # 3a. Segment-level capabilities (need one node).
    for seg in SEGMENTS:
        if seg not in present:
            continue
        seg_cal = _calibrated(montage, seg)
        metrics = []
        warns = []
        for m in SEGMENT_METRICS:
            if m.needs_calibration and not seg_cal:
                warns.append(f"{m.name}: uncalibrated — value is relative, not "
                             f"anatomical")
            metrics.append(m.key)
        caps.append(Capability(
            kind="segment", target=seg, name=SEGMENTS[seg], computable=True,
            metrics=metrics, requires=[seg], warnings=warns))

    # 3b. Joint-level capabilities (need both adjacent nodes).
    computable_joints = set()
    for jkey, j in JOINTS.items():
        need = [j.proximal, j.distal]
        missing = [s for s in need if s not in present]
        cap = Capability(
            kind="joint", target=jkey, name=j.name,
            computable=not missing, requires=need, missing=missing,
            decomposition=j.decomposition,
            dofs=[{"key": d.key, "name": d.name, "plane": d.plane} for d in j.dofs],
        )
        if missing:
            cap.warnings.append(
                "blocked: missing node(s) on " + ", ".join(missing))
        else:
            computable_joints.add(jkey)
            cap.metrics = [m.key for m in JOINT_METRICS]
            both_cal = all(_calibrated(montage, s) for s in need)
            if not both_cal:
                cap.warnings.append(
                    "angle/ROM are RELATIVE only — one or both nodes lack "
                    "anatomical calibration; capture a neutral pose to get "
                    "clinical angles")
            if j.caveat:
                cap.warnings.append(j.caveat)
        caps.append(cap)

    # 3c. Derived capabilities (need a set of joints/segments).
    #     Bilateral JOINT symmetry: both sides of a joint computable (angle/ROM
    #     level — needs the two-node pair on each side).
    for base in ("shoulder", "elbow", "wrist"):
        l, r = f"{base}_l", f"{base}_r"
        if l in computable_joints and r in computable_joints:
            caps.append(Capability(
                kind="derived", target=f"symmetry_{base}",
                name=f"{base.title()} L/R symmetry", computable=True,
                metrics=["symmetry_index", "rom_ratio"], requires=[l, r]))

    #     Bilateral ACTIVITY asymmetry: a matching L/R SEGMENT pair (one node
    #     each side). This is the "which arm is used more" comparison — it needs
    #     no joint and no torso, only the two segment nodes. Its metrics are
    #     session AGGREGATES of activity (integrated angular travel, active-time
    #     fraction), so they are robust even when reconcile cannot time-align the
    #     two sides — independent-limb motion (the low-confidence sync case)
    #     still yields a valid asymmetry number.
    for base in ("upper_arm", "forearm", "hand"):
        l, r = f"{base}_l", f"{base}_r"
        if l in present and r in present:
            caps.append(Capability(
                kind="derived", target=f"activity_asymmetry_{base}",
                name=f"{base.replace('_', ' ').title()} L/R activity asymmetry",
                computable=True,
                metrics=["asymmetry_index", "use_ratio", "active_time_ratio"],
                requires=[l, r],
                warnings=["aggregate over the session — does not need the two "
                          "sides time-aligned, so it holds even when their sync "
                          "confidence is low (independent motion)"]))

    #     Inter-joint coordination: any 2+ computable joints on a side.
    for side in ("l", "r"):
        js = [j for j in computable_joints if j.endswith(f"_{side}")]
        if len(js) >= 2:
            caps.append(Capability(
                kind="derived", target=f"coordination_{side}",
                name=f"Inter-joint coordination ({side.upper()})",
                computable=True, metrics=["cross_correlation", "timing_lag"],
                requires=js))

    #     Trunk compensation: torso present AND a shoulder computable.
    if "torso" in present:
        for jkey in ("shoulder_l", "shoulder_r"):
            if jkey in computable_joints:
                caps.append(Capability(
                    kind="derived", target=f"compensation_{jkey}",
                    name=f"Trunk compensation during {jkey}", computable=True,
                    metrics=["trunk_excursion"], requires=["torso", jkey]))
    return caps


# ─────────────────────────────────────────────────────────────────────────────
# 4. Reporting
# ─────────────────────────────────────────────────────────────────────────────

def to_dict(caps: list, montage: Optional[dict] = None) -> dict:
    """Machine-readable capability report (for a UI to consume)."""
    def cap_d(c: Capability) -> dict:
        d = {"kind": c.kind, "target": c.target, "name": c.name,
             "computable": c.computable, "metrics": c.metrics,
             "requires": c.requires}
        if c.dofs:           d["dofs"] = c.dofs
        if c.missing:        d["missing"] = c.missing
        if c.decomposition:  d["decomposition"] = c.decomposition
        if c.warnings:       d["warnings"] = c.warnings
        return d
    out = {
        "schema_version": SCHEMA_VERSION,
        "segments": [cap_d(c) for c in caps if c.kind == "segment"],
        "joints":   [cap_d(c) for c in caps if c.kind == "joint"],
        "derived":  [cap_d(c) for c in caps if c.kind == "derived"],
    }
    if montage is not None:
        out["montage_warnings"] = montage_warnings(montage)
    return out


def print_report(montage: dict, caps: list) -> None:
    subj = montage.get("subject", {}).get("id", "?")
    sess = montage.get("session", {}).get("id", "?")
    cal = montage.get("calibration", {})
    print(f"Montage — subject {subj}, session {sess}")
    print(f"  nodes: {len(montage.get('nodes', []))}   "
          f"calibration captured: {bool(cal.get('captured'))}")
    mw = montage_warnings(montage)
    if mw:
        print("  labeling notes:")
        for w in mw:
            print(f"    ! {w}")
    print()

    joints = [c for c in caps if c.kind == "joint"]
    ok = [c for c in joints if c.computable]
    print(f"JOINTS  ({len(ok)}/{len(joints)} computable)")
    for c in joints:
        mark = "✓" if c.computable else "✗"
        dof_s = ", ".join(d["name"] for d in c.dofs)
        print(f"  {mark} {c.target:<12} {c.name}")
        print(f"       DOF: {dof_s}")
        if c.computable:
            print(f"       metrics: {', '.join(c.metrics)}")
            print(f"       decomposition: {c.decomposition}")
        for w in c.warnings:
            print(f"       ! {w}")
    print()

    segs = [c for c in caps if c.kind == "segment"]
    print(f"SEGMENTS  ({len(segs)} placed)")
    for c in segs:
        print(f"  ✓ {c.target:<12} metrics: {', '.join(c.metrics)}")
        for w in c.warnings:
            print(f"       ! {w}")
    print()

    derived = [c for c in caps if c.kind == "derived"]
    print(f"DERIVED  ({len(derived)} available)")
    for c in derived:
        print(f"  ✓ {c.target:<24} {c.name}: {', '.join(c.metrics)}")
        for w in c.warnings:
            print(f"       ! {w}")
    if not derived:
        print("  (none — add the missing nodes above to unlock asymmetry / "
              "symmetry / coordination / compensation)")


# ─────────────────────────────────────────────────────────────────────────────
# 5. Example montage + self-test
# ─────────────────────────────────────────────────────────────────────────────

def example_montage() -> dict:
    """A fillable example — right-arm chain + torso, forearm uncalibrated."""
    return {
        "schema_version": SCHEMA_VERSION,
        "subject": {"id": "S01", "notes": ""},
        "session": {"id": "2026-09-04-A", "aligned_csv": "aligned.csv"},
        "calibration": {"neutral_pose": "N-pose", "captured": True,
                        "t_window_ms": [1000, 4000], "functional": []},
        "nodes": [
            {"node_id": "HULC-IMU-D067", "column": "n0", "segment": "torso",       "landmark": "sternum",        "calibrated": True},
            {"node_id": "HULC-IMU-A1B2", "column": "n1", "segment": "upper_arm_r", "landmark": "right humerus",  "calibrated": True},
            {"node_id": "HULC-IMU-C3D4", "column": "n2", "segment": "forearm_r",   "landmark": "right forearm",  "calibrated": False},
            {"node_id": "HULC-IMU-E5F6", "column": "n3", "segment": "hand_r",      "landmark": "right hand",     "calibrated": True},
        ],
    }


def selftest() -> None:
    print("[selftest] validating anatomical model ...")
    # Every joint references known segments.
    for jkey, j in JOINTS.items():
        assert j.proximal in SEGMENTS, f"{jkey}: bad proximal {j.proximal}"
        assert j.distal in SEGMENTS, f"{jkey}: bad distal {j.distal}"
        assert j.dofs, f"{jkey}: no DOFs"
    print(f"           {len(SEGMENTS)} segments, {len(JOINTS)} joints — OK")

    # Full right-arm montage: shoulder+elbow+wrist all computable, coordination
    # + compensation unlocked.
    m = example_montage()
    assert not validate_montage(m), "example montage should validate"
    caps = resolve(m)
    jok = {c.target for c in caps if c.kind == "joint" and c.computable}
    assert {"shoulder_r", "elbow_r", "wrist_r"} <= jok, jok
    assert not any(c.computable for c in caps
                   if c.kind == "joint" and c.target.endswith("_l")), \
        "no left-side nodes → left joints must be blocked"
    dkeys = {c.target for c in caps if c.kind == "derived"}
    assert "coordination_r" in dkeys and "compensation_shoulder_r" in dkeys, dkeys
    print(f"[selftest] full R-arm montage: joints {sorted(jok)} — OK")

    # Drop the torso node: shoulder becomes NON-computable, elbow/wrist survive.
    m2 = example_montage()
    m2["nodes"] = [n for n in m2["nodes"] if n["segment"] != "torso"]
    caps2 = resolve(m2)
    sh = next(c for c in caps2 if c.target == "shoulder_r")
    assert not sh.computable and sh.missing == ["torso"], sh.missing
    el = next(c for c in caps2 if c.target == "elbow_r")
    assert el.computable, "elbow should survive torso removal"
    print("[selftest] torso removed: shoulder blocked (missing torso), "
          "elbow/wrist OK — OK")

    # Uncalibrated forearm → elbow ROM flagged relative-only.
    el_warn = " ".join(next(c for c in caps if c.target == "elbow_r").warnings)
    assert "RELATIVE only" in el_warn, el_warn
    print("[selftest] uncalibrated node surfaces a relative-only warning — OK")

    # Duplicate segment assignment is a hard error.
    bad = example_montage()
    bad["nodes"].append({"node_id": "X", "column": "n9", "segment": "torso",
                         "calibrated": True})
    assert any("already assigned" in e for e in validate_montage(bad))
    print("[selftest] duplicate-segment montage rejected — OK")

    # Bilateral activity asymmetry: two upper-arm nodes (no torso, no joints)
    # still unlock the "which arm is used more" comparison. This is the sparse
    # left/right montage of case 1.
    bilat = {"schema_version": SCHEMA_VERSION, "subject": {"id": "S"},
             "session": {"id": "bilat"}, "calibration": {"captured": False},
             "nodes": [
                 {"node_id": "L", "column": "n0", "segment": "upper_arm_l"},
                 {"node_id": "R", "column": "n1", "segment": "upper_arm_r"}]}
    assert not validate_montage(bilat)
    bcaps = resolve(bilat)
    assert not any(c.computable for c in bcaps if c.kind == "joint"), \
        "no torso/adjacent nodes → no joints"
    dk = {c.target for c in bcaps if c.kind == "derived"}
    assert "activity_asymmetry_upper_arm" in dk, dk
    print("[selftest] two upper-arm nodes → activity_asymmetry_upper_arm "
          "(no joints needed) — OK")

    # Landmark labels: a joint-named landmark is flagged ambiguous; a landmark
    # that disagrees with its segment is flagged as a likely mislabel.
    lm = {"schema_version": SCHEMA_VERSION, "subject": {"id": "S"},
          "session": {"id": "lm"}, "calibration": {"captured": False},
          "nodes": [
              {"node_id": "A", "column": "n0", "segment": "forearm_l",
               "landmark": "left wrist"},              # joint → ambiguous
              {"node_id": "B", "column": "n1", "segment": "upper_arm_r",
               "landmark": "right forearm"}]}          # base mismatch
    w = montage_warnings(lm)
    assert any("names a JOINT" in x for x in w), w
    assert any("suggests segment 'forearm'" in x for x in w), w
    # The example montage's clean labels raise no warnings.
    assert not montage_warnings(example_montage()), montage_warnings(
        example_montage())
    print("[selftest] landmark labels: joint→ambiguous & mismatch flagged, "
          "clean labels silent — OK")

    print("\n[selftest] all checks passed.")


def main() -> None:
    ap = argparse.ArgumentParser(
        description="Resolve which joints/metrics a node montage can compute.")
    ap.add_argument("montage", nargs="?", help="montage JSON file")
    ap.add_argument("--example", action="store_true",
                    help="print a fillable example montage and exit")
    ap.add_argument("--json", action="store_true",
                    help="emit the capability report as JSON")
    ap.add_argument("--selftest", action="store_true",
                    help="validate the model + resolver logic (no montage)")
    args = ap.parse_args()

    if args.selftest:
        selftest()
        return
    if args.example:
        print(json.dumps(example_montage(), indent=2))
        return
    if not args.montage:
        ap.error("give a montage file, or use --example / --selftest")

    with open(args.montage) as f:
        montage = json.load(f)

    errors = validate_montage(montage)
    if errors:
        print("montage INVALID:", file=sys.stderr)
        for e in errors:
            print(f"  - {e}", file=sys.stderr)
        raise SystemExit(2)

    caps = resolve(montage)
    if args.json:
        print(json.dumps(to_dict(caps, montage), indent=2))
    else:
        print_report(montage, caps)


if __name__ == "__main__":
    main()
