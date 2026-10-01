# OpenSense feasibility & partial-montage evaluation

> Follow-up: the OpenSense path is now `tools/opensense_ik.py` (with the model
> fixes below), and the head-to-head accuracy assessment against the default
> chain is in [`SOLVER_COMPARISON.md`](SOLVER_COMPARISON.md).

Question: can OpenSim/OpenSense (Stanford, Apache-2.0) replace our own
calibration + joint-angle stage, and how does it behave when only some nodes
are worn? Harness: `tools/opensense_feasibility.py` (synthetic ground truth, no
hardware). Run 2026-09-25 with OpenSim 4.6.

## Setup

- **Install:** `pip install opensim` — official Stanford wheels for Windows /
  macOS / Linux, Python 3.11–3.13. No conda needed.
- **Model:** `Rajagopal2015_opensense.osim` from `opensim-org/opensim-models`
  (the OpenSense tutorial model). It has a trunk (torso), both humeri,
  ulna/radius (elbow + pronation/supination) and hands (wrist flex/deviation).
  Legs, lumbar and pelvis translation were locked; pelvis rotations stand in
  for trunk motion.
- **Synthetic session:** 20 s at 60 Hz with a 1 s neutral hold. Every arm DOF
  moves at once: shoulder flexion to 140°, abduction to 70°, humeral rotation
  ±35°, elbow 5–125°, pro/sup 7–83°, wrist ±45° / −14–22°. The trunk tilts,
  lists and twists by 8–18°.
- **Virtual sensors:**
  - limb nodes strapped at random orientations; the torso node's +z points
    forward (5° tilt)
  - sensor world is Z-up, and the subject faces 37° off magnetic north
  - 1° orientation noise per sample
- **Calibration:** the Markley average of the neutral hold, fed to IMUPlacer.
  IK runs over the whole record. Error = IK coordinate − truth, reported as
  RMS / max in degrees after the neutral hold.

## Results (wide shoulder ranges; RMS / max °)

"—" = coordinate not measurable with that montage (no sensor on one side of the
joint), so it was locked and is not reported.

| Montage (nodes) | Trunk | Shoulder flex / add / rot | Elbow | Pro/sup | Wrist flex / dev |
|---|---|---|---|---|---|
| **A** torso + UA + FA + hand (R) | 0.7 / 0.8 / 0.6 | 4.2 / 2.9 / 5.0 (max 19) | 0.8 | 3.2 | 1.4 / 2.1 |
| **G** all 7 nodes (bilateral) | same as A | R same as A; L 2.5 / 3.0 | 0.8 (L 0.9) | 3.2 (L 2.7) | 1.4 / 2.1 (L 1.2 / 1.9) |
| **B** torso + UA | 0.7 / 0.8 / 0.6 | 4.1 / 2.7 / 5.2 | — | — | — |
| **C** torso + FA (no UA) | ok | **41 / 22 / 28** | **36** | **19** | — |
| **D** UA + FA, no torso, facing known | — (assumed upright) | 16 / 10 / 17 | 1.0 | 1.1 | — |
| **D2** UA + FA, no torso, facing unknown | — | 18 / 35 / 32 | 5.9 | **25** | — |
| **E** FA + hand, shoulder & elbow locked | — | — | wrong (63) | wrong (32) | **28 / 23** |
| **E2** FA + hand, upper chain left free | — | meaningless | meaningless | meaningless | 0.8 / 0.8 |
| **F** both UAs, no torso (facing known) | — | R 16 / 10 / 17, L 12 / 11 | — | — | — |

IK speed: about 45–75 s per 1200 frames (~40–65 ms per frame) on one core,
which is fine for offline processing.

## Findings

1. **The full set works well.** With the heading handled (point 3) and wide
   shoulder ranges (point 4), elbow and wrist come out at ~1–2° RMS,
   pronation/supination at ~3°, and the trunk at <1°.
2. **The shoulder is the weak joint, because of how the model defines it.**
   Rajagopal's shoulder is a flexion → adduction → rotation Euler sequence
   directly between torso and humerus (no scapula).
   - The 19° spikes all fall in high-abduction frames (abduction ~60–70°).
     There, flexion and rotation errors are equal and opposite: the model is
     near gimbal lock, and one rotation gets split between two nearly
     aligned axes.
   - The humerus orientation relative to the torso is still accurate: 4.5° RMS,
     8.3° max.
   - These are not the ISB Y-X-Y shoulder angles we report today. Switching to
     OpenSense means either accepting the model's shoulder definition or
     re-decomposing the humerus-vs-torso rotation ourselves.
3. **OpenSense's heading correction doesn't reach IK.** `IMUPlacer`'s
   `base_imu_label` / `base_heading_axis` shape only the sensor placement; the
   IK input is not de-rotated.
   - Symptom: a constant 40° trunk-rotation error, plus 4–5° of trunk
     tilt/list crosstalk.
   - Fix: rotate all quaternions about vertical by the torso node's heading at
     neutral before writing the .sto. This is what
     `calibrate_segments.compute_heading` already provides.
4. **The shipped joint ranges must be widened for rehab.** Out of the box,
   shoulder flexion is clamped to ±90° (arms are also locked). With those
   limits, a 140° reach fails silently: flexion error is 50° max, and IK bends
   the trunk (up to 25°) and the elbow (up to 25°) to compensate. That corrupts
   joints that were measured correctly. The harness widens flexion to −90…180°
   and adduction to −180…90°. Clamping will also hide real hypermobility, so
   pick patient-appropriate limits.
5. **Partial montages: OpenSense does not recover what isn't measured.**
   A joint angle is only valid when **both** of its segments carry a node.
   This is the same rule our `motion_capabilities.JOINTS` already encodes.
   - **Missing middle segment (C):** the forearm orientation (3 constraints)
     is shared across shoulder + elbow + pro/sup (5 DOF), so the solution is
     underdetermined. It returns a plausible-looking but wrong pose (elbow
     36° RMS). A torso + forearm montage cannot give elbow or shoulder angles.
   - **No torso node (D, F):** elbow and pro/sup stay good (~1°) *if the
     facing is known*. Shoulder angles become humerus-vs-assumed-upright-trunk,
     so any real trunk sway goes into them (16° RMS here with 8–18° sway).
     Report them as "arm vs vertical", not as shoulder joint angles.
   - **No torso and unknown facing (D2):** the wrong heading gets baked into
     each sensor's calibration offset, and errors spread to every joint,
     including pro/sup (25° RMS). Without a torso node the facing must come
     from elsewhere, e.g. our `--facing-deg`.
   - **Distal-only montage (forearm + hand, E/E2):** the relative joint is
     only right if the chain *above* the proximal node is left free to absorb
     its orientation (E2: wrist 0.8°). Lock it and IK distorts the wrist to
     compromise (E: 28°). Coordinates above the proximal node come out as
     arbitrary numbers and must be masked, not reported.
   - **Coordinates with no node on either side** are unconstrained. IK leaves
     them wherever it likes, so lock them or drop them from the output.

## What adopting OpenSense would take

- **Export:** the aligned stream → a `DataType=Quaternion` .sto with
  `<body>_imu` columns (w,x,y,z), heading-corrected and at the Z-up →
  OpenSim rotation (`sensor_to_opensim_rotations = (−90°, 0, 0)`). Plus a
  one-row calibration .sto from the neutral window.
- **Montage → model mapping:**
  - torso → `torso`
  - upper_arm_* → `humerus_*`
  - forearm_* → `radius_*` (it carries pro/sup)
  - hand_* → `hand_*`
- **A prepared model:** legs locked, shoulder ranges widened, and per-session
  locking of unmeasured coordinates.
- **Output handling:** a mask per montage so only measurable joints are
  reported. Our metrics (ROM, velocity, reps, SPARC) then run on IK
  coordinates instead of our Euler angles. Shoulder definition: see point 2.
- **Validation:** OpenSense's published validation (Al Borno et al., 2022)
  is lower-body; the upper-limb accuracy of *this device* still needs a check
  against optical motion capture or goniometry.
