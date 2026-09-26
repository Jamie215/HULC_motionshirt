# Metrics — what each number is and how it is computed

The per-metric reference for **stage 6** (`metrics.py`). Every value the tool emits
into `metrics.json` is listed below with its plain-language meaning, the exact
method that produces it, its inputs and units, whether it needs calibration, and
the field it lands in. The computation lives in `metrics.py`; this page is the
readable index into it, the same way `MONTAGE_SCHEMA.md` indexes the montage +
resolver.

- **Which** metrics appear for a given placement is decided by the resolver
  (`motion_capabilities.py`, described in `MONTAGE_SCHEMA.md` §5) — you only ever
  get what the montage supports.
- **How** each one is computed is documented here (and in the `metrics.py`
  docstring for the named function).

See `pipeline_walkthrough.html` for the whole pipeline at a glance, and
`SETUP_AND_CALIBRATION_PLAN.md` for the stage-5/6/7 design.

---

## 1. The honesty contract (read this first)

Three rules gate every number, so a soft value never masquerades as a hard one:

- **Blocked ≠ fabricated.** A joint whose montage lacks one of its two adjacent
  nodes is reported in `blocked_joints` with the missing node named — never a
  guessed angle.
- **`clinical: false` ⇒ relative-only.** A joint (or segment) whose node(s) are
  not anatomically calibrated — or, for a joint, whose anatomical axes are unknown
  because no confident facing was recovered (top-level `anatomical_axes: false`) —
  still produces numbers, but its zero is "the pose at the neutral window," not
  the anatomical landmark, and without anatomical axes the DOF labels need not
  match the movement. It is flagged, and in the
  stage-7 viewer its rows dim in raw view. Calibration is what turns an orientation
  into an *anatomical* angle (`MONTAGE_SCHEMA.md` §4).
- **Unwrap before range.** Angle series are unwrapped (`np.unwrap`) before ROM, so
  a real sweep through ±180° reports its true excursion instead of a fake 360°;
  the unwrapped series is then re-centred by whole turns so its median sits in
  ±180° (an unwrap never shifts the zero).
- **Implausible ⇒ flagged.** Every DOF carries a `plausibility` check (§3): a
  range over a full turn, sample-to-sample jumps over `JUMP_MAX_DEG`, or — with
  anatomical axes — more than 2% of samples outside the DOF's physiological
  limits. A failing joint gets a `plausibility_warning` naming the likely causes
  (calibration pose, facing, sensor slip) rather than silently reporting it.
- **Only the protocol is analysed.** By default the stream is trimmed to the
  *analysis window*: from the opening neutral hold to the closing hold
  (`trim_to_analysis`, both located by stage 5). Strapping on and the warm-up
  before, and carrying the nodes to the charger after, never enter ROM, reps or
  activity. Each end is cut only when its hold is this recording's own still
  window (a calibration reused from another take cuts nothing); a missing
  closing hold leaves the tail in and prints a warning. `--keep-pre-neutral`
  analyses the whole stream.

Session-level trust inputs from reconcile — `sync_confidence`, dropout — are **not**
recomputed here; they travel alongside these numbers and should be surfaced with
them (`gates_note` in the output says so).

---

## 2. The shared chain: quaternion → joint angle

Every joint metric starts from the same four steps (`metrics.py` module docstring,
`compute_joint_metrics`):

```
q_seg(t) = q_WS(t) ⊗ q_SB                     apply the cached mounting offset (stage 5)
q_rel(t) = conj(q_seg_prox) ⊗ q_seg_dist      distal relative to proximal
q_anat   = conj(q_WA) ⊗ q_rel ⊗ q_WA          re-express in anatomical axes
(α,β,γ)  = euler(q_anat, sequence)            decompose in the joint's ISB/Wu sequence
angle_dof = (α|β|γ)[seq_index]                pick the slot that IS this clinical DOF
```

- `q_SB` is the sensor→bone mounting offset from `calibration.json`; **identity when
  uncalibrated** (⇒ `clinical: false`).
- `q_WA` is the **anatomical frame at neutral** from `calibration.json`'s
  `anatomical_frame` block: X = anterior, Y = superior (along a hanging limb),
  Z = X × Y = the subject's right. It is built from gravity plus the subject's
  facing — the torso node's heading, else the elbow hinge axis (the facing that
  makes elbow motion a pure hinge, `estimate_facing_from_elbow`), else
  `--facing-deg`. The
  mounting offset alone zeroes each segment but leaves its axes on the world
  compass (Z = up), while the ISB sequences assume anatomical axes — without
  `q_WA` a pure elbow flexion lands in whichever slot the facing puts it. No
  confident facing ⇒ no `q_WA` ⇒ the joint is `clinical: false`.
- **Wrist frame.** The N-pose holds the palms facing the thighs, i.e. the hand
  is turned `NEUTRAL_FOREARM_DEG` (90°) about the long axis from the
  palms-forward anatomical position the wrist's `ZXY` split assumes. Wrist
  joints are therefore split in the hand's own neutral frame
  (`joint_frame_deg`/`joint_frame_quat`: `q_anat` is additionally rotated by
  that angle about Y), so wrist flexion reads as flexion and not as
  radial/ulnar deviation. Elbow and shoulder are unaffected.
- One frame serves both sides, so for a **left** joint `q_anat` is mirrored
  through the sagittal plane (`mirror_left`: `[w,x,y,z] → [w,−x,−y,z]`) before
  the split. A left joint then decomposes exactly like a right one and every DOF
  has the same clinical sign on both sides:

  | Joint | DOF | Positive = |
  |---|---|---|
  | elbow | `flex_ext` | flexion |
  | elbow | `pro_sup` | pronation |
  | wrist | `flex_ext` | flexion |
  | wrist | `rad_uln` | ulnar deviation |
  | shoulder | `plane_elev` | 0° abduction plane, +90° forward flexion, −90° extension |
  | shoulder | `elevation` | raised (always ≥ 0) |
  | shoulder | `axial_rot` | internal rotation |
- The Euler `sequence` and the `seq_index` each clinical DOF occupies both come from
  `motion_capabilities.JOINTS` — the one body model. This tool never re-declares
  anatomy or invents a convention.
- Because calibration zeroes `q_rel` at the neutral pose, a calibrated angle is
  measured from anatomical zero.

**Singularity guard.** Every Euler split has a pole where the outer two axes align
and their angles become ill-defined (a *proper* sequence like `YXY` at the middle
angle ≈ 0/π — the shoulder's "arm at the side" pole; a *Tait-Bryan* sequence like
`ZXY` at middle ≈ ±90°). Within `SINGULARITY_GUARD_DEG` (10°) of that value the
outer-slot DOFs are marked undefined for those samples, so a joint resting at the
pole never emits a spurious 180° swing or an infinite velocity. The shoulder is
the exception for axial rotation: it is reported as the *sum* of the outer
angles, which is exactly what stays defined at the side, so only
`plane_elev` is masked there (both are masked overhead, where the sum is the
ill-defined part). Affected DOFs carry
`defined_frac` (share of samples that were well-conditioned) and a
`singularity_note`, and their series carry `NaN` at the masked samples so every
downstream consumer skips them the same way.

---

## 3. Joint-tier metrics

Emitted per computable joint, into `joints[]`. Each joint object has `key`, `name`,
`clinical`, `decomposition` (the sequence), `dofs[]`, and `reps`.

| Metric | Field | Method | Units | Needs calibration? |
|---|---|---|---|---|
| **Per-DOF angle** | `dofs[].` (series, feeds the rest) | Euler decomposition of `q_rel` in the joint's sequence, unwrapped in radians before converting (§2). | ° | For *anatomical* zero, yes; relative shape works without |
| **Range of motion** | `dofs[].rom` = `{min_deg,max_deg,range_deg,median_deg}` | `range = max − min` over the unwrapped angle (`_rom`/`_stats`). `null` if the DOF is singular for the whole session. | ° | Gates *clinical* ROM; relative-only when `clinical:false` |
| **Angular velocity** | `dofs[].velocity` = `{peak_deg_s,mean_abs_deg_s,rms_deg_s}` | `v = d(angle)/dt` via `np.gradient` (`velocity_stats`); peak = max\|v\|, mean = mean\|v\|, RMS = √mean(v²). Differentiated **only within** contiguous valid runs, so a masked singular gap never fakes a huge peak. | °/s | No (shape metric) |
| **Repetitions** | `reps` = `{count,primary_dof,bouts[]}` | Zig-zag turning points on the joint's primary DOF (`find_rep_bouts`): a rep is one out-and-back swing of at least `rep_threshold` = max(`REP_MIN_AMPLITUDE_DEG` (15°), 25% of the session's 2–98th-percentile range), each swing measured from its own local extremes so a set of small curls still counts beside a big movement elsewhere. Reps are grouped into **bouts**: a dwell at an extreme longer than `REP_PAUSE_S` (5 s) or a swing slower than `REP_MAX_LEG_S` (8 s) ends one — so curls, then rotations, then curls report as separate phases. Each bout = `{t_start_ms,t_end_ms,reps,mean_amplitude_deg}`; `count` is their sum. | count | No — only shape matters |
| **Plausibility** | `dofs[].plausibility` = `{ok,issues[],jumps,outside_limits_frac?,limits_deg?}` | `check_plausibility`: range > 360°, jumps > `JUMP_MAX_DEG` (120°) between samples, and (anatomical axes only) > `PLAUSIBLE_OUTSIDE_FRAC` (2%) of samples outside the DOF's `plausible_deg` from `motion_capabilities`. Any failing DOF adds a joint-level `plausibility_warning`. | — | Limits check needs anatomical axes |

A `clinical:false` joint also carries a `warning` string spelling out that its
angles are relative-only.

**Primary DOF.** Reps, L/R symmetry and coordination use the joint's declared
primary DOF (`Joint.primary`; shoulder = `elevation`, elbow and wrist =
`flex_ext`), falling back to the DOF
that swung the most when none is declared or it is singular all session.

**Shoulder DOFs** follow the ISB `YXY` names rather than flexion/abduction:

| Key | Name | Meaning |
|---|---|---|
| `plane_elev` | Plane of elevation | *Direction* the arm is raised in: 0° = abduction (frontal plane), +90° = forward flexion, −90° = extension. Raw YXY slot 0 + 180° — ISB's negative-elevation branch of the same rotation, so elevation can be reported positive. Undefined with the arm at the side or overhead. |
| `elevation` | Elevation | *How far* the arm is raised, in whatever plane — a forward and a sideways 60° raise both read 60°. Primary DOF. |
| `axial_rot` | Axial rotation (int / ext) | Twist about the humerus, internal positive: raw slot 0 + slot 2. ISB's own third angle trades with the plane (20° of internal rotation reads +20° in abduction but −70° in forward flexion); the sum reads +20° in both, and it stays defined with the arm at the side. Undefined only overhead. |

---

## 4. Segment-tier metrics

Emitted per placed node, into `segments[]` (`segment`, `node_id`, `calibrated`, …).
Speed and travel use the **raw** quaternion (mounting-invariant), so they are honest
with or without calibration; elevation and posture-dwell use the **calibrated**
frame.

| Metric | Field | Method | Units | Needs calibration? |
|---|---|---|---|---|
| **Angular speed** | `angular_speed` = `{mean_deg_s,peak_deg_s}` | Geodesic step between consecutive orientations: `Δ = 2·arccos(\|q_t · q_{t+1}\|)`, divided by `dt` (`segment_angular_speed`). Measures how much the bone turned, not where it points — so it is mounting-invariant. Also feeds SPARC. | °/s | No |
| **Angular travel** | `travel` = `{travel_deg, active_time_frac}` | `travel = ∫\|angular speed\| dt` (cumulative rotation, direction-agnostic). `active_time_frac` = share of time above `ACTIVE_SPEED_DEG_S` (20°/s). Session aggregates — valid even when two limbs aren't time-aligned (`segment_travel_deg`). | °, fraction | No |
| **Elevation** | `elevation` = `{min,max,range,median,mean}_deg` | Tilt of the bone's long axis from vertical: `arccos(R[2,2])` of the (calibrated) segment frame per sample (`segment_elevation_series`). Uncalibrated it is the board's own tilt — a real inclination, just not anchored to the bone (flagged by `calibrated`). | ° | For anatomical meaning, yes |
| **Smoothness (SPARC)** | `smoothness_sparc` | Spectral arc length of the speed profile (Balasubramanian et al. 2015, `sparc`): normalize the magnitude spectrum by DC, take the arc length of the normalized spectrum up to the cutoff band (≤ `SPARC_FC_MAX_HZ` and above `SPARC_AMP_THRESH`). Smoother → nearer 0; jerkier → more negative (≈ −1.5 smooth … −5+ jerky). `null` when there is essentially no movement. | unitless (≤ 0) | No |
| **Posture dwell** | `posture_dwell` = `{edges_deg, fraction}` | Time-in-posture histogram: fraction of the session spent in each elevation band (`POSTURE_BIN_EDGES_DEG` = 0/30/60/90/120/150/180°), **time-weighted** by inter-sample dt so a non-uniform grid stays correct (`posture_dwell`). | fraction per band | **Yes** — emitted only when the segment is calibrated |

---

## 5. Derived-tier metrics

Cross-joint / cross-segment metrics, emitted into `derived[]` **only when the
resolver unlocks them** for the montage (`compute_derived`). Each entry has
`target`, `name`, `requires[]`, and a `metrics{}` block; most carry a `clinical`
flag and/or a `note`.

| Metric | `target` | `metrics{}` | Method |
|---|---|---|---|
| **L/R ROM symmetry** | `symmetry_<joint>` | `symmetry_index`, `rom_ratio`, `left_rom_deg`, `right_rom_deg`, `dof` | On each side's primary DOF: `symmetry_index = 100·\|L−R\| / (½(\|L\|+\|R\|))` (0 = identical, → 200 opposite); `rom_ratio = min/max`. `clinical` = both sides calibrated. |
| **Bilateral activity asymmetry** | `activity_asymmetry_<segment>` | `asymmetry_index`, `use_ratio`, `active_time_ratio`, `left_travel_deg`, `right_travel_deg` | Signed laterality from segment **travel**: `asymmetry_index = 100·(R−L)/(R+L)` in [−100, +100] (+ = right used more). A session aggregate, so **valid even at low sync confidence** (stated in its `note`) — the sparse-montage workhorse. |
| **Inter-joint coordination** | `coordination_<a>_<b>` | `pair`, `peak_r`, `lag_s` | Peak normalized cross-correlation of the two joints' primary-DOF series and its lag (`cross_correlation`): both series mean-removed and unit-normalized so `peak_r ∈ [−1,1]`; positive `lag_s` = the second joint follows the first. |
| **Trunk compensation** | `compensation_<...>` | `trunk_travel_deg`, `trunk_elevation_range_deg` | Trunk excursion during the task, from the torso segment's travel + elevation range. `clinical` = torso calibrated; when false, `note` flags the excursion as relative. |

---

## 6. Constants (the tuning knobs)

All defined at the top of `metrics.py`:

| Constant | Value | What it gates |
|---|---|---|
| `SINGULARITY_GUARD_DEG` | 10° | Band around each Euler pole where outer-slot DOFs are marked undefined |
| `ACTIVE_SPEED_DEG_S` | 20°/s | Threshold for "active" (active-time fraction, activity comparison) |
| `REP_MIN_AMPLITUDE_DEG` | 15° | Floor of the rep threshold (the threshold is the larger of this and 25% of the range) |
| `REP_PAUSE_S` | 5 s | A dwell at an extreme longer than this ends a rep bout |
| `REP_MAX_LEG_S` | 8 s | A single swing slower than this ends a rep bout |
| `NEUTRAL_FOREARM_DEG` | 90° | Hand rotation from palms-forward at the N-pose (wrist frame) |
| `JUMP_MAX_DEG` | 120° | Sample-to-sample step flagged as implausible |
| `PLAUSIBLE_OUTSIDE_FRAC` | 0.02 | Share of samples outside physiological limits that fails plausibility |
| `POSTURE_BIN_EDGES_DEG` | 0/30/60/90/120/150/180° | Elevation bands for the posture-dwell histogram |
| `SPARC_FC_MAX_HZ` | 10 Hz | Upper frequency ignored by SPARC |
| `SPARC_AMP_THRESH` | 0.05 | Normalized-magnitude floor that sets the SPARC cutoff band |
| `_GIMBAL_SIN` | 1e-6 | \|sin(middle)\| below which the Euler split is collapsed onto the well-defined sum |

---

## 7. Output shape (`metrics.json`)

```jsonc
{
  "schema_version": "1.0",
  "subject": { ... }, "session": { ... },
  "calibration_used": true, "anatomical_axes": true,
  "sample_rate_hz": 50.0, "n_samples": 4000, "duration_s": 79.98,
  "analysis_window_ms": [t0, t1],            // opening hold → closing hold
  "trimmed_before_neutral_s": 12.4 | null,   // setup dropped before the hold
  "trimmed_after_closing_s": 8.1,            // tail dropped after the closing hold
  "closing_hold_found": true,
  "joints": [ { "key","name","clinical","decomposition",
                "dofs": [ { "key","name","plane",
                            "rom": {min_deg,max_deg,range_deg,median_deg} | null,
                            "velocity": {peak_deg_s,mean_abs_deg_s,rms_deg_s} | null,
                            "plausibility": {ok,issues,jumps,...},
                            "defined_frac"?, "singularity_note"? } ],
                "reps": {count,primary_dof,
                         bouts: [ {t_start_ms,t_end_ms,reps,mean_amplitude_deg} ]},
                "warning"?, "plausibility_warning"? } ],
  "blocked_joints": [ { "key","name","missing": [ ... ] } ],
  "segments": [ { "segment","node_id","calibrated",
                  "angular_speed": {mean_deg_s,peak_deg_s},
                  "travel": {travel_deg,active_time_frac},
                  "elevation": {min_deg,max_deg,range_deg,median_deg,mean_deg},
                  "smoothness_sparc": <float|null>,
                  "posture_dwell"?: {edges_deg,fraction} } ],
  "derived": [ { "target","name","requires": [...],
                 "metrics": { ... }, "clinical"?, "note"? } ],
  "gates_note": "clinical=false ⇒ relative-only ..."
}
```

---

## 8. Where the code is, and how to check it

- Computation: `tools/metrics.py` — the function named in each row above.
- Catalog / gating (which metrics unlock): `tools/motion_capabilities.py`,
  `MONTAGE_SCHEMA.md` §5.
- Body model (segments, joints, DOFs, Euler sequences): `motion_capabilities.JOINTS`.
- Validation with no hardware:

  ```
  python tools/metrics.py selftest
  ```

  round-trips every Euler sequence, recovers a known injected flexion sweep (and
  a forearm twist) at several facings, checks the calibration and anatomical-axis
  gates and wrap handling, the wrist's hand-frame signs, the analysis-window trim
  (and its guard against a reused calibration), plausibility flags, rep bouts,
  the SPARC/cross-correlation primitives and the full derived tier.
