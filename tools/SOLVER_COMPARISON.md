# Pose solver comparison: default chain vs OpenSense (two models)

Which path should produce the reported joint angles?

- **Default chain:** `calibrate_segments.py` → `metrics.py`.
- **OpenSim OpenSense** (`opensense_ik.py`) on one of two published models:
  - **Thoracoscapular Shoulder Model** (TSM): Seth et al. 2019; right arm; the
    upper-limb model.
  - **Rajagopal 2016**: full body; built for gait.

Assessed 2026-09-25 with OpenSim 4.6, after the pipeline fixes in the same
series of changes: auto neutral window, facing from the elbow hinge, wrist in
its palms-in frame, plausibility flags.

## Verdict

**For a reference-able method, use OpenSense with the Thoracoscapular Shoulder
Model** for right-arm sessions (`--opensense-model ThoracoscapularShoulderModel.osim`):

- It is as accurate as the default chain in every tested condition (within
  about 1°), with and without a torso node.
- Its raw coordinates agree with the reported angles.
- It fits the real capture best (median residual 4.4°) and solves in about 10 s.
- It lets the method be cited as published model + published IK method +
  ISB angle conventions (see *Methods text* below), instead of "our own chain".

Keep the **default chain** as the dependency-free path and as a built-in
cross-check. Use **Rajagopal** only where TSM has no body: the left arm and the
wrists. Report its raw coordinates only with a torso node (see finding 3).

## 1. Synthetic ground truth (`tools/compare_paths.py`)

**Truth.** A protocol-shaped 40 s session is played on the Rajagopal model:

- neutral hold (palms facing the thighs)
- shoulder swing
- 4 supinated curls
- pro/supination at 65° abduction
- full swing with humeral rotation
- rest, with trunk sway throughout

**Nodes.** Virtual nodes have random strap mountings, and the subject faces
37° off world +Y. The TSM runs solve on a *different* model from the one that
made the truth, so its joints don't match the "subject's", as with a real arm.

**Scoring.** Both paths get the same node streams and calibration step, and
all angles go through the same `metrics.py` ISB definitions.

**Conditions:**
- **clean:** 60 Hz, ~1° RMS slow sensor error
- **realistic:** 8.3 Hz (today's firmware), ~3° RMS slow error per strap
  (sensor + soft tissue), 50 ms clock offset on distal nodes

RMS error, degrees. Each solver column sits next to its own run's
default-chain column; the random draws differ slightly between runs.

**Full right arm (torso + upper arm + forearm; + hand for Rajagopal):**

| DOF | clean: default | clean: Rajagopal | clean: default | clean: TSM | realistic: default | realistic: Rajagopal | realistic: default | realistic: TSM |
|---|---|---|---|---|---|---|---|---|
| shoulder elevation | 2.7 | 2.1 | 2.7 | **2.0** | 6.0 | 5.6 | 6.2 | **5.6** |
| shoulder axial rot. | 0.8 | 0.8 | 0.8 | 0.8 | 4.7 | 4.7 | 4.6 | 4.6 |
| elbow flexion | 0.4 | 0.3 | 0.4 | 0.3 | 2.4 | 2.4 | 2.3 | 2.3 |
| elbow pro/sup | 1.5 | 1.0 | 1.5 | 1.5 | 2.1 | 3.1 | 2.0 | 2.0 |
| wrist flex / dev | 0.6 / 0.5 | 1.0 / 0.4 | — | — | 2.1 / 5.1 | 2.3 / 5.3 | — | — |

**Torso + upper arm (the planned 2-node montage):**

| DOF | clean: default | clean: Rajagopal | clean: TSM | realistic: default | realistic: Rajagopal | realistic: TSM |
|---|---|---|---|---|---|---|
| plane of elevation | — | — | — | 10.7 | 10.8 | **9.8** |
| elevation | 0.9 | 0.9 | 1.0 | 5.8 | 5.8 | 6.0 |
| axial rotation | 0.5 | 0.5 | 0.5 | 4.6 | 4.6 | 5.2 |

(Plane of elevation is undefined with the arm near the side; the clean run's
motion kept it there too often to score.)

**Upper arm + forearm (no torso; today's capture):**

| elbow DOF | clean: default | clean: Rajagopal | clean: TSM | realistic: default | realistic: Rajagopal | realistic: TSM |
|---|---|---|---|---|---|---|
| flexion | 1.2 | 1.1 | 1.1 | 4.2 | 4.7 | 4.3 |
| pro/sup | 8.3 | 8.1 | 8.2 | 9.4 | 9.2 | 9.2 |

The ~8° pro/sup floor in that table is common to all three. Truth is split in
the trunk's frame, but the elbow's hinge plane is 13° off it, so part of the
flexion leaks into the "true" pro/sup. Scored against the Rajagopal model's
own coordinates instead, the default chain's pro/sup error is 1.9° (clean)
and 4.5° (realistic).

### Findings
1. **The solvers are equivalent on reported angles.** With every path's pose
   read out through the same ISB definitions, OpenSense and the default chain
   differ by 0.1–1.3° RMS frame-by-frame on the no-torso sessions, and by
   about 1° or less in the tables above otherwise.
   Errors roughly double from clean to realistic, and that comes from the
   input (8 Hz, strap wobble, clock offset), not the solver.
2. **Joint constraints help the shoulder slightly.** Elevation is 0.4–0.7°
   better with either model when the elbow chain is present, because the
   model's joints pull the humerus into a consistent pose.
3. **The raw model coordinates differ, and that decides which model is
   citable.** The raw coordinates are what OpenSense users usually report.
   RMS difference from the reported angles:

   | | full arm + torso | no torso (synthetic) | real capture (no torso) |
   |---|---|---|---|
   | TSM `pro_sup` | 0.2° | 0.6–0.7° | 1.7° |
   | TSM `elbow_flexion` | 0.3° | 3.8° | 4.3° |
   | Rajagopal `pro_sup_r` | 1.9° | 6.3–6.4° | 6.2° |
   | Rajagopal `elbow_flex_r` | 0.3° | 0.7–1.3° | 5.2° |

   - Without a torso node, the facing comes from the elbow hinge. That hinge
     plane is 13° from the Rajagopal model's trunk axis, so Rajagopal's
     humerus is placed 13° rotated and its pro/sup coordinate absorbs the
     difference.
   - TSM's forearm coordinate is unaffected.
   - TSM's `elbow_flexion` differs by ~4% of the flexion range without a
     torso node: its oblique elbow axis defines flexion slightly differently
     from the hinge frame.
4. **TSM needed three preparation fixes to be usable.** All are listed in
   `opensense_ik.py`:
   - the default pose set to the N-pose (the published default has the
     trunk tilted and the arm abducted)
   - clamping on for the elbow and forearm, which ship unclamped (a solve
     wandered to −368°)
   - the glenohumeral angles left unclamped: they must wrap and have poles
     at 0°/180°, where a clamped solve stuck with the arm overhead
5. **Facing recovery:** from the torso, within 0.8–3.7° (clean) and 1.4–4.1°
   (realistic).

## 2. Real capture (upper arm + forearm, 2026-09-25)

No ground truth, so this measures agreement and fit, not accuracy.

| | default | Rajagopal | TSM |
|---|---|---|---|
| neutral window | auto 36.7–38.7 s (placeholder rejected) | same | same |
| facing | elbow hinge −45° | same | same |
| elbow flexion ROM | −21…145° | −18…145° | −12…145° |
| pro/sup ROM | −80…100° | −83…92° | −82…99° |
| vs default, frame by frame | — | 5.4° flex, 2.3° pro/sup | **1.8° flex, 1.4° pro/sup** |
| fit residual, median / p95 | — | 5.6° / 20° | **4.4° / 16°** |
| run time | < 1 s | ~35 s | ~10 s |

The residual marks frames where the nodes disagree with any rigid skeleton.
On this capture those are the fast movements, where 8 Hz sampling and
inter-node timing (sync confidence 0.52) dominate.

## 3. Methods text (draft)

> Segment orientations were measured with inertial nodes (BNO086, 9-axis
> fusion) strapped to the thorax, upper arm and forearm. Sensor-to-segment
> alignment was obtained from a static neutral pose (arms at the sides,
> palms facing the thighs), detected automatically as the first still window
> of each recording. Joint kinematics were estimated with OpenSim 4.6
> OpenSense [Delp 2007; Seth 2018; Al Borno 2022]: IMUPlacer registered the
> nodes to the Thoracoscapular Shoulder Model [Seth 2019; Seth 2016] in the
> neutral pose, and IMU inverse kinematics solved the pose at every sample.
> The model's default pose was set to the neutral pose. Sternoclavicular and
> scapulothoracic coordinates were held at the model's resting posture, as
> the scapula was not instrumented. Ranges were widened to elbow flexion
> −15–160° and pro/supination ±100°. Shoulder, elbow and forearm angles were
> expressed from the solved segment orientations following the ISB
> recommendations [Wu 2005].

References (verify volume/pages before use):
- Delp SL et al. 2007, *IEEE Trans Biomed Eng* 54(11)
- Seth A et al. 2018, *PLoS Comput Biol* 14(7)
- Al Borno M et al. 2022, *J NeuroEng Rehabil* 19:22
- Seth A, Dong M, Matias R, Delp SL 2019, *Front Neurorobot* 13:90
- Seth A et al. 2016, *PLoS ONE* 11(1)
- Wu G et al. 2005, *J Biomech* 38(5)

Limits to state with it:
- **Scapula:** held fixed, so "glenohumeral" motion here is humerothoracic.
- **Validation:** OpenSense's published validation is lower-limb gait. This
  device's accuracy still needs its own check against optical motion capture
  or a goniometer.
- **Coverage:** right arm only (TSM). The left arm needs Rajagopal or a
  mirrored model.

## 4. Recommendations

1. **Collect torso + upper arm** for shoulder work with the two nodes.
   Every path gets the shoulder right there (about 6° RMS realistic, 1° clean).
   Use the upper arm + forearm pair for elbow and forearm sessions.
2. **Report** ISB angles from the TSM-solved pose. The raw TSM coordinates
   agree for pro/sup; use the ISB flexion.
3. **The biggest accuracy lever is still the input, not the solver:** the
   realistic condition roughly doubles every error.
   - Raise the log rate from 10 Hz toward 30–60 Hz.
   - Keep full rate through the neutral hold.
   - Tighten straps.
4. **Both arms at full scale:** TSM is right-arm only. The published model
   could be mirrored for the left arm, which is a modification to report, or
   Rajagopal used for the left, with the raw-coordinate caveat in finding 3.

## Reproduce

```bash
pip install opensim
git clone --depth 1 --filter=blob:none --sparse https://github.com/opensim-org/opensim-core
(cd opensim-core && git sparse-checkout set --no-cone OpenSim/Tests/shared/ThoracoscapularShoulderModel.osim)
git clone --depth 1 --filter=blob:none --sparse https://github.com/opensim-org/opensim-models
(cd opensim-models && git sparse-checkout set Models/Rajagopal_OpenSense)
RAJ=opensim-models/Models/Rajagopal_OpenSense/Rajagopal2015_opensense.osim
TSM=opensim-core/OpenSim/Tests/shared/ThoracoscapularShoulderModel.osim
python tools/compare_paths.py --model $RAJ                    # Rajagopal
python tools/compare_paths.py --model $TSM --truth-model $RAJ # Thoracoscapular
```
