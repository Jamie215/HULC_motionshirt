# Data Collection SOP — motion, timing and acceptance checks

This is the **what to do with the body, and for how long** companion to
[`COLLECTION_CHECKLIST.md`](COLLECTION_CHECKLIST.md). The checklist covers
enrolment, strapping, erasing, BLE offload and the analysis command. This SOP
covers the recording itself: the calibration pose, the sync movement, the
task, the rests and the ending. It also gives the numbers the pipeline checks
afterwards, so you can accept or redo a take on the spot.

Every duration and limit below comes from a setting in the firmware or the
pipeline, named in the *Why* column. If you change one of those settings,
update this SOP too.

---

## 1. What each part of a take is for

| Part | What the pipeline gets from it | Fails without it |
|---|---|---|
| **Warm-up** | Wakes every node (nodes log nothing while IDLE), and gives the sensor fusion wide rotations to settle its magnetic heading | Missing first seconds; nodes disagreeing on "north" |
| **Neutral hold** | The sensor→segment mounting offsets and the zero for every angle (`calibrate_segments.py`) | Every angle is measured against a wrong zero |
| **Sync gesture** | One strong motion shared by all nodes, so their independent clocks can be aligned (`reconcile_nodes.py`) | Offsets off by milliseconds to tens of seconds; joint angles meaningless |
| **Facing** | Which way the subject faces: from the **torso node**, or with no torso node, from **elbow flexion** | Angles split along the wrong axes; flexion mixed with pronation |
| **Task** | The movement of interest | — |
| **Closing hold** | A second still pose, used to check for strap slip (`calibrate_segments.py verify`) | Slip goes undetected |
| **Rest-down** | Nodes drop to IDLE so they can be offloaded | Offload refused while a node is recording |

---

## 2. Before the session

- [ ] **Place.** Stay at least ~1 m from large metal and electronics: steel
      desks, radiators, cabinets, a laptop on the lap. Everything relies on the
      nodes sharing one magnetic-north frame. A metal object near one node and
      not the other breaks the sync, the facing and every relative angle, and
      **nothing in the log flags it.** Use the same spot for the whole take.
- [ ] **Montage for the question.** With two nodes:

      | Question | Nodes | Gives |
      |---|---|---|
      | Shoulder (elevation, plane, rotation) | torso + upper arm | shoulder angles vs the trunk |
      | Elbow / forearm | upper arm + forearm | elbow flexion, pro/supination |
      | Both, with 3 nodes | torso + upper arm + forearm | all of the above |

      Shoulder angles need the torso node. Without it the pipeline blocks
      them, since trunk sway would read as shoulder motion.
- [ ] **Straps.** Firm, on the flat of each segment (see the checklist,
      section 1). Soft-tissue wobble is the largest error left at 10 Hz: about
      3° RMS doubles every angle error in simulation.
- [ ] **Session length.** Per node, at the designed 10 Hz (20 B per sample),
      logging halves its rate after **~2.3 h of ACTIVE recording** (80%
      watermark) and stops at ~2.6 h. **Plan ≤ 90 min of recording between
      offloads.** Still time costs little (STATIC logs 1 sample/s).
- [ ] **Brief the subject** with the script in section 5, and demonstrate the
      N-pose and the sync gesture once.

---

## 3. The take — timeline

Durations: **min** = below this the pipeline may reject it; **target** = what to
aim for; **max** = beyond this something changes (e.g. the nodes' logging rate).

| # | Phase | Min / target / max | Do | Why (the setting behind it) |
|---|---|---|---|---|
| 0 | **Warm-up** | 10 / 15 / — s | Slowly move every instrumented segment through wide orientations: 2–3 big arm circles each way, 2 slow trunk turns left–right, turn the forearm palm-up/palm-down. Smooth, not fast. | Nodes log only in ACTIVE; motion wakes them from IDLE. Wide rotations help the BNO086 settle its magnetic heading (it calibrates itself from motion; the log does not record its status). |
| 1 | **Settle** | 2 / 3 / — s | Step into the N-pose and let the arms come to rest. | Setup motion before the hold is dropped automatically: analysis starts at the neutral hold. |
| 2 | **Neutral hold** | **3 / 5 / 8 s** | **N-pose, fully still.** Details below. | The finder takes the **first 2 s window averaging ≤ 0.10 rad/s (~6°/s)** (`NEUTRAL_MAX_RAD_S`). Over **10 s** without motion a node drops to 1 Hz STATIC (`NOT_MOTION_TO_STATIC_MS`): still usable, but 5 s keeps it at 10 Hz. |
| 3 | **Sync gesture** | 4 / 6 / 10 s | 3–5 cycles of the montage's gesture (below), ~1–1.5 s per cycle, wide and moderate. | Clocks are aligned by matching world angular-velocity vectors; a clear shared burst makes one distinct peak (`sync_peak_ratio` ≥ 1.25). Faster than ~2 cycles/s aliases at 10 Hz. |
| 4 | **Facing (no torso node only)** | 3 reps | Upper arm hanging still, 3 slow elbow flexions from straight to ≥ 90° and back (~2 s each), palm facing the thigh. | Facing comes from the elbow hinge: it needs flexion ≥ 30° (95th percentile, `HINGE_MIN_FLEX_DEG`) and a clear minimum (`HINGE_MAX_COST_RATIO`). Elbow-task curls usually cover it; doing it here makes it reliable. |
| 5 | **Task** | — | The movement of interest. Rules below. | — |
| 6 | **Closing hold** | 3 / 5 / 8 s | Same N-pose as step 2. | A second still pose for `verify` (strap-slip check). Recommended. |
| 7 | **Rest-down** (before offloading only) | 10 / — / — s | Take the nodes off, lay them flat on a table ≥ 10 s (or stay still ≥ 60 s). Between takes with the same straps, skip this: start the next take at step 0. | IDLE needs ON_TABLE (3 s → STATIC, then 5 s → IDLE) or 60 s without motion (`NOT_MOTION_TO_IDLE_MS`). Offload needs IDLE. |

A typical take is **~45 s of protocol plus the task**.

### The N-pose (steps 2 and 6)

- Standing upright, weight even, looking ahead. Seated is fine if the trunk is
  upright and the arms hang freely clear of the chair.
- Arms straight down at the sides, **elbows fully straight**, shoulders relaxed.
- **Palms facing the thighs** (thumbs forward). This is the forearm's zero: it
  is what `metrics.py` and the OpenSense models assume.
- No talking, weight shifting or looking around. Breathing is fine. The hold
  must average under ~6°/s.
- Hold the same pose in steps 2 and 6.

### The sync gesture (step 3) — depends on the montage

| Montage | Gesture | Key points |
|---|---|---|
| upper arm + forearm | **Whole-arm swings** forward and back from the shoulder, elbow **locked straight**, ~60–90° each way | Moving the elbow during the swing weakens the match. |
| torso + upper arm | **Trunk twists** left–right, ±30–45°, with the arm held **against the side** (hand on the hip or thigh) | An arm swing leaves the torso still, so it cannot sync the torso node. |
| torso + upper arm + forearm | **Trunk twists** as above, whole arm held against the side | One gesture moves all three nodes. |

### Task rules (step 5)

- **Speed:** at 10 Hz, keep movements at **≤ ~1 repetition per second** and
  smooth. Fast snaps (>~300°/s) are where the solved skeleton fits worst.
- **Sets and rests:** rest **5–8 s** between sets.
  - ≥ 5 s separates sets in the rep count (`REP_PAUSE_S`).
  - ≤ 8 s keeps the nodes at 10 Hz (STATIC after 10 s still).
  - **Never stay still ≥ 60 s mid-take.** The nodes go IDLE and stop logging
    until moved; the first moments after waking can be lost. For a longer
    rest, keep making small movements, or re-do steps 0–3 afterwards.
- **Range:** move through the full range of interest, but avoid reaching past
  what the subject can hold steadily. Wobble grows with speed and effort.
- **Straps:** if a strap is adjusted or slips, stop and re-do steps 2–3 with
  the new mounting (a new neutral hold).

---

## 4. Timeline templates (read-aloud)

**Elbow / forearm (upper arm + forearm), ~2 min**

| Time | Cue |
|---|---|
| 0:00 | "Circle your arm slowly, both directions … turn your palm up and down." (15 s) |
| 0:15 | "Arms down by your sides, palms facing your legs … and **hold still**." (settle 3 s + hold 5 s) |
| 0:23 | "Swing your straight arm forward and back, big and smooth — five times." (6 s) |
| 0:29 | "Arm by your side, bend your elbow up slowly and down — three times." (6 s) |
| 0:35 | Task, e.g. "Five slow curls" … rest 6 s … "five turns of the palm, up and down" … |
| ~1:45 | "Arms down, palms to your legs, **hold still**." (5 s) |
| ~1:50 | Last take: remove the nodes, lay them flat on the table for 10 s. |

**Shoulder (torso + upper arm), ~2 min**

| Time | Cue |
|---|---|
| 0:00 | "Circle your arm slowly … now turn your upper body slowly left and right." (15 s) |
| 0:15 | "Arms down by your sides, palms facing your legs … and **hold still**." (settle 3 s + hold 5 s) |
| 0:23 | "Hand on your hip. Twist your upper body left and right, smoothly — four times." (6 s) |
| 0:29 | Task, e.g. "Raise your arm forward as high as is comfortable, and down — five times" … rest 6 s … "now out to the side, five times" … |
| ~1:45 | "Arms down, palms to your legs, **hold still**." (5 s) |
| ~1:50 | Last take: remove the nodes, lay them flat on the table for 10 s. |

---

## 5. Accept or redo — right after the session

Offload and run the analysis (checklist, sections 4–5), then read these. They
are all printed by `analyze_session.py` or found in its outputs.

| Check | Where | Accept | If not |
|---|---|---|---|
| Logging rate | `reconcile_nodes.py --inspect <node>.bin` | ~10 Hz while moving, ~1 Hz while still | Firmware version / flash throttle (watermark) — check before the next take |
| Neutral window found | calibrate: `auto-detected first still window …` | At the time of step 2, **still ✓**, ≤ 0.10 rad/s | Hold longer / stiller; or pass `--window t0,t1` if you know when it was |
| Sync | `aligned.quality.json`: `sync_method`, `sync_peak_ratio`, `sync_reliable` | `vector`, ≥ 1.25, `true` | Redo with a bigger, cleaner sync gesture (right one for the montage) |
| Data loss | `aligned.quality.json`: `gap_frac`, `longest_gap_ms` | Gaps only where the subject was still | Check battery / node reset; a gap during motion is lost data |
| Facing | calibrate: `FACING (auto from torso)` or `FACING (from elbow hinge motion)` | confident; hinge contrast ≤ 0.75 | No torso: add the step-4 elbow flexions; torso: check the node is on the sternum, face up |
| Plausibility | metrics: no `! implausible` lines | none | Check the neutral pose (palms, elbows straight), strap slip, metal nearby |
| Fit (OpenSense path, optional) | `OPENSENSE fit` residuals | median ≲ 6°, >20° in < 5% of frames | High everywhere: calibration/facing; only on fast moves: slow the task down |

---

## 6. Common mistakes

| Mistake | What happens | Prevention |
|---|---|---|
| Starting the hold before the nodes woke up | The hold is missing from the log | Always do the warm-up (step 0) |
| Palms forward (anatomical position) instead of to the thighs | Pro/supination zero off by ~90° | Cue "palms facing your legs" |
| Elbows slightly bent in the hold | Elbow zero off by that bend | Cue "arms straight" and look before starting the count |
| Swinging the arm as the sync gesture with a torso node | Torso cannot be synced | Trunk twists with the hand on the hip |
| Fast, snappy task reps | Aliasing at 10 Hz; large fit residuals | ≤ 1 rep/s, smooth |
| Long still pauses (≥ 60 s) mid-take | Nodes go IDLE and stop logging | Keep rests 5–8 s, or re-do steps 0–3 after a long break |
| Working next to a metal desk / laptop | Nodes disagree on north, unflagged | Section 2: place |
| Re-strapping without a new hold | Old mounting applied to the new strap position | Re-do steps 2–3 after any strap change |
