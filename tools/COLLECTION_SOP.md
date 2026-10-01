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
| **Wake-up** | Wakes every node (nodes log nothing while IDLE), and gives the sensor fusion wide rotations to settle its magnetic heading. Nothing before the sync gesture is used for calibration, so setup — nodes powering up one by one, strapping, waiting — can take as long as it takes | Missing first seconds; nodes disagreeing on "north" |
| **Sync gesture** | One strong motion shared by all nodes, so their independent clocks can be aligned (`reconcile_nodes.py`). It is also the **landmark** the calibration uses to find the neutral hold right after it | Offsets off by milliseconds to tens of seconds; the hold cannot be told from setup stillness |
| **Neutral hold** | The sensor→segment mounting offsets and the zero for every angle (`calibrate_segments.py`). Taken after the gesture, so the nodes are awake and settled and any strap shift from the gesture is already in the calibration | Every angle is measured against a wrong zero |
| **Facing** | Which way the subject faces: from the **torso node**, or with no torso node, from **elbow flexion** | Angles split along the wrong axes; flexion mixed with pronation |
| **Task** | The movement of interest | — |
| **Closing hold** | **Where the analysis ends** — the last still N-pose after the task. Also a second pose for the strap-slip check (`calibrate_segments.py verify`) | Taking the nodes off (logged like any motion) is analyzed as if it were movement |
| **Rest-down** | Nodes drop to IDLE so they can be offloaded | Offload refused while a node is recording |

**Order: sync first, then the hold** (`protocol: sync-first`, the default).
Recordings made in the earlier order (hold, then sync) are analyzed with
`--protocol hold-first` (calibrate warns when a recording looks like one).

**Takes and blocks.** A *take* is steps 0–6 of section 3. A *block* is the
takes between two offloads — normally one charge of the nodes, one mounting,
and one capture folder. Nodes come off at the end of every block to charge, and
the logs are offloaded while they charge (section 3b).

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
- [ ] **Block length.** A block ends when the nodes need charging, and the
      logs are offloaded then (section 3b). Storage is rarely the limit first:
      at the designed 10 Hz (20 B per sample) a node halves its logging rate
      after **~2.3 h of ACTIVE recording** (80% watermark) and stops at
      ~2.6 h; still time costs little (1 sample/s). Plan blocks by battery,
      and never past ~90 min of recording.
- [ ] **Brief the subject** with the script in section 4, and demonstrate the
      sync gesture and the N-pose once.

---

## 3. The take — timeline

Durations: **min** = below this the pipeline may reject it; **target** = what to
aim for; **max** = beyond this something changes (e.g. the nodes' logging rate).

| # | Phase | Min / target / max | Do | Why (the setting behind it) |
|---|---|---|---|---|
| 0 | **Wake-up** | 10 / 15 / — s | All nodes on and strapped first (take as long as needed — nothing before step 1 is used). Then slowly move every instrumented segment through wide orientations: 2–3 big arm circles each way, 2 slow trunk turns left–right, turn the forearm palm-up/palm-down. Smooth, not fast. Flow straight into step 1. | Nodes log only in ACTIVE; motion wakes them from IDLE. Wide rotations help the BNO086 settle its magnetic heading (it calibrates itself from motion; the log does not record its status). |
| 1 | **Sync gesture** | 4 / 6 / 10 s | 3–5 cycles of the montage's gesture (below), ~1–1.5 s per cycle, wide and moderate. | Clocks are aligned by matching world angular-velocity vectors; a clear shared burst makes one distinct peak (`sync_peak_ratio` ≥ 1.25). Faster than ~2 cycles/s aliases at 10 Hz. Calibration recognises it as every node above **1 rad/s for ≥ 3 s** (`SYNC_MIN_RAD_S`, `SYNC_MIN_MS`) — a real gesture runs ~2 rad/s. |
| 2 | **Settle** | 2 / 3 / — s | Step into the N-pose and let the arms stop swaying. | The arm keeps swinging for a moment after the gesture; the finder uses the quietest 2 s of the hold, so the settle just has to end before the hold does. |
| 3 | **Neutral hold** | **3 / 5 / 8 s** | **N-pose, fully still.** Details below. Must start **within 20 s** of the end of the gesture. | The finder takes the **first still stretch after the sync gesture lasting ≥ 3 s whose quietest 2 s averages ≤ 0.04 rad/s (~2°/s)** (`SYNC_TO_HOLD_MAX_MS`, `NEUTRAL_MIN_HOLD_MS`, `NEUTRAL_QUIET_RAD_S`) — so **really freeze**. Over **10 s** without motion a node drops to 1 Hz STATIC (`NOT_MOTION_TO_STATIC_MS`): still usable, but 5 s keeps it at 10 Hz. |
| 4 | **Facing (no torso node only)** | 3 reps | Upper arm hanging still, 3 slow elbow flexions from straight to ≥ 90° and back (~2 s each), palm facing the thigh. | Facing comes from the elbow hinge: it needs flexion ≥ 30° (95th percentile, `HINGE_MIN_FLEX_DEG`) and a clear minimum (`HINGE_MAX_COST_RATIO`). Elbow-task curls usually cover it; doing it here makes it reliable. |
| 5 | **Task** | — | The movement of interest. Rules below. | — |
| 6 | **Closing hold** | **3 / 5 / 8 s** | **Same N-pose as step 3, fully still. Required.** | **The analysis ends here:** calibrate looks for the last still 2 s window, ≥ 5 s after the opening hold, where every node is tilted as in the opening hold within 10° (`CLOSING_MIN_GAP_MS`, `CLOSING_POSE_DEG`). If the arm is turned differently (palm back instead of to the thigh) the hold still counts but calibrate warns: keep the palms to the thighs, as in step 3; everything after it — taking the nodes off — is dropped. Without it the analysis runs to the end of the log. Also serves `verify` (strap slip). |
| 7 | **Rest-down** (end of block) | 10 / — / — s | Take the nodes off and lay them flat on the charger ≥ 10 s (or stay still ≥ 60 s), then the charging checkpoint (section 3b). Not needed between takes of a block. | IDLE needs ON_TABLE (3 s → STATIC, then 5 s → IDLE) or 60 s without motion (`NOT_MOTION_TO_IDLE_MS`). Offload needs IDLE. |

A typical take is **~50 s of protocol plus the task**.

### The N-pose (steps 3 and 6)

- Standing upright, weight even, looking ahead. Seated is fine if the trunk is
  upright and the arms hang freely clear of the chair.
- Arms straight down at the sides, **elbows fully straight**, shoulders relaxed.
- **Palms facing the thighs** (thumbs forward). This is the forearm's zero: it
  is what `metrics.py` and the OpenSense models assume.
- No talking, weight shifting or looking around. Breathing is fine. The hold
  must average under ~6°/s.
- Hold the same pose in steps 3 and 6.

### The sync gesture (step 1) — depends on the montage

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
    rest, keep making small movements, or re-do steps 0–3 afterwards
    (wake-up, sync gesture, settle, hold).
- **Range:** move through the full range of interest, but avoid reaching past
  what the subject can hold steadily. Wobble grows with speed and effort.
- **Straps:** if a strap slips or is adjusted mid-block, the rest of the block
  has a different mounting. End the block there (closing hold, charge/offload
  checkpoint, section 3b) and start a new one.

### 3b. Blocks and the charging checkpoint

The nodes run for a limited time and hold a limited log, so a session is a
series of **blocks**, each ending at a charge. The charge is also when the logs
come off the nodes:

1. **End the take** with the closing hold (step 6) — it marks where the
   block's analysis ends.
2. **Take the nodes off and put them on the charger.** Laid flat and still they
   drop to IDLE within ~8 s (logging stops; offload needs IDLE). The motion of
   taking them off is logged, but falls after the closing hold and is dropped.
3. **Offload into a new folder per block** while they charge (nodes stay
   powered from their batteries):
   `python tools/multinode_test.py offload --count 2 --out-dir ./capture/block2`
4. **Quick check** the block before wiping it off the nodes:
   `python tools/reconcile_nodes.py --inspect ./capture/block2/*.bin`
   (records, rate, no clock-restart warning) — or run the full analysis
   (section 5).
5. **Erase** so the next block starts clean:
   `python tools/multinode_test.py erase --count 2`
   (Steps 3 and 5 can be one command with `offload … --erase-after`, which wipes
   each node only after its offload verifies complete — faster, but the block
   is then no longer recoverable from the node.)
6. **Re-mount** — same node on the same segment, same spot and orientation —
   and start the next take at **step 0**.

Why one block per folder:

- **One block = one mounting.** A re-mounted node sits at a slightly different
  angle, so it needs its own neutral hold; the pipeline calibrates each capture
  folder on its first hold. Offloading and erasing at every charge keeps that
  automatic.
- **Takes within a block** share the block's calibration (the first take's
  opening hold) — correct as long as the nodes were not removed in between.
  The block is analyzed from that opening hold to the **last** closing hold.
- **If a node must come off mid-block** without an offload (avoid this),
  analyze the takes after it separately with that take's own hold:
  `analyze_session.py run … --window t0,t1`.
- **If a battery dies mid-block**, the node restarts its clock when recharged.
  The loader detects the restart, keeps the larger clock segment and warns;
  the other part is not analyzed. Offloading at every charge avoids it.
- Put nodes back the same way every time: it keeps the montage valid and lets
  `verify` confirm an old calibration. Never swap nodes between segments
  without updating the montage.

---

## 4. Timeline templates (read-aloud)

**Elbow / forearm (upper arm + forearm), ~2 min**

| Time | Cue |
|---|---|
| 0:00 | "Circle your arm slowly, both directions … turn your palm up and down." (15 s) |
| 0:15 | "Swing your straight arm forward and back, big and smooth — five times." (6 s) |
| 0:21 | "Arms down by your sides, palms facing your legs … and **hold still**." (settle 3 s + hold 5 s) |
| 0:29 | "Arm by your side, bend your elbow up slowly and down — three times." (6 s) |
| 0:35 | Task, e.g. "Five slow curls" … rest 6 s … "five turns of the palm, up and down" … |
| ~1:45 | "Arms down, palms to your legs, **hold still**." (5 s — the closing hold; required) |
| ~1:50 | Next take: back to 0:00. End of block: nodes off, onto the charger (section 3b). |

**Shoulder (torso + upper arm), ~2 min**

| Time | Cue |
|---|---|
| 0:00 | "Circle your arm slowly … now turn your upper body slowly left and right." (15 s) |
| 0:15 | "Hand on your hip. Twist your upper body left and right, smoothly — four times." (6 s) |
| 0:21 | "Arms down by your sides, palms facing your legs … and **hold still**." (settle 3 s + hold 5 s) |
| 0:29 | Task, e.g. "Raise your arm forward as high as is comfortable, and down — five times" … rest 6 s … "now out to the side, five times" … |
| ~1:45 | "Arms down, palms to your legs, **hold still**." (5 s — the closing hold; required) |
| ~1:50 | Next take: back to 0:00. End of block: nodes off, onto the charger (section 3b). |

---

## 5. Accept or redo — at each charging checkpoint

Run the analysis on the block's folder (checklist, section 5), ideally while
the nodes charge, then read these. They are all printed by `analyze_session.py`
or found in its outputs.

| Check | Where | Accept | If not |
|---|---|---|---|
| Logging rate | `reconcile_nodes.py --inspect <node>.bin` | ~10 Hz while moving, ~1 Hz while still | Firmware version / flash throttle (watermark) — check before the next take |
| Neutral window found | calibrate: `auto-detected neutral hold after the sync gesture …` | At the time of step 3, **still ✓**, ≤ 0.04 rad/s | `no hold right after a sync gesture`: make the gesture bigger and freeze within 20 s of it; a `hold-first` hint: an old-order recording — add `--protocol hold-first`; or pass `--window t0,t1` if you know when the hold was |
| Closing pose | calibrate: no `! closing hold: … turned` line | joints within 10° of the opening hold | Same N-pose at the end, palms to the thighs; a large value can also mean strap slip |
| Closing hold found | calibrate: `closing hold: … — analysis ends here`; metrics: `analysis ends at the closing hold` | At the time of the last step 6 | `! no closing hold found`: the take ended without a still N-pose — the analysis includes taking the nodes off; end every take with step 6 |
| No clock restart | reconcile / `--inspect`: no `clock restarted` warning; `clock_restarts: 0` in `aligned.quality.json` | 0 | A node rebooted (battery) mid-block: part of its log was not analyzed; charge earlier |
| Sync | `aligned.quality.json`: `sync_method`, `sync_peak_ratio`, `sync_reliable` | `vector`, ≥ 1.25, `true` | Redo with a bigger, cleaner sync gesture (right one for the montage) |
| Data loss | `aligned.quality.json`: `gap_frac`, `longest_gap_ms` | Gaps only where the subject was still | Check battery / node reset; a gap during motion is lost data |
| Facing | calibrate: `FACING (auto from torso)` or `FACING (from elbow hinge motion)` | confident; hinge contrast ≤ 0.75 | No torso: add the step-4 elbow flexions; torso: check the node is on the sternum, face up |
| Plausibility | metrics: no `! implausible` lines | none | Check the neutral pose (palms, elbows straight), strap slip, metal nearby |
| Fit (OpenSense path, optional) | `OPENSENSE fit` residuals | median ≲ 6°, >20° in < 5% of frames | High everywhere: calibration/facing; only on fast moves: slow the task down |

---

## 6. Common mistakes

| Mistake | What happens | Prevention |
|---|---|---|
| Holding the N-pose before the sync gesture (the old order) | Calibration takes a later rest as neutral; calibrate prints a `hold-first` hint | Gesture first, then the hold; for old recordings `--protocol hold-first` |
| Waiting too long after the gesture (> 20 s) before the hold | The hold is not tied to the gesture; calibrate falls back to the first hold of the log | Go straight from the gesture into settle + hold |
| Palms forward (anatomical position) instead of to the thighs | Pro/supination zero off by ~90° | Cue "palms facing your legs" |
| Elbows slightly bent in the hold | Elbow zero off by that bend | Cue "arms straight" and look before starting the count |
| Swinging the arm as the sync gesture with a torso node | Torso cannot be synced | Trunk twists with the hand on the hip |
| Fast, snappy task reps | Aliasing at 10 Hz; large fit residuals | ≤ 1 rep/s, smooth |
| Long still pauses (≥ 60 s) mid-take | Nodes go IDLE and stop logging | Keep rests 5–8 s, or re-do steps 0–3 after a long break |
| A weak sync gesture | Clocks may not align, and calibration cannot find the hold after it | Wide, moderate cycles that move **every** node (section 3, step 1) |
| Working next to a metal desk / laptop | Nodes disagree on north, unflagged | Section 2: place |
| Skipping the closing hold | Taking the nodes off is analyzed as movement (bogus range, reps) | End every take with step 6 |
| Re-mounting nodes without offloading first | The next takes are calibrated on the old mounting's hold | Charging checkpoint: offload + erase before re-mounting (section 3b) |
| Letting a battery die mid-block | Clock restart; part of that node's log is not analyzed | End the block and charge before the battery runs out |
