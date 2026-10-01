# Collection reference — why each step, the numbers behind it, troubleshooting

The companion to the one-page [recording card](COLLECTION_SOP.md). The card
says what to do; this page says why, which pipeline or firmware setting each
number comes from, what the block check means, and what to do when something
goes wrong. If you change one of the named settings, update both pages.

---

## 1. What each step gives the pipeline

| Step | What the pipeline gets from it | Without it |
|---|---|---|
| **1 Strap on, switch on** | Nothing yet — and nothing here is used. Nodes may power up one by one and sit still for minutes; the analysis only starts after the sync movement. | — |
| **2 Sync movement** | One strong motion shared by every node, so their independent clocks can be aligned (`reconcile_nodes.py`); the **landmark** calibration uses to find the freeze; and motion that wakes every node and settles its magnetic heading. | Clocks off by milliseconds to tens of seconds; the freeze cannot be told from setup stillness |
| **3 Freeze** | The sensor→segment mounting offsets and the zero of every angle (`calibrate_segments.py`), plus which way the subject faces (torso node). Taken after the movement, so the nodes are awake and any strap shift from the twisting is already in the calibration. | Every angle measured against a wrong zero |
| **4 Task** | The movement of interest; with no torso node, elbow bends also give the facing. | — |
| **5 Nodes off** | Where the analysis ends: taken-off nodes lie still on the charger in a pose no body holds, then go IDLE. | — (detected; see §4) |

---

## 2. Timings and the settings behind them

| Step | Min / target / max | Why (the setting) |
|---|---|---|
| Sync movement | 4 / 10 / — s, 3–5 cycles of ~1–1.5 s | Clocks are aligned by matching the nodes' world angular-velocity vectors; one clear shared burst gives a distinct peak (`sync_peak_ratio` ≥ 1.25, `VECTOR_PEAK_RATIO_MIN`). Calibration recognises it as every node moving (≥ 0.4 rad/s) **and turning together** (coherence of their rotation vectors ≥ 0.8) for ≥ 3 s (`SYNC_MIN_RAD_S`, `SYNC_MIN_COHERENCE`, `SYNC_MIN_MS`). Real twists ran the torso at only 0.4–0.8 rad/s but at coherence 0.84–0.96; strapping a node on moves the nodes independently (coherence 0.2–0.7), so it never counts. Faster than ~2 cycles/s aliases at 10 Hz. 10 s lets a node that wakes late still catch most of it. |
| Freeze | 3 / 5 / 8 s, starting within 20 s of the movement | The freeze is the **first still stretch after the sync movement lasting ≥ 3 s whose quietest 2 s averages ≤ 0.04 rad/s (~2°/s)** (`SYNC_TO_HOLD_MAX_MS`, `NEUTRAL_MIN_HOLD_MS`, `NEUTRAL_QUIET_RAD_S`) — pauses in motion only slow to ~0.06–0.09 rad/s, so **really freeze**. Over 10 s still, a node drops to 1 Hz STATIC (`NOT_MOTION_TO_STATIC_MS`): usable, but 5 s keeps 10 Hz. |
| Rests in the task | 5–8 s | ≥ 5 s separates sets in the rep count (`REP_PAUSE_S`); ≤ 8 s keeps the nodes at 10 Hz. |
| Still in the task | < 60 s | After 60 s without motion a node goes IDLE and stops logging until moved (`NOT_MOTION_TO_IDLE_MS`); the first moments after waking can be lost. After a long break, repeat steps 2–3. |
| Nodes off | lying flat ≥ 10 s | IDLE needs ON_TABLE (3 s → STATIC, then 5 s → IDLE); offload needs IDLE. |

### The freeze pose (N-pose)

- Standing upright, weight even, looking ahead. Seated is fine if the trunk is
  upright and the arms hang freely clear of the chair.
- Arms straight down at the sides, **elbows fully straight**, shoulders relaxed.
- **Palms facing the thighs** (thumbs forward). This is the forearm's zero: it
  is what `metrics.py` and the OpenSense models assume.
- No talking, weight shifting or looking around. Breathing is fine.

### The sync movement, per montage

| Montage | Movement | Key points |
|---|---|---|
| upper arm + forearm | **Whole-arm swings** forward and back from the shoulder, elbow **locked straight**, ~60–90° each way | Bending the elbow during the swing weakens the match. |
| torso + upper arm | **Trunk twists** left–right, ±30–45°, with the hand **on the hip** | An arm swing leaves the torso still, so it cannot sync the torso node. |
| torso + upper arm + forearm | **Trunk twists** as above, whole arm against the side | One movement moves all three nodes. |

### Task rules

- **Speed:** at 10 Hz, at most ~1 repetition per second, smooth. Fast snaps
  (>~300°/s) are where angles and model fits are worst.
- **Range:** the full range of interest, but not past what the subject can hold
  steadily. Wobble grows with speed and effort.
- **No torso node:** the front direction comes from the elbow's hinge, which
  needs elbow flexion of ≥ 30° somewhere in the block (`HINGE_MIN_FLEX_DEG`).
  Curls cover it; otherwise add three slow elbow bends.
- **Straps:** if a strap slips or is adjusted, the rest of the block has a
  different mounting. End the block there (nodes off, charger, offload) and
  start a new one.

---

## 3. Before the session, in detail

- **Place.** Everything relies on the nodes sharing one magnetic-north frame. A
  metal object near one node and not the other breaks the sync, the facing and
  every relative angle, and **nothing in the log flags it.**
- **Montage for the question.**

  | Question | Nodes | Gives |
  |---|---|---|
  | Shoulder (elevation, plane, rotation) | torso + upper arm | shoulder angles vs the trunk |
  | Elbow / forearm | upper arm + forearm | elbow flexion, pro/supination |
  | Both, with 3 nodes | torso + upper arm + forearm | all of the above |

  Shoulder angles need the torso node; without it the pipeline blocks them,
  since trunk sway would read as shoulder motion.
- **Straps.** Soft-tissue wobble is the largest error left at 10 Hz: about 3°
  RMS doubles every angle error in simulation.
- **Block length.** A block ends when the nodes need charging. Storage is
  rarely the limit first: at 10 Hz (20 B per sample) a node halves its logging
  rate after ~2.3 h of ACTIVE recording (80% watermark) and stops at ~2.6 h;
  still time costs little (1 sample/s). Plan blocks by battery, never past
  ~90 min of recording.

---

## 4. Blocks, the charging checkpoint, and where the analysis ends

A **block** is everything between two offloads: one charge, one mounting, one
capture folder. The analysis of a block runs from the first freeze after the
sync movement to where the nodes came off.

**Where it ends.** No closing pose is needed. `calibrate_segments.find_session_end`
reads the end of the log node by node: a node laid on the charger rests still,
in a pose no body holds (tilted more than 60° from the freeze,
`OFF_BODY_TILT_DEG`), until the log ends. Nodes come off one at a time, so the
**first** node found lying off the body marks the take-off, even while the
other is still being handled. The analysis ends where taking them off began —
at the last whole-body pause within 10 s before that (`HANDLING_MAX_MS`),
otherwise 5 s before it (`HANDLING_MARGIN_MS`). If the log
ends in motion or with the nodes still worn, the analysis runs to its end and
the block check says so; `--end <t_ms>` sets the end by hand (read the time off
`t_common_ms` in `aligned.csv` or the review page).

**The checkpoint, step by step.**

1. Nodes off and flat on the charger: they drop to IDLE within ~8 s.
2. Offload into a new folder per block while they charge (they stay powered
   from their batteries): `multinode_test.py offload --count 2 --out-dir ./capture/blockN`.
3. Analyze and read the block check (§5). For a faster look without the full
   analysis: `reconcile_nodes.py --inspect ./capture/blockN/*.bin` (records,
   rate, clock restarts).
4. Erase: `multinode_test.py erase --count 2`. (`offload … --erase-after` does
   offload + erase in one go, wiping each node only after its offload verifies —
   faster, but the block is then no longer recoverable from the node.)
5. Re-mount — same node, same segment, same spot and orientation.

**Why one block per folder.**

- One block = one mounting. A re-mounted node sits at a slightly different
  angle, so it needs its own freeze; the pipeline calibrates each folder on its
  first freeze.
- If a node must come off mid-block without an offload (avoid this), analyze
  the part after it separately with that part's own freeze:
  `analyze_session.py run … --window t0,t1`.
- If a battery dies mid-block, the node restarts its clock when recharged. The
  loader detects it, keeps the larger part and warns; the rest is not analyzed.
- Never swap nodes between segments without updating the montage.

---

## 5. The block check, line by line

`analyze_session.py` ends with `BLOCK CHECK: OK` or `REDO` and one line per
check (also saved as `block_check.json`). **REDO** means re-record the block;
**NOTE** means usable, read the note.

| Check | Looks at | REDO / NOTE when | What to do |
|---|---|---|---|
| Sync | `aligned.quality.json` | REDO: a node not aligned, or vector peak ratio < 1.25 | Bigger sync movement that moves every node |
| Clock | `clock_restarts` | NOTE: a node rebooted mid-block | Charge before the battery runs out |
| Logging rate | each log's median rate | NOTE: under 8 Hz | Firmware version, flash watermark |
| Freeze | calibration `neutral.found_by` | REDO: no clean freeze after the sync movement (calibration fell back to a weaker choice); NOTE: a node was not fully still (pose spread > 3°) | Freeze ~5 s right after the movement, palms to the thighs |
| Facing | calibration `heading` | NOTE: front direction unknown, angles relative only | Torso node flat on the sternum; no torso node: include elbow bends |
| Session end | calibration `end` | NOTE: nodes not seen coming off | `--end <t_ms>` if the tail includes taking them off |
| Angles | metrics plausibility | NOTE: implausible angles for a joint | Check the freeze pose and strap slip |
| Model fit | OpenSense `fit_lost` | NOTE: the model lost the sensors for some seconds | Compare with the Sensors view there |

---

## 6. Common mistakes

| Mistake | What happens | Prevention |
|---|---|---|
| Sync movement that leaves a node still (arm swing with a torso node) | Clocks not aligned; no landmark for the freeze — REDO | Trunk twists with the hand on the hip |
| No real freeze (talking, swaying) after the movement | Calibration falls back to a weaker choice — REDO | "Freeze … two, three, four, five" |
| Freezing long after the movement (> 20 s) | The freeze is not tied to the movement; the first still moment of the log is used instead | Go straight from the movement into the freeze |
| Palms forward instead of to the thighs | Pro/supination zero off by ~90° | Cue "palms to your legs" |
| Elbows slightly bent in the freeze | Elbow zero off by that bend | Cue "arms straight" |
| Fast, snappy reps | Aliasing at 10 Hz; poor model fits | ≤ 1 rep/s, smooth |
| Still ≥ 60 s mid-block | Nodes go IDLE and stop logging | Rests of 5–8 s; after a long break repeat steps 2–3 |
| Working next to metal | Nodes disagree on north, unflagged | Before the session: place |
| Re-mounting without offloading | The rest is calibrated on the old mounting's freeze | Checkpoint: offload + erase before re-mounting |
| Battery dying mid-block | Clock restart; part of that node's log not analyzed | Charge before it runs out |

---

## 7. Recordings made with earlier protocols

- **Hold before the sync movement** (recordings before 2026-10): analyze with
  `--protocol hold-first` (or `"protocol": "hold-first"` under `calibration` in
  the montage). Calibrate prints a hint when a recording looks like one: a
  still hold ending right as the movement starts.
- **Closing hold:** no longer needed or looked for. Calibrations written while
  it was are still honoured: their `closing` window ends the analysis.
- **`t_window_ms: [1000, 4000]` in the montage:** the placeholder `enroll` used
  to write. It is ignored (on one capture it was still — the torso node lying
  on the table before strapping — and calibrated the torso ~85° off, drawing
  the trunk face-down). `enroll` no longer writes it; delete it from older
  montages if you like.
