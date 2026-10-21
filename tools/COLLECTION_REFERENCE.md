# Collection reference — setup, why each step, the numbers behind it, troubleshooting

The companion to the one-page [recording card](COLLECTION_SOP.md). The card
says what to do; this page covers setting up the laptop and the nodes, where
to strap each node, why each step is there and which pipeline or firmware
setting each number comes from, the charging checkpoint in detail, what the
block check means, and what to do when something goes wrong. If you change one
of the named settings, update both pages.

Commands run from the repo root, with the nodes flashed from
`firmware/firmware.ino`. The examples use two nodes (`--count 2`); for other
montages only the segment names change.

---

## 1. What each step gives the pipeline

| Step | What the pipeline gets from it | Without it |
|---|---|---|
| **1 Strap on, switch on** | Nothing yet — and nothing here is used. Nodes may power up one by one and sit still for minutes; the analysis only starts after the sync movement. | — |
| **2 Sync movement** | One strong motion shared by every node, so their independent clocks can be aligned (`reconcile_nodes.py`); the **landmark** calibration uses to find the freeze; and motion that wakes every node and settles its magnetic heading. | Clocks off by milliseconds to tens of seconds; the freeze cannot be told from setup stillness |
| **3 Freeze** | The sensor→segment mounting offsets and the zero of every angle (`calibrate_segments.py`), plus which way the subject faces (torso node). Taken after the movement, so the nodes are awake and any strap shift from the twisting is already in the calibration. | Every angle measured against a wrong zero |
| **4 Task** | The movement of interest; with no torso node, elbow bends also give the facing. | — |
| **5 Nodes off** | Where the analysis ends: taken-off nodes lie still on the charger in a pose no body holds, then go IDLE. | — (detected; see §5) |

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
- **Laptop (once per machine).** Either the standalone app (no Python: see
  the README), or Python 3.11–3.13 and, from the repository folder,
  `pip install -r requirements.txt` (the pinned numpy, and bleak for the BLE
  commands; the session app's launcher offers to do this itself).
  The central should be a well-behaved BLE host — **iOS, Android or Linux
  (BlueZ)**: a Windows desktop `bleak` central forces a slow connection
  interval and makes offload crawl (a bench artifact, not a firmware limit).
  Body-model solves are optional (Python install only):
  `pip install -r requirements-opensense.txt` and
  `python tools/opensense_ik.py fetch-models` once.
- **Straps.** Soft-tissue wobble is the largest error left at 10 Hz: about 3°
  RMS doubles every angle error in simulation. Snug, and the same way round
  every time.
- **Block length.** A block ends when the nodes need charging. Storage is
  rarely the limit first: at 10 Hz (20 B per sample) a node halves its logging
  rate after ~2.3 h of ACTIVE recording (80% watermark) and stops at ~2.6 h;
  still time costs little (1 sample/s). Plan blocks by battery, never past
  ~90 min of recording.

### Where on each segment

The freeze calibration measures however each node sits on its segment and
subtracts it, so a static offset in the exact spot does **not** shift the
angles. What it cannot fix is **soft-tissue artifact** (muscle bulging under
the node moves it without the bone moving) and **repeatability** between
wears. One rule handles both: **over the least contractile tissue, referenced
to a bony landmark you can feel, with the roll around the limb pinned to a
consistent flat spot.**

| Segment | Put it | Referenced to | Avoid |
|---|---|---|---|
| torso | **anterior**, flat on the **sternum** | sternal midline | pec / upper abdomen (muscle and breathing motion) |
| upper arm | **lateral**, distal third, just above the elbow | distal humerus | the biceps belly (bulges in flexion); proximal deltoid |
| forearm | **lateral/dorsal**, distal third | the ulnar border (bone under the skin) | palm-side muscle bellies |
| hand | flat on the **dorsum** | 2nd–3rd metacarpal | — |

- **The torso node is anterior for a reason:** the facing is recovered on the
  assumption that the torso node's face points out of the chest
  (`TORSO_FORWARD_IN_SENSOR` in `calibrate_segments.py`). A side-of-trunk
  mount is flagged low-confidence and loses the facing; if the torso spot is
  ever standardised elsewhere, that constant moves with it.
- **Repeatability beats alignment.** The freeze lines the node up with the
  bone; the goal is the same placement every wear, which also lets
  `calibrate_segments.py verify` reuse a cached calibration.

---

## 4. Setting up the nodes

**What a node does on its own.** Each node logs to its own flash. Moving, it
records ~10 samples/s (`ACTIVE_RECORDING`); after 10 s still it drops to
1 sample/s (`STATIC_POSTURE`); after 60 s still — or ~8 s lying flat — it goes
`IDLE` and logs nothing. **Offload works only in `IDLE`**, so the laptop is
disconnected while recording and connects afterwards.

**Connections are the slow part.** Every BLE command connects to each node
first (slow, especially from Windows), so a routine block touches BLE twice:

| Step | BLE connect? |
|---|---|
| Enroll (once per rig) | scan to identify; one connect per node to store its segment |
| Erase to a clean start | yes — skips the ~30 s wipe if the node is already empty |
| Strap on and record | no — laptop disconnected |
| Offload (at the charger) | yes |
| Link check | only for a new rig or a flaky link |

### Enroll — once per rig

Enroll maps each board to a body segment by **powering one node at a time**:
with only one advertising, its `HULC-IMU-XXXX` id is unambiguous. On one
connection per board it smart-erases and **stores the segment on the node**
(log header, command `0x07`), where it survives power cycles and erases.

```bash
python tools/multinode_test.py enroll --segments upper_arm_r,forearm_r
```

1. When asked for `upper_arm_r`, power on **only** that board and press Enter.
2. Type a physical label (e.g. "orange tape") and mark the board.
3. It erases if needed (asks `[y/N]` before wiping data; `--no-erase` skips
   this), stores the segment and reads it back.
4. Repeat for each segment. It writes `montage.json` (a record of the rig) and
   `nodes_registry.json` (id → segment and label).

Next time with the same labelled boards, `enroll … --reuse` only scans. To see
what the nodes store, `python tools/multinode_test.py read-segments` reads
each node's segment and writes a montage from it. Each **offload** writes the
block's own `montage.json` from what the nodes store (labels from the
registry), and that is what the analysis uses. Swapping nodes between
segments therefore means re-enrolling them.

A `montage.json` edited by hand must stay UTF-8: the tools accept a UTF-8 BOM
(Notepad, PowerShell `>`), but a UTF-16 file must be re-saved.

### Link check — occasionally

The analysis aligns the clocks from the motion, so the BLE time sync is not
needed per session. For a new rig or a flaky link, with the nodes still:

```bash
python tools/multinode_test.py check --count 2 --duration 30
```

Every node should read `synced=True` with a small, stable cross-node offset;
a bad offset points at the link before a recording is wasted on it. Offload
framing and recovery are in `firmware/MULTINODE_TESTING.md`.

---

## 5. Blocks, the charging checkpoint, and where the analysis ends

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
the block check says so.

**Correcting a detection.** The review page's **Timeline** strip shows the
whole log (each node's speed), the detected sync movement, the freeze (green
band) and the end (dashed line). Drag the freeze band to where the subject
really froze — it snaps to the quietest 2 s nearby — and/or the end line, then
**Copy command**: it is the same analysis command with `--window t0,t1` /
`--end t` added. Values set by hand stay in the command on the next page.

**The checkpoint, step by step.**

1. **Nodes off** and flat on the charger: they drop to IDLE within ~8 s.
2. **Offload** into a new folder per block while they charge (they stay
   powered from their batteries):

   ```bash
   python tools/multinode_test.py offload --count 2 --out-dir ./capture/blockN --subject S01
   ```

   - Each node reports `COMPLETE — saved N bytes`. Missing records? Re-run the
     same command: it re-requests only the holes, and an incomplete log stays
     on the node.
   - The folder gets `HULC-IMU-XXXX.bin` per node, a `HULC-IMU-XXXX.seg.json`
     with the segment the node stores, and the block's `montage.json` built
     from those (`--session` optional). Keep the folder whole. A
     `montage.json` already there for the same nodes is kept; one for other
     nodes is never overwritten.
3. **Analyze** and read the block check (§6): drop the folder on the
   session app (the standalone **HULC** app, or `tools/app.py` through the
   double-click launchers in the repo folder; it runs the command below and
   shows the block check and review page, with **Apply** on the Timeline), or:

   ```bash
   python tools/analyze_session.py run --capture-dir ./capture/blockN --outdir ./out/blockN
   ```

   - The montage comes from the folder: its `montage.json`, else one built from
     the `.seg.json` files (saved to the output folder). `--montage FILE`
     overrides both, for a node that stored the wrong segment or logs from
     firmware that did not store one. The run prints which it used and warns
     if it disagrees with a node's header.
   - Check the binding table it prints against the placement
     (`n0 upper_arm_r <- HULC-IMU-485C.bin`, …).
   - With OpenSim and the fetched models, every block is also solved on each
     model (`--no-opensense` skips, `--opensense-model PATH` picks others); the
     review page gets a **Sensors / model** switch and a **compare solvers**
     overlay, and stretches where a model lost the sensors are shaded and left
     out of its numbers.
   - On the review page, **Neutral pose** then **Raw ↔ Calibrated**: the
     segments should snap to the freeze pose in Calibrated. That toggle is the
     calibration check.
   - A faster look without the analysis:
     `python tools/reconcile_nodes.py --inspect ./capture/blockN/*.bin` —
     records, ~10 Hz while moving, and no `clock restarted` warning.
4. **Erase**: `python tools/multinode_test.py erase --count 2`. An empty node
   is skipped; one holding data asks `[y/N]` (`--yes` for scripts); do not
   power off mid-wipe. Erasing only after the check keeps the block on the
   node until then, a safety net if a transfer looked complete but was not.
   (`offload … --erase-after` offloads and erases in one connect, wiping each
   node only after its offload verifies — faster, but the block is then gone
   from the node.)
5. **Re-mount** — same node, same segment, same spot and orientation.

**Why one block per folder.**

- One block = one mounting. A re-mounted node sits at a slightly different
  angle, so it needs its own freeze; the pipeline calibrates each folder on its
  first freeze.
- If a node must come off mid-block without an offload (avoid this), analyze
  the part after it separately with that part's own freeze:
  `analyze_session.py run … --window t0,t1`.
- If a battery dies mid-block, the node restarts its clock when recharged. The
  loader detects it, keeps the larger part and warns; the rest is not analyzed.
- Never swap nodes between segments without re-enrolling them: each node
  stores its segment, and the block's montage is built from what the nodes
  store. (A one-off swap can be fixed afterwards with `--montage`.)

**Running the stages by hand.** `analyze_session.py` runs these in order; by
hand, pass the logs to reconcile in montage column order (`n0` first), with
`montage.json` the block's own (copy it from `./capture/blockN/`):

```bash
python tools/reconcile_nodes.py ./capture/blockN/<n0-id>.bin ./capture/blockN/<n1-id>.bin --out aligned.csv
python tools/motion_capabilities.py montage.json
python tools/calibrate_segments.py calibrate aligned.csv montage.json --out calibration.json
python tools/metrics.py compute aligned.csv montage.json --calibration calibration.json --out metrics.json
python tools/skeleton_viewer.py render aligned.csv montage.json --calibration calibration.json --metrics metrics.json --out session.html
```

**Re-don check (optional).** Taking a node off and putting it back can change
its mounting; every block has its own freeze anyway, so this only confirms a
cached calibration still holds. Record a short still hold, offload it into
`./redon`, then:

```bash
python tools/reconcile_nodes.py ./redon/<n0-id>.bin ./redon/<n1-id>.bin --out redon.csv
python tools/calibrate_segments.py verify redon.csv montage.json --calibration calibration.json
```

---

## 6. The block check, line by line

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
| Trunk | metrics: torso median tilt from the freeze | REDO: over 30° for most of the session — the torso was calibrated in the wrong pose (e.g. its node still on the table) or its strap moved | Strap the torso node on before the sync movement; or `--window` the real freeze |
| Angles | metrics plausibility | REDO: a joint outside physiological limits for > 25% of the session (a calibration / facing error); NOTE: implausible angles | Check the freeze pose, the facing and strap slip |
| Model fit | OpenSense `fit_lost` | NOTE: the model lost the sensors for some seconds | Compare with the Sensors view there |

---

## 7. Common mistakes and troubleshooting

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
| Splitting up a capture folder | The block loses its montage or a node's segment | Keep each block's folder whole; if `montage.json` is lost, the analysis rebuilds it from the `.seg.json` files |

| Symptom | Likely cause | Fix |
|---|---|---|
| Offload takes minutes or hours | Windows `bleak` central forces a slow connection interval | iOS / Android / BlueZ central; `offload --count 1` prints the throughput |
| Many `pass 2: re-requesting …` lines | Weak link, distance, the body blocking 2.4 GHz | Node close to the laptop, in line of sight; re-run offload to fill the holes |
| Offload rejected or empty | Node not `IDLE`, or nothing recorded | Lay the nodes flat and wait ~8 s; check the block was recorded |
| `sync_peak_ratio` < 1.25 | The nodes did not move together | Sync movement that moves every node (§2) |
| `json … Expecting value: line 1 column 1` | A hand-edited `montage.json` is empty or UTF-16 | Re-save as UTF-8 |
| Enroll: "N nodes advertising" | More than one board powered on | Power on only the board being enrolled |
| Enroll: "no HULC node advertising" | Board off, or not advertising yet | Power it on, wait a few seconds, retry (a connected node stops advertising) |
| `analyze: node log not found` | Wrong `--capture-dir`, or a node did not offload COMPLETE | Point at the block folder; re-run offload for the missing node |
| `no montage: … cannot describe itself` | A log without a `.seg.json` (older firmware, or a node never enrolled) | Pass `--montage montage.json`, or enroll the node |

---

## 8. Recordings made with earlier protocols

- **Hold before the sync movement** (recordings before 2026-10): analyze with
  `--protocol hold-first` (or `"protocol": "hold-first"` under `calibration` in
  the montage). Calibrate prints a hint when a recording looks like one: a
  still hold ending right as the movement starts.
- **Closing hold:** no longer needed or looked for. Calibrations written while
  it was are still honoured: their `closing` window ends the analysis.
- **No `montage.json` in the capture folder** (offloaded before offload wrote
  one): the analysis builds it from the `.seg.json` files. Logs with no
  `.seg.json` (firmware before nodes stored their segment) need
  `--montage montage.json`.
- **`t_window_ms: [1000, 4000]` in the montage:** the placeholder `enroll` used
  to write. It is ignored (on one capture it was still — the torso node lying
  on the table before strapping — and calibrated the torso ~85° off, drawing
  the trunk face-down). `enroll` no longer writes it; delete it from older
  montages if you like.
