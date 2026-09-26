# Data Collection Checklist

A run-sheet for capturing a session with the HULC Motion Shirt IMU nodes and
taking it all the way to a visual. Commands assume you run them from the repo
root with the nodes flashed from `firmware/firmware.ino`.

**Mental model:** each node logs to its own flash **autonomously**. When it
senses motion it enters `ACTIVE_RECORDING` (~10 samples/s); after 10 s without
motion it drops to `STATIC_POSTURE` (1 sample/s); after 60 s without motion —
or ~8 s lying flat on a table — it sits in `IDLE`, which logs nothing.
**Offload works only while a node is `IDLE`**, so you connect the laptop *after*
the recording and do the movement with the laptop disconnected.

**Blocks.** The nodes' battery and flash are limited, so a session is a series
of **blocks**: record one or more takes, take the nodes off to charge, and
**offload while they charge** into one folder per block. Each block is one
mounting and is analyzed on its own. The recording procedure and the charging
checkpoint are in [`COLLECTION_SOP.md`](COLLECTION_SOP.md).

**Connections are the slow part.** Every BLE command connects to each node up
front (slow, especially on a Windows central), so the flow keeps connects to the
minimum: identification is connect-free, and a routine session touches BLE twice
— a smart erase at the start (which *skips the ~30 s wipe when a node is already
empty*, so it's just a quick check then) and the offload at the end:

| Phase | BLE connect? |
|---|---|
| Enroll → montage (one-time) | **no** — scan only |
| Erase to a clean start (start of each block) | **yes** — but skips the wipe if already empty |
| Strap + record | no — disconnected |
| Offload (while charging, end of each block) | **yes** |
| Sync/link check | only occasionally, not per session |

**Why erase as a separate step, not with the offload:** wiping only after
you have checked the offloaded files (rather than with `offload --erase-after`)
keeps the block's raw capture on the node until you deliberately begin the next
block — a safety net if a transfer looked complete but wasn't or a file is lost.
At a charging checkpoint that means: offload → quick check → erase → re-mount.
The smart-skip keeps the erase cheap when a node is already empty.

The worked example below is the 2-node **elbow** montage
(upper_arm_r + forearm_r). For other placements, only the montage changes — the
steps are identical.

---

## 0. Before you start

- [ ] `pip install bleak` is done (once per machine).
- [ ] Central is a well-behaved BLE host — **iOS / Android / Linux (BlueZ)**.
      A Windows-desktop `bleak` central imposes a slow connection interval and
      makes offload crawl; it's a bench artifact, not a firmware limit.

## 0b. Enroll the nodes → montage (one-time per rig)

Map each board to a body segment by **powering one node at a time** — with only
one advertising, its id is unambiguous. Identification is **scan-only**; then
enroll makes **one BLE connection per board** to (1) smart-erase and (2) **write
the segment into the node's log header** so the offloaded log is self-describing
(says which *body part*, not just which node). Because the `HULC-IMU-XXXX` id is a
permanent per-board property and the segment persists in flash, this is a
**one-time** job per board.

```bash
python tools/multinode_test.py enroll --segments upper_arm_r,forearm_r
```

- [ ] When prompted for `upper_arm_r`, power ON **only** that board (all others
      OFF), press Enter — it finds the single advertising id and reports it.
- [ ] Type a physical **label** (e.g. "orange tape") and mark the board.
- [ ] On the one connection it smart-erases (skips an already-empty board; asks
      `[y/N]` before wiping one that holds data) and writes the segment, then
      confirms the readback.
- [ ] Repeat for `forearm_r` (power ONLY it on).
- [ ] It writes a schema-valid `montage.json` (columns in enrollment order) plus
      `nodes_registry.json` (remembers id → segment/label).

Pass `--no-erase` to skip only the wipe check (the segment is still written).

Next time the same (labeled) boards are used, reuse the saved mapping —
**scan-only, no connect** (the segment already lives on each node):

```bash
python tools/multinode_test.py enroll --segments upper_arm_r,forearm_r --reuse
```

- [ ] It scans, sees both known ids, offers "reuse previous placement?" and
      writes the montage directly.

> **Tip — rebuild the montage from the boards themselves.** Once nodes are
> enrolled, `read-segments` connects to whatever is advertising, reads each
> node's own segment header, and writes the montage from that (you just confirm):
>
> ```bash
> python tools/multinode_test.py read-segments
> ```

> `montage.json` is written UTF-8 without a BOM automatically. If you ever edit
> it by hand, keep it BOM-free (Notepad "Save as UTF-8" and PowerShell `>` add a
> BOM; the tools tolerate a UTF-8 BOM but a UTF-16 file still needs re-saving).

---

## 1. Strap by label

- [ ] Both boards powered.
- [ ] Strap each board to the segment its **enrollment label** says:
      the `upper_arm_r` board → **right upper arm**, the `forearm_r` board →
      **right forearm**. (Enrollment already fixed which id is which.)
- [ ] Snug and consistently oriented — strap tilt shows up downstream as an
      uncalibrated "kink", not motion.

### Where on each segment (and why it matters less than you'd think)

The neutral-pose calibration solves for **however the sensor sits on the bone**
and subtracts it, so the *exact* spot is largely calibrated away — a static
placement offset does **not** shift your angles. What calibration can't fix is
(a) **soft-tissue artifact** (muscle bulging under the sensor moves it without
the bone moving — motion-correlated error that won't average out) and
(b) **re-don repeatability** (the cached calibration is only reusable if the
sensor lands the same way next wear). One rule optimizes both:

> **Place each sensor over the least contractile tissue, referenced to a
> palpable bony landmark, with the axial roll (rotation around the limb) pinned
> to a consistent flat spot.**

That single principle gives a *different* answer per segment because the anatomy
differs — it is not two philosophies:

| Segment | Put it | Referenced to | Avoid |
|---|---|---|---|
| **torso** | **anterior**, flat on the **sternum** | sternal midline | pec / upper abdomen (muscle + breathing motion) |
| **upper arm** | **lateral**, distal third just above the elbow | distal humerus | anterior biceps belly (bulges during flexion); proximal deltoid |
| **forearm** | **lateral/dorsal**, distal third | subcutaneous **ulnar border** | volar (palm-side) muscle bellies |
| **hand** | flat on the **dorsum** | 2nd–3rd metacarpal | — |

- [ ] **Torso is anterior for a specific reason, not just repeatability.** The
      facing/heading recovery assumes the torso board's normal points out of the
      chest (`TORSO_FORWARD_IN_SENSOR` in `calibrate_segments.py`). An anterior
      sternal mount is what lets the pipeline auto-recover which way the subject
      faced; a side-of-trunk mount is self-flagged low-confidence and loses it.
      If you ever standardize the torso sensor elsewhere, that constant has to
      move with it.
- [ ] **Arms are lateral/dorsal** to keep the sensor off the muscle bellies that
      bulge with the very motion you're measuring, and over bone that re-dons
      consistently.
- [ ] **Repeatability beats alignment.** You are not trying to line the sensor up
      with the bone — the pose does that. You are trying to place it the *same
      way every wear*. A bony, roll-referenced spot is what makes `verify` reuse
      the cached calibration instead of demanding a fresh pose.

## 2. Erase to a clean start (smart) — start of each block

Connect once over BLE and wipe only what needs wiping, so this block starts
clean. This is also the point where the previous block's raw is finally
discarded — up to here it was still recoverable on the node. At a charging
checkpoint, do it after the offload has been checked, before re-mounting.

```bash
python tools/multinode_test.py erase --count 2
```

- [ ] A node already at 0KB is **skipped** (no 30 s wipe — just the check).
- [ ] A node holding data prompts `[y/N]`; answer `y` to wipe (`erase --yes`
      skips the prompt for scripting).
- [ ] Wiped nodes report `OK — log is now 0KB`; do **not** power off mid-wipe.

## 3. Record the movement — laptop disconnected

Nothing is connected. The full procedure — warm-up, exact pose, the sync
gesture per montage, task pacing, rests, timings and the accept/redo checks —
is in **[`COLLECTION_SOP.md`](COLLECTION_SOP.md)**. In short:

0. [ ] **Warm-up, ~15 s** — slow arm circles / trunk turns, so every node is
       awake (nodes log nothing while IDLE) and the heading has settled.
1. [ ] **Neutral hold, ~5 s** — stand in the N-pose (arms straight at the
       sides, **palms facing the thighs**), still. This is the
       calibration window.
2. [ ] **Sync gesture, ~5 s** — a movement that turns **every** node together,
       so reconcile has a shared motion to align the clocks on:
       - arm-only montage (upper arm + forearm): 3–5 big **whole-arm** swings,
         elbow locked, moving from the shoulder;
       - montage with a **torso** node: 3–4 **trunk twists** (turn the upper
         body left–right) with the arm held against the side — an arm swing
         leaves the torso still, so it cannot sync it.
3. [ ] **The movement of interest** — e.g. slow elbow flexion/extension reps
       through the target range, ≤ ~1 rep/s, 5–8 s rests between sets.
4. [ ] **Closing hold, ~5 s** in the same N-pose — **required**: the analysis
       ends at it, so taking the nodes off afterwards is not analyzed. (Also
       the second pose for the strap-slip check.)
5. [ ] **End of block** — take the nodes off and lay them flat on the charger
       (they drop to `IDLE` within ~8 s), then the charging checkpoint: offload
       (step 4) → quick check → erase (step 2) → re-mount → next take from the
       warm-up. See [`COLLECTION_SOP.md`](COLLECTION_SOP.md) §3b.

> Why the sync gesture: pure elbow flexion moves the forearm a lot but the upper
> arm barely at all, so on its own it gives weak clock alignment. A whole-arm
> swing (or, with a torso node, a trunk twist) turns every segment together →
> one strong, shared motion. reconcile matches the nodes' **world angular
> velocity** (direction as well as speed), so a single clear shared gesture is
> enough even if the rest of the session is the arm moving on its own. Keep it
> **wide and moderate**, not frantic — very fast motion aliases at ~10 Hz. The
> sync result is in `aligned.quality.json` (`sync_peak_ratio` well above 1.25
> = one clear match).

## 4. Offload the logs — while the nodes charge, one folder per block

With the nodes on the charger (IDLE), offload each log into **a new folder for
this block** (the next block's clean start is step 2):

```bash
python tools/multinode_test.py offload --count 2 --out-dir ./capture/block1
```

- [ ] Each node reports `COMPLETE — saved N bytes`.
- [ ] Files land as `./capture/block1/HULC-IMU-XXXX.bin` (auto-named by id), each with a
      `HULC-IMU-XXXX.seg.json` sidecar recording the node's own segment — so the
      capture dir is self-describing and `analyze_session` can cross-check it
      against the montage (it warns, but the montage stays authoritative).
- [ ] If any records are reported missing, just re-run the same command — it
      re-requests only the holes.
- [ ] Quick check before erasing: `python tools/reconcile_nodes.py --inspect
      ./capture/block1/*.bin` — records present, ~10 Hz while moving, and **no
      `clock restarted` warning** (a battery died mid-block).

> `offload --count 2 --erase-after` wipes each node once its offload verifies
> COMPLETE — offload and erase in one connect, handy when a charging checkpoint
> is short. **But it discards the on-device raw immediately**, so you lose the
> ability to re-offload if the transfer was subtly bad or a file is lost. The
> default flow erases as a separate step after the quick check.

## 5. Analyze — one command from logs to viewer

`analyze_session.py` runs the whole post-offload chain
(reconcile → capability → calibrate → metrics → render, plus the optional
OpenSense path with `--opensense-model`) and binds each `.bin` to its
segment **in montage order automatically**, so there's no hand-ordering of logs:

```bash
python tools/analyze_session.py run --montage montage.json --capture-dir ./capture/block1 --outdir ./out/block1 --out elbow.html
```

Run it once **per block folder** — each block is one mounting with its own
neutral hold.

- [ ] The binding table it prints matches your placement
      (`n0 upper_arm_r ← …485C.bin`, `n1 forearm_r ← …B059.bin`).
- [ ] reconcile sync is reliable (`sync_peak_ratio` ≥ 1.25 in
      `aligned.quality.json`; low = weak or wrong sync gesture).
- [ ] calibrate reports a neutral window marked `[still ✓]` and offsets with a
      small pose spread (a large one = the neutral hold wasn't still), and a
      `closing hold: … — analysis ends here` line (missing = no closing N-pose;
      the analysis then includes taking the nodes off).
- [ ] The full accept/redo table is in [`COLLECTION_SOP.md`](COLLECTION_SOP.md) §5.
- [ ] Open the printed `file://…/elbow.html`, **Jump to neutral**, then flip
      **Raw ↔ Calibrated** — the two bars should snap to the neutral pose in
      Calibrated. That toggle *is* the calibration check.

By default calibrate **auto-detects** the neutral window (the first still
stretch; a montage window is used only if it was actually still) and the
closing hold (the last still stretch in the same pose). If it picks the wrong
opening span, pin it: read the still hold off `t_common_ms` in the emitted
`aligned.csv` and re-run with `--window <t0>,<t1>`.

<details>
<summary>Prefer to run the stages by hand?</summary>

```bash
python tools/reconcile_nodes.py ./capture/<n0-id>.bin ./capture/<n1-id>.bin --out aligned.csv
python tools/motion_capabilities.py montage.json
python tools/calibrate_segments.py calibrate aligned.csv montage.json --window <t0>,<t1> --out calibration.json --update-montage
python tools/metrics.py compute aligned.csv montage.json --calibration calibration.json --out metrics.json
python tools/skeleton_viewer.py render aligned.csv montage.json --calibration calibration.json --metrics metrics.json --out elbow.html
```
Pass the logs to reconcile in montage-column order (`n0` first).
</details>

---

## Occasional: sync / link-quality check (NOT every session)

You do **not** need this per session — reconcile recovers the cross-node timing
from the shared motion, so the BLE time-sync isn't required for the pipeline.
Run it only when **bringing up a new rig or chasing a flaky link** (it needs its
own connect, which is why it's kept off the routine path). Do it with the subject
**still** (nodes IDLE):

```bash
python tools/multinode_test.py check --count 2 --duration 30
```

- [ ] Every node reads `synced=True`; cross-node offset is stable / small. A bad
      offset here points at the link, before you waste a recording on it.

## Optional: re-don verification

Taking the shirt off and back on — or taking a node out and putting it back,
e.g. to charge it — can change the mounting offset (a charge cycle with the node
left in place does not). Every block has its own neutral hold anyway, so this
is only needed to confirm a cached calibration still holds: record a fresh short
still hold, offload it (step 4) into `./redon`, reconcile that capture to a CSV,
then verify against the cached calibration:

```bash
python tools/reconcile_nodes.py ./redon/<n0-id>.bin ./redon/<n1-id>.bin --out redon.csv
python tools/calibrate_segments.py verify redon.csv montage.json --calibration calibration.json
```

- [ ] Pass the logs in montage-column order (`n0` first — the same order as the
      montage), matching how `analyze_session.py` binds them.

---

## Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| Offload takes minutes/hours | Windows `bleak` central forces a slow connection interval | Use an iOS/Android/BlueZ central; confirm with `offload --count 1` (throughput KB/s) |
| Many `pass 2: re-requesting …` lines | Weak link / distance / body blocking 2.4 GHz | Node close and line-of-sight to the central; re-run offload to fill holes |
| `json ... Expecting value: line 1 column 1 (char 0)` | `montage.json` has a BOM or is empty/UTF-16 | Re-save as UTF-8 **without BOM**; check first 3 bytes are not `239 187 191` |
| Offload rejected / empty | Node not in `IDLE`, or nothing recorded | Hold the subject still; confirm motion actually happened in step 3 |
| Low reconcile confidence / `sync_peak_ratio` < 1.25 | Weak or wrong sync gesture — the nodes didn't move together | Redo the sync gesture: whole-arm swings (arm only) or trunk twists (with a torso node), wide and moderate |
| Neutral residual large in calibrate | Neutral hold wasn't still, or wrong window | Redo the hold; or pin `--window` from `t_common_ms` in aligned.csv |
| `! no closing hold found` | Take ended without a still N-pose | End every take with the closing hold; this block's analysis includes the unstrapping |
| `clock restarted` warning | A node rebooted mid-block (battery died, then recharged before offload) | Only the larger clock segment is analyzed; charge before the battery runs out and offload at every charge |
| Enroll: "N nodes advertising" | More than one board powered on | Power ON only the ONE node you're enrolling; others OFF |
| Enroll: "no HULC node advertising" | Board off, or advert not up yet | Power it on, wait a few seconds, retry (a connected node stops advertising) |
| `analyze: node log not found` | `--capture-dir` wrong, or a node didn't offload COMPLETE | Point at the offload dir; re-run offload to finish the missing node |

See `firmware/MULTINODE_TESTING.md` for the offload framing/recovery details and
`tools/SETUP_AND_CALIBRATION_PLAN.md` for the pipeline stages. Tools used here:
`multinode_test.py` (enroll → montage, incl. segment stamp; `read-segments`;
erase/offload; occasional check), `analyze_session.py` (one-shot reconcile →
render).
