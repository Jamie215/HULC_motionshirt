# Data Collection Checklist

A run-sheet for capturing a session with the HULC Motion Shirt IMU nodes and
taking it all the way to a visual. Commands assume you run them from the repo
root with the nodes flashed from `firmware/firmware.ino`.

**Mental model:** each node logs to its own flash **autonomously**. When it
senses motion it enters `ACTIVE_RECORDING` and writes quaternion records; when
still it sits in `IDLE` (which logs nothing). **Offload works only while a node
is `IDLE`**, so you connect the laptop *after* the recording, with the subject at
rest, and do the actual movement with the laptop disconnected.

**Connections are the slow part.** Every BLE command connects to each node up
front (slow, especially on a Windows central), so the flow keeps connects to the
minimum: identification is connect-free, and a routine session touches BLE twice
— a smart erase at the start (which *skips the ~30 s wipe when a node is already
empty*, so it's just a quick check then) and the offload at the end:

| Phase | BLE connect? |
|---|---|
| Enroll → montage (one-time) | **no** — scan only |
| Erase to a clean start | **yes** — but skips the wipe if already empty |
| Strap + record | no — disconnected |
| Offload | **yes** |
| Sync/link check | only occasionally, not per session |

**Why erase at the start, not after offload:** wiping before each recording
(rather than with `offload --erase-after`) keeps the **previous session's raw
capture on the node until you deliberately begin the next one** — a safety net if
a transfer looked complete but wasn't, a file is lost, or an analysis needs
redoing from raw. The smart-skip keeps the start-erase cheap when the node is
already empty. (You erase over BLE while the node is `IDLE`; a strapped-on board
never has to be unmounted to be wiped.)

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

## 2. Erase to a clean start (smart)

Connect once (over BLE — the boards stay strapped on) and wipe only what needs
wiping, so this take starts clean. This is also the point where last session's
raw is finally discarded — up to here it was still recoverable on the node.

```bash
python tools/multinode_test.py erase --count 2
```

- [ ] A node already at 0KB is **skipped** (no 30 s wipe — just the check).
- [ ] A node holding data prompts `[y/N]`; answer `y` to wipe (`erase --yes`
      skips the prompt for scripting).
- [ ] Wiped nodes report `OK — log is now 0KB`; do **not** power off mid-wipe.

## 3. Record the movement — laptop disconnected

Nothing is connected. Run this **4-beat protocol**. It satisfies two separate
needs in one take: a *still* hold for calibration, and *shared* motion (both
segments moving together) so reconcile can lock the clock from the motion alone.

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
       through the target range.
4. [ ] **Still, ~2 s** — so the nodes settle back to `IDLE`.

> Why the sync gesture: pure elbow flexion moves the forearm a lot but the upper
> arm barely at all, so on its own it gives weak clock alignment. A whole-arm
> swing (or, with a torso node, a trunk twist) turns every segment together →
> one strong, shared motion. reconcile matches the nodes' **world angular
> velocity** (direction as well as speed), so a single clear shared gesture is
> enough even if the rest of the session is the arm moving on its own. Keep it
> **wide and moderate**, not frantic — very fast motion aliases at ~10 Hz. The
> sync result is in `aligned.quality.json` (`sync_peak_ratio` well above 1.25
> = one clear match).

## 4. Offload the logs

Bring the subject to rest (nodes IDLE), then offload each log (the next session's
clean start is handled by step 2, not here):

```bash
python tools/multinode_test.py offload --count 2 --out-dir ./capture
```

- [ ] Each node reports `COMPLETE — saved N bytes`.
- [ ] Files land as `./capture/HULC-IMU-XXXX.bin` (auto-named by id), each with a
      `HULC-IMU-XXXX.seg.json` sidecar recording the node's own segment — so the
      capture dir is self-describing and `analyze_session` can cross-check it
      against the montage (it warns, but the montage stays authoritative).
- [ ] If any records are reported missing, just re-run the same command — it
      re-requests only the holes.

> `offload --count 2 --erase-after` exists (wipes each node once its offload
> verifies COMPLETE, saving a connect), **but it discards the on-device raw
> immediately** — so you lose the ability to re-offload if the transfer was
> subtly bad or the files are lost. The default flow deliberately erases at the
> *start* of the next session instead, keeping this capture recoverable until
> then. Use `--erase-after` only when the on-device backup isn't worth the extra
> start-of-session connect.

## 5. Analyze — one command from logs to viewer

`analyze_session.py` runs the whole post-offload chain
(reconcile → capability → calibrate → render) and binds each `.bin` to its
segment **in montage order automatically**, so there's no hand-ordering of logs:

```bash
python tools/analyze_session.py run --montage montage.json --capture-dir ./capture --out elbow.html
```

- [ ] The binding table it prints matches your placement
      (`n0 upper_arm_r ← …485C.bin`, `n1 forearm_r ← …B059.bin`).
- [ ] reconcile **confidence** is healthy (low = weak sync gesture; redo beat 2).
- [ ] calibrate reports a neutral window marked `[still ✓]` and offsets with a
      small pose spread (a large one = the neutral hold wasn't still).
- [ ] Open the printed `file://…/elbow.html`, **Jump to neutral**, then flip
      **Raw ↔ Calibrated** — the two bars should snap to the neutral pose in
      Calibrated. That toggle *is* the calibration check.

By default calibrate **auto-detects** the neutral window. If it picks the wrong
span, pin it: read the opening still hold off `t_common_ms` in the emitted
`aligned.csv` and re-run with `--window <t0>,<t1>`.

<details>
<summary>Prefer to run the four stages by hand?</summary>

```bash
python tools/reconcile_nodes.py ./capture/<n0-id>.bin ./capture/<n1-id>.bin --out aligned.csv
python tools/motion_capabilities.py montage.json
python tools/calibrate_segments.py calibrate aligned.csv montage.json --window <t0>,<t1> --out calibration.json --update-montage
python tools/skeleton_viewer.py render aligned.csv montage.json --calibration calibration.json --out elbow.html
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

Taking the shirt off and back on breaks the mounting offset (a charge cycle does
not). To confirm the cached calibration still holds after a re-don, record a
fresh short still hold, offload it (step 4) into `./redon`, reconcile that
capture to a CSV, then verify against the cached calibration:

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
| Low reconcile confidence | Weak/absent sync gesture — the two segments didn't move together | Redo beat 2 (whole-arm swings), wide and moderate |
| Neutral residual large in calibrate | Neutral hold wasn't still, or wrong window | Redo beat 1; or pin `--window` from `t_common_ms` in aligned.csv |
| Enroll: "N nodes advertising" | More than one board powered on | Power ON only the ONE node you're enrolling; others OFF |
| Enroll: "no HULC node advertising" | Board off, or advert not up yet | Power it on, wait a few seconds, retry (a connected node stops advertising) |
| `analyze: node log not found` | `--capture-dir` wrong, or a node didn't offload COMPLETE | Point at the offload dir; re-run offload to finish the missing node |

See `firmware/MULTINODE_TESTING.md` for the offload framing/recovery details and
`tools/SETUP_AND_CALIBRATION_PLAN.md` for the pipeline stages. Tools used here:
`multinode_test.py` (enroll → montage, incl. segment stamp; `read-segments`;
erase/offload; occasional check), `analyze_session.py` (one-shot reconcile →
render).
