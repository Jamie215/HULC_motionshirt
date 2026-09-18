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
front (slow, especially on a Windows central), so this flow is arranged to need
just **one connect per routine session** — the offload — with identification and
erase either one-time or folded into that single connect:

| Phase | BLE connect? |
|---|---|
| Enroll → montage (one-time) | **no** — scan only |
| Strap + record | no — disconnected |
| Offload **+ self-clean** | **yes — the one connect** |
| Sync/link check | only occasionally, not per session |

The worked example below is the 2-node **elbow** montage
(upper_arm_r + forearm_r). For other placements, only the montage changes — the
steps are identical.

---

## 0. Before you start

- [ ] `pip install bleak` is done (once per machine).
- [ ] Central is a well-behaved BLE host — **iOS / Android / Linux (BlueZ)**.
      A Windows-desktop `bleak` central imposes a slow connection interval and
      makes offload crawl; it's a bench artifact, not a firmware limit.

## 0b. Enroll the nodes → montage (one-time per rig, no BLE connect)

Map each board to a body segment by **powering one node at a time** — with only
one advertising, its id is unambiguous. This only **scans advertisements; it
never connects**, so it's fast. Because the `HULC-IMU-XXXX` id is a permanent
per-board property, this is a **one-time** job per board.

```bash
python tools/multinode_test.py enroll --segments upper_arm_r,forearm_r
```

- [ ] When prompted for `upper_arm_r`, power ON **only** that board (all others
      OFF), press Enter — it finds the single advertising id and reports it.
- [ ] Type a physical **label** (e.g. "orange tape") and mark the board.
- [ ] Repeat for `forearm_r` (power ONLY it on).
- [ ] It writes a schema-valid `montage.json` (columns in enrollment order) plus
      `nodes_registry.json` (remembers id → segment/label).

Next time the same (labeled) boards are used, reuse the saved mapping (also
scan-only, no connect):

```bash
python tools/multinode_test.py enroll --segments upper_arm_r,forearm_r --reuse
```

- [ ] It scans, sees both known ids, offers "reuse previous placement?" and
      writes the montage directly.

> `montage.json` is written UTF-8 without a BOM automatically. If you ever edit
> it by hand, keep it BOM-free (Notepad "Save as UTF-8" and PowerShell `>` add a
> BOM; the tools tolerate a UTF-8 BOM but a UTF-16 file still needs re-saving).

## 0c. One-time only: wipe to a clean start

Do this **once** for a rig (or after a fresh firmware flash) so no stale records
linger. After that you never run a standalone erase again — step 3's
`--erase-after` self-cleans at the end of every session.

```bash
python tools/multinode_test.py erase --count 2
```

- [ ] Both nodes report `OK — log is now 0KB (wipe confirmed)`.
- [ ] Do **not** power off during the ~30 s per-node erase.

---

## 1. Strap by label

- [ ] Both boards powered.
- [ ] Strap each board to the segment its **enrollment label** says:
      the `upper_arm_r` board → **right upper arm**, the `forearm_r` board →
      **right forearm**. (Enrollment already fixed which id is which.)
- [ ] Snug and consistently oriented — strap tilt shows up downstream as an
      uncalibrated "kink", not motion.

## 2. Record the movement — laptop disconnected

Nothing is connected. Run this **4-beat protocol**. It satisfies two separate
needs in one take: a *still* hold for calibration, and *shared* motion (both
segments moving together) so reconcile can lock the clock from the motion alone.

1. [ ] **Neutral hold, ~5 s** — stand in the N-pose, still. This is the
       calibration window.
2. [ ] **Sync gesture, ~5 s** — 3–5 big **whole-arm** swings (elbow locked, move
       from the shoulder) so **both** nodes move together. This is what gives
       reconcile a strong, correlated signal to align on.
3. [ ] **The movement of interest** — e.g. slow elbow flexion/extension reps
       through the target range.
4. [ ] **Still, ~2 s** — so the nodes settle back to `IDLE`.

> Why the sync gesture: pure elbow flexion moves the forearm a lot but the upper
> arm barely at all, so on its own it gives weak clock alignment. A whole-arm
> swing moves both segments together → strong correlation. Keep it **wide and
> moderate**, not frantic — very fast motion aliases at ~10 Hz logging.

## 3. Offload + self-clean — the one connect

Bring the subject to rest (nodes IDLE), then run the **single** BLE command of
the session — it offloads each log **and** wipes the node afterwards, so the next
capture starts clean with no separate erase:

```bash
python tools/multinode_test.py offload --count 2 --out-dir ./capture --erase-after
```

- [ ] Each node reports `COMPLETE — saved N bytes`, then `erasing… 0KB`.
- [ ] Files land as `./capture/HULC-IMU-XXXX.bin` (auto-named by id).
- [ ] If any records are reported missing, just re-run the same command — it
      re-requests only the holes. (A node whose offload was **incomplete** is not
      erased, so you never lose data you didn't fully receive.)

## 4. Analyze — one command from logs to viewer

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
python tools/floating_fbd.py render aligned.csv montage.json --calibration calibration.json --out elbow.html
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
fresh short still hold, offload it (step 3) into `./redon`, reconcile that
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
| Offload rejected / empty | Node not in `IDLE`, or nothing recorded | Hold the subject still; confirm motion actually happened in step 2 |
| Low reconcile confidence | Weak/absent sync gesture — the two segments didn't move together | Redo beat 2 (whole-arm swings), wide and moderate |
| Neutral residual large in calibrate | Neutral hold wasn't still, or wrong window | Redo beat 1; or pin `--window` from `t_common_ms` in aligned.csv |
| Enroll: "N nodes advertising" | More than one board powered on | Power ON only the ONE node you're enrolling; others OFF |
| Enroll: "no HULC node advertising" | Board off, or advert not up yet | Power it on, wait a few seconds, retry (a connected node stops advertising) |
| `analyze: node log not found` | `--capture-dir` wrong, or a node didn't offload COMPLETE | Point at the offload dir; re-run offload to finish the missing node |

See `firmware/MULTINODE_TESTING.md` for the offload framing/recovery details and
`tools/SETUP_AND_CALIBRATION_PLAN.md` for the pipeline stages. Tools used here:
`multinode_test.py` (enroll → montage; erase/offload; occasional check),
`analyze_session.py` (one-shot reconcile → render).
