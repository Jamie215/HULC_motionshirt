# Data Collection Checklist

A run-sheet for capturing a session with the HULC Motion Shirt IMU nodes and
taking it all the way to a visual. Commands assume you run them from the repo
root with the nodes flashed from `firmware/firmware.ino`.

**Mental model:** each node logs to its own flash **autonomously**. When it
senses motion it enters `ACTIVE_RECORDING` and writes quaternion records; when
still it sits in `IDLE` (which logs nothing). **Sync and offload work only while
a node is `IDLE`**, so connect the laptop while the subject is still, and do the
actual movement with the laptop disconnected.

The worked example below is the 2-node **elbow** montage
(upper_arm_r + forearm_r). For other placements, only the montage and the log
order change — the steps are identical.

---

## 0. Before you start

- [ ] `pip install bleak` is done (once per machine).
- [ ] Central is a well-behaved BLE host — **iOS / Android / Linux (BlueZ)**.
      A Windows-desktop `bleak` central imposes a slow connection interval and
      makes offload crawl; it's a bench artifact, not a firmware limit.

## 0b. Enroll the nodes → montage (one-time per rig)

Instead of hand-writing `montage.json` and guessing which board is where, map
each board by **powering one node at a time** — with only one advertising, its id
is unambiguous. No firmware change, no streaming. Because the `HULC-IMU-XXXX` id
is a permanent per-board property, this is a **one-time** job per board.

```bash
# add --erase to also wipe each node's flash in the same pass (see step 1)
python tools/multinode_test.py enroll --segments upper_arm_r,forearm_r --erase
```

- [ ] When prompted for `upper_arm_r`, power ON **only** that board (all others
      OFF), press Enter — it finds the single advertising id and reports it.
- [ ] Type a physical **label** (e.g. "orange tape") and mark the board.
- [ ] With `--erase`, it wipes that node (~30 s) before moving on.
- [ ] Repeat for `forearm_r` (power ONLY it on).
- [ ] It writes a schema-valid `montage.json` (columns in enrollment order) plus
      `nodes_registry.json` (remembers id → segment/label).

Next time the same (labeled) boards are used, skip the power-cycling — power them
all on and reuse the saved mapping:

```bash
python tools/multinode_test.py enroll --segments upper_arm_r,forearm_r --reuse
```

- [ ] It scans, sees both known ids, offers "reuse previous placement?" and
      writes the montage directly.

> `montage.json` is written UTF-8 without a BOM automatically. If you ever edit
> it by hand, keep it BOM-free (Notepad "Save as UTF-8" and PowerShell `>` add a
> BOM; the tools tolerate a UTF-8 BOM but a UTF-16 file still needs re-saving).

---

## 1. Erase both nodes (start clean)

If you used `enroll --erase` above, the flash is already wiped — skip this.
Otherwise, erase both:

```bash
python tools/multinode_test.py erase --count 2
```

- [ ] Both nodes report `OK — log is now 0KB (wipe confirmed)`.
- [ ] Do **not** power off during the ~30 s per-node erase.

## 2. Power on and strap by label

- [ ] Both boards powered.
- [ ] Strap each board to the segment its **enrollment label** says:
      the `upper_arm_r` board → **right upper arm**, the `forearm_r` board →
      **right forearm**. (Enrollment already fixed which id is which.)
- [ ] Snug and consistently oriented — strap tilt shows up downstream as an
      uncalibrated "kink", not motion.
- [ ] Subject held still so both nodes settle into `IDLE`.

## 3. (Recommended) Sync + clock-quality check — while still

```bash
python tools/multinode_test.py check --count 2 --duration 30
```

- [ ] Subject **still** (nodes IDLE) for the whole 30 s.
- [ ] Every node reads `synced=True` after sync.
- [ ] Cross-node offset looks stable / small. (Alignment is ultimately
      recovered from motion in reconcile, so this is insurance — but it catches
      a bad link before you record.)
- [ ] Let it disconnect when done.

## 4. Record the movement — laptop disconnected

With nothing connected, run this **4-beat protocol**. It satisfies two separate
needs in one take: a *still* hold for calibration, and *shared* motion (both
segments moving together) so reconcile can lock the clock.

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

## 5. Offload both logs

Bring the subject to rest (nodes IDLE), then:

```bash
python tools/multinode_test.py offload --count 2 --out-dir ./capture
```

- [ ] Each node reports `COMPLETE — saved N bytes`.
- [ ] Files land as `./capture/HULC-IMU-XXXX.bin` (auto-named by id).
- [ ] If any records are reported missing, just re-run the same command — it
      re-requests only the holes.

*Combine offload + wipe:* `offload --count 2 --erase-after` erases
each node only after a verified-complete transfer, so the next capture starts
clean.

## 6. Analyze — one command from logs to viewer

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

### (Optional) re-don verification

Taking the shirt off and back on breaks the mounting offset (a charge cycle
does not). To confirm the cached calibration still holds after a re-don, record
a fresh short still hold, reconcile it to `redon.csv`, then:

```bash
python tools/calibrate_segments.py verify redon.csv montage.json \
    --calibration calibration.json
```

---

## Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| Offload takes minutes/hours | Windows `bleak` central forces a slow connection interval | Use an iOS/Android/BlueZ central; confirm with `--count 1 --offload` (throughput KB/s) |
| Many `pass 2: re-requesting …` lines | Weak link / distance / body blocking 2.4 GHz | Node close and line-of-sight to the central; re-run offload to fill holes |
| `json ... Expecting value: line 1 column 1 (char 0)` | `montage.json` has a BOM or is empty/UTF-16 | Re-save as UTF-8 **without BOM**; check first 3 bytes are not `239 187 191` |
| Offload rejected / empty | Node not in `IDLE`, or nothing recorded | Hold the subject still; confirm motion actually happened in step 4 |
| Low reconcile confidence | Weak/absent sync gesture — the two segments didn't move together | Redo beat 2 (whole-arm swings), wide and moderate |
| Neutral residual large in calibrate | Neutral hold wasn't still, or wrong window | Redo beat 1; or pin `--window` from `t_common_ms` in aligned.csv |
| Enroll: "N nodes advertising" | More than one board powered on | Power ON only the ONE node you're enrolling; others OFF |
| Enroll: "no HULC node advertising" | Board off, or advert not up yet | Power it on, wait a few seconds, retry (a connected node stops advertising) |
| `analyze: node log not found` | `--capture-dir` wrong, or a node didn't offload COMPLETE | Point at the offload dir; re-run offload to finish the missing node |

See `firmware/MULTINODE_TESTING.md` for the offload framing/recovery details and
`tools/SETUP_AND_CALIBRATION_PLAN.md` for the pipeline stages. Tools used here:
`multinode_test.py` (enroll → montage; erase/check/offload),
`analyze_session.py` (one-shot reconcile → render).
