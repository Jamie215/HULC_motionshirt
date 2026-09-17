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
- [ ] A `montage.json` exists and matches this placement, saved as **UTF-8
      without a BOM** (Notepad "Save as UTF-8" and PowerShell `>` add a BOM;
      the tools tolerate a UTF-8 BOM as of the `utf-8-sig` fix, but a UTF-16
      file still needs re-saving). Verify quickly:
      `python tools/motion_capabilities.py montage.json` should print the
      montage without a traceback.

Example `montage.json` for the elbow test (fill in your real board ids):

```json
{
  "schema_version": "1.0",
  "subject": { "id": "S01", "notes": "2-node elbow test" },
  "session": { "id": "YYYY-MM-DD-A", "aligned_csv": "aligned.csv" },
  "calibration": { "neutral_pose": "N-pose", "captured": true, "t_window_ms": [1000, 4000], "functional": [] },
  "nodes": [
    { "node_id": "HULC-IMU-XXXX", "column": "n0", "segment": "upper_arm_r", "landmark": "right humerus", "calibrated": true },
    { "node_id": "HULC-IMU-YYYY", "column": "n1", "segment": "forearm_r",   "landmark": "right forearm",  "calibrated": true }
  ]
}
```

---

## 1. Erase both nodes (start clean)

```bash
python tools/multinode_test.py --erase --count 2
```

- [ ] Both nodes report `OK — log is now 0KB (wipe confirmed)`.
- [ ] Do **not** power off during the ~30 s per-node erase.

## 2. Power on and confirm identities

- [ ] Both boards powered.
- [ ] Over USB serial each prints a **distinct** `[BLE] Advertising as
      HULC-IMU-XXXX`. Note which id is which — this maps a physical board to a
      montage column.

## 3. Strap the nodes

- [ ] `n0` board → **right upper arm** (humerus).
- [ ] `n1` board → **right forearm**.
- [ ] Snug and consistently oriented — strap tilt shows up downstream as an
      uncalibrated "kink", not motion.
- [ ] Subject held still so both nodes settle into `IDLE`.

## 4. (Recommended) Sync + clock-quality check — while still

```bash
python tools/multinode_test.py --count 2 --duration 30
```

- [ ] Subject **still** (nodes IDLE) for the whole 30 s.
- [ ] Every node reads `synced=True` after sync.
- [ ] Cross-node offset looks stable / small. (Alignment is ultimately
      recovered from motion in reconcile, so this is insurance — but it catches
      a bad link before you record.)
- [ ] Let it disconnect when done.

## 5. Record the movement — laptop disconnected

With nothing connected, run the protocol on the subject:

1. [ ] **Hold the N-pose still for ~5 seconds** at the very start — this is the
       neutral window calibration needs. Note roughly when it happened.
2. [ ] **Do the joint motion** — for the elbow, several slow flexion/extension
       reps through the target range. Motion drives `ACTIVE_RECORDING`.
3. [ ] **Return to still** at the end so the nodes drop back to `IDLE`.

## 6. Offload both logs

Bring the subject to rest (nodes IDLE), then:

```bash
python tools/multinode_test.py --offload --count 2 --out-dir ./capture
```

- [ ] Each node reports `COMPLETE — saved N bytes`.
- [ ] Files land as `./capture/HULC-IMU-XXXX.bin` (auto-named by id).
- [ ] If any records are reported missing, just re-run the same command — it
      re-requests only the holes.

*Combine offload + wipe:* `--offload --erase-after-offload --count 2` erases
each node only after a verified-complete transfer, so the next capture starts
clean.

## 7. Reconcile onto one timeline

**Pass the logs in montage-column order — `n0` first, `n1` second.** That order
is what binds `n0` → upper_arm_r and `n1` → forearm_r to match `montage.json`.

```bash
python tools/reconcile_nodes.py ./capture/HULC-IMU-XXXX.bin ./capture/HULC-IMU-YYYY.bin --out aligned.csv
```

- [ ] Printed **confidence** is healthy (low = the two didn't share enough
      motion; re-record with more overlapping movement).

---

## 8. Post-reconcile (align → capability → calibrate → visualize)

```bash
# which joints this placement can compute (elbow yes; shoulder/wrist blocked)
python tools/motion_capabilities.py montage.json

# solve the sensor->segment mounting from the neutral window (t0,t1 in ms,
# read off t_common_ms in aligned.csv for the opening still hold from step 5)
python tools/calibrate_segments.py calibrate aligned.csv montage.json \
    --window <t0>,<t1> --out calibration.json --update-montage

# bake the self-contained viewer; open elbow.html in a browser
python tools/floating_fbd.py render aligned.csv montage.json \
    --calibration calibration.json --out elbow.html
```

- [ ] Capability report agrees the elbow is computable.
- [ ] Calibrate prints a neutral residual that collapses to ~0° (a large value
      means the "still" window wasn't actually still — re-pick it).
- [ ] In the viewer, **Jump to neutral** then flip **Raw ↔ Calibrated**: the two
      bars should snap to the neutral pose in Calibrated. That toggle *is* the
      calibration check.

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
| Offload rejected / empty | Node not in `IDLE`, or nothing recorded | Hold the subject still; confirm motion actually happened in step 5 |
| Low reconcile confidence | Nodes didn't share enough motion | Record more overlapping movement across both segments |
| Neutral residual large in calibrate | `--window` not a truly still span | Re-read `t_common_ms` and pick a quiet window |

See `firmware/MULTINODE_TESTING.md` for the offload framing/recovery details and
`tools/SETUP_AND_CALIBRATION_PLAN.md` for the pipeline stages.
