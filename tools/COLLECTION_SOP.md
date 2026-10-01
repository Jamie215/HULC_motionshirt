# Recording card

One page for the person running the session. The reasons, thresholds and
troubleshooting behind every step are in
[`COLLECTION_REFERENCE.md`](COLLECTION_REFERENCE.md); enrolment, strapping
and BLE details are in [`COLLECTION_CHECKLIST.md`](COLLECTION_CHECKLIST.md).

## Before the session

- [ ] At least ~1 m from large metal and electronics (steel desk, radiator,
      laptop on the lap). Same spot for the whole session.
- [ ] Nodes for the question:
      **shoulder** → torso + upper arm · **elbow / forearm** → upper arm + forearm.
- [ ] Straps firm, on the flat of each segment. Same node on the same spot,
      same way round, every time.

## Each block

| # | Do | Say |
|---|---|---|
| 1 | **Strap on and switch on** every node. Take as long as you need. | — |
| 2 | **Sync movement, ~10 s.** Every node must move. | *Torso node:* "Hand on your hip. Twist your upper body left and right — five times." <br> *Arm only:* "Arm straight. Swing it forward and back — five times." |
| 3 | **Freeze, ~5 s,** straight after. | "Arms down, palms to your legs. Freeze … two, three, four, five." |
| 4 | **The task.** Smooth, at most ~1 repetition per second. Rest 5–8 s between sets. Never stay still for a minute or more. Arm-only sessions: include a few elbow bends. | e.g. "Raise your arm forward, as high as is comfortable, and down — five times." |
| 5 | **Done:** nodes off, flat on the charger. | — |

More tasks before charging? Carry on with step 4 — or, after a long break,
repeat steps 2–3 first.

## At the charger

1. **Offload** into a new folder for this block:
   `python tools/multinode_test.py offload --count 2 --out-dir ./capture/block1`
2. **Analyze** it:
   `python tools/analyze_session.py run --montage montage.json --capture-dir ./capture/block1 --outdir ./out/block1`
3. **Read the `BLOCK CHECK`** at the end of the output:
   - **OK** — keep going (read any NOTE lines).
   - **REDO** — do what the line under it says when you record the block again.
4. **Erase** the nodes: `python tools/multinode_test.py erase --count 2`
5. **Re-mount** and start the next block at step 1.

Open `out/block1/session.html` to review the movement; **Go to neutral pose**
should show the arms hanging, palms to the legs.
