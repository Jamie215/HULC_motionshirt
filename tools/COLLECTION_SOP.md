# Recording card

One page for the person running the session. The reasons, thresholds and
troubleshooting behind every step are in
[`COLLECTION_REFERENCE.md`](COLLECTION_REFERENCE.md), which also covers
setting up the laptop and nodes (enroll), where to strap each node, and the
charging checkpoint in detail.

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
   `python tools/multinode_test.py offload --count 2 --out-dir ./capture/block1 --subject S01`
   The folder now holds everything about the block: each node's log, which
   body part it was on, and a `montage.json` built from that. Keep the folder
   together; there is no separate montage file to save.
2. **Analyze** it: open the **HULC Sessions** app (double-click
   `HULC Sessions.bat` / `.command` in the repo folder), type the subject and
   drop `./capture/block1` on the page. Or from the command line:
   `python tools/analyze_session.py run --capture-dir ./capture/block1 --outdir ./out/block1`
3. **Read the `BLOCK CHECK`** (top of the app page, or the end of the output):
   - **OK** — keep going (read any NOTE lines).
   - **REDO** — do what the line under it says when you record the block again.
4. **Erase** the nodes: `python tools/multinode_test.py erase --count 2`
5. **Re-mount** and start the next block at step 1.
6. *Optional, but it is how the analysis gets better:* if the block was new in
   some way (new person, montage, room) or the analysis got it wrong, keep it as
   a test case: `python tools/regress.py add <date>_<montage> --capture-dir
   ./capture/block1 --description "what was done, when"`
   (see the README, "Changing the analysis safely").

Review the movement on the review page (in the app, or
`out/block1/session.html`); **Neutral pose** should show the arms hanging,
palms to the legs. If the freeze or the end was found in the wrong place,
click **Timeline** and drag the green freeze band or the end line to where they
really were, then press **Apply** in the app (or **Copy command** and run it).
In the app, **Keep as test case…** does step 6.
