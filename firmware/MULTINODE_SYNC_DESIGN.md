# Multi-Node Sync — Design Decisions

Status: **working design**, pending the central-platform test (below).
Scope: how multiple IMU nodes on one motion shirt stay time-aligned and get
their data off, given what the bench testing has shown so far.

> For the **math and algorithm** behind the offline reconciliation (clock model,
> cross-correlation, confidence, drift, references), see
> [`MULTINODE_SYNC_MATH.md`](MULTINODE_SYNC_MATH.md).

## Requirements

Two places need Bluetooth "sync":

1. **Coordinated collection** — when a stability change (motion) is detected,
   all nodes should record the movement, and their samples must be alignable
   afterward.
2. **Flash offload** — move each node's logged quaternions to a central app so
   the node's flash frees up.

## Constraints (decided)

| Constraint | Decision | Consequence |
|---|---|---|
| Shared wire between nodes | **Avoid** (last resort only) | BLE must carry sync; nodes are effectively independent/wireless. |
| Analysis timing | **Post-collection** (offline) | **No real-time streaming requirement.** Data is logged locally and offloaded in bulk. |
| Data of interest | **Quaternions** for movement analysis | ~10 Hz sample rate → alignment target is a fraction of a sample. |

## The key reframe

Because analysis is **post-hoc**, the ~850 ms BLE read latency seen on the
bench **does not affect the data path** — nodes never stream live. Each
quaternion is already timestamped with the node's `millis()`; alignment is done
**offline** by reconciling each node's clock to a common timeline. Latency only
affects two narrower things: offload throughput, and how precisely we can
*measure* each node's clock offset.

## Architecture (all wireless)

1. **Node**: detect motion → record quaternions to flash with `millis`
   timestamps. *(built)*
2. **Offload**: bulk-transfer each node's log to the central. *(built; see
   throughput risk below)*
3. **Offline reconciliation**: for each node, measure clock **offset + drift**
   vs a common reference (e.g. host epoch) at a few points bracketing the
   recording, fit a line, and map every `millis` timestamp onto the common
   timeline. Then align and analyze the quaternion streams.

### Clock alignment: primary vs refinement

Aligning the nodes' timestamps has two mechanisms, used together:

- **Primary — BLE clock-offset read + a start-of-session sync gesture.** On a
  fast central (phone) each node's clock offset is read directly (A005) at
  connect. A deliberate shared whole-body move at the start (jumping jack, torso
  twist, both arms up) anchors all nodes to one instant; the offset holds for
  the session (drift negligible over minutes). This needs no correlated activity.
- **Refinement — motion cross-correlation** (`reconcile_nodes.py`). When the
  nodes *do* share motion, cross-correlating their angular speed tightens the
  offset with no BLE dependency.

**Limitation of cross-correlation (important):** it needs shared motion to lock
onto. Two nodes on *independently* moving limbs (one arm swings, the other is
still) share nothing to correlate — the offset would be noise. The tool
therefore reports a **confidence** (correlation at the best lag) and flags
low-confidence results as unreliable, so the pipeline falls back to the BLE
offset / sync gesture rather than trusting a bad number. This is why the sync
gesture (a guaranteed shared event) is the primary anchor, not the activity.

### Precision target

Quaternions at 10 Hz = 100 ms/sample. For movement analysis, cross-node
alignment **better than ~25–50 ms** (a fraction of a sample) is the target —
**not** single-digit ms. This is achievable over BLE with offline
reconciliation: it needs *accurate timestamps + a good offset/drift fit*, not
low real-time latency.

## The one thing everything hinges on: the BLE connection interval

A slow negotiated connection interval hurts in exactly two ways:

- **Offload throughput (Req 2)** — throughput ≈ bytes-per-event ÷ interval. At
  ~850 ms it is a few hundred B/s; a full flash would take *hours*. Unusable.
- **Offset-measurement accuracy (Req 1)** — faster reads → tighter offset →
  hits the 25–50 ms target.

Fix the interval and both requirements fall into place. There is no wire to
fall back on, so this is the crux.

### Finding: the bottleneck is the *host*, not the firmware — CONFIRMED

Single-node read latency (~850 ms) matched the two-node case, so it isn't
multi-connection scheduling — it's the connection interval itself. The firmware
requests a fast interval (`BLE.setConnectionInterval`), so the culprit is the
**central**: desktop **Windows' BLE stack (WinRT, via `bleak`) imposes slow
intervals and won't let apps set connection parameters.** Confirmed on **iPhone
(2026-08):** a 48 KB log offloaded in **2.46 s → ~20 KB/s, ~10 ms/chunk** (vs
~850 ms per single read on Windows). So both requirements are met on a good
central — a full 2 MB flash offloads in ~1.7 min, and clock-offset reads drop to
~tens of ms (inside the 25–50 ms target) — and the product's central should be
**iOS/Android/BlueZ, not a Windows desktop** (`bleak`-on-Windows is a bench
artifact).

### Decision: validate on a good central before touching firmware further

- Firmware now requests **15–30 ms** (`setConnectionInterval(12, 24)`). 15 ms is
  the floor on purpose: **Apple's Bluetooth guidelines require Interval Min ≥
  15 ms**, or iOS rejects the request and uses its slow default. 15 ms is
  honored by iOS, Android, and BlueZ alike.
- **Test central = iPhone** (nRF Connect for Mobile) — a well-behaved BLE central
  close to the eventual companion app. iOS hides the negotiated interval from
  apps, so judge it by *behavior* (fast offload = interval honored); see
  `MULTINODE_TESTING.md` for reading the interval on each platform.

## Coordinated trigger (Req 1, the non-alignment half)

Each node runs its **own** stability detector and transitions
independently. Risk: a **low-motion body segment** (e.g. torso) may not trip its
own detector, so it stays in IDLE and misses the event.

Proposal: a **shared "any node detected motion → everyone record" trigger**,
distributed over BLE by the central, plus a short **pre-roll buffer** (keep the
last ~1 s of samples) so the boundary isn't lost. Because samples are
timestamped and analysis is offline, the trigger's BLE latency (hundreds of ms)
is acceptable — it does **not** need ms-level distribution.

**Implementation decision**

After discussion with the PIs, the node's idle state when there isn't a significant 
motion was sufficient for the purpose of the project given implementing such measure
can heavily cost the flexibility in design, battery life and flash storage.

## Open risks / next steps

1. ~~**[blocking] Central-platform test**~~ — **DONE** (see the Finding above):
   iPhone honored the fast interval; use a mobile/BlueZ central going forward.
2. ~~**Offline reconciliation**~~ — **DONE (host prototype).**
   `tools/reconcile_nodes.py` aligns two offloaded logs by cross-correlating the
   motion itself (angular speed, mounting-invariant), so it recovers the clock
   offset from the data and is **immune to BLE latency**. Drift is off by default
   (negligible over minute-scale records; opt-in for long ones). A synthetic
   `--selftest` validates the math with no hardware: recovers a known offset to
   ~15–18 ms, inside the 25–50 ms target. Next: run it on real offloaded logs
   from a shared-motion capture to confirm on-hardware.
4. **Confirm sync latency on iOS** — measure clock-offset read latency from a
   mobile/BlueZ central (should be ~tens of ms) to verify the 25–50 ms target
   is reachable.
5. **Offload pacing** — ~30% of the measured offload time was the fixed 3 ms
   `OFFLOAD_PACING_MS` delay. If faster offload is ever needed, reduce/remove it
   (writeValue() already backpressures) — headroom to ~28+ KB/s.
6. **Sample rate check** — confirm 10 Hz is enough for the intended movement
   analysis; if faster motion matters, raise `activeHz` and re-check the
   alignment target.
