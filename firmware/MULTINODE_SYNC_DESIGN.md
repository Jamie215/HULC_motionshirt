# Multi-Node Sync — Design Decisions

Status: **working design**, pending the central-platform test (below).
Scope: how multiple IMU nodes on one motion shirt stay time-aligned and get
their data off, given what the bench testing has shown so far.

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

### Finding: the slow interval is the *host*, not the firmware

The single-node read latency (~850 ms) was the same as the two-node case, so it
is **not** multi-connection scheduling — it is the connection interval itself.
The firmware requests a fast interval (`BLE.setConnectionInterval`), so the
prime suspect is the **central**: desktop **Windows' BLE stack (WinRT, used by
`bleak`) imposes slow intervals and does not let apps set connection
parameters.**

### Decision: validate on a good central before touching firmware further

- Firmware now requests **15–30 ms** (`setConnectionInterval(12, 24)`). 15 ms is
  the floor on purpose: **Apple's Bluetooth guidelines require Interval Min ≥
  15 ms**, or iOS rejects the request and uses its slow default. 15 ms is
  honored by iOS, Android, and BlueZ alike.
- **Test central = iPhone** (nRF Connect for Mobile). iOS is a well-behaved BLE
  central and is close to the eventual companion app. Note: **iOS does not
  expose the negotiated connection interval to apps** (CoreBluetooth hides it),
  so you can't read the number directly — infer it from *behavior*: trigger an
  offload and see whether the data streams fast (interval honored) or crawls
  (slow interval). If a real number is needed, either run the `--offload`
  harness on a **Mac** (`bleak` works over CoreBluetooth, same commands as the
  Windows run) or use **Android** nRF Connect, which *does* display the
  negotiated interval.

## Coordinated trigger (Req 1, the non-alignment half)

Today each node runs its **own** stability detector and transitions
independently. Risk: a **low-motion body segment** (e.g. torso) may not trip its
own detector, so it stays in IDLE and misses the event.

Plan: a **shared "any node detected motion → everyone record" trigger**,
distributed over BLE by the central, plus a short **pre-roll buffer** (keep the
last ~1 s of samples) so the boundary isn't lost. Because samples are
timestamped and analysis is offline, the trigger's BLE latency (hundreds of ms)
is acceptable — it does **not** need ms-level distribution.

## GPIO shared pulse — demoted

With "no wires" as a constraint, a shared GPIO line is **not** the production
sync mechanism. It remains useful only as an **optional, temporary bench tool**
to ground-truth that the wireless offline-sync method actually hits the 25–50 ms
target. Not on the critical path.

## Open risks / next steps

1. **[blocking] Central-platform test** — connect a non-Windows central and see
   if latency/throughput improve. iPhone (nRF Connect): trigger an offload and
   judge streaming speed (iOS hides the interval). Mac: run
   `multinode_test.py --offload` for a hard KB/s number. Decides whether Windows
   was the bottleneck.
2. **Offload throughput number** — `python tools/multinode_test.py --offload`
   gives a hard KB/s baseline (currently from the Windows host; re-run from a
   good central once available).
3. **Offline reconciliation** — implement offset+drift capture (bracket each
   recording) and a post-processing aligner for the offloaded logs.
4. **Shared trigger + pre-roll** — add the coordinated-record path and buffer.
5. **Sample rate check** — confirm 10 Hz is enough for the intended movement
   analysis; if faster motion matters, raise `activeHz` and re-check the
   alignment target.
