# Multi-Node BLE Sync Testing

This document covers testing the **synced BLE connection across multiple IMU
nodes** — the step after the single-node Phase 3e firmware. A motion shirt uses
several IMUs (one per body segment); their quaternion streams are only fusable
if all nodes share a common time base to within a few milliseconds.

## What changed in the firmware for multi-node

| Change | Where | Why |
|---|---|---|
| **Unique per-board name** `HULC-IMU-XXXX` | `makeDeviceName()` | All boards used to advertise `HULC-IMU-01` and collide. The suffix is the last 4 hex chars of the board's BLE MAC, so it is stable per board with no per-board reflash (and needs no nRF MDK headers). |
| **Millisecond time sync** (control cmd `0x05`) | `handleControl()` | The legacy sync (`0x02`) carried a whole-**second** epoch — far too coarse to align nodes. `0x05` carries a `uint64` epoch in **ms**. |
| **Time Info characteristic** `A005` (read, 16 B) | `updateSyncInfo()` / `onSyncInfoRead()` | Lets the central read each node's live clock and reconstruct its epoch-now, so cross-node offset can be measured. Refreshed on every read. |
| **Fast connection interval** (7.5–15 ms) | `setup()` `BLE.setConnectionInterval(6,12)` | The first 2-node run showed A005 reads taking 300–800 ms, which floored offset measurement. Requesting a tight interval drops GATT read latency so the offset becomes meaningful. The central may clamp it — it is a request, not a guarantee. |

The legacy `0x02` (seconds) command still works and now also populates the ms
mapping, so nothing downstream breaks.

### A005 layout (little-endian)

| Bytes | Type | Field |
|---|---|---|
| 0–3 | uint32 | node `millis()` at the moment of the read (live) |
| 4–11 | uint64 | sync epoch in ms (0 if never synced) |
| 12–15 | uint32 | node `millis()` captured at the last sync |

The central computes each node's current epoch as:

```
node_epoch_now_ms = sync_epoch_ms + (node_millis_now - sync_millis)
```

and compares two nodes to get their clock offset (the host term cancels).

## Test setup (laptop harness, 2 boards)

1. Flash the firmware to **both** XIAO nRF52840 boards from `firmware/firmware.ino`.
   No per-board edits — each derives its own name from its BLE MAC.
2. Power both boards. Confirm over serial that each prints a **distinct**
   `[BLE] Advertising as HULC-IMU-XXXX`.
3. On the laptop:
   ```bash
   pip install bleak
   python tools/multinode_test.py --count 2 --duration 60
   ```

The harness scans, connects to both nodes simultaneously, syncs each with an
ms-resolution timestamp, then reads `A005` from both in a round-robin for the
duration and reports the cross-node offset and its drift.

Other modes:
* `--count 1` — single-node latency diagnostic (isolates per-link latency from
  multi-connection scheduling).
* `--offload` — measure log-offload throughput (KB/s) on one node, the number
  the flash-offload use case depends on.

**Central platform matters.** A slow negotiated connection interval floors both
read latency and offload throughput. Desktop Windows (via `bleak`) tends to
impose a slow interval; iOS/BlueZ honor the firmware's 15–30 ms request. To test
from an **iPhone**, use *nRF Connect for Mobile* and read the negotiated
connection interval on the connected node. See `MULTINODE_SYNC_DESIGN.md` for
the full rationale.

## How to read the results

* **CONNECT** — both nodes should report connected and *stay* connected. A node
  that is connected no longer advertises; if the harness can't find the second
  node, make sure it isn't already connected to something else (e.g. a phone).
* **read window** (per node) — how long each `A005` read took round-trip. This
  is the **measurement floor**: the node's clock is only known to within its
  read window. Bench observation is ~300–800 ms with two simultaneous
  connections, because the firmware services BLE in ~100 ms bursts in IDLE
  (`waitForIMUData`) and does not request a fast connection interval. Until this
  drops, offset cannot be measured to better than a few hundred ms.
* **offset** — reported as a value **± the read-window bound**. Treat it as an
  *upper bound* on the true offset, not a calibrated number. A result like
  "±300 ms" means only that the nodes are aligned to within a few hundred ms —
  it does **not** confirm ms-level sync.
* **drift** — the harness prints `NOT RESOLVABLE` whenever the measured slope is
  below the read-latency noise floor. Real relative crystal drift between two
  nRF52840s is tens of ppm → single-digit ms/min, which is far below that floor,
  so a large or negative "drift" from a noisy run is a **measurement artifact**,
  not physical clock behavior.

**Bottom line:** this method validates *connectivity and the sync path*, but the
BLE read latency makes it unable to confirm ms-level alignment. For that, reduce
the read latency (firmware: fast connection interval + continuous BLE servicing
when connected) and/or use a shared physical event (below).

## Known limitations / next steps

* **Sync precision is bounded by BLE write latency.** `0x05` is issued per node
  over its own connection, so each node's sync moment carries that node's write
  latency (a few ms, variable). For tighter alignment consider: a round-trip
  (NTP-style) estimate that measures and subtracts the one-way delay, or
  periodic re-sync to cancel accumulated drift.
* **No shared physical event yet.** The offset here is a software estimate. To
  ground-truth it, tap all nodes together (a shared impulse in the accel data)
  and compare the recorded timestamps of that event across nodes.
* **Connection count.** Two nodes is the current bench target; the nRF SoftDevice
  and the host adapter both cap simultaneous connections — validate the real
  ceiling before scaling to a full-shirt node count.
