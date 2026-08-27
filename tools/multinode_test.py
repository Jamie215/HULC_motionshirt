#!/usr/bin/env python3
"""
HULC Motion Shirt — multi-node BLE sync test harness.

Acts as the *central* for testing the synced connection across multiple IMU
nodes (Phase 3e firmware, branch: bluetooth-multi-node-testing). It:

  1. Scans for nodes advertising as "HULC-IMU-XXXX" (unique per board).
  2. Connects to N nodes simultaneously.
  3. Time-syncs each node with millisecond resolution (control cmd 0x05).
  4. Reads each node's Time Info characteristic (A005) repeatedly and computes
     the cross-node clock offset and its drift over time.

This is a laptop-side test tool — no phone app required. It needs `bleak`:

    pip install bleak

Typical use (two boards on the bench):

    python tools/multinode_test.py --count 2 --duration 60

What to look for
----------------
* CONNECT: both nodes should connect and stay connected.
* OFFSET (initial): right after sync, pairwise offset should be a few ms
  (bounded by BLE write latency + the firmware's millis() capture).
* DRIFT (per minute): the offset should grow slowly and linearly — that is
  the relative crystal drift between the two nRF52840s (tens of ppm → a few
  ms/min is expected). A large or jumpy offset means the sync path, not the
  crystals, is the problem.

BLE GATT (see firmware.ino header for the authoritative layout):
  Service A0010000-...   Control A0010002 (write)   Time Info A0010005 (read)
"""

import argparse
import asyncio
import statistics
import struct
import sys
import time
from dataclasses import dataclass, field

try:
    from bleak import BleakClient, BleakScanner
except ImportError:  # pragma: no cover - dependency hint
    raise SystemExit("This harness needs bleak:  pip install bleak")

NAME_PREFIX = "HULC-IMU"
UUID_CONTROL = "A0010002-B0CE-4A4A-8F0B-0011223344FF"
UUID_STATUS = "A0010003-B0CE-4A4A-8F0B-0011223344FF"
UUID_OFFLOAD = "A0010004-B0CE-4A4A-8F0B-0011223344FF"
UUID_SYNCINFO = "A0010005-B0CE-4A4A-8F0B-0011223344FF"

CMD_SYNC_MS = 0x05
CMD_OFFLOAD = 0x04


def host_epoch_ms() -> int:
    """Host wall-clock in Unix epoch milliseconds."""
    return int(time.time() * 1000)


@dataclass
class NodeSample:
    """One read of a node's Time Info, paired with host time around the read."""

    host_before_ms: int
    host_after_ms: int
    node_millis_now: int
    sync_epoch_ms: int
    sync_millis: int

    @property
    def host_mid_ms(self) -> float:
        return (self.host_before_ms + self.host_after_ms) / 2.0

    @property
    def read_span_ms(self) -> int:
        """Round-trip window of the read — bounds the per-sample uncertainty."""
        return self.host_after_ms - self.host_before_ms

    @property
    def node_epoch_now_ms(self) -> int:
        """Node's own idea of the current epoch time, reconstructed from A005."""
        return self.sync_epoch_ms + (self.node_millis_now - self.sync_millis)

    @property
    def offset_vs_host_ms(self) -> float:
        """node_epoch_now - host_mid. Signed: +ve means the node clock is ahead."""
        return self.node_epoch_now_ms - self.host_mid_ms


@dataclass
class Node:
    name: str
    address: str
    client: BleakClient
    samples: list = field(default_factory=list)


def parse_syncinfo(data: bytes) -> tuple:
    """Unpack A005: uint32 millis_now, uint64 sync_epoch_ms, uint32 sync_millis."""
    if len(data) < 16:
        raise ValueError(f"Time Info too short: {len(data)} bytes (need 16)")
    node_millis_now, sync_epoch_ms, sync_millis = struct.unpack_from("<IQI", data, 0)
    return node_millis_now, sync_epoch_ms, sync_millis


async def scan(count: int, timeout: float) -> list:
    print(f"[SCAN] Looking for {count} '{NAME_PREFIX}-*' node(s) "
          f"({timeout:.0f}s)...")
    devices = await BleakScanner.discover(timeout=timeout)
    found = []
    for d in devices:
        name = d.name or ""
        if name.startswith(NAME_PREFIX):
            found.append(d)
            print(f"[SCAN]   found {name}  ({d.address})")
    if not found:
        raise SystemExit("[SCAN] No HULC nodes found. Are they powered and "
                         "advertising? (a connected node stops advertising)")
    if len(found) < count:
        print(f"[SCAN] WARNING: wanted {count}, found {len(found)}. "
              f"Continuing with what is available.")
    # Deterministic order so pairwise labels are stable across runs.
    found.sort(key=lambda d: d.name or d.address)
    return found[:count]


async def sync_node(node: Node) -> None:
    """Send a millisecond time-sync to one node (control cmd 0x05)."""
    epoch_ms = host_epoch_ms()
    payload = bytes([CMD_SYNC_MS]) + struct.pack("<Q", epoch_ms)
    await node.client.write_gatt_char(UUID_CONTROL, payload, response=True)
    print(f"[SYNC] {node.name}: sent epoch_ms={epoch_ms}")


async def read_syncinfo(node: Node) -> NodeSample:
    before = host_epoch_ms()
    data = await node.client.read_gatt_char(UUID_SYNCINFO)
    after = host_epoch_ms()
    millis_now, sync_epoch_ms, sync_millis = parse_syncinfo(bytes(data))
    return NodeSample(before, after, millis_now, sync_epoch_ms, sync_millis)


async def read_status(node: Node):
    """Read + print the status char. Returns the 'time synced' flag (or None)."""
    try:
        data = bytes(await node.client.read_gatt_char(UUID_STATUS))
    except Exception as exc:  # noqa: BLE001 - status is informational only
        print(f"[STATUS] {node.name}: read failed ({exc})")
        return None
    state = data[0] if len(data) > 0 else 255
    flags = data[1] if len(data) > 1 else 0
    log_kb = struct.unpack_from("<H", data, 2)[0] if len(data) >= 4 else 0
    state_name = {0: "IDLE", 1: "STATIC", 2: "ACTIVE"}.get(state, f"?{state}")
    synced = bool(flags & 0x02)
    print(f"[STATUS] {node.name}: state={state_name} "
          f"synced={synced} streaming={bool(flags & 0x01)} "
          f"log={log_kb}KB")
    return synced


def report_offsets(nodes: list) -> None:
    """Report pairwise cross-node offset from the collected samples."""
    print("\n===== CROSS-NODE OFFSET REPORT =====")
    for node in nodes:
        if not node.samples:
            continue
        spans = sorted(s.read_span_ms for s in node.samples)
        p90 = spans[min(len(spans) - 1, int(0.9 * len(spans)))]
        print(f"[{node.name}] samples={len(spans)}  read window (ms): "
              f"min {spans[0]}  median {statistics.median(spans):.0f}  "
              f"p90 {p90}  max {spans[-1]}")

    if len(nodes) < 2:
        print("\nSingle-node latency diagnostic — no pairwise offset.")
        print("Compare the read-window numbers above against a 2-node run:")
        print("  * similar (single-node also 100s of ms) -> the firmware/host")
        print("    link is slow even for one connection (interval not honored).")
        print("  * much smaller than 2-node -> the host's 2-connection")
        print("    scheduling is the bottleneck, not the firmware.")
        return

    # Pairwise: align each node's samples by index (reads are round-robin, so
    # sample i across nodes is close in time). Offset = A.offset_vs_host -
    # B.offset_vs_host, which cancels the host clock and leaves the true
    # cross-node clock difference.
    ref = nodes[0]
    for other in nodes[1:]:
        n = min(len(ref.samples), len(other.samples))
        if n < 3:
            print(f"{ref.name} vs {other.name}: not enough samples.")
            continue

        # Each paired sample carries an uncertainty set by how long the two
        # reads took: the node timestamps are only known to within their read
        # windows. Combined per-sample uncertainty ~ half the summed windows.
        rows = []
        for i in range(n):
            diff = (ref.samples[i].offset_vs_host_ms
                    - other.samples[i].offset_vs_host_ms)
            unc = (ref.samples[i].read_span_ms + other.samples[i].read_span_ms) / 2.0
            t = ref.samples[i].host_mid_ms
            rows.append((t, diff, unc))

        best_unc = min(u for _, _, u in rows)          # tightest single sample
        # Use the lowest-uncertainty third (min 5) for the offset estimate.
        clean = sorted(rows, key=lambda r: r[2])[:max(5, n // 3)]
        clean_diffs = [d for _, d, _ in clean]
        offset = statistics.mean(clean_diffs)
        spread = statistics.pstdev(clean_diffs) if len(clean_diffs) > 1 else 0.0

        # Drift = least-squares slope of diff vs time over ALL samples.
        t0 = rows[0][0]
        xs = [(t - t0) / 1000.0 for t, _, _ in rows]     # seconds
        ys = [d for _, d, _ in rows]
        mx, my = statistics.mean(xs), statistics.mean(ys)
        denom = sum((x - mx) ** 2 for x in xs)
        slope_per_min = (sum((x - mx) * (y - my) for x, y in zip(xs, ys))
                         / denom * 60.0) if denom > 0 else 0.0
        span_s = xs[-1] - xs[0]
        # A slope is only real if the offset moved further across the run than
        # the measurement noise. Floor: median per-sample uncertainty / run.
        med_unc = statistics.median(u for _, _, u in rows)
        drift_floor_per_min = (med_unc / span_s * 60.0) if span_s > 0 else float("inf")

        print(f"\n{ref.name}  vs  {other.name}:")
        print(f"  offset     : {offset:+.0f} ms   "
              f"(+/- {max(spread, best_unc):.0f} ms; best read window {best_unc:.0f} ms)")
        print(f"  spread     : {spread:.0f} ms across the clean subset "
              f"({len(clean)}/{n} tightest samples)")
        if abs(slope_per_min) < drift_floor_per_min:
            print(f"  drift      : NOT RESOLVABLE at this read latency "
                  f"(measured {slope_per_min:+.1f} ms/min < noise floor "
                  f"{drift_floor_per_min:.0f} ms/min)")
        else:
            print(f"  drift      : {slope_per_min:+.1f} ms/min "
                  f"(above the {drift_floor_per_min:.0f} ms/min noise floor)")
        print(f"\n  NOTE: read windows of ~{best_unc:.0f}-{med_unc:.0f} ms floor this")
        print(f"  measurement. Treat the offset as an upper bound, not a")
        print(f"  calibrated value. For true ms-level validation use a shared")
        print(f"  physical event (see MULTINODE_TESTING.md).")
    print("====================================\n")


async def measure_offload(node: Node, quiet_s: float = 3.0,
                          max_s: float = 300.0) -> None:
    """Measure log-offload throughput (Req 2) on one node.

    Subscribes to A004, sends control 0x04 (begin offload — IDLE only, does NOT
    erase), counts bytes until notifications go quiet, and reports KB/s. This
    is the hard number the flash-offload use case depends on; at a slow
    connection interval it will be painfully low.
    """
    state = {"bytes": 0, "chunks": 0, "first": None, "last": None}

    def on_chunk(_char, data: bytearray) -> None:
        now = time.monotonic()
        if state["first"] is None:
            state["first"] = now
        state["last"] = now
        state["bytes"] += len(data)
        state["chunks"] += 1

    await read_status(node)  # prints log size / IDLE state for context
    print(f"[OFFLOAD] {node.name}: starting — subscribing to A004...")
    await node.client.start_notify(UUID_OFFLOAD, on_chunk)
    await node.client.write_gatt_char(UUID_CONTROL, bytes([CMD_OFFLOAD]),
                                      response=True)

    t0 = time.monotonic()
    # Wait until the stream has been quiet for `quiet_s` after the last chunk,
    # or we hit max_s. (Firmware streams until all flash data is sent.)
    while True:
        await asyncio.sleep(0.5)
        elapsed = time.monotonic() - t0
        last = state["last"]
        if last is not None and (time.monotonic() - last) > quiet_s:
            break
        if elapsed > max_s:
            print(f"[OFFLOAD] {node.name}: hit max {max_s:.0f}s cap.")
            break
        if state["chunks"] and int(elapsed) % 5 == 0:
            print(f"[OFFLOAD] {node.name}: {state['bytes']/1024:.1f} KB "
                  f"in {elapsed:.0f}s...")

    try:
        await node.client.stop_notify(UUID_OFFLOAD)
    except Exception:  # noqa: BLE001
        pass

    print("\n===== OFFLOAD THROUGHPUT =====")
    if state["first"] is None or state["bytes"] == 0:
        print(f"[{node.name}] no data received. Is there a log to offload "
              f"(status log>0KB) and is the node in IDLE?")
    else:
        dur = max(1e-3, state["last"] - state["first"])
        kb = state["bytes"] / 1024
        print(f"[{node.name}] received {kb:.1f} KB in {dur:.1f}s "
              f"({state['chunks']} chunks)")
        print(f"           throughput: {kb / dur:.2f} KB/s")
        print(f"           => a full 2 MB flash would take "
              f"~{(2048 / (kb / dur)) / 60:.1f} min at this rate")
    print("==============================\n")


# Keep WinRT connection-parameter request objects alive for the whole session:
# Windows withdraws the preferred-parameters request the moment its request
# object is garbage-collected.
_CONN_PARAM_HOLDERS = []


async def request_fast_connection_windows(client, name: str) -> bool:
    """On Windows, ask the OS BLE stack for a ThroughputOptimized connection
    interval (~15 ms) via WinRT RequestPreferredConnectionParameters.

    Windows (bleak/WinRT) otherwise imposes a slow, variable interval no matter
    what the peripheral requests — this is the app-side lever. No-op off Windows.
    Best-effort: any failure is logged and the run continues at the default
    interval. Returns True if the request was issued.
    """
    if sys.platform != "win32":
        return False
    try:
        # Reach the underlying WinRT BluetoothLEDevice that bleak connected with.
        backend = getattr(client, "_backend", client)
        device = None
        for attr in ("_requester", "_device", "_bleak_device"):
            cand = getattr(backend, attr, None)
            if cand is not None and hasattr(
                    cand, "request_preferred_connection_parameters"):
                device = cand
                break
        if device is None:
            print(f"[WIN] {name}: couldn't reach the WinRT device object "
                  f"(bleak internals differ?) — skipping fast-connection request.")
            return False

        from winrt.windows.devices.bluetooth import (
            BluetoothLEPreferredConnectionParameters as Prefs,
        )
        req = device.request_preferred_connection_parameters(
            Prefs.throughput_optimized)
        _CONN_PARAM_HOLDERS.append(req)   # keep alive or Windows reverts
        print(f"[WIN] {name}: requested ThroughputOptimized "
              f"(status={getattr(req, 'status', '?')}).")
        return True
    except ImportError:
        print(f"[WIN] {name}: winrt Bluetooth projection missing — "
              f"`pip install winrt-Windows.Devices.Bluetooth` (or update bleak). "
              f"Skipping fast-connection request.")
    except Exception as exc:  # noqa: BLE001
        print(f"[WIN] {name}: fast-connection request failed ({exc}). "
              f"Continuing at the default interval.")
    return False


async def run(count: int, duration: float, interval: float,
              scan_timeout: float, offload: bool = False) -> None:
    devices = await scan(count, scan_timeout)

    nodes = []
    try:
        for d in devices:
            client = BleakClient(d)
            print(f"[CONNECT] {d.name} ...", end=" ", flush=True)
            await client.connect()
            print("OK" if client.is_connected else "FAILED")
            nodes.append(Node(d.name or d.address, d.address, client))

        connected = [n for n in nodes if n.client.is_connected]
        if len(connected) < 1:
            raise SystemExit("[CONNECT] No nodes connected.")
        print(f"[CONNECT] {len(connected)}/{len(nodes)} node(s) connected "
              f"simultaneously.\n")

        # Windows only: demand a fast connection interval from the OS stack.
        issued = False
        for node in connected:
            issued |= await request_fast_connection_windows(node.client,
                                                            node.name)
        if issued:
            await asyncio.sleep(1.5)   # let Windows renegotiate before sampling
            print()

        if offload:
            # Throughput mode: measure log offload on the first node, then stop.
            await measure_offload(connected[0])
            return

        # Status BEFORE sync — the 'synced' flag here is leftover RAM state from
        # each node's prior session (timeSynced is not persisted and resets on
        # every boot), NOT the result of this run. Expect it to be inconsistent.
        print("[STATUS] before sync (stale — pre-sync snapshot):")
        for node in connected:
            await read_status(node)
        print()

        # Sync all nodes as close together as we can.
        for node in connected:
            await sync_node(node)
        print()

        # Status AFTER sync — every connected node should now report synced=True.
        # This is the authoritative check that the sync command took on each one.
        print("[STATUS] after sync (every node should read synced=True):")
        all_synced = True
        for node in connected:
            synced = await read_status(node)
            if synced is False:
                all_synced = False
        print("[SYNC] all nodes confirmed synced." if all_synced
              else "[SYNC] WARNING: a node did NOT report synced=True after sync.")
        print()

        # Sample loop: round-robin reads of A005 across all nodes.
        deadline = time.time() + duration
        round_idx = 0
        while time.time() < deadline:
            for node in connected:
                if not node.client.is_connected:
                    continue
                try:
                    node.samples.append(await read_syncinfo(node))
                except Exception as exc:  # noqa: BLE001
                    print(f"[READ] {node.name}: {exc}")
            round_idx += 1
            if round_idx % 10 == 0:
                print(f"[SAMPLE] round {round_idx} "
                      f"({int(deadline - time.time())}s left)")
            await asyncio.sleep(interval)

        report_offsets(connected)

    finally:
        for node in nodes:
            try:
                if node.client.is_connected:
                    await node.client.disconnect()
                    print(f"[DISCONNECT] {node.name}")
            except Exception:  # noqa: BLE001
                pass


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--count", type=int, default=2,
                    help="number of nodes to connect (default 2)")
    ap.add_argument("--duration", type=float, default=60.0,
                    help="offset-sampling duration in seconds (default 60)")
    ap.add_argument("--interval", type=float, default=0.5,
                    help="delay between sampling rounds in seconds (default 0.5)")
    ap.add_argument("--scan-timeout", type=float, default=8.0,
                    help="BLE scan timeout in seconds (default 8)")
    ap.add_argument("--offload", action="store_true",
                    help="measure log-offload throughput on one node (Req 2) "
                         "instead of the sync-offset test")
    args = ap.parse_args()

    try:
        asyncio.run(run(args.count, args.duration, args.interval,
                        args.scan_timeout, args.offload))
    except KeyboardInterrupt:
        print("\n[ABORT] Interrupted.")


if __name__ == "__main__":
    main()
