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

This is a laptop-side test tool — no phone app required. BLE actions need
`bleak` (pip install bleak); the offline `enroll` bookkeeping and `selftest` do
not.

Subcommands (preferred)
-----------------------
    python tools/multinode_test.py enroll  --segments upper_arm_r,forearm_r [--erase] [--reuse]
    python tools/multinode_test.py check   --count 2 --duration 60     # sync + offset/drift
    python tools/multinode_test.py erase   --count 2                   # wipe flash (destructive)
    python tools/multinode_test.py offload --count 2 --out-dir ./capture [--erase-after]
    python tools/multinode_test.py selftest                            # offline logic, no hardware

`enroll` maps each board to a body segment by powering ONE node at a time (its
permanent HULC-IMU-XXXX id is unambiguous when it's the only one advertising)
and writes montage.json + nodes_registry.json — no firmware change or streaming.

The legacy flag form still works as a deprecated alias:
    --count/--duration (check) · --erase · --offload/--out-dir/--erase-after-offload

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

from __future__ import annotations   # lazy annotations: Node can name BleakClient
                                      # without importing bleak at module load
import argparse
import asyncio
import json
import os
import re
import statistics
import struct
import sys
import time
from dataclasses import dataclass, field

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from motion_capabilities import (  # noqa: E402
    SEGMENTS, SEGMENT_CODES, SEGMENT_CONFIG_TAG, SEGMENT_UNASSIGNED,
    segment_from_header, validate_montage,
)

# bleak is imported lazily (see _ensure_bleak) so the offline paths — `enroll`
# montage/registry logic and `selftest` — run without the dependency or hardware.
BleakClient = None
BleakScanner = None


def _ensure_bleak():
    """Populate the BleakClient/BleakScanner globals on first BLE use."""
    global BleakClient, BleakScanner
    if BleakClient is None:
        try:
            from bleak import BleakClient as _C, BleakScanner as _S
        except ImportError:  # pragma: no cover - dependency hint
            raise SystemExit("This harness needs bleak:  pip install bleak")
        BleakClient, BleakScanner = _C, _S

NAME_PREFIX = "HULC-IMU"
UUID_CONTROL = "A0010002-B0CE-4A4A-8F0B-0011223344FF"
UUID_STATUS = "A0010003-B0CE-4A4A-8F0B-0011223344FF"
UUID_OFFLOAD = "A0010004-B0CE-4A4A-8F0B-0011223344FF"
UUID_SYNCINFO = "A0010005-B0CE-4A4A-8F0B-0011223344FF"

CMD_SYNC_MS = 0x05
CMD_OFFLOAD = 0x04
CMD_ERASE = 0x03
CMD_OFFLOAD_RANGE = 0x06   # [0x06, offset u32 LE, length u32 LE] — resend one byte range
CMD_SET_SEGMENT = 0x07     # [0x07, segment code 0..6] — assign this node's body part

# Offload framing (must match firmware OFFLOAD_* defines): every A004
# notification is [4-byte LE offset][payload]; the header notification uses a
# sentinel offset and carries the exact total log length. Placing payloads by
# offset makes a dropped (unacknowledged) notification a locatable hole rather
# than a silent, gap-collapsing loss.
OFFLOAD_OFFSET_HDR = 0xFFFFFFFF   # sentinel offset: payload is the 4-byte total length
OFFLOAD_MAX_ATTEMPTS = 4          # full pass + range re-requests, until complete
RECORD_SIZE = 20                  # bytes per quaternion record


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


async def scan(count: int, timeout: float, name_filter: str = None) -> list:
    print(f"[SCAN] Looking for {count} '{NAME_PREFIX}-*' node(s) "
          f"({timeout:.0f}s)"
          f"{' matching ' + name_filter if name_filter else ''}...")
    devices = await BleakScanner.discover(timeout=timeout)
    found = []
    for d in devices:
        name = d.name or ""
        if name.startswith(NAME_PREFIX):
            found.append(d)
            print(f"[SCAN]   found {name}  ({d.address})")
    if name_filter:
        nf = name_filter.lower()
        found = [d for d in found if nf in (d.name or "").lower()]
        if not found:
            raise SystemExit(f"[SCAN] No node matching '{name_filter}' found.")
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
        print("  measurement. Treat the offset as an upper bound, not a")
        print("  calibrated value. For true ms-level validation use a shared")
        print("  physical event (see MULTINODE_TESTING.md).")
    print("====================================\n")


async def _read_log_kb(node: Node):
    """Return the reported log size in KB from the status char, or None."""
    try:
        data = bytes(await node.client.read_gatt_char(UUID_STATUS))
    except Exception:  # noqa: BLE001
        return None
    return struct.unpack_from("<H", data, 2)[0] if len(data) >= 4 else None


async def _read_segment(node: Node):
    """Return the node's assigned segment NAME from the status char.

    Status bytes 4-5 carry (config-tag, segment code); an older firmware sends a
    4-byte status and reads as unassigned. Returns None when the node reports no
    valid assignment (never guessed) — the caller treats that as unknown.
    """
    try:
        data = bytes(await node.client.read_gatt_char(UUID_STATUS))
    except Exception:  # noqa: BLE001
        return None
    if len(data) < 6:
        return None
    return segment_from_header(data[4], data[5])


async def set_segment(node: Node, segment: str) -> bool:
    """Assign a body segment to the node (control 0x07), then verify the readback.

    `segment` is a name from motion_capabilities.SEGMENTS; it is mapped to the
    shared uint8 code. Config-level and persisted in the node's log header, so it
    survives power cycles and log erases. Returns True once the node reports the
    expected segment back on the status char.
    """
    code = SEGMENT_CODES.get(segment)
    if code is None:
        print(f"[SEGMENT] {node.name}: unknown segment '{segment}' — not set.")
        return False
    try:
        await node.client.write_gatt_char(
            UUID_CONTROL, bytes([CMD_SET_SEGMENT, code]), response=True)
    except Exception as exc:  # noqa: BLE001
        print(f"[SEGMENT] {node.name}: write failed ({exc})")
        return False
    back = await _read_segment(node)
    if back == segment:
        print(f"[SEGMENT] {node.name}: set to '{segment}' (code {code}) — confirmed.")
        return True
    print(f"[SEGMENT] {node.name}: set '{segment}' but readback is "
          f"{back!r} — check the node is running updated firmware.")
    return False


async def erase_node(node: Node, wait_s: float = 40.0) -> None:
    """Erase one node's flash log (control 0x03). Wipes ALL logged data.

    eraseLog() blocks the firmware ~30s; the nRF SoftDevice keeps the link up.
    We send the command, wait, then confirm the reported log size dropped to 0.
    The firmware only resets its write pointer after a verified-blank erase, so
    a log that stays non-zero means the wipe did NOT take — check the node's USB
    serial for the '[QSPI] ... FAILED/timeout' line, and confirm the node is
    actually running the updated firmware.

    NOTE: this MUST be a write-WITH-response. The control characteristic is
    declared BLEWrite (Write Request) only, so a Write Command (response=False)
    is silently dropped at the ATT layer — ctrlChar.written() never fires and
    eraseLog() never runs. Sync (0x02) and offload (0x04) already use
    response=True; erase was the odd one out, which is why it never erased.
    """
    before_kb = await _read_log_kb(node)
    await read_status(node)  # show current log size before wiping
    print(f"[ERASE] {node.name}: sending erase (0x03) — this WIPES all logged "
          f"data on this node...")
    try:
        await node.client.write_gatt_char(UUID_CONTROL, bytes([CMD_ERASE]),
                                          response=True)
    except Exception as exc:  # noqa: BLE001
        print(f"[ERASE] {node.name}: write failed ({exc})")
        return
    print(f"[ERASE] {node.name}: erasing (~30s, do NOT power off)...")
    await asyncio.sleep(wait_s)
    await read_status(node)  # should now show log=0KB
    after_kb = await _read_log_kb(node)
    if after_kb is None:
        print(f"[ERASE] {node.name}: could not read status back — reconnect to "
              f"verify (expected log=0KB).")
    elif after_kb == 0:
        print(f"[ERASE] {node.name}: OK — log is now 0KB (wipe confirmed).")
    else:
        print(f"[ERASE] {node.name}: FAILED — log still {after_kb}KB"
              f"{'' if before_kb is None else f' (was {before_kb}KB)'}. Firmware "
              f"did not confirm a blank erase. Check the node's USB serial for a "
              f"'[QSPI] ... FAILED/timeout' line, and confirm the node is running "
              f"the updated firmware.")


def erase_decision(kb, assume_yes: bool, interactive: bool) -> str:
    """Decide what to do for a node reporting `kb` KB of log.

    'skip' — already empty (0KB), nothing to wipe.
    'wipe' — has data (or unknown) and confirmation is assumed.
    'ask'  — has data and we should prompt [y/N].
    'hold' — has data but no way to confirm; leave it (never wipe blind).
    """
    if kb == 0:
        return "skip"
    if assume_yes:
        return "wipe"
    return "ask" if interactive else "hold"


async def smart_erase(node: Node, ask=None, assume_yes: bool = False) -> None:
    """Erase a node's flash only if it holds data, over the OPEN connection.

    Reads the log size first: if it's already 0KB, skip the ~30s wipe entirely
    (just the status read). Otherwise confirm before wiping — `assume_yes` wipes
    without asking; an interactive `ask` coroutine prompts [y/N]; with neither, a
    node holding data is left untouched (safe default) rather than wiped blind.
    """
    kb = await _read_log_kb(node)
    size = "an unknown amount" if kb is None else f"{kb}KB"
    action = erase_decision(kb, assume_yes, ask is not None)
    if action == "skip":
        print(f"[ERASE] {node.name}: flash already empty (0KB) — skipping.")
        return
    if action == "hold":
        print(f"[ERASE] {node.name}: holds {size}; pass --yes to wipe. "
              f"Left in place.")
        return
    if action == "ask":
        ans = (await ask(f"[ERASE] {node.name} holds {size}. Erase now? [y/N] ")
               ).strip().lower()
        if ans != "y":
            print(f"[ERASE] {node.name}: left {size} in place.")
            return
    await erase_node(node)


def _missing_ranges(received: dict, total: int):
    """Given {offset: payload} and the expected total, return the list of
    (start, length) byte ranges that were never received."""
    holes = []
    cursor = 0
    for off in sorted(received):
        if off > cursor:
            holes.append((cursor, off - cursor))
        cursor = max(cursor, off + len(received[off]))
    if total is not None and cursor < total:
        holes.append((cursor, total - cursor))
    return holes


async def _offload_pass(node: Node, received: dict, quiet_s: float, max_s: float,
                        cmd: bytes = None):
    """Run one A004 offload pass, sending control `cmd` (default: full offload).
    Frames are [4B LE offset][payload]; payloads are merged into `received` by
    offset (so passes / range-resends fill each other's holes). Returns
    (expected_total_or_None, stats_dict)."""
    if cmd is None:
        cmd = bytes([CMD_OFFLOAD])
    st = {"bytes": 0, "chunks": 0, "new": 0, "first": None, "last": None,
          "total": None}

    def on_chunk(_char, data: bytearray) -> None:
        b = bytes(data)
        if len(b) < 4:
            return
        now = time.monotonic()
        if st["first"] is None:
            st["first"] = now
        st["last"] = now
        st["chunks"] += 1
        off = int.from_bytes(b[:4], "little")
        if off == OFFLOAD_OFFSET_HDR:            # header: exact total length
            if len(b) >= 8:
                st["total"] = int.from_bytes(b[4:8], "little")
            return
        payload = b[4:]
        if not payload:
            return
        st["bytes"] += len(payload)
        if off not in received:                  # merge; ignore duplicates
            received[off] = payload
            st["new"] += len(payload)

    await node.client.start_notify(UUID_OFFLOAD, on_chunk)
    await node.client.write_gatt_char(UUID_CONTROL, cmd, response=True)
    t0 = time.monotonic()
    no_data_timeout = 8.0    # if nothing ever arrives, the log is empty
    while True:
        await asyncio.sleep(0.5)
        elapsed = time.monotonic() - t0
        last = st["last"]
        if last is None and elapsed > no_data_timeout:
            break
        if last is not None and (time.monotonic() - last) > quiet_s:
            break
        if elapsed > max_s:
            print(f"[OFFLOAD] {node.name}: hit max {max_s:.0f}s cap.")
            break
    try:
        await node.client.stop_notify(UUID_OFFLOAD)
    except Exception:  # noqa: BLE001
        pass
    return st["total"], st


async def measure_offload(node: Node, quiet_s: float = 3.0,
                          max_s: float = 300.0, save_path: str = None,
                          max_attempts: int = OFFLOAD_MAX_ATTEMPTS) -> None:
    """Offload one node's flash log with drop-detection and recovery.

    A004 notifications are unacknowledged, so a notification the central drops
    silently erases a run of records from the middle of the file. Each
    notification is framed [4-byte LE offset][payload] with a header giving the
    exact total, so we place payloads by offset and know precisely what is
    missing. Recovery: the first pass is a full offload; then we re-request only
    the missing byte ranges (control 0x06). A whole-log re-offload recreates the
    same fast burst and so tends to drop the SAME chunks every pass; a small
    targeted resend does not, so it recovers the deterministic holes the naive
    retry could not. The node does not erase until it receives 0x03, so this is
    safe to repeat. Only a verified-complete log is written to save_path.
    """
    await read_status(node)  # prints log size / IDLE state for context
    print(f"[OFFLOAD] {node.name}: starting — subscribing to A004...")

    received: dict = {}
    expected_total = None

    # Pass 1: full offload (also delivers the header total).
    total, full_stats = await _offload_pass(node, received, quiet_s, max_s)
    if full_stats["first"] is None:
        print(f"[OFFLOAD] {node.name}: no data — the node's flash log is "
              f"empty (nothing recorded), or it is not in IDLE.")
        return
    if total is not None:
        expected_total = total
    have = sum(len(v) for v in received.values())
    holes = _missing_ranges(received, expected_total)
    print(f"[OFFLOAD] {node.name}: pass 1 (full): {have}"
          f"{'/' + str(expected_total) if expected_total else ''} bytes, "
          f"{len(holes)} hole(s)")

    # Recovery passes: re-request only the missing ranges.
    attempt = 1
    while attempt < max_attempts and holes:
        attempt += 1
        if expected_total is None:
            # No header yet — can't target ranges; fall back to a full re-offload.
            total, full_stats = await _offload_pass(node, received, quiet_s, max_s)
            if total is not None:
                expected_total = total
        else:
            print(f"[OFFLOAD] {node.name}: pass {attempt}: re-requesting "
                  f"{len(holes)} range(s)...")
            for start, length in holes:
                cmd = struct.pack("<BII", CMD_OFFLOAD_RANGE, start, length)
                await _offload_pass(node, received, quiet_s, max_s, cmd=cmd)
        have = sum(len(v) for v in received.values())
        holes = _missing_ranges(received, expected_total)
        miss = sum(n for _, n in holes)
        print(f"[OFFLOAD] {node.name}: after pass {attempt}: "
              f"{have}/{expected_total if expected_total else '?'} bytes, "
              f"{miss} missing / {miss // RECORD_SIZE} records")

    complete = expected_total is not None and not holes

    print("\n===== OFFLOAD THROUGHPUT =====")
    if full_stats and full_stats["first"] is not None and full_stats["bytes"]:
        dur = max(1e-3, full_stats["last"] - full_stats["first"])
        kb = full_stats["bytes"] / 1024
        print(f"[{node.name}] full pass: {kb:.1f} KB in {dur:.1f}s "
              f"({full_stats['chunks']} chunks), {kb / dur:.2f} KB/s")
        if kb / dur > 0:
            print(f"           => a full 2 MB flash would take "
                  f"~{(2048 / (kb / dur)) / 60:.1f} min at this rate")

    if save_path and received:
        # Assemble by offset. Holes are left as zeroed bytes: they decode to
        # zero-norm quaternions that reconcile_nodes.py drops, so survivors keep
        # their true timestamps instead of collapsing together into a fake,
        # undetectable gap.
        size = expected_total if expected_total is not None else (
            max(off + len(p) for off, p in received.items()))
        buf = bytearray(size)
        for off, payload in received.items():
            buf[off:off + len(payload)] = payload
        with open(save_path, "wb") as f:
            f.write(buf)
        # Sidecar: record the node's self-reported segment next to the .bin so the
        # capture dir is self-describing on disk (survives without the node) and
        # analyze can cross-check it against the montage. Best-effort — a node on
        # older firmware simply reports no segment and no sidecar is written.
        seg = await _read_segment(node)
        if seg is not None:
            side_path = os.path.splitext(save_path)[0] + ".seg.json"
            with open(side_path, "w", encoding="utf-8", newline="\n") as f:
                json.dump({"node_id": node.name, "segment": seg,
                           "code": SEGMENT_CODES[seg]}, f, indent=2)
                f.write("\n")
            print(f"           segment: {seg} -> {side_path}")
        recs = size // RECORD_SIZE
        if complete:
            print(f"           COMPLETE — saved {size} bytes ({recs} records) "
                  f"-> {save_path}")
        else:
            miss = sum(n for _, n in holes)
            print(f"           [!] INCOMPLETE after {max_attempts} attempts — "
                  f"{miss} bytes / {miss // RECORD_SIZE} records still missing "
                  f"at offsets {[(o, n) for o, n in holes]}.")
            print(f"           Saved {size} bytes with holes zero-filled "
                  f"(reconcile drops them) -> {save_path}. Re-run --offload to "
                  f"recover the rest (the node has NOT erased its log).")
    elif not received:
        print(f"[{node.name}] no data received. Is there a log to offload "
              f"(status log>0KB) and is the node in IDLE?")
    print("==============================\n")
    return complete


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


async def connect_with_retry(device, attempts: int = 3):
    """Connect to one device, retrying transient failures. Returns a connected
    BleakClient, or None if every attempt failed.

    Windows' BLE stack intermittently cancels service discovery while a second
    peripheral is connecting (the CancelledError -> TimeoutError seen when
    connecting several nodes back to back). A short retry with a fresh client
    clears it almost every time, so one flaky connect no longer aborts the run.
    """
    for attempt in range(1, attempts + 1):
        client = BleakClient(device)
        try:
            await client.connect()
            if client.is_connected:
                return client
        except Exception as exc:  # noqa: BLE001
            if attempt < attempts:
                wait = 1.5 * attempt
                print(f"[{type(exc).__name__}, retry {attempt}/{attempts - 1} "
                      f"in {wait:.0f}s]", end=" ", flush=True)
        # Failed this attempt — drop the client cleanly before retrying.
        try:
            await client.disconnect()
        except Exception:  # noqa: BLE001
            pass
        if attempt < attempts:
            await asyncio.sleep(1.5 * attempt)
    return None


async def run(count: int, duration: float, interval: float,
              scan_timeout: float, offload: bool = False,
              offload_save: str = None, name_filter: str = None,
              offload_dir: str = None, erase: bool = False,
              erase_after: bool = False, erase_yes: bool = False) -> None:
    _ensure_bleak()
    devices = await scan(count, scan_timeout, name_filter)

    nodes = []
    try:
        for d in devices:
            print(f"[CONNECT] {d.name} ...", end=" ", flush=True)
            client = await connect_with_retry(d)
            if client is not None:
                print("OK")
                nodes.append(Node(d.name or d.address, d.address, client))
            else:
                print("FAILED — skipping this node")

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

        if erase:
            # Wipe each connected node's flash, one at a time — but skip nodes
            # that are already empty, and confirm before wiping ones that aren't
            # (unless --yes). Reuses this single connection for all of them.
            loop = asyncio.get_event_loop()

            async def _ask(m):
                return await loop.run_in_executor(None, input, m)

            for node in connected:
                await smart_erase(node, ask=(None if erase_yes else _ask),
                                  assume_yes=erase_yes)
            return

        if offload:
            # Offload EVERY connected node in turn, each to its own .bin named
            # after the node id (e.g. HULC-IMU-D067.bin). --save overrides the
            # filename only when exactly one node is offloaded.
            if offload_dir:
                os.makedirs(offload_dir, exist_ok=True)
            single = offload_save and len(connected) == 1
            if offload_save and len(connected) > 1:
                print(f"[OFFLOAD] --save ignored ({len(connected)} nodes) — "
                      f"auto-naming each file by node id.")
            for node in connected:
                if single:
                    path = offload_save
                else:
                    safe = re.sub(r"[^A-Za-z0-9._-]", "_", node.name)
                    path = os.path.join(offload_dir or ".", f"{safe}.bin")
                complete = await measure_offload(node, save_path=path)
                # Erase ONLY after a verified-complete offload, so we never wipe
                # data we didn't fully receive. This keeps each node's flash to a
                # single session (no reboot-seam / stale-record accumulation).
                if erase_after:
                    if complete:
                        print(f"[OFFLOAD] {node.name}: verified complete — "
                              f"erasing flash so the next capture starts clean.")
                        await erase_node(node)
                    else:
                        print(f"[OFFLOAD] {node.name}: NOT erasing — offload was "
                              f"incomplete. Re-run to recover, or erase manually "
                              f"with --erase once you have the data.")
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


# ---------------------------------------------------------------------------
# Enrollment (power-one-at-a-time) + montage builder
# ---------------------------------------------------------------------------
# Maps each physical board to a body segment with NO firmware change or live
# streaming: power ON one node at a time; with a single board advertising, its
# permanent id (HULC-IMU-XXXX = last 4 hex of MAC) is unambiguous. Writes a
# schema-valid montage.json plus a persistent nodes_registry.json (id -> segment
# + label) so later sessions can --reuse without power-cycling.
DEFAULT_REGISTRY = "nodes_registry.json"
DEFAULT_MONTAGE = "montage.json"


def build_montage(assignments, subject_id="S01", session_id="", notes=""):
    """assignments: ordered [(node_id, segment, label)] -> montage dict.

    Column order follows the list order (n0, n1, ...) — what reconcile and
    analyze_session use to bind logs to segments.
    """
    nodes = [{"node_id": nid, "column": f"n{i}", "segment": seg,
              "landmark": lbl or "", "calibrated": True}
             for i, (nid, seg, lbl) in enumerate(assignments)]
    return {
        "schema_version": "1.0",
        "subject": {"id": subject_id, "notes": notes},
        "session": {"id": session_id, "aligned_csv": "aligned.csv"},
        "calibration": {"neutral_pose": "N-pose", "captured": True,
                        "t_window_ms": [1000, 4000], "functional": []},
        "nodes": nodes,
    }


def load_registry(path):
    if not os.path.exists(path):
        return {}
    with open(path, encoding="utf-8-sig") as f:
        return json.load(f)


def save_registry(path, registry):
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        json.dump(registry, f, indent=2)
        f.write("\n")


def merge_registry(registry, assignments):
    for node_id, segment, label in assignments:
        entry = registry.get(node_id, {})
        entry["segment"] = segment
        if label:
            entry["label"] = label
        registry[node_id] = entry
    return registry


def reuse_from_registry(registry, present_ids, segments):
    """If every requested segment maps to a present, known node, return an
    ordered assignment list matching `segments`; else None."""
    by_seg = {}
    for node_id, entry in registry.items():
        if node_id in present_ids and entry.get("segment"):
            by_seg[entry["segment"]] = node_id
    if all(s in by_seg for s in segments):
        return [(by_seg[s], s, registry[by_seg[s]].get("label", ""))
                for s in segments]
    return None


def write_montage(path, montage):
    errors = validate_montage(montage)
    if errors:
        raise SystemExit("[enroll] built montage failed validation:\n  - " +
                         "\n  - ".join(errors))
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        json.dump(montage, f, indent=2)
        f.write("\n")


def norm_segments(seg_arg):
    segs = [s.strip() for s in seg_arg.split(",") if s.strip()]
    bad = [s for s in segs if s not in SEGMENTS]
    if bad:
        raise SystemExit(f"[enroll] unknown segment(s): {', '.join(bad)}\n"
                         f"valid: {', '.join(SEGMENTS)}")
    if not segs:
        raise SystemExit("[enroll] --segments is empty")
    return segs


async def _scan_names(timeout):
    """Return {name: address} for HULC nodes currently advertising."""
    return {d.name: d.address for d in await BleakScanner.discover(timeout=timeout)
            if (d.name or "").startswith(NAME_PREFIX)}


async def enroll(segments, montage_path, registry_path, reuse, no_erase,
                 subject, session, scan_timeout):
    _ensure_bleak()
    registry = load_registry(registry_path)
    loop = asyncio.get_event_loop()

    async def ask(msg):
        return await loop.run_in_executor(None, input, msg)

    if reuse:
        print("[enroll] --reuse: scanning for all boards...")
        present = await _scan_names(scan_timeout)
        ordered = reuse_from_registry(registry, set(present), segments)
        if ordered:
            print("[enroll] all segments known from a prior enrollment:")
            for nid, seg, lbl in ordered:
                print(f"        {seg:<14} <- {nid}"
                      f"{'  (' + lbl + ')' if lbl else ''}")
            if (await ask("[enroll] reuse this placement? [Y/n] ")).strip().lower() in ("", "y"):
                write_montage(montage_path, build_montage(ordered, subject, session))
                print(f"[enroll] wrote {montage_path} (reused, no power cycling).")
                return
        else:
            print("[enroll] registry doesn't cover all segments — enrolling.")

    assignments, used = [], set()
    for seg in segments:
        while True:
            await ask(f">>> Power ON ONLY the node for '{seg}' (all others OFF), "
                      f"then press Enter... ")
            names = await _scan_names(scan_timeout)
            fresh = {n: a for n, a in names.items() if n not in used}
            if not fresh:
                print("    [!] " + ("seen board(s) already assigned — power OFF the "
                      "enrolled ones, ON only the new one." if names else
                      "no HULC node advertising. Powered on? wait a few seconds.")
                      + "\n")
                continue
            if len(fresh) > 1:
                print(f"    [!] {len(fresh)} nodes advertising: "
                      f"{', '.join(sorted(fresh))}. Power ON only ONE.\n")
                continue
            node_id, address = next(iter(fresh.items()))
            lbl = (await ask(f"    found {node_id}. Physical label "
                             f"(e.g. 'orange tape', optional): ")).strip()
            # One connection does the config work: smart-erase (unless suppressed)
            # AND stamp the segment into the node's log header, so no separate
            # connect is needed and the offloaded log becomes self-describing.
            # The segment write is not gated by erase — it's independent config.
            client = await connect_with_retry(address)
            if client is None:
                print(f"    [!] couldn't connect to {node_id} — enrolled in the "
                      f"montage anyway, but its segment was NOT written to the "
                      f"node; re-run enroll or set it later.")
            else:
                node = Node(name=node_id, address=address, client=client)
                try:
                    if not no_erase:
                        # Check the log and only wipe (with a y/N confirm) if the
                        # node holds data — no 30s wipe on an already-empty node.
                        await smart_erase(node, ask=ask)
                    await set_segment(node, seg)
                finally:
                    await client.disconnect()
            assignments.append((node_id, seg, lbl))
            used.add(node_id)
            print(f"    -> {seg} = {node_id}\n")
            break

    write_montage(montage_path, build_montage(assignments, subject, session))
    save_registry(registry_path, merge_registry(registry, assignments))
    print(f"[enroll] wrote {montage_path} and updated {registry_path}.")
    print("[enroll] label your boards now so you can --reuse next time.")


async def read_segments(montage_path, registry_path, subject, session,
                        scan_timeout):
    """Build a montage by reading each node's OWN segment assignment.

    The §2.2 "user only confirms" flow: once nodes have been enrolled (their
    segment written to the header), the montage no longer has to be hand-typed —
    connect to whatever is advertising, read each node's self-reported segment,
    and assemble the montage from that. Column order follows the canonical segment
    code so the result is deterministic. Nodes reporting no assignment are listed
    and skipped (never guessed).
    """
    _ensure_bleak()
    registry = load_registry(registry_path)
    loop = asyncio.get_event_loop()

    async def ask(msg):
        return await loop.run_in_executor(None, input, msg)

    print("[from-nodes] scanning for advertising boards...")
    present = await _scan_names(scan_timeout)
    if not present:
        raise SystemExit("[from-nodes] no HULC node advertising.")

    found, unassigned = {}, []
    for node_id, address in present.items():
        client = await connect_with_retry(address)
        if client is None:
            print(f"    [!] couldn't connect to {node_id} — skipped.")
            continue
        try:
            seg = await _read_segment(Node(name=node_id, address=address,
                                           client=client))
        finally:
            await client.disconnect()
        if seg is None:
            unassigned.append(node_id)
            print(f"    {node_id}: no segment assigned — enroll it first.")
        elif seg in found:
            print(f"    [!] {seg} already read from {found[seg]}; {node_id} also "
                  f"reports it — skipping the duplicate.")
        else:
            found[seg] = node_id
            print(f"    {node_id}: {seg}")

    if not found:
        raise SystemExit("[from-nodes] no node reported a segment — run enroll.")
    if unassigned:
        print(f"[from-nodes] {len(unassigned)} node(s) had no assignment: "
              f"{', '.join(unassigned)}")

    # Deterministic column order: canonical segment code.
    ordered = sorted(found.items(), key=lambda kv: SEGMENT_CODES[kv[0]])
    assignments = [(nid, seg, registry.get(nid, {}).get("label", ""))
                   for seg, nid in ordered]
    print("[from-nodes] montage from node headers:")
    for nid, seg, lbl in assignments:
        print(f"        {seg:<14} <- {nid}{'  (' + lbl + ')' if lbl else ''}")
    if (await ask("[from-nodes] write this montage? [Y/n] ")
            ).strip().lower() not in ("", "y"):
        print("[from-nodes] aborted; montage not written.")
        return
    write_montage(montage_path, build_montage(assignments, subject, session))
    save_registry(registry_path, merge_registry(registry, assignments))
    print(f"[from-nodes] wrote {montage_path} and updated {registry_path}.")


def enroll_selftest():
    ok = True

    def check(cond, msg):
        nonlocal ok
        ok = ok and cond
        print(f"[selftest] {'ok ' if cond else 'FAIL'}: {msg}")

    asg = [("HULC-IMU-485C", "upper_arm_r", "orange"),
           ("HULC-IMU-B059", "forearm_r", "blue")]
    m = build_montage(asg, subject_id="S01", session_id="t")
    check(validate_montage(m) == [], "built montage validates clean")
    check([n["column"] for n in m["nodes"]] == ["n0", "n1"], "columns n0,n1 in order")
    check(m["nodes"][0]["segment"] == "upper_arm_r", "n0 -> upper_arm_r")

    reg = merge_registry({}, asg)
    check(reg["HULC-IMU-485C"]["label"] == "orange", "registry stores label")
    reused = reuse_from_registry(reg, {"HULC-IMU-485C", "HULC-IMU-B059"},
                                 ["upper_arm_r", "forearm_r"])
    check(reused is not None and [a[0] for a in reused] ==
          ["HULC-IMU-485C", "HULC-IMU-B059"], "reuse reconstructs ordered assignment")
    check(reuse_from_registry(reg, {"HULC-IMU-485C"},
                              ["upper_arm_r", "forearm_r"]) is None,
          "reuse aborts when a node is absent")
    try:
        norm_segments("upper_arm_r,not_a_segment")
        check(False, "bad segment should raise")
    except SystemExit:
        check(True, "unknown segment name rejected")

    # smart-erase decision: skip empty, confirm data, never wipe blind
    check(erase_decision(0, False, True) == "skip", "empty node -> skip erase")
    check(erase_decision(0, True, False) == "skip", "empty node -> skip even with --yes")
    check(erase_decision(12, False, True) == "ask", "data + interactive -> prompt")
    check(erase_decision(12, True, False) == "wipe", "data + --yes -> wipe")
    check(erase_decision(12, False, False) == "hold", "data, no confirm path -> hold")
    check(erase_decision(None, False, True) == "ask", "unknown size -> prompt, not blind wipe")

    print(f"\n[selftest] {'PASS' if ok else 'FAIL'} — montage build/validate, "
          f"registry reuse, and segment validation.")
    return 0 if ok else 1


# ---------------------------------------------------------------------------
# CLI: unified subcommands, with the legacy flag form kept as a back-compat alias
# ---------------------------------------------------------------------------
_SUBCOMMANDS = {"check", "erase", "offload", "enroll", "selftest"}


def _main_sub(argv) -> None:
    ap = argparse.ArgumentParser(
        prog="multinode_test.py", description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)

    def add_common(p):
        p.add_argument("--count", type=int, default=2, help="nodes to connect")
        p.add_argument("--scan-timeout", type=float, default=8.0)
        p.add_argument("--name", metavar="SUFFIX",
                       help="only the node whose name contains SUFFIX")

    pc = sub.add_parser("check", help="sync + cross-node offset/drift report")
    add_common(pc)
    pc.add_argument("--duration", type=float, default=60.0)
    pc.add_argument("--interval", type=float, default=0.5)

    pe = sub.add_parser("erase", help="wipe each node's flash if it holds data")
    add_common(pe)
    pe.add_argument("--yes", action="store_true",
                    help="wipe without the per-node y/N confirmation")

    po = sub.add_parser("offload", help="offload each node's flash log")
    add_common(po)
    po.add_argument("--out-dir", metavar="DIR", default=".")
    po.add_argument("--save", metavar="PATH", help="single-node: exact .bin path")
    po.add_argument("--erase-after", action="store_true",
                    help="wipe each node after its offload verifies COMPLETE")

    pn = sub.add_parser("enroll", help="power-one-at-a-time -> montage.json")
    pn.add_argument("--segments", required=True,
                    help="comma-separated segments in placement order")
    pn.add_argument("--montage", default=DEFAULT_MONTAGE)
    pn.add_argument("--registry", default=DEFAULT_REGISTRY)
    pn.add_argument("--reuse", action="store_true")
    pn.add_argument("--no-erase", action="store_true",
                    help="skip the smart-erase check on the enroll connection "
                         "(the segment is still written to each node)")
    pn.add_argument("--subject", default="S01")
    pn.add_argument("--session", default="")
    pn.add_argument("--scan-timeout", type=float, default=8.0)

    pf = sub.add_parser("read-segments",
                        help="build montage.json from nodes' own segment headers")
    pf.add_argument("--montage", default=DEFAULT_MONTAGE)
    pf.add_argument("--registry", default=DEFAULT_REGISTRY)
    pf.add_argument("--subject", default="S01")
    pf.add_argument("--session", default="")
    pf.add_argument("--scan-timeout", type=float, default=8.0)

    sub.add_parser("selftest", help="validate the offline enroll logic")

    args = ap.parse_args(argv)
    if args.cmd == "selftest":
        sys.exit(enroll_selftest())
    if args.cmd == "enroll":
        segs = norm_segments(args.segments)
        asyncio.run(enroll(segs, args.montage, args.registry, args.reuse,
                           args.no_erase, args.subject, args.session,
                           args.scan_timeout))
        return
    if args.cmd == "read-segments":
        asyncio.run(read_segments(args.montage, args.registry, args.subject,
                                  args.session, args.scan_timeout))
        return
    try:
        if args.cmd == "check":
            asyncio.run(run(args.count, args.duration, args.interval,
                            args.scan_timeout, name_filter=args.name))
        elif args.cmd == "erase":
            asyncio.run(run(args.count, 0, 0, args.scan_timeout,
                            name_filter=args.name, erase=True,
                            erase_yes=args.yes))
        elif args.cmd == "offload":
            asyncio.run(run(args.count, 0, 0, args.scan_timeout, offload=True,
                            offload_save=args.save, name_filter=args.name,
                            offload_dir=args.out_dir, erase_after=args.erase_after))
    except KeyboardInterrupt:
        print("\n[ABORT] Interrupted.")


def main() -> None:
    argv = sys.argv[1:]
    if argv and argv[0] in _SUBCOMMANDS:
        _main_sub(argv)
        return
    # ---- legacy flag form (deprecated but still supported) ----
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
                    help="offload one node's flash log (measure throughput) "
                         "instead of the sync-offset test")
    ap.add_argument("--save", metavar="PATH",
                    help="with --offload and a SINGLE node: write the log to this "
                         "exact .bin path (overrides the auto name)")
    ap.add_argument("--out-dir", metavar="DIR", default=".",
                    help="with --offload: directory for the auto-named per-node "
                         ".bin files (default: current dir)")
    ap.add_argument("--name", metavar="SUFFIX",
                    help="only connect to the node whose name contains SUFFIX "
                         "(e.g. --name D067). Use with --offload/--erase to pick "
                         "exactly which node.")
    ap.add_argument("--erase", action="store_true",
                    help="WIPE each connected node's flash log (control 0x03), "
                         "then exit. Use before a clean capture. Destructive.")
    ap.add_argument("--erase-after-offload", action="store_true",
                    help="with --offload: after each node's offload is verified "
                         "COMPLETE, wipe its flash so the next capture starts "
                         "clean (avoids reboot-seam / stale-record buildup). "
                         "An incomplete offload is never erased.")
    args = ap.parse_args()
    if args.save and not args.offload:
        ap.error("--save requires --offload")
    if args.erase and args.offload:
        ap.error("--erase and --offload are separate steps; run them separately")
    if args.erase_after_offload and not args.offload:
        ap.error("--erase-after-offload requires --offload")
    hint = ("erase" if args.erase else "offload" if args.offload else "check")
    print(f"[note] the flag form is deprecated; prefer: "
          f"multinode_test.py {hint} ...\n", file=sys.stderr)

    try:
        asyncio.run(run(args.count, args.duration, args.interval,
                        args.scan_timeout, args.offload, args.save, args.name,
                        args.out_dir, args.erase, args.erase_after_offload,
                        erase_yes=args.erase))   # legacy --erase = unconditional
    except KeyboardInterrupt:
        print("\n[ABORT] Interrupted.")


if __name__ == "__main__":
    main()
