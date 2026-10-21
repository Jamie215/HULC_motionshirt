# BLE protocol — talking to a node

What a central (the laptop today, the mobile app later) needs to enroll a
node, get its log off, and write it as a capture folder
([`DATA_FORMAT.md`](DATA_FORMAT.md)). The firmware is the source of truth
(`firmware/firmware.ino`: the header comment and `handleControl()`); the
working reference implementation is `tools/multinode_test.py` (Python,
`bleak`), named per step below.

## Finding a node

- Each node advertises as **`HULC-IMU-XXXX`**, the last 4 hex digits of its BLE
  MAC: unique and stable per board. This is the `node_id` of the capture format.
- It advertises **only in IDLE** (once per second), and stops while a central is
  connected. A node goes IDLE ~8 s after it is laid flat (e.g. on the charger),
  or after 60 s without motion. Recording happens with no central connected.
- Reference: `scan()`, `connect_with_retry()`.

## GATT

Service `A0010000-B0CE-4A4A-8F0B-0011223344FF`; the characteristics share its
suffix:

| UUID | Name | Access | Size | Use |
|---|---|---|---|---|
| `A0010001-…` | Quaternion stream | NOTIFY | 20 B | debug only (a live record, when streaming is on) |
| `A0010002-…` | Control | WRITE | ≤ 9 B | commands, below |
| `A0010003-…` | Status | READ | 6 B | state, log size, segment |
| `A0010004-…` | Log offload | NOTIFY | ≤ 200 B | the log, framed (below) |
| `A0010005-…` | Time Info | READ | 16 B | clock-offset measurement (optional) |

### Status (A0010003), 6 bytes

| Byte | Field |
|---|---|
| 0 | state: 0 IDLE, 1 STATIC_POSTURE, 2 ACTIVE_RECORDING |
| 1 | flags: bit 0 streaming, bit 1 time-synced |
| 2–3 | log size in KB (`uint16` LE) |
| 4 | segment config tag: `0x01` when byte 5 is a valid assignment |
| 5 | segment code (0 torso … 6 hand_r, `0xFF` unassigned; the table in [`DATA_FORMAT.md`](DATA_FORMAT.md) §3) |

Reference: `read_status()`, `_read_segment()`.

### Time Info (A0010005), 16 bytes, little-endian

| Bytes | Field |
|---|---|
| 0–3 | node `millis()` now |
| 4–11 | sync epoch in ms (`uint64`, 0 if never synced) |
| 12–15 | node `millis()` at the last sync |

Not needed for the analysis (it aligns the clocks from the motion); useful to
check a link. Reference: `read_syncinfo()`, `report_offsets()`.

### Control (A0010002)

| Command | Payload | Does | When |
|---|---|---|---|
| `0x00` / `0x01` | — | stop / start the debug quaternion stream | any time |
| `0x02` | `uint32` LE Unix seconds | time sync (seconds, legacy) | any time |
| `0x05` | `uint64` LE Unix ms | time sync (ms) | any time |
| `0x03` | — | erase the log (whole chip, verified blank; ~30 s) | after a verified offload |
| `0x04` | — | start the offload over A0010004 | **IDLE only** (ignored otherwise) |
| `0x06` | `uint32` LE offset, `uint32` LE length | re-send that byte range of the log | **IDLE only** |
| `0x07` | `uint8` segment code 0–6 | store this node's segment in its flash header (survives power cycles and erases) | enroll |

The command byte comes first, e.g. `[0x06, off0, off1, off2, off3, len0, len1, len2, len3]`.

## Getting the log off (offload)

1. Connect to a node in IDLE (status byte 0 = 0). Subscribe to A0010004.
   A fast connection interval helps throughput; the node requests 15–30 ms.
2. Write `0x04`. The node first sends a **header notification**:
   `[0xFFFFFFFF (uint32 LE)][total length (uint32 LE)]` — the exact byte count
   of the log.
3. Then **data notifications**: `[offset (uint32 LE)][up to 196 bytes]`, the
   offset being where the payload sits in the log. Place each payload at its
   offset.
4. Notifications are unacknowledged and some get dropped. When the stream goes
   quiet (~3 s), compute the **missing ranges** against the total and request
   each with `0x06`; repeat until complete (the laptop tries up to 4 passes).
   Reference: `measure_offload()`, `_offload_pass()`, `_missing_ranges()`.
5. Write the bytes as the node's `.bin`, unchanged; any range still missing
   stays **zero-filled** at its offset (never close the gap). Read the status
   bytes 4–5 and write the `.seg.json` header next to it (`node_id`,
   `segment`, `code`). Together these make the capture folder.
6. Only after the transfer is verified complete (bytes received = total), and
   preferably after the folder has been checked (`hulc-ms validate`), write
   `0x03` to erase. The node keeps the log until then, so a failed transfer can
   simply be repeated.

Throughput is set by the central's connection interval: iOS, Android and Linux
(BlueZ) are fast; a Windows desktop central is slow and drops more.

## Enrolling a node (once per board)

Power on one node at a time, connect, write `[0x07, code]` for its segment, and
read the status back to confirm (bytes 4–5). The assignment stays on the node,
so every later offload can write the `.seg.json` from the node itself.
Reference: `enroll()`, `set_segment()`, `read_segments()`.

## Things a central must not do

- Offload or erase while the node is recording: offload is refused outside
  IDLE; an erase loses whatever was not yet offloaded.
- Erase before the offload is verified.
- Re-number segment codes: they are stored on deployed nodes.
