"""
HULC Motion Shirt — one-time node enrollment (shake-to-assign) + montage builder.

Solves the "which physical board is on which body segment?" problem WITHOUT
relying on live streaming during data collection. Collection stays fully offline
(nodes log to flash); this is a **bench setup** step you run once per rig, or
whenever you re-strap onto different segments.

How it works
------------
Each board advertises a stable name `HULC-IMU-XXXX` (last 4 hex of its BLE MAC —
permanent, no reflash). This tool:

  1. scans + connects to every advertising node,
  2. turns on the firmware's DEBUG quaternion stream (control 0x01 -> char A001;
     0x00 stops it) — a bench-only path, never used during real capture,
  3. walks the segments you're placing; for each one it opens a short LISTEN
     window (~3 s) during which you SHAKE the node you're about to strap there,
     while the others sit still. All nodes stream at once, so it discriminates by
     angular speed: the shaken board spins (high deg/s), the still ones read ~0,
     and it reports the id that clearly dominated (and moved for a sustained
     stretch, so a bump can't false-trigger),
  4. writes a schema-valid `montage.json` (columns n0..nN in enrollment order,
     so downstream log ordering is unambiguous) and a persistent
     `nodes_registry.json` (id -> label + last segment).

Because the id is permanent, a second run can REUSE the registry: if every
connected node is already known, it offers to write the montage with no shaking.

Usage
-----
    # validate the pure logic (no hardware / no bleak needed):
    python tools/setup_nodes.py selftest

    # enroll a 2-node elbow rig (interactive, needs the boards + bleak):
    python tools/setup_nodes.py enroll --segments upper_arm_r,forearm_r

    # reuse a prior enrollment if the same boards are present:
    python tools/setup_nodes.py enroll --segments upper_arm_r,forearm_r --reuse

The BLE control conventions mirror multinode_test.py and firmware.ino.
"""
import argparse
import asyncio
import json
import math
import os
import struct
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from motion_capabilities import SEGMENTS, validate_montage  # noqa: E402

NAME_PREFIX = "HULC-IMU"
UUID_QUAT = "A0010001-B0CE-4A4A-8F0B-0011223344FF"
UUID_CONTROL = "A0010002-B0CE-4A4A-8F0B-0011223344FF"
CMD_STREAM_STOP = 0x00
CMD_STREAM_START = 0x01
QUAT_RECORD = struct.Struct("<Iffff")   # t_ms, qw, qx, qy, qz  (matches firmware A001)

DEFAULT_REGISTRY = "nodes_registry.json"
DEFAULT_MONTAGE = "montage.json"

# Shake detection thresholds (deg/s). All nodes stream their live orientation at
# once; a shaken node spins fast, a still one reads ~0. We discriminate on the
# per-node angular speed: the winner must clear an absolute floor AND dominate the
# next-fastest node, and must have MOVED for a sustained stretch (not one spike).
SHAKE_MIN_DPS = 60.0        # the mover's PEAK angular speed must exceed this
SHAKE_DOMINANCE = 3.0       # ...and be >= this many times the next-fastest node
MOVING_FLOOR_DPS = 25.0     # a sample above this counts the node as "moving"
MIN_MOVED_SAMPLES = 4       # ...and it must move for >= this many samples (~0.4s @10Hz)
LISTEN_WINDOW_S = 3.0       # how long we listen per shake prompt


# ---------------------------------------------------------------------------
# Pure logic (unit-tested by selftest — no BLE)
# ---------------------------------------------------------------------------
def quat_speed_dps(q_prev, q_cur, dt_s):
    """Geodesic angular speed (deg/s) between two unit quaternions over dt."""
    if dt_s <= 0:
        return 0.0
    dot = abs(sum(a * b for a, b in zip(q_prev, q_cur)))
    dot = max(-1.0, min(1.0, dot))
    ang = 2.0 * math.acos(dot)               # radians of rotation
    return math.degrees(ang) / dt_s


def eligible_movers(peaks, moved, used, min_moved=MIN_MOVED_SAMPLES):
    """Peaks for nodes that moved for a SUSTAINED stretch and aren't yet assigned.

    Filtering on a sustained sample count (not a single frame) before picking a
    winner is what stops a one-sample glitch or a brief bump from being read as a
    deliberate shake.
    """
    return {k: peaks.get(k, 0.0) for k, n in moved.items()
            if n >= min_moved and k not in used}


def pick_mover(peak_dps, min_dps=SHAKE_MIN_DPS, dominance=SHAKE_DOMINANCE):
    """Given {node_id: peak angular speed}, return the clear single mover or None.

    Returns None when nothing exceeds the floor, or when the top two are too
    close to call (ambiguous — the operator should shake only one node).
    """
    if not peak_dps:
        return None
    ranked = sorted(peak_dps.items(), key=lambda kv: kv[1], reverse=True)
    top_id, top = ranked[0]
    if top < min_dps:
        return None
    nxt = ranked[1][1] if len(ranked) > 1 else 0.0
    if nxt > 0 and top < dominance * nxt:
        return None                          # too close to call
    return top_id


def build_montage(assignments, subject_id="S01", session_id="", notes=""):
    """assignments: ordered list of (node_id, segment, label). -> montage dict.

    Column order follows the list order (n0, n1, ...), which is what reconcile
    and analyze_session use to bind logs to segments.
    """
    nodes = []
    for i, (node_id, segment, label) in enumerate(assignments):
        nodes.append({
            "node_id": node_id,
            "column": f"n{i}",
            "segment": segment,
            "landmark": label or "",
            "calibrated": True,
        })
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
    """Fold this run's (node_id, segment, label) into the persistent registry."""
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
    seg_to_id = {}
    for node_id, entry in registry.items():
        if node_id in present_ids and entry.get("segment"):
            seg_to_id[entry["segment"]] = node_id
    if all(s in seg_to_id for s in segments):
        return [(seg_to_id[s], s, registry[seg_to_id[s]].get("label", ""))
                for s in segments]
    return None


def write_montage(path, montage):
    errors = validate_montage(montage)
    if errors:
        raise SystemExit("[setup] built montage failed validation:\n  - " +
                         "\n  - ".join(errors))
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        json.dump(montage, f, indent=2)
        f.write("\n")


def _norm_segments(seg_arg):
    segs = [s.strip() for s in seg_arg.split(",") if s.strip()]
    bad = [s for s in segs if s not in SEGMENTS]
    if bad:
        raise SystemExit(f"[setup] unknown segment(s): {', '.join(bad)}\n"
                         f"valid: {', '.join(SEGMENTS)}")
    if not segs:
        raise SystemExit("[setup] --segments is empty")
    return segs


# ---------------------------------------------------------------------------
# BLE enrollment (interactive; needs bleak + hardware)
# ---------------------------------------------------------------------------
async def _enroll(segments, montage_path, registry_path, reuse, subject, session):
    try:
        from bleak import BleakClient, BleakScanner
    except ImportError:
        raise SystemExit("[setup] bleak not installed — `pip install bleak` "
                         "(or run `selftest` for the offline logic).")

    print(f"[SCAN] looking for '{NAME_PREFIX}-*' nodes...")
    devices = [d for d in await BleakScanner.discover(timeout=8.0)
               if (d.name or "").startswith(NAME_PREFIX)]
    if not devices:
        raise SystemExit("[SCAN] no HULC nodes found (powered + advertising?).")
    devices.sort(key=lambda d: d.name or d.address)
    for d in devices:
        print(f"[SCAN]   {d.name}  ({d.address})")

    registry = load_registry(registry_path)
    present_ids = {d.name for d in devices}

    if reuse:
        ordered = reuse_from_registry(registry, present_ids, segments)
        if ordered:
            print("[setup] all segments known from a prior enrollment:")
            for nid, seg, lbl in ordered:
                print(f"        {seg:<14} <- {nid}"
                      f"{'  (' + lbl + ')' if lbl else ''}")
            if input("[setup] reuse this placement? [Y/n] ").strip().lower() in ("", "y"):
                write_montage(montage_path, build_montage(ordered, subject, session))
                print(f"[setup] wrote {montage_path} (reused, no shaking).")
                return
        else:
            print("[setup] registry does not cover all segments — enrolling.")

    # peaks: max angular speed seen this window; moved: count of "moving" samples;
    # last: (quat, monotonic-time) of the previous notification. All reset per shake.
    clients, peaks, moved, last = {}, {}, {}, {}
    loop = asyncio.get_event_loop()

    async def ask(msg):
        # Run blocking input() OFF the event loop so BLE notifications keep flowing.
        return await loop.run_in_executor(None, input, msg)

    def make_handler(node_id):
        def handler(_char, data):
            if len(data) < QUAT_RECORD.size:
                return
            _t, qw, qx, qy, qz = QUAT_RECORD.unpack_from(data, 0)
            q = (qw, qx, qy, qz)
            prev = last.get(node_id)
            now = time.monotonic()
            if prev is not None:
                dps = quat_speed_dps(prev[0], q, now - prev[1])
                peaks[node_id] = max(peaks.get(node_id, 0.0), dps)
                if dps >= MOVING_FLOOR_DPS:
                    moved[node_id] = moved.get(node_id, 0) + 1
            last[node_id] = (q, now)
        return handler

    try:
        for d in devices:
            c = BleakClient(d.address)
            await c.connect()
            clients[d.name] = c
            await c.write_gatt_char(UUID_CONTROL, bytes([CMD_STREAM_START]), response=True)
            await c.start_notify(UUID_QUAT, make_handler(d.name))
        print(f"[setup] streaming from {len(clients)} node(s).\n")

        assignments, used = [], set()
        for seg in segments:
            while True:
                await ask(f">>> Set the OTHER boards down still. Press Enter, "
                          f"then SHAKE the node for '{seg}'... ")
                peaks.clear(); moved.clear(); last.clear()
                print(f"    listening {LISTEN_WINDOW_S:.0f}s — shake it now...")
                await asyncio.sleep(LISTEN_WINDOW_S)   # loop runs; peaks accumulate
                # eligible = moved for a sustained stretch, and not already assigned
                eligible = eligible_movers(peaks, moved, used)
                mover = pick_mover(eligible)
                if mover is None:
                    if not any(last.values()):
                        print("    [!] no data — are the nodes streaming? retry.\n")
                    else:
                        top = max(peaks.values()) if peaks else 0.0
                        movers = [k for k, n in moved.items() if n >= MIN_MOVED_SAMPLES]
                        why = ("more than one node moved" if len(movers) > 1
                               else f"peak only {top:.0f} deg/s")
                        print(f"    [!] no clear mover ({why}). Shake ONE node, "
                              f"harder, and keep the others still.\n")
                    continue
                lbl = (await ask(f"    detected {mover} "
                                 f"(peak {peaks[mover]:.0f} deg/s). "
                                 f"Physical label (optional): ")).strip()
                assignments.append((mover, seg, lbl))
                used.add(mover)
                print(f"    -> {seg} = {mover}\n")
                break
    finally:
        for name, c in clients.items():
            try:
                await c.write_gatt_char(UUID_CONTROL, bytes([CMD_STREAM_STOP]), response=True)
                await c.stop_notify(UUID_QUAT)
                await c.disconnect()
            except Exception:  # noqa: BLE001 - best-effort cleanup
                pass

    write_montage(montage_path, build_montage(assignments, subject, session))
    save_registry(registry_path, merge_registry(registry, assignments))
    print(f"[setup] wrote {montage_path} and updated {registry_path}.")
    print("[setup] label your boards now so you can --reuse next time.")


# ---------------------------------------------------------------------------
# Self-test: the pure logic, no BLE
# ---------------------------------------------------------------------------
def selftest():
    ok = True

    def check(cond, msg):
        nonlocal ok
        ok = ok and cond
        print(f"[selftest] {'ok ' if cond else 'FAIL'}: {msg}")

    # angular speed: a 90 deg rotation over 0.5 s -> 180 deg/s
    q0 = (1.0, 0.0, 0.0, 0.0)
    q90 = (math.cos(math.radians(45)), math.sin(math.radians(45)), 0.0, 0.0)
    sp = quat_speed_dps(q0, q90, 0.5)
    check(abs(sp - 180.0) < 1.0, f"quat_speed_dps 90deg/0.5s = {sp:.1f} (~180)")

    # pick_mover: clear winner
    check(pick_mover({"A": 200, "B": 10}) == "A", "clear mover picked")
    # pick_mover: below floor -> None
    check(pick_mover({"A": 20, "B": 5}) is None, "below floor -> None")
    # pick_mover: ambiguous (too close) -> None
    check(pick_mover({"A": 200, "B": 150}) is None, "ambiguous -> None")

    # eligible_movers: sustained motion + not-yet-used filtering
    elig = eligible_movers({"A": 200, "B": 200, "C": 30},
                           moved={"A": 6, "B": 1, "C": 6}, used={"C"})
    check(elig == {"A": 200.0}, "eligible drops one-frame glitch (B) and used (C)")
    check(pick_mover(eligible_movers({"A": 200, "B": 200},
                                     moved={"A": 6, "B": 1}, used=set())) == "A",
          "sustained filter resolves a would-be tie -> A")

    # montage build + validate
    asg = [("HULC-IMU-485C", "upper_arm_r", "orange"),
           ("HULC-IMU-B059", "forearm_r", "blue")]
    m = build_montage(asg, subject_id="S01", session_id="t")
    check(validate_montage(m) == [], "built montage validates clean")
    check([n["column"] for n in m["nodes"]] == ["n0", "n1"], "columns n0,n1 in order")
    check(m["nodes"][0]["segment"] == "upper_arm_r", "n0 -> upper_arm_r")

    # registry round-trip + reuse
    reg = merge_registry({}, asg)
    check(reg["HULC-IMU-485C"]["segment"] == "upper_arm_r", "registry stores segment")
    reused = reuse_from_registry(reg, {"HULC-IMU-485C", "HULC-IMU-B059"},
                                 ["upper_arm_r", "forearm_r"])
    check(reused is not None and [a[0] for a in reused] == ["HULC-IMU-485C", "HULC-IMU-B059"],
          "reuse reconstructs ordered assignment")
    missing = reuse_from_registry(reg, {"HULC-IMU-485C"}, ["upper_arm_r", "forearm_r"])
    check(missing is None, "reuse aborts when a node is absent")

    print(f"\n[selftest] {'PASS' if ok else 'FAIL'} — shake detection, montage "
          f"build/validate, and registry reuse.")
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd")

    pe = sub.add_parser("enroll", help="interactive shake-to-assign enrollment")
    pe.add_argument("--segments", required=True,
                    help="comma-separated segments in placement order, "
                         "e.g. upper_arm_r,forearm_r")
    pe.add_argument("--montage", default=DEFAULT_MONTAGE, help="montage output path")
    pe.add_argument("--registry", default=DEFAULT_REGISTRY, help="registry path")
    pe.add_argument("--reuse", action="store_true",
                    help="reuse a prior enrollment if all boards are known")
    pe.add_argument("--subject", default="S01")
    pe.add_argument("--session", default="")

    sub.add_parser("selftest", help="validate the offline logic (no hardware)")

    args = ap.parse_args()
    if args.cmd == "selftest":
        sys.exit(selftest())
    if args.cmd == "enroll":
        segs = _norm_segments(args.segments)
        asyncio.run(_enroll(segs, args.montage, args.registry, args.reuse,
                            args.subject, args.session))
        return
    ap.error("choose a command: enroll | selftest")


if __name__ == "__main__":
    main()
