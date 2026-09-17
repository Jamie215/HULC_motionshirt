"""
HULC Motion Shirt — node enrollment (power-one-at-a-time) + montage builder.

Maps each physical board to a body segment WITHOUT any firmware change, live
streaming, or shake heuristic. You power ON **one node at a time**; with only one
board advertising, its id (`HULC-IMU-XXXX`, the last 4 hex of its BLE MAC — a
permanent per-board property) is unambiguous, so you assign it to the segment
you're about to strap it on. It can also erase each node's flash in the same pass
(the wipe you do before a session anyway).

Because the id is permanent, enrollment is a ONE-TIME job per board: label the
boards afterwards, and later sessions can `--reuse` the saved registry with every
node powered together — no per-node power cycling.

    frames: nothing to solve here — this is pure bookkeeping (id <-> segment).
    outputs:
      montage.json        — schema-valid, columns n0..nN in enrollment order
      nodes_registry.json — persistent id -> {segment, label} memory

Usage
-----
    # validate the pure logic (no hardware / no bleak needed):
    python tools/setup_nodes.py selftest

    # enroll a 2-node elbow rig: power ONLY the upper-arm board when prompted,
    # then power ONLY the forearm board:
    python tools/setup_nodes.py enroll --segments upper_arm_r,forearm_r

    # also wipe each node's flash as you enroll it (one board on at a time):
    python tools/setup_nodes.py enroll --segments upper_arm_r,forearm_r --erase

    # reuse a saved mapping (all boards powered), no per-node power cycling:
    python tools/setup_nodes.py enroll --segments upper_arm_r,forearm_r --reuse

The BLE conventions (names, control char, erase command) mirror
multinode_test.py and firmware.ino.
"""
import argparse
import asyncio
import json
import os
import struct
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from motion_capabilities import SEGMENTS, validate_montage  # noqa: E402

NAME_PREFIX = "HULC-IMU"
UUID_CONTROL = "A0010002-B0CE-4A4A-8F0B-0011223344FF"
UUID_STATUS = "A0010003-B0CE-4A4A-8F0B-0011223344FF"
CMD_ERASE = 0x03
SCAN_TIMEOUT = 6.0
ERASE_WAIT_S = 35.0     # firmware eraseLog() blocks ~30s; wait then verify

DEFAULT_REGISTRY = "nodes_registry.json"
DEFAULT_MONTAGE = "montage.json"


# ---------------------------------------------------------------------------
# Pure logic (unit-tested by selftest — no BLE)
# ---------------------------------------------------------------------------
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
# BLE (interactive; needs bleak + hardware)
# ---------------------------------------------------------------------------
async def _scan_names():
    """Return the set of HULC node names currently advertising."""
    from bleak import BleakScanner
    devices = await BleakScanner.discover(timeout=SCAN_TIMEOUT)
    return {d.name: d.address for d in devices
            if (d.name or "").startswith(NAME_PREFIX)}


async def _erase_one(address):
    """Wipe one node's flash (control 0x03); verify it reads 0KB after."""
    from bleak import BleakClient
    c = BleakClient(address)
    await c.connect()
    try:
        await c.write_gatt_char(UUID_CONTROL, bytes([CMD_ERASE]), response=True)
        print("    erasing (~30s, do NOT power off)...")
        await asyncio.sleep(ERASE_WAIT_S)
        data = bytes(await c.read_gatt_char(UUID_STATUS))
        kb = struct.unpack_from("<H", data, 2)[0] if len(data) >= 4 else -1
        print("    erase OK — log is now 0KB." if kb == 0
              else f"    [!] log still {kb}KB — check the node's USB serial.")
    finally:
        await c.disconnect()


async def _enroll(segments, montage_path, registry_path, reuse, erase,
                  subject, session):
    try:
        import bleak  # noqa: F401 - probe early with a clear message
    except ImportError:
        raise SystemExit("[setup] bleak not installed — `pip install bleak` "
                         "(or run `selftest` for the offline logic).")

    registry = load_registry(registry_path)
    loop = asyncio.get_event_loop()

    async def ask(msg):
        return await loop.run_in_executor(None, input, msg)

    if reuse:
        print("[setup] --reuse: scanning for all boards...")
        present = await _scan_names()
        ordered = reuse_from_registry(registry, set(present), segments)
        if ordered:
            print("[setup] all segments known from a prior enrollment:")
            for nid, seg, lbl in ordered:
                print(f"        {seg:<14} <- {nid}"
                      f"{'  (' + lbl + ')' if lbl else ''}")
            a = await ask("[setup] reuse this placement? [Y/n] ")
            if a.strip().lower() in ("", "y"):
                write_montage(montage_path, build_montage(ordered, subject, session))
                print(f"[setup] wrote {montage_path} (reused, no power cycling).")
                return
        else:
            print("[setup] registry doesn't cover all segments — enrolling.")

    assignments, used = [], set()
    for seg in segments:
        while True:
            await ask(f">>> Power ON ONLY the node for '{seg}' (all others OFF), "
                      f"then press Enter... ")
            names = await _scan_names()
            fresh = {n: a for n, a in names.items() if n not in used}
            if not fresh:
                if names:
                    print("    [!] the board(s) seen are already assigned. Power "
                          "OFF the enrolled ones and ON only the new one.\n")
                else:
                    print("    [!] no HULC node advertising. Powered on? Give it a "
                          "few seconds and retry.\n")
                continue
            if len(fresh) > 1:
                print(f"    [!] {len(fresh)} nodes advertising: "
                      f"{', '.join(sorted(fresh))}. Power ON only ONE.\n")
                continue
            node_id, address = next(iter(fresh.items()))
            lbl = (await ask(f"    found {node_id}. Physical label "
                             f"(e.g. 'orange tape', optional): ")).strip()
            if erase:
                await _erase_one(address)
            assignments.append((node_id, seg, lbl))
            used.add(node_id)
            print(f"    -> {seg} = {node_id}\n")
            break

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

    # montage build + validate
    asg = [("HULC-IMU-485C", "upper_arm_r", "orange"),
           ("HULC-IMU-B059", "forearm_r", "blue")]
    m = build_montage(asg, subject_id="S01", session_id="t")
    check(validate_montage(m) == [], "built montage validates clean")
    check([n["column"] for n in m["nodes"]] == ["n0", "n1"], "columns n0,n1 in order")
    check(m["nodes"][0]["segment"] == "upper_arm_r", "n0 -> upper_arm_r")
    check(m["nodes"][0]["node_id"] == "HULC-IMU-485C", "n0 keeps enrolled id")

    # registry round-trip + reuse
    reg = merge_registry({}, asg)
    check(reg["HULC-IMU-485C"]["segment"] == "upper_arm_r", "registry stores segment")
    check(reg["HULC-IMU-485C"]["label"] == "orange", "registry stores label")
    reused = reuse_from_registry(reg, {"HULC-IMU-485C", "HULC-IMU-B059"},
                                 ["upper_arm_r", "forearm_r"])
    check(reused is not None and [a[0] for a in reused] == ["HULC-IMU-485C", "HULC-IMU-B059"],
          "reuse reconstructs ordered assignment")
    missing = reuse_from_registry(reg, {"HULC-IMU-485C"}, ["upper_arm_r", "forearm_r"])
    check(missing is None, "reuse aborts when a node is absent")

    # segment validation
    try:
        _norm_segments("upper_arm_r,not_a_segment")
        check(False, "bad segment should raise")
    except SystemExit:
        check(True, "unknown segment name rejected")

    print(f"\n[selftest] {'PASS' if ok else 'FAIL'} — montage build/validate, "
          f"registry reuse, and segment validation.")
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd")

    pe = sub.add_parser("enroll", help="power-one-at-a-time enrollment -> montage")
    pe.add_argument("--segments", required=True,
                    help="comma-separated segments in placement order, "
                         "e.g. upper_arm_r,forearm_r")
    pe.add_argument("--montage", default=DEFAULT_MONTAGE, help="montage output path")
    pe.add_argument("--registry", default=DEFAULT_REGISTRY, help="registry path")
    pe.add_argument("--reuse", action="store_true",
                    help="reuse a prior enrollment if all boards are present")
    pe.add_argument("--erase", action="store_true",
                    help="also wipe each node's flash as it is enrolled")
    pe.add_argument("--subject", default="S01")
    pe.add_argument("--session", default="")

    sub.add_parser("selftest", help="validate the offline logic (no hardware)")

    args = ap.parse_args()
    if args.cmd == "selftest":
        sys.exit(selftest())
    if args.cmd == "enroll":
        segs = _norm_segments(args.segments)
        asyncio.run(_enroll(segs, args.montage, args.registry, args.reuse,
                            args.erase, args.subject, args.session))
        return
    ap.error("choose a command: enroll | selftest")


if __name__ == "__main__":
    main()
