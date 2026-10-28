#!/usr/bin/env python3
"""
HULC Motion Shirt — check a capture folder against the data format.

The capture folder is where acquisition hands over to analysis
(docs/TECHNICAL_REFERENCE.md chapter 3): today `hulc-ms nodes offload` writes it; later the
mobile app's export will. This checks a folder (or, with --recursive, every
block folder under an export) before it is analysed, and says what is wrong
in plain words:

  ERROR  the analysis cannot run (no logs, a log without a header naming its
         node and segment, two nodes on one segment, unreadable files)
  WARN   it runs, but look (bytes not a whole number of records, framing
         shift, dropped records, clock restart, slow logging rate, a montage
         that disagrees with the headers)

Usage (`hulc-ms validate …` runs this)
-----
    hulc-ms validate --capture-dir ./capture/block1
    hulc-ms validate --capture-dir ./phone_export --recursive [--json]
    hulc-ms validate selftest

Exit code 0 when no folder has an ERROR, 1 otherwise.
"""
import argparse
import contextlib
import io
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from motion_capabilities import (  # noqa: E402
    CAPTURE_MONTAGE, SEGMENT_CODES, SEGMENTS, capture_logs, validate_montage)
import reconcile_nodes as rn  # noqa: E402

RECORD_SIZE = rn.RECORD_SIZE
MIN_ACTIVE_HZ = 8.0          # as the block check: ~10 Hz expected while moving
MIN_VALID_FRAC = 0.9         # below this share of good records, say so


def check_log(nid, lg):
    """Findings for one node log: [(level, text)], plus a summary dict."""
    out, info = [], {"node_id": nid, "file": os.path.basename(lg["bin"])}
    path = lg["bin"]
    size = os.path.getsize(path)
    info["bytes"] = size
    if size < RECORD_SIZE:
        return [("ERROR", f"{info['file']}: {size} bytes — no complete "
                          f"{RECORD_SIZE}-byte record")], info
    if size % RECORD_SIZE:
        out.append(("WARN", f"{info['file']}: {size % RECORD_SIZE} trailing bytes "
                            f"(not a whole number of {RECORD_SIZE}-byte records)"))
    with open(path, "rb") as f:
        raw = f.read()
    off = rn._best_offset(raw)
    if off:
        out.append(("WARN", f"{info['file']}: records start at byte {off} "
                            f"(a framing shift in the transfer)"))
    stats = {}
    try:
        with contextlib.redirect_stdout(io.StringIO()):
            t, _ = rn.load_log(path, stats)
    except SystemExit as e:
        return out + [("ERROR", f"{info['file']}: {e}")], info
    n_rec, n_ok = stats.get("n_records", 0), stats.get("n_valid", len(t))
    info.update(records=n_rec, valid=n_ok)
    if n_rec and n_ok / n_rec < MIN_VALID_FRAC:
        out.append(("WARN", f"{info['file']}: only {n_ok}/{n_rec} records usable "
                            f"(holes in the transfer, or erased flash)"))
    if stats.get("clock_restarts"):
        out.append(("WARN", f"{info['file']}: the node's clock restarted "
                            f"{stats['clock_restarts']}× (a reboot mid-block); only "
                            f"the longest part is analysed"))
    if len(t) >= 2:
        dt = np.diff(t.astype(float))
        moving = dt[(dt > 0) & (dt < 500)]
        hz = 1000.0 / float(np.median(moving)) if len(moving) else 0.0
        info.update(span_s=round((t[-1] - t[0]) / 1000.0, 1), active_hz=round(hz, 1))
        if hz and hz < MIN_ACTIVE_HZ:
            out.append(("WARN", f"{info['file']}: logs ~{hz:.1f} Hz while moving "
                                f"(expected ~10 Hz)"))
    head = lg["header"]
    if head is None:
        out.append(("ERROR", f"{info['file']}: no header — add "
                             f"{os.path.splitext(info['file'])[0]}.seg.json with "
                             f"node_id and segment"))
    else:
        seg, code = head.get("segment"), head.get("code")
        info["segment"] = seg
        if not head.get("node_id"):
            out.append(("WARN", f"{info['file']}: header has no node_id (the file "
                                f"name is used)"))
        if seg not in SEGMENTS:
            out.append(("ERROR", f"{info['file']}: header segment {seg!r} is not "
                                 f"one of {', '.join(SEGMENTS)}"))
        elif code is not None and code != SEGMENT_CODES[seg]:
            out.append(("WARN", f"{info['file']}: header code {code} does not match "
                                f"segment {seg} (code {SEGMENT_CODES[seg]})"))
    return out, info


def check_folder(folder):
    """Findings for one capture folder: (findings, summary)."""
    findings, summary = [], {"folder": folder, "logs": []}
    if not os.path.isdir(folder):
        return [("ERROR", f"{folder}: not a folder")], summary
    logs, problems = capture_logs(folder)
    findings += [("ERROR", p) for p in problems]
    if not logs:
        return findings + [("ERROR", "no .bin logs in this folder")], summary
    segs = {}
    for nid, lg in logs.items():
        f, info = check_log(nid, lg)
        findings += f
        summary["logs"].append(info)
        seg = (lg["header"] or {}).get("segment")
        if seg in SEGMENTS:
            if seg in segs:
                findings.append(("ERROR", f"{nid} and {segs[seg]} both report "
                                          f"segment {seg}"))
            segs[seg] = nid
    if len(logs) == 1:
        findings.append(("WARN", "only one node: segment metrics only, no joint "
                                 "angles"))
    mpath = os.path.join(folder, CAPTURE_MONTAGE)
    if os.path.exists(mpath):
        try:
            with open(mpath, encoding="utf-8-sig") as f:
                m = json.load(f)
            errs = validate_montage(m)
            findings += [("ERROR", f"montage.json: {e}") for e in errs]
            for n in m.get("nodes", []) if not errs else []:
                lg = logs.get(n.get("node_id"))
                if lg is None:
                    findings.append(("ERROR", f"montage.json lists {n.get('node_id')} "
                                              f"but the folder has no log for it"))
                elif lg["header"] and lg["header"].get("segment") != n.get("segment"):
                    findings.append(("WARN", f"montage.json says {n.get('node_id')} is "
                                             f"{n.get('segment')}, its header says "
                                             f"{lg['header'].get('segment')} (the "
                                             f"montage wins)"))
        except (OSError, ValueError) as e:
            findings.append(("ERROR", f"montage.json cannot be read ({e})"))
    summary["verdict"] = ("ERROR" if any(l == "ERROR" for l, _ in findings)
                          else "WARN" if findings else "OK")
    return findings, summary


def block_folders(root):
    """Every folder under `root` that holds .bin logs (an export of many blocks)."""
    out = []
    for d, _, files in os.walk(root):
        if any(f.lower().endswith(".bin") for f in files):
            out.append(d)
    return sorted(out)


def main():
    if sys.argv[1:2] == ["selftest"]:
        sys.exit(selftest())
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--capture-dir", required=True,
                    help="a block's capture folder (or an export, with --recursive)")
    ap.add_argument("--recursive", action="store_true",
                    help="check every folder under --capture-dir that holds .bin logs")
    ap.add_argument("--json", action="store_true", help="machine-readable output")
    args = ap.parse_args()
    folders = (block_folders(args.capture_dir) if args.recursive
               else [args.capture_dir])
    if not folders:
        print(f"[validate] no folder with .bin logs under {args.capture_dir}")
        sys.exit(1)
    results = []
    for folder in folders:
        findings, summary = check_folder(folder)
        summary["findings"] = [{"level": l, "text": t} for l, t in findings]
        results.append(summary)
        if not args.json:
            print(f"\n{summary['verdict']:<5} {folder}")
            for lg in summary["logs"]:
                print(f"      {lg.get('segment') or '?':<12} {lg['node_id']:<16} "
                      f"{lg.get('valid', '?')} records, {lg.get('span_s', '?')} s, "
                      f"~{lg.get('active_hz', '?')} Hz  ({lg['file']})")
            for level, text in findings:
                print(f"  {level:<5} {text}")
    bad = sum(r["verdict"] == "ERROR" for r in results)
    if args.json:
        print(json.dumps(results, indent=2))
    else:
        print(f"\n[validate] {len(results)} folder(s): "
              f"{sum(r['verdict'] == 'OK' for r in results)} OK, "
              f"{sum(r['verdict'] == 'WARN' for r in results)} with warnings, "
              f"{bad} with errors")
    sys.exit(1 if bad else 0)


def selftest():
    import tempfile
    from analyze_session import _write_synth_log
    ok = True

    def check(c, m):
        nonlocal ok
        ok = ok and bool(c)
        print(f"[selftest] {'ok ' if c else 'FAIL'}: {m}")

    root = tempfile.mkdtemp(prefix="hulc_validate_")
    good = os.path.join(root, "S01", "block1")
    os.makedirs(good)
    for nid, seg, off in (("HULC-IMU-AAAA", "upper_arm_r", 0),
                          ("HULC-IMU-BBBB", "forearm_r", 37)):
        _write_synth_log(os.path.join(good, f"export_{nid[-4:]}.bin"), off, 1)
        with open(os.path.join(good, f"export_{nid[-4:]}.seg.json"), "w") as f:
            json.dump({"node_id": nid, "segment": seg, "code": SEGMENT_CODES[seg]}, f)
    f1, s1 = check_folder(good)
    check(s1["verdict"] in ("OK", "WARN") and not any(l == "ERROR" for l, _ in f1)
          and len(s1["logs"]) == 2,
          f"a well-formed folder (any file names) passes ({s1['verdict']})")

    bad = os.path.join(root, "S01", "block2")
    os.makedirs(bad)
    _write_synth_log(os.path.join(bad, "a.bin"), 0, 3)              # no header
    with open(os.path.join(bad, "b.bin"), "wb") as f:
        f.write(b"\x00" * 7)                                        # too short
    with open(os.path.join(bad, "b.seg.json"), "w") as f:
        json.dump({"node_id": "N2", "segment": "elbow"}, f)         # bad segment
    f2, s2 = check_folder(bad)
    texts = " | ".join(t for _, t in f2)
    check(s2["verdict"] == "ERROR" and "no header" in texts
          and "no complete" in texts,
          "missing header and a too-short log are errors")
    check(block_folders(root) == sorted([good, bad]),
          "--recursive finds every block folder under an export")
    print(f"\n[selftest] {'PASS' if ok else 'FAIL'} — capture-folder checks")
    return 0 if ok else 1


if __name__ == "__main__":
    main()
