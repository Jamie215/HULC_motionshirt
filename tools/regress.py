#!/usr/bin/env python3
"""
HULC Motion Shirt — regression library of real recordings.

Every real capture is a test case: the analysis must keep finding the same
freeze, the same session end, the same verdict and plausible ranges as people
checked once by hand. Run it before and after any change to the pipeline, so a
fix for one recording can never silently break another.

Layout
------
    tools/regress_cases/<name>.json    in git: the montage, a description of
                                       what was done, and the expectations
    <data>/<name>/<node_id>.bin        NOT in git (real recordings): the
                                       offloaded logs (+ .seg.json headers)

<data> is --data, else $HULC_REGRESS_DATA, else ./regress_data (git-ignored).
A case whose recordings are not there is SKIPPED, so the run also works where
the data is not available (e.g. CI without the lab drive).

A case file
-----------
    {
      "description": "what the subject did, and when (review-page seconds)",
      "recorded": "2026-10-01",
      "protocol": "sync-first",            # or "hold-first"
      "montage": { ... the montage used ... },
      "expect": {
        "freeze_start_s": [50, 72],        # where the freeze may start
        "end_s": [155, 164],               # where the analysis may end; null =
                                           # nodes not expected to be seen off
        "verdict": "OK",                   # block check verdict
        "sync_ok": true,                   # clocks aligned
        "facing_known": true,              # front direction found
        "metrics": {                       # optional: joint.dof.stat -> range
          "shoulder_r.elevation.max_deg": [105, 135]
        }
      }
    }
Times are seconds on the aligned timeline — the clock the review page shows.

Usage
-----
    # check every case (exit 1 if any fails):
    python tools/regress.py run [--data DIR] [--case NAME]

    # add a new recording as a case; the expectations are PRE-FILLED from what
    # the analysis finds now — check them against what really happened, edit
    # the file, then commit it:
    python tools/regress.py add 2026-10-07_torso_upper-arm \\
        --capture-dir ./capture/block1 --montage montage.json \\
        --description "twists 20-30 s, freeze 33-40 s, five forward raises ..."

    python tools/regress.py selftest
"""

import argparse
import glob
import json
import os
import shutil
import subprocess
import sys
import tempfile

TOOLS = os.path.dirname(os.path.abspath(__file__))
CASES = os.path.join(TOOLS, "regress_cases")
DEFAULT_DATA = os.path.join(os.path.dirname(TOOLS), "regress_data")


def data_root(arg=None):
    return arg or os.environ.get("HULC_REGRESS_DATA") or DEFAULT_DATA


def load_case(path):
    with open(path, encoding="utf-8-sig") as f:
        case = json.load(f)
    case["_name"] = os.path.splitext(os.path.basename(path))[0]
    return case


def analyze(case, capture_dir, workdir, opensense=False):
    """Run the whole pipeline on one case; return its outputs (or raise)."""
    mpath = os.path.join(workdir, "montage.json")
    with open(mpath, "w") as f:
        json.dump(case["montage"], f, indent=2)
    out = os.path.join(workdir, "out")
    cmd = [sys.executable, os.path.join(TOOLS, "analyze_session.py"), "run",
           "--montage", mpath, "--capture-dir", capture_dir, "--outdir", out,
           "--protocol", case.get("protocol", "sync-first")]
    if not opensense:
        cmd.append("--no-opensense")
    log = os.path.join(workdir, "analyze.log")
    with open(log, "w") as f:
        r = subprocess.run(cmd, stdout=f, stderr=subprocess.STDOUT)
    if r.returncode != 0:
        raise RuntimeError(f"analyze_session failed (exit {r.returncode}); see {log}")

    def read(name):
        with open(os.path.join(out, name), encoding="utf-8") as f:
            return json.load(f)
    return {"calibration": read("calibration.json"), "metrics": read("metrics.json"),
            "block_check": read("block_check.json"),
            "quality": read("aligned.quality.json"), "log": log}


def metric_value(metrics, key):
    """'joint.dof.stat' (e.g. shoulder_r.elevation.max_deg) from metrics.json."""
    joint, dof, stat = key.split(".")
    j = next((x for x in metrics.get("joints", []) if x.get("key") == joint), None)
    d = next((x for x in (j or {}).get("dofs", []) if x.get("key") == dof), None)
    return ((d or {}).get("rom") or {}).get(stat)


def observed(res):
    """The facts a case can be checked against, from one analysis."""
    cal, bc = res["calibration"], res["block_check"]
    nw = cal.get("neutral", {}).get("t_window_ms") or [None]
    end = (cal.get("end") or {}).get("t_end_ms")
    sync = next((c for c in bc["checks"] if c["check"] == "Sync"), {})
    return {
        "freeze_start_s": nw[0] / 1000.0 if nw[0] is not None else None,
        "end_s": end / 1000.0 if end is not None else None,
        "verdict": bc["verdict"],
        "sync_ok": sync.get("status") == "OK",
        "facing_known": bool(cal.get("heading", {}).get("confident")),
    }


def evaluate(expect, res):
    """[(check, ok, detail)] for one case."""
    obs, out = observed(res), []

    def within(v, rng):
        return v is not None and rng[0] <= v <= rng[1]
    if "freeze_start_s" in expect:
        r = expect["freeze_start_s"]
        out.append(("freeze", within(obs["freeze_start_s"], r),
                    f"{obs['freeze_start_s']} s (want {r[0]}–{r[1]})"))
    if "end_s" in expect:
        r = expect["end_s"]
        ok = obs["end_s"] is None if r is None else within(obs["end_s"], r)
        out.append(("end", ok, f"{obs['end_s']} s (want "
                    + ("none" if r is None else f"{r[0]}–{r[1]}") + ")"))
    for key in ("verdict", "sync_ok", "facing_known"):
        if key in expect:
            out.append((key, obs[key] == expect[key],
                        f"{obs[key]} (want {expect[key]})"))
    for key, r in (expect.get("metrics") or {}).items():
        v = metric_value(res["metrics"], key)
        out.append((key, within(v, r), f"{v} (want {r[0]}–{r[1]})"))
    return out


def cmd_run(args):
    paths = sorted(glob.glob(os.path.join(CASES, "*.json")))
    if args.case:
        paths = [p for p in paths if os.path.splitext(os.path.basename(p))[0]
                 in args.case]
    if not paths:
        print(f"[regress] no cases in {CASES}")
        return 0
    root = data_root(args.data)
    n_pass = n_fail = n_skip = 0
    for p in paths:
        case = load_case(p)
        name = case["_name"]
        cap = os.path.join(root, name)
        if not glob.glob(os.path.join(cap, "*.bin")):
            print(f"SKIP  {name}  (no recordings in {cap})")
            n_skip += 1
            continue
        work = tempfile.mkdtemp(prefix=f"hulc_regress_{name}_")
        try:
            res = analyze(case, cap, work, args.opensense)
            checks = evaluate(case.get("expect", {}), res)
        except Exception as e:                      # a crash is a failure
            print(f"FAIL  {name}  {e}")
            n_fail += 1
            continue
        bad = [c for c in checks if not c[1]]
        print(f"{'FAIL' if bad else 'PASS'}  {name}  ({len(checks) - len(bad)}/"
              f"{len(checks)} checks)")
        for check, ok, detail in checks:
            if not ok or args.verbose:
                print(f"      {'ok ' if ok else 'BAD'} {check:<32} {detail}")
        if bad:
            print(f"      log: {res['log']}")
            n_fail += 1
        else:
            n_pass += 1
            if not args.keep:
                shutil.rmtree(work, ignore_errors=True)
    print(f"\n[regress] {n_pass} passed, {n_fail} failed, {n_skip} skipped "
          f"(data: {root})")
    return 1 if n_fail else 0


def cmd_add(args):
    name = args.name
    path = os.path.join(CASES, f"{name}.json")
    if os.path.exists(path) and not args.force:
        raise SystemExit(f"[regress] {path} exists (--force to overwrite)")
    with open(args.montage, encoding="utf-8-sig") as f:
        montage = json.load(f)
    dest = os.path.join(data_root(args.data), name)
    os.makedirs(dest, exist_ok=True)
    copied = 0
    for f in glob.glob(os.path.join(args.capture_dir, "*")):
        if f.endswith((".bin", ".seg.json")):
            shutil.copy2(f, dest)
            copied += f.endswith(".bin")
    if not copied:
        raise SystemExit(f"[regress] no .bin logs in {args.capture_dir}")
    case = {"description": args.description or "TODO: what the subject did, and "
            "when (seconds on the review page's clock)",
            "recorded": args.recorded or "", "protocol": args.protocol,
            "montage": montage}
    work = tempfile.mkdtemp(prefix=f"hulc_regress_{name}_")
    res = analyze(case, dest, work)
    obs = observed(res)
    expect = {
        "freeze_start_s": ([round(obs["freeze_start_s"] - 3, 1),
                            round(obs["freeze_start_s"] + 3, 1)]
                           if obs["freeze_start_s"] is not None else None),
        "end_s": ([round(obs["end_s"] - 3, 1), round(obs["end_s"] + 3, 1)]
                  if obs["end_s"] is not None else None),
        "verdict": obs["verdict"], "sync_ok": obs["sync_ok"],
        "facing_known": obs["facing_known"], "metrics": {},
    }
    for j in res["metrics"].get("joints", []):
        prim = (j.get("reps") or {}).get("primary_dof")
        for d in j.get("dofs", []):
            if d.get("key") == prim and d.get("rom"):
                v = d["rom"]["max_deg"]
                expect["metrics"][f"{j['key']}.{prim}.max_deg"] = [round(v - 10),
                                                                   round(v + 10)]
    case["expect"] = expect
    os.makedirs(CASES, exist_ok=True)
    with open(path, "w") as f:
        json.dump(case, f, indent=2)
        f.write("\n")
    print(f"[regress] copied {copied} log(s) to {dest}")
    print(f"[regress] wrote {path} — expectations PRE-FILLED from today's analysis:")
    print(json.dumps(expect, indent=2))
    print("\nNow CHECK them against what really happened (open "
          f"{os.path.join(work, 'out', 'session.html')}): is the freeze where the "
          "subject froze, the end where the nodes came off, the verdict right? "
          "Edit the file, describe the movements, then commit the case file.")


def selftest():
    """Case evaluation logic on a hand-made result (no recordings needed)."""
    res = {"calibration": {"neutral": {"t_window_ms": [57246.0, 59246.0]},
                           "end": {"t_end_ms": 158654.0},
                           "heading": {"confident": True}},
           "block_check": {"verdict": "OK", "checks": [{"check": "Sync",
                                                        "status": "OK"}]},
           "metrics": {"joints": [{"key": "shoulder_r", "dofs": [
               {"key": "elevation", "rom": {"max_deg": 121.0}}]}]}}
    good = {"freeze_start_s": [50, 72], "end_s": [155, 164], "verdict": "OK",
            "sync_ok": True, "facing_known": True,
            "metrics": {"shoulder_r.elevation.max_deg": [105, 135]}}
    bad = {"freeze_start_s": [1, 4], "end_s": None, "verdict": "OK",
           "metrics": {"shoulder_r.elevation.max_deg": [140, 170]}}
    g, b = evaluate(good, res), evaluate(bad, res)
    ok = (all(c[1] for c in g) and len(g) == 6
          and [c[0] for c in b if not c[1]] == ["freeze", "end",
                                                "shoulder_r.elevation.max_deg"])
    cases = sorted(glob.glob(os.path.join(CASES, "*.json")))
    parsed = all("expect" in load_case(p) and "montage" in load_case(p)
                 for p in cases)
    ok = ok and parsed
    print(f"[selftest] evaluate: a matching result passes all 6 checks, a wrong "
          f"freeze / end / range fails exactly those: {'OK' if ok else 'FAIL'}")
    print(f"[selftest] {len(cases)} case file(s) parse with montage + expect: "
          f"{'OK' if parsed else 'FAIL'}")
    print(f"\n[selftest] {'PASS' if ok else 'FAIL'}")
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd")
    pr = sub.add_parser("run", help="check every case")
    pr.add_argument("--data", help="folder of recordings (one subfolder per case)")
    pr.add_argument("--case", nargs="*", help="only these cases")
    pr.add_argument("--opensense", action="store_true",
                    help="also run the model solves (slow)")
    pr.add_argument("--verbose", "-v", action="store_true", help="show every check")
    pr.add_argument("--keep", action="store_true", help="keep outputs of passing cases")
    pa = sub.add_parser("add", help="add a recording as a new case")
    pa.add_argument("name", help="case name, e.g. 2026-10-07_torso_upper-arm")
    pa.add_argument("--capture-dir", required=True, help="the block's offloaded logs")
    pa.add_argument("--montage", required=True)
    pa.add_argument("--protocol", choices=["sync-first", "hold-first"],
                    default="sync-first")
    pa.add_argument("--description")
    pa.add_argument("--recorded", help="recording date")
    pa.add_argument("--data", help="folder of recordings")
    pa.add_argument("--force", action="store_true")
    sub.add_parser("selftest", help="validate the case logic (no recordings)")
    args = ap.parse_args()
    if args.cmd == "run":
        sys.exit(cmd_run(args))
    if args.cmd == "add":
        cmd_add(args)
        return
    if args.cmd == "selftest":
        sys.exit(selftest())
    ap.error("choose a command: run | add | selftest")


if __name__ == "__main__":
    main()
