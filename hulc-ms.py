#!/usr/bin/env python3
"""
HULC Motion Shirt — one command for everything (and the standalone app's entry).

Two ways to analyse a session, with the same commands either way:

  * the app:      `python hulc-ms.py` (or double-click hulc-ms.bat / hulc-ms.command,
                  or the standalone HULC-MS app) — drop a block's folder on the page
  * commands:     `python hulc-ms.py <command> …` from the repository, or
                  `hulc-ms <command> …` / `hulc-ms.exe <command> …` with the
                  standalone app (Python built in)

Commands
--------
  app                        the HULC-MS app (the default with no command)
  nodes enroll|offload|erase|check|read-segments …
                             talk to the nodes over BLE
  analyze --capture-dir DIR --outdir OUT [options]
                             analyse one block: block check + review page
  validate --capture-dir DIR [--recursive]
                             check a capture folder (or a whole export) against
                             the data format, before analysing it
  regress run|add …          the regression library of real recordings
  opensense fetch-models     download the OpenSim models (Python install only)
  selftest                   every tool's selftest (checks an install or a build)

`hulc-ms <command> --help` lists a command's options. The individual pipeline
stages (tools/reconcile_nodes.py, calibrate_segments.py, …) stay runnable on
their own for development; see docs/TECHNICAL_REFERENCE.md §9.6.
"""
import importlib
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "tools"))

# The standalone build (PyInstaller) only bundles modules it sees imported.
import analyze_session, app, calibrate_segments, metrics  # noqa: E401,E402,F401
import motion_capabilities, multinode_test, opensense_ik  # noqa: E401,E402,F401
import reconcile_nodes, regress, skeleton_viewer, validate_capture  # noqa: E401,E402,F401

COMMANDS = {"app": "app", "nodes": "multinode_test", "analyze": "analyze_session",
            "validate": "validate_capture", "regress": "regress",
            "opensense": "opensense_ik"}
SELFTESTS = [("reconcile_nodes", "--selftest"), ("motion_capabilities", "--selftest"),
             ("calibrate_segments", "selftest"), ("metrics", "selftest"),
             ("skeleton_viewer", "selftest"), ("analyze_session", "selftest"),
             ("opensense_ik", "selftest"), ("multinode_test", "selftest"),
             ("regress", "selftest"), ("validate_capture", "selftest"),
             ("app", "selftest")]


def _streams():
    """A windowed build (the macOS .app) has no console: write to a log file
    instead of nowhere, and always as UTF-8 (the tools print ° and —)."""
    for name in ("stdout", "stderr"):
        s = getattr(sys, name)
        if s is None:
            from hulc_paths import HOME_DIR
            os.makedirs(HOME_DIR, exist_ok=True)
            setattr(sys, name, open(os.path.join(HOME_DIR, "app.log"), "a",
                                    encoding="utf-8", buffering=1))
        elif hasattr(s, "reconfigure"):
            try:
                s.reconfigure(encoding="utf-8", errors="replace")
            except (ValueError, OSError):
                pass


def run_module(name, argv, prog=None):
    sys.argv = [prog or name] + list(argv)
    mod = importlib.import_module(name)
    try:
        return mod.main()
    except SystemExit as e:
        return e.code


def tool_process(name):
    """How to start one tool as its own process (for `selftest`)."""
    from hulc_paths import tool_cmd
    return tool_cmd(name)


def main():
    _streams()
    os.environ.setdefault("PYTHONIOENCODING", "utf-8")   # for the child processes
    # macOS passes -psn_… to apps started from Finder
    args = [a for a in sys.argv[1:] if not a.startswith("-psn_")]
    if args[:1] == ["--tool"] and len(args) >= 2:          # internal: one tool
        sys.exit(run_module(args[1], args[2:]))
    if args[:1] in (["-h"], ["--help"], ["help"]):
        print(__doc__)
        sys.exit(0)
    if args[:1] == ["selftest"]:
        import subprocess
        bad = [m for m, a in SELFTESTS
               if subprocess.run([*tool_process(m), a]).returncode != 0]
        print(f"\n[hulc-ms] selftests: {len(SELFTESTS) - len(bad)}/{len(SELFTESTS)} "
              f"passed" + (f" — failed: {', '.join(bad)}" if bad else ""))
        sys.exit(1 if bad else 0)
    if args and args[0] in COMMANDS:
        cmd, rest = args[0], args[1:]
        # `analyze --capture-dir …` reads better than `analyze run --capture-dir …`
        if cmd == "analyze" and rest[:1] not in (["run"], ["selftest"]):
            rest = ["run"] + rest
        sys.exit(run_module(COMMANDS[cmd], rest, prog=f"hulc-ms {cmd}"))
    if args and not args[0].startswith("-"):
        print(f"hulc-ms: unknown command {args[0]!r}. Commands: "
              f"{', '.join(COMMANDS)}, selftest  (`hulc-ms --help`; no command = the app)")
        sys.exit(2)
    sys.exit(run_module("app", args, prog="hulc-ms app"))


if __name__ == "__main__":
    main()
