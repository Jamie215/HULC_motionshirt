"""
HULC Motion Shirt — the standalone app's entry point (built by build_app.py).

One program holds Python, numpy and every tool. Double-clicked (no
arguments) it starts the session app. With a command it runs that tool:

    HULC                         the session app (same as `HULC app`)
    HULC app [--port N] [selftest]
    HULC analyze run --capture-dir ./capture/block1 --outdir ./out/block1
    HULC nodes offload --count 2 --out-dir ./capture/block1 --subject S01
    HULC regress run --data D:/regress_data
    HULC selftest                every tool's selftest (checks a build)

The tools start each other as separate processes; inside this program that
is `HULC --tool <module> …` (hulc_paths.tool_cmd), dispatched here.
"""
import importlib
import os
import sys

# PyInstaller only bundles modules it can see imported: name every tool.
import analyze_session, app, calibrate_segments, metrics  # noqa: E401,F401
import motion_capabilities, multinode_test, opensense_ik  # noqa: E401,F401
import reconcile_nodes, regress, skeleton_viewer  # noqa: E401,F401

COMMANDS = {"app": "app", "analyze": "analyze_session", "nodes": "multinode_test",
            "regress": "regress", "opensense": "opensense_ik"}
SELFTESTS = [("reconcile_nodes", "--selftest"), ("motion_capabilities", "--selftest"),
             ("calibrate_segments", "selftest"), ("metrics", "selftest"),
             ("skeleton_viewer", "selftest"), ("analyze_session", "selftest"),
             ("opensense_ik", "selftest"), ("multinode_test", "selftest"),
             ("regress", "selftest"), ("app", "selftest")]


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


def run_module(name, argv):
    sys.argv = [name] + list(argv)
    mod = importlib.import_module(name)
    try:
        rc = mod.main()
    except SystemExit as e:
        rc = e.code
    return rc


def main():
    _streams()
    os.environ.setdefault("PYTHONIOENCODING", "utf-8")   # for the child processes
    args = sys.argv[1:]
    # macOS passes -psn_… to apps started from Finder
    args = [a for a in args if not a.startswith("-psn_")]
    if args[:1] == ["--tool"] and len(args) >= 2:
        sys.exit(run_module(args[1], args[2:]))
    if args[:1] == ["selftest"]:
        import subprocess
        bad = [m for m, a in SELFTESTS
               if subprocess.run([sys.executable, "--tool", m, a]).returncode != 0]
        print(f"\n[HULC] selftests: {len(SELFTESTS) - len(bad)}/{len(SELFTESTS)} "
              f"passed" + (f" — failed: {', '.join(bad)}" if bad else ""))
        sys.exit(1 if bad else 0)
    if args and args[0] in COMMANDS:
        sys.exit(run_module(COMMANDS[args[0]], args[1:]))
    if args and not args[0].startswith("-"):
        print(f"HULC: unknown command {args[0]!r}. Commands: "
              f"{', '.join(COMMANDS)}, selftest (no command = the session app).")
        sys.exit(2)
    sys.exit(run_module("app", args))


if __name__ == "__main__":
    main()
