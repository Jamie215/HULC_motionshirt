#!/usr/bin/env python3
"""
HULC Motion Shirt — build the standalone app (Python included) with PyInstaller.

Builds for the system it runs on; each OS needs its own build (CI builds all
three: .github/workflows/app.yml). Needs numpy and pyinstaller (and bleak, so
the built app can offload from the nodes):

    pip install numpy pyinstaller bleak
    python packaging/build_app.py           # -> dist/hulc-ms/ (Windows, Linux)
                                            #    dist/hulc-ms.app (macOS)
    dist/hulc-ms/hulc-ms selftest                 # check the build

OpenSim is not bundled (hundreds of MB); body-model solves stay a feature of
the Python install.
"""
import os
import shutil
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TOOLS = os.path.join(ROOT, "tools")
NAME = "hulc-ms"


def main():
    try:
        import PyInstaller.__main__ as pyi
    except ImportError:
        raise SystemExit("needs PyInstaller:  pip install pyinstaller")
    sep = os.pathsep
    args = [
        os.path.join(ROOT, "hulc-ms.py"),
        "--name", NAME,
        "--onedir",                       # fast start: the tools run as child processes
        "--noconfirm", "--clean",
        "--paths", TOOLS,
        "--distpath", os.path.join(ROOT, "dist"),
        "--workpath", os.path.join(ROOT, "build", "pyinstaller"),
        "--specpath", os.path.join(ROOT, "build"),
        "--add-data", f"{os.path.join(TOOLS, 'regress_cases')}{sep}regress_cases",
        "--add-data", f"{os.path.join(TOOLS, 'montage.example.json')}{sep}.",
        "--exclude-module", "opensim",
        "--exclude-module", "tkinter",
    ]
    try:
        import bleak  # noqa: F401  — offload / enroll from the built app
        args += ["--collect-submodules", "bleak"]
        if sys.platform == "win32":
            args += ["--collect-submodules", "winrt"]
    except ImportError:
        print("[build] bleak not installed: the built app cannot talk to nodes")
    if sys.platform == "darwin":
        # a double-clickable .app; it has no console, so its output goes to
        # ~/HULC/app.log and the page's Quit button stops it
        args += ["--windowed", "--osx-bundle-identifier", "org.hulc.ms"]
    else:
        args += ["--console"]             # the window shows the app is running
    pyi.run(args)
    out = os.path.join(ROOT, "dist", NAME + (".app" if sys.platform == "darwin" else ""))
    print(f"\n[build] {out}")
    if sys.platform != "darwin":
        shutil.copy(os.path.join(ROOT, "packaging", "README.txt"),
                    os.path.join(out, "README.txt"))


if __name__ == "__main__":
    main()
