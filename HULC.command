#!/bin/bash
# The HULC app and commands, from this repository (Python 3.11-3.13).
#   Double-click (macOS) or `bash HULC.command` (Linux): opens the HULC app.
#   With arguments it runs a command: ./HULC.command nodes offload ... (README).
# The first time, it offers to install what it needs (pip install -r requirements.txt).
cd "$(dirname "$0")" || exit 1
PY=python3
command -v "$PY" >/dev/null 2>&1 || PY=python
pause() { [ $# -eq 0 ] || return 0; read -r -p "Press Enter to close."; }
if ! command -v "$PY" >/dev/null 2>&1; then
  echo "Python 3 is not installed. Get it from https://www.python.org/downloads/"
  echo "(or download the standalone HULC app, which needs no Python - see README)"
  pause "$@"; exit 1
fi
if ! "$PY" -c "import numpy" >/dev/null 2>&1; then
  echo "HULC needs a few Python packages (numpy, bleak)."
  read -r -p "Install them now with pip? [Y/n] " ans
  case "$ans" in [nN]*) exit 1;; esac
  "$PY" -m pip install -r requirements.txt || { echo "Install failed."; pause "$@"; exit 1; }
fi
"$PY" hulc.py "$@"
rc=$?
if [ $rc -ne 0 ] && [ $# -eq 0 ]; then
  echo; echo "The app stopped with an error (see above)."; pause
fi
exit $rc
