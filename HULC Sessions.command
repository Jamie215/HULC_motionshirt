#!/bin/bash
# Double-click (macOS) or run (Linux: bash "HULC Sessions.command") to open the
# HULC session app in your browser. The first time, it offers to install what
# the app needs (pip install -e . : numpy, and the hulc-* commands). Keep this
# window open while you use the app; close it or press Ctrl+C to stop.
cd "$(dirname "$0")" || exit 1
PY=python3
command -v "$PY" >/dev/null 2>&1 || PY=python
if ! command -v "$PY" >/dev/null 2>&1; then
  echo "Python 3 is not installed. Get it from https://www.python.org/downloads/"
  read -r -p "Press Enter to close."; exit 1
fi
if ! "$PY" -c "import numpy" >/dev/null 2>&1; then
  echo "The app needs a few Python packages (numpy)."
  read -r -p "Install them now with pip? [Y/n] " ans
  case "$ans" in [nN]*) exit 1;; esac
  "$PY" -m pip install -e . || { read -r -p "Install failed. Press Enter to close."; exit 1; }
fi
if ! "$PY" tools/app.py "$@"; then
  echo
  read -r -p "The app stopped with an error (see above). Press Enter to close."
fi
