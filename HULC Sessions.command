#!/bin/bash
# Double-click (macOS) or run (Linux: bash "HULC Sessions.command") to open the
# HULC session app (tools/app.py) in your browser. Keep this window open while
# you use the app; close it or press Ctrl+C to stop.
cd "$(dirname "$0")" || exit 1
PY=python3
command -v "$PY" >/dev/null 2>&1 || PY=python
if ! "$PY" tools/app.py "$@"; then
  echo
  echo "The app did not start. It needs Python 3 with numpy: pip install numpy"
  read -r -p "Press Enter to close."
fi
