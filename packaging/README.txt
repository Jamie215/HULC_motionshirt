HULC — standalone app
=====================

No Python needed. Double-click HULC (HULC.exe on Windows) to open the HULC
app in your browser: drop a block's capture folder on the page to get its
block check and review page. Keep the black window open while you use it;
Quit on the page (or closing the window) stops it.

Your sessions are kept in the HULC_sessions folder in your home folder.

From a terminal the same program runs every command (on Windows, open a
Command Prompt in this folder and type hulc; macOS:
HULC.app/Contents/MacOS/HULC; Linux: ./HULC):

  hulc nodes offload --count 2 --out-dir ./capture/block1 --subject S01
  hulc analyze --capture-dir ./capture/block1 --outdir ./out/block1
  hulc nodes erase --count 2
  hulc selftest

The recording card and the full documentation are in the repository:
docs/COLLECTION_SOP.md and docs/COLLECTION_REFERENCE.md.
