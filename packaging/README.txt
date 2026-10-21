HULC Sessions — standalone app
==============================

No Python needed. Double-click HULC (HULC.exe on Windows) to open the session
app in your browser: drop a block's capture folder on the page to get its
block check and review page. Keep the black window open while you use it;
Quit on the page (or closing the window) stops it.

Your sessions are kept in the HULC_sessions folder in your home folder.

From a terminal the same program runs every tool:

  HULC analyze run --capture-dir ./capture/block1 --outdir ./out/block1
  HULC nodes offload --count 2 --out-dir ./capture/block1 --subject S01
  HULC nodes erase --count 2
  HULC selftest

The recording card and the full documentation are in the repository:
tools/COLLECTION_SOP.md and tools/COLLECTION_REFERENCE.md.
