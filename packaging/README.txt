hulc-ms — the HULC-MS standalone app
====================================

No Python needed. Double-click hulc-ms (hulc-ms.exe on Windows) to open the HULC-MS
app in your browser: drop a block's capture folder on the page to get its
block check and review page. Keep the black window open while you use it;
Quit on the page (or closing the window) stops it.

Your sessions are kept in the HULC_sessions folder in your home folder.

From a terminal the same program runs every command (on Windows, open a
Command Prompt in this folder and type hulc-ms; macOS:
hulc-ms.app/Contents/MacOS/hulc-ms; Linux: ./hulc-ms):

  hulc-ms nodes offload --count 2 --out-dir ./capture/block1 --subject S01
  hulc-ms analyze --capture-dir ./capture/block1 --outdir ./out/block1
  hulc-ms nodes erase --count 2
  hulc-ms selftest

The recording card and the full documentation are in the repository:
docs/SOP.md (the operating manual; §6 is the recording routine).
