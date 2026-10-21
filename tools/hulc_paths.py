"""
HULC Motion Shirt — where the tools keep their files.

The tools run two ways: from a checkout of the repository (`python
tools/analyze_session.py …`) and inside the standalone app
(packaging/build_app.py: Python and the tools in one program). In a checkout,
data the tools write lives next to the code, as the docs describe
(regress_data/, models/opensense/, tools/regress_cases/). The standalone app
must not write inside itself, so the same things go under a home folder
instead: $HULC_HOME, default ~/HULC.
"""
import os
import sys

TOOLS_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_DIR = os.path.dirname(TOOLS_DIR)
# the standalone app (PyInstaller): Python and the tools inside one program,
# which must not write next to its own code
FROZEN = bool(getattr(sys, "frozen", False))
HOME_DIR = os.environ.get("HULC_HOME") or os.path.join(os.path.expanduser("~"), "HULC")


def writable_dir(*checkout_parts, installed):
    """A folder the tools write to: under the repo in a checkout, under
    HOME_DIR in the standalone app."""
    if FROZEN:
        return os.path.join(HOME_DIR, installed)
    return os.path.join(REPO_DIR, *checkout_parts)


def tool_cmd(name):
    """The command that runs one tool (`name` = its module, e.g.
    "reconcile_nodes") as a separate process: this Python and the tool's
    script, or — in the standalone app, which has no separate Python — the
    app itself, which dispatches `--tool <name>` (packaging/hulc_entry.py)."""
    if FROZEN:
        return [sys.executable, "--tool", name]
    return [sys.executable, os.path.join(TOOLS_DIR, name + ".py")]
