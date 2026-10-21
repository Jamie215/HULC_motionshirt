"""
HULC Motion Shirt — where the tools keep their files.

The tools run two ways: from a checkout of the repository (`python
tools/analyze_session.py …`, or `pip install -e .`) and from an installed copy
(`pip install .`, which puts this folder in site-packages as `hulc_tools`).
In a checkout, data the tools write lives next to the code, as the docs
describe (regress_data/, models/opensense/, tools/regress_cases/). An
installed copy must not write into site-packages, so the same things go under
a home folder instead: $HULC_HOME, default ~/HULC.
"""
import os

TOOLS_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_DIR = os.path.dirname(TOOLS_DIR)
# installed by pip: the folder is the `hulc_tools` package, not tools/
INSTALLED = os.path.basename(TOOLS_DIR) == "hulc_tools"
HOME_DIR = os.environ.get("HULC_HOME") or os.path.join(os.path.expanduser("~"), "HULC")


def writable_dir(*checkout_parts, installed):
    """A folder the tools write to: under the repo in a checkout, under
    HOME_DIR when installed."""
    if INSTALLED:
        return os.path.join(HOME_DIR, installed)
    return os.path.join(REPO_DIR, *checkout_parts)
