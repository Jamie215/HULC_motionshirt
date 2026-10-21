#!/usr/bin/env python3
"""
HULC Motion Shirt — the session app: drop a block's files, get its review page.

A small local web app around `analyze_session.py`. Launch it, a browser tab
opens; drag in a block's capture folder (or its loose `.bin` / `.seg.json`
files). Each block is analyzed with the same pipeline the command line runs,
and the app shows the block check and the review page. On the review page's
Timeline, drag the freeze or the end and press **Apply** to re-analyze in
place. A block can also be kept as a regression test case.

Everything stays on this computer: the server listens on 127.0.0.1 only, and
the sessions live in a workspace folder (default ~/HULC_sessions, or
--workspace / $HULC_WORKSPACE):

    <workspace>/<session id>/capture/   the dropped files (+ montage.json)
    <workspace>/<session id>/out/       the analysis (session.html, block_check.json, …)
    <workspace>/<session id>/session.json   subject, status, verdict, settings

Needs only Python 3 + numpy (OpenSim model solves run too when installed, as
on the command line).

Usage
-----
    python tools/app.py                    # opens http://127.0.0.1:8765/
    python tools/app.py --port 9000 --no-browser --workspace D:/hulc
    python tools/app.py selftest           # drives the app end to end, no browser
"""
import argparse
import datetime
import json
import os
import queue
import re
import shutil
import subprocess
import sys
import threading
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import unquote, urlparse

TOOLS = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, TOOLS)
from motion_capabilities import CAPTURE_MONTAGE, montage_from_capture  # noqa: E402

DEFAULT_PORT = 8765
MAX_FILE_BYTES = 64 * 1024 * 1024        # a node's whole flash is 2 MB
SESSION_ID = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]{0,80}$")
UPLOAD_NAME = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,80}\.(bin|seg\.json)$")
SUBJECT = re.compile(r"^[A-Za-z0-9_.-]{0,40}$")


def default_workspace():
    return (os.environ.get("HULC_WORKSPACE")
            or os.path.join(os.path.expanduser("~"), "HULC_sessions"))


# ---------------------------------------------------------------------------
# Sessions on disk
# ---------------------------------------------------------------------------
class Store:
    """The workspace: one folder per session, its state in session.json."""

    def __init__(self, root):
        self.root = os.path.abspath(root)
        os.makedirs(self.root, exist_ok=True)
        self.lock = threading.Lock()

    def path(self, sid, *parts):
        if not SESSION_ID.match(sid or ""):
            raise KeyError(sid)
        return os.path.join(self.root, sid, *parts)

    def load(self, sid):
        try:
            with open(self.path(sid, "session.json"), encoding="utf-8") as f:
                return json.load(f)
        except (OSError, ValueError):
            raise KeyError(sid)

    def save(self, meta):
        with self.lock:
            p = self.path(meta["id"], "session.json")
            tmp = p + ".tmp"
            with open(tmp, "w", encoding="utf-8") as f:
                json.dump(meta, f, indent=2)
            os.replace(tmp, p)

    def update(self, sid, **kw):
        meta = self.load(sid)
        meta.update(kw)
        self.save(meta)
        return meta

    def create(self, subject, session, protocol, label=""):
        stamp = datetime.datetime.now().strftime("%Y-%m-%d_%H%M%S")
        base = "_".join(x for x in (stamp, subject, label) if x)
        base = re.sub(r"[^A-Za-z0-9_.-]", "-", base)[:70]
        sid, n = base, 1
        with self.lock:
            while os.path.exists(os.path.join(self.root, sid)):
                n += 1
                sid = f"{base}-{n}"
            os.makedirs(os.path.join(self.root, sid, "capture"))
        meta = {"id": sid, "subject": subject, "session": session,
                "protocol": protocol, "label": label,
                "created": datetime.datetime.now().isoformat(timespec="seconds"),
                "status": "uploading", "verdict": None, "error": None,
                "window": None, "end": None, "files": []}
        self.save(meta)
        return meta

    def list(self):
        out = []
        for sid in sorted(os.listdir(self.root), reverse=True):
            try:
                out.append(self.summary(sid))
            except KeyError:
                continue
        return out

    def summary(self, sid):
        meta = self.load(sid)
        segs = []
        try:
            with open(self.path(sid, "capture", CAPTURE_MONTAGE),
                      encoding="utf-8-sig") as f:
                segs = [n["segment"] for n in json.load(f).get("nodes", [])]
        except (OSError, ValueError, KeyError, TypeError):
            pass
        return {**meta, "segments": segs,
                "has_page": os.path.exists(self.path(sid, "out", "session.html"))}

    def detail(self, sid):
        d = self.summary(sid)
        try:
            with open(self.path(sid, "out", "block_check.json"), encoding="utf-8") as f:
                d["block_check"] = json.load(f)
        except (OSError, ValueError):
            d["block_check"] = None
        try:
            with open(self.path(sid, "out", "analyze.log"), encoding="utf-8",
                      errors="replace") as f:
                d["log"] = f.read()[-20000:]
        except OSError:
            d["log"] = ""
        return d

    def remove(self, sid):
        shutil.rmtree(self.path(sid))


# ---------------------------------------------------------------------------
# The analysis: one block at a time, in the background
# ---------------------------------------------------------------------------
def prepare_montage(capture_dir, subject, session):
    """Give the block a montage.json that carries the subject id.

    A dropped montage.json is kept (subject / session filled in when given);
    otherwise it is built from the nodes' .seg.json headers, as offload does.
    Returns None, or the reason the folder cannot describe itself."""
    path = os.path.join(capture_dir, CAPTURE_MONTAGE)
    if os.path.exists(path):
        try:
            with open(path, encoding="utf-8-sig") as f:
                m = json.load(f)
        except (OSError, ValueError) as e:
            return f"the dropped montage.json cannot be read ({e})"
        if subject:
            m.setdefault("subject", {})["id"] = subject
        if session:
            m.setdefault("session", {})["id"] = session
    else:
        m, problems = montage_from_capture(capture_dir, subject or "S01", session)
        if m is None:
            return "; ".join(problems)
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        json.dump(m, f, indent=2)
        f.write("\n")
    return None


def analyze_command(store, meta):
    cmd = [sys.executable, os.path.join(TOOLS, "analyze_session.py"), "run",
           "--capture-dir", store.path(meta["id"], "capture"),
           "--outdir", store.path(meta["id"], "out")]
    if meta.get("protocol") in ("sync-first", "hold-first"):
        cmd += ["--protocol", meta["protocol"]]
    if meta.get("window"):
        cmd += ["--window", f"{int(meta['window'][0])},{int(meta['window'][1])}"]
    if meta.get("end") is not None:
        cmd += ["--end", f"{float(meta['end']):g}"]
    if meta.get("no_opensense"):
        cmd.append("--no-opensense")
    return cmd


class Worker(threading.Thread):
    """Runs queued analyses one after another (each is a subprocess, exactly
    the command-line run, so the regression library covers the app too)."""

    def __init__(self, store):
        super().__init__(daemon=True)
        self.store, self.q = store, queue.Queue()

    def submit(self, sid):
        self.store.update(sid, status="queued", error=None)
        self.q.put(sid)

    def run(self):
        while True:
            sid = self.q.get()
            try:
                self.analyze(sid)
            except Exception as e:      # noqa: BLE001 — never kill the worker
                try:
                    self.store.update(sid, status="failed", error=str(e))
                except KeyError:
                    pass

    def analyze(self, sid):
        try:
            meta = self.store.update(sid, status="running",
                                     started=datetime.datetime.now()
                                     .isoformat(timespec="seconds"))
        except KeyError:
            return                       # removed while queued
        cap, out = self.store.path(sid, "capture"), self.store.path(sid, "out")
        os.makedirs(out, exist_ok=True)
        problem = prepare_montage(cap, meta.get("subject"), meta.get("session"))
        log = os.path.join(out, "analyze.log")
        if problem:
            with open(log, "w", encoding="utf-8") as f:
                f.write(f"[app] cannot analyze: {problem}\n")
            self.store.update(sid, status="failed", error=problem)
            return
        bc = os.path.join(out, "block_check.json")
        if os.path.exists(bc):
            os.remove(bc)                # a re-run must not show the old verdict
        with open(log, "w", encoding="utf-8") as f:
            r = subprocess.run(analyze_command(self.store, meta), stdout=f,
                               stderr=subprocess.STDOUT,
                               env={**os.environ, "PYTHONIOENCODING": "utf-8"})
        verdict = None
        try:
            with open(bc, encoding="utf-8") as f:
                verdict = json.load(f).get("verdict")
        except (OSError, ValueError):
            pass
        err = None
        if r.returncode != 0:
            err = "the analysis stopped; see the log"
            try:
                with open(log, encoding="utf-8", errors="replace") as f:
                    lines = [l.strip() for l in f if l.strip()]
                stop = [l for l in lines if l.startswith("[analyze]")]
                err = (stop or lines or [err])[-1]
            except OSError:
                pass
        self.store.update(sid, status="done" if r.returncode == 0 else "failed",
                          verdict=verdict, error=err,
                          finished=datetime.datetime.now().isoformat(timespec="seconds"))


# ---------------------------------------------------------------------------
# HTTP
# ---------------------------------------------------------------------------
class Handler(BaseHTTPRequestHandler):
    store = None
    worker = None
    quiet = False

    def log_message(self, fmt, *args):
        if not self.quiet:
            sys.stderr.write("[app] " + fmt % args + "\n")

    # -- helpers
    def send(self, code, body=b"", ctype="application/json; charset=utf-8"):
        if isinstance(body, (dict, list)):
            body = json.dumps(body).encode()
        elif isinstance(body, str):
            body = body.encode()
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def fail(self, code, msg):
        self.send(code, {"error": msg})

    def body(self, limit=1 << 20):
        n = int(self.headers.get("Content-Length") or 0)
        if n > limit:
            raise ValueError("too large")
        return self.rfile.read(n)

    def json_body(self):
        raw = self.body()
        return json.loads(raw or b"{}")

    def same_origin(self):
        """Refuse writes from other sites (a page elsewhere posting to
        localhost): the browser sends Origin on cross-site requests."""
        origin = self.headers.get("Origin")
        host = self.headers.get("Host", "")
        return origin is None or urlparse(origin).netloc == host

    def route(self):
        parts = [unquote(p) for p in urlparse(self.path).path.split("/") if p]
        return parts

    # -- GET
    def do_GET(self):
        p = self.route()
        try:
            if not p:
                return self.send(200, APP_PAGE, "text/html; charset=utf-8")
            if p == ["api", "sessions"]:
                return self.send(200, self.store.list())
            if len(p) == 3 and p[:2] == ["api", "sessions"]:
                return self.send(200, self.store.detail(p[2]))
            if len(p) == 3 and p[0] == "sessions" and p[2] == "session.html":
                path = self.store.path(p[1], "out", "session.html")
                if not os.path.exists(path):
                    return self.fail(404, "no review page yet")
                with open(path, "rb") as f:
                    return self.send(200, f.read(), "text/html; charset=utf-8")
        except KeyError:
            return self.fail(404, "no such session")
        self.fail(404, "not found")

    # -- POST / PUT / DELETE
    def do_POST(self):
        if not self.same_origin():
            return self.fail(403, "cross-origin request refused")
        p = self.route()
        try:
            if p == ["api", "sessions"]:
                d = self.json_body()
                subject = str(d.get("subject") or "").strip()
                session = str(d.get("session") or "").strip()
                label = str(d.get("label") or "").strip()
                protocol = d.get("protocol") or "sync-first"
                if not (SUBJECT.match(subject) and SUBJECT.match(session)):
                    return self.fail(400, "subject / session: letters, digits, . _ - only")
                if protocol not in ("sync-first", "hold-first"):
                    return self.fail(400, "protocol: sync-first or hold-first")
                meta = self.store.create(subject, session, protocol,
                                         re.sub(r"[^A-Za-z0-9_.-]", "-", label)[:30])
                meta["no_opensense"] = bool(d.get("no_opensense"))
                self.store.save(meta)
                return self.send(201, meta)
            if len(p) == 4 and p[:2] == ["api", "sessions"]:
                sid, action = p[2], p[3]
                meta = self.store.load(sid)
                if action == "run":
                    caps = os.listdir(self.store.path(sid, "capture"))
                    if not any(c.endswith(".bin") for c in caps):
                        return self.fail(400, "no .bin logs in this session")
                    self.worker.submit(sid)
                    return self.send(202, self.store.summary(sid))
                if action == "rerun":
                    d = self.json_body()
                    w, e = d.get("window"), d.get("end")
                    if w is not None and not (isinstance(w, list) and len(w) == 2
                                              and all(isinstance(x, (int, float)) for x in w)
                                              and w[0] < w[1]):
                        return self.fail(400, "window: [t0_ms, t1_ms]")
                    if e is not None and not isinstance(e, (int, float)):
                        return self.fail(400, "end: a time in ms")
                    proto = d.get("protocol", meta.get("protocol"))
                    if proto not in ("sync-first", "hold-first"):
                        return self.fail(400, "protocol: sync-first or hold-first")
                    self.store.update(sid, window=w, end=e, protocol=proto)
                    self.worker.submit(sid)
                    return self.send(202, self.store.summary(sid))
                if action == "regress":
                    return self.keep_as_case(sid, self.json_body())
        except KeyError:
            return self.fail(404, "no such session")
        except ValueError as e:
            return self.fail(400, f"bad request: {e}")
        self.fail(404, "not found")

    def do_PUT(self):
        if not self.same_origin():
            return self.fail(403, "cross-origin request refused")
        p = self.route()
        if not (len(p) == 5 and p[:2] == ["api", "sessions"] and p[3] == "files"):
            return self.fail(404, "not found")
        sid, name = p[2], p[4]
        if not (UPLOAD_NAME.match(name) or name == CAPTURE_MONTAGE):
            return self.fail(400, f"{name}: only <node>.bin, <node>.seg.json and "
                                  f"montage.json are taken")
        try:
            meta = self.store.load(sid)
            if meta["status"] not in ("uploading",):
                return self.fail(409, "files can only be added before the analysis")
            data = self.body(MAX_FILE_BYTES)
            if name.endswith(".json"):
                json.loads(data.decode("utf-8-sig"))      # must parse
            path = self.store.path(sid, "capture", name)
            with open(path, "wb") as f:
                f.write(data)
            files = sorted(set(meta.get("files", [])) | {name})
            self.store.update(sid, files=files)
            return self.send(201, {"name": name, "bytes": len(data)})
        except KeyError:
            return self.fail(404, "no such session")
        except (ValueError, UnicodeDecodeError) as e:
            return self.fail(400, f"{name}: {e}")

    def do_DELETE(self):
        if not self.same_origin():
            return self.fail(403, "cross-origin request refused")
        p = self.route()
        if len(p) == 3 and p[:2] == ["api", "sessions"]:
            try:
                if self.store.load(p[2])["status"] in ("queued", "running"):
                    return self.fail(409, "wait until the analysis has finished")
                self.store.remove(p[2])
                return self.send(200, {"removed": p[2]})
            except KeyError:
                return self.fail(404, "no such session")
        self.fail(404, "not found")

    def keep_as_case(self, sid, d):
        name = str(d.get("name") or "").strip()
        if not re.match(r"^[A-Za-z0-9][A-Za-z0-9_.-]{2,80}$", name):
            return self.fail(400, "case name: letters, digits, . _ - (e.g. "
                                  "2026-10-21_torso_upper-arm)")
        cmd = [sys.executable, os.path.join(TOOLS, "regress.py"), "add", name,
               "--capture-dir", self.store.path(sid, "capture"),
               "--description", str(d.get("description") or "")[:2000]]
        meta = self.store.load(sid)
        if meta.get("protocol"):
            cmd += ["--protocol", meta["protocol"]]
        r = subprocess.run(cmd, capture_output=True, text=True, cwd=os.path.dirname(TOOLS),
                           env={**os.environ, "PYTHONIOENCODING": "utf-8"})
        return self.send(200 if r.returncode == 0 else 400,
                         {"ok": r.returncode == 0,
                          "output": (r.stdout + r.stderr)[-6000:]})


def make_server(workspace, port, quiet=False):
    store = Store(workspace)
    worker = Worker(store)
    worker.start()
    # sessions left mid-analysis by a previous run go back on the queue
    for s in store.list():
        if s["status"] in ("queued", "running"):
            worker.submit(s["id"])
    handler = type("H", (Handler,), {"store": store, "worker": worker, "quiet": quiet})
    last = None
    for p in ([port] if port == 0 else range(port, port + 20)):
        try:
            return ThreadingHTTPServer(("127.0.0.1", p), handler), store
        except OSError as e:
            last = e
    raise SystemExit(f"[app] no free port from {port}: {last}")


# ---------------------------------------------------------------------------
# The page
# ---------------------------------------------------------------------------
APP_PAGE = r"""<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>HULC Sessions</title>
<style>
:root{--bg:#f6f7f9;--panel:#fff;--ink:#1d232b;--muted:#5f6b78;--line:#dde2e8;
  --accent:#2563c9;--ok:#1a7f54;--redo:#b42318;--note:#a76500;--soft:#eef2f7}
@media (prefers-color-scheme:dark){:root{--bg:#121519;--panel:#1a1f25;--ink:#e6eaef;
  --muted:#9aa5b1;--line:#2c333b;--accent:#6aa0ff;--ok:#3ecf8e;--redo:#ff7a6b;
  --note:#e0a44a;--soft:#222831}}
*{box-sizing:border-box}
[hidden]{display:none!important}
body{margin:0;background:var(--bg);color:var(--ink);
  font:14px/1.45 system-ui,-apple-system,"Segoe UI",Roboto,sans-serif;height:100vh;
  display:grid;grid-template-columns:320px 1fr;overflow:hidden}
aside{border-right:1px solid var(--line);background:var(--panel);display:flex;
  flex-direction:column;min-height:0}
header{padding:14px 16px 10px;border-bottom:1px solid var(--line)}
h1{font-size:16px;margin:0}
.sub{color:var(--muted);font-size:12.5px}
#drop{margin:12px 16px;border:2px dashed var(--line);border-radius:10px;padding:16px;
  text-align:center;color:var(--muted);cursor:pointer;transition:.15s}
#drop.over{border-color:var(--accent);background:var(--soft);color:var(--ink)}
#drop b{color:var(--ink)}
.row{display:flex;gap:8px;padding:0 16px;margin-bottom:8px;align-items:center}
.row label{font-size:12.5px;color:var(--muted);width:64px;flex:none}
input,select,textarea{font:inherit;color:inherit;background:var(--bg);
  border:1px solid var(--line);border-radius:6px;padding:5px 8px;flex:1;min-width:0}
button{font:inherit;color:inherit;background:var(--soft);border:1px solid var(--line);
  border-radius:6px;padding:5px 12px;cursor:pointer}
button.primary{background:var(--accent);border-color:var(--accent);color:#fff}
button:disabled{opacity:.55;cursor:default}
#list{flex:1;overflow:auto;border-top:1px solid var(--line);margin-top:6px}
.item{padding:9px 16px;border-bottom:1px solid var(--line);cursor:pointer}
.item:hover{background:var(--soft)}
.item.sel{background:var(--soft);box-shadow:inset 3px 0 var(--accent)}
.item .t{display:flex;justify-content:space-between;gap:8px;font-weight:600;font-size:13px}
.item .m{color:var(--muted);font-size:12px}
.badge{font-size:11px;font-weight:700;padding:1px 7px;border-radius:9px;
  background:var(--soft);color:var(--muted);white-space:nowrap}
.badge.OK{color:var(--ok)} .badge.REDO{color:var(--redo)}
.badge.failed{color:var(--redo)} .badge.running,.badge.queued{color:var(--accent)}
main{display:flex;flex-direction:column;min-width:0;min-height:0}
#bar{padding:10px 16px;border-bottom:1px solid var(--line);background:var(--panel);
  display:flex;gap:10px;align-items:center;flex-wrap:wrap}
#title{font-weight:700} #status{color:var(--muted);flex:1;min-width:160px}
#checks{padding:8px 16px;background:var(--panel);border-bottom:1px solid var(--line);
  max-height:30vh;overflow:auto;font-size:13px}
#checks table{border-collapse:collapse;width:100%}
#checks td{padding:2px 8px 2px 0;vertical-align:top}
#checks .st{font-weight:700;width:46px} .st.OK{color:var(--ok)}
.st.REDO{color:var(--redo)} .st.NOTE{color:var(--note)}
#checks .fix{color:var(--muted)}
#frame{flex:1;border:0;width:100%;background:var(--bg);min-height:0}
#empty{flex:1;display:flex;align-items:center;justify-content:center;
  color:var(--muted);padding:24px;text-align:center}
#log{display:none;flex:1;margin:0;padding:12px 16px;overflow:auto;
  font:12px/1.4 ui-monospace,Menlo,Consolas,monospace;white-space:pre-wrap}
dialog{border:1px solid var(--line);border-radius:10px;background:var(--panel);
  color:var(--ink);width:min(520px,92vw)}
dialog textarea{width:100%;min-height:90px}
dialog pre{max-height:220px;overflow:auto;font-size:12px;white-space:pre-wrap}
@media (max-width:760px){body{grid-template-columns:1fr;grid-template-rows:auto 1fr;
  overflow:auto;height:auto}aside{max-height:55vh}main{min-height:80vh}}
</style></head><body>
<aside>
  <header><h1>HULC sessions</h1>
    <div class="sub">Drop a block's capture folder to analyze it. Nothing leaves this computer.</div></header>
  <div id="drop" tabindex="0"><b>Drop a capture folder here</b><br>
    or the block's <code>.bin</code> + <code>.seg.json</code> files<br>
    <span class="sub">click to choose files</span></div>
  <input type="file" id="pick" multiple hidden>
  <input type="file" id="pickdir" webkitdirectory hidden>
  <div class="row"><label for="subject">Subject</label>
    <input id="subject" placeholder="e.g. S01" maxlength="40"></div>
  <div class="row"><label for="session">Session</label>
    <input id="session" placeholder="optional" maxlength="40"></div>
  <div class="row"><label for="protocol">Order</label>
    <select id="protocol" title="The order the block was recorded in">
      <option value="sync-first">Sync, then freeze</option>
      <option value="hold-first">Freeze first (older)</option></select></div>
  <div class="row"><button id="choosedir">Choose folder…</button>
    <span class="sub" id="upmsg"></span></div>
  <div id="list"></div>
</aside>
<main>
  <div id="bar" hidden>
    <span id="title"></span><span id="status"></span>
    <button id="showlog">Log</button>
    <button id="open">Open in new tab</button>
    <button id="keep">Keep as test case…</button>
    <button id="remove">Remove</button>
  </div>
  <div id="checks" hidden></div>
  <iframe id="frame" hidden title="Review page"></iframe>
  <pre id="log"></pre>
  <div id="empty">Drop a block's files on the left. Its block check and review page appear here.</div>
</main>
<dialog id="keepdlg"><form method="dialog">
  <h3 style="margin-top:0">Keep as a regression test case</h3>
  <p class="sub">Copies the logs to the regression data folder and writes
    <code>tools/regress_cases/&lt;name&gt;.json</code> with expectations from this analysis.
    Check them against what really happened, then commit the case file.</p>
  <div class="row" style="padding:0"><label for="kname">Name</label><input id="kname"></div>
  <p class="sub" style="margin:8px 0 4px">What the subject did, and when (seconds on the review page):</p>
  <textarea id="kdesc"></textarea>
  <pre id="kout"></pre>
  <div style="display:flex;gap:8px;justify-content:flex-end;margin-top:10px">
    <button value="cancel">Close</button><button id="kgo" class="primary" value="default">Keep</button></div>
</form></dialog>
<script>
const $=id=>document.getElementById(id);
let SEL=null, LIST=[], SHOWLOG=false, LASTPAGE=null;
const esc=s=>String(s??'').replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
async function api(method,url,body,raw){
  const r=await fetch(url,{method,headers:raw?{}:{'Content-Type':'application/json'},
    body:raw?body:(body===undefined?undefined:JSON.stringify(body))});
  const j=await r.json().catch(()=>({}));
  if(!r.ok) throw new Error(j.error||j.output||r.statusText);
  return j;
}
// ---- dropping files: group them by folder, one session per block folder
const wanted=n=>/\.bin$/i.test(n)||/\.seg\.json$/i.test(n)||n==='montage.json';
async function entriesToFiles(entry,path,out){
  if(entry.isFile){ await new Promise(res=>entry.file(f=>{out.push({f,dir:path});res();},res)); }
  else if(entry.isDirectory){
    const rd=entry.createReader(); let batch;
    do{ batch=await new Promise(res=>rd.readEntries(res,()=>res([])));
        for(const e of batch) await entriesToFiles(e,path+'/'+entry.name,out);
    }while(batch.length);
  }
}
function groups(items){
  const g={};
  for(const {f,dir} of items){ if(!wanted(f.name)) continue; (g[dir]=g[dir]||[]).push(f); }
  return Object.entries(g).filter(([,fs])=>fs.some(f=>/\.bin$/i.test(f.name)));
}
async function upload(items){
  const gs=groups(items);
  if(!gs.length){ $('upmsg').textContent='no .bin logs in what was dropped'; return; }
  for(const [dir,files] of gs){
    const label=dir.split('/').filter(Boolean).pop()||'';
    $('upmsg').textContent=`uploading ${label||'files'}…`;
    try{
      const m=await api('POST','/api/sessions',{subject:$('subject').value.trim(),
        session:$('session').value.trim(),protocol:$('protocol').value,label});
      for(const f of files)
        await api('PUT',`/api/sessions/${encodeURIComponent(m.id)}/files/${encodeURIComponent(f.name)}`,f,true);
      await api('POST',`/api/sessions/${encodeURIComponent(m.id)}/run`,{});
      SEL=m.id;
    }catch(e){ $('upmsg').textContent=e.message; return; }
  }
  $('upmsg').textContent=gs.length>1?`${gs.length} blocks queued`:'';
  refresh();
}
const drop=$('drop');
['dragenter','dragover'].forEach(ev=>drop.addEventListener(ev,e=>{e.preventDefault();drop.classList.add('over');}));
['dragleave','drop'].forEach(ev=>drop.addEventListener(ev,()=>drop.classList.remove('over')));
drop.addEventListener('drop',async e=>{
  e.preventDefault(); const out=[];
  const its=[...e.dataTransfer.items].map(i=>i.webkitGetAsEntry&&i.webkitGetAsEntry()).filter(Boolean);
  if(its.length) for(const en of its) await entriesToFiles(en,'',out);
  else for(const f of e.dataTransfer.files) out.push({f,dir:''});
  upload(out);
});
drop.addEventListener('click',()=>$('pick').click());
drop.addEventListener('keydown',e=>{if(e.key==='Enter'||e.key===' ')$('pick').click();});
$('choosedir').addEventListener('click',()=>$('pickdir').click());
$('pick').addEventListener('change',e=>{upload([...e.target.files].map(f=>({f,dir:''})));e.target.value='';});
$('pickdir').addEventListener('change',e=>{
  upload([...e.target.files].map(f=>({f,dir:'/'+f.webkitRelativePath.split('/').slice(0,-1).join('/')})));
  e.target.value='';});
// ---- the list and the selected session
function badge(s){
  if(s.status==='done') return `<span class="badge ${esc(s.verdict||'')}">${esc(s.verdict||'done')}</span>`;
  return `<span class="badge ${esc(s.status)}">${esc(s.status)}</span>`;
}
function drawList(){
  $('list').innerHTML=LIST.length?LIST.map(s=>`<div class="item${s.id===SEL?' sel':''}" data-id="${esc(s.id)}">
    <div class="t"><span>${esc(s.subject||'—')}${s.label?' · '+esc(s.label):''}</span>${badge(s)}</div>
    <div class="m">${esc(s.created.replace('T',' '))} · ${esc((s.segments||[]).join(', ')||'…')}</div></div>`).join('')
    :'<div class="item m" style="cursor:default">No sessions yet.</div>';
  document.querySelectorAll('.item[data-id]').forEach(el=>el.addEventListener('click',()=>{SEL=el.dataset.id;SHOWLOG=false;refresh();}));
}
async function refresh(){
  try{ LIST=await api('GET','/api/sessions'); }catch(e){ return; }
  if(!SEL&&LIST.length) SEL=LIST[0].id;
  drawList(); await drawSel();
  if(LIST.some(s=>s.status==='queued'||s.status==='running'||s.status==='uploading'))
    setTimeout(refresh,1200);
}
async function drawSel(){
  const s=LIST.find(x=>x.id===SEL);
  $('bar').hidden=!s; $('empty').hidden=!!s;
  if(!s){ $('frame').hidden=true; $('checks').hidden=true; $('log').style.display='none'; return; }
  const d=await api('GET',`/api/sessions/${encodeURIComponent(s.id)}`).catch(()=>null);
  if(!d) return;
  $('title').textContent=(d.subject||'—')+(d.label?' · '+d.label:'');
  const set=[d.window?`freeze set by hand`:null,d.end!=null?`end set by hand`:null].filter(Boolean).join(', ');
  $('status').textContent={uploading:'uploading…',queued:'waiting for the analysis…',
    running:'analyzing…',done:`analyzed ${d.finished?d.finished.replace('T',' '):''}`,
    failed:'failed: '+(d.error||'see the log')}[d.status]+(set?' · '+set:'');
  const bc=d.block_check;
  $('checks').hidden=!bc;
  if(bc) $('checks').innerHTML=`<b>BLOCK CHECK: <span class="st ${esc(bc.verdict)}">${esc(bc.verdict)}</span></b>
    <table>${bc.checks.map(c=>`<tr><td class="st ${esc(c.status)}">${esc(c.status)}</td>
      <td><b>${esc(c.check)}</b></td><td>${esc(c.detail)}${c.status!=='OK'&&c.fix?
      `<div class="fix">→ ${esc(c.fix)}</div>`:''}</td></tr>`).join('')}</table>`;
  $('log').textContent=d.log||'';
  const busy=d.status==='queued'||d.status==='running';
  $('keep').disabled=busy||!d.has_page; $('open').disabled=!d.has_page; $('remove').disabled=busy;
  const showPage=d.has_page&&!SHOWLOG&&d.status!=='failed';
  $('frame').hidden=!showPage; $('log').style.display=showPage?'none':'block';
  $('showlog').textContent=SHOWLOG?'Review page':'Log';
  const key=d.id+'|'+(d.finished||'');
  if(showPage&&LASTPAGE!==key&&!busy){
    LASTPAGE=key; $('frame').src=`/sessions/${encodeURIComponent(d.id)}/session.html?v=${encodeURIComponent(d.finished||'')}`;
  }
}
$('frame').addEventListener('load',()=>{
  try{ $('frame').contentWindow.postMessage({type:'hulc-app'},location.origin); }catch(e){}
});
window.addEventListener('message',async e=>{
  if(e.source!==$('frame').contentWindow||!e.data||e.data.type!=='hulc-apply'||!SEL) return;
  try{ await api('POST',`/api/sessions/${encodeURIComponent(SEL)}/rerun`,{window:e.data.window,end:e.data.end}); }
  catch(err){ alert(err.message); }
  LASTPAGE=null; refresh();
});
$('showlog').addEventListener('click',()=>{SHOWLOG=!SHOWLOG;drawSel();});
$('open').addEventListener('click',()=>window.open(`/sessions/${encodeURIComponent(SEL)}/session.html`,'_blank'));
$('remove').addEventListener('click',async()=>{
  if(!confirm('Remove this session from the app? Its copied files and analysis are deleted; the original files you dropped are not touched.')) return;
  try{ await api('DELETE',`/api/sessions/${encodeURIComponent(SEL)}`); }catch(e){ alert(e.message); }
  SEL=null; LASTPAGE=null; refresh();
});
$('keep').addEventListener('click',()=>{
  const s=LIST.find(x=>x.id===SEL);
  $('kname').value=(s.created||'').slice(0,10)+'_'+(s.segments||[]).map(x=>x.replace(/_[lr]$/,'').replace('_','-')).join('_');
  $('kdesc').value=''; $('kout').textContent=''; $('keepdlg').showModal();
});
$('kgo').addEventListener('click',async e=>{
  e.preventDefault(); $('kgo').disabled=true; $('kout').textContent='running the analysis for the case…';
  try{ const r=await api('POST',`/api/sessions/${encodeURIComponent(SEL)}/regress`,
        {name:$('kname').value.trim(),description:$('kdesc').value});
       $('kout').textContent=r.output; }
  catch(err){ $('kout').textContent=err.message; }
  $('kgo').disabled=false;
});
refresh();
</script></body></html>
"""


# ---------------------------------------------------------------------------
# Selftest: the whole app over HTTP, no browser
# ---------------------------------------------------------------------------
def selftest():
    import tempfile
    import time
    import urllib.error
    import urllib.request
    from analyze_session import _write_synth_log

    ok = True

    def check(c, m):
        nonlocal ok
        ok = ok and bool(c)
        print(f"[selftest] {'ok ' if c else 'FAIL'}: {m}")

    ws = tempfile.mkdtemp(prefix="hulc_app_ws_")
    srv, store = make_server(ws, 0, quiet=True)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    base = f"http://127.0.0.1:{srv.server_address[1]}"

    def req(method, path, body=None, raw=None, headers=None):
        data = raw if raw is not None else (json.dumps(body).encode()
                                            if body is not None else None)
        r = urllib.request.Request(base + path, data=data, method=method,
                                   headers=headers or {"Content-Type": "application/json"})
        try:
            with urllib.request.urlopen(r, timeout=30) as resp:
                return resp.status, resp.read()
        except urllib.error.HTTPError as e:
            return e.code, e.read()

    def wait(sid, limit=240):
        t0 = time.time()
        while time.time() - t0 < limit:
            st = json.loads(req("GET", f"/api/sessions/{sid}")[1])
            if st["status"] in ("done", "failed"):
                return st
            time.sleep(0.3)
        return st

    code, page = req("GET", "/")
    check(code == 200 and b'id="drop"' in page, "the app page is served")

    # a block: two synthetic logs with their segment headers, plus a montage
    # whose freeze window keeps the synthetic run deterministic
    src = tempfile.mkdtemp(prefix="hulc_app_src_")
    ids = ["HULC-IMU-AAAA", "HULC-IMU-BBBB"]
    _write_synth_log(os.path.join(src, f"{ids[0]}.bin"), offset_ms=0, seed=1)
    _write_synth_log(os.path.join(src, f"{ids[1]}.bin"), offset_ms=37, seed=2)
    montage = {"schema_version": "1.0", "subject": {"id": "X", "notes": ""},
               "session": {"id": "", "aligned_csv": "aligned.csv"},
               "calibration": {"neutral_pose": "N-pose", "captured": True,
                               "t_window_ms": [0, 1500], "functional": []},
               "nodes": [{"node_id": ids[0], "column": "n0", "segment": "upper_arm_r",
                          "landmark": "", "calibrated": True},
                         {"node_id": ids[1], "column": "n1", "segment": "forearm_r",
                          "landmark": "", "calibrated": True}]}
    code, body = req("POST", "/api/sessions", {"subject": "S07", "protocol": "sync-first",
                                                "label": "block1", "no_opensense": True})
    meta = json.loads(body)
    sid = meta.get("id", "")
    check(code == 201 and sid.endswith("S07_block1"), f"a session is created ({sid})")
    for nid, seg in zip(ids, ("upper_arm_r", "forearm_r")):
        with open(os.path.join(src, f"{nid}.bin"), "rb") as f:
            c1 = req("PUT", f"/api/sessions/{sid}/files/{nid}.bin", raw=f.read(),
                     headers={})[0]
        c2 = req("PUT", f"/api/sessions/{sid}/files/{nid}.seg.json",
                 raw=json.dumps({"node_id": nid, "segment": seg}).encode(), headers={})[0]
        check(c1 == 201 and c2 == 201, f"{nid}: log and header uploaded")
    check(req("PUT", f"/api/sessions/{sid}/files/montage.json",
              raw=json.dumps(montage).encode(), headers={})[0] == 201,
          "the dropped montage.json is taken")
    bad = [req("PUT", f"/api/sessions/{sid}/files/{n}", raw=b"x", headers={})[0]
           for n in ("..%2Fescape.bin", "notes.txt", "x.json")]
    check(all(c in (400, 404) for c in bad)
          and not os.path.exists(os.path.join(ws, "escape.bin")),
          "other names and path tricks are refused")
    check(req("GET", "/sessions/..%2F..%2Fetc/session.html")[0] == 404,
          "a session id with a path in it is refused")
    check(req("POST", "/api/sessions", {"subject": "S07"},
              headers={"Content-Type": "application/json",
                       "Origin": "http://evil.example"})[0] == 403,
          "a write from another site is refused")

    req("POST", f"/api/sessions/{sid}/run", {})
    st = wait(sid)
    check(st["status"] == "done" and st["verdict"] in ("OK", "REDO")
          and st["block_check"] and st["has_page"],
          f"analyzed in the background (verdict {st.get('verdict')})")
    with open(os.path.join(ws, sid, "capture", "montage.json")) as f:
        check(json.load(f)["subject"]["id"] == "S07",
              "the subject typed in the app is written into the block's montage")
    code, html = req("GET", f"/sessions/{sid}/session.html")
    check(code == 200 and b'"frames"' in html and b'id="tlapply"' in html,
          "the review page is served, with the Timeline's Apply button")
    lst = json.loads(req("GET", "/api/sessions")[1])
    check([s["id"] for s in lst] == [sid] and lst[0]["segments"] == ["upper_arm_r", "forearm_r"],
          "the sessions list shows it with its segments")

    # Apply on the Timeline: re-run with a hand-set freeze and end
    code, _ = req("POST", f"/api/sessions/{sid}/rerun", {"window": [200, 1400], "end": 6000})
    st = wait(sid)
    with open(os.path.join(ws, sid, "out", "calibration.json")) as f:
        cal = json.load(f)
    check(code == 202 and st["status"] == "done"
          and cal["neutral"]["t_window_ms"] == [200.0, 1400.0]
          and cal.get("end", {}).get("method") == "manual",
          "Apply re-runs the block with the dragged freeze and end")
    check(req("POST", f"/api/sessions/{sid}/rerun", {"window": [5, 1]})[0] == 400,
          "a malformed window is refused")

    # a folder without any segment header cannot describe itself: a clear failure
    code, body = req("POST", "/api/sessions", {"subject": "S08", "no_opensense": True})
    sid2 = json.loads(body)["id"]
    with open(os.path.join(src, f"{ids[0]}.bin"), "rb") as f:
        req("PUT", f"/api/sessions/{sid2}/files/{ids[0]}.bin", raw=f.read(), headers={})
    req("POST", f"/api/sessions/{sid2}/run", {})
    st2 = wait(sid2)
    check(st2["status"] == "failed" and "segment header" in (st2["error"] or ""),
          "a log without a segment header fails with the reason")
    check(req("DELETE", f"/api/sessions/{sid2}")[0] == 200
          and not os.path.exists(os.path.join(ws, sid2)),
          "a session can be removed")

    srv.shutdown()
    print(f"\n[selftest] {'PASS' if ok else 'FAIL'} — upload, background analysis, "
          f"review page, Apply re-run, guards (workspace {ws}).")
    return 0 if ok else 1


def main():
    if sys.argv[1:2] == ["selftest"]:
        sys.exit(selftest())
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--workspace", default=default_workspace(),
                    help="where sessions are kept (default ~/HULC_sessions, or "
                         "$HULC_WORKSPACE)")
    ap.add_argument("--port", type=int, default=DEFAULT_PORT)
    ap.add_argument("--no-browser", action="store_true",
                    help="do not open a browser tab")
    args = ap.parse_args()
    srv, store = make_server(args.workspace, args.port)
    url = f"http://127.0.0.1:{srv.server_address[1]}/"
    print(f"[app] HULC sessions at {url}  (workspace {store.root})", flush=True)
    print("[app] leave this window open while you use the app; Ctrl+C to stop.",
          flush=True)
    if not args.no_browser:
        threading.Timer(0.6, lambda: webbrowser.open(url)).start()
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print("\n[app] stopped.")


if __name__ == "__main__":
    main()
