#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
lite6 - run your robot scripts on the Jetson that drives the UFactory Lite 6.

Your script is uploaded to the Jetson, runs there (wired to the robot), its output
streams back to your terminal live, and every file it creates is downloaded into
./lite6_runs/<job-id>/ when it finishes.

    python lite6.py login 10.13.37.101        # once per computer: trust the Jetson + password
    python lite6.py run my_script.py          # upload, run, watch, download results
    python lite6.py run my_project/ --entry main.py --speed 20   (args after the target go to the script)
    python lite6.py stop                      # stop the running job (Ctrl+C during `run` does the same)
    python lite6.py estop                     # stop the robot NOW, kill the job, halt the queue
    python lite6.py resume                    # accept jobs again after an estop
    python lite6.py status | jobs | logs JOB | get JOB
    python lite6.py studio                    # open UFactory Studio through the Jetson
    python lite6.py relay                     # use the xArm SDK on this PC: XArmAPI('<jetson ip>')
    python lite6.py find                      # look for gateways on this network

As a library:
    import lite6
    gw = lite6.connect()                      # uses the saved login
    result = gw.run("my_script.py", args=["--speed", "20"])
    print(result.status, result.exit_code, result.results_dir)

Standard library only; Python 3.6+ on Windows, macOS and Linux.
"""

import argparse
import codecs
import fnmatch
import getpass
import glob
import hashlib
import http.client
import io
import ipaddress
import json
import os
import queue
import re
import shutil
import signal
import socket
import ssl
import subprocess
import sys
import tempfile
import threading
import time
import urllib.parse
import webbrowser
import zipfile

VERSION = "1.0.0"
DEFAULT_PORT = 8443
DISCOVERY_PORT = 8444
STUDIO_PORT = 18333
CONFIG_DIR = os.environ.get("LITE6_HOME") or os.path.join(os.path.expanduser("~"), ".lite6")
CONFIG_PATH = os.path.join(CONFIG_DIR, "config.json")
DATA_MARKER = "@lite6:data "   # lines the job prints with lite6job.emit()
INFO_MARKER = "@lite6:info "   # notes the gateway adds to the log
MARKERS = (DATA_MARKER, INFO_MARKER)
TERMINAL =("succeeded", "failed", "stopped", "timeout", "cancelled", "error", "interrupted")
DEFAULT_EXCLUDES = [".git", ".hg", ".svn", "__pycache__", "*.pyc", "*.pyo", ".venv", "venv",
                    "node_modules", ".idea", ".vscode", ".ipynb_checkpoints", ".DS_Store", "Thumbs.db",
                    "lite6_runs", ".mypy_cache", ".pytest_cache"]
PIP_SCRIPT = ("import subprocess, sys\n"
              "cmd = [sys.executable, '-m', 'pip'] + sys.argv[1:]\n"
              "print('$', ' '.join(cmd), flush=True)\n"
              "sys.exit(subprocess.call(cmd))\n")
HALTED_MESSAGE = ("the gateway is HALTED after an emergency stop - run `lite6 resume` "
                  "(or press Resume on the web page) first")


class Lite6Error(Exception):
    """Anything that should be shown to the user as a one-line error."""


class GatewayError(Lite6Error):
    def __init__(self, message, status=None, payload=None):
        Lite6Error.__init__(self, message)
        self.status = status
        self.payload = payload or {}


class Unreachable(Lite6Error):
    pass


class ConnectFailed(Unreachable):
    """Could not even open a connection (wrong/changed address, Jetson off, other network)."""


class CertMismatch(Lite6Error):
    pass


def _raise_if_halted(e):
    """409 {"halted": true}: after an emergency stop the gateway refuses new work until resumed."""
    if e.status == 409 and e.payload.get("halted"):
        raise GatewayError(HALTED_MESSAGE, e.status, e.payload)


def say(msg):
    """Client messages go to stderr so stdout carries only the job's own output."""
    try:
        sys.stderr.write("[lite6] %s\n" % msg)
    except UnicodeEncodeError:
        sys.stderr.write(("[lite6] %s\n" % msg).encode("ascii", "replace").decode("ascii"))
    sys.stderr.flush()


def _say_list(header, rows, limit=8):
    say(header)
    for row in rows[:limit]:
        say("  " + row)
    if len(rows) > limit:
        say("  ... and %d more" % (len(rows) - limit))


def write_out(text):
    try:
        sys.stdout.write(text)
    except UnicodeEncodeError:
        enc = sys.stdout.encoding or "ascii"
        sys.stdout.write(text.encode(enc, "replace").decode(enc, "replace"))
    sys.stdout.flush()


def fmt_time(ts):
    return time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(ts)) if ts else "-"


def fmt_size(n):
    n = float(n or 0)
    for unit in ("B", "KB", "MB", "GB"):
        if n < 1024 or unit == "GB":
            return ("%d %s" % (n, unit)) if unit == "B" else ("%.1f %s" % (n, unit))
        n /= 1024.0


def cert_fingerprint(der):
    digest = hashlib.sha256(der).hexdigest().upper()
    return ":".join(digest[i:i + 2] for i in range(0, len(digest), 2))


def split_hostport(text, default_port=DEFAULT_PORT):
    text = text.strip()
    for prefix in ("https://", "http://"):
        if text.lower().startswith(prefix):
            text = text[len(prefix):]
    text = text.split("/")[0]
    if text.startswith("["):  # [ipv6]:port
        host, _, rest = text[1:].partition("]")
        return host, int(rest[1:]) if rest.startswith(":") else default_port
    if text.count(":") == 1:
        host, port = text.split(":")
        return host, int(port)
    return text, default_port


def _valid_port(value):
    """Port number from an (unauthenticated) discovery reply, or None if it is not one."""
    if isinstance(value, bool) or not isinstance(value, (int, str)):
        return None
    try:
        port = int(value)
    except ValueError:
        return None
    return port if 0 < port < 65536 else None


# --------------------------------------------------------------------------- config

def load_config():
    try:
        with open(CONFIG_PATH, "r", encoding="utf-8") as f:
            cfg = json.load(f)
    except (OSError, ValueError):
        cfg = {}
    cfg.setdefault("current", "default")
    cfg.setdefault("profiles", {})
    return cfg


def save_config(cfg):
    os.makedirs(CONFIG_DIR, exist_ok=True)
    fd, tmp = tempfile.mkstemp(prefix=".cfg-", dir=CONFIG_DIR)
    with os.fdopen(fd, "w", encoding="utf-8") as f:
        json.dump(cfg, f, indent=1, sort_keys=True)
    if os.name != "nt":
        os.chmod(tmp, 0o600)
    os.replace(tmp, CONFIG_PATH)


# --------------------------------------------------------------------------- HTTP

class _PinnedHTTPSConnection(http.client.HTTPSConnection):
    """HTTPS to a self-signed gateway: the certificate is pinned by its SHA-256
    fingerprint (trust on first use, like SSH) instead of a CA/hostname check."""

    def __init__(self, host, port, expected_fp, timeout):
        ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)
        ctx.check_hostname = False
        ctx.verify_mode = ssl.CERT_NONE
        http.client.HTTPSConnection.__init__(self, host, port, timeout=timeout, context=ctx)
        self.expected_fp = expected_fp
        self.seen_fp = None

    def connect(self):
        http.client.HTTPSConnection.connect(self)
        der = self.sock.getpeercert(binary_form=True)
        self.seen_fp = cert_fingerprint(der) if der else None
        if self.expected_fp and self.seen_fp != self.expected_fp:
            self.sock.close()
            self.sock = None
            raise CertMismatch(
                "the gateway at %s presented a DIFFERENT certificate than the one you trusted.\n"
                "  trusted: %s\n  now:     %s\n"
                "If you re-installed the gateway this is expected: run `lite6 login %s --reset`.\n"
                "Otherwise something else is answering at that address - do not send your password."
                % (self.host, self.expected_fp, self.seen_fp, self.host))


class _Progress(object):
    """File wrapper that prints upload progress for big uploads."""

    def __init__(self, f, total, label, quiet):
        self.f, self.total, self.label, self.quiet = f, total, label, quiet
        self.done, self.last = 0, 0.0

    def seek(self, pos):
        self.f.seek(pos)
        self.done = pos

    def read(self, n=-1):
        data = self.f.read(n if n and n > 0 else 65536)
        self.done += len(data)
        now = time.time()
        if not self.quiet and self.total > 2 * 1024 * 1024 and (now - self.last > 0.5 or not data):
            self.last = now
            sys.stderr.write("\r[lite6] %s %s / %s   " % (self.label, fmt_size(self.done), fmt_size(self.total)))
            if not data or self.done >= self.total:
                sys.stderr.write("\n")
            sys.stderr.flush()
        return data


class RunResult(object):
    def __init__(self, job, results_dir, data_records):
        self.job = job
        self.id = job.get("id")
        self.status = job.get("status")
        self.exit_code = job.get("exit_code")
        self.results_dir = results_dir
        self.data = data_records

    @property
    def ok(self):
        return self.status == "succeeded"

    def __repr__(self):
        return "RunResult(id=%r, status=%r, exit_code=%r, results_dir=%r)" % (
            self.id, self.status, self.exit_code, self.results_dir)


class Gateway(object):
    def __init__(self, host, port=DEFAULT_PORT, fingerprint=None, token=None, profile=None, quiet=False,
                 interactive=False):
        self.host = host
        self.port = int(port)
        self.fingerprint = fingerprint
        self.token = token
        self.profile = profile          # name of the saved profile (lets a new IP / token be saved)
        self.quiet = quiet
        self.interactive = interactive  # may ask for the password again when the login expired
        self._relogin_tried = False

    def __repr__(self):
        return "Gateway(%s:%d)" % (self.host, self.port)

    # ---- low level

    def _raw(self, method, path, query=None, body=None, headers=None, timeout=30, pin=True, sink=None):
        url = path + ("?" + urllib.parse.urlencode(query) if query else "")
        hdrs = {"User-Agent": "lite6-client/%s" % VERSION, "X-Lite6": "1"}
        if self.token:
            hdrs["Authorization"] = "Bearer " + self.token
        if isinstance(body, dict):
            body = json.dumps(body).encode("utf-8")
            hdrs["Content-Type"] = "application/json"
        if headers:
            hdrs.update(headers)
        if body is not None and "Content-Length" not in hdrs and isinstance(body, (bytes, bytearray)):
            hdrs["Content-Length"] = str(len(body))
        conn = _PinnedHTTPSConnection(self.host, self.port, self.fingerprint if pin else None,
                                      min(timeout, 10))
        try:
            try:
                conn.connect()
            except CertMismatch:
                raise
            except (OSError, ssl.SSLError) as e:
                raise ConnectFailed("cannot reach the gateway at %s:%d (%s)" % (self.host, self.port, e))
            conn.sock.settimeout(timeout)
            conn.request(method, url, body=body, headers=hdrs)
            resp = conn.getresponse()
            rh = dict((k.lower(), v) for k, v in resp.getheaders())
            if sink is not None and resp.status < 400:
                while True:
                    chunk = resp.read(256 * 1024)
                    if not chunk:
                        break
                    sink(chunk)
                data = b""
            else:
                data = resp.read()
            return resp.status, rh, data, conn.seen_fp
        except (CertMismatch, ConnectFailed):
            raise
        except ssl.SSLError as e:
            raise Unreachable("TLS problem talking to %s:%d: %s (is this really the gateway port?)" %
                              (self.host, self.port, e))
        except (OSError, http.client.HTTPException) as e:
            raise Unreachable("cannot reach the gateway at %s:%d (%s)" % (self.host, self.port, e))
        finally:
            conn.close()

    def request(self, method, path, query=None, body=None, headers=None, timeout=30, raw=False, sink=None,
                relogin=True):
        try:
            status, rh, data, _ = self._raw(method, path, query, body, headers, timeout, sink=sink)
        except ConnectFailed:
            if not self._rediscover():
                raise
            if hasattr(body, "seek"):
                body.seek(0)
            status, rh, data, _ = self._raw(method, path, query, body, headers, timeout, sink=sink)
        if status == 401 and relogin and path != "/api/login" and self._relogin():
            if hasattr(body, "seek"):
                body.seek(0)
            status, rh, data, _ = self._raw(method, path, query, body, headers, timeout, sink=sink)
        if status >= 400:
            try:
                payload = json.loads(data.decode("utf-8"))
                message = payload.get("error") or str(payload)
            except (ValueError, UnicodeError, AttributeError):
                payload, message = {}, data[:300].decode("utf-8", "replace") or "HTTP %d" % status
            raise GatewayError(message, status, payload)
        if raw:
            return data, rh
        return json.loads(data.decode("utf-8")) if data else {}

    def _relogin(self):
        """Token expired or password changed: ask for the password again (interactive only)."""
        if not self.interactive or self._relogin_tried or not sys.stdin or not sys.stdin.isatty() or \
                threading.current_thread() is not threading.main_thread():
            return False
        self._relogin_tried = True
        say("your login on %s expired or was revoked - please log in again" % self.host)
        try:
            password = getpass.getpass("Gateway password: ")
        except (EOFError, KeyboardInterrupt):
            return False
        self.token = None
        self.login(password)
        return True

    def _probe(self, timeout=5):
        """Pinned GET /api/info: does this address really serve the certificate we trust?"""
        if not self.fingerprint:
            return False
        try:
            return self._raw("GET", "/api/info", timeout=timeout)[0] == 200
        except Lite6Error:
            return False

    def _rediscover(self):
        """The Jetson's IP changed (e.g. DHCP handed out a new address): find it again
        by its certificate fingerprint and update the saved profile."""
        if not self.fingerprint:
            return False
        for found in discover(timeout=1.5):
            port = _valid_port(found.get("port", self.port))
            if found.get("fingerprint") != self.fingerprint or port is None or \
                    (found["host"], port) == (self.host, self.port):
                continue
            candidate = Gateway(found["host"], port, self.fingerprint)
            if candidate._probe():
                say("gateway moved: %s -> %s (same certificate); using the new address" %
                    (self.host, found["host"]))
                self.host, self.port = candidate.host, candidate.port
                if self.profile:
                    cfg = load_config()
                    prof = cfg["profiles"].get(self.profile)
                    if prof:
                        prof["host"], prof["port"] = self.host, self.port
                        save_config(cfg)
                return True
        return False

    # ---- API

    def info(self):
        return self.request("GET", "/api/info")

    def login(self, password, ttl_hours=None):
        body = {"password": password}
        if ttl_hours:
            body["ttl_hours"] = ttl_hours
        result = self.request("POST", "/api/login", body=body)
        self.token = result["token"]
        if self.profile:
            cfg = load_config()
            prof = cfg["profiles"].setdefault(self.profile, {})
            prof.update(host=self.host, port=self.port, fingerprint=self.fingerprint,
                        token=self.token, expires=result.get("expires"), name=result.get("name"))
            save_config(cfg)
        return result

    def status(self):
        return self.request("GET", "/api/status")

    def robot(self, detail=False):
        return self.request("GET", "/api/robot", query={"detail": 1} if detail else None,
                            timeout=40 if detail else 15)

    def jobs(self, limit=20):
        return self.request("GET", "/api/jobs", query={"limit": limit})["jobs"]

    def job(self, job_id):
        return self.request("GET", "/api/jobs/%s" % job_id)["job"]

    def resolve_job(self, ref):
        """'last', a full id, or a unique prefix/suffix -> full job id."""
        jobs = self.jobs(limit=200)
        if not jobs:
            raise Lite6Error("there are no jobs on the gateway yet")
        if ref in (None, "", "last"):
            return jobs[0]["id"]
        if ref == "running":
            running = [j for j in jobs if j["status"] == "running"]
            if not running:
                raise Lite6Error("no job is running")
            return running[0]["id"]
        hits = [j["id"] for j in jobs if j["id"] == ref or j["id"].startswith(ref) or j["id"].endswith(ref)]
        if len(hits) == 1:
            return hits[0]
        raise Lite6Error("job %r %s" % (ref, "is ambiguous: " + ", ".join(hits[:5]) if hits else "not found"))

    def _check_accepting(self, queue_ok):
        """Cheap authenticated call before a (possibly big) upload. An expired login is renewed here,
        and a halted gateway or --no-queue while busy fail fast: the gateway's early error reply to
        a big upload would only arrive as a connection reset."""
        st = self.status()
        jobs = st.get("jobs") or {}
        if jobs.get("halted"):
            raise GatewayError(HALTED_MESSAGE, 409, {"halted": True})
        running, queued = jobs.get("running"), jobs.get("queued") or []
        if not queue_ok and (running or queued):
            busy = ["job %s is running" % running.get("id")] if running else []
            if queued:
                busy.append("%d job(s) waiting" % len(queued))
            raise GatewayError("robot busy: %s (drop --no-queue to wait in line)" % ", ".join(busy), 409, {})
        return st

    def submit(self, target, args=None, entry=None, with_files=None, timeout=None, label=None,
               queue_ok=True, excludes=None):
        """Upload a script (.py/.sh), a .zip or a folder. Returns the job dict."""
        args = [str(a) for a in (args or [])]
        target = os.path.abspath(target)
        if not os.path.exists(target):
            raise Lite6Error("not found: %s" % target)
        st = self._check_accepting(queue_ok)
        cleanup = None
        if os.path.isfile(target) and not with_files and not target.lower().endswith(".zip"):
            upload, filename, ctype = target, os.path.basename(target), "application/octet-stream"
        elif os.path.isfile(target) and target.lower().endswith(".zip") and not with_files:
            upload, filename, ctype = target, os.path.basename(target), "application/zip"
        else:
            upload = cleanup = make_zip(target, with_files or [], excludes or [])
            filename, ctype = "upload.zip", "application/zip"
            if os.path.isfile(target):
                entry = os.path.basename(target)
        query = {"args": json.dumps(args), "queue": "1" if queue_ok else "0"}
        if entry:
            query["entry"] = entry.replace("\\", "/")
        if timeout:
            query["timeout"] = int(timeout)
        if label:
            query["label"] = label
        size = os.path.getsize(upload)
        try:
            limit = (st.get("gateway") or {}).get("max_upload_mb")
            if limit is None:
                try:
                    limit = self.info().get("max_upload_mb")
                except Lite6Error:
                    limit = None
            if limit and size > limit * 1024 * 1024:
                raise Lite6Error("upload is %s but the gateway accepts at most %d MB "
                                 "(add a .lite6ignore or --exclude for big data files)" % (fmt_size(size), limit))
            with open(upload, "rb") as f:
                body = _Progress(f, size, "uploading", self.quiet)
                try:
                    result = self.request("POST", "/api/jobs", query=query, body=body, timeout=600, headers={
                        "Content-Type": ctype, "Content-Length": str(size),
                        "X-Filename": urllib.parse.quote(filename)})
                except GatewayError as e:
                    _raise_if_halted(e)
                    if e.status == 409 and not queue_ok:
                        raise GatewayError("%s (drop --no-queue to wait in line)" % e, e.status, e.payload)
                    raise
        finally:
            if cleanup:
                try:
                    os.unlink(cleanup)
                except OSError:
                    pass
        return result["job"]

    def read_log(self, job_id, offset=0, wait=0, max_bytes=512 * 1024, interactive=True):
        data, h = self.request("GET", "/api/jobs/%s/log" % job_id, raw=True, timeout=wait + 30,
                               query={"offset": offset, "wait": int(wait), "max": max_bytes},
                               relogin=interactive)
        return data, int(h.get("x-log-offset", offset + len(data))), int(h.get("x-log-size", 0)), \
            h.get("x-job-status")

    def stop(self, job_id, force=False):
        return self.request("POST", "/api/jobs/%s/stop" % job_id, query={"force": 1} if force else None)["job"]

    def estop(self):
        return self.request("POST", "/api/robot/stop", timeout=40)

    def resume(self):
        """Accept new jobs (and relay access) again after an emergency stop halted the gateway."""
        return self.request("POST", "/api/robot/resume")

    def delete(self, job_id):
        return self.request("DELETE", "/api/jobs/%s" % job_id)

    def files(self, job_id):
        return self.request("GET", "/api/jobs/%s/files" % job_id)["files"]

    def download_results(self, job_id, dest):
        """Download and unpack everything the job created/changed into dest/. Returns dest.
        Names this computer does not allow are changed and files it cannot write are skipped;
        both are reported on stderr."""
        try:
            os.makedirs(dest, exist_ok=True)
            fd, tmp = tempfile.mkstemp(prefix=".lite6-", suffix=".zip", dir=dest)
        except OSError as e:
            raise Lite6Error("cannot save results in %s: %s" % (dest, e))
        try:
            with os.fdopen(fd, "wb") as f:

                def sink(chunk):
                    try:
                        f.write(chunk)
                    except OSError as e:  # e.g. disk full - not a network problem
                        raise Lite6Error("cannot save results in %s: %s" % (dest, e))

                self.request("GET", "/api/jobs/%s/results.zip" % job_id, timeout=600, sink=sink)
            skipped, renamed = safe_unzip(tmp, dest)
        except (OSError, zipfile.BadZipFile) as e:
            raise Lite6Error("cannot unpack the results into %s: %s" % (dest, e))
        finally:
            try:
                os.unlink(tmp)
            except OSError:
                pass
        if renamed and not self.quiet:
            _say_list("renamed %d result file(s) whose names are not allowed on this computer:" % len(renamed),
                      ["%s -> %s" % r for r in renamed])
        if skipped:
            _say_list("could not save %d result file(s) (they are still on the Jetson):" % len(skipped),
                      ["%s: %s" % s for s in skipped])
        return dest

    def relay_allow(self):
        try:
            return self.request("POST", "/api/relay/allow")
        except GatewayError as e:
            _raise_if_halted(e)
            raise

    def relay_revoke(self):
        return self.request("POST", "/api/relay/revoke")

    def change_password(self, old, new):
        result = self.request("POST", "/api/password", body={"old_password": old, "new_password": new})
        self.token = result["token"]
        if self.profile:
            cfg = load_config()
            prof = cfg["profiles"].get(self.profile)
            if prof:
                prof["token"], prof["expires"] = self.token, result.get("expires")
                save_config(cfg)
        return result

    # ---- high level

    def follow(self, job_id, on_text=None, on_data=None, offset=0, results_dir=None, show_data=False,
               on_info=None):
        """Stream a job's output until it finishes. Ctrl+C: 1st = stop, 2nd = kill, 3rd = detach.
        Returns the final job dict (or None if detached)."""
        q = queue.Queue()
        halt = threading.Event()
        state = {"offset": offset}

        def reader():
            failures = 0
            while not halt.is_set():
                try:
                    data, new_off, size, status = self.read_log(job_id, state["offset"], wait=20,
                                                                interactive=False)
                    failures = 0
                except Lite6Error as e:
                    code = getattr(e, "status", None)
                    if code in (401, 403):
                        q.put(("auth", e))  # retrying cannot help; the main thread may log in again
                        return
                    if code == 404 or isinstance(e, CertMismatch):
                        q.put(("fatal", e))
                        return
                    failures += 1
                    if failures == 1:
                        q.put(("warn", "connection problem (%s) - retrying" % e))
                    if failures > 40:
                        q.put(("lost", str(e)))
                        return
                    time.sleep(min(5.0, 0.5 * failures))
                    continue
                state["offset"] = new_off
                if data:
                    q.put(("data", data))
                if status in TERMINAL and new_off >= size:
                    q.put(("end", status))
                    return

        def start_reader():
            threading.Thread(target=reader, daemon=True).start()

        presses = [0]  # Ctrl+C presses; on the main thread a SIGINT handler only counts them

        def on_sigint(signum, frame):
            presses[0] += 1

        def relogin():
            if not self.interactive:
                return False
            if counting:  # Ctrl+C must be able to cancel the password prompt
                signal.signal(signal.SIGINT, previous)
            try:
                return self._relogin()
            except Lite6Error as e:
                say("login failed: %s" % e)
                return False
            finally:
                if counting:
                    signal.signal(signal.SIGINT, on_sigint)

        out = OutputFilter(on_text or write_out, on_data, results_dir, show_data,
                           on_info or ((lambda text: None) if self.quiet else say))
        decoder = codecs.getincrementaldecoder("utf-8")("replace")
        counting, previous, acted = False, None, 0
        try:
            if threading.current_thread() is threading.main_thread() and \
                    signal.getsignal(signal.SIGINT) is not signal.SIG_IGN:
                try:
                    previous = signal.signal(signal.SIGINT, on_sigint)
                    counting = True
                except (ValueError, OSError):
                    pass
                if previous is None:  # the old handler was not set from Python
                    previous = signal.default_int_handler
            start_reader()
            while True:
                try:
                    if presses[0] > acted:  # includes presses made while a stop request was in flight
                        if acted >= 2:
                            say("detached - this does not change the job on the Jetson. "
                                "Re-attach with: lite6 logs %s -f" % job_id)
                            return None
                        acted = 2 if presses[0] >= 2 else 1
                        say("killing the job ..." if acted == 2 else
                            "Ctrl+C - asking the job to stop (Ctrl+C again = kill, 3rd time = detach)")
                        self._send_stop(job_id, force=acted == 2)
                        continue
                    # Short get() timeouts keep Ctrl+C responsive on Windows too.
                    try:
                        kind, value = q.get(timeout=0.2)
                    except queue.Empty:
                        continue
                    if kind == "data":
                        out.feed(decoder.decode(value))
                    elif kind == "warn":
                        say(value)
                    elif kind == "end":
                        out.feed(decoder.decode(b"", final=True))
                        out.flush()
                        break
                    elif kind == "auth":
                        try:
                            ok = relogin()
                        except KeyboardInterrupt:
                            ok = False
                        if not ok:
                            out.flush()
                            addr = self.host if self.port == DEFAULT_PORT else "%s:%d" % (self.host, self.port)
                            raise GatewayError("login expired or revoked (%s); the job keeps running on the "
                                               "Jetson - run `lite6 login %s`, then `lite6 logs %s -f`" %
                                               (value, addr, job_id), value.status, value.payload)
                        start_reader()  # continues at the current offset with the new token
                    else:
                        out.flush()
                        if kind == "fatal":
                            raise value
                        raise Unreachable("lost the connection to the gateway (%s). The job keeps running "
                                          "on the Jetson; re-attach with: lite6 logs %s -f" % (value, job_id))
                except KeyboardInterrupt:  # only when no SIGINT handler could be installed
                    presses[0] += 1
            return self.job(job_id)
        finally:
            halt.set()
            if counting:
                signal.signal(signal.SIGINT, previous)
            out.close()

    def _send_stop(self, job_id, force):
        what = "kill" if force else "stop"
        try:
            job = self.stop(job_id, force=force)
        except Lite6Error as e:
            say("%s request NOT delivered: %s" % (what, e))
            return False
        except KeyboardInterrupt:
            say("%s request interrupted - not confirmed" % what)
            raise
        say("%s request delivered (job status now: %s)" % (what, job.get("status")))
        return True

    def run(self, target, args=None, entry=None, with_files=None, timeout=None, label=None,
            out_dir="lite6_runs", download=True, on_text=None, on_data=None, queue_ok=True, excludes=None,
            on_info=None):
        """Upload + run + stream + download. Returns a RunResult.
        on_text(str): script output (default: print), on_data(dict): records sent with
        lite6job.emit(), on_info(str): gateway notes (default: stderr)."""
        job = self.submit(target, args=args, entry=entry, with_files=with_files, timeout=timeout,
                          label=label, queue_ok=queue_ok,
                          excludes=list(excludes or []) + _results_excludes(target, out_dir))
        results_dir = os.path.join(out_dir, job["id"])
        if not self.quiet:
            pos = job.get("queue_position")
            say("job %s submitted%s" % (job["id"], " (position %d in the queue)" % pos if pos and pos > 1 else ""))
        records = []

        def collect(record):
            records.append(record)
            if on_data:
                on_data(record)

        final = self.follow(job["id"], on_text=on_text, on_data=collect, results_dir=results_dir,
                            on_info=on_info)
        if final is None:
            return RunResult(job, None, records)
        if download:
            self.download_results(job["id"], results_dir)
        return RunResult(final, results_dir if download else None, records)


class OutputFilter(object):
    """Splits a job log into three streams:
      * the script's own output                -> write() (stdout)
      * '@lite6:data {json}' lines from emit() -> on_data() and <results_dir>/live_data.jsonl
      * '@lite6:info ...' notes of the gateway -> on_info() (stderr by default)
    A marker counts at the start of a line: after a newline, or after a carriage return
    (so emit() still works after a progress line printed with end='\\r')."""

    def __init__(self, write, on_data=None, results_dir=None, show_data=False, on_info=None):
        self.write = write
        self.on_data = on_data
        self.on_info = on_info or say
        self.results_dir = results_dir
        self.show_data = show_data
        self.buf = ""            # held back: a line start that is or may still become a marker line
        self.line_start = True
        self._fh = None

    def _marker_line(self, marker, text):
        if marker == DATA_MARKER:
            self._data(text)
        else:
            self.on_info(text)

    def feed(self, text):
        buf = self.buf + text if self.buf else text
        self.buf = ""
        n = len(buf)
        done = i = 0  # buf[:done] is handled; a marker can only start at a '@'
        hold = -1
        while True:
            p = buf.find("@", i)
            if p < 0:
                break
            i = p + 1
            if not (buf[p - 1] in "\r\n" if p else self.line_start):
                continue
            marker = next((m for m in MARKERS if buf.startswith(m, p)), None)
            if marker is None:
                if any(n - p < len(m) and m.startswith(buf[p:]) for m in MARKERS):
                    hold = p  # could still become a marker line - wait for more text
                    break
                continue
            nl = buf.find("\n", p + len(marker))
            if nl < 0:
                hold = p  # marker line not complete yet
                break
            if p > done:
                self.write(buf[done:p])
            self._marker_line(marker, buf[p + len(marker):nl].rstrip("\r"))
            done = i = nl + 1
        end = n if hold < 0 else hold
        if end > done:
            self.write(buf[done:end])
        if hold >= 0:
            self.buf, self.line_start = buf[hold:], True
        elif n:
            self.line_start = buf[-1] in "\r\n"

    def flush(self):
        if self.buf:
            marker = next((m for m in MARKERS if self.buf.startswith(m)), None) if self.line_start else None
            if marker:
                self._marker_line(marker, self.buf[len(marker):].strip())
            else:
                self.write(self.buf)
            self.buf = ""
        self.line_start = True

    def _data(self, text):
        try:
            record = json.loads(text)
        except ValueError:
            record = {"raw": text}
        if self.results_dir:
            if self._fh is None:
                os.makedirs(self.results_dir, exist_ok=True)
                self._fh = open(os.path.join(self.results_dir, "live_data.jsonl"), "a", encoding="utf-8")
            self._fh.write(json.dumps(record) + "\n")
            self._fh.flush()
        if self.on_data:
            self.on_data(record)
        if self.show_data:
            self.write(DATA_MARKER + text + "\n")

    def close(self):
        if self._fh:
            self._fh.close()
            self._fh = None


# --------------------------------------------------------------------------- files

def _excluded(rel, patterns, is_dir=False):
    """gitignore-like: 'name' matches at any depth, '/name' only from the top, 'name/' only folders."""
    parts = rel.split("/")
    for pat in patterns:
        if pat.endswith("/") and not is_dir:
            continue
        anchored = pat.startswith("/")
        pat = pat.strip("/")
        if pat.startswith("**/"):
            pat, anchored = pat[3:], False
        if pat and (fnmatch.fnmatch(rel, pat) or
                    not anchored and any(fnmatch.fnmatch(p, pat) for p in parts)):
            return True
    return False


def _results_excludes(target, out_dir):
    """--out inside the uploaded folder: do not upload earlier results again."""
    if not out_dir or not os.path.isdir(target):
        return []
    try:
        rel = os.path.relpath(os.path.abspath(out_dir), os.path.abspath(target))
    except ValueError:  # another drive
        return []
    if rel in (os.curdir, os.pardir) or rel.startswith(os.pardir + os.sep) or os.path.isabs(rel):
        return []
    return ["/%s/" % glob.escape(rel.replace(os.sep, "/"))]


def make_zip(target, extras, excludes):
    """Zip a folder (or a file plus --with extras) into a temp file; returns its path."""
    patterns = list(DEFAULT_EXCLUDES) + list(excludes)
    base = target if os.path.isdir(target) else os.path.dirname(target)
    ignore_file = os.path.join(base, ".lite6ignore")
    if os.path.isfile(ignore_file):
        with open(ignore_file, "r", encoding="utf-8-sig", errors="replace") as f:  # -sig: Notepad's BOM
            lines = [ln.strip() for ln in f]
        patterns += [ln for ln in lines if ln and not ln.startswith("#")]
    items = []  # (absolute path, archive name)

    def add_tree(path, arc_prefix, pats):
        if os.path.isfile(path):
            items.append((path, arc_prefix))
            return
        for root, dirs, files in os.walk(path):
            rel_root = os.path.relpath(root, path).replace(os.sep, "/")
            rel_root = "" if rel_root == "." else rel_root + "/"
            dirs[:] = [d for d in dirs if not _excluded(rel_root + d, pats, True)]
            for fn in files:
                rel = rel_root + fn
                if not _excluded(rel, pats):
                    items.append((os.path.join(root, fn), (arc_prefix + "/" + rel) if arc_prefix else rel))

    if os.path.isdir(target):
        add_tree(target, "", patterns)
    else:
        items.append((target, os.path.basename(target)))
    loose = [p for p in patterns if not p.startswith("/")]  # '/name' only applies to the main folder
    for extra in extras:
        extra = os.path.abspath(extra)
        if not os.path.exists(extra):
            raise Lite6Error("--with: not found: %s" % extra)
        try:
            rel = os.path.relpath(extra, base)
        except ValueError:  # another drive (Windows)
            rel = os.pardir
        inside = rel != os.pardir and not rel.startswith(os.pardir + os.sep) and not os.path.isabs(rel)
        add_tree(extra, rel.replace(os.sep, "/") if inside else os.path.basename(extra.rstrip("\\/")), loose)
    total = sum(os.path.getsize(p) for p, _ in items)
    if total > 50 * 1024 * 1024:
        say("note: uploading %s in %d files (add a .lite6ignore file to skip big data)" %
            (fmt_size(total), len(items)))
    fd, tmp = tempfile.mkstemp(prefix="lite6-", suffix=".zip")
    os.close(fd)
    with zipfile.ZipFile(tmp, "w", zipfile.ZIP_DEFLATED) as zf:
        seen = set()
        for path, arc in items:
            if arc in seen:
                continue
            seen.add(arc)
            zf.write(path, arc)
    return tmp


_DRIVE_RX = re.compile(r"^[A-Za-z]:$")
_WIN_BAD_RX = re.compile(r'[<>:"|?*\x00-\x1f]')
_WIN_RESERVED = set(["CON", "PRN", "AUX", "NUL"] + ["COM%d" % i for i in range(1, 10)] +
                    ["LPT%d" % i for i in range(1, 10)])


def _windows_name(part):
    """One path component made valid on Windows: <>:"|?* and control characters -> '_',
    no trailing dot/space, reserved device names (CON, NUL, COM1 ...) get a '_'."""
    part = _WIN_BAD_RX.sub("_", part).rstrip(". ") or "_"
    stem, dot, ext = part.partition(".")
    if stem.rstrip(" ").upper() in _WIN_RESERVED:
        part = stem + "_" + dot + ext
    return part


def safe_unzip(zip_path, dest):
    """Unpack a results zip into dest, never outside it. Returns (skipped, renamed): lists of
    (name in the zip, reason) and (name in the zip, path written). On Windows, names it does not
    allow (e.g. ':' in ISO timestamps) are changed instead of failing."""
    root = os.path.realpath(dest)
    skipped, renamed, claimed = [], [], {}
    with zipfile.ZipFile(zip_path) as zf:
        for info in zf.infolist():
            name = info.filename.replace("\\", "/")
            orig = [p for p in name.split("/") if p not in ("", ".")]
            if not orig:
                continue
            if ".." in orig or name.startswith("/") or (os.name == "nt" and _DRIVE_RX.match(orig[0])):
                skipped.append((info.filename, "unsafe path"))
                continue
            parts = [_windows_name(p) for p in orig] if os.name == "nt" else list(orig)
            is_dir = name.endswith("/")
            if not is_dir:  # two names that became the same: number the later one
                stem, ext = os.path.splitext(parts[-1])
                n = 2
                while claimed.setdefault(os.path.normcase("/".join(parts)), orig) != orig:
                    parts[-1] = "%s~%d%s" % (stem, n, ext)
                    n += 1
            target = os.path.join(dest, *parts)
            if not os.path.realpath(target).startswith(root + os.sep):
                skipped.append((info.filename, "unsafe path"))
                continue
            out = None
            try:
                if is_dir:
                    os.makedirs(target, exist_ok=True)
                    continue
                os.makedirs(os.path.dirname(target), exist_ok=True)
                with zf.open(info) as src:
                    out = open(target, "wb")
                    with out:
                        shutil.copyfileobj(src, out)
            except (OSError, zipfile.BadZipFile) as e:
                if out is not None:
                    try:
                        os.unlink(target)
                    except OSError:
                        pass
                skipped.append((info.filename, getattr(e, "strerror", None) or str(e)))
                continue
            if parts != orig:
                renamed.append((info.filename, "/".join(parts)))
    return skipped, renamed


# --------------------------------------------------------------------------- discovery

_IPV4 = r"\d{1,3}(?:\.\d{1,3}){3}"


def _tool_output(cmd):
    """Output of ipconfig / ip / ifconfig, or '' if the tool is missing, fails or hangs."""
    exe = shutil.which(cmd[0]) or next((p for p in ("/sbin/" + cmd[0], "/usr/sbin/" + cmd[0])
                                        if os.path.isfile(p)), None)
    if not exe:
        return ""
    try:
        out = subprocess.check_output([exe] + cmd[1:], stdin=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                                      timeout=3, creationflags=0x08000000 if os.name == "nt" else 0)
    except (OSError, ValueError, subprocess.SubprocessError):  # 0x08000000 = CREATE_NO_WINDOW
        return ""
    return out.decode("ascii", "replace")


def _netmask_bits(text):
    """'255.255.192.0' or '0xffffc000' -> 18; None if text is not a netmask."""
    try:
        n = int(text, 16) if text.lower().startswith("0x") else int(ipaddress.IPv4Address(text))
    except ValueError:
        return None
    hostmask = ~n & 0xFFFFFFFF
    if n >> 24 != 255 or hostmask & (hostmask + 1):
        return None
    return 32 - hostmask.bit_length()


def _parse_ipconfig(text):
    """Windows `ipconfig`. Its labels are translated, so each address is paired with the
    netmask on the line that follows it."""
    found, last = [], None
    for line in text.splitlines():
        ips = re.findall(_IPV4, line)
        if not ips:
            continue
        bits = _netmask_bits(ips[-1])
        if bits is None:
            last = ips[-1]
        elif last:
            found.append((last, bits))
            last = None
    return found


def _parse_ip_addr(text):
    """Linux `ip -4 -o addr show`: '... inet 192.168.1.5/24 brd ...'."""
    return [(ip, int(bits)) for ip, bits in re.findall(r"\binet\s+(%s)/(\d{1,2})\b" % _IPV4, text)]


def _parse_ifconfig(text):
    """macOS/BSD `ifconfig` ('inet A netmask 0xffffff00 ...') and old Linux ('inet addr:A ... Mask:M')."""
    found = []
    for line in text.splitlines():
        ip = re.search(r"\binet\s+(?:addr:)?(%s)" % _IPV4, line)
        mask = re.search(r"(?:netmask|Mask:)\s*(0x[0-9a-fA-F]+|%s)" % _IPV4, line)
        bits = _netmask_bits(mask.group(1)) if ip and mask else None
        if bits is not None:
            found.append((ip.group(1), bits))
    return found


def _interface_networks():
    """(address, prefix length) of this computer's IPv4 interfaces, from the system tools."""
    if os.name == "nt":
        return _parse_ipconfig(_tool_output(["ipconfig"]))
    found = [] if sys.platform == "darwin" else _parse_ip_addr(_tool_output(["ip", "-4", "-o", "addr", "show"]))
    return found or _parse_ifconfig(_tool_output(["ifconfig"]))


def _local_ipv4_guesses():
    """This computer's IPv4 addresses as far as the socket API tells (no netmask)."""
    ips = set()
    try:
        for info in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
            ips.add(info[4][0])
    except OSError:
        pass
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            s.connect(("10.255.255.255", 1))
            ips.add(s.getsockname()[0])
        finally:
            s.close()
    except OSError:
        pass
    return sorted(ip for ip in ips if not ip.startswith("127."))


def _broadcast_addresses():
    """255.255.255.255 plus the broadcast address of every local IPv4 network, from the real
    netmasks; a /24 guess only for addresses the system tools did not report."""
    addrs = {"255.255.255.255"}
    nets = []
    try:
        for ip, bits in _interface_networks():
            try:
                iface = ipaddress.IPv4Interface("%s/%d" % (ip, bits))
            except ValueError:
                continue
            if bits < 31 and not iface.ip.is_loopback and not iface.ip.is_unspecified:
                nets.append(iface.network)
                addrs.add(str(iface.network.broadcast_address))
    except Exception:  # odd tool output must never break discovery
        pass
    for ip in _local_ipv4_guesses():
        try:
            if not any(ipaddress.IPv4Address(ip) in net for net in nets):
                addrs.add(ip.rsplit(".", 1)[0] + ".255")
        except ValueError:
            pass
    return sorted(addrs)


def discover(timeout=2.0):
    """Broadcast on the local network for Lite6 gateways. Returns a list of dicts."""
    found = {}
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        s.settimeout(0.3)
    except OSError:
        return []
    try:
        for addr in _broadcast_addresses():
            try:
                s.sendto(b"LITE6_DISCOVER", (addr, DISCOVERY_PORT))
            except OSError:
                pass
        end = time.time() + timeout
        while time.time() < end:
            try:
                data, (ip, _) = s.recvfrom(2048)
            except socket.timeout:
                continue
            except OSError:
                break
            try:
                info = json.loads(data.decode("utf-8"))
            except (ValueError, UnicodeError):
                continue
            if isinstance(info, dict) and info.get("lite6"):
                info["host"] = ip
                found[ip] = info
    finally:
        s.close()
    return list(found.values())


# --------------------------------------------------------------------------- profiles

def connect(profile=None, host=None, quiet=False, interactive=False):
    """Gateway object from a saved login (see `lite6 login`)."""
    cfg = load_config()
    name = profile or os.environ.get("LITE6_PROFILE") or cfg.get("current") or "default"
    prof = dict(cfg["profiles"].get(name) or {})
    if not host and not profile:  # an explicit --host or --profile wins over $LITE6_HOST
        host = os.environ.get("LITE6_HOST")
    if host:
        h, p = split_hostport(host)
        explicit_port = ":" in host.split("]")[-1]
        if prof.get("host") != h:
            same = [n for n, pr in cfg["profiles"].items() if pr.get("host") == h]
            name, prof = (same[0], dict(cfg["profiles"][same[0]])) if same else (None, {"host": h, "port": p})
        if explicit_port:
            prof["port"] = p
    if not prof.get("host"):
        raise Lite6Error("not logged in yet. Run:  python lite6.py login <jetson-ip>")
    token = os.environ.get("LITE6_TOKEN") or prof.get("token")
    gw = Gateway(prof["host"], prof.get("port", DEFAULT_PORT), prof.get("fingerprint"), token,
                 profile=name if name in cfg["profiles"] else None, quiet=quiet, interactive=interactive)
    if not gw.fingerprint:
        raise Lite6Error("%s is not trusted yet. Run:  python lite6.py login %s" % (gw.host, gw.host))
    return gw


# --------------------------------------------------------------------------- CLI commands

def _gw(a, quiet=False):
    return connect(a.profile, a.host, quiet=quiet, interactive=True)


def _norm_fp(fp):
    return (fp or "").replace(":", "").replace(" ", "").upper()


def _new_profile_name(host, profiles):
    name, n = host, 2
    while name in profiles:
        name, n = "%s-%d" % (host, n), n + 1
    return name


def _printable(value):
    """Text from an unauthenticated source, safe to print on a terminal."""
    return re.sub(r"[\x00-\x1f\x7f-\x9f]", "?", "%s" % (value,))


def cmd_login(a):
    cfg = load_config()
    host, port = split_hostport(a.host)
    status, _, data, fp = Gateway(host, port)._raw("GET", "/api/info", pin=False, timeout=10)
    try:
        info = json.loads(data.decode("utf-8"))
    except (ValueError, UnicodeError):
        info = {}
    if status != 200 or not isinstance(info, dict) or not info.get("lite6"):
        raise Lite6Error("%s:%d answered, but it is not a Lite6 gateway" % (host, port))
    profiles = cfg["profiles"]
    by_fp = sorted(n for n, p in profiles.items() if p.get("fingerprint") == fp)
    by_host = sorted(n for n, p in profiles.items() if p.get("host") == host)
    name = a.profile or (by_fp or by_host or [None])[0] or \
        ("default" if "default" not in profiles else _new_profile_name(host, profiles))
    if a.fingerprint:
        if _norm_fp(a.fingerprint) != _norm_fp(fp):
            raise CertMismatch("fingerprint mismatch: you expected %s but the gateway shows %s" % (a.fingerprint, fp))
    elif not by_fp:  # a certificate this computer has never trusted
        changed = [n for n in by_host if profiles[n].get("fingerprint")]
        if changed and not a.reset:
            raise CertMismatch("the certificate of %s changed (profile '%s' trusted %s, now %s). If you "
                               "re-installed the gateway, run the login again with --reset." %
                               (host, changed[0], profiles[changed[0]]["fingerprint"], fp))
        say("gateway '%s' (version %s) at %s:%d" % (_printable(info.get("name")), _printable(info.get("version")),
                                                    host, port))
        say("certificate fingerprint: %s" % fp)
        say("it should match the fingerprint shown on the Jetson by:  sudo lite6-info")
        if changed:
            say("this replaces the certificate trusted by profile '%s'" % changed[0])
        trusted = [n for n in sorted(profiles) if profiles[n].get("fingerprint")]
        if trusted and not changed and not a.reset:
            say("WARNING: this is a DIFFERENT certificate than the gateway(s) you already trust:")
            for n in trusted:
                say("  profile %-14s %-16s %s" % (n, profiles[n].get("host"), profiles[n]["fingerprint"]))
            say("fine for a new or re-installed Jetson; if you expected one of the above, something else "
                "is answering at %s - do not continue" % host)
            if a.password_stdin or not sys.stdin or not sys.stdin.isatty():
                raise CertMismatch("not trusted: %s has a certificate you never trusted before. Compare it with "
                                   "`sudo lite6-info` on the Jetson, then log in again with --fingerprint <fp> "
                                   "or --reset" % host)
            if input("Type 'yes' to trust this different certificate: ").strip().lower() != "yes":
                raise Lite6Error("not trusted - nothing saved")
        elif not a.yes:
            if input("Trust this gateway? [y/N] ").strip().lower() not in ("y", "yes"):
                raise Lite6Error("not trusted - nothing saved")
    if a.password_stdin:
        password = sys.stdin.readline().rstrip("\r\n")
    else:
        password = os.environ.get("LITE6_PASSWORD") or getpass.getpass("Gateway password: ")
    gw = Gateway(host, port, fp)
    result = gw.login(password)
    prof = profiles.setdefault(name, {})
    prof.update(host=host, port=port, fingerprint=fp, token=gw.token, expires=result.get("expires"),
                name=result.get("name"))
    if a.ssh_user:
        prof["ssh_user"] = a.ssh_user
    cfg["current"] = name
    save_config(cfg)
    say("logged in to '%s' - valid until %s (profile '%s' in %s)" % (
        result.get("name"), fmt_time(result.get("expires")), name, CONFIG_PATH))
    say("next:  python lite6.py run your_script.py")
    return 0


def cmd_logout(a):
    cfg = load_config()
    name = a.profile or cfg.get("current") or "default"
    prof = cfg["profiles"].get(name)
    if not prof:
        say("not logged in")
        return 0
    try:
        connect(name).request("POST", "/api/logout")
    except Lite6Error:
        pass
    prof.pop("token", None)
    prof.pop("expires", None)
    save_config(cfg)
    say("logged out (profile '%s'; the gateway stays trusted)" % name)
    return 0


def cmd_status(a):
    gw = _gw(a)
    st = gw.status()
    g, r, j, rel = st["gateway"], st["robot"], st["jobs"], st["relay"]
    print("Gateway  %s  v%s  up %s  (%s:%d)" % (g["name"], g["version"], _dur(g["uptime_s"]), gw.host, gw.port))
    print("         addresses: %s" % ", ".join("%s (%s)" % (x["addr"], x["iface"]) for x in g["addresses"]))
    print("Robot    %s  %s" % (r["ip"], "reachable (%.0f ms)" % r["latency_ms"] if r["reachable"]
                                else "NOT reachable: %s" % r.get("error")))
    for line in _safety_lines(st.get("safety")):
        print(line)
    running = j.get("running")
    print("Job      %s" % ("%s  %s  running %s" % (running["id"], running["entry"], _dur(running["duration_s"]))
                           if running else "none running"))
    for qj in j.get("queued", []):
        print("         queued: %s  %s" % (qj["id"], qj["entry"]))
    if j.get("halted"):
        print("Queue    HALTED after emergency stop (since %s, by %s) - run `lite6 resume`" %
              (fmt_time(j.get("halted_since")), j.get("halted_by") or "?"))
    if rel.get("enabled"):
        you = rel.get("you_allowed_until")
        print("Relay    ports %s; you: %s; %d active connection(s)" % (
            ",".join(str(p) for p in rel.get("ports", [])),
            "allowed until %s" % fmt_time(you) if you else "not allowed (run `lite6 studio` or `lite6 relay`)",
            rel.get("active_connections", 0)))
    if a.robot:
        _print_robot(gw.robot(detail=True))
    return 0


def _safety_lines(sf):
    """`lite6 status` lines for the robot limits and the command guard (newer gateways)."""
    if not sf:
        return []
    lim, guard = sf.get("limits") or {}, sf.get("guard") or {}
    if not lim.get("enabled"):
        first = "limits not managed by the gateway"
    elif lim.get("ok"):
        v = lim.get("limits") or {}
        first = "limits on: max %s mm/s, %s deg/s, box %s mm %s, collision sensitivity %s (checked %s)" % (
            v.get("max_tcp_speed"), v.get("max_joint_speed"), v.get("box"), "on" if v.get("box_on") else "OFF",
            v.get("collision_sensitivity"), fmt_time(lim.get("at")))
    elif lim.get("ok") is None:
        first = "limits: %s" % lim.get("message")
    else:
        first = "limits NOT confirmed: %s - jobs do not start" % lim.get("message")
    lines = ["Safety   " + first]
    lines += ["         WARNING: %s" % w for w in lim.get("warnings") or []]
    if guard.get("enabled"):
        c = guard.get("counts") or {}
        lines.append("         command guard on ports %s: %d refused, %d clamped so far" % (
            ",".join(str(p) for p in guard.get("ports") or []) or "NONE (not listening!)",
            c.get("refused", 0), c.get("clamped", 0)))
    else:
        lines.append("         command guard OFF")
    return lines


def _dur(seconds):
    if seconds is None:
        return "-"
    seconds = int(seconds)
    if seconds < 60:
        return "%ds" % seconds
    if seconds < 3600:
        return "%dm%02ds" % (seconds // 60, seconds % 60)
    return "%dh%02dm" % (seconds // 3600, (seconds % 3600) // 60)


def _print_robot(result):
    d = result.get("detail") or {}
    if not d.get("ok"):
        print("Robot detail: %s" % d.get("message"))
        return
    info = d.get("detail") or {}
    for key in ("version", "sn", "axis", "state", "mode", "error_warn", "angles_deg", "position",
                "motor_enable_states"):
        print("  %-20s %s" % (key, info.get(key)))


def cmd_robot(a):
    _print_robot(_gw(a).robot(detail=True))
    return 0


def cmd_run(a):
    gw = _gw(a, quiet=a.quiet)
    target = a.target
    if a.target == "-":
        raise Lite6Error("reading the script from stdin is not supported; pass a file or folder")
    if a.script_args[:1] == ["--"]:   # `lite6 run x.py -- --flag` (older argparse keeps the --)
        a.script_args = a.script_args[1:]
    job = gw.submit(target, args=a.script_args, entry=a.entry, with_files=a.with_files,
                    timeout=a.timeout, label=a.label, queue_ok=not a.no_queue,
                    excludes=a.exclude + _results_excludes(target, a.out))
    if not a.quiet:
        pos = job.get("queue_position")
        say("job %s submitted: %s %s%s" % (job["id"], job["entry"], " ".join(job["args"]),
                                           "  (waiting, position %d in the queue)" % pos
                                           if pos and pos > 1 else ""))
    if a.detach:
        print(job["id"])
        return 0
    results_dir = os.path.join(a.out, job["id"])
    count = [0]

    def on_data(record):
        count[0] += 1

    final = gw.follow(job["id"], on_data=on_data, results_dir=results_dir, show_data=a.show_data)
    if final is None:
        return 130
    status, code = final.get("status"), final.get("exit_code")
    if not a.no_download:
        try:
            gw.download_results(job["id"], results_dir)
            files = [f for f in gw.files(job["id"])]
            if not a.quiet:
                say("results: %s  (%d file(s)%s)" % (
                    results_dir, len(files), ", %d live-data record(s) in live_data.jsonl" % count[0]
                    if count[0] else ""))
        except Lite6Error as e:
            say("could not download results: %s" % e)
    if not a.quiet:
        rs = final.get("robot_stop")
        say("job %s: %s (%s)%s" % (final["id"], status, final.get("message"),
                                   "; robot stop: %s" % rs.get("message") if rs else ""))
    if status == "succeeded":
        return 0
    if isinstance(code, int) and 0 < code < 256:
        return code
    return 130 if status == "stopped" else 1


def cmd_jobs(a):
    gw = _gw(a)
    jobs = gw.jobs(limit=a.n)
    if not jobs:
        print("no jobs yet")
        return 0
    print("%-20s %-10s %-19s %8s  %s" % ("JOB", "STATUS", "CREATED", "TIME", "ENTRY / LABEL"))
    for j in jobs:
        print("%-20s %-10s %-19s %8s  %s%s" % (j["id"], j["status"], fmt_time(j.get("created")),
                                             _dur(j.get("duration_s")), j.get("entry"),
                                             ("  [%s]" % j["label"]) if j.get("label") else ""))
    return 0


def cmd_logs(a):
    gw = _gw(a)
    job_id = gw.resolve_job(a.job)
    if a.follow:
        final = gw.follow(job_id)
        if final:
            say("job %s: %s (%s)" % (job_id, final["status"], final.get("message")))
        return 0
    offset = 0
    decoder = codecs.getincrementaldecoder("utf-8")("replace")
    out = OutputFilter(write_out, show_data=True)
    while True:
        data, offset, size, status = gw.read_log(job_id, offset, max_bytes=4 * 1024 * 1024)
        out.feed(decoder.decode(data))
        if not data or offset >= size:
            break
    out.feed(decoder.decode(b"", final=True))
    out.flush()
    return 0


def cmd_get(a):
    gw = _gw(a)
    job_id = gw.resolve_job(a.job)
    dest = os.path.join(a.out, job_id)
    gw.download_results(job_id, dest)
    say("downloaded results of %s into %s" % (job_id, dest))
    return 0


def cmd_stop(a):
    gw = _gw(a)
    job_id = gw.resolve_job(a.job or "running")
    job = gw.stop(job_id, force=a.force)
    say("%s requested for job %s (status now: %s)" % ("kill" if a.force else "stop", job_id, job["status"]))
    return 0


def cmd_estop(a):
    gw = _gw(a)
    r = gw.estop()
    rs = r.get("robot_stop") or {}
    say("robot stop: %s - %s" % ("OK" if rs.get("ok") else "FAILED", rs.get("message")))
    if r.get("job_stopped"):
        say("killed job %s" % r["job_stopped"])
    if r.get("queued_cancelled"):
        say("cancelled %d queued job(s)" % r["queued_cancelled"])
    if r.get("relay_connections_closed"):
        say("closed %d relay connection(s)" % r["relay_connections_closed"])
    if r.get("halted", True):
        say("the queue is now HALTED: new jobs and relay access are refused until `lite6 resume` "
            "(or Resume on the web page)")
    say("before moving again: clear errors and re-enable (motion_enable(True), set_mode(0), set_state(0))")
    return 0 if rs.get("ok") else 1


def cmd_resume(a):
    gw = _gw(a)
    try:
        r = gw.resume()
    except GatewayError as e:
        if e.status == 404:
            raise Lite6Error("this gateway does not know `resume` (older version): it does not halt after "
                             "an emergency stop, so there is nothing to resume")
        raise
    if r.get("halted"):
        raise Lite6Error("the gateway is still halted: %s" % (r.get("message") or r))
    say("resumed: the gateway accepts new jobs (and relay access) again")
    say("the robot itself stays stopped until something re-enables it: clean_error(), motion_enable(True), "
        "set_state(0) (lite6job.connect() does this at the start of a job)")
    return 0


def cmd_delete(a):
    gw = _gw(a)
    job_id = gw.resolve_job(a.job)
    gw.delete(job_id)
    say("deleted job %s" % job_id)
    return 0


def cmd_studio(a):
    gw = _gw(a)
    r = gw.relay_allow()
    url = "http://%s:%d/" % (gw.host, STUDIO_PORT)
    say("relay open for this computer (%s) until %s" % (r.get("allowed_ip"), fmt_time(r.get("until"))))
    say("UFactory Studio: %s" % url)
    if not a.no_browser:
        webbrowser.open(url)
    return 0


def cmd_relay(a):
    gw = _gw(a)
    if a.off:
        r = gw.relay_revoke()
        say("relay access removed for %s (%d connection(s) closed)" % (r.get("revoked_ip"),
                                                                        r.get("connections_closed", 0)))
        return 0
    r = gw.relay_allow()
    say("relay open for this computer (%s) until %s; ports %s" % (
        r.get("allowed_ip"), fmt_time(r.get("until")), ", ".join(str(p) for p in r.get("ports", []))))
    say("use the SDK on this PC with:  arm = XArmAPI('%s')" % gw.host)
    say("UFactory Studio:              http://%s:%d/" % (gw.host, STUDIO_PORT))
    say("note: this is plain TCP over the network - for timing-critical control run the script ON the "
        "Jetson with `lite6 run`")
    return 0


def cmd_find(a):
    found = discover(timeout=a.timeout)
    if not found:
        say("no gateway answered the broadcast (some networks block broadcasts).")
        say("try the hostname (e.g. yahboom.local) or the IP shown by `sudo lite6-info` on the Jetson")
        return 1
    cfg = load_config()
    changed = False
    for g in sorted(found, key=lambda x: x["host"]):
        port = _valid_port(g.get("port", DEFAULT_PORT))
        mine = sorted(n for n, p in cfg["profiles"].items()
                      if p.get("fingerprint") and p.get("fingerprint") == g.get("fingerprint"))
        # the UDP reply is unauthenticated: only a pinned TLS check proves it is our gateway
        if port is None:
            note, mine = "(invalid port in its reply - ignored)", []
        elif mine and Gateway(g["host"], port, cfg["profiles"][mine[0]]["fingerprint"])._probe():
            note = "(trusted: profile %s)" % ", ".join(mine)
        elif mine:
            note, mine = "(claims the certificate of profile %s but failed the TLS check - NOT trusted)" % mine[0], []
        else:
            note = "(not trusted yet)"
        print("%-16s %-18s port %-5s v%-6s %s" % (g["host"], _printable(g.get("name")), _printable(g.get("port")),
                                                  _printable(g.get("version")), note))
        if mine and a.update:
            for n in mine:
                prof = cfg["profiles"][n]
                if (prof.get("host"), prof.get("port", DEFAULT_PORT)) != (g["host"], port):
                    say("profile %s: %s:%s -> %s:%d" % (n, prof.get("host"), prof.get("port", DEFAULT_PORT),
                                                        g["host"], port))
                    prof["host"], prof["port"] = g["host"], port
                    changed = True
    if changed:
        save_config(cfg)
    return 0


def cmd_web(a):
    gw = _gw(a)
    url = "https://%s:%d/" % (gw.host, gw.port)
    say("opening %s (the browser will warn about the self-signed certificate once - fingerprint %s)" %
        (url, gw.fingerprint))
    webbrowser.open(url)
    return 0


def cmd_ssh(a):
    gw = _gw(a)
    cfg = load_config()
    prof = cfg["profiles"].get(gw.profile or "", {})
    user = a.user or prof.get("ssh_user")
    if not user:
        raise Lite6Error("which Linux user on the Jetson? run: lite6 ssh --user <name>  (remembered next time)")
    if gw.profile and prof.get("ssh_user") != user:
        prof["ssh_user"] = user
        save_config(cfg)
    if shutil.which("ssh") is None:
        raise Lite6Error("no `ssh` command found (Windows: Settings > Optional features > OpenSSH Client)")
    cmd = ["ssh"]
    if a.forward:
        robot_ip = gw.status()["robot"]["ip"]
        for port in a.forward:
            cmd += ["-L", "%d:%s:%d" % (port, robot_ip, port)]
            say("forwarding localhost:%d -> robot %s:%d while this ssh session is open" % (port, robot_ip, port))
    cmd.append("%s@%s" % (user, gw.host))
    cmd += a.command or []
    return subprocess.call(cmd)


def cmd_pip(a):
    gw = _gw(a)
    fd, tmp = tempfile.mkstemp(prefix="lite6_pip_", suffix=".py")
    with os.fdopen(fd, "w") as f:
        f.write(PIP_SCRIPT)
    try:
        result = gw.run(tmp, args=a.pip_args, download=False, label="pip " + " ".join(a.pip_args))
    finally:
        os.unlink(tmp)
    return 0 if result.ok else 1


def cmd_passwd(a):
    gw = _gw(a)
    old = getpass.getpass("Current gateway password: ")
    new = getpass.getpass("New gateway password (min 8 chars): ")
    if getpass.getpass("Repeat new password: ") != new:
        raise Lite6Error("passwords do not match")
    r = gw.change_password(old, new)
    say(r.get("message", "password changed"))
    return 0


def cmd_doctor(a):
    ok = True
    cfg = load_config()
    name = a.profile or cfg.get("current")
    prof = cfg["profiles"].get(name) or {}
    host = a.host or prof.get("host")
    print("config      %s (profile %s)" % (CONFIG_PATH, name))
    if not host:
        print("FAIL        no gateway configured - run: python lite6.py login <jetson-ip>")
        return 1
    h, p = split_hostport(host) if a.host else (prof["host"], prof.get("port", DEFAULT_PORT))
    try:
        socket.create_connection((h, p), timeout=4).close()
        print("ok          TCP %s:%d reachable" % (h, p))
    except OSError as e:
        ok = False
        print("FAIL        TCP %s:%d: %s" % (h, p, e))
        print("            - is this PC on the same Wi-Fi as the Jetson? is the Jetson on?")
        print("            - try: python lite6.py find   (or ping %s)" % h)
        return 1
    gw = Gateway(h, p, prof.get("fingerprint"), prof.get("token"), profile=name)
    try:
        status, _, data, fp = gw._raw("GET", "/api/info", timeout=8)
        print("ok          TLS fingerprint %s %s" % (fp, "(trusted)" if prof.get("fingerprint") else "(NOT trusted yet)"))
    except CertMismatch as e:
        print("FAIL        %s" % e)
        return 1
    try:
        st = gw.request("GET", "/api/status")
        print("ok          logged in (token valid until %s)" % fmt_time(prof.get("expires")))
        r = st["robot"]
        if r["reachable"]:
            print("ok          robot %s reachable from the Jetson (%.0f ms)" % (r["ip"], r["latency_ms"]))
        else:
            ok = False
            print("FAIL        robot %s NOT reachable from the Jetson: %s" % (r["ip"], r.get("error")))
            print("            - robot powered on? E-stop released? Ethernet cable between Jetson and control box?")
    except GatewayError as e:
        ok = False
        print("FAIL        %s (HTTP %s) - run: python lite6.py login %s" % (e, e.status, h))
    return 0 if ok else 1


# --------------------------------------------------------------------------- main

class _ArgParser(argparse.ArgumentParser):
    """allow_abbrev=False (the default here, so for every sub-command too): script arguments such
    as `--n 5` must not be taken for abbreviations of lite6's own options. Python < 3.8 then also
    refused `-dq` and `-n5` (bpo-26967), so single-dash options keep their old matching there."""

    def __init__(self, *args, **kwargs):
        kwargs.setdefault("allow_abbrev", False)
        argparse.ArgumentParser.__init__(self, *args, **kwargs)

    def _parse_optional(self, arg_string):
        if sys.version_info < (3, 8) and len(arg_string) > 1 and arg_string[0] in self.prefix_chars \
                and arg_string[1] not in self.prefix_chars:
            old, self.allow_abbrev = self.allow_abbrev, True
            try:
                return argparse.ArgumentParser._parse_optional(self, arg_string)
            finally:
                self.allow_abbrev = old
        return argparse.ArgumentParser._parse_optional(self, arg_string)


def build_parser():
    p = _ArgParser(prog="lite6", description="Run scripts on the Lite6 Jetson gateway.",
                   formatter_class=argparse.RawDescriptionHelpFormatter, allow_abbrev=False,
                   epilog="Ctrl+C during `run`/`logs -f`: 1st = stop job, 2nd = kill, 3rd = detach.")
    p.add_argument("--profile", help="saved gateway profile to use (default: the last one you logged in to)")
    p.add_argument("--host", help="gateway address (overrides the profile), e.g. 10.13.37.101")
    p.add_argument("--version", action="version", version="lite6 client %s" % VERSION)
    sub = p.add_subparsers(dest="cmd", metavar="COMMAND")

    s = sub.add_parser("login", help="trust a gateway and log in (once per computer)")
    s.add_argument("host", help="Jetson address, e.g. 10.13.37.101 or yahboom.local[:port]")
    s.add_argument("--yes", "-y", action="store_true", help="trust the certificate without asking")
    s.add_argument("--fingerprint", help="expected certificate fingerprint (from the Jetson)")
    s.add_argument("--reset", action="store_true", help="accept a changed certificate (after re-install)")
    s.add_argument("--password-stdin", action="store_true", help="read the password from stdin")
    s.add_argument("--ssh-user", help="Linux user on the Jetson (for `lite6 ssh`)")
    s.set_defaults(func=cmd_login)

    s = sub.add_parser("logout", help="forget the login token")
    s.set_defaults(func=cmd_logout)

    s = sub.add_parser("status", help="gateway, robot and queue status")
    s.add_argument("--robot", action="store_true", help="also query the robot controller (state, errors, pose)")
    s.set_defaults(func=cmd_status)

    s = sub.add_parser("robot", help="detailed robot state from the controller")
    s.set_defaults(func=cmd_robot)

    s = sub.add_parser("run", help="upload a script/folder, run it on the Jetson, stream output, get results",
                       description="Everything after TARGET is passed to your script, e.g.\n"
                                   "  lite6 run --timeout 600 move.py --speed 20",
                       formatter_class=argparse.RawDescriptionHelpFormatter)
    s.add_argument("--entry", help="script to run inside a folder/zip (default main.py)")
    s.add_argument("--with", dest="with_files", action="append", metavar="PATH",
                   help="also upload this file/folder (repeatable)")
    s.add_argument("--exclude", action="append", default=[], metavar="PATTERN",
                   help="skip files matching this pattern when uploading a folder (repeatable)")
    s.add_argument("--out", default="lite6_runs", help="where results are saved (default ./lite6_runs)")
    s.add_argument("--timeout", type=int, help="stop the job after this many seconds")
    s.add_argument("--label", help="short note shown in the job list")
    s.add_argument("--no-queue", action="store_true", help="fail instead of waiting if the robot is busy")
    s.add_argument("--detach", "-d", action="store_true", help="submit and return immediately (prints job id)")
    s.add_argument("--no-download", action="store_true", help="do not download result files")
    s.add_argument("--show-data", action="store_true", help="also print @lite6:data lines")
    s.add_argument("--quiet", "-q", action="store_true", help="only the script's own output")
    s.add_argument("target", help="script (.py/.sh), folder or .zip")
    s.add_argument("script_args", nargs=argparse.REMAINDER, help="arguments for your script")
    s.set_defaults(func=cmd_run)

    s = sub.add_parser("jobs", help="list recent jobs")
    s.add_argument("-n", type=int, default=20)
    s.set_defaults(func=cmd_jobs)

    s = sub.add_parser("logs", help="print a job's output (last job by default)")
    s.add_argument("job", nargs="?", default="last")
    s.add_argument("-f", "--follow", action="store_true", help="keep streaming until the job ends")
    s.set_defaults(func=cmd_logs)

    s = sub.add_parser("get", help="download a job's result files")
    s.add_argument("job", nargs="?", default="last")
    s.add_argument("--out", default="lite6_runs")
    s.set_defaults(func=cmd_get)

    s = sub.add_parser("stop", help="stop the running job (or JOB) - like Ctrl+C in the script")
    s.add_argument("job", nargs="?")
    s.add_argument("--force", action="store_true", help="kill immediately")
    s.set_defaults(func=cmd_stop)

    s = sub.add_parser("estop", help="EMERGENCY: stop the robot (set_state 4), kill the job, cancel the queue "
                                     "and halt it until `lite6 resume`")
    s.set_defaults(func=cmd_estop)

    s = sub.add_parser("resume", help="accept new jobs again after an emergency stop")
    s.set_defaults(func=cmd_resume)

    s = sub.add_parser("delete", help="delete a finished job and its files from the Jetson")
    s.add_argument("job")
    s.set_defaults(func=cmd_delete)

    s = sub.add_parser("studio", help="open UFactory Studio through the Jetson")
    s.add_argument("--no-browser", action="store_true")
    s.set_defaults(func=cmd_studio)

    s = sub.add_parser("relay", help="allow this PC to use the xArm SDK / Studio through the Jetson")
    s.add_argument("--off", action="store_true", help="remove the access again")
    s.set_defaults(func=cmd_relay)

    s = sub.add_parser("find", help="look for gateways on this network (UDP broadcast)")
    s.add_argument("--timeout", type=float, default=2.0)
    s.add_argument("--update", action="store_true", help="update saved profiles whose gateway moved")
    s.set_defaults(func=cmd_find)

    s = sub.add_parser("web", help="open the web UI in your browser")
    s.set_defaults(func=cmd_web)

    s = sub.add_parser("ssh", help="ssh into the Jetson (uses your system's ssh)")
    s.add_argument("--user", help="Linux user on the Jetson (remembered)")
    s.add_argument("--forward", type=int, action="append", metavar="PORT",
                   help="also forward this robot port to localhost (e.g. 18333)")
    s.add_argument("command", nargs=argparse.REMAINDER)
    s.set_defaults(func=cmd_ssh)

    s = sub.add_parser("pip", help="install Python packages for jobs on the Jetson, e.g. `lite6 pip install scipy`")
    s.add_argument("pip_args", nargs=argparse.REMAINDER)
    s.set_defaults(func=cmd_pip)

    s = sub.add_parser("passwd", help="change the gateway password")
    s.set_defaults(func=cmd_passwd)

    s = sub.add_parser("doctor", help="check connectivity step by step")
    s.set_defaults(func=cmd_doctor)
    return p


def _tolerant_stdio():
    """print() of text the console cannot encode (e.g. an Arabic job label while Windows output is
    redirected as cp1252) must not crash the CLI."""
    for name in ("stdout", "stderr"):
        stream = getattr(sys, name, None)
        if stream is None or getattr(stream, "errors", None) not in ("strict", "surrogateescape"):
            continue
        try:
            if hasattr(stream, "reconfigure"):
                stream.reconfigure(errors="replace")  # novermin
            elif hasattr(stream, "buffer") and codecs.lookup(stream.encoding).name != "utf-8":
                stream.flush()
                setattr(sys, name, io.TextIOWrapper(stream.buffer, encoding=stream.encoding, errors="replace",
                                                    line_buffering=stream.line_buffering))
        except (AttributeError, LookupError, TypeError, ValueError, OSError):
            pass


def main(argv=None):
    _tolerant_stdio()
    parser = build_parser()
    a = parser.parse_args(argv)
    if not getattr(a, "func", None):
        parser.print_help()
        return 2
    try:
        return a.func(a) or 0
    except KeyboardInterrupt:
        say("interrupted")
        return 130
    except CertMismatch as e:
        say("SECURITY WARNING: %s" % e)
        return 4
    except Lite6Error as e:
        say("error: %s" % e)
        return 3


if __name__ == "__main__":
    sys.exit(main())
