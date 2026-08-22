#!/usr/bin/env python3
"""
webui/server.py — HTTP + SSE front door for the rope-hung horizontal drone.

Stdlib only (plus ``pymavlink``, which lives behind ``mav_link.py``).

    python3 server.py --connect auto
    python3 server.py --connect udpin:0.0.0.0:14550        # MAVProxy case

Design rules (see WEBUI_CONTRACT.md):
  * ``ThreadingHTTPServer``; HTTP handlers never block on MAVLink I/O. They read
    a snapshot (``MavLink.snapshot()``) or post to a queue and return.
  * ``/api/stream`` is Server-Sent Events at ~10 Hz. A phone locking its screen
    must not produce a traceback, so every socket error on that path is
    swallowed.
  * Every JSON response is ``Cache-Control: no-store``.
  * No authentication, binds 0.0.0.0. Trusted network only. See README.md.

``mav_link.py`` / ``log_download.py`` are owned by other modules. They are
imported normally; if they are missing or broken the server still starts and
serves the UI, reporting the import error through the API instead of dying.
"""

from __future__ import annotations

import os

# MAVLink 2 must be selected before pymavlink is imported anywhere in the
# process (DEBUG_FLOAT_ARRAY, id 350, does not exist in MAVLink 1). mav_link.py
# does this too, but whichever module loads first has to get it right.
os.environ.setdefault("MAVLINK20", "1")
os.environ.setdefault("MAVLINK_DIALECT", "common")

import argparse
import json
import math
import mimetypes
import signal
import socket
import sys
import threading
import time
import traceback
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, unquote, urlparse

HERE = os.path.dirname(os.path.abspath(__file__))
STATIC_DIR = os.path.join(HERE, "static")
DEFAULT_OUT_DIR = os.path.join(HERE, "downloads")

# Listening on UDP is the normal setup here: MAVProxy on the pi is given
# --out=udp:<this-host>:14551 and we are the passive side. 14551 rather than
# 14550 deliberately, so QGroundControl can keep its usual port and both run at
# the same time. Use --connect auto for a direct USB board.
DEFAULT_CONNECT = "udpin:0.0.0.0:14551"

# Sibling modules live next to this file; make that work regardless of cwd.
if HERE not in sys.path:
    sys.path.insert(0, HERE)

SSE_INTERVAL = 0.1        # 10 Hz state push
SSE_KEEPALIVE = 15.0      # ": keepalive" comment so idle proxies keep the pipe
MAX_BODY = 256 * 1024

SHUTDOWN = threading.Event()


# --------------------------------------------------------------------------- #
# optional sibling modules
# --------------------------------------------------------------------------- #

MavLink = None
LogDownloader = None
MAV_IMPORT_ERROR = None
LOG_IMPORT_ERROR = None

try:
    from mav_link import MavLink  # type: ignore
except Exception as exc:  # ImportError, SyntaxError, anything
    MAV_IMPORT_ERROR = "mav_link.py unavailable: %s: %s" % (type(exc).__name__, exc)

try:
    from log_download import LogDownloader  # type: ignore
except Exception as exc:
    LOG_IMPORT_ERROR = "log_download.py unavailable: %s: %s" % (type(exc).__name__, exc)


# --------------------------------------------------------------------------- #
# shared runtime context
# --------------------------------------------------------------------------- #

class Context:
    """Everything the request handlers are allowed to touch."""

    def __init__(self, address: str, baud: int, out_dir: str) -> None:
        self.address = address
        self.baud = baud
        self.out_dir = out_dir
        self.link = None
        self.downloader = None
        self.link_error = MAV_IMPORT_ERROR
        self.log_error = LOG_IMPORT_ERROR
        self.started_at = time.time()
        self.sse_clients = 0
        self._sse_lock = threading.Lock()

    # -- link lifecycle ----------------------------------------------------- #

    def start_link(self) -> None:
        """Build MavLink + LogDownloader. Never raises; records errors instead."""
        if MavLink is None:
            log("!! %s" % self.link_error)
            log("!! telemetry disabled; static UI and /api/* error reporting still work")
            return

        # LogDownloader needs the link, and the link wants the downloader's
        # handler at construction time. Break the cycle with a forwarder that
        # resolves late.
        def log_handler(msg) -> bool:
            d = self.downloader
            if d is None:
                return False
            try:
                return bool(d.handle(msg))
            except Exception:
                traceback.print_exc()
                return False

        try:
            self.link = self._construct_link(log_handler)
        except Exception as exc:
            self.link = None
            self.link_error = "MavLink(%r) failed: %s: %s" % (
                self.address, type(exc).__name__, exc)
            log("!! %s" % self.link_error)
            traceback.print_exc()
            return

        if LogDownloader is None:
            log("!! %s" % self.log_error)
        else:
            try:
                self.downloader = LogDownloader(self.link, self.out_dir)
            except Exception as exc:
                self.log_error = "LogDownloader failed: %s: %s" % (
                    type(exc).__name__, exc)
                log("!! %s" % self.log_error)

        try:
            self.link.start()
        except Exception as exc:
            self.link_error = "MavLink.start() failed: %s: %s" % (
                type(exc).__name__, exc)
            log("!! %s" % self.link_error)
            traceback.print_exc()

    def _construct_link(self, log_handler):
        """
        Hand ``LogDownloader.handle`` to MavLink through its ``message_handlers``
        constructor argument, so log traffic is consumed by the single receive
        thread. Falls back to attribute injection for older signatures.
        """
        try:
            return MavLink(self.address, baud=self.baud,
                           message_handlers=[log_handler])
        except TypeError:
            pass  # signature without message_handlers
        link = MavLink(self.address, baud=self.baud)
        handlers = getattr(link, "message_handlers", None)
        if isinstance(handlers, list):
            handlers.append(log_handler)
        else:
            try:
                link.message_handlers = [log_handler]
            except Exception:
                log("!! MavLink has no message_handlers hook; log download "
                    "messages will not be consumed")
        return link

    def stop_link(self) -> None:
        if self.downloader is not None:
            try:
                self.downloader.cancel()
            except Exception:
                pass
        if self.link is not None:
            try:
                self.link.stop()
            except Exception:
                traceback.print_exc()

    # -- state -------------------------------------------------------------- #

    def snapshot(self) -> dict:
        if self.link is None:
            return degraded_state(self.address, self.link_error or "link unavailable")
        try:
            state = self.link.snapshot()
        except Exception as exc:
            return degraded_state(self.address, "snapshot failed: %s: %s" % (
                type(exc).__name__, exc))
        if not isinstance(state, dict):
            return degraded_state(self.address, "snapshot() did not return a dict")
        return state

    def sse_enter(self) -> int:
        with self._sse_lock:
            self.sse_clients += 1
            return self.sse_clients

    def sse_exit(self) -> int:
        with self._sse_lock:
            self.sse_clients = max(0, self.sse_clients - 1)
            return self.sse_clients


CTX: Context = None  # set in main()


def log(msg: str) -> None:
    sys.stderr.write("%s\n" % msg)
    sys.stderr.flush()


# --------------------------------------------------------------------------- #
# json helpers
# --------------------------------------------------------------------------- #

def json_safe(obj):
    """
    Recursively replace non-finite floats with None.

    The contract forbids bare NaN/Infinity on the wire: they are not valid JSON
    and ``JSON.parse`` throws, which would take the whole UI down.
    """
    if obj is None or isinstance(obj, (str, bool, int)):
        return obj
    if isinstance(obj, float):
        return obj if math.isfinite(obj) else None
    if isinstance(obj, dict):
        return {(k if isinstance(k, str) else str(k)): json_safe(v)
                for k, v in obj.items()}
    if isinstance(obj, (list, tuple, set)):
        return [json_safe(v) for v in obj]
    if isinstance(obj, (bytes, bytearray)):
        return obj.decode("utf-8", "replace")
    return str(obj)


def dump_json(obj) -> bytes:
    try:
        return json.dumps(json_safe(obj), allow_nan=False).encode("utf-8")
    except Exception as exc:
        # Last resort: never emit malformed JSON.
        return json.dumps({"ok": False,
                           "error": "serialisation failed: %s" % exc}).encode("utf-8")


def degraded_state(address: str, error: str) -> dict:
    """A full STATE-shaped object so the frontend can render 'disconnected'."""
    return {
        "t": time.time(),
        "connected": False,
        "address": address,
        "link": {"packets": 0, "drops": 0, "last_rx_age": None},
        "armed": False,
        "arm_state_age": 9999.0,
        "mode": "NO LINK",
        "attitude": {"roll": None, "pitch": None, "yaw": None,
                     "rollspeed": None, "pitchspeed": None, "yawspeed": None,
                     "age": 9999.0},
        "position": {"x": None, "y": None, "z": None,
                     "vx": None, "vy": None, "vz": None, "age": 9999.0},
        "body_vel": {"fwd": None, "right": None},
        "height": {"agl": None, "valid": False, "age": 9999.0},
        "battery": {"voltage": None, "current": None, "remaining": None},
        "ir": {"valid": False, "dx_px": None, "dy_px": None,
               "angle_x": None, "angle_y": None,
               "offset_fwd": None, "offset_right": None,
               "spot_id": 0, "spot_score": 0, "age": 9999.0,
               "via": "DEBUG_FLOAT_ARRAY"},
        "debug_arrays": {},
        "params": {},
        "messages": [],
        "ack": None,
        "error": error,
    }


# --------------------------------------------------------------------------- #
# static files
# --------------------------------------------------------------------------- #

MIME = {
    ".html": "text/html; charset=utf-8",
    ".htm": "text/html; charset=utf-8",
    ".js": "text/javascript; charset=utf-8",
    ".mjs": "text/javascript; charset=utf-8",
    ".css": "text/css; charset=utf-8",
    ".json": "application/json; charset=utf-8",
    ".map": "application/json; charset=utf-8",
    ".txt": "text/plain; charset=utf-8",
    ".svg": "image/svg+xml",
    ".png": "image/png",
    ".jpg": "image/jpeg",
    ".jpeg": "image/jpeg",
    ".gif": "image/gif",
    ".webp": "image/webp",
    ".ico": "image/vnd.microsoft.icon",
    ".woff": "font/woff",
    ".woff2": "font/woff2",
    ".ttf": "font/ttf",
    ".webmanifest": "application/manifest+json",
    ".ulg": "application/octet-stream",
}


def content_type_for(path: str) -> str:
    ext = os.path.splitext(path)[1].lower()
    if ext in MIME:
        return MIME[ext]
    guess = mimetypes.guess_type(path)[0]
    return guess or "application/octet-stream"


class UnsafePath(ValueError):
    pass


def safe_join(root: str, rel: str) -> str:
    """
    Resolve ``rel`` under ``root`` or raise ``UnsafePath``.

    Rejects, before touching the filesystem: percent-decoded ``..`` anywhere,
    absolute paths, leading ``/`` or ``\\``, drive letters, NUL bytes. Then
    confirms the realpath is still inside ``root`` as a second line of defence
    against symlinks.
    """
    rel = unquote(rel or "")
    if not rel:
        raise UnsafePath("empty path")
    if "\x00" in rel:
        raise UnsafePath("NUL in path")
    norm = rel.replace("\\", "/")
    if norm.startswith("/"):
        raise UnsafePath("absolute path")
    if ".." in norm:
        raise UnsafePath("parent traversal")
    if os.path.isabs(rel) or (len(rel) > 1 and rel[1] == ":"):
        raise UnsafePath("absolute path")
    full = os.path.realpath(os.path.join(root, *[p for p in norm.split("/") if p]))
    root_real = os.path.realpath(root)
    if full != root_real and not full.startswith(root_real + os.sep):
        raise UnsafePath("escapes root")
    return full


# --------------------------------------------------------------------------- #
# request handler
# --------------------------------------------------------------------------- #

QUIET_SOCKET_ERRORS = (BrokenPipeError, ConnectionResetError, ConnectionAbortedError,
                       socket.timeout, TimeoutError)


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"
    server_version = "droneui"
    sys_version = ""
    timeout = 30           # a wedged phone cannot pin a thread forever

    # -- plumbing ----------------------------------------------------------- #

    def log_message(self, fmt, *args):  # noqa: A003 - stdlib signature
        pass  # access logging would drown the console at 10 Hz

    def log_error(self, fmt, *args):
        pass

    def handle_one_request(self):
        try:
            BaseHTTPRequestHandler.handle_one_request(self)
        except QUIET_SOCKET_ERRORS:
            self.close_connection = True
        except OSError:
            self.close_connection = True

    def do_GET(self):
        self._dispatch("GET")

    def do_HEAD(self):
        self._dispatch("HEAD")

    def do_POST(self):
        self._dispatch("POST")

    def _dispatch(self, method: str) -> None:
        """One handler exception must never take the server down."""
        try:
            parsed = urlparse(self.path)
            path = parsed.path
            if len(path) > 1 and path.endswith("/"):
                path = path.rstrip("/") or "/"
            query = parse_qs(parsed.query)
            if method == "POST":
                self._route_post(path)
            else:
                self._route_get(method, path, query)
        except QUIET_SOCKET_ERRORS:
            self.close_connection = True
        except ValueError as exc:          # malformed / oversized request body
            try:
                self.send_json({"ok": False, "error": str(exc)}, 400)
            except Exception:
                self.close_connection = True
        except Exception:
            traceback.print_exc()
            try:
                self.send_json({"ok": False, "error": "internal server error"}, 500)
            except Exception:
                self.close_connection = True

    # -- routing ------------------------------------------------------------ #

    def _route_get(self, method: str, path: str, query: dict) -> None:
        if path in ("/", "/index.html"):
            return self.send_static("index.html")

        if path == "/api/state":
            return self.send_json(CTX.snapshot())

        if path == "/api/stream":
            if method == "HEAD":
                return self.send_json({"ok": True, "sse": True})
            return self.serve_sse()

        if path == "/api/params":
            return self.api_params(query)

        if path == "/api/logs":
            return self.api_logs()

        if path == "/api/downloads":
            return self.api_downloads()

        if path.startswith("/api/logs/file/"):
            return self.api_log_file(path[len("/api/logs/file/"):])

        if path.startswith("/static/"):
            return self.send_static(path[len("/static/"):])

        if path == "/favicon.ico":
            if os.path.isfile(os.path.join(STATIC_DIR, "favicon.ico")):
                return self.send_static("favicon.ico")
            return self.send_bytes(b"", 204, "image/vnd.microsoft.icon")

        if path == "/api/health":
            return self.send_json({
                "ok": True,
                "address": CTX.address,
                "uptime_s": round(time.time() - CTX.started_at, 1),
                "sse_clients": CTX.sse_clients,
                "link_error": CTX.link_error,
                "log_error": CTX.log_error,
            })

        return self.send_json({"ok": False, "error": "not found: %s" % path}, 404)

    def _route_post(self, path: str) -> None:
        if path == "/api/command":
            return self.api_command(self.read_json())
        if path == "/api/param":
            return self.api_param(self.read_json())
        if path == "/api/logs/refresh":
            self.read_json()
            return self.api_logs_refresh()
        if path == "/api/logs/download":
            return self.api_logs_download(self.read_json())
        if path == "/api/logs/cancel":
            self.read_json()
            return self.api_logs_cancel()
        if path == "/api/logs/erase":
            return self.api_logs_erase(self.read_json())
        return self.send_json({"ok": False, "error": "not found: %s" % path}, 404)

    # -- response helpers --------------------------------------------------- #

    def read_json(self) -> dict:
        try:
            length = int(self.headers.get("Content-Length") or 0)
        except (TypeError, ValueError):
            length = 0
        if length <= 0:
            return {}
        if length > MAX_BODY:
            raise ValueError("request body too large (%d bytes)" % length)
        raw = self.rfile.read(length)
        if not raw.strip():
            return {}
        try:
            body = json.loads(raw.decode("utf-8", "replace"))
        except Exception as exc:
            raise ValueError("malformed json body: %s" % exc)
        return body if isinstance(body, dict) else {"value": body}

    def send_bytes(self, body: bytes, status: int = 200,
                   ctype: str = "application/octet-stream",
                   extra: dict = None, cache: str = "no-store") -> None:
        self.send_response(status)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        if cache:
            self.send_header("Cache-Control", cache)
        for key, value in (extra or {}).items():
            self.send_header(key, value)
        self.end_headers()
        if self.command != "HEAD" and body:
            self.wfile.write(body)

    def send_json(self, obj, status: int = 200) -> None:
        self.send_bytes(dump_json(obj), status, "application/json; charset=utf-8")

    def send_static(self, rel: str) -> None:
        try:
            full = safe_join(STATIC_DIR, rel)
        except UnsafePath as exc:
            return self.send_json({"ok": False, "error": "forbidden path (%s)" % exc}, 403)
        if not os.path.isfile(full):
            if rel in ("index.html", ""):
                return self.send_bytes(MISSING_INDEX, 200, "text/html; charset=utf-8")
            return self.send_json({"ok": False, "error": "not found: /static/%s" % rel}, 404)
        # /static/* may be revalidated rather than no-store, per the contract.
        return self.send_file(full, content_type_for(full), cache="no-cache")

    def send_file(self, full: str, ctype: str, cache: str = "no-store",
                  download_name: str = None) -> None:
        try:
            size = os.path.getsize(full)
        except OSError as exc:
            return self.send_json({"ok": False, "error": "cannot stat file: %s" % exc}, 404)
        extra = {}
        if download_name:
            extra["Content-Disposition"] = 'attachment; filename="%s"' % (
                download_name.replace('"', ""))
        self.send_response(200)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(size))
        if cache:
            self.send_header("Cache-Control", cache)
        for key, value in extra.items():
            self.send_header(key, value)
        self.end_headers()
        if self.command == "HEAD":
            return
        try:
            with open(full, "rb") as fh:
                while True:
                    chunk = fh.read(64 * 1024)
                    if not chunk:
                        break
                    self.wfile.write(chunk)
        except QUIET_SOCKET_ERRORS:
            self.close_connection = True

    # -- SSE ---------------------------------------------------------------- #

    def serve_sse(self) -> None:
        """
        ~10 Hz ``event: state`` stream.

        Chunked transfer encoding, written by hand: HTTP/1.1 needs a framing
        rule and we have no Content-Length. Every socket error is swallowed —
        a phone locking its screen is the normal way this ends.
        """
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Connection", "keep-alive")
        self.send_header("X-Accel-Buffering", "no")   # defeat nginx buffering
        self.send_header("Transfer-Encoding", "chunked")
        self.end_headers()

        CTX.sse_enter()
        try:
            self._sse_write(b": connected\n\n")
            self._sse_write(b"retry: 2000\n\n")
            next_tick = time.monotonic()
            last_keepalive = next_tick
            while not SHUTDOWN.is_set():
                payload = dump_json(CTX.snapshot())
                self._sse_write(b"event: state\ndata: " + payload + b"\n\n")
                now = time.monotonic()
                if now - last_keepalive >= SSE_KEEPALIVE:
                    self._sse_write(b": keepalive\n\n")
                    last_keepalive = now
                next_tick += SSE_INTERVAL
                sleep = next_tick - now
                if sleep <= 0:
                    next_tick = now           # fell behind; resync, do not spin
                else:
                    if SHUTDOWN.wait(sleep):
                        break
            try:
                self.wfile.write(b"0\r\n\r\n")
                self.wfile.flush()
            except Exception:
                pass
        except QUIET_SOCKET_ERRORS:
            pass                              # client went away: normal
        except OSError:
            pass
        except Exception:
            traceback.print_exc()
        finally:
            self.close_connection = True
            CTX.sse_exit()

    def _sse_write(self, data: bytes) -> None:
        self.wfile.write(b"%x\r\n" % len(data) + data + b"\r\n")
        self.wfile.flush()

    # -- api: link ---------------------------------------------------------- #

    def require_link(self):
        if CTX.link is None:
            self.send_json({"ok": False,
                            "error": CTX.link_error or "no MAVLink link"}, 503)
            return None
        return CTX.link

    def require_downloader(self):
        if CTX.downloader is None:
            self.send_json({"ok": False,
                            "error": CTX.log_error or "log download unavailable"}, 503)
            return None
        return CTX.downloader

    def api_command(self, body: dict) -> None:
        link = self.require_link()
        if link is None:
            return
        cmd = str(body.get("cmd", "")).strip().lower()
        force = bool(body.get("force", False))
        if cmd == "arm":
            link.arm(force=force)
        elif cmd == "disarm":
            link.disarm(force=force)
        else:
            return self.send_json(
                {"ok": False, "error": "unknown cmd %r (expected arm|disarm)" % cmd},
                400)
        return self.send_json({"ok": True, "cmd": cmd, "force": force})

    def api_params(self, query: dict) -> None:
        link = self.require_link()
        if link is None:
            return
        prefix = (query.get("prefix") or [""])[0]
        try:
            link.request_params(prefix)          # async refresh; never blocks
        except Exception as exc:
            log("!! request_params(%r) failed: %s" % (prefix, exc))
        params = CTX.snapshot().get("params") or {}
        if prefix:
            params = {k: v for k, v in params.items() if str(k).startswith(prefix)}
        return self.send_json({"params": params, "prefix": prefix})

    def api_param(self, body: dict) -> None:
        link = self.require_link()
        if link is None:
            return
        name = str(body.get("name", "")).strip()
        if not name:
            return self.send_json({"ok": False, "error": "missing 'name'"}, 400)
        if "value" not in body:
            return self.send_json({"ok": False, "error": "missing 'value'"}, 400)
        try:
            value = float(body["value"])
        except (TypeError, ValueError):
            return self.send_json(
                {"ok": False, "error": "value %r is not a number" % body.get("value")},
                400)
        if not math.isfinite(value):
            return self.send_json({"ok": False, "error": "value must be finite"}, 400)

        # Safety: no parameter write while armed unless explicitly forced.
        force = bool(body.get("force", False))
        if not force and bool(CTX.snapshot().get("armed")):
            return self.send_json({
                "ok": False,
                "armed": True,
                "error": ("refused: vehicle is ARMED. Parameter writes while armed "
                          "can change control behaviour mid-flight. Disarm first, "
                          'or resend with "force": true.'),
            }, 409)

        link.set_param(name, value)
        return self.send_json({"ok": True, "name": name, "value": value,
                               "forced": force})

    # -- api: logs ---------------------------------------------------------- #

    def api_logs(self) -> None:
        d = CTX.downloader
        if d is None:
            return self.send_json({
                "logs": [], "progress": {"active": False},
                "error": CTX.log_error or "log download unavailable",
            }, 200)
        logs, progress, err = [], {"active": False}, None
        try:
            logs = d.logs() or []
        except Exception as exc:
            err = "logs() failed: %s" % exc
        try:
            progress = d.progress() or {"active": False}
        except Exception as exc:
            err = "progress() failed: %s" % exc
        out = {"logs": logs, "progress": progress}
        if err:
            out["error"] = err
        return self.send_json(out)

    def api_logs_refresh(self) -> None:
        d = self.require_downloader()
        if d is None:
            return
        d.refresh()
        return self.send_json({"ok": True})

    def api_logs_download(self, body: dict) -> None:
        d = self.require_downloader()
        if d is None:
            return
        if "id" not in body:
            return self.send_json({"ok": False, "error": "missing 'id'"}, 400)
        try:
            log_id = int(body["id"])
        except (TypeError, ValueError):
            return self.send_json(
                {"ok": False, "error": "id %r is not an integer" % body.get("id")}, 400)
        d.start(log_id)
        return self.send_json({"ok": True, "id": log_id})

    def api_logs_cancel(self) -> None:
        d = self.require_downloader()
        if d is None:
            return
        d.cancel()
        return self.send_json({"ok": True})

    def api_logs_erase(self, body) -> None:
        """
        Erase every log on the VEHICLE. Irreversible, and it destroys the flight
        data this project debugs from, so it needs an explicit confirm in the
        body rather than being reachable by a stray POST.

        Refused while armed: this writes to the SD card the logger is using.
        """
        d = self.require_downloader()
        if d is None:
            return

        if not isinstance(body, dict) or body.get("confirm") is not True:
            return self.send_json(
                {"ok": False,
                 "error": 'refused: erases ALL logs on the drone and cannot be '
                          'undone. resend with {"confirm": true}.'}, 400)

        if bool(CTX.snapshot().get("armed")):
            return self.send_json(
                {"ok": False, "armed": True,
                 "error": "refused: vehicle is ARMED and logging to that card. "
                          "disarm first."}, 409)

        ok, message = d.erase_all()
        return self.send_json({"ok": bool(ok), "message": message},
                              200 if ok else 502)

    def api_log_file(self, name: str) -> None:
        try:
            rel = unquote(name or "")
            if "/" in rel or "\\" in rel:
                raise UnsafePath("filename only")
            full = safe_join(CTX.out_dir, rel)
        except UnsafePath as exc:
            return self.send_json({"ok": False, "error": "forbidden path (%s)" % exc}, 403)
        if not os.path.isfile(full):
            return self.send_json({"ok": False, "error": "no such file: %s" % rel}, 404)
        return self.send_file(full, content_type_for(full), cache="no-store",
                              download_name=os.path.basename(full))

    def api_downloads(self) -> None:
        files = []
        try:
            for entry in sorted(os.listdir(CTX.out_dir)):
                full = os.path.join(CTX.out_dir, entry)
                if not os.path.isfile(full) or entry.startswith("."):
                    continue
                st = os.stat(full)
                files.append({"name": entry, "size": st.st_size,
                              "mtime": st.st_mtime})
        except FileNotFoundError:
            pass
        except OSError as exc:
            return self.send_json({"files": [], "error": str(exc)})
        files.sort(key=lambda f: f["mtime"], reverse=True)
        return self.send_json({"files": files, "dir": CTX.out_dir})


MISSING_INDEX = (
    "<!doctype html><meta charset=utf-8>"
    "<meta name=viewport content='width=device-width,initial-scale=1'>"
    "<title>webui</title>"
    "<body style='background:#111;color:#eee;font:16px -apple-system,sans-serif;"
    "padding:2rem'>"
    "<h2>server is up</h2>"
    "<p><code>static/index.html</code> is not present yet.</p>"
    "<p>API is live: <a style='color:#4af' href='/api/state'>/api/state</a> &middot; "
    "<a style='color:#4af' href='/api/logs'>/api/logs</a> &middot; "
    "<a style='color:#4af' href='/api/health'>/api/health</a></p>"
    "</body>"
).encode("utf-8")


# --------------------------------------------------------------------------- #
# LAN address discovery
# --------------------------------------------------------------------------- #

def local_ips() -> list:
    """
    Best-effort list of this host's IPv4 addresses, stdlib only.

    The UDP trick finds the address of the interface that would be used to
    reach the internet without sending a packet; hostname resolution catches
    additional interfaces (e.g. a Pi's wlan0 plus a USB-ethernet link).
    """
    found = []

    def add(ip):
        if not ip or ip in found:
            return
        if ip.startswith("169.254.") or ip == "0.0.0.0":
            return
        found.append(ip)

    for probe in ("8.8.8.8", "192.168.1.1", "10.0.0.1"):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.settimeout(0.2)
            sock.connect((probe, 9))       # UDP connect() sends nothing
            add(sock.getsockname()[0])
        except OSError:
            pass
        finally:
            sock.close()

    try:
        host = socket.gethostname()
    except OSError:
        host = None
    if host:
        for resolver in (lambda: socket.gethostbyname_ex(host)[2],
                         lambda: [ai[4][0] for ai in
                                  socket.getaddrinfo(host, None, socket.AF_INET)]):
            try:
                for ip in resolver():
                    if not ip.startswith("127."):
                        add(ip)
            except OSError:
                pass

    return found


def print_banner(host: str, port: int, ctx: Context) -> None:
    log("")
    log("  drone webui  —  http server on %s:%d" % (host, port))
    log("  link     : %s @ %d baud" % (ctx.address, ctx.baud))
    log("  out-dir  : %s" % ctx.out_dir)
    log("  static   : %s%s" % (STATIC_DIR,
                               "" if os.path.isdir(STATIC_DIR) else "  (missing)"))
    log("")
    log("  open one of these on your phone (same wifi):")
    urls = []
    if host in ("0.0.0.0", "", "::"):
        urls.append("http://127.0.0.1:%d/" % port)
        for ip in local_ips():
            urls.append("http://%s:%d/" % (ip, port))
        if len(urls) == 1:
            log("    (no LAN interface found — only loopback)")
    else:
        urls.append("http://%s:%d/" % (host, port))
    for url in urls:
        log("      %s" % url)
    log("")
    if host in ("0.0.0.0", "", "::"):
        log("  !! NO AUTHENTICATION, all interfaces. Trusted network only.")
    if ctx.link_error:
        log("  !! link: %s" % ctx.link_error)
    if ctx.log_error:
        log("  !! logs: %s" % ctx.log_error)
    log("  --help  for flags, MAVProxy setup and the api reference")
    log("  ctrl-c to stop")
    log("")


# Shown by --help. Kept in the program rather than only in the README because in
# the field there is no wifi to go and read one.
MANUAL = """\
  ── flags ────────────────────────────────────────────────────────────────────
    --connect ADDR    where the vehicle is. default: udpin:0.0.0.0:14551
                        udpin:0.0.0.0:14551   LISTEN. the MAVProxy case, DEFAULT
                        auto                  scan USB (cu.usbmodem*, ttyACM*)
                        /dev/cu.usbmodem01    serial, uses --baud
                        udpout:HOST:PORT      we connect out to a listener
                        tcp:HOST:PORT         tcp client
    --baud N          serial baud. default 57600. USB CDC ignores it
    --port N          http port. default 8080
    --host ADDR       http bind. default 0.0.0.0 so a phone can reach it.
                      use 127.0.0.1 to keep it on this machine only
    --out-dir DIR     where downloaded .ulg logs land

  ── connecting to MAVProxy on the pi ─────────────────────────────────────────
    MAVProxy's --out=udp:HOST:PORT means MAVProxy SENDS to that host:port,
    so we are the passive side and must LISTEN:  --connect udpin:0.0.0.0:PORT
    udpout would try to initiate to a listener that is not there, and you would
    see nothing with no error. this is the most common mistake here.

      pi:       mavproxy.py --master=/dev/ttyAMA0 --baudrate 921600 \\
                            --out=udp:<this-host>:14550
      here:     python3 server.py --connect udpin:0.0.0.0:14550

  ── already running QGroundControl? ──────────────────────────────────────────
    a UDP port belongs to ONE process. if QGC holds 14550 we cannot also bind
    it. add a SECOND MAVProxy output instead of moving the first:

      in the MAVProxy console:   output add udp:<this-host>:14551
      here:                      python3 server.py --connect udpin:0.0.0.0:14551

    QGC keeps 14550, we get 14551, both work at once.

  ── nothing showing up? check the link on its own ────────────────────────────
    this bypasses the web server entirely and prints one STATE per second, so
    it separates "no data is reaching this machine" from "the ui is wrong":

      python3 mav_link.py --connect udpin:0.0.0.0:14551 --dump

    a heartbeat gives "connected": true. if it stays false, MAVProxy is not
    sending here -- wrong port, wrong host, or QGC already holds the port.

  ── in the browser ───────────────────────────────────────────────────────────
    ARM and DISARM both act on ONE tap -- there is no confirmation step.
    arming spins props. the button follows the vehicle's real HEARTBEAT, never
    what we asked for, so a refused arm cannot make it read armed. a tap with
    no live heartbeat is ignored rather than queued.
    greyed values are stale (age > 1 s). "--" means the vehicle sent no value,
    not zero.
    parameter writes are refused while armed unless you confirm the override.

  ── api, if you want to script it ────────────────────────────────────────────
    GET  /api/state              one STATE snapshot as json
    GET  /api/stream             same, server-sent events at 10 Hz
    GET  /api/health             uptime, sse client count, import errors
    POST /api/command            {"cmd":"arm"|"disarm","force":false}
    GET  /api/params?prefix=DF_  POST /api/param {"name":..,"value":..}
    GET  /api/logs               POST /api/logs/{refresh,download,cancel}
    GET  /api/logs/file/<name>   GET /api/downloads
"""


def manual_text() -> str:
    """MANUAL with this machine's address filled in, so examples paste cleanly."""
    ips = local_ips()
    return MANUAL.replace("<this-host>", ips[0]) if ips else MANUAL


# --------------------------------------------------------------------------- #
# main
# --------------------------------------------------------------------------- #

def parse_args(argv=None):
    p = argparse.ArgumentParser(
        prog="server.py",
        description="Browser UI + telemetry server for the rope-hung drone. "
                    "No authentication: trusted networks only.",
        epilog=manual_text(),
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--connect", default=DEFAULT_CONNECT, metavar="ADDR",
                   help="udpin:0.0.0.0:PORT | auto | /dev/cu.usbmodem01 | "
                        "udpout:host:port | tcp:host:port  (default: %s)"
                        % DEFAULT_CONNECT)
    p.add_argument("--baud", type=int, default=57600,
                   help="serial baud rate (default: 57600)")
    p.add_argument("--port", type=int, default=8080,
                   help="http port (default: 8080)")
    p.add_argument("--host", default="0.0.0.0",
                   help="http bind address (default: 0.0.0.0 so a phone can reach it)")
    p.add_argument("--out-dir", default=DEFAULT_OUT_DIR, metavar="DIR",
                   help="where downloaded .ulg logs are written (default: %s)"
                        % os.path.relpath(DEFAULT_OUT_DIR, HERE))
    return p.parse_args(argv)


def main(argv=None) -> int:
    global CTX
    args = parse_args(argv)

    out_dir = os.path.abspath(os.path.expanduser(args.out_dir))
    try:
        os.makedirs(out_dir, exist_ok=True)
    except OSError as exc:
        log("!! cannot create --out-dir %s: %s" % (out_dir, exc))

    CTX = Context(args.connect, args.baud, out_dir)
    CTX.start_link()

    ThreadingHTTPServer.allow_reuse_address = True
    ThreadingHTTPServer.daemon_threads = True     # SSE threads must not block exit
    try:
        httpd = ThreadingHTTPServer((args.host, args.port), Handler)
    except OSError as exc:
        log("!! cannot bind %s:%d — %s" % (args.host, args.port, exc))
        CTX.stop_link()
        return 1

    print_banner(args.host, args.port, CTX)

    # Clean shutdown when supervised (systemd, `kill`): release the serial port
    # and stop the link instead of dying with it still open. SIGINT already
    # arrives as KeyboardInterrupt below.
    def on_term(signum, frame):
        log("\nsignal %d — shutting down…" % signum)
        SHUTDOWN.set()
        threading.Thread(target=httpd.shutdown, daemon=True).start()

    try:
        signal.signal(signal.SIGTERM, on_term)
    except (ValueError, OSError, AttributeError):
        pass          # not the main thread, or platform without SIGTERM

    try:
        httpd.serve_forever(poll_interval=0.25)
    except KeyboardInterrupt:
        log("\nshutting down…")
    finally:
        SHUTDOWN.set()
        try:
            httpd.shutdown()
        except Exception:
            pass
        try:
            httpd.server_close()
        except Exception:
            pass
        CTX.stop_link()
    return 0


if __name__ == "__main__":
    sys.exit(main())
