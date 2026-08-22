#!/usr/bin/env python3
"""
Onboard log listing + download over the MAVLink log protocol.

This is the same mechanism QGroundControl uses:

    LOG_REQUEST_LIST  -> a series of LOG_ENTRY  (id, num_logs, time_utc, size)
    LOG_REQUEST_DATA  -> a series of LOG_DATA   (ofs, count, data[90])
    LOG_REQUEST_END   -> stop streaming (sent on completion and on cancel)

The transport is lossy: over UDP (MAVProxy -> laptop) chunks arrive out of
order and some never arrive at all, and PX4 will happily stop streaming while
holes remain.  So the interesting part of this module is *gap handling*:

  * every LOG_DATA is recorded as a received byte range in a sparse structure
    and written straight into `<name>.part` at its own offset (seek + write),
    so out-of-order and duplicated chunks are harmless and memory use is O(1)
    in the log size;
  * if no LOG_DATA arrives for `data_timeout` seconds the still-missing ranges
    are re-requested, repeatedly, until the file is complete or the retry
    budget is exhausted -- at which point `progress()["error"]` explains why;
  * the file is only renamed from `<name>.part` to `<name>` once every byte is
    present, so a finished file in `out_dir` is always a complete file.

Threading contract (see WEBUI_CONTRACT.md):

  * `handle(msg)` is called from somebody else's receive thread.  It does no
    I/O, takes no long-held lock and never sends: it just appends to a deque.
    It returns True only for the messages it consumed (LOG_ENTRY / LOG_DATA)
    so the caller keeps parsing everything else.
  * `start()` / `refresh()` / `cancel()` return immediately; a private daemon
    thread does the requesting, timing out and file writing.
  * every send is done while holding `link.send_lock`.

Self test (no hardware needed):

    python3 log_download.py --selftest        # exit 0 = all checks passed

Manual use against a real vehicle (server.py normally owns this object):

    python3 log_download.py --connect udpin:0.0.0.0:14550 --list
    python3 log_download.py --connect udpin:0.0.0.0:14550 --download 3
"""

import argparse
import os
import sys
import threading
import time
from collections import deque

# DEBUG_FLOAT_ARRAY is message id 350, i.e. MAVLink 2 only, and pymavlink
# defaults to the v1.0 ardupilotmega dialect. The whole webui agrees on this
# dialect, so set it here too -- before importing pymavlink.
os.environ.setdefault("MAVLINK20", "1")
os.environ.setdefault("MAVLINK_DIALECT", "common")

try:
    from pymavlink import mavutil
except ImportError:                                          # pragma: no cover
    sys.exit("pymavlink missing:  pip install -r requirements.txt")


DEFAULT_OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                               "downloads")

LOG_DATA_CHUNK = 90          # bytes per LOG_DATA message, fixed by the protocol
UTC_FMT = "%Y-%m-%d-%H-%M-%S"

# eta_s / rate_bps sentinel. NOT float('inf') and NOT NaN: json.dumps() turns
# those into Infinity/NaN, which are not valid JSON and break JSON.parse().
ETA_UNKNOWN = -1.0


# ---------------------------------------------------------------------------
# sparse received-range bookkeeping (module level so the self test can poke it)
# ---------------------------------------------------------------------------
def _add_range(ranges, start, end):
    """Insert [start, end) into `ranges` (sorted, disjoint, merged) in place."""
    if end <= start:
        return
    out = []
    placed = False
    for s, e in ranges:
        if e < start:                      # strictly before, not even touching
            out.append((s, e))
        elif s > end:                      # strictly after
            if not placed:
                out.append((start, end))
                placed = True
            out.append((s, e))
        else:                              # overlapping or adjacent -> absorb
            start = min(start, s)
            end = max(end, e)
    if not placed:
        out.append((start, end))
    ranges[:] = out


def _missing_ranges(ranges, size):
    """Return the list of [start, end) holes below `size`."""
    holes = []
    pos = 0
    for s, e in ranges:
        if s >= size:
            break
        if s > pos:
            holes.append((pos, min(s, size)))
        pos = max(pos, e)
        if pos >= size:
            break
    if pos < size:
        holes.append((pos, size))
    return holes


def _range_total(ranges):
    return sum(e - s for s, e in ranges)


def log_name(log_id, time_utc):
    """`log_<id>_<utc>.ulg`, or `log_<id>.ulg` when the vehicle has no clock."""
    if time_utc:
        try:
            stamp = time.strftime(UTC_FMT, time.gmtime(int(time_utc)))
            return "log_%d_%s.ulg" % (int(log_id), stamp)
        except (ValueError, OSError, OverflowError):
            pass
    return "log_%d.ulg" % int(log_id)


# ---------------------------------------------------------------------------
class LogDownloader:
    """MAVLink log list + download. See WEBUI_CONTRACT.md for the public API."""

    def __init__(self, link, out_dir=DEFAULT_OUT_DIR,
                 data_timeout=3.0, max_retries=12,
                 rate_window=1.5, gaps_per_round=6, poll=0.02):
        self._link = link
        self.out_dir = out_dir or DEFAULT_OUT_DIR
        self._data_timeout = float(data_timeout)
        self._max_retries = int(max_retries)
        self._rate_window = float(rate_window)
        self._gaps_per_round = int(gaps_per_round)
        self._poll = float(poll)

        self._lock = threading.Lock()

        # log listing
        self._entries = {}                 # id -> {"id","size","utc"}
        self._num_logs = None              # from LOG_ENTRY.num_logs
        self._last_log_num = None
        self._list_thread = None

        # active download
        self._q = deque()                  # (ofs, bytes) filled by handle()
        self._dl_id = None                 # int, read lock-free by handle()
        self._active = False
        self._size = 0
        self._ranges = []                  # sorted disjoint [start, end)
        self._received = 0
        self._error = None
        self._path = None
        self._part = None
        self._final = None
        self._fh = None
        self._gen = 0                      # bumped per start(); stale workers
        self._last_rx = 0.0                # that outlive a cancel are ignored
        self._t_start = 0.0
        self._samples = deque()            # (t, received) moving window
        self._final_rate = 0.0
        self._cancel = threading.Event()
        self._thread = None

        try:
            os.makedirs(self.out_dir, exist_ok=True)
        except OSError as exc:             # pragma: no cover - permissions
            self._error = "cannot create %s: %s" % (self.out_dir, exc)

    # -- sending ------------------------------------------------------------
    def _target(self):
        master = self._link.master
        return master, master.target_system, (master.target_component or 1)

    def _send_list(self, start=0, end=0xFFFF):
        master, ts, tc = self._target()
        with self._link.send_lock:
            master.mav.log_request_list_send(ts, tc, start, end)

    def _send_data(self, log_id, ofs, count):
        master, ts, tc = self._target()
        with self._link.send_lock:
            master.mav.log_request_data_send(ts, tc, int(log_id),
                                             int(ofs), int(count))

    def _send_end(self):
        master, ts, tc = self._target()
        with self._link.send_lock:
            master.mav.log_request_end_send(ts, tc)

    # -- listing ------------------------------------------------------------
    def refresh(self):
        """Ask the vehicle for its log list. Returns immediately."""
        with self._lock:
            self._entries = {}
            self._num_logs = None
            self._last_log_num = None
            alive = self._list_thread is not None and self._list_thread.is_alive()
        if alive:
            return
        th = threading.Thread(target=self._list_worker,
                              name="logdl-list", daemon=True)
        with self._lock:
            self._list_thread = th
        th.start()

    def _list_worker(self):
        """Request the list, then re-request whatever LOG_ENTRYs went missing."""
        try:
            self._send_list()
        except Exception as exc:
            with self._lock:
                self._error = "log list request failed: %s" % exc
            return

        for _ in range(4):
            deadline = time.monotonic() + 2.0
            while time.monotonic() < deadline:
                if self._list_complete():
                    return
                time.sleep(self._poll)
            missing = self._missing_entries()
            if not missing:
                return
            try:
                for log_id in missing[:20]:
                    self._send_list(log_id, log_id)
            except Exception:
                return

    def _list_complete(self):
        with self._lock:
            if self._num_logs is None:
                return False
            return len(self._entries) >= self._num_logs

    def _missing_entries(self):
        with self._lock:
            if self._num_logs is None:
                return []
            if len(self._entries) >= self._num_logs:
                return []
            last = self._last_log_num
            if last is None:
                return []
            return [i for i in range(0, int(last) + 1)
                    if i not in self._entries]

    def logs(self):
        """[{id, size, utc, name, local}] sorted by id."""
        with self._lock:
            entries = list(self._entries.values())
        out = []
        for e in entries:
            name = log_name(e["id"], e["utc"])
            out.append({
                "id": e["id"],
                "size": e["size"],
                "utc": e["utc"],
                "name": name,
                "local": os.path.exists(os.path.join(self.out_dir, name)),
            })
        out.sort(key=lambda d: d["id"])
        return out

    # -- download -----------------------------------------------------------
    def start(self, log_id):
        """Begin downloading `log_id`. Returns immediately."""
        log_id = int(log_id)

        # replace any download already running
        if self._thread is not None and self._thread.is_alive():
            self.cancel()
            self._thread.join(timeout=1.0)

        with self._lock:
            entry = self._entries.get(log_id)
            if entry is None:
                self._active = False
                self._error = ("unknown log id %d -- refresh the log list first"
                               % log_id)
                return
            size = int(entry["size"])
            name = log_name(log_id, entry["utc"])

        if size <= 0:
            with self._lock:
                self._active = False
                self._error = "log %d reports size 0" % log_id
            return

        final = os.path.join(self.out_dir, name)
        part = final + ".part"
        try:
            fh = open(part, "w+b")
        except OSError as exc:
            with self._lock:
                self._active = False
                self._error = "cannot open %s: %s" % (part, exc)
            return

        now = time.monotonic()
        self._cancel.clear()
        with self._lock:
            self._gen += 1
            gen = self._gen
            self._q.clear()
            self._ranges = []
            self._received = 0
            self._size = size
            self._error = None
            self._path = None
            self._part = part
            self._final = final
            self._fh = fh
            self._last_rx = now
            self._t_start = now
            self._samples.clear()
            self._samples.append((now, 0))
            self._final_rate = 0.0
            self._active = True
            self._dl_id = log_id

        self._thread = threading.Thread(target=self._dl_worker,
                                        args=(log_id, gen),
                                        name="logdl-data", daemon=True)
        self._thread.start()

    def cancel(self):
        """Stop the running download. Returns immediately."""
        self._cancel.set()

    def erase_all(self):
        """
        Erase every log ON THE VEHICLE (MAVLink LOG_ERASE). Irreversible.

        Local files in out_dir are untouched -- already-downloaded logs survive.

        There is no acknowledgement in the protocol, so success cannot be
        confirmed directly; the list is re-requested afterwards and going empty
        is the evidence. A download in flight is cancelled first, since erasing
        underneath it would otherwise hand back a corrupt file.

        Returns (ok, message).
        """
        if self._active:
            self.cancel()
            time.sleep(0.2)

        master = getattr(self._link, "master", None)

        if master is None:
            return False, "no link"

        try:
            with self._link.send_lock:
                master.mav.log_erase_send(
                    master.target_system, master.target_component or 1)
        except Exception as exc:                              # noqa: BLE001
            return False, "log_erase failed: %s" % exc

        with self._lock:
            self._entries = {}
            self._num_logs = None
            self._last_log_num = None

        # Erasing takes a moment on the SD card; re-list a little later so the
        # UI reflects reality rather than the pre-erase list.
        threading.Timer(1.5, self.refresh).start()
        return True, "erase sent; re-listing"

    def _dl_worker(self, log_id, gen):
        size = self._size
        try:
            self._send_data(log_id, 0, size)
        except Exception as exc:
            self._finish("log data request failed: %s" % exc, gen)
            return

        retries = 0
        best = 0
        while True:
            if self._cancel.is_set():
                self._finish("cancelled", gen)
                return

            got_new = self._drain(gen)
            if not self._active:               # a write error already aborted us
                return

            if self._received >= size:
                self._complete(gen)
                return

            if got_new and self._received > best:
                best = self._received
                retries = 0                      # progress -> fresh budget

            now = time.monotonic()
            self._sample(now)

            if now - self._last_rx > self._data_timeout:
                retries += 1
                if retries > self._max_retries:
                    holes = _missing_ranges(self._ranges, size)
                    self._finish(
                        "timed out after %d retries: %d/%d bytes, %d gap(s) "
                        "still missing (first at offset %d)"
                        % (self._max_retries, self._received, size, len(holes),
                           holes[0][0] if holes else -1), gen)
                    return
                self._last_rx = now              # restart the timeout clock
                if not self._request_gaps(log_id, size, gen):
                    # nothing missing after all
                    self._complete(gen)
                    return

            time.sleep(self._poll)

    def _request_gaps(self, log_id, size, gen):
        """Re-request the still-missing ranges. False if nothing is missing."""
        with self._lock:
            holes = _missing_ranges(self._ranges, size)
        if not holes:
            return False
        try:
            self._send_end()                     # stop the stale stream first
            for start, end in holes[:self._gaps_per_round]:
                self._send_data(log_id, start, end - start)
        except Exception as exc:
            self._finish("re-request failed: %s" % exc, gen)
            return True
        return True

    def _drain(self, gen):
        """Move queued chunks into the .part file. Returns True if any arrived."""
        got = False
        while True:
            try:
                ofs, blob = self._q.popleft()
            except IndexError:
                break
            got = True
            size = self._size
            if ofs >= size:
                continue
            blob = blob[:size - ofs]
            if not blob:
                continue
            try:
                self._fh.seek(ofs)
                self._fh.write(blob)
            except OSError as exc:               # pragma: no cover - disk full
                self._finish("write failed: %s" % exc, gen)
                return got
            with self._lock:
                _add_range(self._ranges, ofs, ofs + len(blob))
                self._received = _range_total(self._ranges)
        return got

    def _sample(self, now):
        with self._lock:
            self._samples.append((now, self._received))
            cutoff = now - self._rate_window
            while len(self._samples) > 2 and self._samples[0][0] < cutoff:
                self._samples.popleft()

    def _complete(self, gen):
        """All bytes present: truncate, fsync, rename .part -> final."""
        if not self._claim(gen):
            return
        try:
            self._fh.truncate(self._size)
            self._fh.flush()
            try:
                os.fsync(self._fh.fileno())
            except (OSError, ValueError):         # pragma: no cover
                pass
            self._fh.close()
            os.replace(self._part, self._final)
        except OSError as exc:
            self._finish("finalise failed: %s" % exc, gen)
            return
        now = time.monotonic()
        elapsed = max(now - self._t_start, 1e-6)
        with self._lock:
            self._final_rate = self._size / elapsed
            self._received = self._size
            self._active = False
            self._error = None
            self._path = self._final
            self._dl_id = None
        try:
            self._send_end()
        except Exception:
            pass

    def _claim(self, gen):
        """False if start() has since been called again: this worker is stale
        and must not touch the file or the state of the newer download."""
        with self._lock:
            return gen == self._gen

    def _finish(self, error, gen):
        """Abort: close and drop the partial file, record the error."""
        if not self._claim(gen):
            return
        try:
            self._fh.close()
        except Exception:
            pass
        if self._part:
            try:
                os.remove(self._part)
            except OSError:
                pass
        with self._lock:
            self._active = False
            self._error = error
            self._path = None
            self._dl_id = None
        try:
            self._send_end()
        except Exception:
            pass

    # -- progress -----------------------------------------------------------
    def progress(self):
        with self._lock:
            active = self._active
            log_id = self._dl_id if self._dl_id is not None else -1
            received = self._received
            size = self._size
            error = self._error
            path = self._path
            samples = list(self._samples)
            final_rate = self._final_rate

        pct = (100.0 * received / size) if size > 0 else 0.0
        pct = max(0.0, min(100.0, pct))

        if active:
            rate = 0.0
            if len(samples) >= 2:
                dt = samples[-1][0] - samples[0][0]
                db = samples[-1][1] - samples[0][1]
                if dt > 1e-6 and db > 0:
                    rate = db / dt
            remaining = max(size - received, 0)
            eta = (remaining / rate) if rate > 0 else ETA_UNKNOWN
        else:
            rate = final_rate
            eta = 0.0 if (size and received >= size) else ETA_UNKNOWN

        return {
            "active": active,
            "id": log_id,
            "received": received,
            "size": size,
            "pct": round(pct, 2),
            "rate_bps": round(rate, 1),
            "eta_s": round(eta, 1),
            "error": error,
            "path": path,
        }

    # -- receive-thread entry point ----------------------------------------
    def handle(self, msg):
        """Consume LOG_ENTRY / LOG_DATA. Fast, non-blocking, never sends."""
        try:
            mtype = msg.get_type()
        except Exception:
            return False

        if mtype == "LOG_ENTRY":
            try:
                self._on_entry(msg)
            except Exception:
                pass
            return True

        if mtype == "LOG_DATA":
            try:
                self._on_data(msg)
            except Exception:
                pass
            return True

        return False

    def _on_entry(self, msg):
        num_logs = int(getattr(msg, "num_logs", 0) or 0)
        with self._lock:
            self._num_logs = num_logs
            self._last_log_num = int(getattr(msg, "last_log_num", 0) or 0)
            if num_logs == 0:
                return                    # "I have no logs" placeholder entry
            log_id = int(msg.id)
            self._entries[log_id] = {
                "id": log_id,
                "size": int(getattr(msg, "size", 0) or 0),
                "utc": int(getattr(msg, "time_utc", 0) or 0),
            }

    def _on_data(self, msg):
        dl_id = self._dl_id
        if dl_id is None or int(msg.id) != dl_id:
            return                        # stale / not ours, but consumed
        count = int(msg.count)
        if count <= 0:
            return                        # PX4's "nothing there" reply
        data = msg.data
        if isinstance(data, (bytes, bytearray)):
            blob = bytes(data[:count])
        else:
            blob = bytes(bytearray(data[:count]))
        if not blob:
            return
        self._q.append((int(msg.ofs), blob))
        self._last_rx = time.monotonic()


# ===========================================================================
# self test -- no hardware, no network
# ===========================================================================
class _CountingLock:
    """threading.Lock that remembers whether it is currently held."""

    def __init__(self):
        self._lock = threading.Lock()
        self.held = False
        self.acquires = 0

    def acquire(self, *a, **kw):
        got = self._lock.acquire(*a, **kw)
        if got:
            self.held = True
            self.acquires += 1
        return got

    def release(self):
        self.held = False
        self._lock.release()

    def __enter__(self):
        self.acquire()
        return self

    def __exit__(self, *exc):
        self.release()
        return False


class _FakeMav:
    def __init__(self, lock):
        self._lock = lock
        self.sent = []                    # ("list"|"data"|"end", ...)
        self.locked_ok = True

    def _record(self, item):
        if not self._lock.held:
            self.locked_ok = False
        self.sent.append(item)

    def log_request_list_send(self, ts, tc, start, end):
        self._record(("list", start, end))

    def log_request_data_send(self, ts, tc, log_id, ofs, count):
        self._record(("data", log_id, ofs, count))

    def log_request_end_send(self, ts, tc):
        self._record(("end",))


class _FakeMaster:
    target_system = 1
    target_component = 1

    def __init__(self, lock):
        self.mav = _FakeMav(lock)


class _FakeLink:
    def __init__(self):
        self.send_lock = _CountingLock()
        self.master = _FakeMaster(self.send_lock)


def _mk_entry(log_id, size, utc, num_logs=1, last=0):
    return mavutil.mavlink.MAVLink_log_entry_message(log_id, num_logs, last,
                                                     utc, size)


def _mk_data(log_id, ofs, chunk):
    payload = list(chunk) + [0] * (LOG_DATA_CHUNK - len(chunk))
    return mavutil.mavlink.MAVLink_log_data_message(log_id, ofs, len(chunk),
                                                    payload)


def _chunks(payload):
    return [(i, payload[i:i + LOG_DATA_CHUNK])
            for i in range(0, len(payload), LOG_DATA_CHUNK)]


def _wait(pred, timeout=6.0, step=0.005):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if pred():
            return True
        time.sleep(step)
    return pred()


def _selftest():
    import shutil
    import tempfile

    results = []

    def check(name, ok, detail=""):
        results.append(ok)
        print("%-4s %s%s" % ("PASS" if ok else "FAIL", name,
                             ("  <- " + detail) if detail and not ok else ""))

    # deterministic payload, last chunk deliberately partial
    payload = bytes(((i * 37 + (i >> 8) * 11 + 5) & 0xFF)
                    for i in range(LOG_DATA_CHUNK * 11 + 37))
    utc = 1700000000
    log_id = 5
    expected_name = "log_5_%s.ulg" % time.strftime(UTC_FMT, time.gmtime(utc))

    tmp = tempfile.mkdtemp(prefix="logdl-selftest-")
    try:
        # ---- range bookkeeping ------------------------------------------
        r = []
        _add_range(r, 90, 180)
        _add_range(r, 0, 90)                       # adjacent below -> merge
        _add_range(r, 270, 360)
        check("ranges: merge adjacent + keep gap", r == [(0, 180), (270, 360)],
              repr(r))
        _add_range(r, 180, 270)                    # closes the hole
        check("ranges: closing a hole merges all", r == [(0, 360)], repr(r))
        _add_range(r, 100, 200)                    # duplicate
        check("ranges: duplicates are idempotent", r == [(0, 360)], repr(r))
        check("missing_ranges: holes found",
              _missing_ranges([(0, 90), (180, 270)], 400)
              == [(90, 180), (270, 400)])
        check("missing_ranges: complete -> []",
              _missing_ranges([(0, 400)], 400) == [])

        # ---- naming ------------------------------------------------------
        check("name with utc", log_name(5, utc) == expected_name,
              log_name(5, utc))
        check("name without utc (utc == 0)", log_name(7, 0) == "log_7.ulg",
              log_name(7, 0))

        # ---- listing -----------------------------------------------------
        link = _FakeLink()
        dl = LogDownloader(link, tmp, data_timeout=0.25, max_retries=6,
                           rate_window=1.0, poll=0.01)
        dl.refresh()
        check("refresh sent LOG_REQUEST_LIST",
              _wait(lambda: any(s[0] == "list" for s in link.master.mav.sent),
                    2.0))
        check("handle(LOG_ENTRY) -> True",
              dl.handle(_mk_entry(log_id, len(payload), utc, num_logs=1)))
        check("handle(HEARTBEAT) -> False",
              dl.handle(mavutil.mavlink.MAVLink_heartbeat_message(
                  0, 0, 0, 0, 0, 3)) is False)
        entries = dl.logs()
        check("logs() shape",
              len(entries) == 1 and entries[0]["id"] == log_id
              and entries[0]["size"] == len(payload)
              and entries[0]["utc"] == utc
              and entries[0]["name"] == expected_name,
              repr(entries))
        check("logs() local is False before download",
              entries[0]["local"] is False)

        # ---- 1. in-order assembly ----------------------------------------
        dl.start(log_id)
        check("start() requested data from offset 0",
              _wait(lambda: ("data", log_id, 0, len(payload))
                    in link.master.mav.sent, 2.0),
              repr(link.master.mav.sent))
        part_path = os.path.join(tmp, expected_name + ".part")
        check(".part file created while active", os.path.exists(part_path))

        cs = _chunks(payload)
        half = len(cs) // 2
        for ofs, blob in cs[:half]:
            dl.handle(_mk_data(log_id, ofs, blob))
        # let the worker drain and take at least two rate samples
        check("mid-transfer progress advances",
              _wait(lambda: 0 < dl.progress()["received"] < len(payload), 2.0))
        time.sleep(0.08)
        p = dl.progress()
        check("mid-transfer pct sane", 0.0 < p["pct"] < 100.0, repr(p))
        check("mid-transfer rate_bps > 0", p["rate_bps"] > 0.0, repr(p))
        check("mid-transfer eta_s finite and >= 0",
              p["eta_s"] >= 0.0 and p["eta_s"] < 1e9, repr(p))
        check("mid-transfer active/id/error",
              p["active"] is True and p["id"] == log_id
              and p["error"] is None and p["path"] is None, repr(p))

        for ofs, blob in cs[half:]:
            dl.handle(_mk_data(log_id, ofs, blob))
        check("in-order download completes",
              _wait(lambda: not dl.progress()["active"], 5.0),
              repr(dl.progress()))
        p = dl.progress()
        final_path = os.path.join(tmp, expected_name)
        check("no error on completion", p["error"] is None, repr(p))
        check("pct == 100 and received == size",
              p["pct"] == 100.0 and p["received"] == len(payload), repr(p))
        check("final rate_bps > 0 and eta_s == 0",
              p["rate_bps"] > 0.0 and p["eta_s"] == 0.0, repr(p))
        check("progress path is the final file", p["path"] == final_path,
              repr(p["path"]))
        check(".part removed after rename", not os.path.exists(part_path))
        check("final file exists", os.path.exists(final_path))
        with open(final_path, "rb") as fh:
            got = fh.read()
        check("in-order: bytes exact", got == payload,
              "%d bytes vs %d" % (len(got), len(payload)))
        check("LOG_REQUEST_END sent on completion",
              ("end",) in link.master.mav.sent)
        check("logs() local becomes True", dl.logs()[0]["local"] is True)
        os.remove(final_path)

        # ---- 2. out-of-order assembly ------------------------------------
        link2 = _FakeLink()
        dl2 = LogDownloader(link2, tmp, data_timeout=0.25, max_retries=6,
                            poll=0.01)
        dl2.handle(_mk_entry(log_id, len(payload), utc, num_logs=1))
        dl2.start(log_id)
        _wait(lambda: any(s[0] == "data" for s in link2.master.mav.sent), 2.0)
        shuffled = list(cs)
        # deterministic scramble: reverse, then swap neighbours, plus a dup
        shuffled.reverse()
        for i in range(0, len(shuffled) - 1, 2):
            shuffled[i], shuffled[i + 1] = shuffled[i + 1], shuffled[i]
        shuffled.insert(3, shuffled[0])
        for ofs, blob in shuffled:
            dl2.handle(_mk_data(log_id, ofs, blob))
        check("out-of-order download completes",
              _wait(lambda: not dl2.progress()["active"], 5.0),
              repr(dl2.progress()))
        with open(final_path, "rb") as fh:
            got2 = fh.read()
        check("out-of-order: bytes exact", got2 == payload,
              "%d bytes vs %d" % (len(got2), len(payload)))
        check("out-of-order: no error", dl2.progress()["error"] is None,
              repr(dl2.progress()))
        os.remove(final_path)

        # ---- 3. missing middle chunk is detected and re-requested --------
        link3 = _FakeLink()
        dl3 = LogDownloader(link3, tmp, data_timeout=0.25, max_retries=6,
                            poll=0.01)
        dl3.handle(_mk_entry(log_id, len(payload), utc, num_logs=1))
        dl3.start(log_id)
        _wait(lambda: any(s[0] == "data" for s in link3.master.mav.sent), 2.0)
        hole_ofs, hole_blob = cs[3]
        for ofs, blob in cs:
            if ofs == hole_ofs:
                continue                        # this one "gets lost"
            dl3.handle(_mk_data(log_id, ofs, blob))
        check("gap: still active with a hole",
              _wait(lambda: dl3.progress()["received"]
                    == len(payload) - len(hole_blob), 2.0)
              and dl3.progress()["active"] is True, repr(dl3.progress()))
        want = ("data", log_id, hole_ofs, len(hole_blob))
        check("gap: re-requested exactly the missing range",
              _wait(lambda: want in link3.master.mav.sent, 3.0),
              repr([s for s in link3.master.mav.sent if s[0] == "data"]))
        dl3.handle(_mk_data(log_id, hole_ofs, hole_blob))
        check("gap: completes after the re-request is answered",
              _wait(lambda: not dl3.progress()["active"], 5.0),
              repr(dl3.progress()))
        with open(final_path, "rb") as fh:
            got3 = fh.read()
        check("gap: bytes exact after repair", got3 == payload,
              "%d bytes vs %d" % (len(got3), len(payload)))
        check("gap: no error", dl3.progress()["error"] is None,
              repr(dl3.progress()))
        os.remove(final_path)

        # ---- 4. retry budget exhausted -> clean error --------------------
        link4 = _FakeLink()
        dl4 = LogDownloader(link4, tmp, data_timeout=0.05, max_retries=3,
                            poll=0.01)
        dl4.handle(_mk_entry(log_id, len(payload), utc, num_logs=1))
        dl4.start(log_id)
        for ofs, blob in cs[:4]:
            dl4.handle(_mk_data(log_id, ofs, blob))
        check("timeout: gives up instead of hanging",
              _wait(lambda: not dl4.progress()["active"], 5.0),
              repr(dl4.progress()))
        p4 = dl4.progress()
        check("timeout: error mentions the retry budget",
              isinstance(p4["error"], str) and "retries" in p4["error"],
              repr(p4["error"]))
        check("timeout: no final file, no .part left",
              not os.path.exists(final_path)
              and not os.path.exists(part_path))

        # ---- 5. cancel ---------------------------------------------------
        link5 = _FakeLink()
        dl5 = LogDownloader(link5, tmp, data_timeout=5.0, poll=0.01)
        dl5.handle(_mk_entry(log_id, len(payload), utc, num_logs=1))
        dl5.start(log_id)
        for ofs, blob in cs[:2]:
            dl5.handle(_mk_data(log_id, ofs, blob))
        dl5.cancel()
        check("cancel: stops promptly",
              _wait(lambda: not dl5.progress()["active"], 3.0),
              repr(dl5.progress()))
        check("cancel: error == 'cancelled'",
              dl5.progress()["error"] == "cancelled",
              repr(dl5.progress()["error"]))
        check("cancel: .part cleaned up, no final file",
              not os.path.exists(part_path)
              and not os.path.exists(final_path))
        check("cancel: LOG_REQUEST_END sent",
              ("end",) in link5.master.mav.sent)

        # ---- 6. unknown id -----------------------------------------------
        link6 = _FakeLink()
        dl6 = LogDownloader(link6, tmp, poll=0.01)
        dl6.start(99)
        p6 = dl6.progress()
        check("unknown id: inactive with an explanatory error",
              p6["active"] is False and isinstance(p6["error"], str)
              and "unknown log id" in p6["error"], repr(p6))

        # ---- 7. restarting replaces the previous download cleanly --------
        link7 = _FakeLink()
        dl7 = LogDownloader(link7, tmp, data_timeout=0.25, max_retries=6,
                            poll=0.01)
        dl7.handle(_mk_entry(log_id, len(payload), utc, num_logs=1))
        dl7.start(log_id)
        for ofs, blob in cs[:3]:
            dl7.handle(_mk_data(log_id, ofs, blob))
        _wait(lambda: dl7.progress()["received"] > 0, 2.0)
        dl7.start(log_id)                      # restart from scratch
        p7 = dl7.progress()
        check("restart: state reset, still active",
              p7["received"] == 0 and p7["active"] is True
              and p7["error"] is None, repr(p7))
        for ofs, blob in cs:
            dl7.handle(_mk_data(log_id, ofs, blob))
        check("restart: completes",
              _wait(lambda: not dl7.progress()["active"], 5.0),
              repr(dl7.progress()))
        with open(final_path, "rb") as fh:
            got7 = fh.read()
        check("restart: bytes exact", got7 == payload,
              "%d bytes vs %d" % (len(got7), len(payload)))
        os.remove(final_path)

        # ---- 8. every send held send_lock; progress() is JSON-safe -------
        import json
        for name, lk in (("1", link), ("2", link2), ("3", link3),
                         ("4", link4), ("5", link5), ("7", link7)):
            if not lk.master.mav.locked_ok:
                check("all sends held link.send_lock (link %s)" % name, False)
                break
        else:
            check("all sends held link.send_lock", True)
        try:
            json.dumps([d.progress()
                        for d in (dl, dl2, dl3, dl4, dl5, dl6, dl7)],
                       allow_nan=False)
            check("progress() is strict-JSON safe (no NaN/Infinity)", True)
        except ValueError as exc:
            check("progress() is strict-JSON safe (no NaN/Infinity)", False,
                  str(exc))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    passed = sum(1 for ok in results if ok)
    print("\n%d/%d checks passed" % (passed, len(results)))
    return 0 if passed == len(results) else 1


# ===========================================================================
# stand-alone CLI (server.py normally drives LogDownloader instead)
# ===========================================================================
class _CliLink:
    """Minimal stand-in for MavLink so this file is usable on its own."""

    def __init__(self, address, baud=57600):
        import glob
        if address == "auto":
            cands = []
            for pat in ("/dev/cu.usbmodem*", "/dev/tty.usbmodem*",
                        "/dev/ttyACM*"):
                cands += sorted(glob.glob(pat))
            if not cands:
                sys.exit("no USB device found; pass --connect explicitly")
            address = cands[0]
        if address.startswith(("udp", "tcp")):
            self.master = mavutil.mavlink_connection(address)
        else:
            self.master = mavutil.mavlink_connection(address, baud=baud)
        self.send_lock = threading.Lock()
        print("waiting for heartbeat on %s ..." % address)
        self.master.wait_heartbeat(timeout=20)
        print("heartbeat from sys %d comp %d" % (self.master.target_system,
                                                 self.master.target_component))
        self._stop = threading.Event()
        self._sink = None
        threading.Thread(target=self._rx, daemon=True).start()

    def attach(self, downloader):
        self._sink = downloader

    def _rx(self):
        while not self._stop.is_set():
            msg = self.master.recv_match(blocking=True, timeout=0.2)
            if msg is None:
                continue
            if self._sink is not None:
                self._sink.handle(msg)

    def stop(self):
        self._stop.set()


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[1],
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--selftest", action="store_true",
                    help="run the offline self test and exit")
    ap.add_argument("--connect", default="auto")
    ap.add_argument("--baud", type=int, default=57600)
    ap.add_argument("--out-dir", default=DEFAULT_OUT_DIR)
    ap.add_argument("--list", action="store_true", help="list onboard logs")
    ap.add_argument("--download", type=int, metavar="ID")
    args = ap.parse_args(argv)

    if args.selftest:
        return _selftest()

    if not args.list and args.download is None:
        ap.error("nothing to do: pass --selftest, --list or --download ID")

    link = _CliLink(args.connect, args.baud)
    dl = LogDownloader(link, args.out_dir)
    link.attach(dl)
    dl.refresh()
    _wait(lambda: bool(dl.logs()), 10.0)
    time.sleep(2.0)                        # let the rest of the list arrive
    for e in dl.logs():
        print("  id=%-3d %9d B  utc=%-11d %s%s"
              % (e["id"], e["size"], e["utc"], e["name"],
                 "  [local]" if e["local"] else ""))
    if args.download is None:
        link.stop()
        return 0

    dl.start(args.download)
    last = ""
    while True:
        p = dl.progress()
        line = ("  %5.1f%%  %d/%d B  %.1f kB/s  eta %.0fs"
                % (p["pct"], p["received"], p["size"],
                   p["rate_bps"] / 1000.0, max(p["eta_s"], 0.0)))
        if line != last:
            sys.stdout.write("\r" + line)
            sys.stdout.flush()
            last = line
        if not p["active"]:
            print()
            if p["error"]:
                print("error: %s" % p["error"])
                link.stop()
                return 1
            print("saved %s" % p["path"])
            link.stop()
            return 0
        time.sleep(0.2)


if __name__ == "__main__":
    sys.exit(main())
