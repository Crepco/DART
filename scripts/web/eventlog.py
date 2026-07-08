"""In-memory event log feeding the dashboard's System Log window.

A bounded ring buffer of structured events the frontend polls via
GET /logs?since=<seq>. Every event is something that actually happened —
emitters live next to the real transitions (runner loop diffs, route
handlers); nothing here is synthesized for display.

Thread-safety: the buffer has its own lock and is written from both the
runner thread and Flask request threads. emit() must never be called while
holding another lock (e.g. DartRunner._lock) — it takes its own lock and
optionally prints, both of which are safe standalone but deadlock bait when
nested.
"""

from __future__ import annotations

import threading
import time
from collections import deque


class EventLog:
    def __init__(self, maxlen: int = 200):
        self._buf: deque = deque(maxlen=maxlen)
        self._lock = threading.Lock()
        self._seq = 0

    def emit(self, level: str, tag: str, msg: str, echo: bool = True):
        """Record an event. level: "info" | "warn" | "error".

        echo=True also prints "[TAG] msg" to stdout; pass echo=False when the
        caller sits next to an existing print for the same fact (no duplicate
        console lines) or emits at frame-loop cadence.
        """
        with self._lock:
            self._seq += 1
            self._buf.append({"seq": self._seq, "ts": time.time(),
                              "level": level, "tag": tag, "msg": msg})
        if echo:
            print(f"[{tag}] {msg}")

    def since(self, seq: int):
        """(events newer than `seq`, latest seq) — `latest` is the client's
        next cursor even when no events matched."""
        with self._lock:
            return [e for e in self._buf if e["seq"] > seq], self._seq


# Process-wide singleton: the runner thread and Flask threads share one log.
log = EventLog()
