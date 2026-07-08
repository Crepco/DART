"""Guided-enrollment sessions for the dashboard's capture wizard.

One session at a time (one camera device, one operator). A session accumulates
quality-gated embeddings per pose across the wizard's two 5-pose sets;
/enroll/finish hands them to the existing enrollment pipeline unchanged.

When DART is stopped the session owns the camera device for its whole life —
the wizard's pace is human (reading prompts, glasses on/off), so per-shot
open/close would be slow and flaky on DSHOW. The wizard keep-alive ping plus
the generous idle timeout mean expiry only fires on truly abandoned sessions
(closed tab kills the ping).

No cv2 at module level: app.py imports this eagerly and must stay light.
"""

from __future__ import annotations

import threading
import time
import uuid

POSES          = ("front", "down", "left", "right", "up")
REQUIRED_POSES = ("front", "left", "right")   # directional floor at /enroll/finish:
                                              # lateral (yaw) diversity is required;
                                              # extreme down/up legitimately breaks
                                              # face detection, so those are skippable
IDLE_TIMEOUT_S = 300.0   # wizard pings every ~45s while open, so a live wizard
                         # never gets close — this only reaps abandoned sessions
MAX_ATTEMPTS   = 40      # defensive bound per session


class EnrollSession:
    """One wizard run: per-pose embedding buckets plus the held camera."""

    def __init__(self, name: str, overwrite: bool, camera: int | None, cap=None):
        self.sid = uuid.uuid4().hex[:8]
        self.name = name
        self.overwrite = overwrite
        self.camera = camera
        self.cap = cap          # owned cv2.VideoCapture while DART is stopped, else None
        self.embeddings_by_pose: dict[str, list] = {}
        self.attempts = 0
        self.last_activity = time.monotonic()

    def touch(self):
        self.last_activity = time.monotonic()

    def expired(self) -> bool:
        return time.monotonic() - self.last_activity > IDLE_TIMEOUT_S

    @property
    def accepted(self) -> int:
        return sum(len(v) for v in self.embeddings_by_pose.values())

    def add(self, pose: str, embedding):
        self.embeddings_by_pose.setdefault(pose, []).append(embedding)

    def embeddings(self) -> list:
        return [e for embs in self.embeddings_by_pose.values() for e in embs]

    def missing_required(self) -> list:
        return [p for p in REQUIRED_POSES if not self.embeddings_by_pose.get(p)]

    def release_camera(self):
        cap, self.cap = self.cap, None
        if cap is not None:
            try:
                cap.release()
            except Exception:
                pass


class EnrollSessionManager:
    """Holds the single active session. The device release always happens
    outside the lock — releasing a DSHOW device can be slow."""

    def __init__(self):
        self._lock = threading.Lock()
        self._session: EnrollSession | None = None

    def replace(self, session: EnrollSession):
        """Install `session`; returns the displaced one (camera released)."""
        with self._lock:
            prev, self._session = self._session, session
        if prev is not None:
            prev.release_camera()
        return prev

    def get(self, sid: str):
        """The active session for `sid`, touched — or None. Expired sessions
        are reaped here (camera released), so every /enroll/* call GCs."""
        expired = None
        with self._lock:
            s = self._session
            if s is None or s.sid != sid:
                return None
            if s.expired():
                self._session, expired = None, s
            else:
                s.touch()
                return s
        expired.release_camera()
        return None

    def end(self, sid: str):
        """Remove + release the session for `sid` (finish/cancel). Idempotent;
        returns the removed session or None."""
        with self._lock:
            s = self._session
            if s is None or s.sid != sid:
                return None
            self._session = None
        s.release_camera()
        return s

    def cancel_active(self):
        """Cancel whatever session exists (used by /start and /enroll/begin);
        returns it or None."""
        with self._lock:
            s, self._session = self._session, None
        if s is not None:
            s.release_camera()
        return s
