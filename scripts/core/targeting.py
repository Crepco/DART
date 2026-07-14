"""Target lock + aim-point selection (pure logic, no model imports).

Round-3b forensics showed pursuit was stop-start for two reasons: a conf dip
below PERSON_CONF unlocked a target ByteTrack never dropped, and both control
loops refused to aim without a face box every frame. `resolve_lock` keeps an
existing lock through bounded conf dips; `AimSelector` keeps an aim point
through face gaps. Fresh locks are unchanged: full PERSON_CONF + confirmed
UNAUTHORIZED only.
"""

import math

from config import (AIM_REACQUIRE_FRAMES, AIM_STEP_LIMIT, CONTROL_DEBUG,
                    HEAD_PROXY_FRAC, TARGET_HOLD_CONF, TARGET_HOLD_MAX_S)


def select_face(faces: list, prev_cx, prev_cy):
    """Pick the aim face: the one nearest the previous aim point — continuity
    beats size. (The old area-dominant score let any big detection outbid the
    anchored face from across the frame: round-4c hardware showed the aim
    walking 100-270 px toward false positives, the visible 'circle shifts
    rapidly' defect.) The largest face only seeds the first frame, when there
    is no anchor yet."""
    if not faces:
        return None
    if prev_cx is None:
        return max(faces, key=lambda f: (f[2] - f[0]) * (f[3] - f[1]))
    return min(faces, key=lambda f: math.hypot((f[0] + f[2]) / 2 - prev_cx,
                                               (f[1] + f[3]) / 2 - prev_cy))


def resolve_lock(prev_locked, prev_verdict, candidates, tracked, now,
                 hold_started):
    """Pick this frame's locked target id.

    candidates — full-conf UNAUTHORIZED persons [(x1, y1, x2, y2, tid)]. They
    always win (single → it, several → largest box = nearest) and end any
    hold; deliberately including a DIFFERENT person appearing while the held
    target is dipped — a clearly-visible unauthorized person beats a low-conf
    box, and full-conf lock swaps are already today's multi-target policy.

    With no candidate, keep prev_locked through a conf dip while ByteTrack
    still reports it at >= TARGET_HOLD_CONF, its persisted verdict is still
    UNAUTHORIZED (enrollment resets must release the lock), and the hold is
    younger than TARGET_HOLD_MAX_S — never steer on stale low-conf boxes
    indefinitely.

    Returns (locked_id, held, hold_started); hold_started is carried across
    frames by the caller and None whenever no hold is running.
    """
    if candidates:
        if len(candidates) == 1:
            return candidates[0][4], False, None
        best = max(candidates, key=lambda p: (p[2] - p[0]) * (p[3] - p[1]))
        return best[4], False, None

    if (prev_locked is not None
            and prev_verdict == "UNAUTHORIZED"
            and tracked.get(prev_locked, 0.0) >= TARGET_HOLD_CONF):
        started = now if hold_started is None else hold_started
        if now - started <= TARGET_HOLD_MAX_S:
            return prev_locked, True, started

    return None, False, None


class AimSelector:
    """Aim centroid for the locked person: the selected face while one is
    visible, otherwise the person box's head-proxy point corrected by the
    offset learned while faces WERE visible — so the face→proxy handoff is
    jump-free by construction and face flapping cannot oscillate the aim
    (D_KICK_LIMIT stays as backstop only). State is per-lock: a new track id
    resets the learned offset.
    """

    # EMA weight on the previous offset — internal shaping like select_face's
    # 0.4 distance weight, not a tuning surface.
    _OFFSET_SMOOTH = 0.8

    def __init__(self):
        self._tid = None
        self._off = None        # (dx, dy): face centre minus head proxy
        self._last_src = None   # "face" | "proxy", for the CONTROL_DEBUG log
        self._last_pt = None
        self._face_streak = 0   # consecutive frames a face has been present

    def select(self, faces, person_box, track_id, prev_cx, prev_cy):
        """Return the (cx, cy) aim point, or None when there is no locked box."""
        if person_box is None or track_id is None:
            self._tid = self._off = None
            self._last_src = self._last_pt = None
            self._face_streak = 0
            return None
        if track_id != self._tid:
            self._tid = track_id
            self._off = None
            # No continuity across a lock change: the last face belonged to
            # the previous target — never anchor or log a delta against it.
            self._last_src = self._last_pt = None
            self._face_streak = 0

        x1, y1, x2, y2 = person_box
        px = (x1 + x2) / 2.0
        py = y1 + HEAD_PROXY_FRAC * (y2 - y1)

        # Only faces on the locked person may steer the aim: gate candidates
        # to the box inflated by half its size (the same tolerance as the
        # handoff anchor — a blur-shifted box still admits its own face, a
        # false positive or bystander across the frame never does).
        mx, my = (x2 - x1) / 2.0, (y2 - y1) / 2.0
        own_faces = [f for f in faces
                     if x1 - mx <= (f[0] + f[2]) / 2.0 <= x2 + mx
                     and y1 - my <= (f[1] + f[3]) / 2.0 <= y2 + my]
        face = select_face(own_faces, prev_cx, prev_cy)

        # Re-acquisition confirmation window (round-4e feedback loop): after a
        # face gap, a SINGLE flickery re-detection used to trigger a full-speed
        # correction whose own motion blurred the face away again (0.46-0.92 s
        # lose->correct->lose cycles). Hold the anchored proxy until the face
        # has been present AIM_REACQUIRE_FRAMES consecutive frames; flickers
        # shorter than that never move the aim. A fresh lock (no proxy history)
        # aims at its face immediately.
        if face is not None:
            if self._last_src == "proxy":
                self._face_streak += 1
                if self._face_streak < AIM_REACQUIRE_FRAMES:
                    face = None
            else:
                self._face_streak = AIM_REACQUIRE_FRAMES
        else:
            self._face_streak = 0
        if face is not None:
            fx = (face[0] + face[2]) / 2.0
            fy = (face[1] + face[3]) / 2.0
            # Learn the face-minus-proxy offset only from a face inside the
            # locked box — a bystander's face must not poison the correction.
            if x1 <= fx <= x2 and y1 <= fy <= y2:
                dx, dy = fx - px, fy - py
                if self._off is None:
                    self._off = (dx, dy)
                else:
                    s = self._OFFSET_SMOOTH
                    self._off = (s * self._off[0] + (1 - s) * dx,
                                 s * self._off[1] + (1 - s) * dy)
            pt, src = (fx, fy), "face"
        else:
            # Handoff anchor: the first faceless frame re-bases the offset on
            # THIS box, so the face→proxy switch is zero-jump even when the
            # held (low-conf, blur-distorted) box geometry differs from the
            # box the offset was learned on — round-4b fix for a measured
            # 112 px aim teleport that swept the turret past the target.
            # Gate: within the box inflated by half its size — a distorted box
            # often no longer contains the last face point (the defect case),
            # but a bystander across the frame stays far outside.
            if self._last_src == "face" and self._last_pt is not None:
                mx, my = (x2 - x1) / 2.0, (y2 - y1) / 2.0
                if (x1 - mx <= self._last_pt[0] <= x2 + mx
                        and y1 - my <= self._last_pt[1] <= y2 + my):
                    self._off = (self._last_pt[0] - px,
                                 self._last_pt[1] - py)
            if self._off is not None:
                pt = (px + self._off[0], py + self._off[1])
            else:
                pt = (px, py)
            src = "proxy"

        # Rate-limit the aim within a lock: NO source may step it faster than
        # AIM_STEP_LIMIT px/frame. Round-4b hardware: after a long face gap the
        # proxy had drifted 100-375 px off the true head (moving/blurred boxes
        # lie about position), and the face's return teleported the aim — the
        # PID chased the step at the rail, reading as overshoot. A capped ramp
        # covers the same correction smoothly; genuine target motion (p95
        # ~400 px/s) stays untouched.
        if self._last_pt is not None:
            dx = pt[0] - self._last_pt[0]
            dy = pt[1] - self._last_pt[1]
            d = math.hypot(dx, dy)
            if d > AIM_STEP_LIMIT:
                if CONTROL_DEBUG:
                    print(f"[AIM] step clamp {d:5.1f}px -> {AIM_STEP_LIMIT:.0f}px "
                          f"({src}) tid={track_id}")
                k = AIM_STEP_LIMIT / d
                pt = (self._last_pt[0] + dx * k, self._last_pt[1] + dy * k)

        if CONTROL_DEBUG and self._last_src is not None and self._last_src != src:
            d = math.hypot(pt[0] - self._last_pt[0], pt[1] - self._last_pt[1])
            print(f"[AIM] {self._last_src}->{src} d={d:5.1f}px tid={track_id}")
        self._last_src, self._last_pt = src, pt
        return pt
