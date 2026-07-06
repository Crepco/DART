"""Per-track authorization state with debouncing."""

import time
from dataclasses import dataclass, field

from config import (CLASSIFY_EVERY_N_FRAMES, CLASSIFY_EVERY_N_FRAMES_UNCONFIRMED,
                    DEBOUNCE_COUNT, DEBOUNCE_COUNT_INITIAL, TRACK_STATE_GRACE_S)


@dataclass
class TrackState:
    auth_status: str = "UNKNOWN"
    frame_counter: int = 0
    consecutive_count: int = 0
    pending_status: str | None = None
    last_seen: float = field(default_factory=time.monotonic)


class TrackManager:
    def __init__(self):
        self.tracks: dict = {}

    def get_or_create(self, track_id) -> TrackState:
        state = self.tracks.get(track_id)
        if state is None:
            state = TrackState()
            self.tracks[track_id] = state
        return state

    def tick(self, track_id):
        state = self.get_or_create(track_id)
        state.frame_counter += 1

    def should_classify(self, track_id) -> bool:
        state = self.get_or_create(track_id)
        # Locked: once confirmed UNAUTHORIZED, the verdict is permanent for this
        # track's lifetime — never re-classify, so motion/blur can't churn a known
        # target back to UNKNOWN and reset the fire dwell.
        if state.auth_status == "UNAUTHORIZED":
            return False
        # Unconfirmed tracks classify every frame so a new face resolves fast; once a
        # status is locked in, fall back to the slow steady-state re-check cadence.
        cadence = (CLASSIFY_EVERY_N_FRAMES_UNCONFIRMED
                   if state.auth_status == "UNKNOWN" else CLASSIFY_EVERY_N_FRAMES)
        return (state.frame_counter - 1) % cadence == 0

    def update_status(self, track_id, new_status):
        state = self.get_or_create(track_id)

        # Locked: an UNAUTHORIZED verdict is permanent for this track's lifetime.
        # Ignores even a stale classifier result that was in-flight when the track
        # was confirmed UNAUTHORIZED.
        if state.auth_status == "UNAUTHORIZED":
            return

        # UNKNOWN means "no usable reading this cycle" — hold the last locked status.
        if new_status == "UNKNOWN":
            return

        if new_status == state.pending_status:
            state.consecutive_count += 1
        else:
            state.pending_status = new_status
            state.consecutive_count = 1

        # Confirm the first status quickly; require the full debounce only to *flip* an
        # already-established status, which guards confirmed tracks against flicker.
        threshold = (DEBOUNCE_COUNT_INITIAL
                     if state.auth_status == "UNKNOWN" else DEBOUNCE_COUNT)
        if state.consecutive_count >= threshold and new_status != state.auth_status:
            old = state.auth_status
            state.auth_status = new_status
            print(f"[AUTH] Track {track_id}: {old} -> {new_status}")

    def is_target(self, track_id) -> bool:
        # Treat UNKNOWN (not yet/never confirmed) the same as UNAUTHORIZED:
        # anyone who is not explicitly AUTHORIZED is a valid target.
        state = self.tracks.get(track_id)
        return state is None or state.auth_status != "AUTHORIZED"

    def get_status(self, track_id) -> str:
        state = self.tracks.get(track_id)
        return state.auth_status if state is not None else "UNKNOWN"

    def reset_unauthorized(self) -> int:
        """After enrollment: drop confirmed-UNAUTHORIZED tracks to UNKNOWN so they
        re-classify against the updated db (the frozen verdict would keep a
        just-enrolled person targeted). Scoped on purpose — AUTHORIZED/UNKNOWN
        tracks untouched; affected tracks sit at UNKNOWN (never a target) until
        re-confirmed, so the turret holds rather than flickers. Returns count.
        """
        n = 0
        for state in self.tracks.values():
            if state.auth_status == "UNAUTHORIZED":
                state.auth_status = "UNKNOWN"
                state.pending_status = None
                state.consecutive_count = 0
                n += 1
        return n

    def cleanup(self, active_ids, now: float | None = None):
        """Delete state only after TRACK_STATE_GRACE_S seconds unseen. ByteTrack
        reports only ACTIVE tracks, so a mid-dip track is absent from active_ids —
        instant deletion wiped verdicts one missed frame before the tracker
        re-attached the same ID. Time-based (not frames) so the budget doesn't
        vary with detector fps. `now` injectable for tests."""
        now = time.monotonic() if now is None else now
        active = set(active_ids)
        for tid, state in list(self.tracks.items()):
            if tid in active:
                state.last_seen = now
            elif now - state.last_seen > TRACK_STATE_GRACE_S:
                del self.tracks[tid]
