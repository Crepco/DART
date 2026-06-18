"""Per-track authorization state with debouncing."""

from dataclasses import dataclass

from config import CLASSIFY_EVERY_N_FRAMES, DEBOUNCE_COUNT


@dataclass
class TrackState:
    auth_status: str = "UNKNOWN"
    frame_counter: int = 0
    consecutive_count: int = 0
    pending_status: str | None = None


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
        return (state.frame_counter - 1) % CLASSIFY_EVERY_N_FRAMES == 0

    def update_status(self, track_id, new_status):
        state = self.get_or_create(track_id)

        # UNKNOWN means "no usable reading this cycle" — hold the last locked status.
        if new_status == "UNKNOWN":
            return

        if new_status == state.pending_status:
            state.consecutive_count += 1
        else:
            state.pending_status = new_status
            state.consecutive_count = 1

        if state.consecutive_count >= DEBOUNCE_COUNT and new_status != state.auth_status:
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

    def cleanup(self, active_ids):
        active = set(active_ids)
        for tid in [t for t in self.tracks if t not in active]:
            del self.tracks[tid]
