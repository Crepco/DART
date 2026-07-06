"""TrackManager: verdict debounce, the frozen-UNAUTHORIZED rule, and the scoped
post-enrollment reset."""

from auth.track_manager import TrackManager

from config import DEBOUNCE_COUNT_INITIAL


def confirm(tm, tid, status):
    for _ in range(DEBOUNCE_COUNT_INITIAL):
        tm.update_status(tid, status)
    assert tm.get_status(tid) == status


def test_unauthorized_verdict_is_frozen():
    tm = TrackManager()
    confirm(tm, 1, "UNAUTHORIZED")
    for _ in range(20):
        tm.update_status(1, "AUTHORIZED")
    assert tm.get_status(1) == "UNAUTHORIZED"


def test_reset_unauthorized_is_scoped():
    tm = TrackManager()
    confirm(tm, 1, "UNAUTHORIZED")
    confirm(tm, 2, "AUTHORIZED")
    tm.get_or_create(3)                      # UNKNOWN track

    assert tm.reset_unauthorized() == 1      # only track 1 affected

    assert tm.get_status(1) == "UNKNOWN"
    assert tm.get_status(2) == "AUTHORIZED"  # untouched
    assert tm.get_status(3) == "UNKNOWN"
    # debounce state cleared so re-classification starts clean
    assert tm.tracks[1].pending_status is None
    assert tm.tracks[1].consecutive_count == 0


def test_reset_track_can_reconfirm_either_way():
    tm = TrackManager()
    confirm(tm, 1, "UNAUTHORIZED")
    tm.reset_unauthorized()
    # after enrollment the same person should be able to confirm AUTHORIZED
    confirm(tm, 1, "AUTHORIZED")


def test_unknown_reads_do_not_advance_debounce():
    tm = TrackManager()
    tm.update_status(1, "UNKNOWN")
    tm.update_status(1, "UNKNOWN")
    assert tm.get_status(1) == "UNKNOWN"
    assert tm.tracks[1].pending_status is None
