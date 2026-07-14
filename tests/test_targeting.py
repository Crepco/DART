"""Pursuit-continuity targeting — conf-dip lock holds + faceless aim fallback.

Round-3b forensics: pursuit was stop-start because (a) a conf dip below
PERSON_CONF unlocked a target ByteTrack never dropped ("the box disappears but
the ID stays"), and (b) both loops refused to aim without a face box every
frame. resolve_lock keeps an existing lock through bounded dips; AimSelector
keeps aiming at an offset-corrected head proxy through face gaps. These tests
pin the acquisition rules (unchanged), the hold bounds, and the jump-free
face→proxy handoff.
"""

import pytest

from config import (HEAD_PROXY_FRAC, PERSON_CONF, TARGET_HOLD_CONF,
                    TARGET_HOLD_MAX_S)
from core.targeting import AimSelector, resolve_lock

NOW = 1000.0


def box(tid, size=100, x=0, y=0):
    return (x, y, x + size, y + size, tid)


# ── resolve_lock: acquisition (today's rules, unchanged) ──

def test_single_candidate_locks():
    lid, held, started = resolve_lock(None, "UNKNOWN", [box(3)], {3: 0.9},
                                      NOW, None)
    assert (lid, held, started) == (3, False, None)


def test_multiple_candidates_largest_wins():
    lid, _, _ = resolve_lock(None, "UNKNOWN", [box(1, 50), box(2, 120)],
                             {1: 0.9, 2: 0.6}, NOW, None)
    assert lid == 2


# ── resolve_lock: conf-dip hold ──

def test_dip_holds_existing_lock():
    lid, held, started = resolve_lock(5, "UNAUTHORIZED", [], {5: 0.30},
                                      NOW, None)
    assert (lid, held, started) == (5, True, NOW)


def test_hold_carries_start_time_across_frames():
    _, _, started = resolve_lock(5, "UNAUTHORIZED", [], {5: 0.30}, NOW, None)
    lid, held, started2 = resolve_lock(5, "UNAUTHORIZED", [], {5: 0.25},
                                       NOW + 0.5, started)
    assert (lid, held, started2) == (5, True, NOW)


def test_hold_needs_min_conf():
    lid, held, _ = resolve_lock(5, "UNAUTHORIZED", [],
                                {5: TARGET_HOLD_CONF - 0.01}, NOW, None)
    assert (lid, held) == (None, False)


def test_bytetrack_loss_unlocks():
    lid, held, _ = resolve_lock(5, "UNAUTHORIZED", [], {}, NOW, None)
    assert (lid, held) == (None, False)


def test_hold_expires_at_cap():
    started = NOW - TARGET_HOLD_MAX_S - 0.1
    lid, held, out = resolve_lock(5, "UNAUTHORIZED", [], {5: 0.30},
                                  NOW, started)
    assert (lid, held, out) == (None, False, None)


def test_hold_requires_unauthorized_verdict():
    # Enrollment resets a track to UNKNOWN mid-hold -> the lock must release.
    lid, held, _ = resolve_lock(5, "UNKNOWN", [], {5: 0.30}, NOW, None)
    assert (lid, held) == (None, False)


# ── resolve_lock: hold-ending arbitration ──

def test_full_conf_candidate_ends_hold():
    # A DIFFERENT clearly-visible unauthorized person beats the dipped box —
    # deliberate (today's full-conf swap policy, not redesigned this round).
    lid, held, started = resolve_lock(5, "UNAUTHORIZED", [box(7)], {5: 0.30, 7: 0.8},
                                      NOW, NOW - 1.0)
    assert (lid, held, started) == (7, False, None)


def test_multiple_candidates_mid_hold_largest_steals():
    lid, held, _ = resolve_lock(5, "UNAUTHORIZED", [box(7, 60), box(8, 200)],
                                {5: 0.30, 7: 0.7, 8: 0.9}, NOW, NOW - 1.0)
    assert (lid, held) == (8, False)


def test_held_target_returning_resumes_normal_lock():
    lid, held, started = resolve_lock(5, "UNAUTHORIZED", [box(5)],
                                      {5: PERSON_CONF + 0.1}, NOW, NOW - 1.0)
    assert (lid, held, started) == (5, False, None)


# ── AimSelector ──

PERSON = (100, 100, 200, 400)                    # w=100, h=300
PROXY = (150.0, 100 + HEAD_PROXY_FRAC * 300)     # (150.0, 154.0)
FACE = (140, 120, 160, 140)                      # centre (150, 130), inside box


def test_face_preferred_when_visible():
    sel = AimSelector()
    assert sel.select([FACE], PERSON, 5, None, None) == pytest.approx((150.0, 130.0))


def test_face_to_proxy_handoff_is_jump_free():
    sel = AimSelector()
    pt_face = sel.select([FACE], PERSON, 5, None, None)
    pt_proxy = sel.select([], PERSON, 5, *pt_face)
    # The learned offset corrects the proxy onto the face point exactly —
    # face flapping produces zero aim oscillation for a static person.
    assert pt_proxy == pytest.approx(pt_face)


def test_handoff_zero_jump_even_when_held_box_geometry_differs():
    # Round-4 hardware defect: the face dropped the same frame the box went
    # low-conf and RESIZED — the learned offset applied to the new geometry
    # teleported the aim 112 px and the turret chased it past the target. The
    # handoff anchor re-bases the offset on the handoff frame's box.
    sel = AimSelector()
    pt_face = sel.select([FACE], PERSON, 5, None, None)
    shifted = (60, 150, 240, 460)                      # blur-distorted held box
    pt_proxy = sel.select([], shifted, 5, *pt_face)
    assert pt_proxy == pytest.approx(pt_face)
    # ...and from the anchor on, the aim rides the box's motion.
    moved = tuple(v + 10 for v in shifted)
    pt_next = sel.select([], moved, 5, *pt_proxy)
    assert pt_next == pytest.approx((pt_face[0] + 10, pt_face[1] + 10))


def test_plain_proxy_when_no_face_ever_seen():
    sel = AimSelector()
    assert sel.select([], PERSON, 5, None, None) == pytest.approx(PROXY)


def test_no_person_box_returns_none_and_resets():
    sel = AimSelector()
    sel.select([FACE], PERSON, 5, None, None)
    assert sel.select([FACE], None, None, 150.0, 130.0) is None
    # State dropped: the next lock starts from the plain prior again.
    assert sel.select([], PERSON, 5, None, None) == pytest.approx(PROXY)


def test_offset_resets_on_new_lock():
    sel = AimSelector()
    sel.select([FACE], PERSON, 5, None, None)          # learn offset for tid 5
    pt = sel.select([], PERSON, 9, 150.0, 130.0)       # new lock, no face yet
    assert pt == pytest.approx(PROXY)


def test_bystander_face_outside_box_never_steers_aim():
    # Round-4c hardware: false-positive "faces" across the frame yanked the
    # aim 100-270 px/frame. Faces outside the locked person's (inflated) box
    # are now invisible to aiming entirely.
    sel = AimSelector()
    bystander = (400, 120, 420, 140)                   # centre (410, 130), outside
    assert sel.select([bystander], PERSON, 5, None, None) == pytest.approx(PROXY)


def test_nearest_face_beats_larger_face_within_box():
    # Continuity beats size: with an anchor, the aim sticks to the near face
    # even when a bigger detection appears elsewhere on the same person.
    sel = AimSelector()
    big_low = (110, 220, 190, 300)                     # centre (150, 260), larger
    pt = sel.select([FACE, big_low], PERSON, 5, 150.0, 130.0)
    assert pt == pytest.approx((150.0, 130.0))


# ── re-acquisition confirmation window (round-4e feedback loop) ──

def test_single_frame_flicker_never_moves_aim():
    # The loop's restart edge: pan blur drops the face, a 1-frame flicker
    # re-detection used to fire a full-speed correction. Now it moves nothing.
    sel = AimSelector()
    pt_face = sel.select([FACE], PERSON, 5, None, None)
    pt_gap = sel.select([], PERSON, 5, *pt_face)           # anchored proxy
    regained = (230, 120, 250, 140)                        # face back, elsewhere
    pt_flick = sel.select([regained], PERSON, 5, *pt_gap)
    assert pt_flick == pytest.approx(pt_gap)
    pt_gone = sel.select([], PERSON, 5, *pt_flick)         # flicker ends
    assert pt_gone == pytest.approx(pt_gap)


def test_sustained_regain_confirms_after_window():
    from config import AIM_REACQUIRE_FRAMES, AIM_STEP_LIMIT
    sel = AimSelector()
    pt = sel.select([FACE], PERSON, 5, None, None)         # (150, 130)
    pt = sel.select([], PERSON, 5, *pt)                    # gap -> anchored
    regained = (230, 120, 250, 140)                        # centre (240, 130)
    for _ in range(AIM_REACQUIRE_FRAMES - 1):
        pt = sel.select([regained], PERSON, 5, *pt)
        assert pt == pytest.approx((150.0, 130.0))         # window: still held
    pt = sel.select([regained], PERSON, 5, *pt)            # confirmed
    assert pt == pytest.approx((150.0 + AIM_STEP_LIMIT, 130.0))  # ramp begins


# ── aim step clamp ──

def test_far_face_return_is_ramped_not_teleported():
    from config import AIM_STEP_LIMIT
    sel = AimSelector()
    pt0 = sel.select([FACE], PERSON, 5, None, None)         # (150, 130)
    far = (230, 120, 250, 140)                              # centre (240, 130), in box
    pt1 = sel.select([far], PERSON, 5, *pt0)
    # One frame moves exactly AIM_STEP_LIMIT toward the new point, no jump.
    assert pt1 == pytest.approx((150.0 + AIM_STEP_LIMIT, 130.0))
    # Repeated frames converge onto the true face.
    pt = pt1
    for _ in range(10):
        pt = sel.select([far], PERSON, 5, *pt)
    assert pt == pytest.approx((240.0, 130.0))


def test_small_genuine_motion_is_never_clamped():
    sel = AimSelector()
    pt0 = sel.select([FACE], PERSON, 5, None, None)
    moved = tuple(v + 20 for v in FACE)                     # 28 px diagonal step
    pt1 = sel.select([moved], PERSON, 5, *pt0)
    assert pt1 == pytest.approx((170.0, 150.0))


def test_clamp_does_not_apply_across_locks():
    sel = AimSelector()
    sel.select([FACE], PERSON, 5, None, None)
    far_person = (600, 100, 700, 400)
    pt = sel.select([], far_person, 9, 150.0, 130.0)        # new lock, far away
    assert pt == pytest.approx((650.0, 100 + HEAD_PROXY_FRAC * 300))
