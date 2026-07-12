"""PID gap-resync + derivative-kick clamp — the hardware "lurch" defect.

Round-1 closed-loop forensics: coast/lost frames freeze the PID (no update()
call), so prev_error goes stale at the last big error; the first post-gap
update computes a fictitious multi-thousand-px/s derivative and pins the
command at MAX_SPEED for several frames (D_SMOOTH memory keeps it there).
resync() re-anchors on tracking resume; D_KICK_LIMIT caps one-frame error
jumps that never pass through a gap (target-box switch, deadband exit).
"""

import pytest

import core.pid as pid_mod
from core.pid import PID

DT = 1 / 15          # hardware loop rate
KP, KD, LIMIT = 0.075, 0.15, 25.0   # pan gains at the time of the defect


def tracked_pid(err=250.0, frames=5):
    """A PID mid-track at a steady large error: prev_error is big, deriv ~0."""
    pid = PID(KP, 0.0, KD, LIMIT)
    for _ in range(frames):
        pid.update(err, DT)
    return pid


def test_defect_stale_gap_derivative_saturates(monkeypatch):
    # Pin the ORIGINAL bug (kick clamp disabled): a detection gap freezes the
    # PID, then re-acquisition near center reads (0 - 250)/dt px/s and pins
    # the output at the limit for multiple frames despite zero actual error.
    monkeypatch.setattr(pid_mod, "D_KICK_LIMIT", float("inf"))
    pid = tracked_pid()
    # ... gap: coast frames make NO update() calls, state stays frozen ...
    out1 = pid.update(0.0, DT)
    out2 = pid.update(0.0, DT)
    out3 = pid.update(0.0, DT)
    assert out1 == -LIMIT                       # full-speed lurch, wrong basis
    assert out2 == -LIMIT                       # D_SMOOTH memory keeps it pinned
    assert abs(out3) > LIMIT * 0.5


def test_kick_clamp_bounds_gap_spike_without_resync():
    # Belt-and-suspenders: even if a resume path misses resync, the clamp
    # keeps the fake kick well off the saturation ceiling.
    pid = tracked_pid()
    out = pid.update(0.0, DT)
    assert abs(out) < LIMIT * 0.8
    assert abs(out) <= KD * (0.3 * pid_mod.D_KICK_LIMIT + 0.7 * abs(pid.deriv)) + 1e-6


def test_resync_kills_gap_kick():
    pid = tracked_pid()
    pid.resync(0.0)                             # tracking resumed near center
    assert pid.update(0.0, DT) == pytest.approx(0.0)


def test_resync_reanchors_but_is_not_reset():
    pid = tracked_pid()
    integral_before = pid.integral
    pid.resync(-42.0)
    assert pid.prev_error == -42.0
    assert pid.deriv == 0.0
    assert pid.integral == integral_before      # unlike reset(): I-state survives


def test_clamp_transparent_for_genuine_motion(monkeypatch):
    # A real pursuit ramp (300 px/s < D_KICK_LIMIT) must be bit-identical to
    # the unclamped controller — steady-state behavior unchanged.
    seq = [0.0, 20.0, 40.0, 60.0, 80.0, 100.0]  # 20 px/frame at 15 fps

    clamped = PID(KP, 0.0, KD, LIMIT)
    got = [clamped.update(e, DT) for e in seq]

    monkeypatch.setattr(pid_mod, "D_KICK_LIMIT", float("inf"))
    reference = PID(KP, 0.0, KD, LIMIT)
    want = [reference.update(e, DT) for e in seq]

    assert got == pytest.approx(want)
