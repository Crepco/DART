"""asym_ema / slew_step — the shaping stages between PID output and servo command.

The regression risks: braking inheriting the attack lag (the overshoot tail), and
direction reversals becoming SLOWER than symmetric behavior.
"""

import pytest

from core.pid import asym_ema, slew_step

ATTACK, DECAY = 0.85, 0.5


def test_ema_onset_uses_attack():
    # magnitude growing -> heavy smoothing (soft spin-up preserved)
    assert asym_ema(0.0, 10.0, ATTACK, DECAY) == pytest.approx(1.5)


def test_ema_release_uses_decay():
    # magnitude shrinking -> light smoothing (fast braking)
    assert asym_ema(10.0, 0.0, ATTACK, DECAY) == pytest.approx(5.0)


def test_ema_reversal_takes_fast_path_through_zero():
    # sign flip counts as braking even though |new| > |prev| — the command must
    # cross zero quickly, not coast on the attack constant
    assert asym_ema(5.0, -8.0, ATTACK, DECAY) == pytest.approx(-1.5)


def test_ema_zero_prev_is_attack():
    # from rest, any command is an onset
    assert asym_ema(0.0, -10.0, ATTACK, DECAY) == pytest.approx(-1.5)


def test_slew_accelerates_at_base_rate():
    assert slew_step(0.0, 10.0, max_step=2.0, brake_mult=3.0) == 2.0
    assert slew_step(0.0, -10.0, max_step=2.0, brake_mult=3.0) == -2.0


def test_slew_brakes_faster():
    assert slew_step(10.0, 0.0, max_step=2.0, brake_mult=3.0) == 4.0


def test_slew_reversal_brakes_then_accelerates_normally():
    # +2 -> target -10: braking (3x) until the zero crossing...
    assert slew_step(2.0, -10.0, max_step=2.0, brake_mult=3.0) == -4.0
    # ...then same-sign motion continues at the base 1x rate
    assert slew_step(-4.0, -10.0, max_step=2.0, brake_mult=3.0) == -6.0


def test_slew_no_overshoot_of_target():
    # a step smaller than the limit lands exactly on target
    assert slew_step(1.0, 0.5, max_step=2.0, brake_mult=3.0) == 0.5
    assert slew_step(-1.0, 0.0, max_step=2.0, brake_mult=3.0) == 0.0
