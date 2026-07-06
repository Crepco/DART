import numpy as np

from config import PAN_KP, PAN_KI, PAN_KD, TILT_KP, TILT_KI, TILT_KD, D_SMOOTH, INTEGRAL_CLAMP


class PID:
    def __init__(self, kp, ki, kd, limit, d_smooth=D_SMOOTH):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.limit      = limit
        self.d_smooth   = d_smooth
        self.integral   = 0.0
        self.prev_error = 0.0
        self.deriv      = 0.0

    def update(self, error: float, dt: float, integral_clamp: float = None) -> float:
        dt    = max(dt, 1e-6)
        i_lim = integral_clamp if integral_clamp is not None else self.limit
        self.integral = float(np.clip(self.integral + error * dt, -i_lim, i_lim))
        raw_deriv       = (error - self.prev_error) / dt
        self.deriv      = self.d_smooth * self.deriv + (1.0 - self.d_smooth) * raw_deriv
        self.prev_error = error
        raw = self.kp * error + self.ki * self.integral + self.kd * self.deriv
        return float(np.clip(raw, -self.limit, self.limit))

    def reset(self):
        self.integral   = 0.0
        self.prev_error = 0.0
        self.deriv      = 0.0


def asym_ema(prev: float, new: float, attack: float, decay: float) -> float:
    """Command EMA with a fast release. `attack` (heavier) smooths while the
    command magnitude grows; `decay` (lighter) while it shrinks or reverses sign.
    Braking must not inherit the attack lag — with a symmetric EMA the turret kept
    coasting for the whole decay tail after the PID already said "stop"."""
    braking = (prev * new < 0) or (abs(new) < abs(prev))
    w = decay if braking else attack
    return w * prev + (1.0 - w) * new


def slew_step(current: float, target: float, max_step: float,
              brake_mult: float) -> float:
    """One slew-rate-limited step from `current` toward `target`. The step limit
    is multiplied by `brake_mult` while the step opposes the current command's
    sign (i.e. the command is being driven toward zero): on a direction reversal
    this brakes fast until the zero crossing, then accelerates outward at the
    normal 1x rate — reversals are never slower than before, only braking is
    faster."""
    step = target - current
    limit = max_step * (brake_mult if current * step < 0 else 1.0)
    return current + max(-limit, min(limit, step))
