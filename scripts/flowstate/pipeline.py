"""Real-time focus pipeline.

Feed it raw samples as they arrive (``add_samples``). Every so often call
``compute()`` to get the current state: cleaned waveform tail, band powers,
engagement index, a 0-100 focus score, and an alert flag.

Three scoring modes, best-available wins:

* **ML model** (after calibrating both states): a logistic-regression
  classifier trained on your focused vs zoned recordings outputs P(focused).
  Most robust; reports a training accuracy.

* **Calibrated** (engagement baselines, no model yet): the score linearly
  maps the engagement index between the zoned baseline (0) and focused (100).

* **Relative / uncalibrated** (works immediately): the engagement index
  z-scored against its own recent history and squashed to 0-100.
"""

from __future__ import annotations

import json
import math
import time
from collections import deque
from pathlib import Path

import numpy as np

from .bands import band_powers, engagement_index, relative_band_powers
from .classifier import LogisticFocusClassifier, feature_vector
from .filters import clean_signal


class FocusPipeline:
    def __init__(
        self,
        fs=500,
        powerline=50.0,
        window_sec=2.0,
        history_sec=10.0,
        display_hz=100,
        display_sec=4.0,
        ema_alpha=0.2,
        alert_threshold=40.0,
        alert_hold_sec=1.0,
        model_path=None,
    ):
        self.fs = int(fs)
        self.powerline = powerline
        self.window_n = int(window_sec * fs)
        self.ema_alpha = ema_alpha
        self.alert_threshold = alert_threshold
        self.alert_hold_sec = alert_hold_sec
        self.model_path = str(model_path) if model_path else None

        # Raw ring buffer of the most recent samples.
        self._raw = deque(maxlen=int(history_sec * fs))

        # Downsampled cleaned waveform for the live trace.
        self.display_hz = display_hz
        self._display = deque(maxlen=int(display_sec * display_hz))

        # Rolling history of engagement values for uncalibrated z-scoring.
        self._eng_hist = deque(maxlen=300)

        # Smoothed focus score state.
        self.focus = 50.0
        self._focus_init = False

        # Calibration baselines (median engagement per labelled state).
        self.baseline = {"focused": None, "zoned": None}
        self._calib_label = None
        self._calib_values = []
        self._calib_until = 0.0

        # ML classifier + collected feature vectors per labelled state.
        self.classifier = LogisticFocusClassifier()
        self._calib_feats = {"focused": [], "zoned": []}

        # Alert state machine.
        self._below_since = None
        self.alert = False

        self.samples_seen = 0

        if self.model_path:
            self.load_model()

    # ------------------------------------------------------------------ input
    def add_samples(self, values):
        """Append raw samples (iterable of numbers)."""
        for v in values:
            self._raw.append(float(v))
        self.samples_seen += len(values) if hasattr(values, "__len__") else 0

    # ------------------------------------------------------ calibration control
    def start_calibration(self, label, seconds=60):
        if label not in ("focused", "zoned"):
            raise ValueError("label must be 'focused' or 'zoned'")
        self._calib_label = label
        self._calib_values = []
        self._calib_feats[label] = []  # re-recording this state replaces it
        self._calib_until = time.time() + seconds

    def cancel_calibration(self):
        self._calib_label = None
        self._calib_values = []

    @property
    def calibrating(self):
        return self._calib_label is not None

    @property
    def is_calibrated(self):
        return (
            self.baseline["focused"] is not None
            and self.baseline["zoned"] is not None
        )

    def reset_calibration(self):
        self.baseline = {"focused": None, "zoned": None}
        self._calib_feats = {"focused": [], "zoned": []}
        self.classifier = LogisticFocusClassifier()
        if self.model_path:
            try:
                Path(self.model_path).unlink(missing_ok=True)
            except OSError:
                pass

    @property
    def score_mode(self):
        if self.classifier.trained:
            return "ml"
        if self.is_calibrated:
            return "calibrated"
        return "relative"

    def _maybe_train(self):
        """Train the classifier once both states have enough feature data."""
        f = self._calib_feats["focused"]
        z = self._calib_feats["zoned"]
        if len(f) >= 5 and len(z) >= 5:
            if self.classifier.fit(f, z) and self.model_path:
                self.save_model()

    # --------------------------------------------------------------- scoring
    def _score(self, eng, feats):
        """Map current features/engagement to a 0-100 focus score."""
        # Best mode: learned classifier.
        if self.classifier.trained:
            return 100.0 * self.classifier.focus_prob(feats)

        # Calibrated linear map on the engagement index.
        if self.is_calibrated:
            lo = self.baseline["zoned"]
            hi = self.baseline["focused"]
            if hi == lo:
                return 50.0
            return float(np.clip(100.0 * (eng - lo) / (hi - lo), 0.0, 100.0))

        # Uncalibrated: z-score against recent history -> logistic -> 0-100.
        if len(self._eng_hist) < 8:
            return 50.0
        arr = np.asarray(self._eng_hist, dtype=float)
        mean, std = arr.mean(), arr.std()
        if std < 1e-9:
            return 50.0
        z = (eng - mean) / std
        return float(100.0 / (1.0 + math.exp(-z)))

    # ----------------------------------------------------------- persistence
    def save_model(self):
        if not self.model_path:
            return
        payload = {
            "baseline": self.baseline,
            "classifier": self.classifier.to_dict(),
            "fs": self.fs,
        }
        path = Path(self.model_path)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(payload, indent=2))

    def load_model(self):
        if not self.model_path:
            return
        path = Path(self.model_path)
        if not path.exists():
            return
        try:
            data = json.loads(path.read_text())
        except (OSError, ValueError):
            return
        base = data.get("baseline") or {}
        self.baseline = {"focused": base.get("focused"), "zoned": base.get("zoned")}
        self.classifier = LogisticFocusClassifier.from_dict(data.get("classifier"))

    # ---------------------------------------------------------------- compute
    def compute(self):
        """Run one analysis step and return a JSON-serialisable dict."""
        now = time.time()
        n = len(self._raw)
        ready = n >= self.window_n

        packet = {
            "t": now,
            "fs": self.fs,
            "samples_seen": self.samples_seen,
            "ready": ready,
            "calibrating": self.calibrating,
            "calibrated": self.is_calibrated,
            "baseline": dict(self.baseline),
            "score_mode": self.score_mode,
            "classifier": {
                "trained": self.classifier.trained,
                "accuracy": round(self.classifier.train_acc, 3),
                "n_focused": self.classifier.n_focused,
                "n_zoned": self.classifier.n_zoned,
            },
        }

        if not ready:
            packet.update(
                state="warmup",
                focus=round(self.focus, 1),
                engagement=0.0,
                bands={}, band_rel={}, alert=False,
                wave=self._wave_list(), wave_hz=self.display_hz,
                quality="warmup",
                progress=round(100.0 * n / self.window_n, 1),
            )
            return packet

        window = np.asarray(self._raw, dtype=float)[-self.window_n:]
        cleaned = clean_signal(window, self.fs, powerline=self.powerline)

        # Update the live display trace (downsample the cleaned tail).
        self._push_display(cleaned)

        powers = band_powers(cleaned, self.fs)
        eng = engagement_index(powers)
        feats = feature_vector(powers)
        self._eng_hist.append(eng)

        # Feed calibration if active: collect both the engagement value (for
        # the linear-map fallback) and the feature vector (for the ML model).
        if self.calibrating:
            label = self._calib_label
            self._calib_values.append(eng)
            self._calib_feats[label].append(feats)
            if now >= self._calib_until and self._calib_values:
                self.baseline[label] = float(np.median(self._calib_values))
                self._calib_label = None
                self._calib_values = []
                self._maybe_train()  # trains once both states are recorded

        # Focus score (EMA-smoothed).
        target = self._score(eng, feats)
        if not self._focus_init:
            self.focus = target
            self._focus_init = True
        else:
            a = self.ema_alpha
            self.focus = (1 - a) * self.focus + a * target

        # Alert state machine: sustained low focus.
        if self.focus < self.alert_threshold:
            if self._below_since is None:
                self._below_since = now
            self.alert = (now - self._below_since) >= self.alert_hold_sec
        else:
            self._below_since = None
            self.alert = False

        # Signal-quality heuristic: how much residual sits at powerline freq.
        quality = self._quality(window)

        if self.calibrating:
            state = "calibrating"
        elif self.alert:
            state = "zoning"
        elif self.focus < self.alert_threshold:
            state = "dipping"
        else:
            state = "focused"

        packet.update(
            state=state,
            focus=round(self.focus, 1),
            engagement=round(eng, 4),
            bands={k: round(v, 8) for k, v in powers.items()},
            band_rel={k: round(v, 4) for k, v in relative_band_powers(powers).items()},
            alert=self.alert,
            wave=self._wave_list(),
            wave_hz=self.display_hz,
            quality=quality,
            calib_label=self._calib_label,
            calib_remaining=max(0.0, round(self._calib_until - now, 1)) if self.calibrating else 0.0,
        )
        return packet

    # ---------------------------------------------------------------- display
    def _push_display(self, cleaned):
        step = max(1, int(self.fs / self.display_hz))
        # Only push the freshly-needed tail to keep a continuous-looking trace.
        tail = cleaned[::step]
        keep = self._display.maxlen
        for v in tail[-keep:]:
            self._display.append(float(v))

    def _wave_list(self):
        return [round(v, 4) for v in self._display]

    def _quality(self, window):
        """Rough: powerline power vs EEG-band power on the *raw* window."""
        from .bands import band_powers as _bp
        raw_powers = _bp(window, self.fs, bands={
            "line": (self.powerline - 1.5, self.powerline + 1.5),
            "eeg": (4.0, 30.0),
        })
        line = raw_powers.get("line", 0.0)
        eeg = raw_powers.get("eeg", 1e-12)
        ratio = line / eeg if eeg > 0 else 999
        if ratio > 8:
            return "noisy"
        if ratio > 2:
            return "fair"
        return "good"
