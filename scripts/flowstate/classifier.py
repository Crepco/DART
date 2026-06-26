"""A small, self-contained focus classifier.

Instead of the fixed rule ``beta / (alpha + theta)``, we learn what *your*
focused vs zoned-out brain looks like from the calibration recordings.

It's logistic regression, implemented from scratch in numpy (no extra
dependency, fully transparent): standardise features, then gradient-descent
on the cross-entropy loss. Output is P(focused) in 0-1, which we scale to a
0-100 focus score.

Why a learned model beats the single ratio:
  * it weighs several band features together, not just one ratio,
  * it adapts to each person's baseline automatically,
  * it reports a training accuracy you can quote.
"""

from __future__ import annotations

import numpy as np

# Stable, fixed feature order so saved models stay valid.
FEATURE_NAMES = [
    "rel_delta", "rel_theta", "rel_alpha", "rel_beta",
    "engagement", "alpha_over_beta", "theta_over_beta",
]


def feature_vector(powers):
    """Turn a band-power dict into the model's feature vector."""
    d = powers.get("delta", 0.0)
    t = powers.get("theta", 0.0)
    a = powers.get("alpha", 0.0)
    b = powers.get("beta", 0.0)
    total = d + t + a + b + 1e-12
    eng = b / (a + t + 1e-12)
    a_b = a / (b + 1e-12)
    t_b = t / (b + 1e-12)
    return [d / total, t / total, a / total, b / total, eng, a_b, t_b]


def _sigmoid(z):
    return 1.0 / (1.0 + np.exp(-np.clip(z, -30, 30)))


class LogisticFocusClassifier:
    def __init__(self):
        self.w = None
        self.b = 0.0
        self.mu = None
        self.sd = None
        self.trained = False
        self.train_acc = 0.0
        self.n_focused = 0
        self.n_zoned = 0

    # ------------------------------------------------------------------- train
    def fit(self, X_focused, X_zoned, epochs=600, lr=0.2, l2=1e-3):
        """Train on focused (label 1) and zoned (label 0) feature lists."""
        Xf = np.asarray(X_focused, dtype=float)
        Xz = np.asarray(X_zoned, dtype=float)
        if Xf.ndim != 2 or Xz.ndim != 2 or len(Xf) < 5 or len(Xz) < 5:
            return False

        X = np.vstack([Xf, Xz])
        y = np.concatenate([np.ones(len(Xf)), np.zeros(len(Xz))])

        self.mu = X.mean(axis=0)
        self.sd = X.std(axis=0) + 1e-9
        Xs = (X - self.mu) / self.sd

        n, d = Xs.shape
        self.w = np.zeros(d)
        self.b = 0.0
        for _ in range(epochs):
            p = _sigmoid(Xs @ self.w + self.b)
            err = p - y
            self.w -= lr * (Xs.T @ err / n + l2 * self.w)
            self.b -= lr * float(np.mean(err))

        preds = _sigmoid(Xs @ self.w + self.b) > 0.5
        self.train_acc = float(np.mean(preds == (y > 0.5)))
        self.n_focused = int(len(Xf))
        self.n_zoned = int(len(Xz))
        self.trained = True
        return True

    # ----------------------------------------------------------------- predict
    def focus_prob(self, features):
        """P(focused) in 0-1 for one feature vector."""
        x = (np.asarray(features, dtype=float) - self.mu) / self.sd
        return float(_sigmoid(x @ self.w + self.b))

    # ----------------------------------------------------------- (de)serialise
    def to_dict(self):
        if not self.trained:
            return {"trained": False}
        return {
            "trained": True,
            "w": self.w.tolist(),
            "b": self.b,
            "mu": self.mu.tolist(),
            "sd": self.sd.tolist(),
            "train_acc": self.train_acc,
            "n_focused": self.n_focused,
            "n_zoned": self.n_zoned,
            "features": FEATURE_NAMES,
        }

    @classmethod
    def from_dict(cls, data):
        clf = cls()
        if not data or not data.get("trained"):
            return clf
        clf.w = np.asarray(data["w"], dtype=float)
        clf.b = float(data["b"])
        clf.mu = np.asarray(data["mu"], dtype=float)
        clf.sd = np.asarray(data["sd"], dtype=float)
        clf.train_acc = float(data.get("train_acc", 0.0))
        clf.n_focused = int(data.get("n_focused", 0))
        clf.n_zoned = int(data.get("n_zoned", 0))
        clf.trained = True
        return clf
