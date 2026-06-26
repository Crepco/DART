"""Frequency-band power and the engagement index.

We estimate power spectral density with Welch's method (averaged periodograms,
robust to noise) and integrate it over the classic EEG bands.

The engagement / focus proxy used in the attention literature is:

    engagement = beta / (alpha + theta)

Higher beta relative to alpha+theta tends to track active concentration;
more alpha/theta tends to track relaxed, drowsy, or mind-wandering states.
It is a *proxy*, only meaningful relative to a person's own baseline.
"""

from __future__ import annotations

import numpy as np
from scipy.signal import welch

# Classic EEG bands (Hz).
BANDS = {
    "delta": (1.0, 4.0),
    "theta": (4.0, 8.0),
    "alpha": (8.0, 12.0),
    "beta": (12.0, 30.0),
    "gamma": (30.0, 40.0),
}

# numpy 2.0 renamed trapz -> trapezoid (and 2.x dropped the old name); support both.
_trapz = getattr(np, "trapezoid", None) or getattr(np, "trapz", None)
if _trapz is None:  # very old/new edge case: integrate manually
    def _trapz(y, x):
        return float(np.sum((y[:-1] + y[1:]) * np.diff(x) / 2.0))


def band_powers(signal, fs, bands=BANDS):
    """Return absolute power in each band as a dict.

    Uses Welch PSD. Power is the integral of the PSD over the band.
    """
    signal = np.asarray(signal, dtype=float)
    out = {name: 0.0 for name in bands}
    if signal.size < 16:
        return out

    nperseg = int(min(signal.size, max(64, fs)))  # ~1 s segments
    freqs, psd = welch(signal, fs=fs, nperseg=nperseg)

    for name, (lo, hi) in bands.items():
        mask = (freqs >= lo) & (freqs <= hi)
        if mask.any():
            out[name] = float(_trapz(psd[mask], freqs[mask]))
    return out


def engagement_index(powers):
    """beta / (alpha + theta), guarding against divide-by-zero."""
    denom = powers.get("alpha", 0.0) + powers.get("theta", 0.0)
    if denom <= 0:
        return 0.0
    return powers.get("beta", 0.0) / denom


def relative_band_powers(powers, bands=BANDS):
    """Each band as a fraction of total power across the tracked bands.

    Handy for a stacked/normalized bar display (values sum to ~1).
    """
    total = sum(powers.get(name, 0.0) for name in bands)
    if total <= 0:
        return {name: 0.0 for name in bands}
    return {name: powers.get(name, 0.0) / total for name in bands}
