"""Filtering for raw EEG.

Forehead EEG from a dry-electrode setup is dominated by powerline noise
(50/60 Hz) and DC drift. Before any band-power analysis we must:

  1. Notch out the powerline frequency (and its first harmonic).
  2. Bandpass to the EEG range (1-40 Hz) to drop DC drift and HF junk.

These use zero-phase ``filtfilt`` so peaks don't get time-shifted. That is
fine for offline analysis and for the short (1-2 s) sliding windows we run
in real time.
"""

from __future__ import annotations

import numpy as np
from scipy.signal import butter, filtfilt, iirnotch


def notch_filter(signal, fs, freq=50.0, q=30.0):
    """Remove a single powerline frequency.

    Parameters
    ----------
    signal : array-like
    fs : float        sampling rate (Hz)
    freq : float      powerline frequency, 50 in IN/EU, 60 in US
    q : float         quality factor (higher = narrower notch)
    """
    signal = np.asarray(signal, dtype=float)
    if signal.size < 9:
        return signal
    b, a = iirnotch(w0=freq, Q=q, fs=fs)
    return filtfilt(b, a, signal)


def bandpass_filter(signal, fs, low=1.0, high=40.0, order=4):
    """Keep only the EEG-relevant frequency band."""
    signal = np.asarray(signal, dtype=float)
    if signal.size < 3 * order:
        return signal
    nyq = 0.5 * fs
    high = min(high, nyq * 0.99)  # never ask for >= Nyquist
    b, a = butter(order, [low / nyq, high / nyq], btype="band")
    return filtfilt(b, a, signal)


def clean_signal(signal, fs, powerline=50.0, low=1.0, high=40.0,
                 notch_harmonic=True):
    """Full cleanup: notch powerline (+harmonic) then bandpass to EEG range.

    This is the function the rest of the pipeline calls. Returns the cleaned
    signal, same length as the input.
    """
    signal = np.asarray(signal, dtype=float)
    if signal.size < 9:
        return signal
    out = notch_filter(signal, fs, freq=powerline)
    harmonic = powerline * 2
    if notch_harmonic and harmonic < 0.5 * fs:
        out = notch_filter(out, fs, freq=harmonic)
    out = bandpass_filter(out, fs, low=low, high=high)
    return out
