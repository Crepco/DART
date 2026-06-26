"""FlowState signal-processing package.

Vendored into DART from the FlowState project (https://github.com/Crepco/flowstate),
its signal_processing/ package. Kept verbatim so upstream fixes drop in cleanly. Used
by DART's web UI (mode 1) to turn shared-serial EEG into a focus score / zone-out flag.

Turns raw forehead-EEG samples into a relative focus score:

    raw samples -> notch(50Hz) + bandpass(1-40Hz) -> band powers
                -> engagement index (beta / (alpha + theta))
                -> focus score (0-100, relative to a personal baseline)
"""

# OpenBLAS/OMP allocate a memory pool *per thread*. When SciPy/NumPy are called
# from many threads at once (the Flask server + our background compute loop),
# that pool can be exhausted, crashing with:
#   "OpenBLAS error: Memory allocation still failed after 10 retries".
# Pin BLAS to a single thread BEFORE numpy is first imported below. Our FFT
# windows are tiny (~1000 samples), so single-threaded BLAS costs us nothing.
import os as _os
_os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
_os.environ.setdefault("OMP_NUM_THREADS", "1")
_os.environ.setdefault("MKL_NUM_THREADS", "1")

from .filters import notch_filter, bandpass_filter, clean_signal
from .bands import BANDS, band_powers, engagement_index
from .classifier import LogisticFocusClassifier, feature_vector, FEATURE_NAMES
from .pipeline import FocusPipeline

__all__ = [
    "notch_filter",
    "bandpass_filter",
    "clean_signal",
    "BANDS",
    "band_powers",
    "engagement_index",
    "LogisticFocusClassifier",
    "feature_vector",
    "FEATURE_NAMES",
    "FocusPipeline",
]
