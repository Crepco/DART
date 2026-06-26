"""FlowState focus bridge for DART mode 1.

Runs the vendored FocusPipeline in a background thread and exposes the current focus
score + a debounced `zone_out` flag the runner uses to gate firing. Two sample sources:

* ``serial`` -- drains EEG samples the SerialLink reader pulled off the shared R3 link
  (the BioAmp EXG Pill on A0, streamed as "E###"). Sampled on the R3 at ~250 Hz.
* ``csv``    -- replays the bundled sample recording, so mode 1 (focus gauge + the
  zone-out fire logic) is fully demoable with no EEG hardware attached.
"""

from __future__ import annotations

import csv
import pathlib
import threading
import time

import numpy as np

from flowstate import FocusPipeline

SAMPLE_CSV = pathlib.Path(__file__).resolve().parent.parent / "flowstate" / "sample_eeg.csv"

# The R3 (10-bit ADC) streams at this rate; the bundled CSV was recorded at 500 Hz.
SERIAL_FS = 250
CSV_FS = 500


class FocusBridge:
    def __init__(self, link=None, source: str = "serial", alert_threshold: float = 40.0):
        self.source = source if source in ("serial", "csv") else "csv"
        self.link = link
        self.fs = SERIAL_FS if self.source == "serial" else CSV_FS
        self.pipeline = FocusPipeline(fs=self.fs, alert_threshold=alert_threshold,
                                      display_hz=100)
        self._lock = threading.Lock()
        self._latest = {"state": "warmup", "focus": 50.0, "alert": False,
                        "ready": False, "score_mode": "relative", "quality": "warmup"}
        self._stopped = False
        self._thread: threading.Thread | None = None

        self._csv = None       # np.ndarray of samples, lazily loaded for csv source
        self._csv_i = 0

    # --------------------------------------------------------------- lifecycle
    def start(self):
        if self.source == "csv":
            self._csv = self._load_csv()
        elif self.link is not None:
            self.link.set_stream(True)   # tell the R3 to start streaming A0
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        self._stopped = True
        if self.source == "serial" and self.link is not None:
            self.link.set_stream(False)
        if self._thread:
            self._thread.join(timeout=1.0)

    # ------------------------------------------------------------------- state
    @property
    def latest(self) -> dict:
        with self._lock:
            return dict(self._latest)

    @property
    def zone_out(self) -> bool:
        """True when focus has been sustained-low long enough to count as zoning out."""
        with self._lock:
            return bool(self._latest.get("alert", False))

    @property
    def focus(self) -> float:
        with self._lock:
            return float(self._latest.get("focus", 50.0))

    # --------------------------------------------------------------------- run
    def _run(self):
        last_compute = 0.0
        csv_chunk = max(1, self.fs // 50)
        csv_period = csv_chunk / self.fs
        next_csv = time.perf_counter()

        while not self._stopped:
            if self.source == "serial":
                if self.link is not None:
                    vals = self.link.drain_eeg()
                    if vals:
                        self.pipeline.add_samples(vals)
                time.sleep(0.01)
            else:  # csv replay, paced at real time
                now = time.perf_counter()
                if now >= next_csv and self._csv is not None and len(self._csv):
                    end = self._csv_i + csv_chunk
                    if end <= len(self._csv):
                        self.pipeline.add_samples(self._csv[self._csv_i:end])
                        self._csv_i = end
                    else:
                        self.pipeline.add_samples(self._csv[self._csv_i:])
                        self._csv_i = 0   # loop forever
                    next_csv += csv_period
                    if next_csv < now:           # fell behind; resync
                        next_csv = now + csv_period
                else:
                    time.sleep(0.002)

            now = time.monotonic()
            if now - last_compute >= 0.1:        # ~10 Hz focus updates
                packet = self.pipeline.compute()
                with self._lock:
                    self._latest = packet
                last_compute = now

    # --------------------------------------------------------------- csv load
    @staticmethod
    def _load_csv() -> np.ndarray:
        vals = []
        with open(SAMPLE_CSV, newline="") as f:
            reader = csv.DictReader(f)
            fields = reader.fieldnames or ["Channel1"]
            col = "Channel1" if "Channel1" in fields else fields[-1]
            for row in reader:
                try:
                    vals.append(float(row[col]))
                except (ValueError, KeyError, TypeError):
                    continue
        return np.asarray(vals, dtype=float)
