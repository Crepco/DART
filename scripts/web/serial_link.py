"""Unified serial link to the single Arduino Uno R3.

DART and FlowState now share ONE board on ONE serial port, so they must share one
serial connection too. This class owns the `serial.Serial`, runs a reader thread,
and multiplexes the two data streams that cross the wire:

  host -> R3 :  "P###T###F#Z#\\n"  servo pan/tilt + fire flag (+ zone-out flag for pin 12)
                "S#\\n"             EEG stream enable(1)/disable(0)
  R3   -> host: "READY\\n"          one-shot boot handshake
                "E####\\n"          one raw EEG ADC sample (0..1023), only while streaming

The reader routes "READY" to the handshake and "E###" into an EEG buffer the focus
bridge drains; everything else is logged. Writes are serialized behind a lock so the
runner thread (servo commands) and the focus loop (stream toggle) can't interleave.
"""

from __future__ import annotations

import threading
import time
from collections import deque

import serial

from config import PORT, BAUD_RATE, STOP_PAN, STOP_TILT, MAX_SPEED


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


def build_command(pan: int, tilt: int, fire: bool, zone: bool = False) -> bytes:
    """`P###T###F#Z#` — pan/tilt clamped to the safe speed window, fire + zone flags.

    `Z` mirrors the FlowState zone-out state onto R3 pin 12 (status LED/buzzer). It is
    independent of `F` (the trigger servo) so pin 12 can show "armed because you zoned
    out" even before a lock dwells into an actual shot.
    """
    p = _clamp(pan,  STOP_PAN  - MAX_SPEED, STOP_PAN  + MAX_SPEED)
    t = _clamp(tilt, STOP_TILT - MAX_SPEED, STOP_TILT + MAX_SPEED)
    return f"P{p:03d}T{t:03d}F{1 if fire else 0}Z{1 if zone else 0}\n".encode("ascii")


class SerialLink:
    """Owns the one shared serial port; safe for one writer + the reader thread."""

    def __init__(self, port: str = PORT, baud: int = BAUD_RATE):
        self.port = port
        self.baud = baud
        self.ser: serial.Serial | None = None
        self.connected = False

        self._write_lock = threading.Lock()
        self._eeg_lock = threading.Lock()
        self._eeg = deque(maxlen=20000)   # ~80 s at 250 Hz; drained continuously
        self._stopped = False
        self._reader: threading.Thread | None = None
        self.streaming = False

    # ------------------------------------------------------------- connection
    def connect(self, handshake_timeout: float = 5.0) -> bool:
        """Open the port and wait for the board's one-shot READY.

        Returns True if the Arduino answered; False (preview mode) if the port can't
        be opened or no board responds — callers then run without servos/EEG.
        """
        try:
            # write_timeout guards against a phantom COM port that opens but nothing
            # drains (e.g. a Bluetooth port with no Arduino behind it).
            self.ser = serial.Serial(self.port, self.baud, timeout=0.5, write_timeout=1.0)
        except serial.SerialException as exc:
            print(f"[WARN] Could not open {self.port}: {exc}")
            print("[WARN] Running in preview-only mode (no serial output).")
            self.ser = None
            return False

        # Opening the port asserts DTR, which resets the Uno. Clear buffers BEFORE the
        # board boots so we don't drop the "READY" it prints ~700 ms into setup().
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        print("[INFO] Waiting for Arduino reset ...")

        got_ready = False
        deadline = time.monotonic() + handshake_timeout
        while time.monotonic() < deadline:
            if self.ser.in_waiting:
                line = self.ser.readline().decode("ascii", errors="ignore").strip()
                print(f"[ARDUINO] {line}")
                if line == "READY":
                    got_ready = True
                    break
            time.sleep(0.05)

        if not got_ready:
            print(f"[WARN] No READY handshake on {self.port} — no Arduino responding.")
            print("[WARN] Running in preview-only mode (no serial output).")
            self.ser.close()
            self.ser = None
            return False

        self.connected = True
        self.send_stop()
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()
        print(f"[INFO] Serial ready on {self.port} @ {self.baud}")
        return True

    # ------------------------------------------------------------------ reader
    def _read_loop(self):
        assert self.ser is not None
        while not self._stopped:
            try:
                line = self.ser.readline().decode("ascii", errors="ignore").strip()
            except (serial.SerialException, OSError):
                break
            if not line:
                continue
            if line[0] == "E":
                try:
                    val = int(line[1:])
                except ValueError:
                    continue
                with self._eeg_lock:
                    self._eeg.append(val)
            elif line == "READY":
                continue  # spurious reset; ignore
            else:
                print(f"[ARDUINO] {line}")

    def drain_eeg(self) -> list[int]:
        """Return and clear all EEG samples received since the last call."""
        with self._eeg_lock:
            if not self._eeg:
                return []
            out = list(self._eeg)
            self._eeg.clear()
            return out

    # ------------------------------------------------------------------ writer
    def _write(self, payload: bytes, flush: bool = False) -> None:
        if not self.ser:
            return
        with self._write_lock:
            try:
                self.ser.write(payload)
                if flush:
                    self.ser.flush()
            except serial.SerialException:
                pass

    def send_command(self, pan: int, tilt: int, fire: bool, zone: bool = False,
                     flush: bool = False) -> None:
        self._write(build_command(pan, tilt, fire, zone), flush=flush)

    def set_stream(self, enable: bool) -> None:
        """Enable/disable the R3's EEG sample stream (off by default => mode 2 quiet)."""
        self.streaming = enable
        self._write(f"S{1 if enable else 0}\n".encode("ascii"), flush=True)

    def send_stop(self) -> None:
        self._write(build_command(STOP_PAN, STOP_TILT, False, False), flush=True)

    # ------------------------------------------------------------------- close
    def close(self) -> None:
        self._stopped = True
        if self.streaming:
            self.set_stream(False)
        self.send_stop()
        if self._reader:
            self._reader.join(timeout=1.0)
        if self.ser:
            try:
                self.ser.close()
            except serial.SerialException:
                pass
        self.ser = None
        self.connected = False
