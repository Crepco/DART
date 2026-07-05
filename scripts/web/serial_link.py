"""Serial link to the single Arduino Uno R3.

Owns the `serial.Serial` and writes servo commands to the board. The link is write-only
after the boot handshake — the Arduino only ever emits a one-shot "READY" (consumed during
connect); nothing streams back during operation. Writes are serialized behind a lock.

  host -> R3 :  "P###T###F#\\n"    servo pan/tilt + fire flag
  R3   -> host: "READY\\n"          one-shot boot handshake
"""

from __future__ import annotations

import threading
import time

import serial

from config import PORT, BAUD_RATE, STOP_PAN, STOP_TILT, MAX_SPEED


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


def build_command(pan: int, tilt: int, fire: bool) -> bytes:
    """`P###T###F#` — pan/tilt clamped to the safe speed window, plus the fire flag."""
    p = _clamp(pan,  STOP_PAN  - MAX_SPEED, STOP_PAN  + MAX_SPEED)
    t = _clamp(tilt, STOP_TILT - MAX_SPEED, STOP_TILT + MAX_SPEED)
    return f"P{p:03d}T{t:03d}F{1 if fire else 0}\n".encode("ascii")


class SerialLink:
    """Owns the serial port; safe for a single writer."""

    def __init__(self, port: str = PORT, baud: int = BAUD_RATE):
        self.port = port
        self.baud = baud
        self.ser: serial.Serial | None = None
        self.connected = False

        self._write_lock = threading.Lock()

    # ------------------------------------------------------------- connection
    def connect(self, handshake_timeout: float = 5.0) -> bool:
        """Open the port and wait for the board's one-shot READY.

        Returns True if the Arduino answered; False (preview mode) if the port can't
        be opened or no board responds — callers then run without servos.
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
        print(f"[INFO] Serial ready on {self.port} @ {self.baud}")
        return True

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

    def send_command(self, pan: int, tilt: int, fire: bool,
                     flush: bool = False) -> None:
        self._write(build_command(pan, tilt, fire), flush=flush)

    def send_stop(self) -> None:
        self._write(build_command(STOP_PAN, STOP_TILT, False), flush=True)

    # ------------------------------------------------------------------- close
    def close(self) -> None:
        self.send_stop()
        if self.ser:
            try:
                self.ser.close()
            except serial.SerialException:
                pass
        self.ser = None
        self.connected = False
