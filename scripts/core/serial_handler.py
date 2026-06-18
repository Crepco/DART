import serial

from config import STOP_PAN, STOP_TILT, MAX_SPEED


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def build_command(pan: int, tilt: int, fire: bool) -> bytes:
    p = clamp(pan,  STOP_PAN  - MAX_SPEED, STOP_PAN  + MAX_SPEED)
    t = clamp(tilt, STOP_TILT - MAX_SPEED, STOP_TILT + MAX_SPEED)
    return f"P{p:03d}T{t:03d}F{1 if fire else 0}\n".encode("ascii")


def send_stop(ser):
    if ser:
        try:
            ser.write(build_command(STOP_PAN, STOP_TILT, False))
            ser.flush()
        except serial.SerialException:
            pass
