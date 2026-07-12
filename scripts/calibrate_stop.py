"""Find each continuous-rotation servo's true neutral (the command where it
physically stops).

Why: write(90) is not guaranteed to be a CR servo's null. Hardware validation
(2026-07-12) showed the tilt axis creeping at a commanded 90 — 23/23 deadband
escapes in one direction with zero commanded output — so the control loop
endlessly re-centers a self-moving plant and the turret never rests. Run this,
find the value per axis where the turret visibly holds still for ~30 s, and
put those values in scripts/config.py as STOP_PAN / STOP_TILT (every command
is built relative to them).

Usage:  python scripts/calibrate_stop.py [--port COM6]

  a / d   pan  -1 / +1
  w / s   tilt -1 / +1
  q       quit — prints ready-to-paste config lines

Safety: fire is never sent (F0 always). Commands resend at ~5 Hz because the
firmware stops all servos after 500 ms of host silence (TIMEOUT_MS failsafe) —
which also means a hard crash of this script leaves the turret parked by the
firmware within half a second. Note the firmware failsafe parks at its own
hardcoded 90/90, so a trimmed axis may still creep slowly while DART is NOT
running; the config trim fixes at-rest behavior while DART runs. If no integer
degree holds an axis still, the servo's physical trim pot is the precise fix
(and cures idle creep too).
"""

from __future__ import annotations

import argparse
import msvcrt
import pathlib
import sys
import time

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from config import PORT, STOP_PAN, STOP_TILT              # noqa: E402
from web.serial_link import SerialLink                    # noqa: E402

SEND_HZ = 5          # > firmware 500 ms timeout with plenty of margin
TRIM_RANGE = 10      # sanity clamp: candidates stay within STOP_* ± this

KEYMAP = {"a": (-1, 0), "d": (+1, 0), "w": (0, -1), "s": (0, +1)}


def main() -> int:
    ap = argparse.ArgumentParser(
        description="Interactive servo neutral-point calibration.")
    ap.add_argument("--port", default=PORT,
                    help=f"Arduino serial port (default {PORT})")
    args = ap.parse_args()

    link = SerialLink(args.port)
    if not link.connect():
        print("[ERROR] No Arduino answered — check the port (try --port COM6).")
        return 1

    pan, tilt = STOP_PAN, STOP_TILT
    print()
    print("Adjust each axis until the turret visibly holds still for ~30s.")
    print("  a/d = pan -/+    w/s = tilt -/+    q = finish")
    print(f"  holding pan={pan} tilt={tilt} — watch the turret\n")

    try:
        while True:
            if msvcrt.kbhit():
                key = msvcrt.getwch().lower()
                if key == "q":
                    break
                if key in KEYMAP:
                    dp, dt = KEYMAP[key]
                    pan = max(STOP_PAN - TRIM_RANGE,
                              min(STOP_PAN + TRIM_RANGE, pan + dp))
                    tilt = max(STOP_TILT - TRIM_RANGE,
                               min(STOP_TILT + TRIM_RANGE, tilt + dt))
                    print(f"  pan={pan:3d}  tilt={tilt:3d}")
            link.send_command(pan, tilt, False)
            time.sleep(1.0 / SEND_HZ)
    except KeyboardInterrupt:
        pass
    finally:
        # Leave the turret on the best-known neutral. Deliberately NOT
        # link.close(): that would first send the (uncalibrated) config STOP
        # values over our candidates. ~0.5s later the firmware failsafe takes
        # over at its own 90/90 regardless.
        link.send_command(pan, tilt, False, flush=True)
        if link.ser:
            link.ser.close()

    print("\nCalibrated neutrals — paste into scripts/config.py:")
    print(f"STOP_PAN    = {pan}")
    print(f"STOP_TILT   = {tilt}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
