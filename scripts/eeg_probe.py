"""Standalone BioAmp / EEG (A0) stream probe for the DART Arduino.

Isolates the board from the whole DART stack (no YOLO / Flask / FocusPipeline) to
answer one question: when told to stream, does the Arduino actually send EEG
samples? Use it when FlowState shows "Serial: connected" but the focus gauge is
stuck on "warming up..." — that means no E#### samples are arriving, and this
tells you whether the fault is the board/firmware/wiring or something downstream.

It opens the port, waits for the board's one-shot READY, sends S1 to start the A0
stream, prints what comes back for a few seconds, then sends S0 and reports.

    python scripts/eeg_probe.py --port COM5 [--seconds 5]

Note: close the DART web app / main.py first — only one process can hold the port.
"""

from __future__ import annotations

import argparse
import time

import serial


def main():
    ap = argparse.ArgumentParser(description="Probe the DART Arduino EEG (A0) stream.")
    ap.add_argument("--port", default="COM3", help="serial port, e.g. COM5")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--seconds", type=float, default=5.0, help="how long to sample")
    args = ap.parse_args()

    print(f"[probe] opening {args.port} @ {args.baud} ...")
    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.5, write_timeout=1.0)
    except serial.SerialException as exc:
        print(f"[probe] FAIL: could not open {args.port}: {exc}")
        print("[probe] Is the port right, and is the DART app/main.py closed?")
        return

    # Opening asserts DTR -> the Uno resets. Clear buffers before it boots so we
    # don't miss the READY it prints ~700 ms into setup().
    ser.reset_input_buffer()
    ser.reset_output_buffer()

    print("[probe] waiting for READY ...")
    got_ready = False
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline:
        if ser.in_waiting:
            line = ser.readline().decode("ascii", errors="ignore").strip()
            print(f"[arduino] {line}")
            if line == "READY":
                got_ready = True
                break
        time.sleep(0.02)
    if not got_ready:
        print("[probe] WARN: no READY — wrong port, or board isn't running DART firmware.")

    print("[probe] sending S1 (start EEG stream) ...")
    ser.write(b"S1\n")
    ser.flush()

    n_eeg = 0
    other: list[str] = []
    vmin, vmax, vsum = 1 << 30, -(1 << 30), 0
    end = time.monotonic() + args.seconds
    while time.monotonic() < end:
        line = ser.readline().decode("ascii", errors="ignore").strip()
        if not line:
            continue
        if line[0] == "E":
            try:
                v = int(line[1:])
            except ValueError:
                continue
            n_eeg += 1
            vmin = min(vmin, v)
            vmax = max(vmax, v)
            vsum += v
        else:
            other.append(line)

    try:
        ser.write(b"S0\n")
        ser.flush()
    except serial.SerialException:
        pass
    ser.close()

    print("\n========== RESULT ==========")
    print(f"EEG samples received : {n_eeg}  (~{n_eeg / args.seconds:.0f}/s, expect ~250/s)")
    if n_eeg:
        print(f"value range          : {vmin}..{vmax}  mean {vsum / n_eeg:.0f}  (10-bit, 0..1023)")
        if vmax - vmin < 3:
            print("DIAGNOSIS: stream works but the signal is FLAT.")
            print("           -> check pill power (3.3/5V + GND), electrode contact, and that")
            print("              the pill's signal output is wired to A0.")
        else:
            print("DIAGNOSIS: the EEG stream is ALIVE and varying.")
            print("           -> the board is fine; if FlowState still warms up forever the")
            print("              problem is downstream in Python (report this back).")
    else:
        print("DIAGNOSIS: NO E#### samples — the board answered but never streamed A0.")
        print("           Most likely the Arduino has OLDER firmware without the S# / EEG")
        print("           feature. Re-upload:")
        print("             arduino/arduino_servo_controller/arduino_servo_controller.ino")
        print("           (and confirm the BioAmp signal pin is on A0).")
    if other:
        shown = other[:5]
        print(f"other lines seen     : {shown}{' ...' if len(other) > 5 else ''}")


if __name__ == "__main__":
    main()
