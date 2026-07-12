"""DART body-tracking pan-tilt turret (YOLO edition). See docs/ARCHITECTURE.md and docs/SERIAL_PROTOCOL.md."""

import argparse
import math
import os
import time
import warnings

import cv2
import numpy as np
import serial

from config import *
from core import CameraStream, YOLODetector, PID, build_command, send_stop
from core.detector import select_face
from core.pid import asym_ema, slew_step

warnings.filterwarnings("ignore", category=FutureWarning)


def apply_output_deadband(value: float, band: float) -> float:
    return 0.0 if abs(value) < band else value


def update_fire_state(error_x, error_y, locked_face_auth, fire, lock_count):
    """Lock-on + auth fire gate. Returns (fire, lock_count, locked).

    Auth gate: only an UNAUTHORIZED target may ever arm/fire. For an AUTHORIZED
    or UNKNOWN target, hold fire and don't accrue dwell. Dwell + hysteresis stop
    the trigger chattering when the error hovers at the boundary.
    """
    lock_err = math.hypot(error_x, error_y)
    if locked_face_auth != "UNAUTHORIZED":
        return False, 0, lock_err <= LOCK_ON_RADIUS
    if fire:
        if lock_err > LOCK_RELEASE_RADIUS:
            return False, 0, lock_err <= LOCK_ON_RADIUS
        return True, lock_count, lock_err <= LOCK_ON_RADIUS
    lock_count = lock_count + 1 if lock_err <= LOCK_ON_RADIUS else 0
    return lock_count >= FIRE_DWELL_FRAMES, lock_count, lock_err <= LOCK_ON_RADIUS


AUTH_COLS = {"AUTHORIZED":   COL_AUTHORIZED,
             "UNAUTHORIZED": COL_UNAUTHORIZED,
             "UNKNOWN":      COL_AUTH_UNKNOWN}


def draw_hud(frame, status, pan_cmd, tilt_cmd,
             smooth_cx, smooth_cy,
             person_box, all_persons, faces, track_id,
             w_f, h_f, error_x, error_y, fire, locked, locked_face_auth, auth_statuses):

    tracking = (status == "TRACKING")
    bar_col  = COL_TRACKING if tracking else COL_NO_BODY
    cx_f, cy_f = w_f // 2, h_f // 2

    for (x1, y1, x2, y2, tid) in all_persons:
        auth_status = auth_statuses.get(tid, "UNKNOWN")
        col         = AUTH_COLS[auth_status]
        is_target   = (tid == track_id)
        thickness   = 3 if is_target else 1
        cv2.rectangle(frame, (x1, y1), (x2, y2), col, thickness, cv2.LINE_AA)
        cv2.putText(frame, f"ID{tid} {auth_status}", (x1, y1 - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, col, 1, cv2.LINE_AA)
        if auth_status == "UNAUTHORIZED":
            cv2.circle(frame, (x2 - 8, y1 + 8), 5, COL_UNAUTHORIZED, -1, cv2.LINE_AA)

    for (x1, y1, x2, y2) in faces:
        cv2.rectangle(frame, (x1, y1), (x2, y2), COL_FACE_BOX, 2, cv2.LINE_AA)
        cv2.putText(frame, "Face", (x1, y1 - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, COL_FACE_BOX, 1, cv2.LINE_AA)

    if smooth_cx is not None and tracking:
        cv2.line(frame, (int(smooth_cx), int(smooth_cy)), (cx_f, cy_f),
                 (80, 80, 80), 1, cv2.LINE_AA)
        cv2.circle(frame, (int(smooth_cx), int(smooth_cy)), 8, COL_CENTROID, -1, cv2.LINE_AA)
        cv2.circle(frame, (int(smooth_cx), int(smooth_cy)), 8, (0, 0, 0), 1, cv2.LINE_AA)

    overlay = frame.copy()
    cv2.rectangle(overlay, (0, 0), (w_f, 40), (10, 10, 10), -1)
    cv2.addWeighted(overlay, 0.45, frame, 0.55, 0, frame)

    if fire:
        fire_label, fire_col = "FIRING", COL_FIRING
    elif locked and locked_face_auth == "UNAUTHORIZED":
        fire_label, fire_col = "ARMED", COL_ARMED
    elif locked and locked_face_auth == "AUTHORIZED":
        fire_label, fire_col = "SAFE/AUTH", COL_AUTHORIZED
    else:
        fire_label, fire_col = "SAFE", (120, 120, 120)
    tid_label  = f"  ID={track_id}" if track_id is not None else ""
    bar_text   = (f"[{status}]{tid_label}   Pan {pan_cmd:3d}  Tilt {tilt_cmd:3d}"
                  f"   Err ({int(error_x):+d}, {int(error_y):+d})")
    cv2.putText(frame, bar_text, (10, 27),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, bar_col, 2, cv2.LINE_AA)
    cv2.putText(frame, fire_label, (w_f - 140, 27),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, fire_col, 2, cv2.LINE_AA)

    arm = 10 if tracking else 18
    cv2.line(frame, (cx_f - arm, cy_f), (cx_f + arm, cy_f), COL_RETICLE, 1, cv2.LINE_AA)
    cv2.line(frame, (cx_f, cy_f - arm), (cx_f, cy_f + arm), COL_RETICLE, 1, cv2.LINE_AA)
    cv2.circle(frame, (cx_f, cy_f), arm + 8, COL_RETICLE, 1, cv2.LINE_AA)

    cv2.circle(frame, (cx_f, cy_f), LOCK_ON_RADIUS, fire_col, 1, cv2.LINE_AA)

    cv2.putText(frame, "Q = quit", (10, h_f - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.38, (100, 100, 100), 1, cv2.LINE_AA)


def main():
    parser = argparse.ArgumentParser(description="DART body tracker — YOLO Edition")
    parser.add_argument("--camera",       type=int, default=CAM_INDEX, help="Camera index (default 0)")
    parser.add_argument("--port",         type=str, default=PORT,      help="Serial port (default COM3)")
    parser.add_argument("--person-model", type=str, default=PERSON_MODEL)
    parser.add_argument("--face-model",   type=str, default=FACE_MODEL)
    args = parser.parse_args()

    print(f"[INFO] Opening camera {args.camera} ...")
    cam = CameraStream(src=args.camera)
    grabbed, test_frame = cam.read()
    if not grabbed or test_frame is None:
        raise RuntimeError("Camera opened but returned no frame.")
    print(f"[INFO] Camera OK — {test_frame.shape[1]}×{test_frame.shape[0]}")

    face_path = args.face_model if os.path.exists(args.face_model) else None
    detector  = YOLODetector(args.person_model, face_path)

    pid_pan  = PID(PAN_KP,  PAN_KI,  PAN_KD,  MAX_SPEED)
    pid_tilt = PID(TILT_KP, TILT_KI, TILT_KD, MAX_SPEED)

    ser = None
    try:
        # write_timeout: don't hang forever on a port nothing drains (phantom
        # Bluetooth COM, no Arduino).
        ser = serial.Serial(args.port, BAUD_RATE, timeout=0.5, write_timeout=1.0)
        # Opening asserts DTR -> Uno resets. Clear buffers BEFORE it boots so
        # the one-shot "READY" (~700ms later) isn't discarded.
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        print("[INFO] Waiting for Arduino reset ...")
        got_ready = False
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            if ser.in_waiting:
                line = ser.readline().decode("ascii", errors="ignore").strip()
                print(f"[ARDUINO] {line}")
                if line == "READY":
                    got_ready = True
                    break
            time.sleep(0.05)
        if got_ready:
            send_stop(ser)
            print(f"[INFO] Serial ready on {args.port} @ {BAUD_RATE}")
        else:
            # Port opened but no Arduino answered — preview-only beats a loop
            # stalled on a dead port.
            print(f"[WARN] No READY handshake on {args.port} — no Arduino responding.")
            print("[WARN] Running in preview-only mode (no serial output).")
            ser.close()
            ser = None
    except serial.SerialException as exc:
        print(f"[WARN] Could not open {args.port}: {exc}")
        print("[WARN] Running in preview-only mode (no serial output).")
        ser = None

    smooth_cx     = None
    smooth_cy     = None
    no_body_count = 0
    last_seq      = -1
    smooth_pan    = 0.0
    smooth_tilt   = 0.0
    slew_pan      = 0.0
    slew_tilt     = 0.0
    last_send     = 0.0
    last_time     = time.monotonic()
    error_x       = 0.0
    error_y       = 0.0
    last_sent_pan  = STOP_PAN
    last_sent_tilt = STOP_TILT
    last_sent_fire = False
    fire          = False
    lock_count    = 0
    locked        = False
    pid_live      = False   # False across coast/lost frames -> resync on resume

    print("[INFO] Running — press Q to quit")

    while True:
        grabbed, frame, seq = cam.read_seq()
        # Gate on NEW frames — re-processing the latest frame at CPU speed runs
        # the control loop at thousands of Hz: the PID derivative collapses
        # (error unchanged between duplicates) and the per-frame EMAs decay far
        # faster than their constants suggest.
        if not grabbed or frame is None or seq == last_seq:
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
            time.sleep(0.002)
            continue
        last_seq = seq

        now       = time.monotonic()
        dt        = max(now - last_time, 1e-6)
        last_time = now

        h_f, w_f  = frame.shape[:2]
        cx_frame  = w_f // 2
        cy_frame  = h_f // 2

        detector.submit(frame)
        det = detector.get_result()

        person_box    = det["person_box"]
        track_id      = det["track_id"]
        all_persons   = det["all_persons"]
        faces         = det["faces"]
        auth_statuses = det["auth_statuses"]
        locked_face_auth = det.get("locked_face_auth", "UNKNOWN")

        pan_raw  = 0.0
        tilt_raw = 0.0
        pan_pid  = 0.0
        tilt_pid = 0.0
        status   = "NO BODY"

        target_box = select_face(faces, smooth_cx, smooth_cy)

        if target_box is not None and person_box is not None:
            no_body_count = 0
            x1, y1, x2, y2 = target_box
            raw_cx = (x1 + x2) // 2
            raw_cy = (y1 + y2) // 2

            if smooth_cx is None:
                smooth_cx = float(raw_cx)
                smooth_cy = float(raw_cy)
            else:
                smooth_cx = SMOOTH * smooth_cx + (1.0 - SMOOTH) * raw_cx
                smooth_cy = SMOOTH * smooth_cy + (1.0 - SMOOTH) * raw_cy

            error_x = smooth_cx - cx_frame
            error_y = smooth_cy - cy_frame

            # Lock-on detection uses the true centre error (hysteresis + dwell).
            fire, lock_count, locked = update_fire_state(
                error_x, error_y, locked_face_auth, fire, lock_count)

            if abs(error_x) < INPUT_DEADBAND:
                error_x = 0.0
            if abs(error_y) < INPUT_DEADBAND:
                error_y = 0.0

            # Re-anchor the derivative after any gap: coast/lost frames freeze
            # the PID (no update), so prev_error is stale — a derivative
            # computed across the gap is fictitious and saturates the command
            # (the hardware "lurch").
            if not pid_live:
                pid_pan.resync(error_x * INVERT_PAN)
                pid_tilt.resync(error_y * INVERT_TILT)
                pid_live = True

            pan_pid  = pid_pan.update(error_x  * INVERT_PAN,  dt, INTEGRAL_CLAMP)
            tilt_pid = pid_tilt.update(error_y * INVERT_TILT, dt, INTEGRAL_CLAMP)

            pan_raw  = apply_output_deadband(pan_pid,  OUTPUT_DEADBAND)
            tilt_raw = apply_output_deadband(tilt_pid, OUTPUT_DEADBAND)

            status = "TRACKING"

        elif smooth_cx is not None and no_body_count < FIRE_COAST_FRAMES:
            # Coast through brief target loss (motion blur drops the face for a
            # few frames): hold motion at zero — settle via EMA + slew, don't
            # drive blind or reset the PID. The fire dwell keeps accruing while
            # the target is still tracked UNAUTHORIZED and HOLDS otherwise;
            # lock_count only resets in the lost branch below.
            no_body_count += 1
            pid_live = False
            error_x = smooth_cx - cx_frame
            error_y = smooth_cy - cy_frame
            if locked_face_auth == "UNAUTHORIZED":
                fire, lock_count, locked = update_fire_state(
                    error_x, error_y, locked_face_auth, fire, lock_count)
            # else: hold fire / lock_count / locked unchanged
            error_x = error_y = 0.0
            status = "TRACKING"

        else:
            no_body_count += 1
            pid_live = False
            error_x = error_y = 0.0
            fire = False
            locked = False
            lock_count = 0
            if no_body_count > 15:
                smooth_cx   = smooth_cy   = None   # reset anchor — next face becomes the anchor
                smooth_pan  = smooth_tilt = 0.0
                slew_pan    = slew_tilt   = 0.0
                pid_pan.reset()
                pid_tilt.reset()

        smooth_pan  = asym_ema(smooth_pan,  pan_raw,  CMD_SMOOTH, CMD_SMOOTH_DECAY)
        smooth_tilt = asym_ema(smooth_tilt, tilt_raw, CMD_SMOOTH, CMD_SMOOTH_DECAY)

        slew_pan  = slew_step(slew_pan,  smooth_pan,
                              MAX_CMD_CHANGE_PER_SEC_PAN  * dt, SLEW_BRAKE_MULT)
        slew_tilt = slew_step(slew_tilt, smooth_tilt,
                              MAX_CMD_CHANGE_PER_SEC_TILT * dt, SLEW_BRAKE_MULT)

        # Deadband the FINAL offset too: the EMA/slew decay tail otherwise
        # dribbles 1-3 unit commands the CR servos act on (creep past center).
        pan_out  = apply_output_deadband(slew_pan,  OUTPUT_DEADBAND)
        tilt_out = apply_output_deadband(slew_tilt, OUTPUT_DEADBAND)
        pan_cmd  = STOP_PAN  + int(round(pan_out))
        tilt_cmd = STOP_TILT + int(round(tilt_out))

        if CONTROL_DEBUG:
            print(f"[CTRL] dt={dt*1000:4.0f}ms "
                  f"pan(err={error_x:+6.1f} pid={pan_pid:+6.2f} ema={smooth_pan:+6.2f} "
                  f"slew={slew_pan:+6.2f} cmd={pan_cmd:3d}) "
                  f"tilt(err={error_y:+6.1f} pid={tilt_pid:+6.2f} ema={smooth_tilt:+6.2f} "
                  f"slew={slew_tilt:+6.2f} cmd={tilt_cmd:3d})")

        draw_hud(frame, status, pan_cmd, tilt_cmd,
                 smooth_cx, smooth_cy,
                 person_box, all_persons, faces, track_id,
                 w_f, h_f, error_x, error_y, fire, locked, locked_face_auth, auth_statuses)

        if ser:
            time_since_send = now - last_send
            fire_changed = (fire != last_sent_fire)
            moved        = (pan_cmd != last_sent_pan or tilt_cmd != last_sent_tilt)
            if (fire_changed
                    or (moved and time_since_send >= SEND_INTERVAL)
                    or (time_since_send >= HEARTBEAT_INTERVAL)):
                try:
                    ser.write(build_command(pan_cmd, tilt_cmd, fire))
                    if fire_changed:
                        ser.flush()   # fire/cease must hit the wire immediately
                    last_sent_pan  = pan_cmd
                    last_sent_tilt = tilt_cmd
                    last_sent_fire = fire
                    last_send      = now
                except serial.SerialException:
                    pass

        cv2.imshow("DART  [YOLO + Slew Rate Limiter]", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    send_stop(ser)
    if ser:
        ser.close()
    detector.stop()
    cam.stop()
    cv2.destroyAllWindows()
    print("[INFO] Shut down.")


if __name__ == "__main__":
    main()
