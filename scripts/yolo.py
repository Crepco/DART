"""DART body-tracking pan-tilt turret (YOLO edition). See DOCS_ARCHITECTURE.md and DOCS_SERIAL_PROTOCOL.md."""

import argparse
import copy
import math
import os
import pathlib
import threading
import time
import warnings

import cv2
import numpy as np
import serial
from ultralytics import YOLO

warnings.filterwarnings("ignore", category=FutureWarning)


# ── Camera ──
CAM_INDEX  = 0
CAM_WIDTH  = 640
CAM_HEIGHT = 480
CAM_FPS    = 15

# ── Models ──
SCRIPT_DIR   = pathlib.Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parent
PERSON_MODEL = str(PROJECT_ROOT / "models" / "yolov8n.pt")
FACE_MODEL   = str(PROJECT_ROOT / "models" / "yolov8n-face.pt")
LABELS_FILE  = str(PROJECT_ROOT / "labels.txt")
PERSON_CONF  = 0.50
FACE_CONF    = 0.45

# ── Servo ──
STOP_PAN    = 90
STOP_TILT   = 90
MAX_SPEED   = 25
INVERT_PAN  = -1
INVERT_TILT = -1

# ── Trigger (MG90) — angles live on the Arduino; mirrored here for parity ──
TRIGGER_REST_ANGLE = 60
TRIGGER_FIRE_ANGLE = 150
LOCK_ON_RADIUS      = 40    # px from centre to engage
LOCK_RELEASE_RADIUS = 70    # px from centre to disengage (hysteresis)
FIRE_DWELL_FRAMES   = 3     # consecutive on-target frames before firing

# ── Dead bands ──
OUTPUT_DEADBAND = 3
INPUT_DEADBAND  = 15

# ── PID gains (tuning guide: DOCS_ARCHITECTURE.md) ──
PAN_KP,  PAN_KI,  PAN_KD  = 0.04, 0.001, 0.12
TILT_KP, TILT_KI, TILT_KD = 0.04, 0.001, 0.11
INTEGRAL_CLAMP = 10.0

# ── Smoothing ──
SMOOTH     = 0.85   # centroid EMA (weight on previous value)
CMD_SMOOTH = 0.80   # servo command EMA — new-sample weight = 1-CMD_SMOOTH ≈ 0.2

# ── Slew rate limiter (servo-offset units / second, dt-scaled) ──
MAX_CMD_CHANGE_PER_SEC = 25.0

# ── Serial ──
PORT          = "COM3"
BAUD_RATE     = 115200
SEND_HZ       = 30
SEND_INTERVAL = 1.0 / SEND_HZ
HEARTBEAT_INTERVAL = 0.4

# ── HUD colours (BGR) ──
COL_TRACKING  = (0, 220,  80)
COL_NO_BODY   = (0,  60, 220)
COL_RETICLE   = (220, 220,  0)
COL_PERSON    = (0, 200, 255)
COL_CENTROID  = (0, 255, 255)
COL_FACE_BOX  = (0, 220,  80)
COL_TRACK_ID  = (255, 180,  0)
COL_ARMED     = (0, 180, 255)
COL_FIRING    = (0,  40, 255)


class PID:
    def __init__(self, kp, ki, kd, limit):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.limit      = limit
        self.integral   = 0.0
        self.prev_error = 0.0

    def update(self, error: float, dt: float, integral_clamp: float = None) -> float:
        dt    = max(dt, 1e-6)
        i_lim = integral_clamp if integral_clamp is not None else self.limit
        self.integral = float(np.clip(self.integral + error * dt, -i_lim, i_lim))
        derivative      = (error - self.prev_error) / dt
        self.prev_error = error
        raw = self.kp * error + self.ki * self.integral + self.kd * derivative
        return float(np.clip(raw, -self.limit, self.limit))

    def reset(self):
        self.integral   = 0.0
        self.prev_error = 0.0


class CameraStream:
    def __init__(self, src: int = 0):
        self.cap = cv2.VideoCapture(src)
        if not self.cap.isOpened():
            raise RuntimeError(f"Camera {src} not available. Try --camera 0 or 1")
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  CAM_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_HEIGHT)
        self.cap.set(cv2.CAP_PROP_FPS,          CAM_FPS)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE,   1)
        self.grabbed, self.frame = self.cap.read()
        self._lock    = threading.Lock()
        self._stopped = False
        self._thread  = threading.Thread(target=self._update, daemon=True)
        self._thread.start()

    def _update(self):
        while not self._stopped:
            g, f = self.cap.read()
            with self._lock:
                self.grabbed, self.frame = g, f

    def read(self):
        with self._lock:
            return self.grabbed, (self.frame.copy() if self.frame is not None else None)

    def stop(self):
        self._stopped = True
        self._thread.join()
        self.cap.release()


class YOLODetector:
    def __init__(self, person_model_path: str, face_model_path: str | None):
        self._lock      = threading.Lock()
        self._frame     = None
        self._new_frame = False
        self._result    = {
            "person_box":  None,
            "track_id":    None,
            "all_persons": [],
            "faces":       [],
        }
        self._stopped = False

        print(f"[INFO] Loading person model: {person_model_path}")
        self._person_model = YOLO(person_model_path)

        self._face_model = None
        if face_model_path and os.path.exists(face_model_path):
            print(f"[INFO] Loading face model: {face_model_path}")
            self._face_model = YOLO(face_model_path)
        else:
            print(f"[WARN] Face model not found ({face_model_path}). Face boxes disabled.")

        dummy = np.zeros((CAM_HEIGHT, CAM_WIDTH, 3), dtype=np.uint8)
        self._person_model.track(dummy, persist=True, verbose=False)
        if self._face_model:
            self._face_model(dummy, verbose=False)

        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def submit(self, frame):
        with self._lock:
            self._frame     = frame
            self._new_frame = True

    def get_result(self):
        with self._lock:
            return copy.deepcopy(self._result)

    def stop(self):
        self._stopped = True
        self._thread.join()

    def _run(self):
        locked_id = None

        while not self._stopped:
            with self._lock:
                if not self._new_frame or self._frame is None:
                    frame = None
                else:
                    frame = self._frame.copy()
                    self._new_frame = False

            if frame is None:
                time.sleep(0.005)
                continue

            try:
                p_results = self._person_model.track(
                    frame, persist=True, classes=[0], conf=PERSON_CONF,
                    verbose=False, tracker="bytetrack.yaml",
                )
            except Exception as e:
                print(f"[WARN] Person model error: {e}")
                continue

            all_persons = []
            person_box  = None
            track_id    = None

            r = p_results[0]
            if r.boxes is not None and len(r.boxes):
                ids = (r.boxes.id.int().cpu().tolist()
                       if r.boxes.id is not None else
                       list(range(len(r.boxes))))

                for box, tid in zip(r.boxes.xyxy.cpu().tolist(), ids):
                    x1, y1, x2, y2 = [int(v) for v in box]
                    all_persons.append((x1, y1, x2, y2, tid))

                current_ids = [p[4] for p in all_persons]
                if locked_id not in current_ids:
                    locked_id = current_ids[0]

                for (x1, y1, x2, y2, tid) in all_persons:
                    if tid == locked_id:
                        person_box = (x1, y1, x2, y2)
                        track_id   = tid
                        break
            else:
                locked_id = None

            faces = []
            if self._face_model is not None:
                try:
                    f_results = self._face_model(frame, conf=FACE_CONF, verbose=False)
                    fr = f_results[0]
                    if fr.boxes is not None:
                        for box in fr.boxes.xyxy.cpu().tolist():
                            faces.append(tuple(int(v) for v in box))
                except Exception:
                    pass

            with self._lock:
                self._result = {
                    "person_box":  person_box,
                    "track_id":    track_id,
                    "all_persons": all_persons,
                    "faces":       faces,
                }


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def select_face(faces: list, prev_cx, prev_cy):
    """Pick the face nearest the previous anchor (area minus distance). If no
    anchor yet, pick the largest face. Prevents frame-to-frame target hopping."""
    if not faces:
        return None
    if prev_cx is None:
        return max(faces, key=lambda f: (f[2] - f[0]) * (f[3] - f[1]))

    def score(f):
        x1, y1, x2, y2 = f
        cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
        area = (x2 - x1) * (y2 - y1)
        dist = math.hypot(cx - prev_cx, cy - prev_cy)
        return area - dist * 0.4

    return max(faces, key=score)


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


def apply_output_deadband(value: float, band: float) -> float:
    return 0.0 if abs(value) < band else value


def draw_hud(frame, status, pan_cmd, tilt_cmd,
             smooth_cx, smooth_cy,
             person_box, all_persons, faces, track_id,
             w_f, h_f, error_x, error_y, fire, locked):

    tracking = (status == "TRACKING")
    bar_col  = COL_TRACKING if tracking else COL_NO_BODY
    cx_f, cy_f = w_f // 2, h_f // 2

    for (x1, y1, x2, y2, tid) in all_persons:
        is_target = (tid == track_id)
        col       = COL_PERSON if is_target else (80, 80, 80)
        thickness = 2 if is_target else 1
        cv2.rectangle(frame, (x1, y1), (x2, y2), col, thickness, cv2.LINE_AA)
        cv2.putText(frame, f"ID{tid}", (x1, y1 - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, COL_TRACK_ID, 1, cv2.LINE_AA)

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

    fire_label = "FIRING" if fire else ("ARMED" if locked else "SAFE")
    fire_col   = COL_FIRING if fire else (COL_ARMED if locked else (120, 120, 120))
    tid_label  = f"  ID={track_id}" if track_id is not None else ""
    bar_text   = (f"[{status}]{tid_label}   Pan {pan_cmd:3d}  Tilt {tilt_cmd:3d}"
                  f"   Err ({int(error_x):+d}, {int(error_y):+d})")
    cv2.putText(frame, bar_text, (10, 27),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, bar_col, 2, cv2.LINE_AA)
    cv2.putText(frame, fire_label, (w_f - 110, 27),
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
        ser = serial.Serial(args.port, BAUD_RATE, timeout=0.5)
        print("[INFO] Waiting for Arduino reset ...")
        time.sleep(2.5)
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            if ser.in_waiting:
                line = ser.readline().decode("ascii", errors="ignore").strip()
                print(f"[ARDUINO] {line}")
                if line == "READY":
                    break
            time.sleep(0.05)
        send_stop(ser)
        print(f"[INFO] Serial ready on {args.port} @ {BAUD_RATE}")
    except serial.SerialException as exc:
        print(f"[WARN] Could not open {args.port}: {exc}")
        print("[WARN] Running in preview-only mode (no serial output).")

    smooth_cx     = None
    smooth_cy     = None
    no_body_count = 0
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

    print("[INFO] Running — press Q to quit")

    while True:
        grabbed, frame = cam.read()
        if not grabbed or frame is None:
            time.sleep(0.01)
            continue

        now       = time.monotonic()
        dt        = max(now - last_time, 1e-6)
        last_time = now

        h_f, w_f  = frame.shape[:2]
        cx_frame  = w_f // 2
        cy_frame  = h_f // 2

        detector.submit(frame)
        det = detector.get_result()

        person_box  = det["person_box"]
        track_id    = det["track_id"]
        all_persons = det["all_persons"]
        faces       = det["faces"]

        pan_raw  = 0.0
        tilt_raw = 0.0
        status   = "NO BODY"

        target_box = select_face(faces, smooth_cx, smooth_cy)

        if target_box is not None:
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
            lock_err = math.hypot(error_x, error_y)
            if fire:
                if lock_err > LOCK_RELEASE_RADIUS:
                    fire = False
                    lock_count = 0
            else:
                lock_count = lock_count + 1 if lock_err <= LOCK_ON_RADIUS else 0
                if lock_count >= FIRE_DWELL_FRAMES:
                    fire = True
            locked = lock_err <= LOCK_ON_RADIUS

            if abs(error_x) < INPUT_DEADBAND:
                error_x = 0.0
            if abs(error_y) < INPUT_DEADBAND:
                error_y = 0.0

            pan_pid  = pid_pan.update(error_x  * INVERT_PAN,  dt, INTEGRAL_CLAMP)
            tilt_pid = pid_tilt.update(error_y * INVERT_TILT, dt, INTEGRAL_CLAMP)

            pan_raw  = apply_output_deadband(pan_pid,  OUTPUT_DEADBAND)
            tilt_raw = apply_output_deadband(tilt_pid, OUTPUT_DEADBAND)

            status = "TRACKING"

        else:
            no_body_count += 1
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

        smooth_pan  = CMD_SMOOTH * smooth_pan  + (1.0 - CMD_SMOOTH) * pan_raw
        smooth_tilt = CMD_SMOOTH * smooth_tilt + (1.0 - CMD_SMOOTH) * tilt_raw

        max_step = MAX_CMD_CHANGE_PER_SEC * dt
        slew_pan  += clamp(smooth_pan  - slew_pan,  -max_step, max_step)
        slew_tilt += clamp(smooth_tilt - slew_tilt, -max_step, max_step)

        pan_cmd  = STOP_PAN  + int(round(slew_pan))
        tilt_cmd = STOP_TILT + int(round(slew_tilt))

        draw_hud(frame, status, pan_cmd, tilt_cmd,
                 smooth_cx, smooth_cy,
                 person_box, all_persons, faces, track_id,
                 w_f, h_f, error_x, error_y, fire, locked)

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
