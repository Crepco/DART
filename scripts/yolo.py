"""
DART — Body-Tracking Pan-Tilt Turret  [YOLO Edition]
=====================================================
YOLOv8 person tracking  (yolov8n.pt)     → turret centroid + person box
YOLOv8 face detection   (yolov8n-face.pt) → face box overlay (HUD only)

All PID / EMA / dead-band / serial logic is identical to the
Anti-Overshoot PID edition — only the detector is replaced.

Tracking strategy
-----------------
  • .track() with ByteTrack locks onto the FIRST detected person (ID=1).
  • If that ID disappears, the tracker re-acquires the next available person.
  • Face model runs on every frame in the background thread (HUD only).
  • Turret is driven by the person box centre — not the face.

Hardware
--------
  Pin 9  = TILT  (up/down)
  Pin 10 = PAN   (left/right)
  Continuous-rotation MG996R servos

Usage
-----
  python body_tracker.py [--camera N] [--port PORT]

Install
-------
  pip install ultralytics opencv-python pyserial numpy

Models
------
  yolov8n.pt        → auto-downloaded by ultralytics on first run
  yolov8n-face.pt   → download from:
                      https://github.com/akanametov/yolov8-face/releases
                      Place in the same folder as this script.
                      If missing, face boxes are simply skipped.
"""

import argparse
import copy
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


# ══════════════════════════════════════════════════════════════════
#  CONFIG
# ══════════════════════════════════════════════════════════════════

# --- Camera ---
CAM_INDEX  = 0
CAM_WIDTH  = 640
CAM_HEIGHT = 480
CAM_FPS    = 15

# --- YOLO models ---
SCRIPT_DIR = pathlib.Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parent
PERSON_MODEL = str(PROJECT_ROOT / "models" / "yolov8n.pt")
FACE_MODEL   = str(PROJECT_ROOT / "models" / "yolov8n-face.pt")
LABELS_FILE  = str(PROJECT_ROOT / "labels.txt")
PERSON_CONF  = 0.50                 # min confidence for person detection
FACE_CONF    = 0.45                 # min confidence for face detection

# --- Servo ---
STOP_PAN    = 90
STOP_TILT   = 90
MAX_SPEED   = 35
INVERT_PAN  = -1
INVERT_TILT =  1

# --- Output dead band ---
OUTPUT_DEADBAND = 3

# --- PID gains ---
#
#   Tuning guide (MG996R continuous rotation, ~50-100ms mechanical lag):
#
#   KP — Proportional anchor
#     Keep low (~0.04). Too high → violent overshoot as turret
#     tries to centre too fast against the servo's mechanical lag.
#
#   KD — Anti-overshoot brake  ← most important knob for this hardware
#     ByteTrack eliminates jitter so KD is reliable now.
#     Turret overshoots when target stops? Raise KD in steps:
#     0.10 → 0.12 → 0.15. Acts as a brake as error approaches zero.
#
#   KI — Integral (keep very low, dangerous on physical hardware)
#     If a person stands just outside servo reach, integral winds up.
#     When they return, turret snaps aggressively. Keep at 0.001.
#     Integral accumulation is also hard-clamped by INTEGRAL_CLAMP.
#
#   OUTPUT_DEADBAND — Buzzing at centre? Raise to 4 or 5.
#
PAN_KP,  PAN_KI,  PAN_KD  = 0.04, 0.001, 0.12   # KD raised: 0.10→0.12
TILT_KP, TILT_KI, TILT_KD = 0.04, 0.001, 0.11   # KD raised: 0.09→0.11

# Integral wind-up hard clamp (tighter than MAX_SPEED)
# Limits how much integral can accumulate — prevents snap-on-return
INTEGRAL_CLAMP = 10.0   # units: same as servo offset

# --- Smoothing ---
SMOOTH     = 0.65   # body centroid EMA
CMD_SMOOTH = 0.70   # servo command EMA

# --- Slew rate limiter ---
# Physical acceleration limit in servo-offset units per SECOND.
# Scaled by dt each frame so behaviour is identical regardless of FPS.
# At 15 FPS (dt≈66ms): effective per-frame cap ≈ 75 * 0.066 = ~5 units
# At 30 FPS (dt≈33ms): effective per-frame cap ≈ 75 * 0.033 = ~2.5 units
#   Still jerking on sudden target jump? Lower to 50.0
#   Too sluggish to start moving?        Raise to 100.0
MAX_CMD_CHANGE_PER_SEC = 75.0

# --- Serial ---
PORT          = "COM3"
BAUD_RATE     = 115200
SEND_HZ       = 30
SEND_INTERVAL = 1.0 / SEND_HZ

# --- HUD colours (BGR) ---
COL_TRACKING  = (0, 220,  80)
COL_NO_BODY   = (0,  60, 220)
COL_RETICLE   = (220, 220,  0)
COL_PERSON    = (0, 200, 255)   # person bounding box
COL_CENTROID  = (0, 255, 255)   # tracked centroid dot
COL_FACE_BOX  = (0, 220,  80)   # face bounding box
COL_TRACK_ID  = (255, 180,  0)  # track-ID label


# ══════════════════════════════════════════════════════════════════
#  PID Controller
# ══════════════════════════════════════════════════════════════════
class PID:
    """Continuous PID — dead-band on output, not input."""

    def __init__(self, kp, ki, kd, limit):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.limit      = limit
        self.integral   = 0.0
        self.prev_error = 0.0

    def update(self, error: float, dt: float, integral_clamp: float = None) -> float:
        dt     = max(dt, 1e-6)
        i_lim  = integral_clamp if integral_clamp is not None else self.limit
        self.integral = float(np.clip(
            self.integral + error * dt, -i_lim, i_lim))
        derivative      = (error - self.prev_error) / dt
        self.prev_error = error
        raw = self.kp * error + self.ki * self.integral + self.kd * derivative
        return float(np.clip(raw, -self.limit, self.limit))

    def reset(self):
        self.integral   = 0.0
        self.prev_error = 0.0


# ══════════════════════════════════════════════════════════════════
#  Threaded Camera Stream
# ══════════════════════════════════════════════════════════════════
class CameraStream:
    def __init__(self, src: int = 0):
        self.cap = cv2.VideoCapture(src)
        if not self.cap.isOpened():
            raise RuntimeError(f"❌ Camera {src} not available. Try --camera 0 or 1")
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


# ══════════════════════════════════════════════════════════════════
#  Threaded YOLO Detector
# ══════════════════════════════════════════════════════════════════
class YOLODetector:
    """
    Runs YOLOv8 person tracking + face detection in a background thread.

    submit(frame)    → non-blocking frame hand-off
    get_result()     → {
        'person_box': (x1,y1,x2,y2) | None,   ← locked target box
        'track_id':   int | None,
        'all_persons': [(x1,y1,x2,y2,tid), ...],
        'faces':       [(x1,y1,x2,y2), ...],
    }
    """

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
        self._stopped   = False

        # Load models
        print(f"[INFO] Loading person model: {person_model_path}")
        self._person_model = YOLO(person_model_path)

        self._face_model = None
        if face_model_path and os.path.exists(face_model_path):
            print(f"[INFO] Loading face model: {face_model_path}")
            self._face_model = YOLO(face_model_path)
        else:
            print(f"[WARN] Face model not found ({face_model_path}). Face boxes disabled.")

        # Warm-up pass so first real frame isn't slow
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
        locked_id = None   # track ID we are following

        while not self._stopped:
            # Grab latest frame
            with self._lock:
                if not self._new_frame or self._frame is None:
                    frame = None
                else:
                    frame = self._frame.copy()
                    self._new_frame = False

            if frame is None:
                time.sleep(0.005)
                continue

            # ── Person tracking ───────────────────────────────
            try:
                p_results = self._person_model.track(
                    frame,
                    persist=True,
                    classes=[0],           # class 0 = person
                    conf=PERSON_CONF,
                    verbose=False,
                    tracker="bytetrack.yaml",
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

                # Lock onto first seen ID; re-acquire if lost
                current_ids = [p[4] for p in all_persons]
                if locked_id not in current_ids:
                    locked_id = current_ids[0]   # grab whatever appeared

                for (x1, y1, x2, y2, tid) in all_persons:
                    if tid == locked_id:
                        person_box = (x1, y1, x2, y2)
                        track_id   = tid
                        break

            else:
                locked_id = None   # lost — will re-acquire next detection

            # ── Face detection ────────────────────────────────
            faces = []
            if self._face_model is not None:
                try:
                    f_results = self._face_model(
                        frame, conf=FACE_CONF, verbose=False)
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


# ══════════════════════════════════════════════════════════════════
#  Serial helpers
# ══════════════════════════════════════════════════════════════════
def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def build_command(pan: int, tilt: int) -> bytes:
    p = clamp(pan,  STOP_PAN  - MAX_SPEED, STOP_PAN  + MAX_SPEED)
    t = clamp(tilt, STOP_TILT - MAX_SPEED, STOP_TILT + MAX_SPEED)
    return f"P{p:03d}T{t:03d}\n".encode("ascii")

def send_stop(ser):
    if ser:
        try:
            ser.write(build_command(STOP_PAN, STOP_TILT))
        except serial.SerialException:
            pass


# ══════════════════════════════════════════════════════════════════
#  Output dead band
# ══════════════════════════════════════════════════════════════════
def apply_output_deadband(value: float, band: float) -> float:
    return 0.0 if abs(value) < band else value


# ══════════════════════════════════════════════════════════════════
#  HUD
# ══════════════════════════════════════════════════════════════════
def draw_hud(frame, status, pan_cmd, tilt_cmd,
             smooth_cx, smooth_cy,
             person_box, all_persons, faces, track_id,
             w_f, h_f, error_x, error_y):

    tracking = (status == "TRACKING")
    bar_col  = COL_TRACKING if tracking else COL_NO_BODY
    cx_f, cy_f = w_f // 2, h_f // 2

    # All detected persons (dim boxes)
    for (x1, y1, x2, y2, tid) in all_persons:
        is_target = (tid == track_id)
        col       = COL_PERSON if is_target else (80, 80, 80)
        thickness = 2 if is_target else 1
        cv2.rectangle(frame, (x1, y1), (x2, y2), col, thickness, cv2.LINE_AA)
        cv2.putText(frame, f"ID{tid}", (x1, y1 - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, COL_TRACK_ID, 1, cv2.LINE_AA)

    # Face boxes (green)
    for (x1, y1, x2, y2) in faces:
        cv2.rectangle(frame, (x1, y1), (x2, y2), COL_FACE_BOX, 2, cv2.LINE_AA)
        cv2.putText(frame, "Face", (x1, y1 - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, COL_FACE_BOX, 1, cv2.LINE_AA)

    # Smoothed centroid + line to frame centre
    if smooth_cx is not None and tracking:
        cv2.line(frame,
                 (int(smooth_cx), int(smooth_cy)),
                 (cx_f, cy_f), (80, 80, 80), 1, cv2.LINE_AA)
        cv2.circle(frame, (int(smooth_cx), int(smooth_cy)),
                   8, COL_CENTROID, -1, cv2.LINE_AA)
        cv2.circle(frame, (int(smooth_cx), int(smooth_cy)),
                   8, (0, 0, 0), 1, cv2.LINE_AA)

    # Semi-transparent top bar
    overlay = frame.copy()
    cv2.rectangle(overlay, (0, 0), (w_f, 40), (10, 10, 10), -1)
    cv2.addWeighted(overlay, 0.45, frame, 0.55, 0, frame)

    tid_label = f"  ID={track_id}" if track_id is not None else ""
    bar_text  = (f"[{status}]{tid_label}   Pan {pan_cmd:3d}  Tilt {tilt_cmd:3d}"
                 f"   Err ({int(error_x):+d}, {int(error_y):+d})")
    cv2.putText(frame, bar_text, (10, 27),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, bar_col, 2, cv2.LINE_AA)

    # Crosshair reticle
    arm = 10 if tracking else 18
    cv2.line(frame, (cx_f - arm, cy_f), (cx_f + arm, cy_f), COL_RETICLE, 1, cv2.LINE_AA)
    cv2.line(frame, (cx_f, cy_f - arm), (cx_f, cy_f + arm), COL_RETICLE, 1, cv2.LINE_AA)
    cv2.circle(frame, (cx_f, cy_f), arm + 8, COL_RETICLE, 1, cv2.LINE_AA)

    # Dead band ring
    cv2.circle(frame, (cx_f, cy_f), OUTPUT_DEADBAND * 12,
               (50, 50, 50), 1, cv2.LINE_AA)

    cv2.putText(frame, "Q = quit", (10, h_f - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.38, (100, 100, 100), 1, cv2.LINE_AA)


# ══════════════════════════════════════════════════════════════════
#  Main
# ══════════════════════════════════════════════════════════════════
def main():
    parser = argparse.ArgumentParser(description="DART body tracker — YOLO Edition")
    parser.add_argument("--camera",       type=int, default=CAM_INDEX,  help="Camera index (default 0)")
    parser.add_argument("--port",         type=str, default=PORT,       help="Serial port (default COM3)")
    parser.add_argument("--person-model", type=str, default=PERSON_MODEL)
    parser.add_argument("--face-model",   type=str, default=FACE_MODEL)
    args = parser.parse_args()

    # ── Camera ────────────────────────────────────────────────────
    print(f"[INFO] Opening camera {args.camera} ...")
    cam = CameraStream(src=args.camera)
    grabbed, test_frame = cam.read()
    if not grabbed or test_frame is None:
        raise RuntimeError("❌ Camera opened but returned no frame.")
    print(f"[INFO] Camera OK — {test_frame.shape[1]}×{test_frame.shape[0]}")

    # ── YOLO detector ─────────────────────────────────────────────
    face_path = args.face_model if os.path.exists(args.face_model) else None
    detector  = YOLODetector(args.person_model, face_path)

    # ── PID controllers ───────────────────────────────────────────
    pid_pan  = PID(PAN_KP,  PAN_KI,  PAN_KD,  MAX_SPEED)
    pid_tilt = PID(TILT_KP, TILT_KI, TILT_KD, MAX_SPEED)

    # ── Serial ────────────────────────────────────────────────────
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

    # ── State ─────────────────────────────────────────────────────
    smooth_cx     = None
    smooth_cy     = None
    no_body_count = 0
    smooth_pan    = 0.0
    smooth_tilt   = 0.0
    slew_pan      = 0.0     # slew rate limiter state for pan
    slew_tilt     = 0.0     # slew rate limiter state for tilt
    last_send     = 0.0
    last_time     = time.monotonic()
    error_x       = 0.0
    error_y       = 0.0

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

        # ── YOLO detection (non-blocking) ─────────────────────
        detector.submit(frame)
        det = detector.get_result()

        person_box  = det["person_box"]
        track_id    = det["track_id"]
        all_persons = det["all_persons"]
        faces       = det["faces"]

        pan_raw  = 0.0
        tilt_raw = 0.0
        status   = "NO BODY"

        if person_box is not None:
            no_body_count = 0
            x1, y1, x2, y2 = person_box
            raw_cx = (x1 + x2) // 2
            raw_cy = (y1 + y2) // 2

            # EMA on centroid position
            if smooth_cx is None:
                smooth_cx = float(raw_cx)
                smooth_cy = float(raw_cy)
            else:
                smooth_cx = SMOOTH * smooth_cx + (1.0 - SMOOTH) * raw_cx
                smooth_cy = SMOOTH * smooth_cy + (1.0 - SMOOTH) * raw_cy

            error_x = smooth_cx - cx_frame
            error_y = smooth_cy - cy_frame

            # Continuous PID — tighter integral clamp prevents snap-on-return
            pan_pid  = pid_pan.update(error_x  * INVERT_PAN,  dt, INTEGRAL_CLAMP)
            tilt_pid = pid_tilt.update(error_y * INVERT_TILT, dt, INTEGRAL_CLAMP)

            # Output dead band
            pan_raw  = apply_output_deadband(pan_pid,  OUTPUT_DEADBAND)
            tilt_raw = apply_output_deadband(tilt_pid, OUTPUT_DEADBAND)

            status = "TRACKING"

        else:
            no_body_count += 1
            error_x = error_y = 0.0
            if no_body_count > 15:
                smooth_cx   = smooth_cy  = None
                smooth_pan  = smooth_tilt = 0.0   # reset EMA
                slew_pan    = slew_tilt   = 0.0   # reset slew — safe here, target is gone
                pid_pan.reset()
                pid_tilt.reset()

        # ── EMA smoothing ──────────────────────────────────────
        # No instant-brake zero here — letting the slew limiter
        # ramp down naturally prevents the dead-band chatter bug
        # (brake→zero→ramp-from-zero→chatter on every oscillation).
        # State is only fully zeroed when target is completely lost
        # (no_body_count > 15 branch below resets PID + slew).
        smooth_pan  = CMD_SMOOTH * smooth_pan  + (1.0 - CMD_SMOOTH) * pan_raw
        smooth_tilt = CMD_SMOOTH * smooth_tilt + (1.0 - CMD_SMOOTH) * tilt_raw

        # ── Slew rate limiter (dt-scaled) ─────────────────────
        # MAX_CMD_CHANGE_PER_SEC * dt gives a consistent physical
        # acceleration cap regardless of camera FPS or frame drops.
        max_step = MAX_CMD_CHANGE_PER_SEC * dt

        pan_delta  = clamp(smooth_pan  - slew_pan,  -max_step, max_step)
        slew_pan   = slew_pan  + pan_delta

        tilt_delta = clamp(smooth_tilt - slew_tilt, -max_step, max_step)
        slew_tilt  = slew_tilt + tilt_delta

        pan_cmd  = STOP_PAN  + int(round(slew_pan))
        tilt_cmd = STOP_TILT + int(round(slew_tilt))

        # ── HUD ───────────────────────────────────────────────
        draw_hud(frame, status, pan_cmd, tilt_cmd,
                 smooth_cx, smooth_cy,
                 person_box, all_persons, faces, track_id,
                 w_f, h_f, error_x, error_y)

        # ── Serial send (rate-limited) ─────────────────────────
        if ser and (now - last_send) >= SEND_INTERVAL:
            try:
                ser.write(build_command(pan_cmd, tilt_cmd))
            except serial.SerialException:
                pass
            last_send = now

        cv2.imshow("DART  [YOLO + Slew Rate Limiter]", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    # ── Shutdown ──────────────────────────────────────────────────
    send_stop(ser)
    if ser:
        ser.close()
    detector.stop()
    cam.stop()
    cv2.destroyAllWindows()
    print("[INFO] Shut down.")


if __name__ == "__main__":
    main()