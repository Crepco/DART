"""
Body-Tracking Pan-Tilt Turret  —  Anti-Overshoot PID Edition
=============================================================
MediaPipe Pose (33 landmarks) + properly tuned PID for
continuous-rotation MG996R servos.

Key changes vs previous version
--------------------------------
1. Continuous PID   — no input dead zone resetting the controller.
                      D term now sees the full error curve and can
                      generate negative braking force as target
                      approaches centre.
2. Output dead zone — if |pid_output| < OUTPUT_DEADBAND, force to 0.
                      Stops servo buzzing without killing D term.
3. Instant brake    — if raw PID output == 0.0, skip EMA and set
                      smooth output to 0.0 immediately. Prevents
                      EMA coasting past centre.
4. Retuned gains    — KP lowered, KD raised for heavy braking profile.
5. Baud rate 115200 — lower serial latency.
                      !! Arduino sketch must also use 115200 !!

Hardware
--------
  Pin 9  = TILT (up/down)
  Pin 10 = PAN  (left/right)
  Continuous rotation MG996R servos

Usage
-----
  python body_tracker.py [--camera N] [--port PORT]

Install
-------
  pip install mediapipe==0.10.9 opencv-python pyserial numpy
"""

import argparse
import threading
import time

import cv2
import mediapipe as mp
import numpy as np
import serial

# ══════════════════════════════════════════════════════════════════
#  CONFIG
# ══════════════════════════════════════════════════════════════════

# --- Servo ---
STOP_PAN    = 90        # Continuous rotation neutral
STOP_TILT   = 90
MAX_SPEED   = 35        # Max offset from 90  (range 55..125)
INVERT_PAN  = -1        # Flip if turret pans wrong way
INVERT_TILT =  1        # Flip if turret tilts wrong way

# --- Output dead band ---
# PID output values inside this range are forced to 0.
# Stops servo buzzing on tiny corrections without killing the D term.
OUTPUT_DEADBAND = 3     # units match servo offset (same scale as MAX_SPEED)

# --- PID gains (heavy-braking profile) ---
# Tuning guide:
#   Still overshoots?     → raise KD  (try 0.12, 0.15)
#   Oscillates?           → lower KP  (try 0.03, 0.02)
#   Too slow to centre?   → raise KP  (try 0.05, 0.06)
#   Drifts when still?    → raise KI  (try 0.002, 0.005)
#   Buzzing at centre?    → raise OUTPUT_DEADBAND (try 4, 5)
PAN_KP,  PAN_KI,  PAN_KD  = 0.04, 0.001, 0.10
TILT_KP, TILT_KI, TILT_KD = 0.04, 0.001, 0.09

# --- Smoothing ---
SMOOTH     = 0.65       # Body centroid EMA  (higher = smoother position)
CMD_SMOOTH = 0.70       # Servo command EMA  (only applied when output != 0)

# --- Serial ---
BAUD_RATE     = 115200  # Must match Arduino Serial.begin(115200)
SEND_HZ       = 30
SEND_INTERVAL = 1.0 / SEND_HZ

# --- Camera ---
CAM_WIDTH  = 1280
CAM_HEIGHT = 720
CAM_FPS    = 30

# --- MediaPipe ---
USE_LANDMARKS            = list(range(33))  # all 33 body landmarks
FACE_LANDMARK_IDS        = [0,1,2,3,4,5,6,7,8,9,10]  # face region
MIN_DETECTION_CONFIDENCE = 0.5
MIN_TRACKING_CONFIDENCE  = 0.5

# --- HUD colours (BGR) ---
COL_TRACKING = (0, 220,  80)
COL_NO_BODY  = (0,  60, 220)
COL_RETICLE  = (220, 220,  0)
COL_LANDMARK = (0, 200, 255)
COL_CENTROID = (0, 255, 255)
COL_FACE_BOX = (0, 220,  80)


# ══════════════════════════════════════════════════════════════════
#  PID Controller
# ══════════════════════════════════════════════════════════════════
class PID:
    """
    Continuous PID with integral wind-up clamp.

    NOTE: No input dead zone here. The controller runs on every
    frame so the D term can see the full error slope and generate
    braking force as the target approaches centre.
    Dead-banding is applied to the OUTPUT instead.
    """

    def __init__(self, kp: float, ki: float, kd: float, limit: float):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.limit      = limit
        self.integral   = 0.0
        self.prev_error = 0.0

    def update(self, error: float, dt: float) -> float:
        dt = max(dt, 1e-6)

        # Integral with wind-up clamp
        self.integral = float(np.clip(
            self.integral + error * dt,
            -self.limit, self.limit
        ))

        # Derivative — sees full error slope including braking region
        derivative      = (error - self.prev_error) / dt
        self.prev_error = error

        raw = (self.kp * error
               + self.ki * self.integral
               + self.kd * derivative)

        return float(np.clip(raw, -self.limit, self.limit))

    def reset(self):
        """Call only when tracking is fully lost."""
        self.integral   = 0.0
        self.prev_error = 0.0


# ══════════════════════════════════════════════════════════════════
#  Camera stream (threaded)
# ══════════════════════════════════════════════════════════════════
class CameraStream:
    def __init__(self, src: int = 1):
        backend = cv2.CAP_DSHOW if hasattr(cv2, "CAP_DSHOW") else 0
        self.cap = cv2.VideoCapture(src, backend)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
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
#  MediaPipe Pose detector (threaded)
# ══════════════════════════════════════════════════════════════════
class PoseDetector:
    """
    Runs MediaPipe Pose in a background thread.
    submit(frame)  → non-blocking hand-off
    get_result()   → (centroid_x, centroid_y, [(x,y),...]) or None
    """

    def __init__(self):
        self._lock      = threading.Lock()
        self._frame     = None
        self._new_frame = False
        self._result    = None
        self._stopped   = False
        self._thread    = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def submit(self, frame):
        with self._lock:
            self._frame     = frame
            self._new_frame = True

    def get_result(self):
        with self._lock:
            return self._result

    def stop(self):
        self._stopped = True
        self._thread.join()

    def _run(self):
        pose = mp.solutions.pose.Pose(
            static_image_mode=False,
            model_complexity=1,
            smooth_landmarks=True,
            min_detection_confidence=MIN_DETECTION_CONFIDENCE,
            min_tracking_confidence=MIN_TRACKING_CONFIDENCE,
        )

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

            h, w = frame.shape[:2]
            rgb  = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            try:
                results = pose.process(rgb)
            except Exception:
                with self._lock:
                    self._result = None
                continue

            if not results.pose_landmarks:
                with self._lock:
                    self._result = None
                continue

            lm  = results.pose_landmarks.landmark
            pts = []
            for idx in USE_LANDMARKS:
                p = lm[idx]
                if p.visibility > 0.3:
                    pts.append((int(p.x * w), int(p.y * h)))

            if not pts:
                with self._lock:
                    self._result = None
                continue

            cx = int(np.mean([p[0] for p in pts]))
            cy = int(np.mean([p[1] for p in pts]))

            with self._lock:
                self._result = (cx, cy, pts)

        pose.close()


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
    """
    Force small PID outputs to exactly 0.0.
    This stops servo buzzing without removing the D term's
    ability to see the error slope.
    """
    return 0.0 if abs(value) < band else value


# ══════════════════════════════════════════════════════════════════
#  HUD
# ══════════════════════════════════════════════════════════════════
def draw_hud(frame, status, pan_cmd, tilt_cmd,
             smooth_cx, smooth_cy, landmarks,
             w_f, h_f, error_x, error_y):

    tracking = (status == "TRACKING")
    bar_col  = COL_TRACKING if tracking else COL_NO_BODY
    cx_f, cy_f = w_f // 2, h_f // 2

    # Body landmark dots
    for (x, y) in landmarks:
        cv2.circle(frame, (x, y), 3, COL_LANDMARK, -1, cv2.LINE_AA)

    # Green face box from MediaPipe face landmarks
    if landmarks:
        face_pts = [landmarks[i] for i in FACE_LANDMARK_IDS
                    if i < len(landmarks)]
        if face_pts:
            xs  = [p[0] for p in face_pts]
            ys  = [p[1] for p in face_pts]
            pad = 20
            x1  = max(0,   min(xs) - pad)
            y1  = max(0,   min(ys) - pad)
            x2  = min(w_f, max(xs) + pad)
            y2  = min(h_f, max(ys) + pad)
            cv2.rectangle(frame, (x1, y1), (x2, y2),
                          COL_FACE_BOX, 2, cv2.LINE_AA)
            cv2.putText(frame, "Face", (x1, y1 - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        COL_FACE_BOX, 1, cv2.LINE_AA)

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

    bar_text = (f"[{status}]   Pan {pan_cmd:3d}  Tilt {tilt_cmd:3d}"
                f"   Err ({int(error_x):+d}, {int(error_y):+d})")
    cv2.putText(frame, bar_text, (10, 27),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, bar_col, 2, cv2.LINE_AA)

    # Reticle
    arm = 10 if tracking else 18
    cv2.line(frame, (cx_f-arm, cy_f), (cx_f+arm, cy_f), COL_RETICLE, 1, cv2.LINE_AA)
    cv2.line(frame, (cx_f, cy_f-arm), (cx_f, cy_f+arm), COL_RETICLE, 1, cv2.LINE_AA)
    cv2.circle(frame, (cx_f, cy_f), arm + 8, COL_RETICLE, 1, cv2.LINE_AA)

    # Output deadband ring (visual reference)
    cv2.circle(frame, (cx_f, cy_f), OUTPUT_DEADBAND * 4,
               (50, 50, 50), 1, cv2.LINE_AA)

    cv2.putText(frame, "Q = quit", (10, h_f - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.38, (100, 100, 100), 1, cv2.LINE_AA)


# ══════════════════════════════════════════════════════════════════
#  Main
# ══════════════════════════════════════════════════════════════════
def main():
    parser = argparse.ArgumentParser(description="DART body tracker — anti-overshoot PID")
    parser.add_argument("--camera", type=int, default=1,        help="Camera index (1=USB)")
    parser.add_argument("--port",   type=str, default="COM3",   help="Serial port")
    args = parser.parse_args()

    print(f"[INFO] Opening camera {args.camera} ...")
    cam      = CameraStream(src=args.camera)
    detector = PoseDetector()

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
        print("[WARN] Preview-only mode.")

    # State
    smooth_cx     = None
    smooth_cy     = None
    no_body_count = 0
    smooth_pan    = 0.0
    smooth_tilt   = 0.0
    last_send     = 0.0
    last_time     = time.monotonic()
    error_x       = 0.0
    error_y       = 0.0

    print("[INFO] Running — Q to quit")

    while True:
        grabbed, frame = cam.read()
        if not grabbed or frame is None:
            time.sleep(0.01)
            continue

        now       = time.monotonic()
        dt        = max(now - last_time, 1e-6)
        last_time = now

        h_f, w_f = frame.shape[:2]
        cx_frame = w_f // 2
        cy_frame = h_f // 2

        # ── Pose detection ────────────────────────────────────
        detector.submit(frame)
        result    = detector.get_result()
        landmarks = []
        pan_raw   = 0.0
        tilt_raw  = 0.0
        status    = "NO BODY"

        if result is not None:
            no_body_count = 0
            raw_cx, raw_cy, landmarks = result

            # EMA smoothing on body centroid position
            if smooth_cx is None:
                smooth_cx = float(raw_cx)
                smooth_cy = float(raw_cy)
            else:
                smooth_cx = SMOOTH * smooth_cx + (1.0 - SMOOTH) * raw_cx
                smooth_cy = SMOOTH * smooth_cy + (1.0 - SMOOTH) * raw_cy

            error_x = smooth_cx - cx_frame
            error_y = smooth_cy - cy_frame

            # ── Continuous PID (no input dead zone) ──────────
            # D term runs on full error curve including near-centre
            # so it generates braking force as error approaches 0
            pan_pid  = pid_pan.update(error_x * INVERT_PAN,   dt)
            tilt_pid = pid_tilt.update(error_y * INVERT_TILT, dt)

            # ── Output dead band ─────────────────────────────
            # Kill tiny outputs that cause buzzing — after PID,
            # not before, so D term is never starved of data
            pan_raw  = apply_output_deadband(pan_pid,  OUTPUT_DEADBAND)
            tilt_raw = apply_output_deadband(tilt_pid, OUTPUT_DEADBAND)

            status = "TRACKING"

        else:
            no_body_count += 1
            error_x = error_y = 0.0
            if no_body_count > 15:
                smooth_cx = smooth_cy = None
                pid_pan.reset()
                pid_tilt.reset()

        # ── EMA with instant brake (Change 3) ─────────────────
        # If raw output is 0 → bypass EMA and brake immediately.
        # Prevents EMA from coasting the servo past centre.
        if pan_raw == 0.0:
            smooth_pan  = 0.0           # instant brake
        else:
            smooth_pan  = CMD_SMOOTH * smooth_pan  + (1.0 - CMD_SMOOTH) * pan_raw

        if tilt_raw == 0.0:
            smooth_tilt = 0.0           # instant brake
        else:
            smooth_tilt = CMD_SMOOTH * smooth_tilt + (1.0 - CMD_SMOOTH) * tilt_raw

        pan_cmd  = STOP_PAN  + int(round(smooth_pan))
        tilt_cmd = STOP_TILT + int(round(smooth_tilt))

        # ── HUD ───────────────────────────────────────────────
        draw_hud(frame, status, pan_cmd, tilt_cmd,
                 smooth_cx, smooth_cy, landmarks,
                 w_f, h_f, error_x, error_y)

        # ── Serial ────────────────────────────────────────────
        if ser and (now - last_send) >= SEND_INTERVAL:
            try:
                ser.write(build_command(pan_cmd, tilt_cmd))
            except serial.SerialException:
                pass
            last_send = now

        cv2.imshow("DART  [Body Tracker — Anti-Overshoot PID]", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    # ── Shutdown ──────────────────────────────────────────────
    send_stop(ser)
    if ser:
        ser.close()
    detector.stop()
    cam.stop()
    cv2.destroyAllWindows()
    print("[INFO] Shut down.")


if __name__ == "__main__":
    main()