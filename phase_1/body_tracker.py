"""
Body-Tracking Pan-Tilt Turret  —  MediaPipe Edition
====================================================
Replaces face detection with MediaPipe Pose (33 landmarks, full body).
Servo smoothing values ported from the v1 smooth controller.

Tracking target : centroid of all 33 visible pose landmarks
Detector        : MediaPipe Pose (runs in background thread)
Servo control   : proportional gain + EMA smoothing (proven smooth values)

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
  pip install mediapipe opencv-python pyserial
"""

import argparse
import threading
import time

import cv2
import mediapipe as mp
import numpy as np
import serial

# ══════════════════════════════════════════════════════════════════
#  CONFIG  — tuned for smooth servo motion (ported from v1)
# ══════════════════════════════════════════════════════════════════

# --- Servo ---
STOP_PAN    = 90        # Continuous rotation neutral
STOP_TILT   = 90
MAX_SPEED   = 35        # Max offset from 90 (range 55..125)
INVERT_PAN  = -1        # Flip if turret moves wrong way
INVERT_TILT =  1

# --- Gain (proportional only — proven smooth from v1) ---
GAIN_PAN    = 0.10      # Speed per pixel of horizontal error
GAIN_TILT   = 0.09      # Speed per pixel of vertical error

# --- Dead zone ---
DEAD_ZONE   = 40        # Pixels — no correction inside this radius

# --- Smoothing (v1 values — smooth and stable) ---
SMOOTH      = 0.65      # Body centre EMA  (higher = smoother, laggier)
CMD_SMOOTH  = 0.70      # Servo command EMA (higher = less jitter)

# --- Serial ---
BAUD_RATE     = 9600
SEND_HZ       = 30
SEND_INTERVAL = 1.0 / SEND_HZ

# --- Camera ---
CAM_WIDTH  = 1280
CAM_HEIGHT = 720
CAM_FPS    = 30

# --- MediaPipe Pose ---
# Landmarks to use for centroid (all 33 by default)
# Set to a subset for upper-body-only if needed:
#   UPPER = [0,11,12,13,14,15,16,23,24]
USE_LANDMARKS = list(range(33))   # all 33
MIN_DETECTION_CONFIDENCE = 0.5
MIN_TRACKING_CONFIDENCE  = 0.5

# --- HUD colours (BGR) ---
COL_TRACKING  = (0, 220,  80)
COL_NO_BODY   = (0,  60, 220)
COL_RETICLE   = (220, 220,  0)
COL_LANDMARK  = (0, 200, 255)
COL_CENTROID  = (0, 255, 255)


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
    submit(frame)      → non-blocking hand-off
    get_result()       → (centroid_x, centroid_y, landmarks_px) or None
    """

    def __init__(self):
        self._lock      = threading.Lock()
        self._frame     = None
        self._new_frame = False
        self._result    = None          # (cx, cy, [(x,y), ...]) or None
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
            model_complexity=1,              # 0=fast, 1=balanced, 2=accurate
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

            lm = results.pose_landmarks.landmark

            # Collect all visible landmarks from USE_LANDMARKS
            pts = []
            for idx in USE_LANDMARKS:
                p = lm[idx]
                # visibility > 0.3 means the landmark is reasonably visible
                if p.visibility > 0.3:
                    pts.append((int(p.x * w), int(p.y * h)))

            if not pts:
                with self._lock:
                    self._result = None
                continue

            # Centroid of all visible landmarks
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
#  HUD
# ══════════════════════════════════════════════════════════════════
def draw_hud(frame, status, pan_cmd, tilt_cmd,
             smooth_cx, smooth_cy, landmarks, w_f, h_f):

    tracking = (status == "TRACKING")
    bar_col  = COL_TRACKING if tracking else COL_NO_BODY

    # Draw landmark dots
    if landmarks:
        for (x, y) in landmarks:
            cv2.circle(frame, (x, y), 3, COL_LANDMARK, -1, cv2.LINE_AA)

    # Draw centroid
    if smooth_cx is not None and tracking:
        cv2.circle(frame, (int(smooth_cx), int(smooth_cy)),
                   8, COL_CENTROID, -1, cv2.LINE_AA)
        cv2.circle(frame, (int(smooth_cx), int(smooth_cy)),
                   8, (0, 0, 0), 1, cv2.LINE_AA)

    # Semi-transparent top bar
    overlay = frame.copy()
    cv2.rectangle(overlay, (0, 0), (w_f, 40), (10, 10, 10), -1)
    cv2.addWeighted(overlay, 0.45, frame, 0.55, 0, frame)

    bar_text = (f"[{status}]   Pan {pan_cmd:3d}  Tilt {tilt_cmd:3d}")
    cv2.putText(frame, bar_text, (10, 27),
                cv2.FONT_HERSHEY_SIMPLEX, 0.58, bar_col, 2, cv2.LINE_AA)

    # Reticle (shrinks when locked)
    cx, cy = w_f // 2, h_f // 2
    arm    = 10 if tracking else 18
    cv2.line(frame, (cx-arm, cy), (cx+arm, cy), COL_RETICLE, 1, cv2.LINE_AA)
    cv2.line(frame, (cx, cy-arm), (cx, cy+arm), COL_RETICLE, 1, cv2.LINE_AA)
    cv2.circle(frame, (cx, cy), arm+8, COL_RETICLE, 1, cv2.LINE_AA)

    # Quit hint
    cv2.putText(frame, "Q = quit", (10, h_f - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.38, (100, 100, 100), 1, cv2.LINE_AA)


# ══════════════════════════════════════════════════════════════════
#  Main
# ══════════════════════════════════════════════════════════════════
def main():
    parser = argparse.ArgumentParser(description="DART body tracker — MediaPipe edition")
    parser.add_argument("--camera", type=int, default=1,     help="Camera index (1=USB)")
    parser.add_argument("--port",   type=str, default="COM3",help="Serial port")
    args = parser.parse_args()

    print(f"[INFO] Opening camera {args.camera} ...")
    cam      = CameraStream(src=args.camera)
    detector = PoseDetector()

    # Serial
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
        print(f"[INFO] Serial ready on {args.port}")
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

    print("[INFO] Running — Q to quit")

    while True:
        grabbed, frame = cam.read()
        if not grabbed or frame is None:
            time.sleep(0.01)
            continue

        now = time.monotonic()
        h_f, w_f   = frame.shape[:2]
        cx_frame   = w_f // 2
        cy_frame   = h_f // 2

        # ── Pose detection ────────────────────────────────────
        detector.submit(frame)
        result = detector.get_result()

        pan_int  = 0
        tilt_int = 0
        status   = "NO BODY"
        landmarks = []

        if result is not None:
            no_body_count = 0
            raw_cx, raw_cy, landmarks = result

            # EMA smoothing on body centroid (v1 proven values)
            if smooth_cx is None:
                smooth_cx, smooth_cy = float(raw_cx), float(raw_cy)
            else:
                smooth_cx = SMOOTH * smooth_cx + (1.0 - SMOOTH) * raw_cx
                smooth_cy = SMOOTH * smooth_cy + (1.0 - SMOOTH) * raw_cy

            error_x = smooth_cx - cx_frame
            error_y = smooth_cy - cy_frame

            # Proportional control with dead zone (v1 style)
            if abs(error_x) > DEAD_ZONE:
                sign_x   = 1.0 if error_x > 0 else -1.0
                eff_x    = abs(error_x) - DEAD_ZONE
                raw_pan  = clamp(sign_x * eff_x * GAIN_PAN * INVERT_PAN,
                                 -MAX_SPEED, MAX_SPEED)
                pan_int  = int(round(raw_pan))

            if abs(error_y) > DEAD_ZONE:
                sign_y   = 1.0 if error_y > 0 else -1.0
                eff_y    = abs(error_y) - DEAD_ZONE
                raw_tilt = clamp(sign_y * eff_y * GAIN_TILT * INVERT_TILT,
                                 -MAX_SPEED, MAX_SPEED)
                tilt_int = int(round(raw_tilt))

            status = "TRACKING"

        else:
            no_body_count += 1
            if no_body_count > 15:
                smooth_cx = None
                smooth_cy = None

        # ── Servo command smoothing (v1 CMD_SMOOTH) ───────────
        smooth_pan  = CMD_SMOOTH * smooth_pan  + (1.0 - CMD_SMOOTH) * pan_int
        smooth_tilt = CMD_SMOOTH * smooth_tilt + (1.0 - CMD_SMOOTH) * tilt_int

        pan_cmd  = STOP_PAN  + int(round(smooth_pan))
        tilt_cmd = STOP_TILT + int(round(smooth_tilt))

        # ── HUD ───────────────────────────────────────────────
        draw_hud(frame, status, pan_cmd, tilt_cmd,
                 smooth_cx, smooth_cy, landmarks, w_f, h_f)

        # ── Serial ────────────────────────────────────────────
        if ser and (now - last_send) >= SEND_INTERVAL:
            try:
                ser.write(build_command(pan_cmd, tilt_cmd))
            except serial.SerialException:
                pass
            last_send = now

        cv2.imshow("DART  [Body Tracker]", frame)
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