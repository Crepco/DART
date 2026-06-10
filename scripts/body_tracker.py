import os
# Silences TensorFlow/MediaPipe startup logs for a clean terminal
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3' 

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

# --- Camera ---
CAM_INDEX  = 0
CAM_WIDTH  = 640
CAM_HEIGHT = 480
CAM_FPS    = 15

# --- Servo / Hardware ---
STOP_PAN   = 90
STOP_TILT  = 90
MAX_SPEED  = 35
INVERT_PAN  = -1
INVERT_TILT = 1

# --- PID & Movement ---
OUTPUT_DEADBAND = 3
PAN_KP,  PAN_KI,  PAN_KD  = 0.04, 0.001, 0.10
TILT_KP, TILT_KI, TILT_KD = 0.04, 0.001, 0.09

SMOOTH     = 0.65 # Centroid EMA
CMD_SMOOTH = 0.70 # Servo CMD EMA

# --- Communication ---
PORT      = "COM3"
BAUD_RATE = 115200
SEND_HZ   = 30
SEND_INTERVAL = 1.0 / SEND_HZ

# --- MediaPipe / HUD ---
FACE_IDS = list(range(11))
VISIBILITY_THRESHOLD = 0.5
COL_TRACKING = (0, 220, 80)
COL_NO_BODY  = (0, 60, 220)
COL_RETICLE  = (220, 220, 0)
COL_LANDMARK = (0, 200, 255)
COL_CENTROID = (0, 255, 255)
COL_FACE_BOX = (0, 220, 80)

# ══════════════════════════════════════════════════════════════════
#  CORE CLASSES
# ══════════════════════════════════════════════════════════════════

class PID:
    def __init__(self, kp, ki, kd, limit):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.limit = limit
        self.integral = 0.0
        self.prev_error = 0.0

    def update(self, error, dt):
        dt = max(dt, 1e-6)
        # Integral wind-up clamp
        self.integral = float(np.clip(self.integral + error * dt, -self.limit, self.limit))
        derivative = (error - self.prev_error) / dt
        self.prev_error = error
        raw = (self.kp * error + self.ki * self.integral + self.kd * derivative)
        return float(np.clip(raw, -self.limit, self.limit))

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0

class CameraStream:
    def __init__(self, src=0):
        # CAP_DSHOW prevents most Windows-related freezes
        self.cap = cv2.VideoCapture(src, cv2.CAP_DSHOW)
        if not self.cap.isOpened():
            raise RuntimeError(f"❌ Camera {src} failed.")
        
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAM_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_HEIGHT)
        self.cap.set(cv2.CAP_PROP_FPS, CAM_FPS)

        self.grabbed, self.frame = self.cap.read()
        self._lock = threading.Lock()
        self._stopped = False
        self._thread = threading.Thread(target=self._update, daemon=True)
        self._thread.start()

    def _update(self):
        while not self._stopped:
            g, f = self.cap.read()
            with self._lock:
                self.grabbed, self.frame = g, f
            time.sleep(0.005) # Crucial sleep to prevent loop locking

    def read(self):
        with self._lock:
            return self.grabbed, (self.frame.copy() if self.frame is not None else None)

    def stop(self):
        self._stopped = True
        self._thread.join()
        self.cap.release()

class PoseDetector:
    def __init__(self):
        self._lock = threading.Lock()
        self._frame = None
        self._new_frame = False
        self._result = None
        self._stopped = False
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def submit(self, frame):
        with self._lock:
            self._frame = frame
            self._new_frame = True

    def get_result(self):
        with self._lock:
            return self._result

    def stop(self):
        self._stopped = True
        self._thread.join()

    def _run(self):
        # model_complexity=0 is the fastest for real-time tracking
        pose = mp.solutions.pose.Pose(
            static_image_mode=False,
            model_complexity=0, 
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
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
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = pose.process(rgb)
            
            if results.pose_landmarks:
                lm = results.pose_landmarks.landmark
                all_pts = [(int(p.x * w), int(p.y * h), p.visibility) for p in lm]
                
                # Tracking centroid uses all visible landmarks
                track_pts = [(p[0], p[1]) for p in all_pts if p[2] > VISIBILITY_THRESHOLD]
                
                if track_pts:
                    cx = int(np.mean([p[0] for p in track_pts]))
                    cy = int(np.mean([p[1] for p in track_pts]))
                    with self._lock:
                        self._result = (cx, cy, all_pts)
                else:
                    with self._lock: self._result = None
            else:
                with self._lock: self._result = None
        pose.close()

# ══════════════════════════════════════════════════════════════════
#  HUD LOGIC
# ══════════════════════════════════════════════════════════════════

def draw_hud(frame, status, pan_cmd, tilt_cmd, smooth_cx, smooth_cy, landmarks, error_x, error_y):
    h_f, w_f = frame.shape[:2]
    cx_f, cy_f = w_f // 2, h_f // 2
    tracking = (status == "TRACKING")
    bar_col = COL_TRACKING if tracking else COL_NO_BODY

    # 1. Landmark Dots
    if landmarks:
        for x, y, vis in landmarks:
            if vis > VISIBILITY_THRESHOLD:
                cv2.circle(frame, (x, y), 2, COL_LANDMARK, -1)

    # 2. Face Box
    if landmarks:
        face_pts = [landmarks[i] for i in FACE_IDS if landmarks[i][2] > VISIBILITY_THRESHOLD]
        if face_pts:
            xs, ys = [p[0] for p in face_pts], [p[1] for p in face_pts]
            cv2.rectangle(frame, (min(xs)-20, min(ys)-20), (max(xs)+20, max(ys)+20), COL_FACE_BOX, 2)

    # 3. Target Line & Centroid
    if tracking and smooth_cx is not None:
        cv2.line(frame, (int(smooth_cx), int(smooth_cy)), (cx_f, cy_f), (100, 100, 100), 1)
        cv2.circle(frame, (int(smooth_cx), int(smooth_cy)), 8, COL_CENTROID, -1)

    # 4. Status Bar (Semi-Transparent)
    overlay = frame.copy()
    cv2.rectangle(overlay, (0, 0), (w_f, 40), (10, 10, 10), -1)
    cv2.addWeighted(overlay, 0.5, frame, 0.5, 0, frame)
    bar_text = f"[{status}]  Pan: {pan_cmd}  Tilt: {tilt_cmd}  Err: ({int(error_x)}, {int(error_y)})"
    cv2.putText(frame, bar_text, (10, 27), cv2.FONT_HERSHEY_SIMPLEX, 0.55, bar_col, 2)

    # 5. Crosshair
    arm = 12 if tracking else 20
    cv2.line(frame, (cx_f-arm, cy_f), (cx_f+arm, cy_f), COL_RETICLE, 1)
    cv2.line(frame, (cx_f, cy_f-arm), (cx_f, cy_f+arm), COL_RETICLE, 1)

# ══════════════════════════════════════════════════════════════════
#  MAIN LOOP
# ══════════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--camera", type=int, default=CAM_INDEX)
    parser.add_argument("--port", type=str, default=PORT)
    args = parser.parse_args()

    cam = CameraStream(src=args.camera)
    detector = PoseDetector()
    pid_pan = PID(PAN_KP, PAN_KI, PAN_KD, MAX_SPEED)
    pid_tilt = PID(TILT_KP, TILT_KI, TILT_KD, MAX_SPEED)

    ser = None
    try:
        ser = serial.Serial(args.port, BAUD_RATE, timeout=0.1)
        print(f"[INFO] Connected to {args.port}")
        time.sleep(2) # Wait for Arduino reset
    except:
        print("[WARN] Serial failed. Preview only.")

    smooth_cx = smooth_cy = None
    smooth_pan = smooth_tilt = 0.0
    last_send = last_time = time.monotonic()
    frame_count = 0

    while True:
        grabbed, frame = cam.read()
        if not grabbed or frame is None:
            time.sleep(0.01)
            continue

        frame_count += 1
        now = time.monotonic()
        dt, last_time = max(now - last_time, 1e-6), now
        h_f, w_f = frame.shape[:2]

        # Submit to MediaPipe every 2nd frame to keep main loop snappy
        if frame_count % 2 == 0:
            detector.submit(frame)

        result = detector.get_result()
        pan_raw = tilt_raw = 0.0
        error_x = error_y = 0.0
        status = "NO BODY"
        landmarks = None

        if result:
            raw_cx, raw_cy, landmarks = result
            status = "TRACKING"

            # 1. EMA Centroid
            if smooth_cx is None:
                smooth_cx, smooth_cy = float(raw_cx), float(raw_cy)
            else:
                smooth_cx = SMOOTH * smooth_cx + (1.0 - SMOOTH) * raw_cx
                smooth_cy = SMOOTH * smooth_cy + (1.0 - SMOOTH) * raw_cy

            error_x, error_y = smooth_cx - (w_f//2), smooth_cy - (h_f//2)

            # 2. Continuous PID
            pan_pid = pid_pan.update(error_x * INVERT_PAN, dt)
            tilt_pid = pid_tilt.update(error_y * INVERT_TILT, dt)

            # 3. Output Deadband
            pan_raw = 0.0 if abs(pan_pid) < OUTPUT_DEADBAND else pan_pid
            tilt_raw = 0.0 if abs(tilt_pid) < OUTPUT_DEADBAND else tilt_pid
        else:
            pid_pan.reset()
            pid_tilt.reset()

        # 4. Instant Brake + EMA Command
        smooth_pan = 0.0 if pan_raw == 0.0 else (CMD_SMOOTH * smooth_pan + (1.0 - CMD_SMOOTH) * pan_raw)
        smooth_tilt = 0.0 if tilt_raw == 0.0 else (CMD_SMOOTH * smooth_tilt + (1.0 - CMD_SMOOTH) * tilt_raw)

        pan_cmd = STOP_PAN + int(round(smooth_pan))
        tilt_cmd = STOP_TILT + int(round(smooth_tilt))

        # 5. Serial Send
        if ser and (now - last_send) >= SEND_INTERVAL:
            ser.write(f"P{pan_cmd:03d}T{tilt_cmd:03d}\n".encode("ascii"))
            last_send = now

        # 6. HUD Draw
        draw_hud(frame, status, pan_cmd, tilt_cmd, smooth_cx, smooth_cy, landmarks, error_x, error_y)
        
        cv2.imshow("DART System - High Precision", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'): break

    if ser: ser.close()
    detector.stop()
    cam.stop()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()