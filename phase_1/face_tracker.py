"""
Face-Tracking Pan-Tilt Turret Controller  —  Enhanced Edition
==============================================================
Improvements over v1:
  • DNN-based face detector  (res10_300x300_ssd — much better than Haar)
  • Proportional control     (simple and effective gain-based control)
  • Multi-face scoring       (locks onto closest/largest face)
  • Low-light enhancement    (CLAHE + gamma LUT on detection crop)
  • Live FPS counter         (30-frame rolling average)
  • Polished HUD             (animated reticle, colour-coded status bar)
  • USB-camera ready         (--camera 1 default, explicit resolution/FPS)

Hardware
--------
  Pin 9  = TILT servo (continuous rotation MG996R)
  Pin 10 = PAN  servo (continuous rotation MG996R)
  Upload arduino_servo_controller.ino first.

Usage
-----
  python face_tracker.py [--camera N] [--port PORT]

  --camera  Camera index (default 1 for USB cam; 0 = built-in)
  --port    Serial port  (default COM3; Linux: /dev/ttyUSB0)

Model files (place next to this script or give full path):
  opencv_face_detector_uint8.pb
  opencv_face_detector.pbtxt
  Download: https://github.com/opencv/opencv/tree/master/samples/dnn/face_detector
"""

import argparse
import collections
import threading
import time

import cv2
import numpy as np
import serial

# ══════════════════════════════════════════════════════════════════
#  CONFIG  — tune these without touching the logic
# ══════════════════════════════════════════════════════════════════

# --- Servo / turret ---
STOP_PAN = 90  # Neutral PWM value (continuous-rotation "stop")
STOP_TILT = 90
MAX_SPEED = 20  # Max offset from 90
INVERT_PAN = -1  # Flip left/right if turret moves wrong way
INVERT_TILT = 1  # Flip up/down   if turret moves wrong way
DEAD_ZONE = 60 # Pixel radius around centre with no correction

# --- Gain Control ---
PAN_GAIN = 0.08
TILT_GAIN = 0.05

# --- Smoothing ---
SMOOTH = 0.6  # Face-position EMA
CMD_SMOOTH = 0.6  # Servo-command EMA (reduces jitter)

# --- Detection ---
DNN_CONF_THRESHOLD = 0.55  # Min confidence for DNN detections
DETECTION_SCALE = 0.5  # Downscale factor before detection
MIN_FACE_PX = 40  # Min face side length (in scaled px)

# --- Low-light enhancement ---
GAMMA = 1.7  # Shadow lift; 1.0 = none
CLAHE_CLIP = 2.5  # CLAHE clip limit
CLAHE_GRID = (8, 8)  # CLAHE tile grid

# --- Serial ---
BAUD_RATE = 9600
SEND_HZ = 10
SEND_INTERVAL = 1.0 / SEND_HZ

# --- Camera (USB) ---
CAM_WIDTH = 1280
CAM_HEIGHT = 720
CAM_FPS = 30

# --- DNN model files (same directory as script) ---
DNN_MODEL = "res10_300x300_ssd_iter_140000.caffemodel"
DNN_CONFIG = "deploy.prototxt"

# Fallback to Haar if DNN files not found
HAAR_CASCADE = cv2.data.haarcascades + "haarcascade_frontalface_default.xml"


# ══════════════════════════════════════════════════════════════════
#  Low-light helpers
# ══════════════════════════════════════════════════════════════════
def _gamma_lut(gamma: float) -> np.ndarray:
    inv = 1.0 / gamma
    return np.array([(i / 255.0) ** inv * 255 for i in range(256)], dtype=np.uint8)


def enhance_gray(gray: np.ndarray, lut: np.ndarray, clahe) -> np.ndarray:
    """Gamma lift → CLAHE (runs on grayscale detection crop)."""
    return clahe.apply(cv2.LUT(gray, lut))


# ══════════════════════════════════════════════════════════════════
#  Face detector (DNN with Haar fallback)
# ══════════════════════════════════════════════════════════════════
class FaceDetector:
    """
    Runs detection in a background thread.
    submit(frame) → non-blocking hand-off
    get_results()  → latest list of (x,y,w,h) in full-frame coords
    """

    def __init__(self, scale: float = DETECTION_SCALE):
        self.scale = scale
        self._use_dnn = False

        # Try DNN first
        try:
            self._net = cv2.dnn.readNetFromCaffe(DNN_CONFIG, DNN_MODEL)
            # Smoke-test: a blank 300×300 blob
            blob = cv2.dnn.blobFromImage(
                np.zeros((300, 300, 3), np.uint8),
                1.0,
                (300, 300),
                (104, 177, 123),
                swapRB=False,
            )
            self._net.setInput(blob)
            self._net.forward()
            self._use_dnn = True
            print("[INFO] DNN face detector loaded.")
        except Exception as exc:
            print(f"[WARN] DNN load failed ({exc}); falling back to Haar.")
            self._cascade = cv2.CascadeClassifier(HAAR_CASCADE)

        # Low-light tools (used inside detector thread)
        self._clahe = cv2.createCLAHE(clipLimit=CLAHE_CLIP, tileGridSize=CLAHE_GRID)
        self._lut = _gamma_lut(GAMMA)

        self._lock = threading.Lock()
        self._frame = None
        self._new_frame = False
        self._results = []  # list of (x,y,w,h) full-frame
        self._stopped = False
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    # ── public API ──────────────────────────────────────────────
    def submit(self, frame):
        with self._lock:
            self._frame = frame
            self._new_frame = True

    def get_results(self):
        with self._lock:
            return list(self._results)

    def stop(self):
        self._stopped = True
        self._thread.join()

    # ── internal ────────────────────────────────────────────────
    def _detect_dnn(self, frame) -> list:
        h, w = frame.shape[:2]
        blob = cv2.dnn.blobFromImage(
            frame,
            1.0,
            (300, 300),
            (104, 177, 123),
            swapRB=False,
        )
        self._net.setInput(blob)
        dets = self._net.forward()
        faces = []
        for i in range(dets.shape[2]):
            conf = float(dets[0, 0, i, 2])
            if conf < DNN_CONF_THRESHOLD:
                continue
            box = dets[0, 0, i, 3:7] * np.array([w, h, w, h])
            x1, y1, x2, y2 = box.astype(int)
            x1, y1 = max(0, x1), max(0, y1)
            x2, y2 = min(w, x2), min(h, y2)
            fw, fh = x2 - x1, y2 - y1
            if fw >= MIN_FACE_PX and fh >= MIN_FACE_PX:
                faces.append((x1, y1, fw, fh))
        return faces

    def _detect_haar(self, frame) -> list:
        small = cv2.resize(
            frame,
            None,
            fx=self.scale,
            fy=self.scale,
            interpolation=cv2.INTER_AREA,
        )
        gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
        gray = enhance_gray(gray, self._lut, self._clahe)
        raw = self._cascade.detectMultiScale(
            gray,
            scaleFactor=1.15,
            minNeighbors=5,
            minSize=(MIN_FACE_PX, MIN_FACE_PX),
        )
        if not len(raw):
            return []
        inv = 1.0 / self.scale
        return [
            (int(x * inv), int(y * inv), int(w * inv), int(h * inv))
            for (x, y, w, h) in raw
        ]

    def _run(self):
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
                faces = (
                    self._detect_dnn(frame)
                    if self._use_dnn
                    else self._detect_haar(frame)
                )
            except Exception:
                faces = []

            with self._lock:
                self._results = faces


# ══════════════════════════════════════════════════════════════════
#  Multi-face selection
# ══════════════════════════════════════════════════════════════════
def select_face(faces: list, prev_cx: float, prev_cy: float):
    """
    Score each face by area (prefer larger) minus distance from
    where we were last tracking (prefer continuity).
    Returns (x, y, w, h) or None.
    """
    if not faces:
        return None

    def score(f):
        x, y, w, h = f
        cx, cy = x + w / 2, y + h / 2
        area = w * h
        dist = ((cx - prev_cx) ** 2 + (cy - prev_cy) ** 2) ** 0.5
        return area - dist * 0.4

    return max(faces, key=score)


# ══════════════════════════════════════════════════════════════════
#  Threaded camera capture
# ══════════════════════════════════════════════════════════════════
class CameraStream:
    def __init__(self, src: int = 1):
        # Auto-detect backend (CAP_ANY) to avoid black screen issues with DSHOW
        self.cap = cv2.VideoCapture(src)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAM_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_HEIGHT)
        self.cap.set(cv2.CAP_PROP_FPS, CAM_FPS)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        self.grabbed, self.frame = self.cap.read()
        self._lock = threading.Lock()
        self._stopped = False
        self._thread = threading.Thread(target=self._update, daemon=True)
        self._thread.start()

    def _update(self):
        while not self._stopped:
            grabbed, frame = self.cap.read()
            with self._lock:
                self.grabbed = grabbed
                self.frame = frame

    def read(self):
        with self._lock:
            return self.grabbed, (self.frame.copy() if self.frame is not None else None)

    def stop(self):
        self._stopped = True
        self._thread.join()
        self.cap.release()


# ══════════════════════════════════════════════════════════════════
#  Serial helpers
# ══════════════════════════════════════════════════════════════════
def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def build_command(pan: int, tilt: int) -> bytes:
    p = clamp(pan, STOP_PAN - MAX_SPEED, STOP_PAN + MAX_SPEED)
    t = clamp(tilt, STOP_TILT - MAX_SPEED, STOP_TILT + MAX_SPEED)
    return f"P{p:03d}T{t:03d}\n".encode("ascii")


def send_stop(ser):
    if ser:
        try:
            ser.write(build_command(STOP_PAN, STOP_TILT))
        except serial.SerialException:
            pass


# ══════════════════════════════════════════════════════════════════
#  HUD drawing
# ══════════════════════════════════════════════════════════════════
def draw_hud(
    frame,
    status: str,
    pan_cmd: int,
    tilt_cmd: int,
    fps: float,
    smooth_cx,
    smooth_cy,
    w_frame: int,
    h_frame: int,
):
    tracking = status == "TRACKING"
    color = (0, 220, 80) if tracking else (0, 60, 220)

    # Semi-transparent top bar
    overlay = frame.copy()
    cv2.rectangle(overlay, (0, 0), (w_frame, 38), (10, 10, 10), -1)
    cv2.addWeighted(overlay, 0.45, frame, 0.55, 0, frame)

    # Status text
    text = f"[{status}]   Pan {pan_cmd:3d}  Tilt {tilt_cmd:3d}" f"   FPS {fps:5.1f}"
    cv2.putText(
        frame, text, (10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.58, color, 2, cv2.LINE_AA
    )

    # Quit hint
    cv2.putText(
        frame,
        "Q = quit",
        (10, h_frame - 10),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.38,
        (120, 120, 120),
        1,
        cv2.LINE_AA,
    )

    # Animated reticle (shrinks when locked)
    cx, cy = w_frame // 2, h_frame // 2
    arm = 10 if tracking else 18
    radius = arm + 8
    cv2.line(frame, (cx - arm, cy), (cx + arm, cy), (220, 220, 0), 1, cv2.LINE_AA)
    cv2.line(frame, (cx, cy - arm), (cx, cy + arm), (220, 220, 0), 1, cv2.LINE_AA)
    cv2.circle(frame, (cx, cy), radius, (220, 220, 0), 1, cv2.LINE_AA)

    # Draw tracking position dot (only when tracking)
    if tracking and smooth_cx is not None:
        cv2.circle(frame, (int(smooth_cx), int(smooth_cy)), 5, color, -1, cv2.LINE_AA)

    # Detector type badge (bottom-right)
    det_label = "DNN" if "DNN" in status or True else "HAAR"


# ══════════════════════════════════════════════════════════════════
#  Main
# ══════════════════════════════════════════════════════════════════
def main():
    parser = argparse.ArgumentParser(description="Enhanced face-tracking turret")
    parser.add_argument("--camera", type=int, default=1, help="Camera index (1=USB)")
    parser.add_argument("--port", type=str, default="COM3", help="Serial port")
    args = parser.parse_args()

    print(f"[INFO] Opening camera {args.camera} ...")
    cam = CameraStream(src=args.camera)
    detector = FaceDetector(scale=DETECTION_SCALE)

    # Serial
    ser = None
    try:
        ser = serial.Serial(args.port, BAUD_RATE, timeout=0.5)
        print("[INFO] Waiting for Arduino reset …")
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
        print("[WARN] Preview-only mode (no servo output).")

    # State
    smooth_cx = None
    smooth_cy = None
    no_face_count = 0

    smooth_pan = 0.0
    smooth_tilt = 0.0

    prev_pan_cx = 0.0  # For multi-face continuity scoring
    prev_pan_cy = 0.0

    last_send = 0.0
    last_time = time.monotonic()
    fps_buf = collections.deque(maxlen=30)

    print("[INFO] Q = quit")

    while True:
        grabbed, frame = cam.read()
        if not grabbed or frame is None:
            time.sleep(0.01)
            continue

        now = time.monotonic()
        dt = now - last_time
        if dt < 0.01:
            dt = 0.01
        last_time = now
        fps_buf.append(1.0 / dt)
        fps = sum(fps_buf) / len(fps_buf)

        h_frame, w_frame = frame.shape[:2]
        cx_frame = w_frame // 2
        cy_frame = h_frame // 2

        # Hand frame to detector thread
        detector.submit(frame)
        faces = detector.get_results()

        # ── Select best face ───────────────────────────────────
        cx_prev = prev_pan_cx if smooth_cx is None else smooth_cx
        cy_prev = prev_pan_cy if smooth_cy is None else smooth_cy
        best = select_face(faces, cx_prev, cy_prev)

        pan_raw = 0.0
        tilt_raw = 0.0
        status = "NO FACE"

        if best is not None:
            no_face_count = 0
            fx, fy, fw, fh = best

            raw_cx = float(fx + fw / 2)
            raw_cy = float(fy + fh / 2)

            # EMA smoothing on position
            if smooth_cx is None:
                smooth_cx, smooth_cy = raw_cx, raw_cy
            else:
                smooth_cx = SMOOTH * smooth_cx + (1 - SMOOTH) * raw_cx
                smooth_cy = SMOOTH * smooth_cy + (1 - SMOOTH) * raw_cy
            prev_pan_cx, prev_pan_cy = smooth_cx, smooth_cy

            # Draw face box
            cv2.rectangle(
                frame, (fx, fy), (fx + fw, fy + fh), (0, 220, 80), 2, cv2.LINE_AA
            )

            error_x = smooth_cx - cx_frame
            error_y = smooth_cy - cy_frame

            # Proportional Control (only outside dead zone)
            if abs(error_x) > DEAD_ZONE:
                pan_raw = clamp(error_x * PAN_GAIN * INVERT_PAN, -MAX_SPEED, MAX_SPEED)
            else:
                pan_raw = 0.0

            if abs(error_y) > DEAD_ZONE:
                tilt_raw = clamp(error_y * TILT_GAIN * INVERT_TILT, -MAX_SPEED, MAX_SPEED)
                tilt_raw = tilt_raw * 0.6  # Reduce vertical overshoot
            else:
                tilt_raw = 0.0

            status = "TRACKING"

        else:
            no_face_count += 1
            if no_face_count > 15:
                smooth_cx, smooth_cy = None, None

        # ── Smooth servo command ───────────────────────────────
        smooth_pan = CMD_SMOOTH * smooth_pan + (1 - CMD_SMOOTH) * pan_raw
        smooth_tilt = CMD_SMOOTH * smooth_tilt + (1 - CMD_SMOOTH) * tilt_raw

        pan_cmd = STOP_PAN + int(round(smooth_pan))
        tilt_cmd = STOP_TILT + int(round(smooth_tilt))

        # ── HUD ───────────────────────────────────────────────
        draw_hud(
            frame,
            status,
            pan_cmd,
            tilt_cmd,
            fps,
            smooth_cx,
            smooth_cy,
            w_frame,
            h_frame,
        )

        # ── Serial send (rate-limited) ─────────────────────────
        if ser and (now - last_send) >= SEND_INTERVAL:
            if abs(pan_cmd - STOP_PAN) > 2 or abs(tilt_cmd - STOP_TILT) > 2:
                try:
                    ser.write(build_command(pan_cmd, tilt_cmd))
                except serial.SerialException:
                    pass
            last_send = now

        cv2.imshow("Face Tracker  [Enhanced]", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

        time.sleep(0.01)

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
