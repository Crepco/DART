"""
Face-Tracking Pan-Tilt Turret  —  Recognition Edition
======================================================
Detector  : Caffe DNN  (res10_300x300_ssd_iter_140000.caffemodel + deploy.prototxt)
Recogniser: face_recognition (dlib)
Enrolment : offline — run enrol.py first to populate known_faces/

What it does
------------
  • Tracks ALL faces with pan/tilt servos
  • Labels each face as Known (green) or Unknown (orange)
  • Sends extended serial command when a known face is centred & confirmed:
      Normal   → P090T085\n
      Known    → P090T085KAlice\n
  • HUD shows name + confidence % on every face

Folder layout
-------------
  phase_1/
    deploy.prototxt
    res10_300x300_ssd_iter_140000.caffemodel
    known_faces/
      Alice/
        0000.jpg  0001.jpg  ...
      Bob/
        0000.jpg  ...
    face_tracker_recognition.py   <- this file
    enrol.py

Usage
-----
  python face_tracker_recognition.py [--camera N] [--port PORT]
                                     [--faces PATH] [--tolerance 0.5]

Install
-------
  pip install -r requirements.txt
"""

import argparse
import collections
import os
import threading
import time

import cv2
import face_recognition
import numpy as np
import serial

# ══════════════════════════════════════════════════════════════════
#  CONFIG
# ══════════════════════════════════════════════════════════════════

# --- Servo ---
STOP_PAN    = 90
STOP_TILT   = 90
MAX_SPEED   = 35
INVERT_PAN  = -1
INVERT_TILT =  1
DEAD_ZONE   = 40

# --- Gain Control ---
PAN_GAIN  = 0.10
TILT_GAIN = 0.09

# --- Smoothing ---
SMOOTH     = 0.0
CMD_SMOOTH = 0.70

# --- Detection (Caffe DNN) ---
DNN_MODEL          = "res10_300x300_ssd_iter_140000.caffemodel"
DNN_CONFIG         = "deploy.prototxt"
DNN_CONF_THRESHOLD = 0.55
DETECTION_SCALE    = 0.5
MIN_FACE_PX        = 40
HAAR_CASCADE       = cv2.data.haarcascades + "haarcascade_frontalface_default.xml"

# --- Recognition ---
KNOWN_FACES_DIR      = "known_faces"
RECOG_TOLERANCE      = 0.50
RECOG_SCALE          = 0.25
RECOG_INTERVAL       = 6
LOCKED_FRAMES_NEEDED = 4

# --- Low-light ---
GAMMA      = 1.7
CLAHE_CLIP = 2.5
CLAHE_GRID = (8, 8)

# --- Serial ---
BAUD_RATE     = 9600
SEND_HZ       = 30
SEND_INTERVAL = 1.0 / SEND_HZ

# --- Camera ---
CAM_WIDTH  = 1280
CAM_HEIGHT = 720
CAM_FPS    = 30

# --- HUD colours (BGR) ---
COL_KNOWN   = (0, 220,  80)
COL_UNKNOWN = (0, 140, 255)
COL_LOCKED  = (255, 220,  0)
COL_RETICLE = (220, 220,  0)


# ══════════════════════════════════════════════════════════════════
#  Enrolment
# ══════════════════════════════════════════════════════════════════
def load_known_faces(folder: str):
    known_encodings, known_names = [], []

    if not os.path.isdir(folder):
        print(f"[WARN] Known-faces folder not found: {folder!r}")
        print("[WARN] Run enrol.py first.  Recognition disabled.")
        return known_encodings, known_names

    exts = {".jpg", ".jpeg", ".png", ".bmp"}
    total = 0

    for name in sorted(os.listdir(folder)):
        person_dir = os.path.join(folder, name)
        if not os.path.isdir(person_dir):
            continue
        count = 0
        for fname in sorted(os.listdir(person_dir)):
            if os.path.splitext(fname)[1].lower() not in exts:
                continue
            path = os.path.join(person_dir, fname)
            img  = face_recognition.load_image_file(path)
            encs = face_recognition.face_encodings(img)
            if not encs:
                print(f"  [SKIP] No face in {path}")
                continue
            known_encodings.append(encs[0])
            known_names.append(name)
            count += 1
        if count:
            print(f"  [ENROL] {name}: {count} photo(s)")
            total += count

    print(f"[INFO] Enrolled {len(set(known_names))} person(s), {total} encoding(s).")
    return known_encodings, known_names


# ══════════════════════════════════════════════════════════════════
#  PID
# ══════════════════════════════════════════════════════════════════
class PID:
    def __init__(self, kp, ki, kd, limit):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.limit    = limit
        self.integral = 0.0
        self.prev_err = 0.0

    def update(self, error, dt):
        dt = max(dt, 1e-6)
        self.integral = float(np.clip(
            self.integral + error * dt, -self.limit, self.limit))
        d = (error - self.prev_err) / dt
        self.prev_err = error
        return float(np.clip(
            self.kp * error + self.ki * self.integral + self.kd * d,
            -self.limit, self.limit))

    def reset(self):
        self.integral = 0.0
        self.prev_err = 0.0


# ══════════════════════════════════════════════════════════════════
#  Kalman filter
# ══════════════════════════════════════════════════════════════════
def make_kalman():
    kf = cv2.KalmanFilter(4, 2)
    kf.measurementMatrix   = np.array([[1,0,0,0],[0,1,0,0]], np.float32)
    kf.transitionMatrix    = np.array(
        [[1,0,1,0],[0,1,0,1],[0,0,1,0],[0,0,0,1]], np.float32)
    kf.processNoiseCov     = np.eye(4, dtype=np.float32) * 0.03
    kf.measurementNoiseCov = np.eye(2, dtype=np.float32) * 0.5
    kf.errorCovPost        = np.eye(4, dtype=np.float32)
    return kf


# ══════════════════════════════════════════════════════════════════
#  Low-light helpers
# ══════════════════════════════════════════════════════════════════
def _gamma_lut(g):
    return np.array([(i/255.0)**(1.0/g)*255 for i in range(256)], dtype=np.uint8)

def enhance_gray(gray, lut, clahe):
    return clahe.apply(cv2.LUT(gray, lut))


# ══════════════════════════════════════════════════════════════════
#  Face Detector  (Caffe DNN + Haar fallback)
# ══════════════════════════════════════════════════════════════════
class FaceDetector:
    def __init__(self, scale=DETECTION_SCALE):
        self.scale    = scale
        self._use_dnn = False

        try:
            self._net = cv2.dnn.readNetFromCaffe(DNN_CONFIG, DNN_MODEL)
            blob = cv2.dnn.blobFromImage(
                np.zeros((300,300,3), np.uint8), 1.0, (300,300),
                (104,177,123), swapRB=False)
            self._net.setInput(blob)
            self._net.forward()
            self._use_dnn = True
            print("[INFO] Caffe DNN face detector loaded.")
        except Exception as exc:
            print(f"[WARN] DNN load failed ({exc}); using Haar fallback.")
            self._cascade = cv2.CascadeClassifier(HAAR_CASCADE)

        self._clahe = cv2.createCLAHE(clipLimit=CLAHE_CLIP, tileGridSize=CLAHE_GRID)
        self._lut   = _gamma_lut(GAMMA)

        self._lock      = threading.Lock()
        self._frame     = None
        self._new_frame = False
        self._results   = []
        self._stopped   = False
        self._thread    = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def submit(self, frame):
        with self._lock:
            self._frame     = frame
            self._new_frame = True

    def get_results(self):
        with self._lock:
            return list(self._results)

    def stop(self):
        self._stopped = True
        self._thread.join()

    def _detect_dnn(self, frame):
        h, w = frame.shape[:2]
        blob = cv2.dnn.blobFromImage(
            frame, 1.0, (300,300), (104,177,123), swapRB=False)
        self._net.setInput(blob)
        dets  = self._net.forward()
        faces = []
        for i in range(dets.shape[2]):
            conf = float(dets[0,0,i,2])
            if conf < DNN_CONF_THRESHOLD:
                continue
            box = dets[0,0,i,3:7] * np.array([w,h,w,h])
            x1,y1,x2,y2 = box.astype(int)
            x1,y1 = max(0,x1), max(0,y1)
            x2,y2 = min(w,x2), min(h,y2)
            fw,fh = x2-x1, y2-y1
            if fw >= MIN_FACE_PX and fh >= MIN_FACE_PX:
                faces.append((x1,y1,fw,fh))
        return faces

    def _detect_haar(self, frame):
        small = cv2.resize(frame, None, fx=self.scale, fy=self.scale,
                           interpolation=cv2.INTER_AREA)
        gray  = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
        gray  = enhance_gray(gray, self._lut, self._clahe)
        raw   = self._cascade.detectMultiScale(
            gray, 1.15, 5, minSize=(MIN_FACE_PX, MIN_FACE_PX))
        if not len(raw):
            return []
        inv = 1.0 / self.scale
        return [(int(x*inv),int(y*inv),int(w*inv),int(h*inv)) for x,y,w,h in raw]

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
                faces = self._detect_dnn(frame) if self._use_dnn \
                        else self._detect_haar(frame)
            except Exception:
                faces = []
            with self._lock:
                self._results = faces


# ══════════════════════════════════════════════════════════════════
#  Recognition Worker
# ══════════════════════════════════════════════════════════════════
class RecognitionWorker:
    def __init__(self, known_encodings, known_names, tolerance, scale):
        self._encs      = known_encodings
        self._names     = known_names
        self._tolerance = tolerance
        self._scale     = scale

        self._lock      = threading.Lock()
        self._pending   = None
        self._has_work  = False
        self._labels    = []
        self._stopped   = False
        self._thread    = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def submit(self, frame, boxes):
        with self._lock:
            self._pending  = (frame, list(boxes))
            self._has_work = True

    def get_labels(self):
        with self._lock:
            return list(self._labels)

    def stop(self):
        self._stopped = True
        self._thread.join()

    def _run(self):
        while not self._stopped:
            with self._lock:
                if not self._has_work or self._pending is None:
                    work = None
                else:
                    work = self._pending
                    self._has_work = False
            if work is None:
                time.sleep(0.01)
                continue

            frame, boxes = work
            labels = []

            if not self._encs or not boxes:
                with self._lock:
                    self._labels = [("Unknown", 0)] * len(boxes)
                continue

            small = cv2.resize(frame, None,
                               fx=self._scale, fy=self._scale,
                               interpolation=cv2.INTER_AREA)
            rgb   = cv2.cvtColor(small, cv2.COLOR_BGR2RGB)
            s     = self._scale

            dlib_boxes = []
            for (x, y, w, h) in boxes:
                dlib_boxes.append((
                    max(0, int(y*s)),
                    int((x+w)*s),
                    int((y+h)*s),
                    max(0, int(x*s)),
                ))

            try:
                encodings = face_recognition.face_encodings(rgb, dlib_boxes)
            except Exception:
                encodings = []

            for enc in encodings:
                distances = face_recognition.face_distance(self._encs, enc)
                best_idx  = int(np.argmin(distances))
                best_dist = float(distances[best_idx])
                if best_dist <= self._tolerance:
                    name = self._names[best_idx]
                    conf = int(max(0, min(100,
                               (1.0 - best_dist / self._tolerance) * 100)))
                else:
                    name, conf = "Unknown", 0
                labels.append((name, conf))

            while len(labels) < len(boxes):
                labels.append(("Unknown", 0))

            with self._lock:
                self._labels = labels


# ══════════════════════════════════════════════════════════════════
#  Target selection
# ══════════════════════════════════════════════════════════════════
def select_target(faces, labels, prev_cx, prev_cy):
    if not faces:
        return -1
    known = [(i, labels[i][1]) for i in range(len(faces))
             if i < len(labels) and labels[i][0] != "Unknown"]
    if known:
        return max(known, key=lambda t: t[1])[0]
    def score(i):
        x, y, w, h = faces[i]
        cx, cy = x+w/2, y+h/2
        return w*h - ((cx-prev_cx)**2 + (cy-prev_cy)**2)**0.5 * 0.4
    return max(range(len(faces)), key=score)


# ══════════════════════════════════════════════════════════════════
#  Camera stream
# ══════════════════════════════════════════════════════════════════
class CameraStream:
    def __init__(self, src=1):
        # Auto-detect backend to avoid black screen issues with DSHOW
        self.cap = cv2.VideoCapture(src)
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
#  Serial helpers
# ══════════════════════════════════════════════════════════════════
def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def build_command(pan: int, tilt: int, locked_name: str = "") -> bytes:
    p = clamp(pan,  STOP_PAN  - MAX_SPEED, STOP_PAN  + MAX_SPEED)
    t = clamp(tilt, STOP_TILT - MAX_SPEED, STOP_TILT + MAX_SPEED)
    if locked_name:
        tag = locked_name.replace(" ", "_")[:12]
        return f"P{p:03d}T{t:03d}K{tag}\n".encode("ascii")
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
def draw_hud(frame, faces, labels, target_idx,
             status, pan_cmd, tilt_cmd, fps,
             smooth_cx, smooth_cy, locked_name):

    h_f, w_f = frame.shape[:2]

    for i, (x, y, w, h) in enumerate(faces):
        name, conf = labels[i] if i < len(labels) else ("?", 0)
        is_target  = (i == target_idx)

        if name != "Unknown" and is_target and locked_name:
            col, thick = COL_LOCKED, 3
        elif name != "Unknown":
            col, thick = COL_KNOWN, 2
        else:
            col, thick = COL_UNKNOWN, 2

        cv2.rectangle(frame, (x, y), (x+w, y+h), col, thick, cv2.LINE_AA)

        label_str = f"{name}" + (f"  {conf}%" if conf else "")
        (tw, th), _ = cv2.getTextSize(label_str, cv2.FONT_HERSHEY_SIMPLEX, 0.52, 1)
        tag_y = max(y - 6, th + 4)
        cv2.rectangle(frame, (x, tag_y-th-4), (x+tw+6, tag_y+2), col, -1)
        cv2.putText(frame, label_str, (x+3, tag_y-1),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.52, (0,0,0), 1, cv2.LINE_AA)

    overlay = frame.copy()
    cv2.rectangle(overlay, (0,0), (w_f, 40), (10,10,10), -1)
    cv2.addWeighted(overlay, 0.45, frame, 0.55, 0, frame)

    if locked_name:
        bar_col  = COL_LOCKED
        bar_text = (f"[LOCKED: {locked_name}]   "
                    f"Pan {pan_cmd:3d}  Tilt {tilt_cmd:3d}   FPS {fps:4.1f}")
    elif status == "TRACKING":
        bar_col  = COL_KNOWN
        bar_text = f"[TRACKING]   Pan {pan_cmd:3d}  Tilt {tilt_cmd:3d}   FPS {fps:4.1f}"
    elif status == "PREDICTING":
        bar_col  = (180, 180, 0)
        bar_text = f"[PREDICTING]   Pan {pan_cmd:3d}  Tilt {tilt_cmd:3d}   FPS {fps:4.1f}"
    else:
        bar_col  = (60, 60, 220)
        bar_text = f"[NO FACE]   FPS {fps:4.1f}"

    cv2.putText(frame, bar_text, (10, 27),
                cv2.FONT_HERSHEY_SIMPLEX, 0.58, bar_col, 2, cv2.LINE_AA)

    cx, cy = w_f//2, h_f//2
    arm    = 10 if status == "TRACKING" else 18
    cv2.line(frame, (cx-arm, cy), (cx+arm, cy), COL_RETICLE, 1, cv2.LINE_AA)
    cv2.line(frame, (cx, cy-arm), (cx, cy+arm), COL_RETICLE, 1, cv2.LINE_AA)
    cv2.circle(frame, (cx, cy), arm+8, COL_RETICLE, 1, cv2.LINE_AA)

    if smooth_cx is not None and status in ("TRACKING", "PREDICTING"):
        cv2.circle(frame, (int(smooth_cx), int(smooth_cy)), 5, bar_col, -1, cv2.LINE_AA)

    cv2.putText(frame, "Q = quit", (10, h_f-10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.38, (100,100,100), 1, cv2.LINE_AA)


# ══════════════════════════════════════════════════════════════════
#  Main
# ══════════════════════════════════════════════════════════════════
def main():
    parser = argparse.ArgumentParser(description="DART - face tracking + recognition")
    parser.add_argument("--camera",    type=int,   default=1,               help="Camera index")
    parser.add_argument("--port",      type=str,   default="COM3",          help="Serial port")
    parser.add_argument("--faces",     type=str,   default=KNOWN_FACES_DIR, help="known_faces folder")
    parser.add_argument("--tolerance", type=float, default=RECOG_TOLERANCE, help="Recognition tolerance")
    args = parser.parse_args()

    print("[INFO] Loading known faces ...")
    known_encs, known_names = load_known_faces(args.faces)

    print(f"[INFO] Opening camera {args.camera} ...")
    cam        = CameraStream(src=args.camera)
    detector   = FaceDetector(scale=DETECTION_SCALE)
    recogniser = RecognitionWorker(known_encs, known_names,
                                   args.tolerance, RECOG_SCALE)

    pid_pan  = PID(PAN_KP,  PAN_KI,  PAN_KD,  MAX_SPEED)
    pid_tilt = PID(TILT_KP, TILT_KI, TILT_KD, MAX_SPEED)
    kalman   = make_kalman()
    kalman_init = False

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

    smooth_cx = smooth_cy = None
    no_face_count = 0
    smooth_pan = smooth_tilt = 0.0
    prev_cx = prev_cy = 0.0

    frame_count   = 0
    cached_labels = []
    lock_counter  = 0
    locked_name   = ""

    last_send = 0.0
    last_time = time.monotonic()
    fps_buf   = collections.deque(maxlen=30)

    print("[INFO] Running - Q to quit")

    while True:
        grabbed, frame = cam.read()
        if not grabbed or frame is None:
            time.sleep(0.01)
            continue

        now       = time.monotonic()
        dt        = max(now - last_time, 1e-6)
        last_time = now
        fps_buf.append(1.0 / dt)
        fps = sum(fps_buf) / len(fps_buf)

        h_f, w_f  = frame.shape[:2]
        cx_frame  = w_f // 2
        cy_frame  = h_f // 2
        frame_count += 1

        detector.submit(frame)
        faces = detector.get_results()

        if faces and frame_count % RECOG_INTERVAL == 0:
            recogniser.submit(frame, faces)

        labels = recogniser.get_labels()
        while len(labels) < len(faces):
            labels.append(("Unknown", 0))
        cached_labels = labels[:len(faces)]

        cx_prev = smooth_cx if smooth_cx is not None else prev_cx
        cy_prev = smooth_cy if smooth_cy is not None else prev_cy
        t_idx   = select_target(faces, cached_labels, cx_prev, cy_prev)

        pan_raw = tilt_raw = 0.0
        status  = "NO FACE"

        if t_idx >= 0:
            no_face_count = 0
            fx, fy, fw, fh = faces[t_idx]
            raw_cx = float(fx + fw/2)
            raw_cy = float(fy + fh/2)

            if smooth_cx is None:
                smooth_cx, smooth_cy = raw_cx, raw_cy
            else:
                smooth_cx = SMOOTH*smooth_cx + (1-SMOOTH)*raw_cx
                smooth_cy = SMOOTH*smooth_cy + (1-SMOOTH)*raw_cy

            meas = np.array([[smooth_cx],[smooth_cy]], np.float32)
            if not kalman_init:
                kalman.statePre = np.array(
                    [[smooth_cx],[smooth_cy],[0],[0]], np.float32)
                kalman_init = True
            kalman.correct(meas)

            prev_cx, prev_cy = smooth_cx, smooth_cy

            error_x = smooth_cx - cx_frame
            error_y = smooth_cy - cy_frame

            if abs(error_x) > DEAD_ZONE:
                sign_x  = 1.0 if error_x > 0 else -1.0
                pan_raw = pid_pan.update(
                    (abs(error_x)-DEAD_ZONE)*sign_x*INVERT_PAN, dt)
            else:
                pid_pan.reset()

            if abs(error_y) > DEAD_ZONE:
                sign_y   = 1.0 if error_y > 0 else -1.0
                tilt_raw = pid_tilt.update(
                    (abs(error_y)-DEAD_ZONE)*sign_y*INVERT_TILT, dt)
            else:
                pid_tilt.reset()

            status = "TRACKING"

            t_name  = cached_labels[t_idx][0] if t_idx < len(cached_labels) else "Unknown"
            centred = abs(error_x) <= DEAD_ZONE and abs(error_y) <= DEAD_ZONE

            if t_name != "Unknown" and centred:
                lock_counter += 1
            else:
                lock_counter = max(0, lock_counter - 1)

            locked_name = t_name if lock_counter >= LOCKED_FRAMES_NEEDED else ""

        else:
            no_face_count += 1
            lock_counter   = 0
            locked_name    = ""

            if kalman_init and no_face_count <= 8:
                pred      = kalman.predict()
                smooth_cx = float(pred[0,0])
                smooth_cy = float(pred[1,0])
                status    = "PREDICTING"
            else:
                kalman.predict()
                if no_face_count > 15:
                    smooth_cx = smooth_cy = None
                    kalman_init = False
                    pid_pan.reset()
                    pid_tilt.reset()

        smooth_pan  = CMD_SMOOTH*smooth_pan  + (1-CMD_SMOOTH)*pan_raw
        smooth_tilt = CMD_SMOOTH*smooth_tilt + (1-CMD_SMOOTH)*tilt_raw
        pan_cmd  = STOP_PAN  + int(round(smooth_pan))
        tilt_cmd = STOP_TILT + int(round(smooth_tilt))

        draw_hud(frame, faces, cached_labels, t_idx,
                 status, pan_cmd, tilt_cmd, fps,
                 smooth_cx, smooth_cy, locked_name)

        if ser and (now - last_send) >= SEND_INTERVAL:
            try:
                ser.write(build_command(pan_cmd, tilt_cmd, locked_name))
            except serial.SerialException:
                pass
            last_send = now

        cv2.imshow("DART  [Recognition]", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    send_stop(ser)
    if ser:
        ser.close()
    recogniser.stop()
    detector.stop()
    cam.stop()
    cv2.destroyAllWindows()
    print("[INFO] Shut down.")


if __name__ == "__main__":
    main()