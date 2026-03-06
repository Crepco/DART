"""
Face-Tracking Pan-Tilt Turret Controller
=========================================
For CONTINUOUS ROTATION (360 deg) MG996R servos.

Pin 9  = TILT (up/down)
Pin 10 = PAN  (left/right)

When no face is detected the servos stop (hold position).

Upload arduino_servo_controller.ino to your Arduino FIRST.
"""

import argparse   # Parse command-line arguments (--camera, --port)
import threading  # Run camera and detector in background threads
import time       # Sleep, timing, and monotonic clock

import cv2    # OpenCV: video capture and face detection
import serial  # PySerial: talk to Arduino over USB

# ══════════════════════════════════════════════════════════════════
#  CONFIG
# ══════════════════════════════════════════════════════════════════

STOP_PAN  = 90   # Servo value for pan "stop" (continuous rotation neutral)
STOP_TILT = 90   # Servo value for tilt "stop" (continuous rotation neutral)

MAX_SPEED = 35   # Max offset from 90; servo range becomes 65..115

GAIN_PAN  = 0.10  # How much pan speed per pixel of horizontal error (after dead zone)
GAIN_TILT = 0.09  # How much tilt speed per pixel of vertical error (after dead zone)

DEAD_ZONE = 40   # Pixels from center: no correction inside this; reduces jitter

INVERT_PAN  = -1  # Multiply pan command by this (use -1 to reverse left/right)
INVERT_TILT = 1   # Multiply tilt command by this (use -1 to reverse up/down)

SMOOTH = 0.65  # Face position smoothing (0..1); higher = smoother but laggier

# Smoothing on the servo command itself (0..1).  Higher = more stable but slower to react.
CMD_SMOOTH = 0.7

DETECTION_SCALE = 0.5   # Run face detection on half-size frame for speed
MIN_FACE_SIZE = 30      # Smallest face width/height to detect (in scaled pixels)

BAUD_RATE = 9600        # Serial baud rate (must match Arduino)
SEND_HZ = 30            # Max commands per second to Arduino
SEND_INTERVAL = 1.0 / SEND_HZ  # Seconds between sends


# ══════════════════════════════════════════════════════════════════
#  Threaded camera capture
# ══════════════════════════════════════════════════════════════════
class CameraStream:
    def __init__(self, src: int = 0):
        self.cap = cv2.VideoCapture(src, cv2.CAP_DSHOW)  # Open camera; use DirectShow on Windows
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))  # Prefer MJPEG for low lag
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Minimize buffered frames
        self.grabbed, self.frame = self.cap.read()  # Get first frame
        self._lock = threading.Lock()   # Protect shared frame from main + thread
        self._stopped = False           # Signal to stop the capture thread
        self._thread = threading.Thread(target=self._update, daemon=True)  # Background reader
        self._thread.start()           # Start the thread

    def _update(self):
        while not self._stopped:        # Run until stopped
            grabbed, frame = self.cap.read()   # Read next frame (blocking)
            with self._lock:            # Update shared state safely
                self.grabbed = grabbed
                self.frame = frame

    def read(self):
        with self._lock:                # Copy latest frame for caller
            return self.grabbed, (self.frame.copy() if self.frame is not None else None)

    def stop(self):
        self._stopped = True           # Tell thread to exit
        self._thread.join()            # Wait for it to finish
        self.cap.release()             # Release camera


# ══════════════════════════════════════════════════════════════════
#  Threaded face detector
# ══════════════════════════════════════════════════════════════════
class FaceDetector:
    def __init__(self, scale: float = DETECTION_SCALE):
        self.cascade = cv2.CascadeClassifier(  # Load Haar cascade for frontal faces
            cv2.data.haarcascades + "haarcascade_frontalface_default.xml"
        )
        self.scale = scale             # Downscale factor for detection (e.g. 0.5)
        self._lock = threading.Lock()  # Protect frame and result
        self._frame = None             # Latest frame handed to detector
        self._new_frame = False        # True when a new frame is waiting
        self._result = None            # Latest (x, y, w, h) or None
        self._stopped = False          # Signal to stop the detector thread
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def submit(self, frame):
        with self._lock:
            self._frame = frame        # Give detector a new frame to process
            self._new_frame = True

    def get_result(self):
        with self._lock:
            return self._result        # Return latest detection or None

    def _run(self):
        while not self._stopped:
            with self._lock:
                if not self._new_frame or self._frame is None:
                    frame = None       # Nothing new to process
                else:
                    frame = self._frame.copy()  # Take a copy so main can keep updating
                    self._new_frame = False

            if frame is None:
                time.sleep(0.005)      # Short sleep to avoid busy loop
                continue

            small = cv2.resize(        # Shrink frame for faster detection
                frame, None, fx=self.scale, fy=self.scale,
                interpolation=cv2.INTER_AREA,
            )
            gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)  # Cascade needs grayscale
            faces = self.cascade.detectMultiScale(
                gray,
                scaleFactor=1.15,       # Scale step between pyramid levels
                minNeighbors=6,        # Fewer = more detections, more false positives
                minSize=(MIN_FACE_SIZE, MIN_FACE_SIZE),
            )

            if len(faces) == 0:
                with self._lock:
                    self._result = None  # No face found
                continue

            largest = max(faces, key=lambda r: r[2] * r[3])  # Pick biggest face (w * h)
            x, y, w, h = largest       # Bounding box in scaled image
            inv = 1.0 / self.scale     # Scale factor back to full resolution
            with self._lock:
                self._result = (       # Store box in full-frame coordinates
                    int(x * inv), int(y * inv),
                    int(w * inv), int(h * inv),
                )

    def stop(self):
        self._stopped = True
        self._thread.join()


# ══════════════════════════════════════════════════════════════════
#  Helpers
# ══════════════════════════════════════════════════════════════════
def clamp(value, lo, hi):
    return max(lo, min(hi, value))  # Clamp value between lo and hi


def to_int_offset(raw_offset):
    return int(round(raw_offset))  # Convert float speed offset to integer for serial


def build_command(pan_speed: int, tilt_speed: int) -> bytes:
    p = clamp(pan_speed, STOP_PAN - MAX_SPEED, STOP_PAN + MAX_SPEED)   # Keep in safe range
    t = clamp(tilt_speed, STOP_TILT - MAX_SPEED, STOP_TILT + MAX_SPEED)
    return f"P{p:03d}T{t:03d}\n".encode("ascii")  # e.g. P090T085\n


def send_stop(ser):
    if ser:
        try:
            ser.write(build_command(STOP_PAN, STOP_TILT))  # Send 90,90 to stop both servos
        except serial.SerialException:
            pass


# ══════════════════════════════════════════════════════════════════
#  Main
# ══════════════════════════════════════════════════════════════════
def main():
    parser = argparse.ArgumentParser(description="Face-tracking turret")
    parser.add_argument("--camera", type=int, default=0)   # Camera index (0 = first)
    parser.add_argument("--port", type=str, default="COM3") # Serial port (e.g. COM3, /dev/ttyUSB0)
    args = parser.parse_args()

    print(f"[INFO] Opening camera {args.camera} ...")
    cam = CameraStream(args.camera)    # Start threaded camera
    detector = FaceDetector(scale=DETECTION_SCALE)  # Start threaded face detector

    ser = None
    try:
        ser = serial.Serial(args.port, BAUD_RATE, timeout=0.5)  # Open serial port
        print(f"[INFO] Waiting for Arduino reset ...")
        time.sleep(2.5)                # Arduino resets when serial opens; wait for boot
        ser.reset_input_buffer()       # Discard any boot garbage
        ser.reset_output_buffer()

        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:  # Wait for Arduino to say READY
            if ser.in_waiting:
                line = ser.readline().decode("ascii", errors="ignore").strip()
                print(f"[ARDUINO] {line}")
                if line == "READY":
                    break
            time.sleep(0.05)

        send_stop(ser)                 # Send stop so servos don't drift on startup
        print(f"[INFO] Serial ready on {args.port}")
    except serial.SerialException as exc:
        print(f"[WARN] Could not open {args.port}: {exc}")
        print("[WARN] Preview-only mode.")

    smooth_cx = None   # Smoothed face center X (None until first detection)
    smooth_cy = None   # Smoothed face center Y
    no_face_count = 0 # Consecutive frames with no face (reset when face seen)

    smooth_pan = 0.0   # Smoothed pan command (reduces servo jitter)
    smooth_tilt = 0.0  # Smoothed tilt command

    last_send = 0.0    # Time of last serial send (for rate limiting)

    print("[INFO] Q = quit")

    while True:
        grabbed, frame = cam.read()    # Get latest frame from camera thread
        if not grabbed or frame is None:
            time.sleep(0.01)
            continue

        now = time.monotonic()
        h_frame, w_frame = frame.shape[:2]   # Frame height and width
        cx_frame, cy_frame = w_frame // 2, h_frame // 2  # Center of frame (target)

        detector.submit(frame)         # Hand frame to detector thread
        face = detector.get_result()   # Get latest detection (may be stale by one frame)

        pan_int = 0    # Pan speed offset to send this frame (-MAX_SPEED .. +MAX_SPEED)
        tilt_int = 0   # Tilt speed offset

        if face is not None:
            no_face_count = 0          # We saw a face; reset dropout counter

            fx, fy, fw, fh = face      # Face bounding box (full-frame coords)
            raw_cx = float(fx + fw // 2)   # Raw face center X
            raw_cy = float(fy + fh // 2)   # Raw face center Y

            if smooth_cx is None:
                smooth_cx = raw_cx     # First time: use raw position
                smooth_cy = raw_cy
            else:
                smooth_cx = SMOOTH * smooth_cx + (1.0 - SMOOTH) * raw_cx   # Exponential moving average
                smooth_cy = SMOOTH * smooth_cy + (1.0 - SMOOTH) * raw_cy

            error_x = smooth_cx - cx_frame   # Positive = face is right of center
            error_y = smooth_cy - cy_frame   # Positive = face is below center

            if abs(error_x) > DEAD_ZONE:
                sign_x = 1.0 if error_x > 0 else -1.0
                effective_x = abs(error_x) - DEAD_ZONE   # Only correct error outside dead zone
                raw = clamp(sign_x * effective_x * GAIN_PAN * INVERT_PAN,
                            -MAX_SPEED, MAX_SPEED)
                pan_int = to_int_offset(raw)

            if abs(error_y) > DEAD_ZONE:
                sign_y = 1.0 if error_y > 0 else -1.0
                effective_y = abs(error_y) - DEAD_ZONE
                raw = clamp(sign_y * effective_y * GAIN_TILT * INVERT_TILT,
                            -MAX_SPEED, MAX_SPEED)
                tilt_int = to_int_offset(raw)

            cv2.rectangle(frame, (fx, fy), (fx + fw, fy + fh), (0, 255, 0), 2)  # Draw face box
            cv2.circle(frame, (int(smooth_cx), int(smooth_cy)), 4, (0, 255, 0), -1)  # Draw smoothed center
            status = "TRACKING"
        else:
            no_face_count += 1
            if no_face_count > 15:     # After 15 frames with no face, clear smoothed position
                smooth_cx = None       # So next detection doesn't jump
                smooth_cy = None
            status = "NO FACE"

        smooth_pan  = CMD_SMOOTH * smooth_pan  + (1.0 - CMD_SMOOTH) * pan_int   # Smooth command output
        smooth_tilt = CMD_SMOOTH * smooth_tilt + (1.0 - CMD_SMOOTH) * tilt_int

        pan_cmd  = STOP_PAN  + int(round(smooth_pan))   # Final servo value (e.g. 90 + offset)
        tilt_cmd = STOP_TILT + int(round(smooth_tilt))

        # HUD
        cv2.line(frame, (cx_frame - 15, cy_frame), (cx_frame + 15, cy_frame), (255, 0, 0), 1)   # Crosshair horizontal
        cv2.line(frame, (cx_frame, cy_frame - 15), (cx_frame, cy_frame + 15), (255, 0, 0), 1)  # Crosshair vertical
        cv2.putText(frame, f"Pan {pan_cmd}  Tilt {tilt_cmd}  [{status}]",
                     (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 200, 255), 2)
        cv2.putText(frame, "Q = quit",
                     (10, h_frame - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (150, 150, 150), 1)

        # Send to Arduino
        if ser and (now - last_send) >= SEND_INTERVAL:  # Rate limit serial sends
            cmd = build_command(pan_cmd, tilt_cmd)
            try:
                ser.write(cmd)
            except serial.SerialException:
                pass
            last_send = now

        cv2.imshow("Face Tracker", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):  # Quit on 'q'
            break

    send_stop(ser)         # Stop servos before exit
    if ser:
        ser.close()
    detector.stop()        # Stop detector thread
    cam.stop()             # Stop camera thread
    cv2.destroyAllWindows()
    print("[INFO] Shut down.")


if __name__ == "__main__":
    main()  # Run only when script is executed directly (not imported)
