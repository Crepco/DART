# Phase 1 — Python Script: `face_tracker.py`

Face-tracking pan–tilt turret controller for the laptop side of Phase 1. It captures video, detects faces with OpenCV, computes pan/tilt commands for **continuous-rotation** MG996R servos, and sends them to the Arduino over USB serial.

---

## Overview

| Item | Detail |
|------|--------|
| **Script** | `face_tracker.py` |
| **Role** | Vision (camera + detection) and command generation; sends `P###T###` to Arduino |
| **Servo type** | Continuous rotation MG996R (90 = stop; &lt;90 / &gt;90 = spin) |
| **Dependencies** | OpenCV, PySerial (see `requirements.txt`) |

**Run after** uploading `arduino_servo_controller.ino` and connecting the Arduino via USB.

---

## Command-line arguments

| Argument | Type | Default | Description |
|----------|------|---------|-------------|
| `--camera` | int | `0` | Camera index (0 = first webcam) |
| `--port` | str | `COM3` | Serial port (e.g. `COM3` on Windows, `/dev/ttyUSB0` on Linux) |

**Example:**
```bash
python face_tracker.py --camera 0 --port COM3
```

---

## Configuration (constants)

All tuning is at the top of the script.

| Constant | Default | Meaning |
|----------|---------|--------|
| `STOP_PAN` / `STOP_TILT` | `90` | Servo value for “stop” (continuous-rotation neutral) |
| `MAX_SPEED` | `25` | Max offset from 90 → effective range 65..115 |
| `GAIN_PAN` | `0.10` | Pan speed per pixel of horizontal error (outside dead zone) |
| `GAIN_TILT` | `0.09` | Tilt speed per pixel of vertical error (outside dead zone) |
| `DEAD_ZONE` | `40` | Pixels from frame center with no correction (reduces jitter) |
| `INVERT_PAN` / `INVERT_TILT` | `-1` / `1` | Flip direction if servos turn the wrong way |
| `SMOOTH` | `0.65` | Face position smoothing (0..1); higher = smoother, more lag |
| `CMD_SMOOTH` | `0.7` | Smoothing on the servo command; higher = more stable, slower reaction |
| `DETECTION_SCALE` | `0.5` | Run detection on half-size frame for speed |
| `MIN_FACE_SIZE` | `30` | Minimum face width/height in the *scaled* image (pixels) |
| `BAUD_RATE` | `9600` | Must match Arduino `Serial.begin(9600)` |
| `SEND_HZ` | `30` | Max commands per second to Arduino |
| `SEND_INTERVAL` | `1/30` s | Minimum time between serial sends |

---

## Architecture (threading)

Three main components run in parallel:

1. **Camera thread** (`CameraStream`) — continuously reads frames so the main loop always has a fresh image.
2. **Detector thread** (`FaceDetector`) — runs Haar face detection on scaled, grayscale frames; main loop submits frames and reads the latest result.
3. **Main loop** — combines latest frame + latest detection, computes pan/tilt, draws HUD, sends commands to Arduino at `SEND_HZ`, and handles `q` to quit.

This keeps camera and detection from blocking each other and keeps serial traffic rate-limited.

---

## Components (in code order)

### 1. `CameraStream` (threaded capture)

- Opens the camera with `cv2.VideoCapture(src, cv2.CAP_DSHOW)` on Windows to reduce lag.
- Prefers MJPEG and a small buffer (`CAP_PROP_BUFFERSIZE = 1`).
- Background thread calls `cap.read()` in a loop and updates a shared frame under a lock.
- `read()` returns a copy of the latest frame so the main loop and detector never share the same buffer.
- `stop()` joins the thread and releases the camera.

### 2. `FaceDetector` (threaded detection)

- Loads the built-in Haar cascade: `haarcascade_frontalface_default.xml`.
- Main loop calls `submit(frame)` to hand a new frame; the detector thread processes it when free.
- Detection runs on a **scaled** frame (`DETECTION_SCALE`, e.g. 0.5) and grayscale for speed.
- `detectMultiScale(scaleFactor=1.15, minNeighbors=6, minSize=(MIN_FACE_SIZE, MIN_FACE_SIZE))`.
- If multiple faces, the **largest** (by area) is chosen; result is converted back to full-frame coordinates and stored as `(x, y, w, h)`.
- `get_result()` returns the latest `(x, y, w, h)` or `None` if no face.

### 3. Helpers

- **`clamp(value, lo, hi)`** — clamps a value to `[lo, hi]`.
- **`to_int_offset(raw_offset)`** — rounds float speed offset to int for serial.
- **`build_command(pan_speed, tilt_speed)`** — clamps to safe range and returns a bytes string like `P090T085\n` (3-digit pan, 3-digit tilt).
- **`send_stop(ser)`** — sends `P090T090\n` to stop both servos (e.g. on startup or exit).

### 4. Main loop logic

1. **Frame** — get latest from `CameraStream`.
2. **Detection** — `detector.submit(frame)` and `face = detector.get_result()`.
3. **Face center** — if a face exists, center = `(fx + fw/2, fy + fh/2)`; smoothed with an exponential moving average using `SMOOTH`. If no face for 15+ frames, smoothed center is cleared so the next detection doesn’t jump.
4. **Error** — `error_x = smooth_cx - frame_center_x`, `error_y = smooth_cy - frame_center_y` (positive = face right of center / below center).
5. **Dead zone** — no correction if `|error_x|` or `|error_y|` ≤ `DEAD_ZONE`; otherwise only the part outside the dead zone is used, scaled by `GAIN_PAN` / `GAIN_TILT` and `INVERT_*`, then clamped to ±`MAX_SPEED`.
6. **Command smoothing** — `smooth_pan` / `smooth_tilt` are smoothed with `CMD_SMOOTH`; final servo values = `STOP_PAN + round(smooth_pan)`, same for tilt.
7. **Serial** — every `SEND_INTERVAL` seconds, send `build_command(pan_cmd, tilt_cmd)`.
8. **HUD** — crosshair at frame center, face rectangle, smoothed center dot, and status text (pan/tilt values and TRACKING / NO FACE).
9. **Quit** — on `q`, call `send_stop(ser)`, close serial, stop detector and camera, destroy windows.

---

## Serial protocol (Python → Arduino)

- **Format:** one line per command: `P###T###\n` (e.g. `P090T085\n`).
- **Meaning:** pan and tilt are **speed** values 065–115; 090 = stop; &lt;90 one direction, &gt;90 the other.
- **Rate:** capped at `SEND_HZ` (default 30) to avoid flooding the Arduino.

---

## Startup and Arduino handshake

1. Open serial at `BAUD_RATE` (9600).
2. Wait 2.5 s for Arduino to reset after USB connect.
3. Flush input/output buffers.
4. Wait up to 2 s for a line from Arduino containing `READY` (Arduino sends this once in `setup()`).
5. Send stop command so servos don’t drift.
6. If serial fails to open, script runs in “preview-only” mode (no Arduino, no crash).

---

## Display (HUD)

- Blue crosshair at frame center (target).
- Green rectangle around detected face and green dot at smoothed center.
- Text: `Pan XXX  Tilt XXX  [TRACKING]` or `[NO FACE]`.
- Bottom: `Q = quit`.

---

## Dependencies

From `requirements.txt`:

- `opencv-python` — video capture and Haar face detection.
- `pyserial` — serial communication with Arduino.

Install: `pip install -r requirements.txt` (use a venv if desired).

---

## Pin mapping (for reference)

The script does not control pins; it only sends numbers. The Arduino maps them as:

- **Pin 9** — TILT servo (up/down).
- **Pin 10** — PAN servo (left/right).

See `arduino_servo_controller/ARDUINO_CONTROLLER.md` for Arduino-side details.
