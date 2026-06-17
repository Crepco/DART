# DART — Architecture & Tracking Notes

System and algorithm notes for the YOLO tracking pipeline (`scripts/yolo.py`) and the
Arduino servo controller (`arduino_servo_controller/arduino_servo_controller.ino`). The
wire format is documented separately in [DOCS_SERIAL_PROTOCOL.md](DOCS_SERIAL_PROTOCOL.md).

## Overview

A laptop webcam feeds two YOLOv8 models. The turret is driven toward the tracked **face**
centroid by a PID controller whose output is smoothed and rate-limited before being sent
over serial to the Arduino, which drives two continuous-rotation MG996R servos (pan, tilt)
and one positional MG90 trigger servo.

```
Camera ─► YOLO (person track + face detect) ─► face centroid
       ─► error vs frame centre ─► PID ─► EMA ─► slew limiter ─► serial ─► Arduino ─► servos
       ─► lock-on test (centre error) ─► fire flag ─► trigger servo
```

## Models

| Model            | File                     | Role                                            |
|------------------|--------------------------|-------------------------------------------------|
| `yolov8n.pt`     | `models/yolov8n.pt`      | Person tracking (ByteTrack), HUD person boxes.  |
| `yolov8n-face.pt`| `models/yolov8n-face.pt` | Face detection — the turret tracks the face.    |

`yolov8n.pt` is auto-downloaded by ultralytics on first run. The face model comes from
<https://github.com/akanametov/yolov8-face/releases>; if missing, face boxes are skipped.

## Threading model

Three threads keep the control loop responsive and decoupled from model latency:

- **`CameraStream`** — continuously grabs the latest frame (1-frame buffer) so the main
  loop never blocks on capture.
- **`YOLODetector`** — runs person tracking + face detection in the background; `submit()`
  hands off the newest frame non-blocking, `get_result()` returns the most recent result.
- **Main loop** — reads the latest frame + detection, runs PID/smoothing, draws the HUD,
  and sends serial at a fixed rate (decoupled from YOLO inference frequency).

## Tracking strategy

- Person tracking uses `.track()` with ByteTrack and locks the first seen ID, re-acquiring
  the next available person if it disappears (used for HUD context).
- The **turret is driven by the face centroid**, not the person body.
- **Continuity selection** (`select_face`): the active face is the one nearest the previous
  smoothed centroid (score = area − 0.4·distance). With no anchor yet, the largest face is
  chosen. This stops the turret hopping between faces frame-to-frame.
- **Ghost-anchor reset**: after the target is lost for >15 frames, the smoothed centroid
  anchor is reset to `None` (along with the PID/slew/EMA state). The next face to appear —
  the same person returning or a new person entering from the opposite side — instantly
  becomes the new anchor instead of being dragged toward stale coordinates.

## Control law & anti-jitter

The MG996R servos have ~50–100 ms mechanical lag, so several filters stack to keep motion
smooth and silent at rest:

1. **Centroid EMA** (`SMOOTH = 0.85`) — smooths the measured face position.
2. **Input deadband** (`INPUT_DEADBAND = 15 px`) — ignores tiny centre errors.
3. **PID** — per-axis, with output clamped to `MAX_SPEED`.
   - **KP** (~0.04): proportional anchor. Too high → overshoot against servo lag.
   - **KD** (0.11–0.12): anti-overshoot brake — the most important knob. If the turret
     overshoots when the target stops, raise KD in steps (0.10 → 0.12 → 0.15).
   - **KI** (0.001): keep very low. A target just out of reach winds up the integral and
     snaps on return; also hard-clamped by `INTEGRAL_CLAMP`.
4. **Output deadband** (`OUTPUT_DEADBAND = 3`) — suppresses buzz at centre. Raise to 4–5
   if the servos chatter when locked.
5. **Command EMA** (`CMD_SMOOTH = 0.80`) — smooths the PID output. New-sample weight is
   `1 − CMD_SMOOTH ≈ 0.2`, matching the standard form
   `out_new = α·target + (1−α)·out_old` with α ≈ 0.2.
6. **Slew-rate limiter** (`MAX_CMD_CHANGE_PER_SEC = 25.0`, dt-scaled) — caps acceleration
   to a consistent physical limit regardless of camera FPS. The limiter is allowed to ramp
   down naturally (no instant brake-to-zero) to avoid dead-band chatter on oscillation;
   state is only fully zeroed when the target is completely lost.

## Fire / lock-on logic

The MG90 trigger fires when a face is tracked **and** held near frame centre:

- `lock_err = hypot(error_x, error_y)` measured against the true frame centre (independent
  of the input deadband).
- **Engage**: `lock_err ≤ LOCK_ON_RADIUS` (40 px) for `FIRE_DWELL_FRAMES` (3) consecutive
  frames → `fire = True`.
- **Release (hysteresis)**: `lock_err > LOCK_RELEASE_RADIUS` (70 px), or target lost →
  `fire = False`.

The dwell + hysteresis prevent the trigger servo chattering between rest and fire when the
error hovers at the boundary. The fire flag is carried into the serial layer and sent
immediately on any transition (see protocol doc). Authorization gating is intentionally not
implemented in this version — any centred face fires.

## Arduino side

The firmware is deliberately dumb: it executes pan/tilt/fire commands as fast as they
arrive, with hardware bounds checking. All smoothing and lock-on logic live in Python. A
`TIMEOUT_MS` (500 ms) watchdog stops all servos and forces the trigger to rest if Python
crashes or disconnects; an internal `fireState` flag is also cleared so a stray packet
arriving at watchdog recovery cannot re-fire from stale state.

## Hardware

| Function | Pin | Servo                          |
|----------|-----|--------------------------------|
| Tilt     | 9   | MG996R (continuous rotation)   |
| Pan      | 10  | MG996R (continuous rotation)   |
| Trigger  | 8   | MG90 (positional, 60°↔150°)    |
| Status   | 13  | LED                            |

## Usage

```
pip install -r requirements.txt
python scripts/yolo.py [--camera N] [--port PORT]
```
