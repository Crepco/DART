# DART — Dynamic Autonomous Recognition Turret

Autonomous pan–tilt security turret: a laptop runs the vision pipeline (person tracking +
face authorization), computes servo commands, and streams them to an Arduino Uno R3 that
drives the turret. It tracks everyone in frame but the fire gate only ever arms on a
**confirmed UNAUTHORIZED** target held on a dwelled lock — authorized faces are never
engaged.

## How it works

```
Camera ─► YOLOv8n + ByteTrack (person identity, custom bytetrack_dart.yaml)
       ─► YOLOv8n-face (aim point = face centroid)
       ─► InsightFace buffalo_sc embeddings vs authorized.pkl  ─► per-track verdict
       ─► PID ─► asymmetric EMA ─► brake-aware slew limiter ─► "P###T###F#" serial
       ─► Arduino Uno R3 ─► MG996R pan/tilt (continuous rotation) + MG90 trigger
```

- **Tracking** ([scripts/core/detector.py](scripts/core/detector.py)): YOLOv8n person
  detection with ByteTrack association. The tracker is fed detections down to
  `TRACK_CONF` (0.1) so its low-confidence second stage can hold identity through
  blur/pose/lighting dips, while only boxes ≥ `PERSON_CONF` (0.45) are displayed,
  targeted, or classified. Tracker thresholds live in
  [scripts/bytetrack_dart.yaml](scripts/bytetrack_dart.yaml).
- **Authorization** ([scripts/auth/](scripts/auth/)): InsightFace `buffalo_sc` embeddings
  compared against `scripts/embeddings/authorized.pkl`. Verdicts are debounced per track,
  frozen once UNAUTHORIZED is confirmed, and survive short tracking dropouts
  (`TRACK_STATE_GRACE_S`). The run page shows the active ONNX provider and per-verdict
  latency — `CPU (degraded)` means a CUDA GPU is present but unused (see install notes).
- **Control** ([scripts/core/pid.py](scripts/core/pid.py), [scripts/config.py](scripts/config.py)):
  per-axis PID on the face-centroid error, then an attack/decay command EMA and a slew
  limiter that brakes `SLEW_BRAKE_MULT`× faster than it accelerates — the servos are
  velocity-commanded, so fast release is what stops overshoot.
- **Serial** ([docs/SERIAL_PROTOCOL.md](docs/SERIAL_PROTOCOL.md)): `P###T###F#\n` at up to
  30 Hz with a 0.4 s heartbeat; the Arduino stops everything after 500 ms of silence.

## Hardware

| Function     | Pin | Part                              |
|--------------|-----|-----------------------------------|
| Pan servo    | 10  | MG996R (continuous rotation)      |
| Tilt servo   | 9   | MG996R (continuous rotation)      |
| Trigger      | 8   | MG90 (positional)                 |
| Status LED   | 13  | onboard LED                       |

Arduino Uno R3 over USB serial @ 115200 baud; servo power from an external supply with
common ground. Firmware:
[arduino/arduino_servo_controller/](arduino/arduino_servo_controller/arduino_servo_controller.ino).

## Install

```
pip install -r requirements.txt        # NVIDIA GPU (CUDA 12.6 torch + onnxruntime-gpu)
pip install -r requirements-cpu.txt    # CPU-only machine (much smaller download)
```

> **GPU hosts:** insightface also pulls in the CPU `onnxruntime`, which shadows
> `onnxruntime-gpu` and silently forces face auth onto the CPU (5–10× slower — the run
> page will show `CPU (degraded)`). After installing: `pip uninstall -y onnxruntime`.
> Details in [requirements.txt](requirements.txt).

Models: `models/yolov8n.pt` (auto-downloaded by ultralytics) and `models/yolov8n-face.pt`
(from <https://github.com/akanametov/yolov8-face/releases>; face boxes are skipped if
missing).

## Run

```
python scripts/run_web.py        # web UI -> http://127.0.0.1:5000
# or double-click run_dart_web.bat on Windows
python scripts/main.py           # desktop OpenCV window (same pipeline)
```

**Landing page** — pick a camera and the Arduino serial port, launch DART, or authorize a
person without launching anything. **Run page** — live annotated feed, status strip
(status / serial / FPS / auth provider+latency), STOP, and the same Authorize Person
panel.

### Authorize Person

Enter a name, then either **capture from the camera** (a ~2.5 s burst; frames must pass a
quality gate — face size, detector confidence, sharpness — before anything is saved) or
**upload photos**. Works with DART running (uses the live feed), or stopped (opens the
camera directly). Enrollment updates `scripts/embeddings/authorized.pkl`, hot-reloads the
classifier, and re-checks currently tracked UNAUTHORIZED persons — no restart. CLI
equivalents: [scripts/auth/enroll.py](scripts/auth/enroll.py) and
[scripts/auth/enroll_from_images.py](scripts/auth/enroll_from_images.py).

> **Security note:** the `/authorize` endpoints ARE the turret's access control and have
> no authentication of their own. The server binds `127.0.0.1` by default and must stay
> that way — if you expose it on `0.0.0.0`, anyone on the network can authorize
> themselves.

## Tuning knobs worth knowing (scripts/config.py)

| Constant | What it does |
|----------|--------------|
| `CONTROL_DEBUG` | Per-frame `err → pid → ema → slew → cmd` log line per axis — compare raw PID vs post-slew when tuning overshoot. |
| `TRACK_DEBUG` | Logs track-ID set changes (`+id(conf)` / `-id`) — see exactly when detection confidence dips and whether identity survives. |
| `SLEW_BRAKE_MULT` | Braking slew multiplier (default 3.0, deliberately unproven — sweep 2/3/4 on hardware with `CONTROL_DEBUG` on). |
| `SMOOTH` | Centroid EMA. 0.70 balances lag vs jitter; if new at-rest oscillation appears, step to 0.80 before touching gains. |
| `PERSON_CONF` / `TRACK_CONF` | Display/targeting bar vs the low bar fed to ByteTrack so identity survives confidence dips. |
| `TRACK_STATE_GRACE_S` | Seconds an unseen track keeps its auth verdict (time-based, fps-independent). |

## Tests

```
python -m pip install pytest    # dev-only dependency
python -m pytest tests/
```

Covers the control helpers (asymmetric EMA, brake slew), `/start` idempotency, the
authorize dispatch (running/starting/stopped), enrollment save/overwrite + quality gate,
and track-verdict debounce/grace logic.

## Docs

- [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) — pipeline, threading, control law, tuning guide
- [docs/FACE_TRACKER.md](docs/FACE_TRACKER.md) — tracking + authorization pipeline in detail
- [docs/SERIAL_PROTOCOL.md](docs/SERIAL_PROTOCOL.md) — wire format, timing, watchdog
