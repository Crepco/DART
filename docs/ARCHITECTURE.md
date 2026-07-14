# DART — Architecture & Tuning Notes

System and algorithm notes for the tracking/control pipeline. Entry points:
`scripts/main.py` (desktop OpenCV window) and `scripts/web/runner.py` (headless runner
behind the Flask UI in `scripts/web/app.py`). The wire format is documented in
[SERIAL_PROTOCOL.md](SERIAL_PROTOCOL.md); the vision/auth pipeline in more detail in
[FACE_TRACKER.md](FACE_TRACKER.md).

## Overview

A laptop webcam feeds two YOLOv8 models plus an InsightFace classifier. The turret is
driven toward the tracked **face** centroid by a PID controller whose output is smoothed
and rate-limited before going over serial to the Arduino (two continuous-rotation MG996R
servos for pan/tilt, one positional MG90 trigger).

```
Camera ─► YOLO person track (ByteTrack) ─► per-track auth verdict (InsightFace)
       ─► target = confirmed-UNAUTHORIZED person, face centroid = aim point
       ─► error vs frame centre ─► PID ─► asym EMA ─► brake slew ─► serial ─► servos
       ─► lock-on test (centre error) + auth gate ─► fire flag ─► trigger servo
```

## Models

| Model             | File                      | Role                                           |
|-------------------|---------------------------|------------------------------------------------|
| `yolov8n.pt`      | `models/yolov8n.pt`       | Person detection + ByteTrack identity.         |
| `yolov8n-face.pt` | `models/yolov8n-face.pt`  | Face detection — the turret aims at the face.  |
| `buffalo_sc`      | auto-downloaded           | InsightFace embeddings for authorization.      |

`yolov8n.pt` is auto-downloaded by ultralytics. The face model comes from
<https://github.com/akanametov/yolov8-face/releases>; if missing, face boxes are skipped.
Tracker parameters live in `scripts/bytetrack_dart.yaml` (see below).

## Threading model

Four threads keep the control loop responsive and decoupled from model latency:

- **`CameraStream`** — grabs the latest frame continuously (latest-frame-wins) and
  self-heals on read failures. Exposes `read_seq()` so consumers can gate on NEW frames.
- **`YOLODetector`** — person tracking + face detection in the background; `submit()`
  hands off the newest frame, `get_result()` returns the latest result.
- **`_ClassifierWorker`** — runs InsightFace classification off the detection thread
  (single-slot mailbox, latest job wins) so a slow embed never blocks tracking.
- **Main / runner loop** — gated on new camera frames (`seq` check): re-processing the
  same frame at CPU speed used to run the loop at kHz, collapsing the PID derivative and
  making the per-frame EMAs decay far faster than their constants suggest.

## Tracking & identity

- `model.track()` is called with `conf=TRACK_CONF` (0.1), **not** `PERSON_CONF` (0.45):
  ultralytics filters detections at the predictor's `conf` *before* the tracker sees
  them, and ByteTrack's second-stage association — its mechanism for holding a track
  through blur/pose/lighting dips — needs those 0.1–0.45 detections. Downstream
  (HUD/targeting/classification) only sees boxes ≥ `PERSON_CONF`, so low-conf detections
  maintain identity and nothing else.
- `scripts/bytetrack_dart.yaml`: `track_high_thresh 0.45`, `track_low_thresh 0.1`,
  `new_track_thresh 0.5` (low-conf detections may *maintain* tracks, never *spawn* them),
  `track_buffer 60` (~4 s at 15 fps, longer as fps drops), `match_thresh 0.8`.
  Escalation order if hardware still shows fresh IDs after occlusions:
  `track_buffer` → 90, `match_thresh` → 0.7, then spatial re-ID inheritance.
- **Verdict grace**: `TrackManager` keeps a track's auth verdict for
  `TRACK_STATE_GRACE_S` (8 s) after the tracker stops reporting it — time-based, so the
  budget doesn't shrink with fps. ByteTrack only outputs *active* tracks, so without the
  grace a single missed frame wiped the verdict even when the same ID re-attached.
- `TRACK_DEBUG = True` logs track-ID set changes (`+id(conf)` / `-id`) for diagnosing
  identity drops.

## Authorization pipeline

- InsightFace `buffalo_sc` (det 256×256 + ArcFace embedding) classifies the target's
  person-crop against `scripts/embeddings/authorized.pkl`
  (cosine similarity ≥ `SIMILARITY_THRESHOLD` 0.4 → AUTHORIZED).
- Unconfirmed tracks classify every frame (fast acquire, initial debounce 2 agreeing
  reads); confirmed tracks re-check every 10 frames and need 5 agreeing reads to flip —
  except **UNAUTHORIZED is frozen** for the track's lifetime so blur can't churn a known
  target back to UNKNOWN and reset the fire dwell.
- Enrollment (web Authorize Person or the CLI tools) hot-reloads the classifier db and
  resets confirmed-UNAUTHORIZED tracks to UNKNOWN so a just-enrolled person re-classifies
  within ~2 verdicts; UNKNOWN is never a target, so the turret holds during the re-check.
- The runner exposes `auth_provider` + `auth_ms` in `/state`; `CPU (degraded)` means a
  CUDA GPU is present but the CPU `onnxruntime` build is shadowing `onnxruntime-gpu`.

## Target selection

Acquisition is stateless per frame: only **confirmed-UNAUTHORIZED** persons at full
`PERSON_CONF` are valid candidates — none means hold (the turret waits rather than
chasing UNKNOWN/AUTHORIZED tracks). With several candidates it locks the largest bbox
(nearest the camera), which is deterministic and so doesn't oscillate — and deliberately
still wins over a held lock (below): a clearly-visible unauthorized person beats a
low-conf box.

**Conf-dip hold** (`resolve_lock`, `core/targeting.py`): an *existing* lock survives
detection-confidence dips. Round-3b forensics showed pursuit was stop-start because a
dip below `PERSON_CONF` unlocked a target ByteTrack never actually dropped ("the box
disappears but the ID stays") — the loss rate was 6.6× baseline at fast pan (motion
blur), i.e. exactly mid-pursuit. The lock now persists while ByteTrack still reports
the id at ≥ `TARGET_HOLD_CONF` (0.2), its persisted verdict is still UNAUTHORIZED
(enrollment resets release the lock), and the hold is younger than `TARGET_HOLD_MAX_S`
(2 s) — bounded, so stale low-conf boxes can never steer indefinitely. Held boxes are
re-surfaced into `all_persons` so the HUD bracket and `/state` don't flicker either.
`TRACK_DEBUG` logs `hold start/end (returned|superseded|expired|lost)` markers.

**Aim point** (`AimSelector`, `core/targeting.py`): only faces **inside the locked
person's box** (inflated by half its size) may steer the aim — a false positive or
bystander across the frame is invisible to aiming (round-4c hardware: an
area-dominant global pick let big spurious detections walk the aim 100–270 px/frame,
the visible "circle shifts rapidly" defect). Among the person's own faces the one
nearest the previous aim point wins (continuity beats size; the largest face only
seeds the first frame). When the face drops — faces flap far more than person boxes
in dim light — the aim falls back to the
person box's head proxy (`HEAD_PROXY_FRAC` down from the box top) **corrected by the
face-minus-proxy offset EMA-learned while the face was visible** (only from faces
inside the locked box, so a bystander can't poison it). On the handoff frame the
offset is additionally **re-based against that frame's box** (gated to the box
inflated by half its size): a held low-conf box is often blur-distorted, and applying
an offset learned on the old geometry teleported the aim 112 px on hardware — the
turret chased it past the target. With the anchor the handoff is jump-free by
construction whatever the box does, so face flapping cannot oscillate the aim;
`D_KICK_LIMIT` remains as backstop only. A face gap therefore no longer stalls
pursuit; the coast branch now bridges only true ByteTrack losses and hold expiries.
Aim state never crosses a lock change. Finally the aim is **rate-limited within a
lock** (`AIM_STEP_LIMIT`, 45 px/frame ≈ 675 px/s > p95 genuine motion): after a long
face gap the drifted proxy's correction back onto the returning face measured
100–375 px in ONE frame on hardware — the PID chased the teleport at the rail
(perceived overshoot). A capped ramp covers the same correction smoothly; no aim
source can step the turret's target faster than physics.

**Re-acquisition confirmation** (`AIM_REACQUIRE_FRAMES`, 3): the pan's own motion
blurs the face away, and reacting to the FIRST re-detected frame fired a full-speed
correction whose motion caused the next loss — a self-sustaining 0.46–0.92 s
lose→correct→lose cycle (round-4e logs; face-loss rate is ~6× the parked baseline
at ANY nonzero pan speed in dim light, so speed-capping alone cannot break the
loop). After a face gap the aim holds the anchored proxy until the face survives
3 consecutive frames; flickers never move the aim, genuine regains start ~0.2 s
late.

**Pan speed cap**: the pan PID is limited to `PAN_MAX_SPEED` (18) instead of the
serial clamp `MAX_SPEED` (25). Measured (round 4): the track-loss rate while the pan
slews is 6–8× the parked baseline (motion blur at dim-light exposure), and every
dropout/aim artifact concentrated in saturated chases — a slower top speed buys
detection continuity and shorter blind travel. Sweep 20/18/16 if pursuit lags.

## Control law & anti-jitter

The MG996R servos are continuous-rotation: the command is a *velocity*, which makes the
plant an integrator — lag anywhere in the command path turns directly into overshoot.
Stages, in order:

1. **Centroid EMA** (`SMOOTH = 0.70`) — smooths the measured face position.
   τ ≈ 1/(1−SMOOTH) frames; 0.90 meant ~10 frames (~0.7 s) of measurement lag and was the
   main overshoot driver. Tradeoff: less jitter filtering — if new at-rest oscillation
   appears, step to 0.80 before touching gains.
2. **Input deadband** (`INPUT_DEADBAND = 30 px`) — zeroes small centre errors before the
   PID so a still target produces no command. Kept below `LOCK_ON_RADIUS` (40) so the
   turret settles inside the fire zone.
3. **PID** — per-axis, output clamped to `MAX_SPEED` (25).
   - `PAN_KP 0.075 / TILT_KP 0.04`; `PAN_KD 0.15 / TILT_KD 0.09` (anti-overshoot brake —
     raise KD before lowering KP if it still rings).
   - **Filtered derivative** (`D_SMOOTH = 0.70`): the raw derivative amplifies detection
     noise into command spikes on a velocity-commanded servo; the D term is low-passed.
   - **Derivative-kick clamp** (`D_KICK_LIMIT = 400 px/s`, applied to the raw derivative
     input before smoothing): a one-frame error jump from a mid-track target-box switch
     or detection wobble reads as thousands of px/s of fictitious motion; `KD ×` that
     pinned the command at `MAX_SPEED` for 3–8 frames (the `D_SMOOTH` memory) in hardware
     logs. Typical pursuit rates pass untouched (measured p99 ≈ 560 px/s at 15 fps, so
     only the fastest ~1% of genuine transients get softened D).
   - **Gap resync** (`PID.resync()`, called by both loops when tracking resumes): coast
     and lost frames don't call `update()`, so `prev_error` goes stale; a derivative
     computed across the gap divides a many-frame delta by a one-frame `dt` and lurches
     the turret on re-acquisition. Resync re-anchors `prev_error` to the current error
     and clears the derivative memory — the first post-gap frame is P-only, D rebuilds
     over ~2–3 frames. Fires after *every* gap (even 1-frame flickers): the cross-gap
     derivative is invalid by construction, and the cost is one P-only frame.
   - **Deadband-crossing resync** (per axis, same `PID.resync()`): crossing the
     `INPUT_DEADBAND` boundary is the same class of discontinuity, hit while the PID
     stays live — entering hides up to 30 px in one frame (hardware logs showed the
     command pinned at −25 *at zero error*, sweeping the turret for ~1 s after it had
     settled), and exiting reveals the hidden step all at once. Both loops re-anchor
     whichever axis crossed: entry yields a true zero command the frame the target
     centres; exit is P-only for one frame, exactly like a gap resume.
   - **KI = 0 on purpose**: at 0.001 the clamped integral contributed ≤ 0.01 units while
     remaining a windup liability. Tradeoff: nothing corrects steady-state bias — if a
     persistent small-angle offset appears, restore a small KI (0.001–0.005) rather than
     chasing a "bug".
4. **Asymmetric command EMA** (`asym_ema`, `CMD_SMOOTH 0.85` attack / `CMD_SMOOTH_DECAY
   0.50` release) — onset stays soft, but braking must not inherit the attack lag: a
   symmetric 0.85 EMA kept the turret coasting ~0.4 s past centre after the PID said
   "stop".
5. **Brake-aware slew limiter** (`slew_step`, 25/s pan, 15/s tilt, dt-scaled) — caps the
   command rate of change. While the step opposes the current command's sign (driving
   toward zero) the limit is multiplied by `SLEW_BRAKE_MULT` (3.0 — a starting point, not
   a measured value; sweep 2/3/4 with `CONTROL_DEBUG` on). On a direction reversal this
   brakes fast to the zero crossing, then accelerates outward at the normal rate. Tilt is
   slower because the camera rides the tilt joint (less USB-cable flex).
6. **Final output deadband** (`OUTPUT_DEADBAND = 4`, applied to the *slewed* offset) —
   without it the EMA/slew decay tail dribbles 1–3 unit commands that continuous-rotation
   servos act on (creep past centre). Also applied to the raw PID output.

`CONTROL_DEBUG = True` prints one line per frame per axis:
`err → pid → ema → slew → cmd` — compare raw PID vs post-slew at the moment of overshoot.

## Fire / lock-on logic

The trigger arms only when **all** hold:

- the locked target's *persisted* verdict is UNAUTHORIZED (auth gate — AUTHORIZED and
  UNKNOWN targets never arm, and a motion-blur frame can't reset this);
- `lock_err = hypot(error_x, error_y) ≤ LOCK_ON_RADIUS` (40 px) for `FIRE_DWELL_FRAMES`
  (3) consecutive frames.

Release with hysteresis at `LOCK_RELEASE_RADIUS` (70 px) or on target loss. Brief
detection dropouts are coasted for `FIRE_COAST_FRAMES` (5) so the dwell isn't reset by a
single blurred frame. The fire flag is sent immediately on any transition (bypasses the
serial rate gate, explicit flush).

## Arduino side

The firmware is deliberately dumb: it executes pan/tilt/fire commands as fast as they
arrive, with bounds checking. All smoothing and lock-on logic live in Python. A
`TIMEOUT_MS` (500 ms) watchdog stops all servos and forces the trigger to rest if the
host goes silent; the internal `fireState` flag is cleared too, so a stray packet at
watchdog recovery cannot re-fire from stale state.

## Hardware

| Function | Pin | Servo                          |
|----------|-----|--------------------------------|
| Tilt     | 9   | MG996R (continuous rotation)   |
| Pan      | 10  | MG996R (continuous rotation)   |
| Trigger  | 8   | MG90 (positional, 60°↔150°)    |
| Status   | 13  | LED                            |

## Usage

```
pip install -r requirements.txt          # NVIDIA GPU (CUDA 12.6 torch + onnxruntime-gpu)
pip install -r requirements-cpu.txt      # CPU-only machine
python scripts/run_web.py                # web UI (recommended)
python scripts/main.py [--camera N] [--port PORT]
```

Both requirement files install identical versions — only the torch/onnxruntime builds
differ; the runtime auto-detects CUDA and no code changes are needed to switch.
