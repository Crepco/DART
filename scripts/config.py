"""DART turret — central configuration. All constants live here."""

import pathlib
import sys

import cv2


# ── Camera ──
CAM_INDEX  = 0
CAM_WIDTH  = 640
CAM_HEIGHT = 480
CAM_FPS    = 15

# MSMF (OpenCV's Windows default) stalls USB webcams after ~60s
# (grabFrame Error -1072873822); DirectShow is stable. Other OSes use default.
CAM_BACKEND = cv2.CAP_DSHOW if sys.platform == "win32" else cv2.CAP_ANY
CAM_FOURCC  = None     # raw/default format. MJPEG decodes USB glitches into corrupt
                       # (tiled/grayscale) frames that pass as "valid"; raw drops them
                       # cleanly. 640x480@15 fits USB 2.0 uncompressed. "MJPG" to force.

CAM_MAX_READ_FAILURES   = 30    # consecutive bad reads before reconnect (~150ms)
CAM_RECONNECT_DELAY     = 0.5   # backoff base (s)
CAM_RECONNECT_MAX_DELAY = 5.0   # backoff cap (s)
CAM_READ_FAIL_SLEEP     = 0.005 # throttle a single failed read (~5ms)

# Startup open retries: Windows/DSHOW doesn't release a device instantly after
# another process (a probe, a stopping runner) closes it, and the delay isn't
# predictable — so the first open gets several attempts, each logged.
CAM_OPEN_RETRIES     = 5
CAM_OPEN_RETRY_DELAY = 0.6      # seconds between attempts

# ── Models ──
SCRIPT_DIR   = pathlib.Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parent
PERSON_MODEL = str(PROJECT_ROOT / "models" / "yolov8n.pt")
FACE_MODEL   = str(PROJECT_ROOT / "models" / "yolov8n-face.pt")
LABELS_FILE  = str(PROJECT_ROOT / "labels.txt")
PERSON_CONF  = 0.45   # min conf for a person box to be shown/targeted/classified
FACE_CONF    = 0.45

# ── Tracking (ByteTrack) ──
# The tracker is fed detections down to TRACK_CONF so ByteTrack's second-stage
# association can hold a track through conf dips (blur, pose, lighting) — the
# predictor's conf filters dets BEFORE the tracker, so feeding it PERSON_CONF
# starved that mechanism and caused ~1s track drops + fresh IDs. Boxes below
# PERSON_CONF maintain identity only; they are never displayed or targeted.
TRACK_CONF     = 0.1
TRACKER_CONFIG = str(SCRIPT_DIR / "bytetrack_dart.yaml")   # tuned thresholds + buffer
TRACK_STATE_GRACE_S = 8.0   # seconds an unseen track keeps its auth verdict —
                            # time-based (not frames) so it doesn't shrink with fps;
                            # sized to outlive track_buffer at both fps extremes
TRACK_DEBUG = False         # True: log track-ID set changes (+id(conf) / -id)

# ── Servo ──
# Hardware-measured neutrals (calibrate_stop.py, 2026-07-12): write(90) creeps
# on this build's CR servos — 87 holds both axes still (373s dead-still
# verified). Re-run the calibration if a servo is swapped; the servo trim pot
# is the fix if no integer degree holds (and would also cure the slow idle
# creep while DART is stopped — the firmware failsafe parks at its own 90/90).
STOP_PAN    = 87
STOP_TILT   = 87
MAX_SPEED   = 25
INVERT_PAN  = -1
INVERT_TILT = 1

# ── Trigger (MG90) — angles live on the Arduino; mirrored here for parity ──
TRIGGER_REST_ANGLE = 150
TRIGGER_FIRE_ANGLE = 60
LOCK_ON_RADIUS      = 40    # px from centre to engage
LOCK_RELEASE_RADIUS = 70    # px from centre to disengage (hysteresis)
FIRE_DWELL_FRAMES   = 3     # consecutive on-target frames before firing
FIRE_COAST_FRAMES   = 5     # bridge brief face-detection dropouts during motion so the
                            # fire dwell isn't reset; keep < the 15-frame loss grace

# ── Dead Bands ──
OUTPUT_DEADBAND = 4
INPUT_DEADBAND  = 30   # px stop-zone — must be < LOCK_ON_RADIUS so it settles inside fire zone

# ── PID Gains (tuning guide: docs/ARCHITECTURE.md) ──
# KI=0 on purpose: at KI=0.001 the clamped integral contributed <=0.01 units
# (noise), while still being a windup liability. Tradeoff: nothing corrects
# steady-state bias now (off-axis mount, backlash) — if a persistent small-angle
# offset appears, restore a small KI (~0.001-0.005) rather than chasing a "bug".
PAN_KP,  PAN_KI,  PAN_KD  = 0.075, 0.0, 0.15
TILT_KP, TILT_KI, TILT_KD = 0.04, 0.0, 0.09
INTEGRAL_CLAMP = 10.0
D_SMOOTH = 0.70     # derivative low-pass (weight on previous) — stops D-term amplifying detection noise
D_KICK_LIMIT = 400.0  # px/s cap on the raw derivative input. A mid-track
                      # target-box jump (ID switch, detection wobble) puts a
                      # one-frame error delta into the derivative that reads
                      # as thousands of px/s — KD x that saturates the command
                      # at MAX_SPEED for several frames (D_SMOOTH memory). The
                      # cap de-fangs those while keeping real pursuit damping
                      # intact (measured in-track rates at 15fps: p99~560, so
                      # the top ~1% of genuine transients get softer D —
                      # deliberate). Detection gaps and INPUT_DEADBAND
                      # crossings do NOT rely on this cap: both resync the PID
                      # instead (see the resync blocks in runner.py/main.py).

# ── Smoothing ──
# SMOOTH is the dominant measurement lag: tau ~ 1/(1-SMOOTH) frames. 0.90 meant
# ~10 frames (~0.7s at 15fps) of phase lag — the turret was always steering at
# where the target USED to be, the main overshoot driver. 0.70 ~ 3.3 frames.
# Tradeoff: less jitter filtering. If new at-rest oscillation appears on
# hardware, step to 0.80 before anything else.
SMOOTH           = 0.70   # centroid EMA (weight on previous value)
CMD_SMOOTH       = 0.85   # servo command EMA onset (attack) — softer spin-up
CMD_SMOOTH_DECAY = 0.50   # ...but release faster than attack: after the PID says
                          # "stop", 0.85 kept the turret coasting ~0.4s past center

# ── Slew Rate (servo-offset units / second, dt-scaled) ──
MAX_CMD_CHANGE_PER_SEC_PAN  = 25.0   # pan slew rate
MAX_CMD_CHANGE_PER_SEC_TILT = 15.0   # tilt slew — gentler: the camera rides the tilt
                                     # joint, so slower tilt = less USB-cable flex/shock
SLEW_BRAKE_MULT = 3.0   # braking (command magnitude decreasing) may slew this much
                        # faster than attack — decelerating from cruise 20 took 0.8s
                        # at 1x, all of it overshoot. NOT a measured value: sweep
                        # 2/3/4 on hardware with CONTROL_DEBUG on.

# ── Control debug ──
CONTROL_DEBUG = False   # True: print per-frame err/pid/ema/slew/cmd for both axes
                        # (compare raw PID vs post-slew at the overshoot moment)

# ── Serial ──
PORT          = "COM3"
BAUD_RATE     = 115200
SEND_HZ       = 30
SEND_INTERVAL = 1.0 / SEND_HZ
HEARTBEAT_INTERVAL = 0.4

# ── Enrollment (web "Authorize Person" flow) ──
ENROLL_FRAMES         = 8     # frames grabbed per live capture
ENROLL_FRAME_INTERVAL = 0.3   # s between grabs (~2.4s capture window)
ENROLL_MIN_ACCEPTED   = 4     # quality-gated embeddings required before saving
# Quality gate — a frame only contributes an embedding if the face is big,
# confident and sharp; bad captures would permanently pollute the pkl db.
ENROLL_MIN_FACE       = 80    # px, min side of the face bbox
ENROLL_MIN_DET_SCORE  = 0.5   # InsightFace detector confidence
ENROLL_MIN_SHARPNESS  = 60.0  # variance-of-Laplacian on the face crop

# ── Auth ──
SIMILARITY_THRESHOLD     = 0.4
AUTH_MIN_CROP    = 120   # px: person crops smaller than this on either side get
AUTH_UPSCALE     = 2.0   # upscaled before InsightFace detection — a distant face
                         # the 256px detector would miss otherwise returns UNKNOWN
                         # forever, which stalls the verdict debounce indefinitely.
CLASSIFY_EVERY_N_FRAMES  = 10   # steady-state re-check cadence for a confirmed track
CLASSIFY_EVERY_N_FRAMES_UNCONFIRMED = 1   # classify every frame until a track is confirmed
DEBOUNCE_COUNT           = 5    # agreeing reads to *flip* an already-established status
DEBOUNCE_COUNT_INITIAL   = 2    # agreeing reads to confirm the FIRST status (fast acquire)
EMBEDDINGS_PATH          = str(SCRIPT_DIR / "embeddings" / "authorized.pkl")

# ── HUD Colours (BGR) ──
COL_TRACKING  = (0, 220,  80)
COL_NO_BODY   = (0,  60, 220)
COL_RETICLE   = (220, 220,  0)
COL_PERSON    = (0, 200, 255)
COL_CENTROID  = (0, 255, 255)
COL_FACE_BOX  = (0, 220,  80)
COL_TRACK_ID  = (255, 180,  0)
COL_ARMED     = (0, 180, 255)
COL_FIRING    = (0,  40, 255)
COL_AUTHORIZED   = (0, 255, 0)
COL_UNAUTHORIZED = (0, 0, 255)
COL_AUTH_UNKNOWN = (128, 128, 128)
