"""DART turret — central configuration. All constants live here."""

import pathlib


# ── Camera ──
CAM_INDEX  = 0
CAM_WIDTH  = 640
CAM_HEIGHT = 480
CAM_FPS    = 15

# ── Models ──
SCRIPT_DIR   = pathlib.Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parent
PERSON_MODEL = str(PROJECT_ROOT / "models" / "yolov8n.pt")
FACE_MODEL   = str(PROJECT_ROOT / "models" / "yolov8n-face.pt")
LABELS_FILE  = str(PROJECT_ROOT / "labels.txt")
PERSON_CONF  = 0.50
FACE_CONF    = 0.45

# ── Servo ──
STOP_PAN    = 90
STOP_TILT   = 90
MAX_SPEED   = 25
INVERT_PAN  = -1
INVERT_TILT = -1

# ── Trigger (MG90) — angles live on the Arduino; mirrored here for parity ──
TRIGGER_REST_ANGLE = 60
TRIGGER_FIRE_ANGLE = 150
LOCK_ON_RADIUS      = 40    # px from centre to engage
LOCK_RELEASE_RADIUS = 70    # px from centre to disengage (hysteresis)
FIRE_DWELL_FRAMES   = 3     # consecutive on-target frames before firing

# ── Dead Bands ──
OUTPUT_DEADBAND = 4
INPUT_DEADBAND  = 30   # px stop-zone — must be < LOCK_ON_RADIUS so it settles inside fire zone

# ── PID Gains (tuning guide: docs/ARCHITECTURE.md) ──
PAN_KP,  PAN_KI,  PAN_KD  = 0.04, 0.001, 0.10
TILT_KP, TILT_KI, TILT_KD = 0.04, 0.001, 0.09
INTEGRAL_CLAMP = 10.0
D_SMOOTH = 0.70     # derivative low-pass (weight on previous) — stops D-term amplifying detection noise

# ── Smoothing ──
SMOOTH     = 0.90   # centroid EMA (weight on previous value)
CMD_SMOOTH = 0.80   # servo command EMA — new-sample weight = 1-CMD_SMOOTH ≈ 0.2

# ── Slew Rate (servo-offset units / second, dt-scaled) ──
MAX_CMD_CHANGE_PER_SEC = 25.0

# ── Serial ──
PORT          = "COM3"
BAUD_RATE     = 115200
SEND_HZ       = 30
SEND_INTERVAL = 1.0 / SEND_HZ
HEARTBEAT_INTERVAL = 0.4

# ── Auth ──
SIMILARITY_THRESHOLD     = 0.4
CLASSIFY_EVERY_N_FRAMES  = 10
DEBOUNCE_COUNT           = 5

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
