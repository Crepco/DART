# DART — Tracking & Authorization Pipeline

How a camera frame becomes a target decision. Code: `scripts/core/detector.py`,
`scripts/auth/`. Control-law and serial details: [ARCHITECTURE.md](ARCHITECTURE.md),
[SERIAL_PROTOCOL.md](SERIAL_PROTOCOL.md).

## Per-frame flow

```
CameraStream (threaded, self-healing)
  └► YOLODetector thread
       ├► YOLOv8n person detection  ─►  ByteTrack association (bytetrack_dart.yaml)
       │     tracked ids + confs ──► TrackManager (verdicts, debounce, grace)
       │     due tracks ──► _ClassifierWorker ──► InsightFace verdict (async)
       ├► YOLOv8n-face detection  ─►  face boxes
       └► result: target person_box + track_id, all persons, faces, auth statuses
  └► main/runner loop: select_face() aim point ─► PID ─► serial
```

## Person tracking (identity)

- `model.track(conf=TRACK_CONF=0.1, tracker=bytetrack_dart.yaml, persist=True)`.
  The low `conf` matters: ultralytics filters detections **before** the tracker, and
  ByteTrack's second-stage association needs the 0.1–0.45 detections to hold a track
  through confidence dips (motion blur, awkward pose, dim lighting). Boxes below
  `PERSON_CONF` (0.45) maintain identity only — they are never displayed, targeted, or
  classified.
- Frames where the tracker activates nothing (`boxes.id is None`) are treated as empty:
  those results are raw unmatched detections, and inventing ids for them would collide
  with real track ids.
- `TRACK_DEBUG = True` logs id-set changes (`+id(conf)` / `-id`) per frame.

## Authorization (per-track verdict)

- **Classifier** (`auth/classifier.py`): InsightFace `buffalo_sc`, det_size 256×256 (the
  dominant per-classify cost), cosine similarity vs `scripts/embeddings/authorized.pkl`,
  `SIMILARITY_THRESHOLD 0.4`. Small person-crops are upscaled 2× before detection so
  distant faces still resolve. Runs on `_ClassifierWorker` (single-slot, latest job wins)
  so classification never blocks tracking; the active provider and a latency EMA are
  surfaced to the UI (`auth_provider`, `auth_ms`).
- **Debounce** (`auth/track_manager.py`): a new track needs `DEBOUNCE_COUNT_INITIAL` (2)
  agreeing reads for its first verdict (classified every frame until confirmed), then
  `DEBOUNCE_COUNT` (5) agreeing reads to *flip* an established one (re-checked every
  `CLASSIFY_EVERY_N_FRAMES` (10)). UNKNOWN reads ("no usable face this cycle") never
  advance the debounce.
- **Frozen UNAUTHORIZED**: once confirmed, an UNAUTHORIZED verdict is permanent for the
  track's lifetime — motion blur can't churn a known target back to UNKNOWN and reset
  the fire dwell.
- **Verdict grace**: state survives `TRACK_STATE_GRACE_S` (8 s) after the tracker stops
  reporting the id — ByteTrack only outputs *active* tracks, so instant deletion would
  wipe the verdict one missed frame before the same id re-attaches.

## Target & aim-point selection

- Target: only **confirmed-UNAUTHORIZED** persons qualify; none → hold. Several → the
  largest bbox (nearest the camera), a deterministic choice that doesn't oscillate.
- Aim point: `select_face()` picks the face nearest the previous smoothed centroid
  (score = area − 0.4·distance; largest face when no anchor), preventing face hopping.
- The fire gate reads the locked target's *persisted* verdict, so a blurred frame can't
  reset the dwell; with no locked track the verdict is UNKNOWN and fire is inhibited.

## Enrollment (who is AUTHORIZED)

Three ways into `authorized.pkl`, all sharing `save_identity()` (mean of the captured
embeddings, L2-normalized):

| Path | When |
|------|------|
| Web **Authorize Person** (capture) | DART running (live feed frames) or stopped (opens the camera directly). Frames pass a quality gate: face ≥ `ENROLL_MIN_FACE` px, det score ≥ 0.5, sharpness ≥ `ENROLL_MIN_SHARPNESS`; ≥ `ENROLL_MIN_ACCEPTED` usable frames or nothing is saved. |
| Web **Authorize Person** (upload) | Photo files; works in any state. |
| CLI | `python scripts/auth/enroll.py` (webcam) / `python scripts/auth/enroll_from_images.py <folder> <name>`. |

Web enrollment hot-reloads the classifier db and drops confirmed-UNAUTHORIZED tracks back
to UNKNOWN so a just-enrolled person re-classifies within ~2 verdicts (UNKNOWN is never a
target, so the turret holds during the re-check). Duplicate names require an explicit
overwrite. The `/authorize` endpoints carry no auth of their own — the server must stay
bound to `127.0.0.1` (see the security note in the README).
