import copy
import math
import os
import threading
import time

import numpy as np
import torch
from ultralytics import YOLO

from config import CAM_HEIGHT, CAM_WIDTH, PERSON_CONF, FACE_CONF
from auth import FaceClassifier, TrackManager

# Run YOLO on the GPU when available; falls back to CPU transparently.
DEVICE = 0 if torch.cuda.is_available() else "cpu"


def _iou(boxA, boxB) -> float:
    """Intersection-over-union of two (x1, y1, x2, y2) boxes."""
    ax1, ay1, ax2, ay2 = boxA
    bx1, by1, bx2, by2 = boxB
    ix1, iy1 = max(ax1, bx1), max(ay1, by1)
    ix2, iy2 = min(ax2, bx2), min(ay2, by2)
    iw, ih = max(0, ix2 - ix1), max(0, iy2 - iy1)
    inter = iw * ih
    if inter == 0:
        return 0.0
    areaA = max(0, ax2 - ax1) * max(0, ay2 - ay1)
    areaB = max(0, bx2 - bx1) * max(0, by2 - by1)
    union = areaA + areaB - inter
    return inter / union if union > 0 else 0.0


class _ClassifierWorker:
    """Runs FaceClassifier.classify() off the detection thread so heavy CPU
    inference never blocks YOLO tracking. Single-slot mailbox, latest-job-wins."""

    def __init__(self, classifier):
        self._classifier = classifier
        self._lock    = threading.Lock()
        self._pending = None   # (track_id, crop)
        self._result  = None   # (track_id, status)
        self._busy    = False
        self._stopped = False
        self._thread  = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def submit(self, track_id, crop):
        with self._lock:
            self._pending = (track_id, crop)
            self._busy    = True

    def is_idle(self):
        with self._lock:
            return not self._busy

    def poll(self):
        with self._lock:
            r, self._result = self._result, None
            return r

    def stop(self):
        self._stopped = True
        self._thread.join()

    def _run(self):
        while not self._stopped:
            with self._lock:
                job, self._pending = self._pending, None
            if job is None:
                time.sleep(0.005)
                continue
            track_id, crop = job
            status = self._classifier.classify(crop)
            with self._lock:
                self._result = (track_id, status)
                self._busy   = False


class YOLODetector:
    def __init__(self, person_model_path: str, face_model_path: str | None):
        self._lock      = threading.Lock()
        self._frame     = None
        self._new_frame = False
        self._result    = {
            "person_box":   None,
            "track_id":     None,
            "all_persons":  [],
            "faces":        [],
            "auth_statuses": {},
            "locked_face_auth": "UNKNOWN",
        }
        self._stopped = False

        print(f"[INFO] YOLO device: {DEVICE}")
        print(f"[INFO] Loading person model: {person_model_path}")
        self._person_model = YOLO(person_model_path)

        self._face_model = None
        if face_model_path and os.path.exists(face_model_path):
            print(f"[INFO] Loading face model: {face_model_path}")
            self._face_model = YOLO(face_model_path)
        else:
            print(f"[WARN] Face model not found ({face_model_path}). Face boxes disabled.")

        dummy = np.zeros((CAM_HEIGHT, CAM_WIDTH, 3), dtype=np.uint8)
        self._person_model.track(dummy, persist=True, verbose=False, device=DEVICE)
        if self._face_model:
            self._face_model(dummy, verbose=False, device=DEVICE)

        self._classifier    = FaceClassifier()
        self._track_manager = TrackManager()
        self._worker        = _ClassifierWorker(self._classifier)

        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def submit(self, frame):
        with self._lock:
            self._frame     = frame
            self._new_frame = True

    def get_result(self):
        with self._lock:
            return copy.deepcopy(self._result)

    def stop(self):
        self._stopped = True
        self._thread.join()
        self._worker.stop()

    def _run(self):
        locked_id = None

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
                p_results = self._person_model.track(
                    frame, persist=True, classes=[0], conf=PERSON_CONF,
                    verbose=False, tracker="bytetrack.yaml", device=DEVICE,
                )
            except Exception as e:
                print(f"[WARN] Person model error: {e}")
                continue

            all_persons = []
            person_box  = None
            track_id    = None
            auth_statuses = {}
            locked_face_auth = "UNKNOWN"

            h_f, w_f = frame.shape[:2]

            r = p_results[0]
            if r.boxes is not None and len(r.boxes):
                ids = (r.boxes.id.int().cpu().tolist()
                       if r.boxes.id is not None else
                       list(range(len(r.boxes))))

                for box, tid in zip(r.boxes.xyxy.cpu().tolist(), ids):
                    x1, y1, x2, y2 = [int(v) for v in box]
                    all_persons.append((x1, y1, x2, y2, tid))

                current_ids = [p[4] for p in all_persons]

                self._track_manager.cleanup(current_ids)

                # Apply any finished async classification (skip departed tracks).
                res = self._worker.poll()
                if res is not None and res[0] in current_ids:
                    self._track_manager.update_status(res[0], res[1])

                # Tick every track; collect those due for (re)classification.
                due = []
                for (x1, y1, x2, y2, tid) in all_persons:
                    self._track_manager.tick(tid)
                    if self._track_manager.should_classify(tid):
                        due.append((tid, (x1, y1, x2, y2)))

                auth_statuses = {tid: self._track_manager.get_status(tid)
                                 for tid in current_ids}

                # Sticky target acquisition: hold the current threat; otherwise
                # prefer UNAUTHORIZED, fall back to UNKNOWN, never lock AUTHORIZED.
                unauthorized_ids = [tid for tid in current_ids
                                    if self._track_manager.get_status(tid) == "UNAUTHORIZED"]
                unknown_ids = [tid for tid in current_ids
                               if self._track_manager.get_status(tid) == "UNKNOWN"]

                keep_current = (locked_id in current_ids
                                and self._track_manager.get_status(locked_id) != "AUTHORIZED")
                if keep_current:
                    pass                          # hold the current threat
                elif unauthorized_ids:
                    locked_id = unauthorized_ids[0]
                elif unknown_ids:
                    locked_id = unknown_ids[0]
                else:
                    locked_id = current_ids[0]    # only AUTHORIZED present — no threat

                # Hand one crop to the worker if it's free — prioritise the target.
                if due and self._worker.is_idle():
                    tid, (x1, y1, x2, y2) = next((d for d in due if d[0] == locked_id), due[0])
                    x1c, y1c = max(0, x1), max(0, y1)
                    x2c, y2c = min(w_f, x2), min(h_f, y2)
                    crop = frame[y1c:y2c, x1c:x2c].copy()
                    self._worker.submit(tid, crop)

                for (x1, y1, x2, y2, tid) in all_persons:
                    if tid == locked_id:
                        person_box = (x1, y1, x2, y2)
                        track_id   = tid
                        break
            else:
                locked_id = None

            faces = []
            if self._face_model is not None:
                try:
                    f_results = self._face_model(frame, conf=FACE_CONF, verbose=False, device=DEVICE)
                    fr = f_results[0]
                    if fr.boxes is not None:
                        for box in fr.boxes.xyxy.cpu().tolist():
                            faces.append(tuple(int(v) for v in box))
                except Exception:
                    pass

            # Link the locked person to the face we'd aim at: only a face that
            # actually overlaps the locked person's box counts. Fire is therefore
            # gated on the *target's* auth status (no overlapping face → UNKNOWN
            # → fire inhibited downstream).
            if person_box is not None and track_id is not None and faces:
                best_face = max(faces, key=lambda f: _iou(f, person_box))
                if _iou(best_face, person_box) > 0.0:
                    locked_face_auth = self._track_manager.get_status(track_id)

            with self._lock:
                self._result = {
                    "person_box":   person_box,
                    "track_id":     track_id,
                    "all_persons":  all_persons,
                    "faces":        faces,
                    "auth_statuses": auth_statuses,
                    "locked_face_auth": locked_face_auth,
                }


def select_face(faces: list, prev_cx, prev_cy):
    """Pick the face nearest the previous anchor (area minus distance). If no
    anchor yet, pick the largest face. Prevents frame-to-frame target hopping."""
    if not faces:
        return None
    if prev_cx is None:
        return max(faces, key=lambda f: (f[2] - f[0]) * (f[3] - f[1]))

    def score(f):
        x1, y1, x2, y2 = f
        cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
        area = (x2 - x1) * (y2 - y1)
        dist = math.hypot(cx - prev_cx, cy - prev_cy)
        return area - dist * 0.4

    return max(faces, key=score)
