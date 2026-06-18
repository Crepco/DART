import copy
import math
import os
import threading
import time

import numpy as np
from ultralytics import YOLO

from config import CAM_HEIGHT, CAM_WIDTH, PERSON_CONF, FACE_CONF


class YOLODetector:
    def __init__(self, person_model_path: str, face_model_path: str | None):
        self._lock      = threading.Lock()
        self._frame     = None
        self._new_frame = False
        self._result    = {
            "person_box":  None,
            "track_id":    None,
            "all_persons": [],
            "faces":       [],
        }
        self._stopped = False

        print(f"[INFO] Loading person model: {person_model_path}")
        self._person_model = YOLO(person_model_path)

        self._face_model = None
        if face_model_path and os.path.exists(face_model_path):
            print(f"[INFO] Loading face model: {face_model_path}")
            self._face_model = YOLO(face_model_path)
        else:
            print(f"[WARN] Face model not found ({face_model_path}). Face boxes disabled.")

        dummy = np.zeros((CAM_HEIGHT, CAM_WIDTH, 3), dtype=np.uint8)
        self._person_model.track(dummy, persist=True, verbose=False)
        if self._face_model:
            self._face_model(dummy, verbose=False)

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
                    verbose=False, tracker="bytetrack.yaml",
                )
            except Exception as e:
                print(f"[WARN] Person model error: {e}")
                continue

            all_persons = []
            person_box  = None
            track_id    = None

            r = p_results[0]
            if r.boxes is not None and len(r.boxes):
                ids = (r.boxes.id.int().cpu().tolist()
                       if r.boxes.id is not None else
                       list(range(len(r.boxes))))

                for box, tid in zip(r.boxes.xyxy.cpu().tolist(), ids):
                    x1, y1, x2, y2 = [int(v) for v in box]
                    all_persons.append((x1, y1, x2, y2, tid))

                current_ids = [p[4] for p in all_persons]
                if locked_id not in current_ids:
                    locked_id = current_ids[0]

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
                    f_results = self._face_model(frame, conf=FACE_CONF, verbose=False)
                    fr = f_results[0]
                    if fr.boxes is not None:
                        for box in fr.boxes.xyxy.cpu().tolist():
                            faces.append(tuple(int(v) for v in box))
                except Exception:
                    pass

            with self._lock:
                self._result = {
                    "person_box":  person_box,
                    "track_id":    track_id,
                    "all_persons": all_persons,
                    "faces":       faces,
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
