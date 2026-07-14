import copy
import os
import threading
import time

import numpy as np
import torch
from ultralytics import YOLO

from config import (CAM_HEIGHT, CAM_WIDTH, PERSON_CONF, FACE_CONF,
                    TARGET_HOLD_CONF, TRACK_CONF, TRACKER_CONFIG, TRACK_DEBUG)
from auth import FaceClassifier, TrackManager
from .targeting import AimSelector, resolve_lock, select_face  # noqa: F401 —
#   select_face/AimSelector re-exported: the loops and older imports pull the
#   targeting helpers through core.detector

# Run YOLO on the GPU when available; falls back to CPU transparently.
DEVICE = 0 if torch.cuda.is_available() else "cpu"


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

    @property
    def classifier(self):
        return self._classifier

    def reset_unauthorized_tracks(self) -> int:
        """See TrackManager.reset_unauthorized. Called from Flask request threads
        after enrollment; the status writes are plain attribute assignments, so the
        worst cross-thread effect is one frame of stale verdict."""
        return self._track_manager.reset_unauthorized()

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
        hold_started = None         # monotonic start of the current conf-dip hold
        prev_held = False
        prev_hold_tid = None
        prev_tracked: set = set()   # for TRACK_DEBUG id-change logging

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
                # conf=TRACK_CONF (0.1), not PERSON_CONF: the predictor's conf
                # filters detections BEFORE the tracker, and ByteTrack's second
                # stage needs the low-conf dets to hold a track through dips.
                # Downstream only ever sees boxes >= PERSON_CONF (filter below).
                p_results = self._person_model.track(
                    frame, persist=True, classes=[0], conf=TRACK_CONF,
                    verbose=False, tracker=TRACKER_CONFIG, device=DEVICE,
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
            tracked = {}       # every id ByteTrack reports this frame -> det conf
            boxes_by_id = {}   # ...and its box, incl. sub-PERSON_CONF dips —
                               # the conf-dip hold below targets these
            # boxes.id is None when the tracker activated nothing this frame —
            # the results are then raw unmatched detections (mostly sub-threshold
            # noise at TRACK_CONF). No identity -> nothing to show or track;
            # inventing sequential fallback ids here collides with real track ids.
            if r.boxes is not None and len(r.boxes) and r.boxes.id is not None:
                ids   = r.boxes.id.int().cpu().tolist()
                confs = r.boxes.conf.cpu().tolist()

                for box, tid, conf in zip(r.boxes.xyxy.cpu().tolist(), ids, confs):
                    tracked[tid] = conf
                    x1, y1, x2, y2 = [int(v) for v in box]
                    boxes_by_id[tid] = (x1, y1, x2, y2)
                    # TRACK_CONF..PERSON_CONF dets keep ByteTrack's identity
                    # alive through dips — hidden from HUD/targeting unless the
                    # conf-dip hold re-surfaces the locked one below.
                    if conf < PERSON_CONF:
                        continue
                    all_persons.append((x1, y1, x2, y2, tid))

            if TRACK_DEBUG:
                cur = set(tracked)
                added   = [f"+{tid}({tracked[tid]:.2f})" for tid in sorted(cur - prev_tracked)]
                removed = [f"-{tid}" for tid in sorted(prev_tracked - cur)]
                if added or removed:
                    print(f"[TRACK] {' '.join(added + removed)} active={sorted(cur)}")
                prev_tracked = cur

            # Runs every frame (even with zero detections): any id the tracker
            # still reports counts as seen; the time-based verdict grace only
            # starts once ByteTrack itself loses the track.
            self._track_manager.cleanup(tracked.keys())

            # Apply any finished async classification. Accept it for any id the
            # tracker still knows (even conf-dipped, even with nothing displayed
            # this frame), so a verdict that finished during a dip isn't discarded.
            res = self._worker.poll()
            if res is not None and res[0] in tracked:
                self._track_manager.update_status(res[0], res[1])

            # Tick every displayed track; collect those due for (re)classification.
            due = []
            for (x1, y1, x2, y2, tid) in all_persons:
                self._track_manager.tick(tid)
                if self._track_manager.should_classify(tid):
                    due.append((tid, (x1, y1, x2, y2)))

            # Target selection: only confirmed-UNAUTHORIZED persons are targets
            # (none -> resolve_lock may still HOLD the previous one through a
            # conf dip). With several, lock the largest bbox (nearest the
            # camera) — deterministic, so no target oscillation.
            candidates = [(x1, y1, x2, y2, tid)
                          for (x1, y1, x2, y2, tid) in all_persons
                          if self._track_manager.get_status(tid) == "UNAUTHORIZED"]
            prev_verdict = (self._track_manager.get_status(locked_id)
                            if locked_id is not None else "UNKNOWN")
            locked_id, held, hold_started = resolve_lock(
                locked_id, prev_verdict, candidates, tracked,
                time.monotonic(), hold_started)

            if held:
                # Conf-dip hold: surface the real (sub-PERSON_CONF) box so the
                # HUD bracket, /state n_persons and event timing don't flicker
                # while the turret keeps aiming at it.
                all_persons.append((*boxes_by_id[locked_id], locked_id))

            if TRACK_DEBUG:
                if held and not prev_held:
                    print(f"[TRACK] hold start {locked_id} "
                          f"conf={tracked.get(locked_id, 0.0):.2f}")
                elif prev_held and not held:
                    if locked_id is not None:
                        why = ("returned" if locked_id == prev_hold_tid
                               else "superseded")
                    elif tracked.get(prev_hold_tid, 0.0) >= TARGET_HOLD_CONF:
                        why = "expired"
                    else:
                        why = "lost"
                    print(f"[TRACK] hold end {prev_hold_tid} ({why})")
            prev_held = held
            if held:
                prev_hold_tid = locked_id

            auth_statuses = {tid: self._track_manager.get_status(tid)
                             for tid in (p[4] for p in all_persons)}

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

            # Fire gate reads the locked target's PERSISTED auth status, so a
            # motion-blur frame can't drop it to UNKNOWN and reset the fire
            # dwell. No locked track -> UNKNOWN -> fire inhibited downstream.
            if track_id is not None:
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

