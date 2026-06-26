"""Enrollment CLI — capture webcam photos and store an averaged ArcFace embedding.

Run:  python scripts/auth/enroll.py   (or  python -m auth.enroll  from scripts/)
"""

import sys
import pathlib

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))

import os
import pickle

import cv2
import numpy as np
from insightface.app import FaceAnalysis

from config import CAM_INDEX, CAM_BACKEND, EMBEDDINGS_PATH

MAX_CAPTURES = 10


def enroll():
    app = FaceAnalysis(name="buffalo_sc")
    app.prepare(ctx_id=0, det_size=(640, 640))

    name = input("Enter person name: ").strip()
    if not name:
        print("[WARN] No name given — aborting.")
        return

    cap = cv2.VideoCapture(CAM_INDEX, CAM_BACKEND)
    if not cap.isOpened():
        print(f"[ERROR] Camera {CAM_INDEX} not available.")
        return

    embeddings = []
    print("[INFO] SPACE = capture, Q = finish")

    while True:
        grabbed, frame = cap.read()
        if not grabbed or frame is None:
            continue

        view = frame.copy()
        cv2.putText(view, f"SPACE=capture ({len(embeddings)}/{MAX_CAPTURES})   Q=finish",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2, cv2.LINE_AA)
        cv2.imshow("DART Enroll", view)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        if key == ord(" "):
            faces = app.get(frame)
            if faces:
                face = max(faces, key=lambda f: (f.bbox[2] - f.bbox[0]) * (f.bbox[3] - f.bbox[1]))
                embeddings.append(face.normed_embedding)
                print(f"[OK] captured {len(embeddings)}/{MAX_CAPTURES}")
                if len(embeddings) >= MAX_CAPTURES:
                    break
            else:
                print("[WARN] No face detected — try again.")

    cap.release()
    cv2.destroyAllWindows()

    if not embeddings:
        print("[WARN] No embeddings captured — nothing saved.")
        return

    mean = np.mean(embeddings, axis=0)
    mean = mean / np.linalg.norm(mean)

    db = {}
    if os.path.exists(EMBEDDINGS_PATH):
        try:
            with open(EMBEDDINGS_PATH, "rb") as f:
                db = pickle.load(f)
        except Exception as e:
            print(f"[WARN] Could not read existing embeddings: {e}")
            db = {}

    db[name] = mean

    os.makedirs(os.path.dirname(EMBEDDINGS_PATH), exist_ok=True)
    with open(EMBEDDINGS_PATH, "wb") as f:
        pickle.dump(db, f)

    print(f"[INFO] Enrolled '{name}' from {len(embeddings)} photos -> {EMBEDDINGS_PATH}")
    print(f"[INFO] Database now holds {len(db)} identities: {', '.join(db.keys())}")


if __name__ == "__main__":
    enroll()
