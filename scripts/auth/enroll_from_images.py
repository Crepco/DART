"""Enroll a person from a folder of images (no webcam).

Run:  python scripts/auth/enroll_from_images.py <folder> <name>
"""

import sys
import pathlib

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))

import os
import argparse
import pickle

import cv2
import numpy as np
from insightface.app import FaceAnalysis

from config import CAM_INDEX, EMBEDDINGS_PATH

IMAGE_EXTS = (".jpg", ".jpeg", ".png")


def enroll_from_images(folder: str, name: str):
    name = name.strip()
    if not name:
        print("[WARN] No name given — aborting.")
        return

    if not os.path.isdir(folder):
        print(f"[ERROR] Not a folder: {folder}")
        return

    images = [os.path.join(folder, f) for f in sorted(os.listdir(folder))
              if f.lower().endswith(IMAGE_EXTS)]
    if not images:
        print(f"[WARN] No jpg/png images found in {folder}")
        return

    app = FaceAnalysis(name="buffalo_sc")
    app.prepare(ctx_id=0, det_size=(640, 640))

    embeddings = []
    for path in images:
        img = cv2.imread(path)
        if img is None:
            print(f"[WARN] Could not read image: {path}")
            continue

        faces = app.get(img)
        if faces:
            face = max(faces, key=lambda f: (f.bbox[2] - f.bbox[0]) * (f.bbox[3] - f.bbox[1]))
            embeddings.append(face.normed_embedding)
            print(f"[OK] {os.path.basename(path)}")
        else:
            print(f"[WARN] No face detected — skipping {os.path.basename(path)}")

    if not embeddings:
        print("[WARN] No embeddings collected — nothing saved.")
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

    print(f"[INFO] Enrolled '{name}' from {len(embeddings)}/{len(images)} images -> {EMBEDDINGS_PATH}")
    print(f"[INFO] Database now holds {len(db)} identities: {', '.join(db.keys())}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Enroll a person from a folder of images.")
    parser.add_argument("folder", type=str, help="Path to folder of jpg/png images")
    parser.add_argument("name",   type=str, help="Person name to enroll")
    args = parser.parse_args()
    enroll_from_images(args.folder, args.name)
