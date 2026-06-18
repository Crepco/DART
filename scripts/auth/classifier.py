"""Face authorization classifier — InsightFace buffalo_sc + cosine similarity."""

import os
import pickle

import numpy as np
from insightface.app import FaceAnalysis

from config import SIMILARITY_THRESHOLD, EMBEDDINGS_PATH


class FaceClassifier:
    def __init__(self):
        self.app = FaceAnalysis(name="buffalo_sc")
        self.app.prepare(ctx_id=0, det_size=(320, 320))

        self.db = {}
        if os.path.exists(EMBEDDINGS_PATH):
            try:
                with open(EMBEDDINGS_PATH, "rb") as f:
                    self.db = pickle.load(f)
            except Exception as e:
                print(f"[WARN] Could not load embeddings ({EMBEDDINGS_PATH}): {e}")
                self.db = {}
        else:
            print(f"[WARN] Embeddings file not found ({EMBEDDINGS_PATH}).")

        if not self.db:
            print("[WARN] No authorized embeddings loaded — run enroll.py. "
                  "All detected faces will read UNAUTHORIZED.")
        else:
            print(f"[INFO] Loaded {len(self.db)} authorized identities: "
                  f"{', '.join(self.db.keys())}")

    def classify(self, crop) -> str:
        if crop is None or crop.size == 0:
            return "UNKNOWN"

        faces = self.app.get(crop)
        if not faces:
            return "UNKNOWN"

        face = max(faces, key=lambda f: (f.bbox[2] - f.bbox[0]) * (f.bbox[3] - f.bbox[1]))
        emb = face.normed_embedding

        best = -1.0
        for ref in self.db.values():
            score = float(np.dot(emb, ref))
            if score > best:
                best = score

        return "AUTHORIZED" if best >= SIMILARITY_THRESHOLD else "UNAUTHORIZED"
