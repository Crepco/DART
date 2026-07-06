"""Face authorization classifier — InsightFace buffalo_sc + cosine similarity."""

import os
import pickle
import threading

import cv2
import numpy as np

# onnxruntime-gpu's CUDA provider needs the CUDA 12 / cuDNN 9 runtime DLLs
# (cublasLt64_12.dll, cudnn64_9.dll, ...). The torch+cu126 wheel already bundles
# them in torch/lib, but onnxruntime doesn't search there — so without this it
# fails to load onnxruntime_providers_cuda.dll and silently drops to CPU.
# Register that dir on the DLL search path before insightface imports onnxruntime.
# Windows-only (no-op elsewhere).
if hasattr(os, "add_dll_directory"):
    try:
        import torch
        _torch_lib = os.path.join(os.path.dirname(torch.__file__), "lib")
        if os.path.isdir(_torch_lib):
            os.add_dll_directory(_torch_lib)
    except Exception as e:
        print(f"[WARN] Could not register torch CUDA DLL dir for onnxruntime: {e}")

from insightface.app import FaceAnalysis

from config import (SIMILARITY_THRESHOLD, EMBEDDINGS_PATH,
                    ENROLL_MIN_FACE, ENROLL_MIN_DET_SCORE, ENROLL_MIN_SHARPNESS)


class FaceClassifier:
    def __init__(self):
        # Prefer CUDA; onnxruntime silently drops to CPU if the GPU provider is absent.
        self.app = FaceAnalysis(
            name="buffalo_sc",
            providers=["CUDAExecutionProvider", "CPUExecutionProvider"],
        )
        # det_size drives the InsightFace detector's input resolution and is the
        # dominant per-classify CPU cost. 256 is noticeably faster than 320 and
        # still resolves faces inside the (already cropped) person box.
        self.app.prepare(ctx_id=0, det_size=(256, 256))
        self._warn_if_cpu_fallback()

        # classify() runs on the detector's worker thread; extract_embedding()
        # runs on Flask request threads — one session, one user at a time.
        self._session_lock = threading.Lock()

        self.db = {}
        self.reload_db()

    def reload_db(self):
        """(Re)load the authorized-embeddings pkl. Called at init and after the web
        enrollment flow writes a new identity."""
        db = {}
        if os.path.exists(EMBEDDINGS_PATH):
            try:
                with open(EMBEDDINGS_PATH, "rb") as f:
                    db = pickle.load(f)
            except Exception as e:
                print(f"[WARN] Could not load embeddings ({EMBEDDINGS_PATH}): {e}")
                db = {}
        else:
            print(f"[WARN] Embeddings file not found ({EMBEDDINGS_PATH}).")

        self.db = db
        if not self.db:
            print("[WARN] No authorized embeddings loaded — run enroll.py. "
                  "All detected faces will read UNAUTHORIZED.")
        else:
            print(f"[INFO] Loaded {len(self.db)} authorized identities: "
                  f"{', '.join(self.db.keys())}")

    def _warn_if_cpu_fallback(self):
        """Loud, actionable warning when classify() landed on CPU despite a GPU
        being present — almost always the CPU `onnxruntime` (pulled in by
        insightface) shadowing onnxruntime-gpu after `pip install -r requirements.txt`.
        Stays silent on a genuinely CPU-only host."""
        on_gpu = any(m.session.get_providers()[0] == "CUDAExecutionProvider"
                     for m in self.app.models.values())
        if on_gpu:
            print("[INFO] InsightFace running on GPU (CUDAExecutionProvider).")
            return
        try:
            import torch
            gpu_present = torch.cuda.is_available()
        except Exception:
            gpu_present = False
        if not gpu_present:
            return   # no NVIDIA GPU — CPU is expected, nothing to warn about
        print("[WARN] InsightFace fell back to CPU though a CUDA GPU is present.")
        print("[WARN] The CPU 'onnxruntime' (pulled in by insightface) is shadowing "
              "onnxruntime-gpu. Restore GPU with:")
        print("[WARN]   pip uninstall -y onnxruntime onnxruntime-gpu && "
              "pip install onnxruntime-gpu==1.26.0")

    def classify(self, crop) -> str:
        if crop is None or crop.size == 0:
            return "UNKNOWN"

        with self._session_lock:
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

    def extract_embedding(self, frame):
        """Quality-gated embedding of the largest face in `frame`, for enrollment.

        Returns (embedding, "ok") or (None, reason). The gate rejects small, low
        confidence, or blurred faces — a bad capture saved to the pkl would degrade
        recognition for that identity permanently.
        """
        if frame is None or frame.size == 0:
            return None, "empty frame"

        with self._session_lock:
            faces = self.app.get(frame)
        if not faces:
            return None, "no face found"

        face = max(faces, key=lambda f: (f.bbox[2] - f.bbox[0]) * (f.bbox[3] - f.bbox[1]))
        x1, y1, x2, y2 = (int(v) for v in face.bbox)
        if min(x2 - x1, y2 - y1) < ENROLL_MIN_FACE:
            return None, "face too small — move closer"
        if face.det_score < ENROLL_MIN_DET_SCORE:
            return None, "face unclear — face the camera"

        h, w = frame.shape[:2]
        crop = frame[max(0, y1):min(h, y2), max(0, x1):min(w, x2)]
        if crop.size == 0:
            return None, "face out of frame"
        sharpness = cv2.Laplacian(cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY), cv2.CV_64F).var()
        if sharpness < ENROLL_MIN_SHARPNESS:
            return None, "too blurry — hold still"

        return face.normed_embedding, "ok"
