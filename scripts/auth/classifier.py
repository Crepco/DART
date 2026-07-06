"""Face authorization classifier — InsightFace buffalo_sc + cosine similarity."""

import os
import pickle
import threading
import time

import cv2
import numpy as np

# onnxruntime-gpu needs the CUDA/cuDNN DLLs that the torch+cu126 wheel bundles in
# torch/lib, but doesn't search there — register that dir BEFORE insightface
# imports onnxruntime, or it silently drops to CPU. Windows-only (no-op elsewhere).
if hasattr(os, "add_dll_directory"):
    try:
        import torch
        _torch_lib = os.path.join(os.path.dirname(torch.__file__), "lib")
        if os.path.isdir(_torch_lib):
            os.add_dll_directory(_torch_lib)
    except Exception as e:
        print(f"[WARN] Could not register torch CUDA DLL dir for onnxruntime: {e}")

from insightface.app import FaceAnalysis

from config import (SIMILARITY_THRESHOLD, EMBEDDINGS_PATH, CAM_WIDTH, CAM_HEIGHT,
                    AUTH_MIN_CROP, AUTH_UPSCALE,
                    ENROLL_MIN_FACE, ENROLL_MIN_DET_SCORE, ENROLL_MIN_SHARPNESS)

SLOW_CLASSIFY_MS = 200.0   # per-verdict latency worth a console warning
SLOW_LOG_EVERY_S = 5.0     # ...but rate-limit the warning


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
        self.provider = "CPU"          # set properly by _detect_provider
        self._detect_provider()

        # classify() runs on the detector's worker thread; extract_embedding()
        # runs on Flask request threads — one session, one user at a time.
        self._session_lock = threading.Lock()

        self.classify_ms: float | None = None   # EMA of per-verdict latency
        self._last_slow_log = 0.0

        self.db = {}
        self._ref_mat = None           # (N, 512) stacked embeddings for matching
        self.reload_db()

        # Warm-up: the first ONNX run pays session/graph init (can be ~1s); take
        # that hit here rather than on the first real verdict. Same trick as
        # YOLODetector's dummy inference.
        t0 = time.perf_counter()
        self.app.get(np.zeros((CAM_HEIGHT, CAM_WIDTH, 3), dtype=np.uint8))
        print(f"[INFO] InsightFace warm-up: {(time.perf_counter() - t0) * 1000:.0f}ms "
              f"(provider={self.provider})")

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
        # Stack refs into one matrix: matching is a single matvec instead of a
        # Python loop, and stays flat as identities are added.
        self._ref_mat = (np.stack(list(db.values())).astype(np.float32)
                         if db else None)
        if not self.db:
            print("[WARN] No authorized embeddings loaded — run enroll.py. "
                  "All detected faces will read UNAUTHORIZED.")
        else:
            print(f"[INFO] Loaded {len(self.db)} authorized identities: "
                  f"{', '.join(self.db.keys())}")

    def _detect_provider(self):
        """Set self.provider so the UI can show what classify() actually runs on.
        "CPU (degraded)" — CPU inference despite a CUDA GPU being present — is
        almost always the CPU `onnxruntime` (pulled in by insightface) shadowing
        onnxruntime-gpu after `pip install -r requirements.txt`; it costs 5-10x
        per verdict, dwarfing any other latency fix, so it must never be silent."""
        on_gpu = any(m.session.get_providers()[0] == "CUDAExecutionProvider"
                     for m in self.app.models.values())
        if on_gpu:
            self.provider = "CUDA"
            print("[INFO] InsightFace running on GPU (CUDAExecutionProvider).")
            return
        try:
            import torch
            gpu_present = torch.cuda.is_available()
        except Exception:
            gpu_present = False
        if not gpu_present:
            self.provider = "CPU"   # no NVIDIA GPU — CPU is expected
            return
        self.provider = "CPU (degraded)"
        print("[WARN] InsightFace fell back to CPU though a CUDA GPU is present.")
        print("[WARN] The CPU 'onnxruntime' (pulled in by insightface) is shadowing "
              "onnxruntime-gpu. Restore GPU with:")
        print("[WARN]   pip uninstall -y onnxruntime onnxruntime-gpu && "
              "pip install onnxruntime-gpu==1.26.0")

    def classify(self, crop) -> str:
        if crop is None or crop.size == 0:
            return "UNKNOWN"
        t0 = time.perf_counter()
        verdict = self._classify(crop)
        ms = (time.perf_counter() - t0) * 1000.0
        self.classify_ms = (ms if self.classify_ms is None
                            else 0.8 * self.classify_ms + 0.2 * ms)
        if ms > SLOW_CLASSIFY_MS and time.monotonic() - self._last_slow_log > SLOW_LOG_EVERY_S:
            self._last_slow_log = time.monotonic()
            print(f"[WARN] Slow face classify: {ms:.0f}ms (provider={self.provider})")
        return verdict

    def _classify(self, crop) -> str:
        # A face that's tiny inside the person crop slips past the 256px detector
        # and returns UNKNOWN — which never advances the verdict debounce, so a
        # distant person would stay verdict-less indefinitely. Upscale small crops.
        if min(crop.shape[:2]) < AUTH_MIN_CROP:
            crop = cv2.resize(crop, None, fx=AUTH_UPSCALE, fy=AUTH_UPSCALE,
                              interpolation=cv2.INTER_LINEAR)

        with self._session_lock:
            faces = self.app.get(crop)
        if not faces:
            return "UNKNOWN"

        face = max(faces, key=lambda f: (f.bbox[2] - f.bbox[0]) * (f.bbox[3] - f.bbox[1]))
        emb = face.normed_embedding

        ref_mat = self._ref_mat   # local ref: reload_db() may swap it mid-call
        best = float((ref_mat @ emb).max()) if ref_mat is not None else -1.0

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
