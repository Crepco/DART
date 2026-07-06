"""save_identity round-trip / overwrite semantics, and the enrollment quality gate
(with InsightFace stubbed out — no model download or inference)."""

import pickle
import threading
from types import SimpleNamespace

import numpy as np
import pytest

from auth.enroll import save_identity
from auth.classifier import FaceClassifier
from config import ENROLL_MIN_FACE


# ── save_identity ───────────────────────────────────────────────────────────

def test_save_identity_roundtrip(tmp_path):
    path = str(tmp_path / "auth.pkl")
    names = save_identity("alice", [np.array([1.0, 1.0, 0.0, 0.0])], path=path)
    assert names == ["alice"]

    with open(path, "rb") as f:
        db = pickle.load(f)
    assert np.isclose(np.linalg.norm(db["alice"]), 1.0)   # stored L2-normalized

    names = save_identity("bob", [np.array([0.0, 0.0, 1.0, 0.0])], path=path)
    assert names == ["alice", "bob"]                       # merged, not replaced


def test_save_identity_duplicate_and_overwrite(tmp_path):
    path = str(tmp_path / "auth.pkl")
    save_identity("alice", [np.array([1.0, 0.0])], path=path)

    with pytest.raises(ValueError):
        save_identity("alice", [np.array([0.0, 1.0])], overwrite=False, path=path)

    save_identity("alice", [np.array([0.0, 1.0])], path=path)  # default overwrites
    with open(path, "rb") as f:
        db = pickle.load(f)
    assert db["alice"][1] == pytest.approx(1.0)


def test_save_identity_rejects_empty(tmp_path):
    path = str(tmp_path / "auth.pkl")
    with pytest.raises(ValueError):
        save_identity("alice", [], path=path)
    with pytest.raises(ValueError):
        save_identity("   ", [np.array([1.0, 0.0])], path=path)


# ── extract_embedding quality gate ──────────────────────────────────────────

def make_classifier(faces):
    """FaceClassifier with __init__ skipped and InsightFace faked: `faces` is what
    app.get() returns. Only the gate logic runs."""
    clf = FaceClassifier.__new__(FaceClassifier)
    clf._session_lock = threading.Lock()
    clf.app = SimpleNamespace(get=lambda img: faces)
    return clf


def make_face(x1, y1, x2, y2, det_score=0.9):
    return SimpleNamespace(bbox=np.array([x1, y1, x2, y2], dtype=float),
                           det_score=det_score,
                           normed_embedding=np.ones(4) / 2.0)


def sharp_frame():
    rng = np.random.default_rng(7)   # high-frequency noise -> high Laplacian var
    return rng.integers(0, 255, (480, 640, 3), dtype=np.uint8)


def test_gate_rejects_no_face():
    emb, why = make_classifier([]).extract_embedding(sharp_frame())
    assert emb is None and "no face" in why


def test_gate_rejects_small_face():
    small = ENROLL_MIN_FACE - 10
    clf = make_classifier([make_face(10, 10, 10 + small, 10 + small)])
    emb, why = clf.extract_embedding(sharp_frame())
    assert emb is None and "small" in why


def test_gate_rejects_low_confidence():
    clf = make_classifier([make_face(10, 10, 200, 200, det_score=0.2)])
    emb, why = clf.extract_embedding(sharp_frame())
    assert emb is None and "unclear" in why


def test_gate_rejects_blurred_face():
    clf = make_classifier([make_face(10, 10, 200, 200)])
    flat = np.full((480, 640, 3), 128, dtype=np.uint8)   # zero Laplacian variance
    emb, why = clf.extract_embedding(flat)
    assert emb is None and "blur" in why


def test_gate_accepts_good_face_and_picks_largest():
    big, small = make_face(10, 10, 300, 300), make_face(400, 10, 460, 70)
    clf = make_classifier([small, big])
    emb, why = clf.extract_embedding(sharp_frame())
    assert why == "ok"
    assert np.array_equal(emb, big.normed_embedding)


def test_gate_rejects_empty_frame():
    emb, why = make_classifier([]).extract_embedding(None)
    assert emb is None and "empty" in why
