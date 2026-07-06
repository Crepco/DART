"""/authorize dispatch — enrollment must work with DART running, starting, or
stopped (the always-available Authorize Person requirement).

Heavy pieces are stubbed (_standalone_frames, _runner_frames, _get_classifier,
_finish_enrollment), so no camera or InsightFace is involved — only the routing
under test."""

import pytest

import web.app as webapp


class FakeClassifier:
    def __init__(self, db=None):
        self.db = db or {}

    def extract_embedding(self, frame):
        return ([1.0], "ok") if frame is not None else (None, "no frame")


class FakeRunner:
    def __init__(self, status="RUNNING", has_frames=True):
        self._status = status
        self._has_frames = has_frames

    def is_alive(self):
        return True

    def get_state(self):
        return {"running": True, "status": self._status}

    def get_raw_frame(self):
        return "frame" if self._has_frames else None


@pytest.fixture
def client(monkeypatch):
    monkeypatch.setattr(webapp, "_runner", None)
    monkeypatch.setattr(webapp, "_get_classifier", lambda r: FakeClassifier())
    monkeypatch.setattr(webapp, "_finish_enrollment",
                        lambda clf, name, embs: sorted(["Lateef", name]))
    webapp.app.config["TESTING"] = True
    return webapp.app.test_client()


def test_missing_name_is_400(client):
    assert client.post("/authorize", data={}).status_code == 400


def test_stopped_dart_uses_standalone_capture(client, monkeypatch):
    calls = {}

    def fake_standalone(camera, n, interval):
        calls["camera"] = camera
        return ["frame"] * n

    monkeypatch.setattr(webapp, "_standalone_frames", fake_standalone)
    j = client.post("/authorize", data={"name": "Zed", "camera": "1"}).get_json()
    assert j["ok"] is True
    assert calls["camera"] == 1        # landing picker's selection honoured


def test_standalone_defaults_to_config_camera(client, monkeypatch):
    from config import CAM_INDEX
    calls = {}
    monkeypatch.setattr(webapp, "_standalone_frames",
                        lambda camera, n, i: calls.setdefault("camera", camera) is None
                        or ["frame"] * n)
    client.post("/authorize", data={"name": "Zed"})
    assert calls["camera"] == CAM_INDEX


def test_starting_runner_is_409_never_fights_for_camera(client, monkeypatch):
    monkeypatch.setattr(webapp, "_runner",
                        FakeRunner(status="LOADING MODELS", has_frames=False))
    res = client.post("/authorize", data={"name": "Zed"})
    assert res.status_code == 409
    assert "starting" in res.get_json()["error"]


def test_running_dart_uses_runner_frames(client, monkeypatch):
    r = FakeRunner()
    monkeypatch.setattr(webapp, "_runner", r)
    seen = {}

    def fake_runner_frames(rr, n, interval):
        seen["runner"] = rr
        return ["frame"] * n

    monkeypatch.setattr(webapp, "_runner_frames", fake_runner_frames)
    j = client.post("/authorize", data={"name": "Zed"}).get_json()
    assert j["ok"] is True
    assert seen["runner"] is r


def test_duplicate_name_is_409_exists(client, monkeypatch):
    monkeypatch.setattr(webapp, "_get_classifier",
                        lambda r: FakeClassifier(db={"Zed": object()}))
    res = client.post("/authorize", data={"name": "Zed"})
    assert res.status_code == 409
    assert res.get_json()["error"] == "exists"


def test_camera_unavailable_is_clean_409(client, monkeypatch):
    def boom(camera, n, interval):
        raise RuntimeError("camera 0 unavailable (in use or not connected)")

    monkeypatch.setattr(webapp, "_standalone_frames", boom)
    res = client.post("/authorize", data={"name": "Zed"})
    assert res.status_code == 409
    assert "unavailable" in res.get_json()["error"]


def test_cameras_serves_cache_while_probe_lock_busy(client, monkeypatch):
    monkeypatch.setattr(webapp, "_probe_cache", [0, 2])
    assert webapp._probe_lock.acquire(blocking=False)
    try:   # lock held = standalone capture in flight; /cameras must not stall
        j = client.get("/cameras").get_json()
        assert j["cameras"] == [0, 2]
    finally:
        webapp._probe_lock.release()
