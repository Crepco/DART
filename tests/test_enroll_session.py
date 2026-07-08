"""/enroll/* wizard session — begin/capture/ping/finish/cancel lifecycle.

Heavy pieces are stubbed (camera open, classifier, _finish_enrollment,
thumbnails), mirroring test_authorize_dispatch: only the session/routing
logic is under test. The directional floor (front/left/right required,
down/up skippable) and the single-session takeover semantics are the
behaviours most worth pinning down.
"""

import pytest

import web.app as webapp
from web.enroll_session import (IDLE_TIMEOUT_S, MAX_ATTEMPTS,
                                EnrollSessionManager)


class FakeClassifier:
    def __init__(self, db=None, reject_with=None):
        self.db = db or {}
        self.reject_with = reject_with

    def extract_embedding(self, frame):
        if self.reject_with:
            return None, self.reject_with
        return ([1.0], "ok") if frame is not None else (None, "no frame")


class FakeCap:
    def __init__(self):
        self.released = False

    def grab(self):
        pass

    def read(self):
        return True, "frame"

    def release(self):
        self.released = True


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
def caps():
    return []


@pytest.fixture
def client(monkeypatch, caps):
    def fake_open(camera):
        cap = FakeCap()
        caps.append(cap)
        return cap

    monkeypatch.setattr(webapp, "_runner", None)
    monkeypatch.setattr(webapp, "_enroll_sessions", EnrollSessionManager())
    monkeypatch.setattr(webapp, "_get_classifier", lambda r: FakeClassifier())
    monkeypatch.setattr(webapp, "_finish_enrollment",
                        lambda clf, name, embs: sorted(["Lateef", name]))
    monkeypatch.setattr(webapp, "_open_enroll_camera", fake_open)
    monkeypatch.setattr(webapp, "_thumb_data_uri", lambda frame: "thumb")
    webapp.app.config["TESTING"] = True
    return webapp.app.test_client()


def begin(client, name="Zed", **extra):
    return client.post("/enroll/begin", data={"name": name, **extra})


def capture(client, sid, pose, **extra):
    return client.post("/enroll/capture",
                       data={"session": sid, "pose": pose, **extra})


def active(sid):
    """The live session object (bypasses touch/expiry side effects)."""
    s = webapp._enroll_sessions._session
    assert s is not None and s.sid == sid
    return s


# ── begin ────────────────────────────────────────────────────────────────────

def test_begin_missing_name_is_400(client):
    assert client.post("/enroll/begin", data={}).status_code == 400


def test_begin_contract(client):
    j = begin(client).get_json()
    assert j["ok"] is True
    assert j["poses"] == ["front", "down", "left", "right", "up"]
    assert j["required_poses"] == ["front", "left", "right"]
    assert (j["sets"], j["per_set"]) == (2, 5)
    assert j["live"] is False        # no runner -> standalone camera held
    assert "cancelled_previous" not in j


def test_begin_duplicate_name_is_409_exists(client, monkeypatch):
    monkeypatch.setattr(webapp, "_get_classifier",
                        lambda r: FakeClassifier(db={"Zed": object()}))
    res = begin(client)
    assert res.status_code == 409
    assert res.get_json()["error"] == "exists"


def test_begin_while_starting_is_409(client, monkeypatch):
    monkeypatch.setattr(webapp, "_runner",
                        FakeRunner(status="LOADING MODELS", has_frames=False))
    res = begin(client)
    assert res.status_code == 409
    assert "starting" in res.get_json()["error"]


def test_begin_camera_unavailable_is_409(client, monkeypatch):
    def boom(camera):
        raise RuntimeError("camera 0 unavailable (in use or not connected)")

    monkeypatch.setattr(webapp, "_open_enroll_camera", boom)
    res = begin(client)
    assert res.status_code == 409
    assert "unavailable" in res.get_json()["error"]


def test_live_runner_session_holds_no_camera(client, monkeypatch, caps):
    monkeypatch.setattr(webapp, "_runner", FakeRunner())
    j = begin(client).get_json()
    assert j["live"] is True
    assert caps == []                # no device opened
    r = capture(client, j["session"], "front")
    assert r.get_json()["ok"] is True


def test_second_begin_cancels_first_and_reports_it(client, caps):
    sid1 = begin(client, name="Ann").get_json()["session"]
    capture(client, sid1, "front")

    j = begin(client, name="Ben").get_json()
    assert j["cancelled_previous"] == {"name": "Ann", "accepted": 1}
    assert caps[0].released is True

    res = capture(client, sid1, "front")
    assert res.status_code == 404
    assert res.get_json()["error"] == "session expired — restart capture"


# ── capture ──────────────────────────────────────────────────────────────────

def test_capture_bad_pose_is_400(client):
    sid = begin(client).get_json()["session"]
    assert capture(client, sid, "backwards").status_code == 400


def test_capture_unknown_session_is_404_with_message(client):
    res = capture(client, "nope1234", "front")
    assert res.status_code == 404
    assert res.get_json()["error"] == "session expired — restart capture"


def test_capture_accepts_and_counts(client):
    sid = begin(client).get_json()["session"]
    j = capture(client, sid, "front").get_json()
    assert j == {"ok": True, "pose": "front", "accepted": 1, "thumb": "thumb"}


def test_duplicate_pose_capture_appends_not_errors(client):
    sid = begin(client).get_json()["session"]
    capture(client, sid, "front")
    j = capture(client, sid, "front").get_json()
    assert j["ok"] is True and j["accepted"] == 2


def test_capture_gate_rejection_reports_reason(client, monkeypatch):
    monkeypatch.setattr(webapp, "_get_classifier",
                        lambda r: FakeClassifier(reject_with="too blurry — hold still"))
    sid = begin(client).get_json()["session"]
    j = capture(client, sid, "front").get_json()
    assert j == {"ok": False, "reason": "too blurry — hold still", "accepted": 0}


def test_capture_attempt_limit_is_409(client):
    sid = begin(client).get_json()["session"]
    active(sid).attempts = MAX_ATTEMPTS
    assert capture(client, sid, "front").status_code == 409


# ── finish: directional floor ────────────────────────────────────────────────

def test_finish_missing_required_poses_is_422_and_session_survives(client):
    sid = begin(client).get_json()["session"]
    for pose in ("front", "down", "up", "front"):     # 4 accepted, no left/right
        capture(client, sid, pose)
    res = client.post("/enroll/finish", data={"session": sid})
    assert res.status_code == 422
    assert res.get_json()["error"] == "missing required poses: left, right"

    # session still alive: retake just the missing poses, then finish
    capture(client, sid, "left")
    capture(client, sid, "right")
    j = client.post("/enroll/finish", data={"session": sid}).get_json()
    assert j["ok"] is True and j["accepted"] == 6
    assert j["identities"] == ["Lateef", "Zed"]


def test_finish_downup_absence_alone_does_not_422(client):
    sid = begin(client).get_json()["session"]
    for pose in ("front", "left", "right", "front"):  # no down/up: skippable
        capture(client, sid, pose)
    j = client.post("/enroll/finish", data={"session": sid}).get_json()
    assert j["ok"] is True


def test_finish_under_min_accepted_is_422(client):
    sid = begin(client).get_json()["session"]
    for pose in ("front", "left", "right"):           # floor met, total too low
        capture(client, sid, pose)
    res = client.post("/enroll/finish", data={"session": sid})
    assert res.status_code == 422
    assert "only 3 usable captures" in res.get_json()["error"]


def test_finish_releases_camera_and_ends_session(client, caps):
    sid = begin(client).get_json()["session"]
    for pose in ("front", "left", "right", "down", "up"):
        capture(client, sid, pose)
    assert client.post("/enroll/finish", data={"session": sid}).status_code == 200
    assert caps[0].released is True
    assert client.post("/enroll/finish",
                       data={"session": sid}).status_code == 404


# ── ping / expiry ────────────────────────────────────────────────────────────

def test_ping_keeps_session_alive_expiry_reaps_it(client, caps):
    sid = begin(client).get_json()["session"]
    s = active(sid)

    s.last_activity -= IDLE_TIMEOUT_S - 1     # nearly idle: ping refreshes it
    assert client.post("/enroll/ping", data={"session": sid}).status_code == 200
    assert capture(client, sid, "front").get_json()["ok"] is True

    s.last_activity -= IDLE_TIMEOUT_S + 1     # past the threshold: reaped
    res = client.post("/enroll/ping", data={"session": sid})
    assert res.status_code == 404
    assert res.get_json()["error"] == "session expired — restart capture"
    assert caps[0].released is True


# ── cancel / start interaction ───────────────────────────────────────────────

def test_cancel_is_idempotent_and_releases_camera(client, caps):
    sid = begin(client).get_json()["session"]
    assert client.post("/enroll/cancel", data={"session": sid}).status_code == 204
    assert caps[0].released is True
    assert client.post("/enroll/cancel", data={"session": sid}).status_code == 204


def test_start_cancels_active_session(client, monkeypatch, caps):
    class StubRunner:
        def __init__(self, camera=None, port=None):
            pass

        def start(self):
            pass

    monkeypatch.setattr(webapp, "_get_runner_cls", lambda: StubRunner)
    sid = begin(client).get_json()["session"]

    assert client.post("/start", data={}).status_code == 302
    assert caps[0].released is True
    assert capture(client, sid, "front").status_code == 404
