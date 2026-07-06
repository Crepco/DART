"""/start idempotency — the regression that caused the 'runner dies at startup' bug.

A second POST /start (double-click, stale tab) must never replace a healthy
runner; only dead (ERROR/STOPPED) runners are replaced. DartRunner is stubbed via
_get_runner_cls(), so no torch/camera is involved.
"""

from concurrent.futures import ThreadPoolExecutor

import pytest

import web.app as webapp


class FakeRunner:
    instances = []

    def __init__(self, camera=None, port=None):
        self.camera = camera
        self.port = port
        self._alive = False
        self._status = "STARTING"
        FakeRunner.instances.append(self)

    def start(self):
        self._alive = True
        self._status = "RUNNING"

    def stop(self, reason="", join_timeout=10.0):
        self._alive = False
        self._status = "STOPPED"

    def is_alive(self):
        return self._alive

    def get_state(self):
        return {"running": self._alive, "status": self._status}

    def get_frame_jpeg(self):
        return None


@pytest.fixture
def client(monkeypatch):
    monkeypatch.setattr(webapp, "_get_runner_cls", lambda: FakeRunner)
    monkeypatch.setattr(webapp, "_runner", None)
    FakeRunner.instances.clear()
    webapp.app.config["TESTING"] = True
    return webapp.app.test_client()


def test_second_start_ignored_while_runner_alive(client):
    assert client.post("/start", data={}).status_code == 302
    assert client.post("/start", data={}).status_code == 302
    assert len(FakeRunner.instances) == 1


def test_concurrent_starts_yield_one_runner(client):
    def hit(_):
        # a client per thread: the werkzeug test client isn't thread-safe
        return webapp.app.test_client().post("/start", data={}).status_code

    with ThreadPoolExecutor(max_workers=8) as ex:
        codes = list(ex.map(hit, range(8)))
    assert all(c == 302 for c in codes)
    assert len(FakeRunner.instances) == 1


def test_stop_then_start_creates_fresh_runner(client):
    client.post("/start", data={})
    client.post("/stop")
    client.post("/start", data={})
    assert len(FakeRunner.instances) == 2


def test_dead_runner_is_replaced(client):
    client.post("/start", data={})
    FakeRunner.instances[0]._alive = False
    FakeRunner.instances[0]._status = "ERROR"
    client.post("/start", data={})
    assert len(FakeRunner.instances) == 2


def test_state_idle_without_runner(client):
    j = client.get("/state").get_json()
    assert j == {"running": False, "status": "IDLE"}
