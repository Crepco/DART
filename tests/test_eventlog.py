"""EventLog ring buffer — the System Log window's data source.

Covers the polling contract (`since` cursor semantics), seq monotonicity
across eviction, and the /logs endpoint shape.
"""

import pytest

import web.app as webapp
from web.eventlog import EventLog


def test_seq_is_monotonic_and_since_filters():
    log = EventLog()
    for i in range(5):
        log.emit("info", "T", f"event {i}", echo=False)
    events, latest = log.since(0)
    assert [e["seq"] for e in events] == [1, 2, 3, 4, 5]
    assert latest == 5

    events, latest = log.since(3)
    assert [e["msg"] for e in events] == ["event 3", "event 4"]
    assert latest == 5


def test_since_latest_returns_empty_but_keeps_cursor():
    log = EventLog()
    log.emit("info", "T", "only", echo=False)
    events, latest = log.since(1)
    assert events == []
    assert latest == 1   # client keeps polling with the same cursor


def test_eviction_keeps_seq_and_latest_correct():
    log = EventLog(maxlen=3)
    for i in range(10):
        log.emit("info", "T", f"e{i}", echo=False)
    events, latest = log.since(0)
    assert latest == 10
    assert [e["seq"] for e in events] == [8, 9, 10]   # oldest 7 evicted
    # a cursor pointing into the evicted range still only gets what survives
    events, _ = log.since(5)
    assert [e["seq"] for e in events] == [8, 9, 10]


def test_event_shape_and_levels():
    log = EventLog()
    log.emit("warn", "AUTH", "Track 3: UNKNOWN -> UNAUTHORIZED", echo=False)
    (e,), _ = log.since(0)
    assert e["level"] == "warn"
    assert e["tag"] == "AUTH"
    assert e["msg"] == "Track 3: UNKNOWN -> UNAUTHORIZED"
    assert isinstance(e["ts"], float)


@pytest.fixture
def client(monkeypatch):
    monkeypatch.setattr(webapp, "_runner", None)
    webapp.app.config["TESTING"] = True
    return webapp.app.test_client()


def test_logs_endpoint_contract(client, monkeypatch):
    fresh = EventLog()
    monkeypatch.setattr(webapp, "event_log", fresh)
    fresh.emit("info", "SYS", "hello", echo=False)
    fresh.emit("error", "SYS", "boom", echo=False)

    j = client.get("/logs").get_json()
    assert j["latest"] == 2
    assert [e["msg"] for e in j["events"]] == ["hello", "boom"]

    j = client.get("/logs?since=2").get_json()
    assert j["events"] == [] and j["latest"] == 2
