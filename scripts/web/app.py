"""DART web front-end (Flask).

Landing page lets you pick a camera + Arduino serial port, then launches the DART security
turret: it tracks everyone but only fires at UNAUTHORIZED locked targets (auth gate).

The heavy DART runner (camera + YOLO + torch) is imported lazily on /start so the landing
page loads instantly. One runner exists at a time, guarded by a lock.

Run it:  python scripts/run_web.py     (or, from the scripts/ dir:  python -m web.app)
"""

from __future__ import annotations

import threading
import time

from flask import (Flask, Response, jsonify, redirect, render_template,
                   request, url_for)

app = Flask(__name__)
# Re-read templates from disk per request and don't let the browser cache static
# assets, so editing the HTML/CSS/JS shows up on a refresh without restarting Python.
app.config["TEMPLATES_AUTO_RELOAD"] = True
app.config["SEND_FILE_MAX_AGE_DEFAULT"] = 0

_runner = None
_runner_lock = threading.Lock()
_probe_lock = threading.Lock()   # serialize device scans — concurrent probes fight
                                 # over the same DSHOW devices

# Statuses during which the runner hasn't finished claiming its camera yet.
_STARTING_STATUSES = ("STARTING", "LOADING MODELS")
# Statuses meaning the runner is done and safe to replace.
_DEAD_STATUSES = ("ERROR", "STOPPED")


def _get_runner_cls():
    """Lazy import: torch/YOLO load only when a runner actually starts. A separate
    function so tests can stub the heavyweight class."""
    from .runner import DartRunner   # noqa: PLC0415
    return DartRunner


def _probe_cameras(skip=None):
    """Working camera indices (see core.camera.probe_cameras). `skip` is the index a
    running DART already holds — reported as available without re-opening it. The
    import stays local so the landing page loads without cv2."""
    from core.camera import probe_cameras
    with _probe_lock:
        return probe_cameras(skip=skip)


def _probe_ports():
    """Best-effort list of serial ports the Arduino might be on. Returns dicts of
    {device, desc} (e.g. {"COM5", "Arduino Uno (COM5)"}) so the UI can show a hint."""
    try:
        from serial.tools import list_ports
    except Exception:
        return []
    out = []
    for p in list_ports.comports():
        out.append({"device": p.device, "desc": p.description or p.device})
    out.sort(key=lambda d: d["device"])
    return out


@app.route("/")
def index():
    return render_template("index.html")


@app.route("/start", methods=["POST"])
def start():
    global _runner
    camera = request.form.get("camera", type=int)   # None -> runner uses config CAM_INDEX
    port = request.form.get("port") or None          # None/"" -> runner uses config PORT

    # Heavy import outside the lock (Python's import machinery serializes it), so
    # /state and /cameras aren't blocked for the 10-20s a first import can take.
    runner_cls = _get_runner_cls()

    # Check-and-create is atomic under the lock: two near-simultaneous POSTs must
    # never both decide to create/replace. A second /start during model load used
    # to kill the healthy loading runner — the "instant stop after startup" bug.
    with _runner_lock:
        r = _runner
        if r is not None:
            status = r.get_state().get("status")
            if r.is_alive() and status not in _DEAD_STATUSES:
                print(f"[INFO] /start ignored — runner already active (status={status}).")
                return redirect(url_for("run_view"))
            print(f"[INFO] Replacing dead runner (status={status}).")
            r.stop(reason="/start replacing dead runner")
        _runner = runner_cls(camera=camera, port=port)
        _runner.start()
    return redirect(url_for("run_view"))


@app.route("/cameras")
def cameras():
    with _runner_lock:
        r = _runner
    cur = r.camera if r is not None else None
    # While a runner is still claiming its camera, don't open ANY device — a probe
    # holding the index mid-startup makes the runner's own open fail.
    if r is not None and r.get_state().get("status") in _STARTING_STATUSES:
        return jsonify({"cameras": [cur], "current": cur})
    idxs = _probe_cameras(skip=cur)
    if cur is not None and cur not in idxs:
        idxs = sorted(set(idxs + [cur]))
    return jsonify({"cameras": idxs, "current": cur})


@app.route("/ports")
def ports():
    """Serial ports for the landing page's Arduino picker. `default` is the config
    fallback used when no port is chosen (the form sends an empty value)."""
    from config import PORT
    return jsonify({"ports": _probe_ports(), "default": PORT})


@app.route("/set_camera", methods=["POST"])
def set_camera():
    idx = request.form.get("camera", type=int)
    if idx is None:
        return jsonify({"ok": False, "error": "missing camera index"}), 400
    with _runner_lock:
        r = _runner
    if r is None:
        return jsonify({"ok": False, "error": "no runner active"}), 409
    ok, err = r.set_camera(idx)
    return jsonify({"ok": ok, "error": err, "current": r.camera})


@app.route("/run")
def run_view():
    with _runner_lock:
        if _runner is None:
            return redirect(url_for("index"))
    return render_template("run.html")


@app.route("/stop", methods=["POST"])
def stop():
    global _runner
    with _runner_lock:
        r, _runner = _runner, None
    if r is not None:
        r.stop(reason="/stop route")
    return ("", 204)


@app.route("/state")
def state():
    with _runner_lock:
        r = _runner
    if r is None:
        return jsonify({"running": False, "status": "IDLE"})
    return jsonify(r.get_state())


@app.route("/video_feed")
def video_feed():
    def gen():
        while True:
            with _runner_lock:
                r = _runner
            if r is None:
                break
            jpg = r.get_frame_jpeg()
            if jpg is None:
                time.sleep(0.05)
                continue
            yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpg + b"\r\n")
            time.sleep(0.03)   # ~30 fps cap on the stream
    return Response(gen(), mimetype="multipart/x-mixed-replace; boundary=frame")


def main(host: str = "127.0.0.1", port: int = 5000):
    print(f"[INFO] DART web UI -> http://{host}:{port}")
    # threaded so the MJPEG stream and /state polling don't block each other.
    app.run(host=host, port=port, threaded=True, debug=False)


if __name__ == "__main__":
    main()
