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


def _runner_ref():
    with _runner_lock:
        return _runner


# One-off classifier for enrolling from uploads while DART is stopped; created
# lazily (InsightFace load) and cached for the life of the process.
_fallback_classifier = None
_fallback_clf_lock = threading.Lock()


def _get_classifier(r):
    """The live runner's classifier when DART is running, else the cached fallback."""
    if r is not None and r.classifier is not None:
        return r.classifier
    global _fallback_classifier
    with _fallback_clf_lock:
        if _fallback_classifier is None:
            from auth.classifier import FaceClassifier
            print("[INFO] Loading standalone classifier for enrollment…")
            _fallback_classifier = FaceClassifier()
        return _fallback_classifier


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


# ── Authorize Person ─────────────────────────────────────────────────────────
# SECURITY: these endpoints ARE the turret's access control and carry no auth of
# their own. The server binds 127.0.0.1 by default and that is the operating
# assumption — on 0.0.0.0, anyone who can reach the port can authorize themselves.


def _finish_enrollment(clf, name, embeddings):
    """Persist + activate a new identity: save to the pkl, hot-reload the classifier
    db, and let confirmed-UNAUTHORIZED tracks re-classify (their verdict is frozen
    per track, so without the reset a just-enrolled person stays targeted)."""
    from auth.enroll import save_identity
    names = save_identity(name, embeddings, overwrite=True)
    clf.reload_db()
    r = _runner_ref()
    if r is not None and r.detector is not None:
        n = r.detector.reset_unauthorized_tracks()
        if n:
            print(f"[INFO] Enrollment: {n} UNAUTHORIZED track(s) reset for re-check.")
    return names


def _reject_reasons(reasons):
    """'no face found x3, too blurry x2' — most common first."""
    counts = {}
    for why in reasons:
        counts[why] = counts.get(why, 0) + 1
    ordered = sorted(counts.items(), key=lambda kv: -kv[1])
    return ", ".join(f"{why} x{n}" for why, n in ordered)


@app.route("/authorized")
def authorized():
    """Enrolled identity names. Reads the pkl directly so listing never has to load
    InsightFace."""
    import os
    import pickle
    from config import EMBEDDINGS_PATH
    names = []
    if os.path.exists(EMBEDDINGS_PATH):
        try:
            with open(EMBEDDINGS_PATH, "rb") as f:
                names = sorted(pickle.load(f))
        except Exception:
            pass
    return jsonify({"identities": names})


@app.route("/authorize", methods=["POST"])
def authorize():
    """Enroll from the live feed: grab frames over a short window, keep the
    quality-gated ones, and save only if enough survived."""
    from config import (ENROLL_FRAMES, ENROLL_FRAME_INTERVAL, ENROLL_MIN_ACCEPTED)

    name = (request.form.get("name") or "").strip()
    overwrite = request.form.get("overwrite") == "1"
    if not name:
        return jsonify({"ok": False, "error": "missing name"}), 400

    r = _runner_ref()
    if r is None or not r.is_alive():
        return jsonify({"ok": False, "error": "DART is not running"}), 409
    if r.get_raw_frame() is None:
        return jsonify({"ok": False, "error": "DART is still starting"}), 409

    clf = _get_classifier(r)
    if not overwrite and name in clf.db:
        return jsonify({"ok": False, "error": "exists", "name": name}), 409

    embeddings, reasons = [], []
    for _ in range(ENROLL_FRAMES):
        frame = r.get_raw_frame()
        if frame is None:
            reasons.append("no frame")
        else:
            emb, why = clf.extract_embedding(frame)
            if emb is not None:
                embeddings.append(emb)
            else:
                reasons.append(why)
        time.sleep(ENROLL_FRAME_INTERVAL)

    if len(embeddings) < ENROLL_MIN_ACCEPTED:
        return jsonify({
            "ok": False, "accepted": len(embeddings), "captured": ENROLL_FRAMES,
            "error": (f"only {len(embeddings)}/{ENROLL_FRAMES} usable frames "
                      f"({_reject_reasons(reasons)})"),
        }), 422

    names = _finish_enrollment(clf, name, embeddings)
    return jsonify({"ok": True, "name": name, "captured": ENROLL_FRAMES,
                    "accepted": len(embeddings), "identities": names})


@app.route("/authorize/upload", methods=["POST"])
def authorize_upload():
    """Enroll from uploaded photos; works with DART stopped (cached classifier)."""
    import cv2
    import numpy as np

    name = (request.form.get("name") or "").strip()
    overwrite = request.form.get("overwrite") == "1"
    if not name:
        return jsonify({"ok": False, "error": "missing name"}), 400
    files = [f for f in request.files.getlist("photos") if f and f.filename]
    if not files:
        return jsonify({"ok": False, "error": "no photos uploaded"}), 400

    clf = _get_classifier(_runner_ref())
    if not overwrite and name in clf.db:
        return jsonify({"ok": False, "error": "exists", "name": name}), 409

    embeddings, reasons = [], []
    for f in files:
        img = cv2.imdecode(np.frombuffer(f.read(), np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            reasons.append("unreadable image")
            continue
        emb, why = clf.extract_embedding(img)
        if emb is not None:
            embeddings.append(emb)
        else:
            reasons.append(why)

    # Photos are deliberate (unlike the live burst), so one usable face is enough —
    # same bar as the enroll_from_images CLI.
    if not embeddings:
        return jsonify({
            "ok": False, "accepted": 0, "captured": len(files),
            "error": f"no usable face in uploads ({_reject_reasons(reasons)})",
        }), 422

    names = _finish_enrollment(clf, name, embeddings)
    return jsonify({"ok": True, "name": name, "captured": len(files),
                    "accepted": len(embeddings), "identities": names})


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
