"""
enrol.py  —  Face Enrolment Tool for DART
==========================================
Captures photos of a person from your USB cam and saves them
into the correct known_faces/<name>/ folder structure.

Usage
-----
  python enrol.py --name "Alice" --count 10 --camera 1

Arguments
---------
  --name    Person's name (creates known_faces/<name>/ folder)
  --count   Number of photos to capture (default: 10)
  --camera  Camera index (default: 1 for USB cam)
  --delay   Seconds between auto-captures (default: 0.5)

Controls
--------
  SPACE     Capture a photo manually
  A         Toggle auto-capture on/off
  Q         Quit (even if target count not reached)

Tips for best recognition accuracy
-----------------------------------
  • Capture 10-15 photos minimum
  • Vary your head angle slightly (left, right, up, down)
  • Include photos with and without glasses if applicable
  • Ensure good, even lighting — avoid harsh shadows
  • Stay roughly the same distance from camera as during demo
"""

import argparse
import os
import time

import cv2

# ── Config ───────────────────────────────────────────────────────
KNOWN_FACES_DIR = "known_faces"
CAM_WIDTH       = 1280
CAM_HEIGHT      = 720
CAM_FPS         = 30
FACE_CASCADE    = cv2.data.haarcascades + "haarcascade_frontalface_default.xml"


def main():
    parser = argparse.ArgumentParser(description="DART face enrolment tool")
    parser.add_argument("--name",   type=str,   required=True,  help="Person's name")
    parser.add_argument("--count",  type=int,   default=10,     help="Number of photos")
    parser.add_argument("--camera", type=int,   default=1,      help="Camera index")
    parser.add_argument("--delay",  type=float, default=0.5,    help="Auto-capture delay (s)")
    args = parser.parse_args()

    # ── Prepare output folder ────────────────────────────────────
    save_dir = os.path.join(KNOWN_FACES_DIR, args.name)
    os.makedirs(save_dir, exist_ok=True)
    print(f"[INFO] Saving photos to: {save_dir}/")
    print(f"[INFO] Target: {args.count} photos")
    print()
    print("  SPACE = manual capture")
    print("  A     = toggle auto-capture")
    print("  Q     = quit")
    print()

    # ── Camera ───────────────────────────────────────────────────
    backend = cv2.CAP_DSHOW if hasattr(cv2, "CAP_DSHOW") else 0
    cap = cv2.VideoCapture(args.camera, backend)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  CAM_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS,          CAM_FPS)
    cap.set(cv2.CAP_PROP_BUFFERSIZE,   1)

    if not cap.isOpened():
        print(f"[ERROR] Could not open camera {args.camera}")
        return

    # ── Face detector (Haar — lightweight, no model files needed) ─
    detector = cv2.CascadeClassifier(FACE_CASCADE)

    # ── State ────────────────────────────────────────────────────
    captured     = 0
    auto_capture = False
    last_auto    = 0.0

    # Find next available photo index (don't overwrite existing)
    existing = [f for f in os.listdir(save_dir) if f.endswith(".jpg")]
    photo_idx = len(existing)

    print(f"[INFO] Found {photo_idx} existing photo(s) for {args.name!r}.")

    while captured < args.count:
        ret, frame = cap.read()
        if not ret or frame is None:
            continue

        display = frame.copy()
        h, w    = frame.shape[:2]

        # ── Detect face for guide box ────────────────────────────
        gray  = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = detector.detectMultiScale(gray, 1.1, 5, minSize=(80, 80))

        face_detected = len(faces) > 0

        for (x, y, fw, fh) in faces:
            col = (0, 220, 80) if face_detected else (0, 60, 220)
            cv2.rectangle(display, (x, y), (x+fw, y+fh), col, 2)

        # ── Auto-capture ─────────────────────────────────────────
        now = time.monotonic()
        if auto_capture and face_detected and (now - last_auto) >= args.delay:
            _save(frame, save_dir, photo_idx)
            photo_idx += 1
            captured  += 1
            last_auto  = now
            print(f"  [AUTO] Captured {captured}/{args.count}")

        # ── HUD ──────────────────────────────────────────────────
        overlay = display.copy()
        cv2.rectangle(overlay, (0, 0), (w, 44), (10, 10, 10), -1)
        cv2.addWeighted(overlay, 0.5, display, 0.5, 0, display)

        mode_str = "AUTO" if auto_capture else "MANUAL"
        face_str = "FACE OK" if face_detected else "NO FACE"
        face_col = (0, 220, 80) if face_detected else (0, 60, 220)

        cv2.putText(display,
                    f"Enrolling: {args.name}   [{mode_str}]   "
                    f"{face_str}   {captured}/{args.count}",
                    (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    face_col, 2, cv2.LINE_AA)

        cv2.putText(display, "SPACE=capture  A=auto  Q=quit",
                    (10, h - 10), cv2.FONT_HERSHEY_SIMPLEX,
                    0.38, (120, 120, 120), 1, cv2.LINE_AA)

        # Flash effect on capture
        cv2.imshow(f"Enrol — {args.name}", display)

        key = cv2.waitKey(1) & 0xFF

        if key == ord("q"):
            print("[INFO] Quit by user.")
            break

        elif key == ord("a"):
            auto_capture = not auto_capture
            state = "ON" if auto_capture else "OFF"
            print(f"[INFO] Auto-capture {state}")

        elif key == ord(" "):
            if face_detected:
                _save(frame, save_dir, photo_idx)
                photo_idx += 1
                captured  += 1
                print(f"  [MANUAL] Captured {captured}/{args.count}")
                # Brief white flash
                flash = np.ones_like(display) * 255
                cv2.addWeighted(flash, 0.3, display, 0.7, 0, display)
                cv2.imshow(f"Enrol — {args.name}", display)
                cv2.waitKey(80)
            else:
                print("  [SKIP] No face detected — reposition and try again")

    cap.release()
    cv2.destroyAllWindows()

    total = len([f for f in os.listdir(save_dir) if f.endswith(".jpg")])
    print()
    print(f"[DONE] {captured} photo(s) captured this session.")
    print(f"[DONE] {total} total photo(s) in {save_dir}/")
    print()
    print("Next step:")
    print(f"  python face_tracker_recognition.py --camera 1 --port COM3")


def _save(frame, save_dir: str, idx: int):
    """Save a single frame as JPEG."""
    import numpy as np  # already imported at top of real usage
    path = os.path.join(save_dir, f"{idx:04d}.jpg")
    cv2.imwrite(path, frame)


if __name__ == "__main__":
    import numpy as np  # noqa: F401  (used inside _save flash effect)
    main()