# DART — Auto Aim Vision Controlled Turret

Autonomous target tracking system: detect, track, and aim at moving objects using computer vision, with laptop-based image processing and Arduino-controlled servo actuation.

**There are two phases:**  
- **Phase 1 — Tracking:** Software prototype and proof of concept (vision + serial + servo control, no weapon).  
- **Phase 2 — With gun:** Mechanical integration with physical pan–tilt turret, trigger mechanism, and safety systems.

---

## Technical objectives

- Object detection with OpenCV under varying lighting.
- Serial link (laptop ↔ Arduino) with low latency.
- Coordinate transformation: 2D image pixels → 3D servo angles.
- PID control for smooth servo motion without oscillation.
- Pan–tilt: 360° horizontal, 90° vertical.
- End-to-end response &lt; 500 ms from detection to positioning.
- Safety: user authorization required for trigger activation.

---

## System architecture (three layers)

| Layer | Role | Technical details |
|-------|------|-------------------|
| **1. Vision (laptop)** | Capture and process video | USB webcam @ 30 fps; Python + OpenCV; color segmentation, contour detection, or motion analysis; pixel coordinates of detected objects. |
| **2. Communication** | Laptop ↔ Arduino | USB serial @ 115200 baud; packet format `X:###,Y:###`; checksum; bidirectional status. |
| **3. Control (Arduino)** | Servos and trigger | Parse packets; pixel→angle transform; PID; MG996R for pan (360°) and tilt (90°); SG90 for trigger; safety logic and LED state. |

---

## Hardware

- **Controller:** Arduino Uno or Mega  
- **Vision:** USB webcam (e.g. 720p)  
- **Actuation:** MG996R (pan and tilt), SG90 (trigger)  
- **Power:** 6 V, 2 A supply  
- **I/O:** Push button (e.g. trigger authorization), LED indicators  

---

## Software

- **Host:** Python 3.8+, OpenCV 4.x, NumPy, PySerial  
- **Firmware:** Arduino IDE, Servo library  

---

## Algorithms

- **Detection:** HSV color filtering, contour analysis, frame differencing.  
- **Geometry:** Coordinate transformation using camera FOV and trigonometry.  
- **Control:** PID for servo positioning.  

---

## Phase 1 — Tracking (software prototype)

Phase 1 validates the full pipeline (camera → detection → serial → servos) **without** any weapon. You get real-time face tracking with a USB webcam and two continuous-rotation MG996R servos (pan and tilt) on a breadboard or simple mount.

### Goal

- Prove vision pipeline, serial protocol, and servo control before building the physical turret (Phase 2).
- Use **face detection** (Haar cascade) as the target; same architecture can later use color/contour or other detectors.

### Setup

- **Laptop:** USB webcam, Python 3 with OpenCV and PySerial.
- **Arduino:** Uno (or compatible) with two MG996R continuous-rotation servos.
- **Wiring:** Pan servo on pin 10, tilt servo on pin 9; servo power from external 6 V supply (common GND with Arduino).

### What runs where

| Component | Location | Role |
|-----------|----------|------|
| **Camera + detection** | Laptop (`face_tracker.py`) | Capture video, detect face, compute pan/tilt speed commands. |
| **Serial link** | USB | Send commands `P###T###` at up to 30 Hz from Python to Arduino. |
| **Servo drive** | Arduino (`arduino_servo_controller.ino`) | Parse commands, clamp to safe range, drive servos; safety timeout if no command. |

### Pipeline (step by step)

1. **Webcam** — Threaded capture; optional MJPEG and small buffer for low latency.
2. **Detection** — Scaled frame → grayscale → Haar cascade (frontal face); largest face chosen; coordinates scaled back to full frame.
3. **Control law** — Face center vs frame center → error in X/Y; dead zone to reduce jitter; proportional gain → pan/tilt speed; smoothing on position and on command.
4. **Serial** — One line per update: `P###T###\n` (pan and tilt as speed 065–115; 090 = stop). Rate-limited (e.g. 30 Hz).
5. **Arduino** — Line-based parser; clamp values to safe range; drive servos; if no command for 250 ms, stop both servos.

### Phase 1 documentation (in repo)

- **[Phase 1 — Python script](phase_1/FACE_TRACKER.md)** — `face_tracker.py`: config, threading (camera + detector), control logic, serial protocol, HUD, dependencies, and usage.
- **[Phase 1 — Arduino controller](phase_1/arduino_servo_controller/ARDUINO_CONTROLLER.md)** — `arduino_servo_controller.ino`: pins, constants, serial protocol, parsing, safety timeout, and wiring.

### Targets (from project abstract)

- ~90% detection in good lighting.
- Serial latency &lt; 50 ms; servo response &lt; 200 ms; total system latency ~350 ms (under 500 ms goal).

---

## Phase 2 — With gun (mechanical integration)

- **Goal:** Integrate code with physical turret, gun mount, and trigger; add safety and robustness.
- **Mechanical:**  
  - Base platform (e.g. 150×120 mm); pan/tilt mounts; camera bracket aligned with barrel; gun clamp (e.g. Nerf); 3D-printed parts (e.g. OpenSCAD), typical print ~0.2 mm layers, 20–40% infill.
- **Control / logic:**  
  - PID and acceleration/deceleration; optional target prediction; multi-level detection confidence.
- **Safety:**  
  - Manual trigger authorization button; LED states (e.g. searching / locked / armed / firing); software angle limits; emergency stop; minimum engagement distance (e.g. &gt; 1 m).
- **Targets (from abstract):** Static aim ±4°; tracking up to ~1.3 m/s; 360° pan verified; total mass ~850 g.

---

## Performance (from validation)

- **Static aim:** Mean error ~3.2° @ 1 m to ~6.8° @ 5 m.  
- **Moving target:** Good tracking up to ~1.3 m/s; performance drops at higher speeds.  
- **Lighting:** Detection rate ~92% (daylight), ~88% (indoor fluorescent), lower in dim/backlight.  
- **Serial:** &gt;99% packet delivery; mean latency ~43 ms.  
- **Response:** Image capture + detection + serial + Arduino + servo ≈ 363 ms total (under 500 ms target).  

---

## References 

1. Bradski & Kaehler, *Learning OpenCV*, O'Reilly.  
2. OpenCV Documentation — Image Processing.  
3. Arduino Documentation — Servo library.  
4. PySerial Documentation.  
5. Szeliski, *Computer Vision: Algorithms and Applications* (2nd ed.), Springer.  
6. Åström & Murray, *Feedback Systems*, Princeton.  
7. OpenSCAD Documentation.  
