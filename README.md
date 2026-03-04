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

- **Goal:** Validate vision pipeline, serial protocol, and servo control before mechanical integration.
- **Setup:** Webcam on laptop; Arduino with test servos (e.g. on breadboard).
- **Steps:**  
  1. Webcam at 640×480.  
  2. Object detection: HSV, morphology, contours, centroid.  
  3. Pixel → servo angle conversion (FOV-based).  
  4. Serial: connect, send coordinate packets, validate.  
  5. Arduino: parse packets and drive servos from coordinates.
- **Targets (from abstract):** ~90% detection in good light; serial latency &lt; 50 ms; servo response &lt; 200 ms; total latency ~350 ms.

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
