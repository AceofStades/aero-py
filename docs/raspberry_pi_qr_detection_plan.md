# Raspberry Pi QR Detection & Autonomous Architecture Plan

This document contains the detailed software and hardware integration plan for the Raspberry Pi 5 companion computer, specifically focusing on the QR detection pipeline, sensor fusion, and ROS 2 architecture for the AEROTHON 2026 Micro-UAS.

## 1. QR Scanning Library: `qreader` vs `pyzbar`

For a drone-based vision system hovering at 5 meters, **`qreader` is vastly superior** to `pyzbar`:
*   **pyzbar:** The industry standard, but highly dependent on image quality. Struggles significantly with motion blur, low lighting, off-angle perspectives, and distance.
*   **qreader:** A robust wrapper that uses `pyzbar` under the hood. It utilizes a lightweight AI model (YOLOv7 or YOLOv8) to first detect and segment the QR code within the broader image. Once segmented, it applies image pre-processing (sharpening, Otsu's binarization, blurring, rescaling) before passing the optimized crop to `pyzbar` for decoding. This drastically reduces the "false negative" read rate.

## 2. Implementation Plan for Raspberry Pi QR Scanning

### Phase 1: Hardware and OS Optimization
*   **OS Selection:** 64-bit version of Ubuntu Server (compatible with ROS2) or Raspberry Pi OS. A headless setup (no GUI) is mandatory to save RAM and CPU cycles.
*   **Camera Selection:** Global Shutter camera (e.g., Raspberry Pi Global Shutter Camera or Arducam) to eliminate the "jello" effect caused by multirotor vibrations, which breaks both pyzbar and qreader.
*   **Thermal Management:** Active cooling fan integrated into the 3D-printed avionics bay to prevent thermal throttling during the 25-minute flight window.

### Phase 2: Environment and Dependency Configuration
*   **Virtual Environment:** Isolate the vision stack to prevent package conflicts.
*   **Core Dependencies:** OpenCV, `qreader` library, and base system dependencies for Pyzbar (`libzbar0` on Linux).
*   **ROS2 Integration:** Use `rclpy` (ROS2 for Python) or `rclrs` (Rust) so the vision script can communicate with the flight controller.

### Phase 3: Software Architecture & Pipeline (ROS 2 Nodes)
*   **The Capture Node:** 
    *   Initialize camera stream with manual exposure and focus locked to infinity/5-meter hover distance. Fast shutter speed to freeze motion.
    *   Publish frames at a manageable framerate (e.g., 10-15 FPS).
*   **The Perception Node (Scanner):**
    *   Ingest frames. 
    *   *Optimization Step:* Crop the image (Region of Interest) to speed up YOLO detection.
    *   Pass the cropped frame to `qreader`.
*   **The Decision Node:**
    *   Validate the decoded string format.
    *   Trigger a flag to stop the Capture Node (saving CPU).
    *   Send decoded data to the flight controller (via MAVROS) to transition from "QR Scan State" to "Transit State".

### Phase 4: Field Testing and Calibration
*   **Static Testing:** Test printed QR codes at exact AEROTHON specifications under varying lighting/angles.
*   **Dynamic Testing:** Record video feed while flying manually over the QR code at 5m height. Feed recorded video through the pipeline to tune camera exposure and Region of Interest crop.

## 3. Minimum Sensor Suite for "SkyScan"

1.  **Localization and Stability ("Inner Loop"):**
    *   **Optical Flow Sensor (PMW3901):** Maintains horizontal position without GPS.
    *   **Downward-Facing LiDAR (TF-Luna / TFMini-S):** Precision altitude hold (10ft/3m corridor) and provides scale for optical flow.
    *   **Internal IMU & Barometer:** Standard on Pixhawk/Orange Cube.
2.  **Navigation and Obstacle Avoidance ("Outer Loop"):**
    *   **Forward-Facing LiDAR (1D):** Detects obstacles in path to trigger avoidance.
    *   **Lateral (Side) LiDARs (1D x2):** Critical for "Wall Following" algorithm to stay centered in the 3.5m corridor. (Four 1D LiDARs weigh ~20g total, preferred over a heavy 150g 360-degree LiDAR).
3.  **Mission Perception ("Task Loop"):**
    *   **Mission Camera (Global Shutter):** Downward or 45-degree angle for QR and Green Banner ID.
    *   **FPV Camera (Analog/Digital):** Dedicated low-latency camera for Mission 1 manual flight.

## 4. Compute: Raspberry Pi 5 vs NVIDIA Jetson Nano

*   **Raspberry Pi 5 (Recommended):** Significantly lighter. Excellent support for `qreader` and general OpenCV tasks. More than enough CPU for 1D LiDAR distance logic. Fits strictly within the 2 kg MTOW limit.
*   **Jetson Nano / Orin Nano:** Better for heavy Neural Network vision (semantic segmentation), but the module + carrier board + heatsink can easily exceed 150g-200g, and it is power-hungry.

## 5. Flight Controller & Pi 5 Integration

### The Physical Layer: UART Wiring
The serial connection uses the Pixhawk `TELEM 2` port connected to the Pi 5 GPIO header:
*   **TX (Pixhawk)** -> **RX (Pi Pin 10 / GPIO 15)**
*   **RX (Pixhawk)** -> **TX (Pi Pin 8 / GPIO 14)**
*   **GND (Pixhawk)** -> **GND (Pi Pin 6)**
*   *Critical Warning:* Do NOT connect the 5V power line from the Pixhawk TELEM port to the Pi. Power the Pi 5 directly from the main drone battery using a dedicated 5V/5A UBEC.

### The Language: MAVLink & Data Flow
*   **Handshake:** Devices exchange `HEARTBEAT` messages at 1Hz.
*   **Telemetry Stream (Pixhawk -> Pi):** Pixhawk streams Attitude (50Hz), Local Position (30Hz), and Sensor Raw (10Hz).
*   **Command Execution (Pi -> Pixhawk):** The Pi sends MAVLink commands (e.g., `SET_POSITION_TARGET_LOCAL_NED`) when the drone is in `GUIDED` (ArduPilot) or `OFFBOARD` (PX4) mode.

### ArduPilot vs PX4
*   **ArduPilot:** Older, highly robust, massively feature-rich. Recommended for path of least resistance with basic MAVLink commands, LiDAR logic, and winch payloads.
*   **PX4:** Built for research and companion computers. Slightly steeper learning curve but interfaces incredibly well with ROS 2 and Gazebo simulations via `Offboard` mode.

## 6. Performance Benchmarks: Rust vs Python

### Raspberry Pi 5 (CPU Bound)
*   **Python (qreader + OpenCV + PyMAVLink):** ~800MB-1.2GB RAM. 85-100% CPU. Suffer from the Global Interpreter Lock (GIL) preventing true multithreading. Vision performance: 4-8 FPS. Risk of blocking MAVLink heartbeats.
*   **Rust (OpenCV bindings + ONNX + `mavlink` crate):** ~150-250MB RAM. 50-65% CPU. Flawless concurrency (MAVLink on isolated real-time thread). Vision performance: 10-14 FPS.

### Jetson Orin Nano (GPU Bound)
*   **Python (Ultralytics + PyTorch):** ~2.5GB-3.2GB RAM (maxes out 4GB boards, causing swap stutter). Vision: 15-18 FPS (bottlenecked by Python feeding frames).
*   **Rust/C++ (TensorRT):** ~400-600MB RAM. 10-20% CPU. Vision: 30-35+ FPS.

## 7. ROS 2 Architecture & Multithreading

ROS 2 acts as the "central nervous system" middleware, using **Nodes** (micro-programs) and **Topics** (data streams) without a central master (DDS protocol). It integrates with MAVROS to translate ROS topics directly into MAVLink bytes for the Pixhawk.

### Multithreading via Executors
ROS 2 uses Executors to manage callbacks instead of manual while-loops.
*   **Python (`rclpy`):** MultiThreadedExecutor will hit the GIL. Good for I/O bound tasks (LiDAR serial reading), but CPU-bound vision tasks will lock the GIL and block other threads, causing control loop stutters.
*   **Rust (`rclrs`) / C++ (`rclcpp`):** No GIL. MultiThreadedExecutor distributes callbacks flawlessly across the Pi 5's 4 cores (e.g., Core 1 for vision, Core 2 for LiDAR, Core 3 for MAVLink).

### Recommended Hybrid Architecture (Solo Developer)
1.  **The Heavy Lifting (Rust / C++):** `camera_node` and `vision_processing_node`. Captures frame, runs qreader, publishes decoded string. Maximizes CPU efficiency on an isolated process.
2.  **The Fast I/O (Rust / C++):** `lidar_node`. Reads UART/I2C sensors rapidly and publishes distance arrays.
3.  **The Brain (Python):** `mission_control_node`. Subscribes to vision/LiDAR topics. Handles state machine logic (Takeoff -> Navigate -> Scan -> Drop) utilizing very low CPU power, allowing rapid iteration at the field without recompiling.