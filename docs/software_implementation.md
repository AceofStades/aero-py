# Software Implementation & Autonomous Navigation

The software architecture of the AEROTHON 2026 Micro-UAS handles complex environmental interpretation without human intervention. This document details the perception, mapping, and flight logic required for Mission 2 ("SkyScan").

## Perception and Sensor Fusion

The transition from a manual to an autonomous flight stack requires a robust layer of sensor integration.

### Distance Sensing and Obstacle Avoidance

LiDAR (Light Detection and Ranging) is integrated for precision distancing in a 3.5-meter corridor.
*   **Hardware:** Benewake TF02-Pro (40m range) or TFMini-S (12m range).
*   **Software Configuration:** Using ArduPilot or PX4 "Simple Avoidance" modes with up to nine rangefinders to create a virtual safety bubble.
*   **Parameters:** `AVOID_MARGIN` defines the minimum distance the drone can approach an obstacle before halting or deviating.
*   **Setup:** Forward-facing LiDAR for obstacle avoidance, downward-facing LiDAR for maintaining a sub-centimeter precise 10-foot (approx. 3-meter) altitude.

### GPS-Denied Localization: Optical Flow

In the corridor mission, GPS signals suffer from multipath interference or complete obstruction.
*   **Sensor:** Optical flow (e.g., PMW3901) combined with a downward LiDAR altimeter.
*   **Function:** Tracks ground texture patterns to calculate velocity, functioning similarly to a computer mouse.
*   **Outcome:** Achieves extremely stable hover to support the precision scanning of QR codes at specific altitudes when satellite-based positioning is unavailable.

## Computer Vision Stack

The visual processing required for "SkyScan" is offloaded to a companion computer (NVIDIA Orin NX or Raspberry Pi) running the Robot Operating System (ROS / ROS2).

**Hardware & Software Allocation**
*   **QR Decoding:** `pyzbar` / `quirc` libraries running on the onboard CPU for initial delivery instructions.
*   **Shape/Marker Detection:** OpenCV (`HoughCircles`) running on the Companion Computer for observation tasks.
*   **Green Banner Identification:** HSV Color Masking running on a global camera stream to identify the corridor entrance.
*   **Collision Prediction:** LSTM / Neural Networks running on the GPU/NPU for cluttered forest/corridor operations (future trajectory: Int8 quantization).

**Key Implementation Details:**
The software must implement adaptive thresholding to account for outdoor variables like shadows or sunlight glare. The vision stack commands the drone to ascend to 5 meters to capture, decode, and apply the initial QR data to identify the target QR code at the end of the corridor.

## Autonomous Flight Logic and Navigation Algorithms

High-level path planning is executed in ROS and translated into motor commands by the flight controller via MAVROS.

### Corridor Navigation and Wall Following

Navigating a 3.5m-wide corridor necessitates strict horizontal constraint management.
*   **Traditional PID Controller:** Utilizes two side-facing LiDAR sensors to calculate the distance from the left and right walls. A Proportional-Integral-Derivative controller adjusts yaw and roll to maintain the drone exactly in the center of the path.
*   **Advanced Path Planning:** A* algorithm modified for 3D environments calculates optimal, collision-free routes in under 10 milliseconds.
*   **Deep Reinforcement Learning (DRL):** Models like TD3 (trained in Gazebo or AirSim) navigate corridors with stationary and dynamic obstacles, capable of high success rates (92%) in cluttered environments.

### Autonomous Mission State Machine (ROS2)

The autonomous workflow is managed through a deterministic state machine transitioning strictly based on sensor feedback:

1.  **Takeoff State:** Ascend to 5m utilizing LiDAR altitude feedback.
2.  **QR Scan State:** Activate the camera, rotate/loiter if necessary to identify the initial QR code.
3.  **Transit State:** Detect the "AeroTHON 2026 Green Banner" using HSV masking and align with the corridor entrance.
4.  **Corridor State:** Descend/maintain 3m altitude; engage wall-following and forward obstacle avoidance.
5.  **Target State:** Ascend to 10m; search for the matching target QR code among multiple candidates.
6.  **Delivery State:** Descend to 5m; activate the payload mechanism (winch). The LiDAR monitors ground proximity during descent for a precise drop.
7.  **Return State:** Re-enter the corridor using optical flow and wall-following, return to the launch point, and land.
