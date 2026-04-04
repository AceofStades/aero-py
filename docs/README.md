# AEROTHON 2026: Autonomous Micro-UAS Architecture

## Technical Engineering Synthesis

This documentation outlines the hardware and software architectures for an autonomous Micro-UAS developed for the SAEINDIA AEROTHON 2026 Rotorcraft Systems Challenge. The project focuses on high-autonomy, micro-scale platforms capable of executing complex logistics and surveillance tasks in GPS-denied environments.

### Mission Requirements and Tactical System Constraints

The system is designed within the **Micro-UAS** category, with a strict Maximum Take-Off Weight (MTOW) of under 2 kg, requiring an uncompromising focus on structural efficiency, high-density power solutions, and edge-computing software stacks capable of real-time perception and decision-making. 

#### Physical Constraints
*   **Configuration:** Electric Multirotor (Fixed-wing and hybrid VTOL excluded).
*   **MTOW:** < 2 kg
*   **Payload Capacity:** 100g (Rapid Delivery)
*   **Range Connectivity:** At least 1 km

#### Operational Profiles

**Mission 1: "Eyes in the Sky" (Manual)**
Evaluates the human-machine interface and the platform's aerodynamic stability.
*   **Tasks:** Navigate through hurdles, tunnels, and slaloms.
*   **Objectives:** Identify visual objects at designated observation zones using low-latency video transmission (FPV).

**Mission 2: "SkyScan" (Fully Autonomous)**
Elevates the technical requirement to full autonomy in GPS-denied environments.
*   **Tasks:** Autonomous takeoff, QR code scanning (5m altitude) for delivery instructions.
*   **Navigation:** Identify a green banner (corridor entrance), navigate a 3.5m-wide corridor at 10ft altitude while avoiding static obstacles.
*   **Delivery:** Identify target QR code at 10m, execute precision payload delivery from 5m, and return to launch autonomously.

---

### Project Documentation Index

1. [Hardware Architecture](./hardware_architecture.md)
2. [Software Implementation (Perception, Logic, & Autonomy)](./software_implementation.md)
3. [Safety Systems and Business Logic](./safety_and_business.md)
