# Safety Systems and Business Logic

The successful operation of the AEROTHON 2026 Micro-UAS depends on uncompromising safety protocols and a scalable business model for the "Tiger's Cave" pitch.

## Safety Systems and Fail-Safe Configurations

Given the 2 kg limit and proximity to obstacles, robust fail-safe logic in ArduPilot and PX4 must be tuned to the AEROTHON environment.

### Geofencing and Return-to-Home Settings

Geofencing is a virtual containment system ensuring the drone operates only within safe boundaries.
*   **Parameters:** `FENCE_TYPE` and `FENCE_RADIUS` in ArduPilot define a cylindrical boundary centered on the home point.
*   **Action:** Breaching the limit immediately triggers a Return-to-Launch (RTL) or Land action.
*   **Configuration:** "Inclusion/Exclusion" fence plan in QGroundControl keeps the drone within the designated corridor and delivery area while avoiding pre-defined "Red Zones".

### Battery and Communication Fail-safes

Battery management for 25-minute missions uses a standard three-tier threshold system:
1.  **Low Voltage (Warn):** Notifies the operator via telemetry.
2.  **Critical Voltage (RTL):** Triggers an autonomous return to the home base.
3.  **Emergency Voltage (Land):** Forces an immediate landing to prevent a crash from total power failure.

For manual control and data link loss, parameters like `COM_RC_LOSS_T` and `COM_DL_LOSS_T` dictate the drone's behavior. After a 5-second loss, the drone will loiter for a delay and then land or return home, preventing unpredictable "flyaways" in a competition setting.

---

## Innovation and Business Logic: The Tiger’s Cave

The "Tiger's Cave" stage evaluates the entrepreneurial viability of the UAS design as a scalable solution for real-world industries.

### Market Use Cases and Scalability

The pitch focuses on the **Unique Selling Point (USP)**: navigating complex, tight spaces (industrial boiler rooms, high-density urban corridors) inaccessible to larger drones.
Innovations like semi-solid-state battery integration significantly lower the Total Cost of Ownership (TCO) by providing 2-3 times the cycle life of traditional batteries—a compelling financial metric for logistics firms.

**Industry Applications**
*   **Healthcare:** Remote medicine delivery using precision winch/LiDAR delivery.
*   **Infrastructure:** GPS-denied inspection utilizing Optical Flow and Wall Following algorithms.
*   **Agriculture:** Micro-seeding / Monitoring via autonomous path planning.
*   **Logistics:** Last-mile urban delivery leveraging QR code and Banner detection.

### Future Technological Trajectories (2025-2027)

The evolution of the UAS industry relies on breakthroughs in materials science and edge-AI.

*   **Materials Science:** Lithium sulfide solid-state battery market growth (from $7.4M in 2024 to $700M by 2034) will enable drones with 800+ km ranges. For the 2 kg segment, this means 60-90 minute flight times, tripling the current operational radius.
*   **Edge-AI Processing:** The shift from 16-bit to aggressive 8-bit integer quantization (Int8) in onboard neural networks allows sub-50g processors to execute complex navigation tasks previously requiring power-hungry GPUs. This "intelligence-at-the-edge" enables micro-scale drones to perform robust obstacle avoidance and semantic mapping without high-bandwidth links.
