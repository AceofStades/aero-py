# Hardware Architecture & System Integration

The physical architecture of the AEROTHON 2026 Micro-UAS focuses on structural efficiency, high-density power solutions, and redundant flight electronics to maintain performance under the 2 kg MTOW limitation.

## Structural Architecture: Advanced Materials

The airframe maximizes stiffness-to-weight ratio to mitigate structural resonances that disrupt flight controller (FC) high-frequency control loops.

*   **Hybrid Material Strategy:**
    *   **Carbon Fiber:** Standard tubes for motor arms ensure maximum longitudinal stiffness.
    *   **Additive Manufacturing:** 3D-printed central hub using carbon-fiber-infused thermoplastics (PETG-CF or PA-CF/Nylon). Allows for integrated conduits, internal avionics bays, and eliminates "hanging wires".
    *   **Topology Optimization:** Target a 40% reduction in weight through 3D-printed composite frames with continuous carbon fiber (CCF) reinforcement and internal infill strategies like Honeycomb or Cubic.

**Comparative Material Performance**
*   **Standard Carbon Fiber:** Arms, main plates. (Density 1.5-1.8, Modulus 150-250 GPa).
*   **PA-CF (Nylon):** Motor mounts, landing gear. (Density 1.3-1.5, Modulus 10-20 GPa).
*   **PETG-CF:** Battery trays, sensor mounts. (Density 1.27-1.32, Modulus 4-6 GPa).
*   **Polycarbonate (PC):** Protective canopies. (Density 1.2, Modulus 2-3 GPa).

## Avionics and Flight Control

The flight controller is the nexus for processing sensor data and stabilizing the rotorcraft.
*   **Processor:** STM32H753 (Arm Cortex-M7 at 480MHz, 2MB Flash) integrated into Pixhawk 5X or 6C platforms.
*   **Sensor Redundancy:** Triple-redundant IMUs and dual barometers. Critical for the corridor mission to mitigate magnetic interference and motor vibrations.
*   **Thermal Management:** Sensor heating resistors and vibration isolation minimize IMU thermal drift, crucial for precision hovering during QR scanning.

**Hardware Connectivity**
*   **UART/Serial:** GPS, Companion Computer, LiDAR.
*   **I2C:** Compass, Barometer, Optical Flow.
*   **CAN/DroneCAN:** Redundant GPS, Laser Altimeters (preferred for noise immunity).
*   **PWM/SBUS:** ESCs, Servo Grippers.

## Propulsion Dynamics & Energy Storage

*   **Thrust-to-Weight:** Target a ratio of at least 2:1 for a 2kg system to handle aerodynamic disturbances while carrying a 100g payload.
*   **Motors:** Brushless DC (BLDC) motors, KV 1750–2400 (e.g., T-MOTOR KV1750). Paired with 5-7 inch carbon fiber propellers to reduce rotational inertia.
*   **ESCs:** 20A–40A Electronic Speed Controllers for handling aggressive maneuvers.
*   **Batteries:** Transitioning from Standard LiPo (100-200 Wh/kg) to **Semi-Solid State**.
    *   *Why?* Energy density of 350-400 Wh/kg doubles flight time. Over 90% capacity retention at -10°C, zero risk of leakage, and significantly reduced thermal runaway probability.

## Payload Delivery Systems (100g)

*   **Servo-Triggered Latch:** Simple, 9g servo rotating a cam/pin. Configurable via `GRIP_ENABLE` in ArduCopter.
*   **Winch System (Chosen for Mission 2):** High-efficiency brushed motor (775/540) with a self-locking worm gear reducer allows for controlled 5-meter descent. Integrated with LiDAR for "automatic" ground detection and release.
*   **Electromagnet (Alternative):** 12V DC electromagnet controlled via MOSFET switching circuit for silent, moving-part-free drops.
