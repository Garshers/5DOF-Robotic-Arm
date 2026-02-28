# Robotic Manipulator Control Using Matrix Operations and Sequential Motion Programming

## Project Overview
This repository contains the complete control system for a **5-Degree-of-Freedom (5-DOF) robotic manipulator**, developed as part of a formal engineering thesis. The project integrates advanced mathematical modeling with real-time embedded systems to achieve high-precision robotic motion.

The system utilizes a dual-layer architecture, splitting tasks between a high-level Python application and a low-level ESP32 embedded controller.



---

## Key Features
* **Analytical Inverse Kinematics (IK):** High-speed, deterministic calculation of joint angles using closed-form trigonometric solutions.
* **Dual-Core Execution:** Leveraging the ESP32's dual-core architecture to isolate motor pulse generation from serial communication.
* **DH Notation Framework:** Forward kinematics modeled using the Denavit-Hartenberg convention for modularity and scalability.
* **Sequential Programming:** A built-in GUI tool for recording, editing, and playing back complex motion paths via waypoints.
* **Real-time Telemetry:** Live feedback of joint positions and Cartesian coordinates ($X, Y, Z$).

---

## System Architecture

The software is divided into two primary layers to ensure low latency and a rich user experience:

| Layer | Technology | Primary Responsibilities |
| :--- | :--- | :--- |
| **High-Level** | Python (NumPy, Tkinter) | GUI, Kinematic Engine, Trajectory Planning, Serial Management. |
| **Low-Level** | C/C++ (ESP32, FreeRTOS) | PWM/Step Generation, Homing, Multithreaded Task Management. |

### Embedded Task Distribution (FreeRTOS)
* **Core 0 (Communication):** Handles UART serial interface and command parsing.
* **Core 1 (Control):** Manages high-priority motion loops and acceleration/deceleration profiles.

---

## Control Theory and System Implementation

### 1. Kinematic Solution Verification and Selection
The system does not only calculate potential positions but actively filers them through a rigorous verification engine. The `solve_ik` module ensures operational stability by:
* **Eliminating Unreachable Solutions:** Automatic rejection of coordinates outside the mechanical workspace or joint limits.
* **Optimal Configuration Selection:** When multiple solutions exist (e.g., different elbow orientations), the algorithm selects the one that:
    * **Minimizes Displacement:** Chooses the configuration closest to the current joint state to ensure fluid motion.
    * **Avoids Singularities:** Identifies and bypasses configurations where the manipulator loses a degree of freedom.
    * **Respects Range Constraints:** Ensures no motor exceeds its physical hard-stops.

### 2. Communication Protocol (NMEA 0183 Standard)
To ensure data integrity between the Python GUI and the ESP32, the system implements a communication fail-safe based on the industrial **NMEA 0183** standard:
* **Input Validation:** Every incoming packet is verified for structural correctness before processing.
* **Checksum Mechanism:** A high-reliability 8-bit XOR checksum is calculated for all data characters preceding the `*` separator. If the calculated checksum does not match the transmitted value, the command is discarded to prevent erratic movements.

### 3. Advanced Motor Control
* **Proportional Controller:** Used for precise position tracking and smooth velocity ramping.
* **Backlash Compensation:** The software includes an offset-correction algorithm to compensate for mechanical play (backlash) in the gearboxes, significantly increasing the precision of the TCP (Tool Center Point) positioning.

### 4. Master-Slave Synchronization
The architecture utilizes a **Master-Slave** implementation to manage multi-axis coordination. The system continuously monitors the execution state of each joint and applies **desynchronization correction** to ensure that all 5 axes reach their target waypoints simultaneously, preventing path distortion during complex moves.

### 5. Orientation Optimization Algorithm
In automatic mode, where the effector orientation angle ($\phi$) is not manually defined, the system performs a one-dimensional optimization to find the most efficient approach:
* **Global Search (Exploration):** The system iteratively scans the $[-180^\circ, 180^\circ]$ range with a $1^\circ$ step to identify the global minimum and discard mechanically unreachable zones.
* **Golden Section Search:** Once a coarse global minimum is found, the algorithm narrows the window to $\pm 2.0^\circ$ and applies the Golden Section Search method. This achieves a final precision of $0.01^\circ$ without excessive CPU load, ensuring a perfectly optimized grip orientation.

### 6. Sequential Motion and Path Planning
The system implements a **Teach-In** programming method, allowing users to define complex trajectories:
* **Point-To-Point (PTP) Execution:** The robot moves through a list of waypoints stored in the `sequence_data` vector.
* **Programming Modes:**
    * **Cartesian Space (XYZ):** User defines $X, Y, Z$ and orientation. The system runs real-time IK verification before saving the point.
    * **Joint Space ($\theta_1 \dots \theta_5$):** Direct manipulation of joint angles via sliders for maximum manual control.
* **Persistence:** Sequences are serialized into **JSON** format, allowing for saving, loading, and sharing motion profiles across different sessions via `save_sequence_to_json` and `load_sequence_from_json`.

---

## Summary
The developed system successfully integrates a 5-DOF manipulator with a dual-layer control architecture, combining advanced orientation optimization with robust industrial communication standards. Through the application of a hybrid optimization algorithm and real-time kinematic verification, the project achieves high-precision motion execution and reliable sequential programming.

---

## Getting Started

### 1. Hardware Configuration
* **Connection:** Interface the 5-DOF manipulator actuators with the designated ESP32 GPIO pins.
* **Power Supply:** Verify that the external power source provides sufficient current and matches the voltage specifications of the motors to prevent logic brownouts.



### 2. Firmware Deployment
* Open the `/embedded` directory in **VS Code** with the **PlatformIO** extension or the **Arduino IDE**.
* Select the appropriate board configuration (ESP32 Dev Module).
* Compile and flash the firmware to the microcontroller.

### 3. Software Environment Setup
Ensure you have Python 3.x installed, then install the required numerical and communication libraries:

```bash
pip install numpy pyserial
```

### 4. System Initialization
Launch the high-level control application by executing the main script:

```bash
python app/main.py
```
## Video Demonstration
Observe the system performance, including manual joint manipulation, Inverse Kinematics (IK) positioning, and automated sequence execution:

[Click here to watch the 5-DOF Manipulator in action](https://www.youtube.com/watch?v=tdGiQl89dQU&feature=youtu.be)

Technical Note: This project was developed as a comprehensive engineering thesis, emphasizing the synergy between mechatronics, matrix-based control theory, and real-time embedded systems.