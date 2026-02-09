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

## Mathematical Foundations

### Forward Kinematics (DH Notation)
The spatial configuration is defined by the transformation matrix $A_i$:

$$A_i = \begin{bmatrix} \cos\theta_i & -\sin\theta_i\cos\alpha_i & \sin\theta_i\sin\alpha_i & a_i\cos\theta_i \\ \sin\theta_i & \cos\theta_i\cos\alpha_i & -\cos\theta_i\sin\alpha_i & a_i\sin\theta_i \\ 0 & \sin\alpha_i & \cos\alpha_i & d_i \\ 0 & 0 & 0 & 1 \end{bmatrix}$$

The total transformation from base to end-effector is:
$$T_5^0 = A_1 A_2 A_3 A_4 A_5$$

### Inverse Kinematics
To enable Cartesian control, the base angle is first isolated:
$$\theta_1 = \operatorname{atan2}(Y, X)$$
The remaining joint angles ($\theta_2, \theta_3, \theta_4$) are solved using the Law of Cosines within the radial plane, ensuring instantaneous calculation without the need for iterative numerical solvers.

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