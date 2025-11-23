# 5DOF-Robotic-Arm
Robotic Manipulator Control Using Matrix Operations and Sequential Motion Programming

## Table of Contents
1. [Project Outline](#project-outline)
    - [Kinematic Analysis and Modeling of the Manipulator](#kinematic-analysis-and-modeling-of-the-manipulator)
    - [Chapter 2: Programming](#chapter-2-programming)
    - [Chapter 3: Integration and Testing](#chapter-3-integration-and-testing)

## Project Outline

### Kinematic Analysis and Modeling of the Manipulator
5. **Robot Geometry Analysis:**
    * Detailed analysis of link lengths and mechanical assembly.
    * Detailed description of degrees of freedom for each axis.
    * Determination of motion ranges for individual links.
6. **Application of Denavit-Hartenberg Method:**
    * Determination of homogeneous transformation matrices for each joint.
    * Derivation of the transformation matrix from base to end-effector.
7. **Derivation of Inverse Kinematics Algorithm:**
    * Mathematical derivation of the IK algorithm - calculating joint angles and end-effector orientation in the workspace.
    * Analysis of potential multiple solutions for a given position/orientation and criteria for selecting the best one (collision avoidance -> motion minimization).
    * Identification and analysis of singularities. Defining how they will be handled in software.

### Chapter 2: Programming
**Mathematical and Graphical Modeling:**
8. **Mathematical Modeling:**
    * Implementation of functions for homogeneous transformation matrices and forward kinematics. [Python]
9. **Graphical Modeling and 3D Visualization:**
    * Creation of a simplified graphical model. [Python]
    * Integration of the graphical model with the mathematical model to visualize position and orientation in real-time based on joint angles.
10. **Implementation of Inverse Kinematics Algorithms**
    * Implementation of the IK algorithm using forward kinematics results.
11. **Software Testing**
12. **User Interface Development:**
    * Designing GUI layout: buttons, input fields (coordinates, angles), sliders for individual axes, sequence execution button (opening saved file).
    * GUI implementation in selected technology.
    * Connecting FK and IK to GUI for real-time visualization and control (input positions -> see 3D model, input angles -> see effector position).
13. **Implementation of Sequential Programming Functionality:**
    * Implementation of adding, removing, and editing points in a sequence.
    * Implementation of saving and loading sequences to/from file (e.g., CSV, JSON).
    * Development of sequence playback mechanism: robot moves point-to-point using IK.
    * Implementation of trajectory interpolation and handling of unreachable positions.

### Chapter 3: Integration and Testing
**Software Integration with Robot Prototype:**
14. **Communication Protocol Analysis:**
    * Analysis of communication protocol - output data format, input data format, scaling/unit conversion.
15. **Communication Module Implementation:**
    * Implementation of communication module to send calculated angles to robot controllers and receive feedback (positions, errors).
16. **Robot Calibration:**
    * Matching mathematical model to real robot (measuring real link lengths, zero point calibration).
**Verification and Testing:**
17. **Conducting a Series of Tests:**
    * Single axis movement tests.
    * Forward kinematics tests (input angles, measure effector position).
    * Inverse kinematics tests (input position, measure effector position vs target, check accuracy).
    * Sequential programming functionality tests (playback of saved trajectories, movement smoothness).
    * Singularity handling tests.
    * Unreachable point handling tests.
18. **Bug Identification and Fixing.**