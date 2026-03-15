# Bachelor-Thesis-Launcher-Attitude-Velocity-Control
**Attitude and Velocity Control of a Non-Flexible Launcher: A Comparison Between Different Control Strategies**

This repository contains the code and models developed for my Bachelor's Thesis in Computer Engineering and Automation at **Sapienza University of Rome** (A.Y. 2023/2024).

## Project Overview
The goal of this project is to design and compare various control strategies to stabilize the attitude (Roll, Pitch, Yaw) and the velocity of a non-flexible launcher (based on the Ares I model). 

The system is modeled as a rigid body with 9 degrees of freedom. Due to the inherent instability of the launcher (center of pressure above the center of mass), several linear controllers were designed:
- **Attitude Control:** PID and frequency-based controllers.
- **Velocity Control:** PID controllers designed to track inertial or body-frame references.

## Repository Structure
- `model/`: Contains the Simulink model (`.slx`).
- `code/`: Contains MATLAB scripts for initialization and data processing.
  - `initialization.m`: The main script to initialize all parameters and controller gains.
  - `plot_results.m`: Script to generate high-quality plots from simulation data.
- `figures/`: A collection of representative results from the simulations.
- `thesis/`:The final thesis in italian

## Requirements
- **MATLAB R2024a** or newer.
- **Simulink** and **Control System Toolbox**.

## How to Run
1. Run `code/initialization.m` to load the launcher parameters and controller gains into the workspace.
2. Open and run the Simulink model located in `models/`.
3. Execute `code/plot_results.m` to visualize the performance and control effort.

## Key Results
The controllers were tested under various conditions, including:
- Initial state offsets (e.g., 5° pitch offset).
- Wind disturbances (up to 50 m/s).
- Variable atmospheric environments and real motor thrust profiles.

# Summary of Control Strategies for the Ares I Launcher

This document summarizes the control strategies designed for the stabilization of a non-flexible launcher, based on the mathematical models and simulations provided in this project.

## 1. Roll Controller (RCS)
* **Structure:** The controller $G_{roll_2}(s)$ is a PID with a filter applied to the derivative action.
* **Objective:** It is designed to stabilize the roll angle ($\chi$) and angular velocity ($p$).
* **Constraints:** It operates within a control effort limit of $10^4$ Nm.
* **Performance:** The controller successfully returns the angle to zero after a 1-degree offset, ensuring the independence of the yaw and pitch axes.

## 2. PID Controller (Yaw and Pitch)
* **Logic:** Applied to both axes due to their identical transfer functions when only angles are considered as outputs.
* **Design:** The final version, $G_{pid_3}(s)$, was obtained empirically by increasing derivative action to stabilize the system and reducing integral action to limit oscillations.
* **Performance:** It provides a very fast response with negligible steady-state error and overshoot.
* **Control Effort:** It requires a high initial effort that can lead to actuator saturation for brief periods (approx. 0.20s - 0.75s).

## 3. Frequency-Based Controller
* **Methodology:** Designed using the Root Locus method to tailor the response to the specific process dynamics.
* **Design:** The chosen configuration $G_{cf_3}(s)$ uses a specific pole-zero placement to ensure closed-loop poles remain in the negative half-plane with minimal complexity.
* **Performance:** It is more precise and requires less control effort than the PID, although the response is slightly slower.

## 4. Velocity Controller
* **Implementation:** A PID controller ($G_{v_3}$) that acts on the system already stabilized by an attitude controller ($G_a$).
* **Capability:** It is designed to track vertical and horizontal velocity references, including those expressed in the inertial frame using the $R^{BI}$ rotation matrix.
* **Critical Conclusion:** While the controller tracks references effectively in simulation, the required control effort is often excessive and deemed not fully realizable in real-world conditions.

---

## Conclusion and Comparison

| Feature | PID Controller | Frequency Controller ($G_{cf}$) |
| :--- | :--- | :--- |
| **Response Time** | Faster | Slower |
| **Precision** | High | Very High (more precise) |
| **Effort Efficiency** | Lower efficiency; higher saturation risk | Higher efficiency; lower effort required |
| **Design Approach** | Empirical / Trial-and-error | Analytical / Root Locus |

**Final Assessment:** Both attitude controllers meet the design specifications. The **PID** is preferred for missions requiring rapid maneuvers, while the **Frequency-Based** controller is superior for precision and energy efficiency. The **Velocity Controller** serves as a valid theoretical baseline but requires further optimization to respect physical actuator limits.

## Author
**Chiara Serafini**
Bachelor's Degree in Computer Engineering and Automation - Sapienza University of Rome.
Supervisor: Prof. Mattia Mattioni.
