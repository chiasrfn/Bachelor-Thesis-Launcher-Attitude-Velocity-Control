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

## Control Strategies Comparison

[cite_start]The project evaluates different architectures to manage the trade-off between response speed, precision, and physical feasibility[cite: 101, 102].

### 1. Attitude Control (Pitch & Yaw)
[cite_start]Two main approaches were compared for the pitch and yaw axes[cite: 2344]:

| Feature | [cite_start]Frequency-based ($G_{cf}$) [cite: 653] | [cite_start]PID Controller ($PID_{yaw}$) [cite: 465] |
| :--- | :--- | :--- |
| **Design Method** | [cite_start]Root Locus tailoring [cite: 662] | [cite_start]Empirical tuning [cite: 466] |
| **Response Speed** | [cite_start]Slower [cite: 829] | [cite_start]Faster [cite: 2345] |
| **Precision** | [cite_start]Higher (lower steady-state error) [cite: 829, 2345] | Slightly lower |
| **Control Effort** | [cite_start]Lower/More efficient [cite: 832, 2345] | [cite_start]Higher (risk of saturation) [cite: 647] |

### 2. Roll Control
* [cite_start]**Criticality**: Roll stabilization is vital because any angular velocity along the roll axis ($p$) couples pitch and yaw dynamics, breaking their independence[cite: 272, 1291].
* [cite_start]**Strategy**: A filtered PID ($G_{roll_2}$) was implemented to ensure rapid stabilization within the structural torque limit of $10^4$ Nm[cite: 286, 376, 427].

### 3. Velocity Control ($G_v$)
* [cite_start]**Implementation**: An outer-loop PID was designed to manage the vehicle's velocity vector[cite: 859, 875].
* [cite_start]**Tracking**: The controller demonstrates high accuracy in tracking references, including those converted from the inertial frame via the $R^{BI}$ rotation matrix[cite: 1192, 1197, 2210].
* [cite_start]**Feasibility**: While theoretically effective, the current implementation is deemed **unfeasible** for real-world deployment because the control effort frequently exceeds physical actuator limits during complex maneuvers[cite: 1087, 2148, 2346].

## Future Developments
* [cite_start]**Propellant Dynamics**: Modeling liquid propellant "sloshing" effects[cite: 2349].
* [cite_start]**Environment**: Transitioning from a flat-Earth model to a curved-Earth model for high-altitude flight[cite: 145, 2351].
* [cite_start]**Non-linear Control**: Implementing non-linear control laws to increase robustness in conditions far from the equilibrium point[cite: 2354].
## Author
**Chiara Serafini**
Bachelor's Degree in Computer Engineering and Automation - Sapienza University of Rome.
Supervisor: Prof. Mattia Mattioni.
