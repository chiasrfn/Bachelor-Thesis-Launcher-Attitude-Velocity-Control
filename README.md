# Bachelor-Thesis-Launcher-Attitude-Velocity-Control
# Attitude and Velocity Control of a Non-Flexible Launcher
**A Comparison Between Different Control Strategies**

This repository contains the code and models developed for my Bachelor's Thesis in Computer Engineering and Automation at **Sapienza University of Rome** (A.Y. 2023/2024).

## Project Overview
The goal of this project is to design and compare various control strategies to stabilize the attitude (Roll, Pitch, Yaw) and the velocity of a non-flexible launcher (based on the Ares I model). 

The system is modeled as a rigid body with 9 degrees of freedom. Due to the inherent instability of the launcher (center of pressure above the center of mass), several linear controllers were designed:
- **Attitude Control:** PID and frequency-based controllers.
- **Velocity Control:** PID controllers designed to track inertial or body-frame references.

## Repository Structure
- `models/`: Contains the Simulink model (`.slx`).
- `scripts/`: Contains MATLAB scripts for initialization and data processing.
  - `init_controllers.m`: The main script to initialize all parameters and controller gains.
  - `plot_results.m`: Script to generate high-quality plots from simulation data.
- `figures/`: A collection of representative results from the simulations.

## Requirements
- **MATLAB R2023b** or newer.
- **Simulink** and **Control System Toolbox**.

## How to Run
1. Run `scripts/init_controllers.m` to load the launcher parameters and controller gains into the workspace.
2. Open and run the Simulink model located in `models/`.
3. Execute `scripts/plot_results.m` to visualize the performance and control effort.

## Key Results
The controllers were tested under various conditions, including:
- Initial state offsets (e.g., 5° pitch offset).
- Wind disturbances (up to 50 m/s).
- Variable atmospheric environments and real motor thrust profiles.

*(Optional: Insert a brief comparison between PID and Frequency-based control here)*

## Author
**Chiara Serafini**
Bachelor's Degree in Computer Engineering and Automation - Sapienza University of Rome.
Supervisor: Prof. Mattia Mattioni.
