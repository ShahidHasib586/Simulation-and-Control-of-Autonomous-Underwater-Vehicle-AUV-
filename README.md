# Autonomous Underwater Vehicle Depth Control Using Linear Quadratic Regulator (LQR)

This repository contains the implementation and simulation of an LQR-based control system for the depth control of an Autonomous Underwater Vehicle (AUV) operating in challenging underwater environments. The project was developed as part of the **Control System** course.

## 🚀 Project Overview

### Problem Statement
The underwater environment is characterized by high pressures, limited visibility, dynamic currents, and unpredictable gradients. Traditional PID controllers often fail to provide sufficient robustness and stability in such conditions. This project aims to design an LQR-based control system for the vertical motion of an AUV, focusing on depth command optimization.

### Objectives
1. Identify and calculate missing hydrodynamic parameters.
2. Linearize the AUV's nonlinear model and derive the state-space representation.
3. Design and implement an optimal LQR controller to:
   - Minimize pitch oscillations.
   - Ensure smooth lift speed transitions.
   - Reduce control overshoot while maintaining stability.

## 🛠 Features
- **Mathematical Modeling**: Linearization of the nonlinear equations of motion.
- **Control Design**: Implementation of an LQR controller in MATLAB.
- **Simulation**: Validation of the controller with real-world scenarios, focusing on depth, pitch angle, and actuator dynamics.
- **Performance Metrics**:
  - Minimal overshoot (1.4 m, within the 2 m limit).
  - Zero steady-state error.
  - Smooth and stable actuator adjustments.

## 📂 Repository Structure
```plaintext
.
├── src/
│   ├── identification.m        # MATLAB scripts for parameter identification
│   ├── linearization.m         # Linearization and state-space derivation
│   ├── lqr_controller.m        # Implementation of the LQR control
│   ├── simulation_results.m    # Simulation and plotting scripts
├── docs/
│   ├── report.pdf              # Full project report
│   ├── presentation.pdf        # Project presentation slides
└── README.md                   # Project description and usage guide
