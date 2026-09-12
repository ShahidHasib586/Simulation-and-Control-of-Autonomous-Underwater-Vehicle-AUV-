# AUV depth control with LQR

MATLAB and Simulink coursework on modelling and controlling the vertical motion of an autonomous underwater vehicle. The project covers hydrodynamic parameters, linearisation, and an LQR controller for depth and pitch response.

## Start here

| Path | Purpose |
| --- | --- |
| `Example_RunSimulation.m` | Main simulation entry point. |
| `MIR_AUVSimulator_R2021b.slx` | Simulink vehicle and control model. |
| `Conf/AUVParameters.json` | Vehicle parameters. |
| `Conf/PilotParameters.json` | Controller parameters. |
| `Initialization/`, `Inputs/`, `Noise/` | Initial state, commands, and noise configuration. |
| `Forces/`, `Piloting/`, `Tools/` | Model, controller, and supporting functions. |
| `Plots/`, `Doc/` | Visualisations and project documentation. |

## Run the simulation

Clone the repository and open its root folder in MATLAB. MATLAB, Simulink, and any toolboxes used by the model must be available.

```matlab
run('Example_RunSimulation.m')
```

The script configures the MATLAB path, loads the JSON parameters, creates the initial state and Simulink buses, runs the model, and plots the result. It begins by clearing the workspace and resetting the MATLAB path; save other work before running it.

The supplied example uses a 0.1 second sample interval and an 870 second simulation duration. Edit the parameters and input functions to investigate different cases. Inspect the model callbacks and referenced functions when using another MATLAB release.

## Evaluation

Compare depth error, pitch, actuator response, and control effort for a defined input and noise configuration. The original project investigates overshoot and steady state response. Performance should be reported with the exact configuration used; simulation results do not establish hardware performance.

## Context

Developed as part of a Control System course. Retain the course material and third party notices supplied with the simulator.

Related project: [Sparus AUV modelling and simulation](https://github.com/ShahidHasib586/Simulation-of-Underwater-Vehicle-Sparus-AUV).
