# Ball and Plate System Simulation

This repository contains the simulation files and control architecture for the Ball and Plate integration project. The simulation models the kinematics, actuator dynamics, and ball dynamics of the physical rig, allowing for rapid testing and tuning of multiple outer-loop controllers.

## Prerequisites

**MATLAB Version:** * This project was developed and tested using **MATLAB R2025a**. Using older versions may cause compatibility issues with Simulink blocks or specific toolbox functions.

**Required Toolboxes:**
To run the simulation and synthesize the controllers, ensure you have the following toolboxes installed:
* Simulink
* Control System Toolbox (for LQR and state-space generation)
* Model Predictive Control Toolbox (for the MPC block and `mpc` objects)
* Robust Control Toolbox (for $H_\infty$ controller synthesis)
* Optimization Toolbox
* shapeit

## How to Run the Simulation

1. Open MATLAB and navigate to the project directory.
2. Open the main execution script: `run_sim.m`. Change the variable 'sim_time' at the top to set the total sim time. 
3. Run the script. 

`run_sim.m` acts as the master initialization file. It will automatically load the plant parameters, compile the state-space models, synthesize the required controller matrices (LQR gains, MPC objects, etc.), and start the Simulink model (`sim_ballAndPlate.slx`). It also handles plotting the data once the simulation finishes.

## Controller Configuration

The Simulink model is equipped with a multiport switch that allows you to easily seamlessly toggle between four different outer-loop control strategies. 

Before running `run_sim.m`, open the Simulink model and adjust the **Controller Selection** constant block to one of the following values:

* **`1`** : Linear Quadratic Regulator (LQR)
* **`2`** : Robust Control ($H_\infty$ Synthesis)
* **`3`** : Model Predictive Control (MPC)
* **`4`** : Traditional PID Control

### Feedforward Control
There is also an acceleration feedforward controller implemented to improve trajectory tracking. You can toggle this on or off using the **Feedforward Enable** constant block in the Simulink model:
* **`1`** : Feedforward Enabled
* **`0`** : Feedforward Disabled
