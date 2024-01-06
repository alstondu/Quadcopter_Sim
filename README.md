# Quadcopter Simulator

## Overview
This GitHub README outlines the process of modeling, simulating, and controlling a quadcopter drone under various conditions using MATLAB. The project is based on the dynamic model and control strategies provided in "Quadcopter Dynamics, Simulation, and Control" by A. Gibiansky [[1]] and corrections in the attached errata document.

## Scenario Descriptions

### Scenario 1: Non-Linear Model Implementation
- **Objective**: Implement a simulation following the non-linear model in the reference material.
- **Inputs**: Squared angular velocities of the 4 propellers.
- **Parameters**: Fixed parameters like quadcopter mass, friction constant, gravitational acceleration, propeller constants, etc.

- **Simulation Tests**:
  - Hovering at a specific position for 5 seconds.
  - Change in rotation at a constant altitude for 5 seconds.
  - Free fall simulation.

### Scenario 2: Linear Model Approximation
- **Objective**: Implement a second simulation using a linearised model approximation and compare it with the non-linear model from Scenario 1.
- **Comparison Tests**:
  - Drone yawing in position for the first 4 seconds, observing the small error in the linear model.
  - Quadcopter falling in the opposite direction when two rotors stop, noting the large error in the linear model.

### Scenario 3: Full-State Feedback Control
- **Assumptions**: 100% accurate state measurement by sensors and limited input to each propeller.
- **Objective**: Implement a full-state feedback controller to perform a specific trajectory.

- **Trajectory Steps**:
  1. Start at (0,0,0).
  2. Move up to (0,0,5).
  3. Stay at (0,0,5) for 5 seconds.
  4. Move to (0,2.5,5).
  5. Follow a circular trajectory on a vertical plane.
  6. Move to (2.5,2.5,2.5).
  7. Land safely at (2.5,2.5,0).

## Instructions
- The MATLAB code for each scenario is located in separate folders labeled S1, S2, and S3.
- Please refer to the referenced document for a detailed understanding of the dynamics and control strategies.
- It's crucial to read the errata document for any corrections to the original material.

## Additional Notes
- The simulations are designed to test and demonstrate the functionality of the quadcopter under various conditions and control strategies.
- Users are encouraged to experiment with different parameters and scenarios to understand the behavior of the quadcopter in different situations.

## How to Use
- Clone the repository and navigate to the respective scenario folders.
- Open the MATLAB file Sim_Quadcopter.m in the cooresponding folder run the simulations and observe the quadcopter's behaviour.
- Modify parameters and scenarios as needed to test different conditions and control strategies.

## Contributing
- If you find any errors or have suggestions for improvement, please open an issue or pull request with your changes.
- This is an open-source project, and contributions are welcome to enhance the simulator's capabilities and accuracy.


## References
- [[1]] A. Gibiansky Quadcopter Dynamics, Simulation, and Control.

[1]: https://andrew.gibiansky.com/downloads/pdf/Quadcopter%20Dynamics,%20Simulation,%20and%20Control.pdf

