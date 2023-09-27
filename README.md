
# Sliding Mode Controller for Crazyflie 2.0 Quadrotor

## Overview

This project aims to design a Sliding Mode Controller (SMC) for Crazyflie 2.0, a quadrotor, to enable it to track desired trajectories and visit a set of waypoints. The primary objectives are to ensure robust performance, achieve precise tracking, and minimize control efforts.

**Note:** This is a private repository. The code and detailed implementation are not publicly accessible.

## Table of Contents

1. [Problem Statement](#problem-statement)
2. [Solution Approach](#solution-approach)
    - [Part 1: Trajectory Generation](#part-1-trajectory-generation)
    - [Part 2: Controller Design](#part-2-controller-design)
    - [Part 3: Simulation and Evaluation](#part-3-simulation-and-evaluation)
3. [Tuning Parameters](#tuning-parameters)
4. [Performance Metrics](#performance-metrics)
5. [Usage](#usage)
6. [Results](#results)
7. [License](#license)

## Problem Statement

The goal is to develop a Sliding Mode Controller (SMC) for the Crazyflie 2.0 quadrotor to make it follow a pre-defined trajectory even in the presence of external disturbances.

## Solution Approach

### Part 1: Trajectory Generation

We generated a polynomial trajectory of magnitude 5 for the x, y, and z directions. The drone has to reach specific waypoints in a given time frame, starting from the origin (0,0,0).

### Part 2: Controller Design

We used the given equations of motion to design a boundary layer sliding mode control law. The controller is designed for each of the generalized coordinates (z, φ, θ, ψ) to ensure the drone follows the desired trajectory.

### Part 3: Simulation and Evaluation

We wrote a Python script that uses ROS and Gazebo to simulate the performance of the Crazyflie 2.0 quadrotor. The code initializes a ROS node and subscribes to the relevant topics to obtain real-time odometry data.

## Tuning Parameters

We adjusted the tuning parameters for optimal performance. The parameters and their effects are as follows:

- K(z): Affects the controller for z. Value used = 13.
- K(phi): Affects the controller for φ. Value used = 180.
- K(theta): Affects the controller for θ. Value used = 150.
- K(psi): Affects the controller for ψ. Value used = 25.

## Performance Metrics

After testing in Gazebo, the quadrotor successfully followed the desired trajectory. The rotor velocities were well within the desired limit.

## Usage

To run the code, initialize the ROS node and run the simulation in Gazebo.

```
rosrun project your_script.py
```

For visualization, use the following command:

```
rosrun project visualize.py
```

## Results
* Video Demonstration



https://github.com/shreyas-chigurupati07/Sliding_mode_controller/assets/84034817/0bfbb45f-9dfc-42dc-bcd4-12f837da261f



* Trajectory Plot<br>



<img source="https://github.com/shreyas-chigurupati07/Sliding_mode_controller/assets/84034817/c308e51d-289d-485f-ae9a-cfc891d05e7e" width="200" height="200"/>


## License
This project is licensed under the MIT License - see the LICENSE.md file for details.
  

