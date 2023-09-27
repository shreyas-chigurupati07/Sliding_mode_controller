
# Sliding Mode Controller for Crazyflie 2.0 Quadrotor

## Overview

This project aims to design a Sliding Mode Controller (SMC) for Crazyflie 2.0, a quadrotor, to enable it to track desired trajectories and visit a set of waypoints. The primary objectives are to ensure robust performance, achieve precise tracking, and minimize control efforts.

**Note:** This is a private repository. The code and detailed implementation are not publicly accessible.

## Table of Contents

1. [Problem Statement](#problem-statement)
2. [Solution Approach](#solution-approach)
    - [Part 1: Mathematical Model](#part-1-mathematical-model)
    - [Part 2: Trajectory Generation](#part-2-trajectory-generation)
    - [Part 3: Controller Design](#part-3-controller-design)
    - [Part 4: Simulation and Evaluation](#part-4-simulation-and-evaluation)
4. [Tuning Parameters](#tuning-parameters)
5. [Performance Metrics](#performance-metrics)
6. [Usage](#usage)
7. [Results](#results)
8. [License](#license)

## Problem Statement

The goal is to develop a Sliding Mode Controller (SMC) for the Crazyflie 2.0 quadrotor to make it follow a pre-defined trajectory even in the presence of external disturbances.

## Solution Approach

### Part 1: Mathematical Model
The equations of motion for the Crazyflie 2.0 are modeled as a set of nonlinear differential equations:

```math
m\ddot{x} = -F_{dx} + F_{th} \sin(\phi) \cos(\theta)
```
```math
m\ddot{y} = -F_{dy} + F_{th} \sin(\theta) \cos(\phi)
```
```math
m\ddot{z} = -F_{dz} + F_{th} \cos(\theta) \cos(\phi) - mg
``` 

where $`F_{dx}, F_{dy}, F_{dz}`$ are the drag forces and $`F_{th}`$ is the thrust force generated by the motors.

### Part 2: Trajectory Generation

Generated a polynomial trajectory of magnitude 5 for the x, y, and z directions. The drone has to reach specific waypoints in a given time frame, starting from the origin (0,0,0).<br>
The quadrotor's trajectory is planned using quintic polynomial equations:
```math
x_d(t) = a_0 + a_1t + a_2t^2 + a_3t^3 + a_4t^4 + a_5t^5
```
The coefficients $`a_0, a_1, \ldots, a_5 `$ are computed to satisfy boundary conditions at waypoints $`p_0, p_1, \ldots, p_5 `$.
### Part 3: Controller Design

Used the given equations of motion to design a boundary layer sliding mode control law. The controller is designed for each of the generalized coordinates (z, φ, θ, ψ) to ensure the drone follows the desired trajectory.<br>

A sliding mode controller is employed to control the Crazyflie 2.0. The sliding surfaces \( S \) are defined as:
```math
S = \dot{e} + \lambda e 
```
where \( e \) is the tracking error. The control laws \( u \) are designed to ensure that the system state reaches the sliding surfaces in finite time and stays there.
```math
u = -K 	ext{sgn}(S)
```
#### Controller for \( z \)
```math


u_1 = m \left( g + \ddot{z}_d + \lambda_1 ( \dot{z}_d - \dot{z} ) + K_1 \text{sat}( S_1 ) \right) / ( \cos(\phi) \cos(\theta) )


```
Where $`S_1 = \dot{z}_d - \dot{z} + \lambda_1 ( z_d - z )`$

#### Controller for $`(\phi) `$
```math

u_2 = -     ext{(rotational terms)} - \lambda_2 \dot{\phi} I_x - K_2 I_x     ext{sat}( S_2 )

```
Where $`S_2 = \dot{\phi}_d - \dot{\phi} + \lambda_2 ( \phi_d - \phi ) `$

#### Controller for $`\theta`$ and $`\psi`$

Similar equations were derived for $`\theta`$ and $`\psi`$.

### Part 4: Simulation and Evaluation

Wrote a Python script that uses ROS and Gazebo to simulate the performance of the Crazyflie 2.0 quadrotor. The code initializes a ROS node and subscribes to the relevant topics to obtain real-time odometry data.

## Tuning Parameters

Adjusted the tuning parameters for optimal performance. The parameters and their effects are as follows:

- K(z): Affects the controller for z. Value used = 13.
- K($`\phi`$): Affects the controller for φ. Value used = 180.
- K($`\theta`$): Affects the controller for θ. Value used = 150.
- K($`\psi`$): Affects the controller for ψ. Value used = 25.

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
* Gazebo simulation video of Quadrotor tracing the desired trajectory.  




https://github.com/shreyas-chigurupati07/Sliding_mode_controller/assets/84034817/e9479e0d-edf8-43a1-bc57-53c8e792c44b




* Trajectory Plot<br>

<p align="center">
    <img src="https://github.com/shreyas-chigurupati07/Sliding_mode_controller/assets/84034817/1946558a-0bcc-49ad-ae35-c20afc0bd5f0" />
</p>







## License
This project is licensed under the MIT License - see the LICENSE.md file for details.


## Note:
This is a private repository. Access is restricted to specific collaborators.

