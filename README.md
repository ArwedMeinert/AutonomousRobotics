# AutonomousRobotics
Documentation on the AutonomousRobotics course conducted in University West. The goal is to implement different SLAM algorithms using ROS and MatLab. In the Lab, an EKF and MCL was implemented in a Matlab enviroment. Project 1 was about using the ROS mapping functionallity. In Project 2, the source code of the ROS implementation was read and summariesed. Project 3 was about the connection between ROS and Matlab. Here, the robot was controlled in a Gazebo enviroment to first, find a path outside the maze and secondly find a cone shaped object in the maze.
___
# Lab 
The Lab was implemented in MatLab. Here, a robot is controlled in a simulated enviroment.
## EKF and Monte Carlo Localization  

This repository implements **Extended Kalman Filter (EKF) SLAM** and **Sequential Monte Carlo (SMC) Localization** for mobile robot localization using simulated sensor data.  

### **Extended Kalman Filter (EKF) SLAM**  
The EKF model predicts the robot’s position based on a motion model and corrects it using sensor measurements. Key components include:  
- **State Prediction**: Uses linearized motion equations to estimate the next position.  
- **Measurement Model**: Computes expected sensor readings from predicted states.  
- **Correction Step**: Updates state and covariance using Kalman gain.  

### **Sequential Monte Carlo (SMC) Localization**  
The Monte Carlo method represents the state with a **particle filter**, allowing localization without a precise initial estimate. Key aspects:  
- **Particle Initialization**: Randomly distributed based on prior uncertainty.  
- **Motion Update**: Particles are moved using the same motion model as EKF.  
- **Weighting & Resampling**: Particles are weighted based on sensor likelihood and resampled to refine the estimate.  

For full implementation details, check the [code here](./Lab).

# Project 1
The first project focuses on getting used to ROS and its connection to MatLab. The report can be read [here](https://github.com/ArwedMeinert/AutonomousRobotics/tree/aeae27e2b5e97d7a2a206e67119e5ac9f68d4e14/Project%201).
## Implemented Features

### 1. Robot Control
- Integrated ROS-based control for velocity and angular velocity adjustment.
- Enabled manual navigation via `rqt` remote control panel.

### 2. Mapping
- Implemented **GMapping** and **Hector Mapping** algorithms to generate environment maps.
- Processed LiDAR sensor data (`/scan` topic) and transformations (`/tf` topic).
- Stored results in `/map` and `/map_metadata` topics.
- Optimized mapping accuracy by selecting **GMapping** due to better handling of fast movements.
- Analyzed odometry (`/odom` topic) for improved localization (used in **GMapping** but not in **Hector Mapping**).

### 3. Map Refinement
- Manually cleaned up the generated map to remove artifacts.
- Blocked undesired movement areas (e.g., open doors) using image editing tools.

### 4. Navigation System
- Integrated **global and local planners** for pathfinding.
- Configured `move_base` to define costmaps and path penalties.
- Adjusted costmap parameters (inflation radius, scaling factor) to optimize path planning.
- Implemented real-time obstacle avoidance using LiDAR input.
- Simulated navigation in **Gazebo**, including sensor data processing.

### 5. ROS Topic Management
- Structured node interactions:
  - `/move_base` for navigation control.
  - `/cmd_vel` for movement commands.
  - `/scan` for LiDAR data.
  - `/tf` for robot transformations.
- Implemented **AMCL-based localization** for dynamic positioning.

## Summary
This project implements a full **ROS-based navigation system**, including mapping, localization, and autonomous movement. The system was tested in a simulated house environment and optimized for efficient pathfinding and obstacle avoidance. The final implementation ensures reliable robot operation using GMapping, real-time costmap adjustments, and sensor-based navigation strategies. 

# Project 2
The first three parts of this project are about looking into the already existing implementations of the MCL and EKF and looking into how they were implemented.
## 1. Motion Model Implementation  
- Implemented a motion model in a particle filter.  
- Uses Gaussian noise to simulate real-world uncertainties.  
- Updates particle poses based on sampled motion estimates.  

## 2. Measurement Model Implementation  
- Implemented in `amcl_laser`.  
- Uses a beam model to compare sensor readings with expected values from a map.  
- Computes probabilities for sensor data, considering noise, obstacles, and false readings.  

## 3. Comparison Between EKF and AMCL  
- Both use ROS communication and motion models.  
- **AMCL:** More flexible with configurable parameters and sensor models.  
- **EKF:** Limited to differential-drive robots, simpler implementation.  
- AMCL allows fine-tuning of laser parameters, while EKF is more rigid but easier to manage.  

## 4. Mapping in MATLAB  
- EKF-based mapping implemented in MATLAB.  
- Uses laser scans to update a grid map, distinguishing between occupied, free, and unknown cells.  
- Measurement function simulates laser beams to predict distances to obstacles.  

## 5. Findings  
- AMCL is more adaptable for different robots and environments.  
- EKF provides a structured, linearized approach but lacks flexibility.  
- Mapping in MATLAB is effective for visualizing sensor data but requires tuning for accuracy.  
- Measurement models must balance accuracy and computational efficiency.  

# Project 3

## Overview
This project implements autonomous robot navigation using the Robot Operating System (ROS) and MATLAB. The goal is to navigate a simulated maze while avoiding obstacles, identifying objects, and finding the exit without mapping. The implementation can be found [here](https://github.com/ArwedMeinert/AutonomousRobotics/tree/aeae27e2b5e97d7a2a206e67119e5ac9f68d4e14/Project%203)

## Tasks and Implementation

### 1. Moving Through the Maze Without Touching Walls
- The robot scans its surroundings using a 360-degree LiDAR sensor.
- A control algorithm processes the sensor data and determines movement commands.
- If an obstacle is detected within a threshold distance, the robot stops and rotates to avoid collisions.
- The decision-making logic ensures that the robot moves forward while adapting to obstacles on the left and right sides.

### 2. Identifying a Cone
- The robot must recognize a cone-shaped object within the maze.
- Sensor readings are converted into Cartesian coordinates.
- A second-order polynomial fit is applied to distinguish curved surfaces (cones) from flat walls.
- A curvature threshold is used to confirm the presence of a cone, reducing false positives through consecutive detections.

### 3. Finding the Exit by Following the Right Wall
- The robot follows the right wall using a proportional controller.
- Distance readings from the right-side sensor guide the robot’s movements.
- The robot adjusts its heading dynamically to maintain a safe distance from the wall.
- Edge cases, such as dead-ends or narrow passages, are handled by modifying the control parameters.

## Key Findings
- The robot successfully navigates without requiring a global map.
- Polynomial fitting effectively differentiates between cones and walls.
- Proportional control allows smooth wall-following and exit detection, but tuning is necessary for different maze layouts.

## Usage
1. Start the ROS environment.
2. Run the MATLAB script to connect to ROS and execute navigation tasks.
3. Adjust control parameters as needed for different maze configurations.

## Dependencies
- ROS (Robot Operating System)
- MATLAB with ROS Toolbox
- TurtleBot3 Gazebo simulation


