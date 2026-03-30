# MBot Autonomous Navigation and SLAM System

## Overview

This project implements a complete autonomous robotics pipeline for the MBot platform using ROS2. The system integrates motion control, mapping, localization, path planning, and exploration to enable the robot to autonomously navigate and explore unknown environments.

The project was developed through a sequence of robotics labs and integrates several fundamental robotics algorithms, including PID control for motion execution, Monte Carlo Localization (MCL), occupancy grid mapping, A* path planning, and frontier-based exploration.

The system enables the robot to:

- Estimate its pose using probabilistic localization
- Build a map of the environment using occupancy grid mapping
- Plan collision-free paths using A* search
- Autonomously explore unknown environments using frontier exploration
- Integrate mapping and localization into a SLAM pipeline

---

# System Architecture

The overall system consists of several ROS2 nodes that work together to enable autonomous navigation.


Major modules include:

- Motion control
- Localization
- Mapping
- Path planning
- Autonomous exploration
- SLAM integration

All modules communicate through ROS2 topics.

---

# Project Components

## 1. PID Motion Control

The motion controller is responsible for executing navigation commands by converting waypoint targets into velocity commands.

A PID-based controller regulates the robot's linear and angular velocity to follow the desired path.

### Responsibilities

- Track waypoint targets
- Control robot orientation
- Generate velocity commands for the differential drive robot

### Key Files
```tree
mbot_setpoint/
    src/
        motion_controller_diff.cpp
```


---

## 2. Monte Carlo Localization (MCL)

Monte Carlo Localization estimates the robot's pose using a particle filter. The algorithm maintains a set of particles representing possible robot poses and updates them using motion and sensor models.

### Algorithm Steps

1. Initialize particles
2. Apply motion update using odometry
3. Evaluate particles using a sensor model
4. Resample particles based on weights
5. Estimate robot pose

### Key Files
```tree
mbot_localization/
    src/
        particle_filter.cpp
        obstacle_distance_grid.cpp
        action_model.cpp
        sensor_model.cpp
```


---

## 3. Occupancy Grid Mapping

The mapping module constructs a grid-based map using LiDAR sensor data.

Each grid cell stores the probability that the space is occupied.

### Mapping Process

1. Convert LiDAR scans into world coordinates
2. Update cell probabilities using log-odds

### Key Files
```tree
mbot_mapping/
    src/
        mapping.cpp
        mapping_node.cpp
```


---

## 4. A* Path Planning

The navigation system uses the A* algorithm to compute collision-free paths between two poses.

The planner operates on an obstacle distance grid derived from the occupancy map.

### Planning Process

1. Convert start and goal poses to grid coordinates
2. Perform A* search
3. Reconstruct the path
4. Convert path to world coordinates
5. Publish waypoints

### Key Files
```tree
mbot_nav/
    src/
        astar.cpp
        frontier_explorer.cpp
```

---

### Frontier Detection

A cell is considered a frontier if:

- The cell itself is free space
- At least one neighboring cell is unknown


---

## 5. Frontier-Based Exploration and Navigation

Frontier exploration allows the robot to autonomously explore unknown environments.

A frontier is defined as the boundary between known free space and unknown space.

### Exploration Pipeline

1. Detect frontier cells
2. Filter unsafe frontier points
3. Cluster nearby frontier cells
4. Select a frontier goal
5. Plan a path using A*

### Key Files
```tree
mbot_nav/
    src/
        frontier_exploration.cpp
        exploration_node.cpp
        navigation_node.cpp
```

---

### Frontier Clustering

Detected frontier cells are grouped into clusters using a Breadth-First Search (BFS) clustering method.

Each cluster is represented by its centroid.

---

### Goal Selection

The robot selects the closest frontier centroid that satisfies safety constraints.

---

### Navigation Node Responsibilities

- Receive goal poses
- Maintain the obstacle distance grid
- Call the A* planner
- Publish paths and waypoints

