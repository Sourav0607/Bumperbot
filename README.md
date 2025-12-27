# Bumperbot - ROS2 Robot Description

A ROS2 workspace for learning robot simulation and control. This project contains a differential drive robot with wheel and caster configurations for use in Gazebo simulation and RViz visualization.

## Contents

- **bumperbot_description**: Robot URDF/Xacro files, meshes, and launch files for visualization and simulation

## Prerequisites

- ROS2 Humble
- Gazebo (gz-sim)
- RViz2

## Building

```bash
cd ~/bumperbot_ws
colcon build
source install/setup.bash
```

## Usage

### Visualize in RViz

```bash
ros2 launch bumperbot_description display.launch.py
```

### Launch in Gazebo

```bash
ros2 launch bumperbot_description gazebo.launch.py
```

## Robot Structure

- **Base**: Main robot chassis
- **Wheels**: Two continuous drive wheels (left and right)
- **Casters**: Front and rear caster wheels for stability



## Repository

[GitHub Repository](https://github.com/Sourav0607/Bumperbot)
