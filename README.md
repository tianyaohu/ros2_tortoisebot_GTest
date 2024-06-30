# ROS2 TortoiseBot GTest 🐢

## Overview 🌟
Welcome to the automated testing universe of the ROS2 TortoiseBot! This repository houses the mighty GTest (Google Test) implementations tailored for the ROS2 TortoiseBot. Designed to ensure the robustness and reliability of the Action Server `/tortoisebot_waypoints`, this suite comprises essential tests, including positional accuracy and orientation precision checks.

## Features 🚀
- **Automated Testing**: Empower your TortoiseBot with automated tests to confirm its operational excellence.
- **ROS2 Integration**: Seamlessly woven into the ROS2 fabric, these tests offer a native testing experience in the ROS ecosystem.

## Installation and Usage 🛠️

### Prerequisites
- ROS2 Galactic 🌌
- GTest library 📚

### Installation
Time to get everything set up! Follow these steps:
```
cd ~/ros2_ws
git clone https://github.com/tianyaohu/ros2_tortoisebot_GTest.git
git clone https://github.com/rigbetellabs/tortoisebot.git
```
Build and source work space:
```
colcon build; source install/setup.bash
```

### Start Gazebo Simulation Simulation
In *Terminal #1*
```
source ~/ros2_ws/install/setup.bash
ros2 launch tortoisebot_bringup bringup.launch.py use_sim_time:=True
```
### Start Waypoint Action Server
In *Terminal #2*
```
source ~/ros2_ws/install/setup.bash
ros2 run tortoisebot_waypoints tortoisebot_action_server
```

## Start Testing🎯
In *Terminal #3*
```
colcon build --packages-select tortoisebot_waypoints
```
Expected result:

In *Terminal #3*
```
#                                                 This flag here is for printing test results
colcon test --packages-select tortoisebot_waypoints --event-handler=console_direct+
```
Expected result:
{GIF HERE}

### View Test Result📊
In *Terminal #3*
```
colcon test-result --verbose
```
Expeceted result:

