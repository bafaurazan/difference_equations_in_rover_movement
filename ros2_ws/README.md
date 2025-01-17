# Introduction

This project is a ROS 2 package designed to control a rover using Python-based publishers. It provides functionality to send movement commands to the rover's left and right wheels via ROS2 topics. The package includes a Python publisher node, which allows you to test and control the rover's behavior programmatically. The setup and usage instructions guide you through building, sourcing, and testing the package.

**The main features include**:
- Publishing velocity commands to individual wheels (`/diff_drive_controller_left/cmd_vel_unstamped` and `/diff_drive_controller_right/cmd_vel_unstamped`) using `geometry_msgs/Twist`.
- CLI tools for quick testing and debugging.
- A Python publisher for automated control of wheel movements.

## Installation

### Install ROS2 Humble
Follow the official [ROS2 Humble installation guide](https://docs.ros.org/en/humble/Installation.html).

### Setup the Workspace
1. Clone this repository:
```bash
    cd /difference_equations_in_rover_movement/TrailblazerML
```
2. Install dependencies:
    ```bash
    sudo apt update && sudo apt install -y \
      python3-colcon-common-extensions \
      ros-humble-gazebo-ros-pkgs \
      ros-humble-ros2-control \
      ros-humble-ros2-controllers \
      ros-humble-gazebo-ros2-control \
      ros-humble-position-controllers \
      ros-humble-xacro \
      joystick \
      jstest-gtk \
      evtest \
      ros-humble-twist-mux \
      ros-humble-rviz2
    ```

3. Build the workspace:
    ```bash
    colcon build --symlink-install
    source install/setup.bash
    ```

---

## Running the Project
1. install meshes
```
chmod +x scripts/dowload_rover_stl.sh 
scripts/dowload_rover_stl.sh 
```
## Add meshes to Gazebo
1. Go to gazebo folder:
```bash
   mkdir ~/.gazebo/models
   cd ~/.gazebo/models
   mkdir gazebo_viz
```
2. Add meshes:
```bash
  cp -r <Path_to_project>/TrailblazerML/src/gazebo_viz/meshes ./gazebo_viz
```

### Launching Simulation
1. test simulation
```bash
    cd ~/difference_equations_in_rover_movement/TrailblazerML
    source install/local_setup.bash
```
1. Launch the Gazebo simulator with the robot:
```bash
    ros2 launch gazebo_viz launch_sim.launch.py
```
2. To visualize the robot in RViz:
```bash
    ros2 launch gazebo_viz rsp.launch.py
```

### Teleoperation
To control the robot manually:
1. Connect your joystick or set up keyboard teleoperation.
2. Launch the teleoperation node:
```bash
    ros2 launch rover_teleop_twist_joy teleop_twist_launch.py
```
   
---

## 1. After cloning repository or changing publisher:

```
~/ros2_ws/src/rover_control/publisher_member_function.py
```

## 2. build package

```
cd ros2_ws
colcon build
```

# Testing

## 3. Source installation

```
source install/local_setup.bash
```

## 4. opening Python Publisher (To get rover moving effect, you must setup micro-ros)

```
ros2 run rover_control test_node
```

## 5. Open cli Publisher test (Dependency micro-ros setup)

```
//start

ros2 topic pub --once /diff_drive_controller_left/cmd_vel_unstamped geometry_msgs/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

ros2 topic pub --once /diff_drive_controller_right/cmd_vel_unstamped geometry_msgs/Twist "{linear: {x: -2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

//stop

ros2 topic pub --once /diff_drive_controller_left/cmd_vel_unstamped geometry_msgs/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

ros2 topic pub --once /diff_drive_controller_right/cmd_vel_unstamped geometry_msgs/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

# Fast test
```bash
cd ~/difference_equations_in_rover_movement/TrailblazerML
source install/local_setup.bash
ros2 launch gazebo_viz launch_sim.launch.py
```
---
```bash
cd ~/difference_equations_in_rover_movement/ros2_ws
source install/local_setup.bash
ros2 run rover_control diff_test 2.3 1.5 2.0 2.0
# args: [right, left, forward, backward]
```
