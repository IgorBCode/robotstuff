# ROS 2 Color-Detecting Robot Simulation

This project simulates a mobile robot in **Gazebo Harmonic** using **ROS 2 Jazzy** 

## Project Structure

| Folder | Purpose |
|--------|---------|
| `my_robot_description` | Contains the robot's URDF/Xacro files and simulation configuration |
| `my_robot_bringup` | Launches the full system including robot description, RViz, and bridges |
| `my_robot_vision` | Contains the Python node that detects colors and commands the robot's motion |

---

## Tech Stack

- **ROS 2 Jazzy** (middleware framework)
- **Gazebo Harmonic** (physics simulator)
- **rviz2** (3D visualization)
- **OpenCV** (image processing)
- **cv_bridge** (bridge between ROS Image and OpenCV)
- **Gazebo-ROS 2 bridge** (connects Gazebo topics to ROS 2)

## Required Dependencies

Ensure the following ROS and Gazebo plugins/libraries are installed:

### ROS 2 Packages
```bash
sudo apt install \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-cv-bridge \
  ros-jazzy-image-transport \
  ros-jazzy-nav-msgs \
  ros-jazzy-geometry-msgs \
  ros-jazzy-sensor-msgs \
  python3-opencv
```

## Build and run commands
- cd ~/ros2_ws
- colcon build
- source install/setup.bash
- ros2 launch my_robot_bringup my_robot_gazebo.launch.xml
