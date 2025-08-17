## Features
- **ROS 2 nodes**: `color_detector` (HSV color detection + RViz markers), `arm_controller` (IK + pick/place state machine), optional `loop_targets` (endless motion for demos).
- **URDF + robot_state_publisher** to visualize the arm.
- **RViz** markers for boxes/bins; easy to record a GIF/MP4.
- Fully self-contained; runs smoothly in a VM.

---

## Requirements
- Ubuntu **24.04** + **ROS 2 Jazzy** (desktop)
- Packages:
  ```bash
  sudo apt update
  sudo apt install -y ros-jazzy-robot-state-publisher ros-jazzy-xacro \
                      ros-jazzy-cv-bridge ros-jazzy-image-transport \
                      ros-jazzy-rviz2 python3-opencv python3-numpy
## Demo

![Demo](media/miniproject.gif)
