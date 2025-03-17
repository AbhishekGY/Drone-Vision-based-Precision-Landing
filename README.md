# Drone Vision-based Precision Landing

A ROS package for autonomous precision landing of drones using computer vision. The system detects a landing pad with visual markers and guides the drone to land precisely on the target.

## System Overview

This project implements a vision-based precision landing system for drones using ROS and PX4 Autopilot in a Gazebo simulation environment. The system:

1. Detects a landing pad with distinctive markers using computer vision
2. Calculates the 3D position of the landing target relative to the drone
3. Controls the drone to align with and land on the target

## Requirements

- ROS Noetic
- Gazebo 11
- PX4 Autopilot
- MAVROS
- Python 3
- OpenCV

## Project Structure

```
.
├── CMakeLists.txt -> /opt/ros/noetic/share/catkin/cmake/toplevel.cmake
└── drone_vision_landing/
    ├── CMakeLists.txt
    ├── include/
    │   └── drone_vision_landing/
    ├── launch/
    │   └── precision_landing.launch
    ├── models/
    │   ├── drone_with_camera/
    │   │   ├── model.config
    │   │   └── model.sdf
    │   └── landing_pad/
    │       ├── model.config
    │       └── model.sdf
    ├── package.xml
    ├── scripts/
    │   ├── landing_detector.py
    │   ├── landing_pad_teleop.py
    │   └── px4_sitl_launcher.sh
    ├── src/
    └── worlds/
        ├── landing_pad.world
        └── landing_pad.world.save
```

## Setup Requirements

### PX4 Autopilot
This project requires PX4 Autopilot for drone simulation. Before running:

1. Install PX4 Autopilot by following the [official installation guide](https://docs.px4.io/main/en/dev_setup/building_px4.html#gazebo-classic)

2. **Update the PX4 path in the launcher script:**
   ```bash
   # Open the launcher script
   nano scripts/px4_sitl_launcher.sh
   
   # Update the PX4_PATH variable to your installation path
   # Example:
   PX4_PATH="$HOME/PX4-Autopilot"  # Change this to your actual path
   ```

3. Make the script executable:
   ```bash
   chmod +x scripts/px4_sitl_launcher.sh
   ```

This configuration only needs to be done once after cloning the repository.

## Installation

1. Clone this repository into your catkin workspace:
   ```bash
   cd ~/catkin_ws/src
   git clone https://github.com/AbhishekGY/drone-vision-landing.git
   ```

2. Install dependencies:
   ```bash
   sudo apt-get update
   sudo apt-get install ros-noetic-mavros ros-noetic-mavros-extras
   pip3 install opencv-python numpy
   ```

3. Build the workspace:
   ```bash
   cd ~/catkin_ws
   catkin_make
   ```

4. Make sure PX4-Autopilot is installed in `~/Desktop/drone_landing/src/PX4-Autopilot`
   (Or update the path in `px4_sitl_launcher.sh`)

## Running the Simulation

You can launch the simulation with:

```bash
# Using default PX4 path
roslaunch drone_vision_landing precision_landing.launch

# OR specify your custom PX4 path
roslaunch drone_vision_landing precision_landing.launch px4_path:=/path/to/your/PX4-Autopilot
```

This will start Gazebo, MAVROS, the landing detector, and the PX4 SITL simulation.

## Landing Detection Algorithm

The landing detector node uses OpenCV to detect a red circular landing pad:

1. Converts the image to HSV color space
2. Applies color thresholding to identify red regions
3. Finds contours in the binary mask
4. Identifies the largest contour as the landing pad
5. Calculates the position offset of the landing pad relative to the drone camera
6. Publishes the landing target position for the controller

## Landing Controller

The landing sequence follows these stages:

1. **SEARCH**: Hover at 3m altitude while searching for the landing pad
2. **APPROACH**: Move horizontally to position above the landing pad
3. **ALIGN**: Fine alignment directly above the landing pad at 2m altitude
4. **DESCEND**: Controlled descent while maintaining alignment
5. **LAND**: Final landing using PX4's AUTO.LAND mode

## Nodes

### `landing_detector`

A node that processes camera images to detect the landing pad and control the drone for precision landing.

- **Subscribes to:**
  - `/iris/usb_cam/image_raw` (Image): Camera feed from the drone
  - `/mavros/local_position/pose` (PoseStamped): Current drone position
  - `/mavros/state` (State): Current drone state

- **Publishes to:**
  - `/mavros/setpoint_position/local` (PoseStamped): Position setpoints
  - `/landing_target/position` (PoseStamped): Detected landing target position

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## License

This project is licensed under the MIT License - see the LICENSE file for details.
