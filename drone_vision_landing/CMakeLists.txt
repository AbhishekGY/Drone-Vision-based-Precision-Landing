cmake_minimum_required(VERSION 3.0.2)
project(drone_vision_landing)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  cv_bridge
  image_transport
  mavros
  mavros_msgs
  message_generation
  gazebo_ros
  gazebo_msgs
)

find_package(OpenCV REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs geometry_msgs cv_bridge image_transport mavros mavros_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# Python scripts don't need to be compiled, but we make them executable in the install step
catkin_install_python(PROGRAMS
  scripts/landing_detector.py
  scripts/landing_pad_teleop.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install launch and world files
install(DIRECTORY launch worlds models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
