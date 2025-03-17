#!/bin/bash

# Use environment variable if set, otherwise use default path
PX4_PATH=${PX4_AUTOPILOT_PATH:-"$HOME/Desktop/drone_landing/src/PX4-Autopilot"}

# Check if the path exists
if [ ! -d "$PX4_PATH" ]; then
    echo "Error: PX4-Autopilot not found at $PX4_PATH"
    echo "Please set the correct path using the PX4_AUTOPILOT_PATH environment variable"
    echo "or update the default path in this script"
    exit 1
fi

# Navigate to PX4-Autopilot directory
cd "$PX4_PATH" || exit 1

# Set environment variables
export PX4_SIM_MODEL=none
export HEADLESS=0

# Launch PX4 SITL connected to already-running Gazebo
exec make px4_sitl none
