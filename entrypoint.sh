#!/bin/bash
set -e

# Source ROS 2 Humble setup
source /opt/ros/humble/setup.bash

# Source the workspace setup
source /ros2_ws/install/setup.bash

# Export CycloneDDS implementation
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Ensure ROS_DOMAIN_ID is set (defaults to 0 if not provided)
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}

# Execute the command passed into this entrypoint
exec "$@"
