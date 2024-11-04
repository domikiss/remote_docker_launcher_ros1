#!/bin/bash
# Basic entrypoint for ROS / Catkin Docker containers
 
# Source ROS
source /opt/ros/${ROS_DISTRO}/setup.bash
 
# Execute the command passed into this entrypoint
exec "$@"
