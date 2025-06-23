#!/bin/bash
set -e

: "${ROS2_VER:=humble}"

source "/opt/ros/${ROS2_VER}/setup.bash"

if [ -f "/ros2_ws/install/setup.bash" ]; then
    source "/ros2_ws/install/setup.bash"
fi

export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}

exec "$@"