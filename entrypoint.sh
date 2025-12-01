#!/bin/bash
set -e

# ROS 2 ortamini yukle
source "/opt/ros/humble/setup.bash"

# Eger build alinmissa onu da yukle
if [ -f "/ros2_ws/install/setup.bash" ]; then
  source "/ros2_ws/install/setup.bash"
fi

exec "$@"
