#!/bin/bash

set -e

export MOVEIT_RUNTIME=$SNAP/moveit-runtime

source $MOVEIT_RUNTIME/usr/bin/setup-env.sh

source $SNAP/local_setup.bash

exec $ROS_BASE/opt/ros/humble/bin/ros2 launch \
  vhit_robot_moveit_config \
  demo.launch.py