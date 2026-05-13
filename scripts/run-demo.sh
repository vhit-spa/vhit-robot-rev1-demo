#!/bin/bash

set -e

export MOVEIT_RUNTIME=$SNAP/moveit-runtime

source $MOVEIT_RUNTIME/usr/bin/setup-env.sh

exec $ROS_BASE/opt/ros/humble/bin/ros2 launch \
  moveit_resources_panda_moveit_config \
  demo.launch.py \
  use_rviz:=false