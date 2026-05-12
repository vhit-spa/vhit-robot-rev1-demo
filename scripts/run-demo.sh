#!/bin/bash

set -e

source $SNAP/usr/bin/setup-env.sh

exec $ROS_BASE/opt/ros/humble/bin/ros2 launch \
  moveit_resources_panda_moveit_config \
  demo.launch.py \
  use_rviz:=false