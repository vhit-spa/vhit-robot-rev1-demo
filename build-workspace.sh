#!/bin/bash

set -e

source /opt/ros/humble/setup.bash

vcs import src < vhit_robot_demo.repos

rosdep install -i \
  --from-path src \
  --rosdistro humble \
  -y

rm -rf build log install

colcon build \
  --merge-install