#!/bin/bash

set -e

source /opt/ros/humble/setup.bash

rosdep install -i \
  --from-path src \
  --rosdistro humble \
  -y

rm -rf build log install

colcon build \
  --merge-install