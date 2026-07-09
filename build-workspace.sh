#!/bin/bash

set -e

source /opt/ros/jazzy/setup.bash

rosdep install -i \
  --from-path src \
  --rosdistro jazzy \
  -y

rm -rf build log install

colcon build \
  --merge-install