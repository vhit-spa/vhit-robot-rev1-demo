#!/bin/bash

set -e

export MOVEIT_RUNTIME=$SNAP/moveit-runtime

source $MOVEIT_RUNTIME/usr/bin/setup-env.sh

source $SNAP/local_setup.bash

exec colcon test --packages-select vhit_robot_driver