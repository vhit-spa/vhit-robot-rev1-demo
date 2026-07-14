# vhit_robot_demo

Robot description package for the VHIT Robot Rev 1.

This package contains the base robot assets used by the MoveIt configuration package:

- `urdf/vhit_robotics_rev_1.urdf`
- STL meshes in `meshes/`
- RViz configuration in `rviz/`
- A minimal robot-state-publisher launch in `launch/demo.launch.py`

## Installed Assets

The package installs:

```text
urdf/
meshes/
launch/
rviz/
```

These assets are referenced by `vhit_robot_moveit_config`, especially through:

```xml
<xacro:include filename="$(find vhit_robot_demo)/urdf/vhit_robotics_rev_1.urdf" />
```

## Basic Launch

After building the workspace:

```bash
source install/setup.bash
ros2 launch vhit_robot_demo demo.launch.py
```

This launch publishes the robot description using `robot_state_publisher`. The main MoveIt and ros2_control runtime is launched from `vhit_robot_moveit_config`.

## Jog Topic

`vhit_elac_tester` accepts manual actuator commands on
`/elac_tester_node/jog` using `std_msgs/msg/Int8`. The message's `data` field
selects the jog direction:

- `1` moves the target position up by the node's configured `jog_step`.
- `-1` moves the target position down by the configured `jog_step`.

Build and source the tester package:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select vhit_elac_tester
source install/setup.bash
```

Start the tester in manual mode. For offline testing, enable mock hardware:

```bash
ros2 launch vhit_elac_tester elac_tester.launch.py \
  use_mock_hardware:=true \
  automatic_test:=false \
  point_duration_s:=0.25
```

Increment the actuator target once:

```bash
ros2 topic pub --once /elac_tester_node/jog \
  std_msgs/msg/Int8 "{data: 1}"
```

Decrement the actuator target once:

```bash
ros2 topic pub --once /elac_tester_node/jog \
  std_msgs/msg/Int8 "{data: -1}"
```

The tester ignores jog commands until it has received the configured joint's
position from `/joint_states`. Commands other than `1` and `-1` are rejected.
The resulting target is clamped between the tester's `min_position` and
`max_position` parameters.

## Notes

- Keep mesh filenames stable because the URDF uses `package://vhit_robot_demo/...` paths.
- This package should remain mostly robot-description data. MoveIt, ros2_control, and ctrlX Data Layer behavior live in the other packages.
- If the static URDF is regenerated, check that any Gazebo or control-specific content still matches the MoveIt xacro flow.
