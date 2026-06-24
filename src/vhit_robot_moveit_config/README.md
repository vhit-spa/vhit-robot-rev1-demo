# vhit_robot_moveit_config

MoveIt 2 and ros2_control configuration package for `vhit_robotics_rev_1`.

This package is the primary runtime package for the demo. It combines the base robot description from `vhit_robot_demo` with MoveIt configuration, ros2_control hardware selection, controller YAML, and launch files.

## Main Files

```text
config/
├── elac_mapping.yaml
├── initial_positions.yaml
├── joint_limits.yaml
├── kinematics.yaml
├── moveit_controllers.yaml
├── ros2_controllers.yaml
├── vhit_robotics_rev_1.gazebo.xacro
├── vhit_robotics_rev_1.ros2_control.xacro
├── vhit_robotics_rev_1.srdf
└── vhit_robotics_rev_1.urdf.xacro
```

## Launch Files

`headless.launch.py`

Primary maintained runtime launch. It:

- processes `vhit_robotics_rev_1.urdf.xacro`
- passes `hardware=real`
- passes `config/elac_mapping.yaml` into the xacro
- starts `robot_state_publisher`
- starts `controller_manager/ros2_control_node`
- spawns `joint_state_broadcaster`
- spawns `vhit_arm_rev1_controller`
- starts `move_group`

Run it with:

```bash
source install/setup.bash
ros2 launch vhit_robot_moveit_config headless.launch.py
```

`rviz.launch.py`

Visualization launch for RViz and MoveIt display configuration. It uses the same xacro path with mock hardware intent. If the xacro requires additional arguments, such as `elac_mapping_file`, make sure the launch passes them before using it as a runtime entry point.

Generated MoveIt wrappers:

```text
demo.launch.py
move_group.launch.py
moveit_rviz.launch.py
rsp.launch.py
setup_assistant.launch.py
spawn_controllers.launch.py
static_virtual_joint_tfs.launch.py
warehouse_db.launch.py
```

These are useful for MoveIt workflows, but `headless.launch.py` is the launch file that currently carries the custom real-hardware mappings.

## Hardware Selection

`config/vhit_robotics_rev_1.urdf.xacro` exposes:

```xml
<xacro:arg name="hardware" default="mock" />
```

The ros2_control xacro maps this value to:

| `hardware` value | Plugin |
| --- | --- |
| `mock` | `mock_components/GenericSystem` |
| `gazebo` | `gz_ros2_control/GazeboSimSystem` |
| `real` | `vhit_robot_driver/VhitRobotHardwareInterface` |

`headless.launch.py` currently selects `real`.

## EtherCAT Mapping

`config/elac_mapping.yaml` maps MoveIt/URDF joint names to EtherCAT Data Layer node names:

```yaml
elac_mapping:
  rotbase: "ELAC_node_LAN9253"
  shoulder_joint: "ELAC_node_LAN9253"
  elbow: "ELAC_node_LAN9253"
  wrist_pitch: "ELAC_node_LAN9253"
  wrist_roll: "ELAC_node_LAN9253"
```

The xacro turns those values into per-joint ros2_control parameters named `DL_node`. The hardware driver uses those values to build Data Layer variable names for actual and target position PDOs.

## Controllers

`config/ros2_controllers.yaml` configures ros2_control:

- `joint_state_broadcaster`
- `vhit_arm_rev1_controller`

`vhit_arm_rev1_controller` currently uses:

```text
joint_trajectory_controller/JointTrajectoryController
```

`config/moveit_controllers.yaml` maps MoveIt trajectory execution to the same controller through the `follow_joint_trajectory` action namespace.

## Notes

- Keep joint names synchronized across URDF, SRDF, `joint_limits.yaml`, `ros2_controllers.yaml`, `moveit_controllers.yaml`, and `elac_mapping.yaml`.
- If connection parameters for the real hardware driver are needed, expose them through the xacro and add `<param>` entries under the real `<hardware>` block. See `vhit_robot_driver/README.md`.

