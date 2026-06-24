# vhit_passthrough_controller

Custom ros2_control controller package for the VHIT robot.

The package exports this controller plugin:

```text
vhit_passthrough_controller/VhitPassthroughController
```

## Current Status

This controller is present as a scaffold for a future passthrough trajectory controller.

Implemented pieces:

- pluginlib export
- ROS 2 lifecycle callbacks
- parameter reads for joint and interface lists
- `FollowJointTrajectory` action server creation

Still incomplete:

- command interface selection
- state interface selection
- trajectory transfer logic
- controller `update()` behavior
- goal completion and tolerance handling

The default MoveIt/ros2_control configuration currently uses:

```text
joint_trajectory_controller/JointTrajectoryController
```

not this custom controller.

## Expected Parameters

The controller reads these parameters during `on_init()`:

```yaml
joint_names:
  - rotbase
  - shoulder_joint
  - elbow
  - wrist_pitch
  - wrist_roll

command_interfaces:
  - position

state_interfaces:
  - position
```

Before this controller can be used as the active arm controller, the controller implementation and `ros2_controllers.yaml` must be updated together.

## Action Server

The controller creates a `FollowJointTrajectory` action server at:

```text
/<controller_node_name>/follow_joint_trajectory
```

This matches the action namespace shape used by MoveIt controllers, but the controller logic still needs to forward accepted trajectories into command interfaces.

## Tests

After building:

```bash
source install/setup.bash
colcon test --packages-select vhit_passthrough_controller
colcon test-result --verbose
```

