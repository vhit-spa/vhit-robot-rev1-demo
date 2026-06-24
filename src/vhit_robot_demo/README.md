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

## Notes

- Keep mesh filenames stable because the URDF uses `package://vhit_robot_demo/...` paths.
- This package should remain mostly robot-description data. MoveIt, ros2_control, and ctrlX Data Layer behavior live in the other packages.
- If the static URDF is regenerated, check that any Gazebo or control-specific content still matches the MoveIt xacro flow.

