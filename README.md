# VHIT Robot Rev 1 Demo

ROS 2 Humble demo workspace for the VHIT Robot Rev 1. The repository contains the robot description, MoveIt 2 configuration, ros2_control integration, a ctrlX Data Layer hardware interface, and snap packaging for deployment on ctrlX.

## Package Documentation

Each ROS package has its own README:

- [vhit_robot_demo](src/vhit_robot_demo/README.md): robot URDF, meshes, and basic visualization assets.
- [vhit_robot_moveit_config](src/vhit_robot_moveit_config/README.md): MoveIt 2, xacro, controller, and launch configuration.
- [vhit_robot_driver](src/vhit_robot_driver/README.md): ros2_control hardware plugin, Data Layer parameters, TCP access, and shared-memory notes.
- [vhit_passthrough_controller](src/vhit_passthrough_controller/README.md): custom ros2_control controller plugin scaffold.
- [vhit_elac_tester](src/vhit_elac_tester/README.md): standalone single-joint ELAC controller tester for mock hardware, real hardware, and the standalone mock Data Layer.

## Repository Layout

```text
.
├── build-workspace.sh
├── build-snap.sh
├── scripts/
│   ├── run-demo.sh
│   └── run-test.sh
├── snap/
│   └── snapcraft.yaml
└── src/
    ├── vhit_robot_demo/
    ├── vhit_robot_moveit_config/
    ├── vhit_robot_driver/
    ├── vhit_passthrough_controller/
    └── vhit_elac_tester/
```

## Local Hardware Build And Run

Use this path for local development on Ubuntu 22.04 with ROS 2 Humble. This can build and launch the stack from a desktop, VM, or WSL workspace, but ctrlX realtime shared-memory access may not work outside the ctrlX runtime environment.

Prerequisites:

- ROS 2 Humble
- `colcon`
- `rosdep`
- MoveIt 2 and ros2_control runtime packages
- ctrlX Data Layer runtime libraries when building the real hardware driver

Build from the repository root:

```bash
source /opt/ros/humble/setup.bash
rosdep install -i --from-path src --rosdistro humble -y
colcon build --merge-install
source install/setup.bash
```

The helper script performs a clean build:

```bash
./build-workspace.sh
```

Run the main headless launch:

```bash
source install/setup.bash
ros2 launch vhit_robot_moveit_config headless.launch.py
```

`headless.launch.py` currently processes the MoveIt xacro with `hardware=real`, so ros2_control loads `vhit_robot_driver/VhitRobotHardwareInterface`.

Important Data Layer limitation for local runs:

- Normal ctrlX Data Layer TCP access can work from a local machine when the ctrlX CORE is reachable.
- EtherCAT realtime shared-memory access is local to the ctrlX runtime environment. A local/WSL process can connect over TCP and read normal Data Layer nodes, but it may still fail when calling `openMemory()` or `getMemoryMap()` for realtime EtherCAT memory.
- A typical symptom is `DL_RT_INVALIDMEMORYMAP` even after the Data Layer client successfully connects.

For the in-depth driver behavior and connection parameters, see the [vhit_robot_driver README](src/vhit_robot_driver/README.md).

For a smaller single-joint controller test, including offline mock hardware and
the standalone `vhit_mock_datalayer` workflow, see the
[vhit_elac_tester README](src/vhit_elac_tester/README.md).

## Snap Build And Run

Use this path for running the demo as a ctrlX app/snap. This is the intended deployment path for realtime shared-memory access, because the process runs inside the ctrlX environment.

Build the snap from the repository root:

```bash
./build-snap.sh
```

The script first builds the ROS workspace, then runs:

```bash
snapcraft clean --destructive-mode
snapcraft pack --build-for=amd64 --verbosity=verbose --destructive-mode
```

Install the generated snap on the target system:

```bash
sudo snap install --dangerous ./vhit-robot-rev1-demo_*.snap
```

The snap app entry point is:

```text
scripts/run-demo.sh
```

It sources the MoveIt runtime content snap, sources this package's setup file, and launches:

```bash
ros2 launch vhit_robot_moveit_config headless.launch.py
```

### Manual Shared-Memory Plug Connection

The snap must have its shared-memory plug connected before the driver can use ctrlX Data Layer shared memory.

Check the current connection:

```bash
snap connections vhit-robot-rev1-demo | grep datalayer
```

Connect the shared-memory plug manually if it is disconnected:

```bash
sudo snap connect vhit-robot-rev1-demo:datalayer-shm rexroth-automationcore:datalayer-shm
```

If the provider snap name differs, inspect it first:

```bash
snap list | grep rexroth
snap connections rexroth-automationcore | grep datalayer-shm
```

The `datalayer-shm` plug must use the `shared-memory` interface. If snap reports a content/shared-memory mismatch, rebuild with a matching shared-memory plug declaration before reconnecting.

More troubleshooting details are in the [vhit_robot_driver README](src/vhit_robot_driver/README.md).

## Hardware Modes

The MoveIt xacro supports these hardware modes:

| Mode | ros2_control plugin |
| --- | --- |
| `mock` | `mock_components/GenericSystem` |
| `gazebo` | `gz_ros2_control/GazeboSimSystem` |
| `real` | `vhit_robot_driver/VhitRobotHardwareInterface` |

`headless.launch.py` selects `real`. Other launches or xacro mappings can select `mock` or `gazebo` for visualization and simulation workflows.

## Controllers

The default ros2_control configuration is:

```text
src/vhit_robot_moveit_config/config/ros2_controllers.yaml
```

It configures:

- `joint_state_broadcaster`
- `vhit_arm_rev1_controller`

MoveIt maps execution to `vhit_arm_rev1_controller` through:

```text
src/vhit_robot_moveit_config/config/moveit_controllers.yaml
```

## Tests

Run package tests after building:

```bash
source install/setup.bash
colcon test --packages-select vhit_robot_driver vhit_passthrough_controller vhit_elac_tester
colcon test-result --verbose
```

## Development Notes

- `build-workspace.sh` removes `build/`, `log/`, and `install/` before building.
- Keep URDF joints, SRDF groups, controller YAML, `elac_mapping.yaml`, and hardware-interface joint names synchronized.
- ros2_control loads every `<ros2_control>` block present in the final expanded `robot_description`.
- `headless.launch.py` is the primary maintained runtime launch for this demo.
