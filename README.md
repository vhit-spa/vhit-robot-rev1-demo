# VHIT Robot Rev 1 Demo

Integration and deployment repository for the VHIT Robot Rev 1 ROS 2 stack.

This repository no longer contains copies of the ROS packages. Instead,
`vhit_robot_demo.repos` pins compatible releases of the standalone driver,
robot-description, and MoveIt-configuration repositories. The build scripts
assemble those packages into a local ROS workspace and package the resulting
merged installation as a ctrlX-compatible snap. A small integration package,
`vhit_robot_demo`, owns the parameterized headless launch used by deployment.

## Source repositories

| Repository | Packages provided | Version used here |
| --- | --- | --- |
| [VHIT Robot Driver](https://github.com/vhit-spa/VHIT-Robot-Driver) | `vhit_robot_driver` | `v1.0.0` |
| [VHIT Robot Description](https://github.com/vhit-spa/VHIT-Robot-Description) | `vhit_robot_description`, `vhit_robot_moveit_config` | `v1.0.0` |

The ELAC tester and passthrough controller are maintained and deployed
separately; the current demo does not launch them, so they are not included in
`vhit_robot_demo.repos`.

## Repository layout

```text
vhit-robot-demo/
├── scripts/
│   └── run-demo.sh
├── snap/
│   └── snapcraft.yaml
├── src/
│   └── vhit_robot_demo/
│       └── launch/
│           └── headless.launch.py
├── build-snap.sh
├── build-workspace.sh
└── vhit_robot_demo.repos
```

The integration wrapper under `src/vhit_robot_demo` is tracked. The driver and
description repositories imported alongside it are ignored.

## Prerequisites

For a local build on Ubuntu 22.04:

- ROS 2 Humble;
- `colcon` and `rosdep`;
- `vcstool` (`vcs` command);
- MoveIt 2 and ros2_control dependencies; and
- the configured ctrlX Data Layer package source required by
  `vhit_robot_driver`.

Install the standard workspace tools if needed:

```bash
sudo apt update
sudo apt install python3-colcon-common-extensions python3-rosdep python3-vcstool
```

Initialize rosdep once per machine if it has not already been initialized:

```bash
sudo rosdep init
rosdep update
```

## Build and run locally

Clone the integration repository and select the desired branch or release:

```bash
git clone https://github.com/vhit-spa/vhit-robot-rev1-demo.git
cd vhit-robot-rev1-demo
```

Build the complete workspace:

```bash
./build-workspace.sh
```

The script performs these steps:

1. sources ROS 2 Humble;
2. imports the repositories from `vhit_robot_demo.repos` into `src/`;
3. installs resolvable system dependencies with rosdep;
4. removes previous `build/`, `install/`, and `log/` directories; and
5. creates a merged colcon installation in `install/`.

Source the resulting workspace in every terminal used to run ROS commands:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

### Complete MoveIt demo

Start the generated MoveIt demo, including RViz:

```bash
ros2 launch vhit_robot_moveit_config demo.launch.py
```

This uses the default mock-hardware configuration and is the recommended local
functional test.

### Headless demo

Start robot-state publishing, ros2_control, the configured controllers, and
`move_group` without RViz:

```bash
ros2 launch vhit_robot_demo headless.launch.py
```

The integration wrapper defaults to mock hardware for safe local use. Select a
mode explicitly with the `hardware` launch argument:

```bash
ros2 launch vhit_robot_demo headless.launch.py hardware:=mock
ros2 launch vhit_robot_demo headless.launch.py hardware:=gazebo
ros2 launch vhit_robot_demo headless.launch.py hardware:=real
```

When `hardware:=real` is selected, the wrapper loads
`vhit_robot_driver/VhitRobotHardwareInterface` and automatically supplies the
ELAC mapping file from `vhit_robot_moveit_config`.

### RViz-only visualization

Start RViz with the URDF, SRDF, kinematics, planning pipelines, and joint
limits, but without `move_group` or ros2_control:

```bash
ros2 launch vhit_robot_moveit_config moveit_rviz.launch.py
```

Warnings about unavailable MoveIt action servers are expected in this mode.

## Run tests

After building and sourcing the workspace:

```bash
colcon test
colcon test-result --verbose
```

## Real-hardware integration

The robot Xacro supports these hardware selections:

| Mode | ros2_control plugin |
| --- | --- |
| `mock` | `mock_components/GenericSystem` |
| `gazebo` | `gz_ros2_control/GazeboSimSystem` |
| `real` | `vhit_robot_driver/VhitRobotHardwareInterface` |

For `hardware:=real`, the integration wrapper passes the ELAC mapping file from
`vhit_robot_moveit_config/config/elac_mapping.yaml`. The physical driver
requires access to the ctrlX Data Layer and EtherCAT realtime shared memory.

The standalone MoveIt configuration remains unchanged; hardware selection for
the deployed headless system belongs to the integration wrapper.

Normal Data Layer nodes may be reachable over TCP from a workstation, but
EtherCAT realtime shared memory is local to the ctrlX runtime. A remote process
can therefore connect successfully and still fail when opening or mapping the
realtime memory area.

## Build the snap

Snap deployment packages the same merged workspace used by the local build.
Install Snapcraft on the build machine and run:

```bash
cd ~/vhit-robot-demo
./build-snap.sh
```

The script first calls `build-workspace.sh`, then runs:

```bash
snapcraft clean --destructive-mode
snapcraft pack \
  --build-for=amd64 \
  --verbosity=verbose \
  --destructive-mode
```

The current build script produces:

```text
vhit-robot-rev1-demo_1.3.10_amd64.snap
```

`snapcraft.yaml` declares both amd64 and arm64, but `build-snap.sh` currently
targets amd64. Produce arm64 artifacts through an appropriate native or
cross-build workflow rather than relabeling an amd64 build.

Destructive mode builds against the host environment. Use a dedicated build
machine or container if the host must remain isolated from Snapcraft build
changes.

## Install the snap

Copy the generated snap to the target and install the unsigned local build:

```bash
sudo snap install --dangerous ./vhit-robot-rev1-demo_1.3.10_amd64.snap
```

Inspect its content and hardware-interface connections:

```bash
snap connections vhit-robot-rev1-demo
```

The application uses these interfaces:

- `ros-base` and `moveit-runtime` for the ROS and MoveIt content snaps;
- `network` and `network-bind` for ROS communication;
- `process-control` for the runtime processes;
- `datalayer` for the ctrlX Data Layer content; and
- `datalayer-shm` for realtime shared-memory access.

Connect any content plugs that are not auto-connected to their provider snaps.
For the standard ctrlX Automation Core provider, the Data Layer connections
are:

```bash
sudo snap connect \
  vhit-robot-rev1-demo:datalayer \
  rexroth-automationcore:datalayer

sudo snap connect \
  vhit-robot-rev1-demo:datalayer-shm \
  rexroth-automationcore:datalayer-shm
```

If the provider snap has a different name, discover its slots first:

```bash
snap list
snap connections rexroth-automationcore | grep datalayer
```

## Run the installed snap

Start the application in the foreground:

```bash
vhit-robot-rev1-demo
```

The snap wrapper:

1. initializes the `moveit-runtime` content environment;
2. sources the merged workspace installed in the snap; and
3. launches `vhit_robot_demo/headless.launch.py` with `hardware:=real`.

Stop it with `Ctrl-C`.

Unlike the local wrapper default, the snap always selects real hardware. It
loads `vhit_robot_driver/VhitRobotHardwareInterface`, so the Data Layer content
and shared-memory plugs must be connected before starting the application.

## Troubleshooting

### A source repository cannot be imported

Confirm that the release tags referenced by `vhit_robot_demo.repos` exist and
that the build machine can access GitHub:

```bash
vcs validate --input vhit_robot_demo.repos
vcs import src < vhit_robot_demo.repos
```

### rosdep cannot resolve `vhit_robot_driver`

Run the import before rosdep. The driver is a source dependency, not a rosdep
system key:

```bash
vcs import src < vhit_robot_demo.repos
rosdep install --from-paths src --ignore-src -r -y
```

### The snap cannot access realtime memory

Check both Data Layer connections:

```bash
snap connections vhit-robot-rev1-demo | grep datalayer
```

Verify that `datalayer-shm` connects a `shared-memory` plug to a compatible
`shared-memory` slot. A content/shared-memory interface mismatch requires a
manifest correction and a rebuilt snap.

## Updating component versions

Update versions in `vhit_robot_demo.repos` only after the corresponding
standalone repositories have been tagged and tested together. Then validate the
integration from a clean checkout before tagging a new demo release.
