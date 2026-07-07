# vhit_elac_tester

Standalone single-joint ELAC test package for exercising `vhit_elac_controller`
and, optionally, `vhit_robot_driver/VhitRobotHardwareInterface`.

The package launches a minimal one-joint robot description, `ros2_control_node`,
`joint_state_broadcaster`, `vhit_elac_controller`, and `elac_tester_node`.
The tester node publishes small cyclic `JointTrajectory` commands to:

```text
/vhit_elac_controller/joint_trajectory
```

## Build

From the `vhit-robot-demo` workspace root:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select vhit_robot_driver vhit_elac_tester
source install/setup.bash
```

## Offline Mock Hardware

Use this mode when you only want to validate the launch file, controller
spawning, trajectory topic, and joint-state feedback without ctrlX Data Layer or
EtherCAT access.

```bash
ros2 launch vhit_elac_tester elac_tester.launch.py use_mock_hardware:=true
```

This loads `mock_components/GenericSystem` through:

```text
config/elac_tester_mock.urdf.xacro
```

## Real Hardware

Use this mode when the process can connect to a real ctrlX Data Layer and the
EtherCAT realtime shared-memory areas are available.

```bash
ros2 launch vhit_elac_tester elac_tester.launch.py use_mock_hardware:=false
```

Optional real-hardware arguments:

```bash
ros2 launch vhit_elac_tester elac_tester.launch.py \
  use_mock_hardware:=false \
  dl_node:=ELAC_node_LAN9253 \
  amplitude:=0.02 \
  period_s:=6.0 \
  point_duration_s:=2.0 \
  publish_rate_hz:=0.5
```

In normal real-hardware mode, `vhit_robot_driver` uses its default Data Layer
connection behavior: IPC inside a snap, or TCP parameters outside a snap.

## Real Driver With Mock Data Layer

Use this mode to exercise the real `vhit_robot_driver` shared-memory read/write
path without ctrlX hardware. Start the standalone mock Data Layer first:

```bash
cd /home/riky/vhit_mock_datalayer
cmake -S . -B build
cmake --build build
cd build
./vhit_mock_datalayer
```

Then launch the ELAC tester in another terminal:

```bash
source /opt/ros/humble/setup.bash
source /home/riky/vhit-robot-demo/install/setup.bash
ros2 launch vhit_elac_tester elac_tester.launch.py \
  use_mock_hardware:=false \
  connection_string:=ipc://
```

This still loads `vhit_robot_driver/VhitRobotHardwareInterface`; only the ctrlX
Data Layer endpoint is redirected to the local IPC mock.

The mock Data Layer provides:

```text
framework/metrics/system/cpu-utilisation-percent
fieldbuses/ethercat/master/instances/ethercatmaster/realtime_data/input
fieldbuses/ethercat/master/instances/ethercatmaster/realtime_data/output
```

and loops the output target-position DINT back into the input actual-position
DINT.

## Launch Arguments

| Argument | Default | Description |
| --- | --- | --- |
| `use_mock_hardware` | `false` | Use ros2_control `mock_components/GenericSystem` instead of `vhit_robot_driver`. |
| `connection_string` | empty | Explicit Data Layer connection string for real-driver mode, for example `ipc://`. |
| `dl_node` | `ELAC_node_LAN9253` | ELAC Data Layer node name used to build PDO variable paths. |
| `initial_position` | `0.0` | Initial joint position used by ros2_control. |
| `amplitude` | `0.02` | Sine-wave command amplitude in radians. |
| `period_s` | `6.0` | Sine-wave period in seconds. |
| `point_duration_s` | `2.0` | Time-from-start for each trajectory point. |
| `publish_rate_hz` | `0.5` | Trajectory publish rate. |

## Expected Success Signs

During a successful run you should see:

- `joint_state_broadcaster` configured and activated.
- `vhit_elac_controller` configured and activated.
- `elac_tester_node` initialized from `/joint_states`.
- repeated `Published target: elac_node=...` messages.

For mock Data Layer runs, `ros2_control_node` should also log that it is using
the explicit Data Layer connection string `ipc://` and that CPU utilization was
read as `0.000000`.
