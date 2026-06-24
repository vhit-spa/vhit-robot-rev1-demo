# vhit_robot_driver

ros2_control hardware interface package for the VHIT Robot Rev 1.

The package exports this hardware plugin:

```text
vhit_robot_driver/VhitRobotHardwareInterface
```

The MoveIt config selects this plugin when the xacro argument `hardware` is set to `real`.

## What The Driver Does

At initialization time, the driver:

- reads the ros2_control hardware and joint configuration
- validates that each joint has a `DL_node` parameter
- creates position state and command interfaces
- builds Data Layer variable names for actual and target position PDOs

At configure time, the driver:

- starts the ctrlX Data Layer system
- creates a Data Layer client
- verifies the connection by reading `framework/metrics/system/cpu-utilisation-percent`
- opens Data Layer shared-memory helpers for EtherCAT input and output realtime data
- reads the memory maps and checks that all configured PDO variables are present

Current implementation status:

- `on_init()` and `on_configure()` perform connection and mapping validation.
- `read()` and `write()` currently return `OK` and still need the runtime read/write logic wired to `SharedMemoryArea::readVariable()` and `SharedMemoryArea::writeVariable()`.

## Required Joint Parameter

Each ros2_control joint must provide:

```xml
<param name="DL_node">...</param>
```

The MoveIt config generates this from:

```text
src/vhit_robot_moveit_config/config/elac_mapping.yaml
```

Example:

```yaml
elac_mapping:
  rotbase: "ELAC_node_LAN9253"
  shoulder_joint: "ELAC_node_LAN9253"
  elbow: "ELAC_node_LAN9253"
  wrist_pitch: "ELAC_node_LAN9253"
  wrist_roll: "ELAC_node_LAN9253"
```

For each joint, the driver builds:

```text
<DL_node>/PdoTx1_MappingParameters.Position_Actual_Value
<DL_node>/PdoRx1_MappingParameters.Target_Position
```

Those names must exist in the ctrlX EtherCAT realtime memory maps.

## Optional Connection Parameters

The driver supports these optional hardware parameters:

| Parameter | Default | Used when |
| --- | --- | --- |
| `ip_address` | `10.0.2.2` | Non-snap TCP Data Layer connection |
| `username` | `boschrexroth` | Non-snap TCP Data Layer connection |
| `password` | `boschrexroth` | Non-snap TCP Data Layer connection |
| `ssl_port` | `8443` | Non-snap TCP Data Layer connection |

Inside a snap, the driver detects the `SNAP` environment variable and uses the local IPC Data Layer connection instead of these TCP parameters.

To override the TCP defaults, add parameters under the real hardware block in `vhit_robotics_rev_1.ros2_control.xacro`, or expose them as xacro arguments and pass them from launch:

```xml
<xacro:if value="${hardware == 'real'}">
  <hardware>
    <plugin>vhit_robot_driver/VhitRobotHardwareInterface</plugin>
    <param name="ip_address">192.168.1.1</param>
    <param name="username">boschrexroth</param>
    <param name="password">boschrexroth</param>
    <param name="ssl_port">443</param>
  </hardware>
</xacro:if>
```

## Data Layer Access Modes

The driver has two different Data Layer paths depending on where it runs.

### Non-Snap / TCP

When `SNAP` is not set, the driver creates a TCP Data Layer client using the optional connection parameters.

This can work for normal Data Layer reads from a desktop, VM, or WSL environment. For example, the driver may successfully connect and read CPU utilization.

However, TCP Data Layer access is not the same as local realtime shared-memory access. A remote process can read Data Layer nodes over TCP but cannot attach to the ctrlX EtherCAT realtime shared-memory image. In that case, the memory-map read may fail with:

```text
DL_RT_INVALIDMEMORYMAP
```

This does not necessarily mean that the TCP connection is broken. It usually means the process is not running in the same shared-memory environment as the ctrlX realtime provider.

### Snap / IPC And Shared Memory

When `SNAP` is set, the driver uses the local Data Layer IPC connection. This is the intended deployment path for realtime shared-memory access on ctrlX hardware.

The driver opens these shared-memory areas:

```text
fieldbuses/ethercat/master/instances/ethercatmaster/realtime_data/input
fieldbuses/ethercat/master/instances/ethercatmaster/realtime_data/output
```

For each area it reads:

```text
<area>/map
```

Then it calls `openMemory()` and `getMemoryMap()` before accessing variables.

The snap must have the Data Layer shared-memory plug connected:

```bash
snap connections vhit-robot-rev1-demo | grep datalayer
sudo snap connect vhit-robot-rev1-demo:datalayer-shm rexroth-automationcore:datalayer-shm
```

Use the exact provider snap name shown on the target:

```bash
snap list | grep rexroth
snap connections rexroth-automationcore | grep datalayer-shm
```

The `datalayer-shm` plug must use the `shared-memory` interface. If snap reports that a `content` plug cannot connect to a `shared-memory` slot, fix the snap declaration, rebuild, reinstall, and connect again.

## `DL_RT_INVALIDMEMORYMAP` Checklist

If the driver connects to the Data Layer but fails while refreshing the memory map:

1. Check that `vhit-robot-rev1-demo:datalayer-shm` is connected to the ctrlX `datalayer-shm` slot.
2. Confirm the snap plug interface is `shared-memory`, not `content`.
3. Confirm the EtherCAT Master app and EtherCAT realtime provider are running.
4. Test the input memory area before the output memory area.
5. Verify that `<area>/map` is a valid MemoryMap FlatBuffer before calling `getMemoryMap()`.
6. On ctrlX CORE Virtual, confirm that the virtual environment actually provides the EtherCAT realtime shared-memory backend required by the driver.

Normal Data Layer node visibility does not guarantee that the corresponding shared-memory backend is valid and attachable.

## Tests

After building:

```bash
source install/setup.bash
colcon test --packages-select vhit_robot_driver
colcon test-result --verbose
```

