# ROS Workspace

[![Build](https://github.com/ClemensElflein/open_mower_ros/actions/workflows/build-image.yaml/badge.svg)](https://github.com/ClemensElflein/open_mower_ros/actions/workflows/build-image.yaml)

This folder is the ROS workspace, which should be used to build the OpenMower ROS software.
This repository contains the ROS package for controlling the OpenMower.

## Packages

### Application
- **open_mower** - Main launch files and configuration
- **mower_logic** - High-level mowing behaviors (mowing, docking, undocking, area recording)
- **mower_map** - Map management service (JSON-based map storage, occupancy grid generation)
- **mower_comms_v1 / mower_comms_v2** - Hardware communication with the mower mainboard
- **mower_simulation** - Simulation environment for testing without hardware
- **mower_msgs** - Custom ROS2 message and service definitions
- **mower_utils** - Utility tools (pose converter, planner test)

### Navigation
- **global_planner** - Global path planner (A*/Dijkstra on costmap)
- **ftc_local_planner** - Follow-the-carrot local planner with costmap support
- **slic3r_coverage_planner** - Coverage path planner based on Slic3r 3D printer slicing

### Move Base Flex (ported from ROS1)
- **mbf_costmap_nav** - Costmap navigation server
- **mbf_costmap_core** - Costmap-aware plugin interfaces (CostmapPlanner, CostmapController, CostmapRecovery)
- **mbf_abstract_nav / mbf_abstract_core** - Abstract navigation server and plugin base classes
- **mbf_msgs** - MBF action and service definitions
- **mbf_utility** - MBF utility functions
- **mbf_simple_nav / mbf_simple_core** - Simple (non-costmap) navigation server
- **move_base_flex** - Metapackage

### Hardware Drivers
- **xbot_framework** - Communication framework for xBot hardware services
- **xbot_driver_gps** - GPS driver (u-blox and NMEA)
- **xbot_positioning** - Robot localization and positioning
- **xbot_monitoring** - Sensor monitoring and MQTT bridge
- **xbot_remote** - WebSocket remote control
- **xbot_msgs / xbot_rpc** - xBot message and RPC definitions
- **xesc** - xESC motor controller metapackage
- **xesc_driver / xesc_interface / xesc_msgs** - xESC driver, interface and messages
- **xesc_2040_driver / xesc_yfr4_driver** - Hardware-specific xESC drivers
- **serial** - Serial port library

## Container images: Default vs Legacy

If your robot runs the latest OpenMower OS (v2): use the images without prefix or suffix (e.g. `latest`, `v1.2.3`).
These images only contain the OpenMower ROS stack and expect the OS to provide web and MQTT services (for example via your system’s compose setup).

If your robot runs an old version of OpenMower OS v1 (Legacy): use the legacy image.
The OS doesn't provide web and MQTT services, so the image contains nginx and mosquitto to provide these services inside the container.
The Docker images have a `-legacy` suffix or `releases-` prefix: (e.g. `releases-edge`, `v1.2.3-legacy`).

## Prerequisites

- Ubuntu 24.04
- [ROS2 Jazzy](https://docs.ros.org/en/jazzy/Installation.html)
- nav2_costmap_2d: `sudo apt install ros-jazzy-nav2-costmap-2d`

## Getting Started

### Install Dependencies

```bash
sudo apt install python3-rosdep
sudo rosdep init  # only needed once
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### Build

```bash
source /opt/ros/jazzy/setup.bash
colcon build
```

### Source the Workspace

```bash
source install/setup.bash
```

### Configure

```bash
cp src/open_mower/config/mower_config.sh.example mower_config.sh
# Edit mower_config.sh to match your hardware setup
source mower_config.sh
```

### Launch

**On real hardware:**

```bash
ros2 launch open_mower open_mower_launch.py
```

**Simulation:**

```bash
ros2 launch open_mower sim_mower_logic_launch.py
```

## Available Launch Files

| Launch file | Description |
|---|---|
| `open_mower_launch.py` | Full system (hardware) |
| `sim_mower_logic_launch.py` | Simulated mowing logic |
| `sim_navigation_launch.py` | Simulated navigation only |
| `planner_launch.py` | Planner test |
| `gamepad_and_gps_launch.py` | Gamepad control with GPS |
| `remote_rviz_launch.py` | Remote RViz visualization |

## Docker

### Build

```bash
docker build -t openmower-ros2 -f docker/Dockerfile .
```

### Run

```bash
docker run --rm --network host --privileged -v /dev:/dev openmower-ros2
```

### Run with custom configuration

```bash
docker run --rm --network host --privileged -v /dev:/dev -v /path/to/your/params:/config/params -e MOWER=CUSTOM -e PARAMS_PATH=/config/params openmower-ros2
```

### Debug mode (verbose logging)

```bash
docker run --rm --network host --privileged -v /dev:/dev -e DEBUG=true openmower-ros2
```

## Notes

- Move Base Flex (`mbf_costmap_core`, `mbf_costmap_nav`) has been ported to ROS2 with `nav2_costmap_2d` as the costmap backend.
- If the map has no docking point set, planning may fail when attempting to approach the dock.

## License

This work is licensed under the [GNU General Public License version 3](https://www.gnu.org/licenses/gpl-3.0.html). See the [LICENSE](LICENSE) file for details.
