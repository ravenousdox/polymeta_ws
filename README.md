# MAVLink-ROS2 Dual Drone Bridge for ArduPilot on ROS 2 Humble

This workspace provides a minimal setup for running two MAVLink bridge nodes against ArduPilot-based drones. It targets ROS 2 Humble on Ubuntu 22.04 and assumes a MAVLink router or MAVProxy instance is forwarding telemetry.

## Capabilities
- Publishes periodic heartbeat messages and battery information for each drone
- Streams telemetry from ArduPilot to ROS 2 topics
- Offers services for arming and mode control
- Uses separate namespaces (`/drone1`, `/drone2`) to avoid topic conflicts

## System Requirements
- Ubuntu 22.04
- ROS 2 Humble installed and sourced
- ArduPilot >= 4.5 running with MAVLink forwarding
- MAVLink-router or MAVProxy for UDP routing
- Python >= 3.10 with the `pymavlink` library

## Setup
Clone and build the workspace:
```bash
git clone <repository_url> polymeta_ws
cd polymeta_ws
rosdep install --from-paths src -yi
colcon build
source install/setup.bash
```

## Launch and Usage
Start the bridge nodes:
```bash
ros2 launch pm_swarm dual_drone_mavlink_bridge.launch.py
```
Arm each drone via the provided services:
```bash
ros2 service call /drone1/arm_drone std_srvs/srv/SetBool '{data: true}'
ros2 service call /drone2/arm_drone std_srvs/srv/SetBool '{data: true}'
```

## Topics and Messages
- `/drone1/heartbeat_mode`, `/drone2/heartbeat_mode` – heartbeat and mode strings
- `/drone1/battery_status`, `/drone2/battery_status` – `sensor_msgs/BatteryState`

The bridge listens for service requests and forwards MAVLink commands to the corresponding vehicle while mirroring basic status over ROS 2 topics.

## Tests
Run the Python test suite with:
```bash
python3 -m pytest
```
