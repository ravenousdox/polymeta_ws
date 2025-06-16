# pm_swarm Workspace

This repository contains a minimal ROS 2 Humble workspace for experiments with multiple ArduPilot quadcopters on Ubuntu 22.04. The single package `pm_swarm` provides launch and parameter files for running MAVLink bridge nodes.

## Package Overview

The `pm_swarm` package is a Python package that declares dependencies on standard ROS 2 interfaces and `pymavlink`. Metadata is defined in `package.xml` and `setup.py`, which lists `mavlink_ros2_arm` as the provided console script.

The package directory `pm_swarm/pm_swarm` is currently empty, so the repository focuses on configuration rather than application logic. Launch files and parameter samples are stored in the `launch` and `params` folders respectively.

## Launch Configuration

`dual_drone_mavlink_bridge.launch.py` starts two bridge nodes. Each node connects over UDP and publishes heartbeats and battery status under its own namespace. The first node uses port `14561` and the second uses port `14562`. Both nodes are returned in the launch description so running the launch file will bring up both bridges.

The parameter file `params.yaml` provides matching settings. For each drone there is a `mavlink_endpoint` entry, along with heartbeat and battery topics and the MAVLink system identifiers.

## Building and Running

After installing ROS 2 Humble and its build tools, build the workspace with `colcon build` and source the generated environment. Running `ros2 launch pm_swarm dual_drone_mavlink_bridge.launch.py` will start the two bridge nodes configured in the launch file. They are meant to communicate with simulated or real drones over the specified UDP ports.

## Tests

The package includes simple test suites that run style and linter checks using `ament_flake8`, `ament_pep257`, and copyright verification. Tests can be executed with `python3 -m pytest`.

