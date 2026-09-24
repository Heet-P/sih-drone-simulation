# SIH Drone Simulation

A search-and-rescue drone in simulation. An S500 quadcopter (ArduPilot SITL + Gazebo Harmonic, ROS 2 Humble) grid-searches a post-disaster zone. It detects a casualty with an RGB camera by day or a thermal camera at night, verifies the detection, and lands beside them. Everything is driven from a web dashboard.

## Run

```bash
./start.sh            # post-disaster world
./start.sh runway     # flat runway world
```

Open the dashboard (it opens automatically at http://localhost:5000) and press **Start Search & Rescue**. Ctrl+C stops everything. See [command_center/README.md](command_center/README.md) for what each control does, how detection works, and the post-disaster world.

## Layout

- `command_center/`: web dashboard (Flask backend + plain HTML/JS frontend)
- `ardu_ws/`: ArduPilot + Gazebo workspace
  - `person_detector.py`: RGB (YOLOv8m) and thermal (`thermal_best.pt`, trained on HIT-UAV) person detector
  - `src/ardupilot_gz/`: launch files and worlds (`runway.sdf`, `disaster.sdf`)
  - `src/ardupilot_gazebo/`: the S500 quad and the world models (collapsed buildings etc. from osrf/gazebo_models)
  - `deps.repos`: pinned upstream repos that aren't committed (ArduPilot, ros_gz, ...)
- `ros2_ws/src/`: `resq_mavlink_bridge` (mission logic) and `resq_mesh_core`
- `resq_mavlink/`: `mission.json` (search area) and standalone MAVLink scripts

## Setting up from a fresh clone

Requires Ubuntu 22.04 with ROS 2 Humble and Gazebo Harmonic.

```bash
cd ardu_ws/src
vcs import --recursive < ../deps.repos
(cd Micro-XRCE-DDS-Gen && git apply ../../patches/Micro-XRCE-DDS-Gen-gradle.patch)
cd .. && colcon build            # ArduPilot SITL, ros_gz, ardupilot_gz, ...
cd ../ros2_ws && colcon build
pip3 install --user -r ../command_center/backend/requirements.txt
```

`command_center/backend/ros_env.sh` sources both workspaces and sets `GZ_SIM_RESOURCE_PATH` for everything the dashboard launches.
