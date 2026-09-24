# SIH-Drone Command Center

A web dashboard for the SAR drone simulation. The whole project lives in
`~/Desktop/SIH-Drone-Simulation`:

- `ardu_ws/` — ArduPilot + Gazebo workspace (drone model, world, SITL), plus `person_detector.py` and its models
- `ros2_ws/` — `resq_mavlink_bridge` (mission logic) and `resq_mesh_core`
- `resq_mavlink/` — `mission.json` (search area) and standalone MAVLink scripts
- `command_center/` — this dashboard

`~/ardu_ws`, `~/ros2_ws` and `~/resq_mavlink` are links into this folder.
Keep them: the compiled `build/` and `install/` trees and `~/.bashrc` still
refer to the old paths, and without the links the sim wouldn't start until
a full rebuild of both workspaces. Rebuilding individual packages from the
new folder works normally (tested).

Every button drives the real ArduPilot + Gazebo simulation on this machine.

## Run it

One command starts everything (dashboard, Gazebo + SITL + RViz, mission bridge, live feed) and opens the dashboard; Ctrl+C stops it all:

```bash
~/Desktop/SIH-Drone-Simulation/start.sh           # post-disaster world
~/Desktop/SIH-Drone-Simulation/start.sh runway    # flat runway world
```

Then press **Start Search & Rescue**. Or start the pieces yourself:

```bash
cd ~/Desktop/SIH-Drone-Simulation/command_center
pip3 install --user -r backend/requirements.txt   # first time only
python3 backend/server.py
```

Open `http://localhost:5000`.

## What each button does

- **World** picker — which Gazebo world **Launch Simulation** starts: *Post-disaster zone* (`disaster.sdf`, default) or *Runway (flat)* (`runway.sdf`). Locked while a sim runs. See "Post-disaster world" below.
- **Launch Simulation** — runs `ros2 launch ardupilot_gz_bringup s500_quad_runway.launch.py world:=<world> rviz:=true use_gz_tf:=true` and starts the `mavlink_bridge` ROS2 node (telemetry + mission services). Opens the real Gazebo and RViz windows.
- **Stop Simulation** — stops both (whole process groups, so nothing is orphaned even if `ros2 launch` already died), then a targeted `pkill` sweep for anything left over.
- **Switch to Thermal / Night** — toggles day/night at any time, even mid-mission: the detector switches camera and model, the Gazebo world goes to night (sun → dim moonlight, no shadows), and the dashboard switches to a dark red night theme with a NIGHT badge. Switching back restores daylight.
- **Start / Stop Live Feed** — runs `ardu_ws/person_detector.py`. The video panel shows the drone camera with a banner for the last detection; when a detection completes, the exact analysed frame is shown for a moment with its boxes.
- **Start Search & Rescue** — calls `/resq/mission/start`: GUIDED → arm → takeoff to 8m → **lawnmower grid search** over the area in `~/resq_mavlink/mission.json` at 2 m/s → on any detection, **stop, fly over it and hover to verify** → once the detector confirms it over several frames, **land 2.5m beside the person** (not on them). A candidate that turns out not to be a person is rejected and the search resumes where it left off (see "Verification behaviour"). If the whole grid is covered with no confirmed person, the drone returns to launch (RTL).
- **Land Now** — `/resq/mission/land`: lands immediately, independent of the mission.
- **Drop Payload** — detaches the payload box via the Gazebo `DetachableJoint` topic `/model/s500_quad/payload/detach`; it falls under real physics.
- **Emergency Stop** — kills everything and runs the cleanup sweep.

## Verification behaviour

A candidate is only rejected ("not a person") on real evidence: at least 3 frames analysed while hovering over it, with the person missing from most of them. If too few frames were analysed (e.g. a slow machine), it keeps hovering up to 90 s and never blacklists the spot. (Before this fix, a slow detector — ~2.9 s/frame with the Gazebo GUI and RViz open — plus a 3 s freshness limit meant almost every verification frame was discarded, the real person's location got blacklisted, and the drone searched on past them.)

## Tested end-to-end

The full stack (Gazebo headless + ArduPilot SITL + bridge + detector) was run
through complete missions, including one started through this server's HTTP
API. Result: grid search at 2 m/s, the drone's own shadow detected and rejected
several times, the person detected on the lane passing over them (0.89), verified
while hovering (0.95, estimate within ~0.2m of the true position), landed 2.3m
from the person with a 2.5m target.

## Thermal / night mode

- **Camera**: the drone carries a second, boresighted thermal camera (`thermal_camera` in `s500_quad/model.sdf`: same mount, FOV and resolution as the RGB camera, 16-bit radiometric at 0.01 K per unit), bridged to ROS as `/thermal/image`.
- **Model**: your `best.pt` (YOLOv8n trained on HIT-UAV, a drone thermal-infrared dataset), installed as `ardu_ws/thermal_best.pt`. `last.pt` is the same training run's final checkpoint and scores lower (mAP50 0.85 vs 0.91), so it isn't used; the copies in `~/Downloads` are byte-identical duplicates.
- **Preprocessing**: temperatures are mapped white-hot over 283–313 K (like a real thermal camera's output) before inference; the dashboard shows the same frames in false colour.
- **Heat source**: Gazebo's thermal system doesn't apply to animated `<actor>`s, so the casualty has an invisible, shadowless 310 K "heat body" (`test_person_heat` in `runway.sdf`) shaped and positioned to match the actor. The RGB camera doesn't see it at all.
- **Measured**: on rendered thermal frames at 6–20 m the model scores 0.51–0.73 on the casualty and 0.00 on empty ground, at ~0.15 s/frame on CPU (8× faster than the RGB detector). Tested end-to-end: a mission started in day mode, switched to night 20 s into the search, found the person on thermal, verified and landed 2.4 m beside them.
- **Lighting**: scene ambient light can't change while the sim runs, so it's kept at 0.15 and a shadowless "fill" light supplies the rest of daytime light; night turns the sun into dim moonlight and the fill off. Daytime brightness and RGB detection were re-measured to match the old setup. Night is dim/dusky rather than pitch black, since ambient can't go to zero at runtime.

## How detection works, and why

- **Detector**: COCO-trained YOLOv8m (`ardu_ws/yolov8m.pt`), run on the frame
  plus 90/180/270° rotations (a person lying on the ground can face any way from
  above). The previous VisDrone model scored the lying person ≤0.31 at every
  altitude tested — it is trained on upright pedestrians seen at an angle.
- **Camera**: horizontal FOV narrowed from 114° to 69° (`s500_quad/model.sdf`).
  Through the 114° lens the person was a ~40px speck at 8–10m.
- **Shadow**: the drone's X-shaped shadow does score as a "person" (0.6–0.85). It
  is rejected by geometry: `mavlink_bridge.py` knows the sun direction in
  `runway.sdf`, so it predicts exactly where the shadow falls from the drone's
  pose when the frame was captured, and ignores detections there. In testing the
  shadow detections landed within 0.1m of the prediction.
- **Latency**: inference takes ~1.9s per frame on CPU with the sim running. Each
  detection is timestamped with its frame's capture time, and the bridge projects
  it using where the drone was at that moment, including roll/pitch (no gimbal).

## GPU (optional, much faster)

This machine has an RTX 4050 but the installed PyTorch is the CPU-only build
(`torch 2.13.0+cpu`). Installing the CUDA build of PyTorch would cut inference
from ~2s to roughly 0.1s per frame, allowing faster search speeds. It replaces a
core Python package (~2.5GB), so it hasn't been done automatically.

## Post-disaster world

`ardu_ws/src/ardupilot_gz/ardupilot_gz_gazebo/worlds/disaster.sdf` (Gazebo world name `disaster`) is an earthquake aftermath around the same search area. From a terminal: `ros2 launch ardupilot_gz_bringup s500_quad_disaster.launch.py` (same as `s500_quad_runway.launch.py world:=disaster`).

- **Kept identical to `runway.sdf`**: frame and GPS origin, sun/fill lights (so day/night and the shadow filter work unchanged), the casualty and heat body at (10 E, 15 N), and the payload box. `mission.json` needs no change.
- **Around the grid**: the four osrf/gazebo_models collapsed buildings (police station W, fire station N, industrial E, plus four collapsed houses) with smoke plumes. They are up to 24 m tall, so they sit outside the 40×40 m search grid plus a 4 m margin. The edge lanes still see them.
- **Inside the grid** everything is under ~2 m: rubble mounds, tilted concrete slabs, wrecked cars, a fallen telephone pole and lamp post, cinder blocks, planks, pallets, a rubble spill at the casualty's feet, and a smouldering debris pile at (21 E, 31 N) at 330–360 K for the thermal camera. The spawn point and the landing spot beside the casualty are kept clear.
- **Response staging** south-west of the spawn: ambulance, fire truck, barriers, cones.
- **Models** live in `ardu_ws/src/ardupilot_gazebo/models`. Taken from osrf/gazebo_models: `collapsed_*`, vehicles, `jersey_barrier`, `construction_*`, `cinder_block*`, `drc_practice_*`, `euro_pallet`, `telephone_pole`, `lamp_post`, `dumpster`, `fire_hydrant`, `cardboard_box`. Made for this world: `disaster_ground` (generated ground and rubble textures and meshes) and `smoke_plume` (particle emitter). The osrf trees are not used, because their classic OGRE materials render black in Gazebo Harmonic.

### Detector results in this world (8 m, 81 positions over the grid)

Same frames and settings as `person_detector.py`, compared against `runway.sdf`:

| | Runway | Disaster |
|---|---|---|
| RGB: casualty hits (conf) | 9 (0.76–0.95) | 5 (0.80–0.94) |
| RGB: false hits | 0 | 4 (a traffic cone 0.63; the charred debris pile 0.78) |
| Thermal: casualty hits (conf) | 20 (0.45–0.61) | 40 (0.47–0.74) |
| Thermal: false hits | 0 | 37 (cones, the smouldering pile's embers, a wrecked car; up to 0.77) |

The thermal model (HIT-UAV) fires on small warm or saturated blobs. The cones near the spawn are seen from the first search lane, so without a fix a night mission would stop at a cone before it reached the casualty. To handle this, `person_detector.py` now checks each thermal box against the frame's real temperatures (`BODY_TEMP_RANGE_K`). It keeps a box only if the 97th-percentile temperature inside it is 303–316 K. On the same frames, that removed 16 of 17 false hits and none of the 17 casualty hits, and changed nothing on the runway world. The remaining false hit is the warm ash around the burning pile (305.8 K), which is genuinely body temperature. A night mission in the disaster world then rejected 178 cold detections (around 288–290 K, up to 0.81 confidence), confirmed the casualty about 0.5 m from their true position, and landed 2.2 m beside them.

## Things tied to specific values

- The casualty (`test_person` actor in `runway.sdf` and `disaster.sdf`) is at 10m east, 15m north of
  the drone spawn; `mission.json`'s search area is centered near it.
- The payload box pose in `runway.sdf` matches only the default spawn
  (`x=0 y=0 z=0.2`, yaw 90°). It sits 5cm aft of center so the downward camera
  doesn't see it when the drone tilts to fly.
- If you move or rotate `test_person`, move `test_person_heat` identically (same x, y, yaw), or the thermal camera will see the heat somewhere else.
- `CAMERA_*` constants in `mavlink_bridge.py` must match the camera in
  `s500_quad/model.sdf`; `SUN_DIRECTION_ENU` must match the sun in `runway.sdf` and `disaster.sdf`.
- Every object in `disaster.sdf` must be static. Gazebo's physics engine (DART) can't build mesh collisions, so a dynamic gazebo_models object that collides only through a mesh (the cones and barrels) falls through the ground forever. Six such objects held the whole sim at about 0.3× real time with stalls, and the drone moved in jerks. With them static it runs at 1.0×.
- The Gazebo window loads `ardupilot_gz_bringup/config/gz_gui.config`, a copy of the default GUI config with `start_paused` set to false. The default pauses the simulation as soon as the window opens, and a paused sim sends ArduPilot no sensor data, so the mission hangs at ARMING.
- A new world's `<world name>` must equal its file name (the launch file uses one `world` argument for both), and it needs lights named `sun` and `fill` for day/night switching.

## Files

- `backend/server.py` — Flask app: process management, an `rclpy` node bridging ROS2 topics/services to REST + SSE, MJPEG video.
- `backend/ros_env.sh` — sources the ROS2/Gazebo environment for subprocesses, and forces the system Qt plugin path (OpenCV's bundled one otherwise leaks in and crashes Gazebo).
- `frontend/` — plain HTML/CSS/JS dashboard (no build step).
