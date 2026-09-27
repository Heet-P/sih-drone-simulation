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

One command starts everything (dashboard, Gazebo + SITL, mission bridge, live feed; RViz only with `RESQ_RVIZ=1`) and opens the dashboard; Ctrl+C stops it all:

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

## Layout

The sidebar switches between four views:

- **Home**: the camera feed, a compact live map, drone status (altitude, speed, battery, satellites), environment and camera pickers, and a mission strip (stage, probability of success, casualties and hazards as chips).
- **Map**: the full situational map with the rescue-priority cards, hazards and the alert log.
- **Telemetry**: every vehicle value, process status and the console.
- **Settings**: world, search planner, and what to do when a casualty is confirmed.

The header shows system state, GPS lock and satellites, battery, and UTC time.

**Camera feed**:
- *AI FEED* shows the detector's output with boxes.
- *THERMAL* shows the raw thermal camera in false colour (`/video_feed/thermal`, streamed by the server straight from `/thermal/image`, whatever the day/night mode).
- *SPLIT* overlays the two with a draggable diagonal divider.

The feed also carries a HUD (altitude, speed, heading, mission stage), a compass that turns with the heading, a LIVE badge, a full-screen button and a reticle toggle. Critical alerts (casualty, fire) pop up as toasts over the video with a chime.

The dashboard uses no external fonts, libraries or tiles, so it works on an offline laptop.

## What each button does

The action bar is on every view:

- **Launch / Stop Simulation**: launch runs `ros2 launch ardupilot_gz_bringup s500_quad_runway.launch.py world:=<world> rviz:=false use_gz_tf:=true` and starts the `mavlink_bridge` ROS2 node (telemetry + mission services); it opens the real Gazebo window. RViz is off by default, since the dashboard covers it and the sim runs faster without it. Start with `RESQ_RVIZ=1 ./start.sh` to open it too. Stop ends both (whole process groups, so nothing is orphaned even if `ros2 launch` already died), then runs a targeted `pkill` sweep for anything left over. The world comes from Settings: *Post-disaster zone* (`disaster.sdf`, default) or *Runway (flat)* (`runway.sdf`), locked while a sim runs. See "Post-disaster world" below.
- **Start / Stop Camera Feed**: runs `ardu_ws/person_detector.py`. When a detection completes, the exact analysed frame is shown for a moment with its boxes.
- **Day / Night** (or the Environment picker): toggles day/night at any time, even mid-mission. The detector switches camera and model, the Gazebo world goes to night (sun → dim moonlight, no shadows), and the dashboard switches to a dark red night theme with a NIGHT badge. Switching back restores daylight.
- **Start Search & Rescue** calls `/resq/mission/start` with the planner and on-find choices from Settings:
  1. GUIDED → arm → take off to 8 m.
  2. Search the area in `~/resq_mavlink/mission.json` at 2 m/s with the chosen planner.
  3. On any detection, **stop, fly over it and hover to verify**. A candidate that turns out not to be a person is rejected and the search resumes (see "Verification behaviour").
  4. Once the detector confirms it over several frames, the casualty is geo-tagged and ranked.
  5. In *mark and continue* mode (default), the drone flies 2.5 m to the side, drops the aid kit (first casualty only; there is one on board) and keeps searching for more casualties. When the probability of success reaches `target_pos` (90%), it returns to launch.
  6. In *land* mode, it lands 2.5 m beside the first casualty instead.
- **Land Now**: `/resq/mission/land`, lands immediately, independent of the mission.
- **Drop Payload**: detaches the payload box via the Gazebo `DetachableJoint` topic `/model/s500_quad/payload/detach`; it falls under real physics.
- **SITREP**: opens `/report`, a printable situation report of the current mission (see below).
- **Emergency Stop**: kills everything and runs the cleanup sweep.

## Obstacle-aware altitude

`resq_mavlink_bridge/obstacle_guard.py`, used by every flight command in `mavlink_bridge.py`.

- **Sensors** (`s500_quad/model.sdf`):
  - a forward 3D lidar: 48 × 16 beams, ±34° horizontal, −40° to +20° vertical, 40 m range
  - a downward single-beam rangefinder
  - both bridged to ROS as `/obstacle_lidar/points` and `/down_range`
- **Corridor check**: each scan is levelled with the drone's roll and pitch, then cropped to a 5 m-wide corridor ahead. The look-ahead grows with speed, from 10 to 35 m. Anything in it more than 1 m above the ground below the drone is an obstacle (building, tree, pole, rising terrain).
- **Climb**: the drone climbs to 4 m above the tallest obstacle in the corridor, stays up while the rangefinder shows it is over a roof, and comes back down once the corridor has been clear for 3 s.
- **Blocked**: if an obstacle it can't yet clear is within stopping distance, the drone **holds position and climbs** until the lidar sees over it. If an obstacle fills the lidar's top beam, its real top may be higher, so it counts as at least 3 m above what is visible.
- **Guarded return**: instead of ArduPilot's RTL, which flies straight at a fixed height, the drone flies home at the guarded altitude, then lands.
- **Looks from higher up**: while raised, camera looks still update the probability map, with detection probability scaled by (search altitude ÷ altitude)².
- **Dashboard**: the Forward lidar card on Home shows a side view of the corridor and the altitude being held, and each climb and return appears in the alerts.

**Measured** (test mission over the collapsed police station, `RESQ_MISSION_FILE` pointing at a mission centred on it): the drone climbed from 8 m to 12.5 m as the building came within 5 m, searched over it, came back down, and flew the guarded return. Before a fix it could hover indefinitely at the building's edge ("blocked" flickered at exactly the clear height); blocking now has 1 m of slack.

`RESQ_MISSION_FILE=/path/to/mission.json` overrides the mission file the bridge loads.

**Safety, learned from a crash in the region world.** At 5 m/s the drone turned toward an 18 m office block that came into the lidar's view only 9 m away. It was told to "hold position", but braking from 5 m/s takes ~5 m, so it drifted into the wall, and ArduPilot's crash check disarmed it (`Crash: Disarming: AngErr=84>30`). The bridge then kept "searching" with a dead drone. Now:
- **Caution speed:** while an obstacle ahead still needs climbing over, the drone flies at most 2 m/s and doesn't aim past its target.
- **Hold means brake:** the hold point is placed one second of travel *behind* the drone (up to 5 m), so it stops short of the obstacle.
- **Crash detection:** a disarm in flight raises a critical alert (`DRONE DISARMED DURING …`) and ends the mission (stage `ABORTED`).
- **Livelock guard:** if the planner spends 30 s at one target without draining it (e.g. beside a tall building it must overfly too high for its looks to count), most of that area's probability is written off and the drone moves on, with an alert.
- **Carrot:** in open flight, the bridge aims 2 s of travel past the planner's target so ArduPilot doesn't brake at every hop. Without it the drone averaged 1.47 m/s when commanded 5 m/s; with it, about 3.4 m/s.



## Video feed and simulation performance

What changed:
- The detector publishes **JPEG** frames (`/detections/debug_image/compressed`). Filling a raw 640×480 ROS `Image` from Python took ~93 ms per frame, and the server forwards the JPEG bytes as they are.
- The MJPEG streams push each frame as it arrives instead of on a 10 Hz timer.
- `ros_env.sh` points Gazebo's rendering at the NVIDIA GPU. On this PRIME laptop, headless EGL rendering fell back to software ("failed to create dri2 screen").
- When inference is fast (< 0.25 s), boxes are drawn on the live frames instead of freezing on the analysed frame.

Measured effect (headless, 20 Hz cameras): AI feed 3.4 → 16 fps, detector 0.20 → 0.07 s per frame.

**Cameras stay at 10 Hz.** At 20 Hz, with the Gazebo window and RViz open, the sim fell to ~0.59× real time on average with deep stalls, and the drone visibly stopped and dragged. Measured with a continuous sim-time trace over full missions, cameras at 10 Hz:

| Setup | Mean real-time factor | Worst 2 s window |
|---|---|---|
| Gazebo window + RViz (`RESQ_RVIZ=1`) | 0.82 | 0.58 |
| Gazebo window, no RViz (`start.sh` default) | 0.85 | 0.60 |
| Headless | 0.90–0.92 | 0.67–0.68 |

The trade-off is the AI feed at about 6 fps. The lidar costs little: with the Gazebo window, removing it changed the mean real-time factor from 0.85 to 0.87.

**The detector is capped at 5 analyses per second, in FP16** (`MAX_INFERENCE_HZ`, `USE_FP16` in `person_detector.py`). Uncapped on the GPU it ran ~12 analyses/s (4 rotations each) and kept the laptop GPU busy (63% mean, 95% peak, against 24% with the detector off). The Gazebo window then couldn't draw on time, and the drone looked like it stopped mid-air and jumped forward. Capped, over a full mission with the Gazebo window and no RViz:

| | Uncapped | Capped 5 Hz + FP16 |
|---|---|---|
| GPU busy, mean / peak | 63% / 95% | 36% / 63% |
| Real-time factor, mean / worst 2 s | 0.85 / 0.60 | 0.92 / 0.74 |
| Casualty evidence | 8/8 + 3/8 frames | 8/8 + 8/8 frames |

Inference is 0.05 s per analysis. Between analyses, the live feed keeps drawing the latest boxes.

## Verification and RGB + thermal fusion

With a fast detector, the original rule ("2 samples + the detector's 3-frame streak") confirmed the two dogs as casualties. A hit ratio alone can't fix this: before fusion a dog scored 12/40 frames, and a real casualty in a busy run scored 35/121. What separates them is physics:

- **Fusion by day**: every RGB "person" box is also checked in the boresighted thermal frame, with the same checks as night mode.
  - **Temperature:** the hottest pixels must be at body temperature (303–316 K).
  - **Size:** the warm region must be at least 1.1 m long. The dogs measured 0.72–0.86 m.
  - **Fire:** the warm region must not contain fire-temperature pixels (≥ 323 K). This rejects the smouldering debris pile, whose ash is genuinely 305 K but is connected to 330–360 K embers.

  The same checks run at night.
- **Confirmation**: at least 8 frames analysed while hovering overhead, with a person in at least 20% of them. The detector's streak is no longer required, because it needs more than ~50% recall to build up.
- **Fast rejection**: fewer than 8% of 25+ frames after 4 s overhead. Otherwise rejection happens at the 30 s timeout if under 20%.

**Measured:**
- Fused, the dog candidates scored 0/46 and 0/38 frames.
- Full missions after the change found exactly the two casualties every time, within 0.5 m of their true positions: 8/8 + 3/8 frames (window + RViz), 8/8 + 8/8 (window only), 8/8 + 2/8 (headless).
- On synthetic thermal frames, the fire check rejects debris (ash + embers) and keeps a person lying 3 m from a fire.

## Logs

The **Logs** button in the header (or the `L` key, on any view) opens a drawer with two tabs:
- **Events**: the readable mission timeline.
- **Console**: raw output from the mission bridge, detector, dashboard and simulator. Filter by source or text. The simulator is off by default because it is very verbose. Errors show in red, warnings in yellow.

A badge counts new alerts while the drawer is closed.

## Restarting

`./start.sh` first stops anything left from a previous run: another `start.sh`, or the sim, bridge, detector and dashboard processes it left behind. It asks them to stop cleanly, then forces any that remain, then waits for port 5000 to be free.

## Probability-map search

`resq_mavlink_bridge/search_map.py`, used by `mavlink_bridge.py`. This is search theory, the method real SAR agencies use to plan searches, run live on the drone:

- The search area is split into 2 × 2 m cells, each holding the probability that a casualty is there. The **prior** comes from `prior_zones` in `mission.json`: debris fields and building spill zones, where people are most likely trapped, get more weight than open ground.
- Every frame the detector analyses is a **look** at the ground under it. If nobody is detected, each cell in the central 80% of the footprint is multiplied by (1 − POD), the chance the detector would have missed a person there (0.7 per RGB frame, 0.6 per thermal frame; frames less than 1 s apart count once). Cells around a pending detection aren't reduced until verification decides.
- The **planner** flies to the cell with the most probability within 3 m (always inside the footprint, at any heading) per second of flight. It re-plans every second, and switches target only if the new one is 25% better.
- **Probability of success (POS)** = the fraction of the prior probability searched away. The mission ends at 90% POS (`target_pos`) or after `max_search_s`.
- A confirmed casualty removes the probability within 4 m, and later detections within 4 m of them are treated as the same person.

On the dashboard map, the heatmap is the live probability: bright means likely and not yet searched, dark means searched.

### Benchmark

`python3 resq_mavlink/search_benchmark.py [--night]` flies both planners over the same `mission.json` area 300 times each, using the bridge's own map code, a kinematic drone and the same POD model for both. It measures the planners, not the detector.

| Casualty placed | Detector | Median time to find, grid → probability map | Found within 60 s |
|---|---|---|---|
| where the intel prior says | RGB (day) | 96 s → 48 s (**51% faster**) | 31% → 57% |
| where the intel prior says | thermal (night) | 86 s → 50 s (42% faster) | 32% → 61% |
| uniformly at random (intel worthless) | RGB (day) | 85 s → 69 s (19% faster) | 34% → 47% |
| uniformly at random (intel worthless) | thermal (night) | 78 s → 62 s (21% faster) | 39% → 49% |

The grid stops after one pass, as in the mission, so it also misses more (it found 87–94% within 300 s; the probability planner found 96–99%). It is faster even when the intel is worthless because it doesn't fly overlapping lanes and turns over ground it has already searched well.

## Hazard detection

`person_detector.py` scans every thermal frame (once a second, in day **and** night mode; the thermal camera is always on):

- **Fire / hot spot**: regions above 50 °C (323 K), well above body temperature and sun-warmed ground.
- **Flood water**: regions colder than 283.5 K and larger than 1.5 m² on the ground. By day the boresighted RGB frame must agree: water is flat and textureless (its mean |Laplacian| is under half the frame's), unlike rubble and gravel. That makes it **multi-sensor fusion**: a cold patch of rubble is rejected. At night the thermal signature alone decides.

Sightings go out on `/detections/hazards` (JSON, image offsets). The bridge projects each one to the ground with the drone's pose at capture time (like person detections), merges repeated sightings into one zone, and reports a zone once it has been seen twice. The AI feed shows a hazard banner, and the dashboard's split view shows the raw thermal camera beside it.

`known_hazards` in `mission.json` are pre-mission intel (the collapsed buildings around the zone). They appear on the map and in the SITREP, and ground routes avoid them.

## Situational map, rescue priorities and SITREP

The bridge publishes the whole picture on `/resq/map` (JSON, 2 Hz). The server relays it as `/api/map` (SSE) and `/api/map/latest`. The dashboard map shows:

- the probability heatmap and the search area
- the flight trail, the live camera footprint, and the planner's next target
- candidates being verified (pulsing) and rejected ones (×)
- casualties (red, with priority badges)
- hazard zones with their safety buffers (dashed)
- the team staging point, and an animated **safe ground route** to each casualty

**Safe routes** use A* on a 1 m grid from the staging point (`staging` in `mission.json`, beside the ambulance). Each hazard is blocked out by its radius plus a buffer (fire 4 m, flood water 1.5 m, structure 3 m), and ground within 3 m beyond that costs extra.

**Rescue priorities**: casualties are ranked by nearby threats (fire within 15 m, flood water within 10 m, structures within 8 m; closer means higher), then by how quickly a team on foot (1 m/s over rubble) can reach them. Each card gives a recommended action with route length and ETA.

**Alerts**: casualty and fire alerts are *critical* (red, two-tone chime); other hazards are warnings.

**SITREP** (`/report`, **Open SITREP** button): a printable page with KPIs, the map, the rescue-priority table with coordinates, hazards, and the full mission log, generated from the drone's own data. Use the browser's *Save as PDF*.

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
- **Heat source**: Gazebo's thermal system doesn't apply to animated `<actor>`s, so the casualty has an invisible, shadowless "heat body" (`test_person_heat` in both worlds) shaped and positioned to match the actor. Exposed skin (head, neck, hands) is 309 K and clothing is 305 K, as a real thermal camera sees a clothed person. The thermal camera can't see through the actor's mesh, so the heat body sits just above it (z 0.27) and is about 12% larger. Before, only fragments showed, and at 8 m the casualty covered 879 warm pixels; now it covers about 2,500, a full body 1.79 m long. The RGB camera doesn't see the heat body at all.
- **Background temperature**: objects with no thermal plugin render at the atmosphere temperature (288 K), varied by how bright their texture is. Gazebo scales that spread with the atmosphere's `temperature_gradient`. At the standard -0.0065 K/m, white stones in the rubble read up to 347 K and dark pixels hit the camera's 250 K floor, which gave the thermal model "people" all over the debris. Both worlds now use -0.0003, which gives a realistic 285–289 K background.
- **Animals**: `disaster.sdf` has two sitting dogs (`dog_open` at (-3, 12), `dog_rubble` at (16, 26)). The model is Google Scanned Objects' "Dog" (CC BY 4.0), scaled ×3.5, with a fur heat signature of 302–307 K. The thermal model calls them "person" (0.55–0.72), and the temperature check passes them. The detector's size check (below) rejects them. The day RGB model never scores them "person" above 0.45; it reads them as "teddy bear".
- **Detector checks on thermal detections** (`person_detector.py`): (1) temperature: the 97th-percentile temperature in the box must be 303–316 K (`BODY_TEMP_RANGE_K`); (2) size: the warm region (above 300 K) the box sits on must be at least 1.1 m long (`MIN_BODY_LENGTH_M`), measured from the drone's altitude on `/resq/drone/position`. The size check is skipped below 3 m or before the altitude is known. Trade-off: an adult curled up tighter than 1.1 m would also be rejected. Over an 81-view thermal sweep of the disaster grid at 8 m, only the casualty passes both checks. The dogs fail on size, and one other detection fails on temperature.
- **Measured**: on rendered thermal frames at 6–20 m the model scores 0.51–0.73 on the casualty and 0.00 on empty ground, at ~0.15 s/frame on CPU (8× faster than the RGB detector). Tested end-to-end: a mission started in day mode, switched to night 20 s into the search, found the person on thermal, verified and landed 2.4 m beside them.
- **Lighting**: scene ambient light can't change while the sim runs, so it's kept at 0.15 and a shadowless "fill" light supplies the rest of daytime light; night turns the sun into dim moonlight and the fill off. Daytime brightness and RGB detection were re-measured to match the old setup. Night is dim/dusky rather than pitch black, since ambient can't go to zero at runtime.
- **Sky**: a world's `<sky>` and background can't change at runtime either, so both worlds instead have two emissive sky domes: `sky_day` (hazy blue) and `sky_night` (dark, with stars and a fire glow on the horizon). Night mode moves `sky_day` out of sight and `sky_night` into place with `set_pose`, and moves the smoke plumes away (particles are unlit and glowed white against the dark sky). Day mode reverses it. The poses are in `NIGHT_POSES` in `server.py` and must match the world files.

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

## GPU

The CUDA build of PyTorch is installed (`torch 2.11.0+cu128`). Ultralytics uses the RTX 4050 automatically, so no code changes were needed. Measured: YOLOv8m on the 4-rotation batch takes **0.045 s on the GPU vs ~1.9 s on the CPU**.

Installing it with `pip3 install --user --force-reinstall torch torchvision --index-url https://download.pytorch.org/whl/cu128` also upgraded numpy to 2.x. That breaks ROS Humble's `cv_bridge` (`AttributeError: _ARRAY_API not found`), so numpy was put back with `pip3 install --user "numpy==1.26.4"`. Pip then warns that opencv-python 5.0 wants numpy ≥ 2; that warning can be ignored (OpenCV, cv_bridge, torch and YOLO were all tested working with 1.26.4). To go back to the CPU build, use the same install command with `https://download.pytorch.org/whl/cpu`, then restore numpy the same way.

## Region world (200 × 200 m)

`./start.sh region` (new random casualty placement) or `./start.sh region 42` (repeat placement 42). In the dashboard, pick *Region 200 m* under Settings → World, with an optional seed. Generated by `worldgen/generate_region.py`.

**Layout**, with the launch pad and command post at the centre:
- **City (north-east):** apartment, office and post office models, procedural blocks with window facades, collapsed blocks with rubble, a burning block with smoke, and a smouldering debris pile.
- **Forest (north-west):** 167 low-poly trees, clearings and a dirt trail.
- **Village (south-west):** Indian houses, 26 thatched huts (some collapsed), farmland and lanes.
- **River (south-east):** a river with a truss bridge, flood water over the west bank, and a stranded bus.

**Casualties.** Each launch places 6 casualties from a seed, only on open ground the camera can see: rubble, streets, lanes, fields, clearings and riverbanks. Never under tree crowns, inside buildings or in water. The ground truth goes to `resq_mavlink/region_truth.json`, which is for scoring test runs only; the drone never reads it.

**Casualty poses.** Each casualty gets a pose from a separate random stream, so a seed's positions never change: lying 35%, crawling 15%, sitting 20%, waving 15%, walking 15%.
- The animations are the local actor clips: *stand* rotated flat, *walk* turned face-down and moved slowly for crawling, *sitting*, *talk_b* for waving, and *walk*.
- Walkers pace 6 m at 1 m/s, and crawlers 3 m at 0.3 m/s, on paths checked to be clear.
- Each pose has a matching heat shape: flat, upright or seated, with skin at 309 K and clothing at 305 K.
- Gazebo's thermal system doesn't apply to actors, so `worldgen/heat_follower.py` moves the heat shape of each moving casualty with its actor, from the sim clock, at 10 Hz. The dashboard server starts it with the region world.
- Demo seed 3 has 2 crawling, 2 sitting, 1 waving and 1 lying casualty.

**Detecting every pose from above** (drone camera at 8 m, the detector's own code on rendered frames):

| Pose | RGB model | Thermal model | Warm shape from above | Found by |
|---|---|---|---|---|
| Lying | 0.92 | 0.60 | 1.79 m | all three |
| Crawling | 0.91 | 0.58 | 1.78 m | all three |
| Walking | 0.64 | 0.52 | 0.74 m | thermal model + signature |
| Waving | 0.33 (missed) | 0.56 | 0.73 m | thermal model + signature |
| Sitting | missed | missed | 0.85 m | thermal signature only |
| Dog | (0.67–0.83 in flight) | fires | 0.82 m | **rejected**: no exposed skin (peak 305.8 K) |

Upright people are dog-sized from above, so the old "shorter than 1.1 m = animal" rule would have thrown them away. A compact warm shape (0.35–1.1 m) now counts as a person only if its peak reaches **skin temperature, 308 K**: face and hands are at 309 K, while the dog's fur is at most 307 K. That's only a 2 K margin in this simulation, and real thermal footage would need to confirm it.

By day the detector now runs three sources, and all of them go through the same thermal checks:
- the RGB model, cross-checked against the thermal frame
- the thermal model
- a **thermal-signature** detector: person-sized warm shapes with a skin-temperature peak and no fire-temperature pixels. This is the only one of the three that finds sitting people.

**Triage:** during verification, the drone follows the latest sightings.
- If the mean of the first three sightings and the mean of the last three are more than **2.5 m** apart, the casualty is recorded as **moving (responsive)**, like START's walking wounded, and ranked after still, unresponsive casualties with the same nearby threats.
- The threshold comes from measurement: still casualties' sightings drifted up to 1.7 m (projection jitter, standard deviation 0.7–1.0 m).
- So only **walkers** are flagged. A crawler at 0.3 m/s moves about 0.6 m during a ~2 s verification, which can't be separated from the jitter, and it isn't claimed.

A casualty is also checked at confirmation time against those already located. In the city, a first sighting from high above the blocks can project several metres off and slip past the check made when a candidate first appears; one crawler was confirmed three times before this fix.

**Built for frame rate:**
- The landscape is one static model (`region_scenery`) of a few merged meshes: all trees in one OBJ, all procedural buildings in another, ground, roads and fields in a third, about 6,500 triangles in total.
- Only the ground collides.
- Textures are generated and tile seamlessly.
- The hero models were render-checked in Harmonic. Fuel's House 1, 2 and 3 render black and are not used.

Measured with the mission searching:

| | Real-time factor (mean / worst 2 s) | GPU |
|---|---|---|
| Headless | 1.00 / 1.00 | 16% |
| With the Gazebo window | 0.96–0.99 / 0.81–0.99 | 46% |

The window's cost is fixed. It stayed at 46% GPU even with the whole landscape removed, so the world's content isn't the bottleneck.

**Mission intel** (`resq_mavlink/mission_region.json`, written by the generator) is pre-disaster information only:
- **Building footprints and collapsed structures:** hazards on the map, which ground routes avoid.
- **The river's outline:** water the drone detects there is the river, not flooding, so only water *outside* it is reported as flood.
- **A basemap** (`frontend/basemaps/region.png`), drawn under the dashboard map.
- **A land-use prior** (`prior_region.npy`, 2 m grid), built from the map layers, never from casualty positions:

| Map layer | Weight |
|---|---|
| Around collapsed buildings | 3 |
| Around homes | 1 |
| Roads and lanes | 1 |
| Riverbank | 1.2 |
| Clearings and trail | 1.2 |
| Fields | 0.6 |
| Open ground elsewhere | 0.15 |
| Under tree crowns | 0.02 |
| Water and roofs | 0 |

**Demo seed: 3.** Rendering the drone's 8 m view over each casualty (centred and 2.5 m off-centre) and running the detector on it, seed 3 had all 6 casualties clearly visible. Seeds 2, 4, 5 and 7 had 5 of 6, and seed 6 had 4. The usual misses are casualties half-hidden by rubble or at the water's edge. The mission budget is 25 min: at 15 min a real headless run (seed 1) found 2 of 6, even though the benchmark predicted about 80%. The benchmark flies at constant speed, while the real drone brakes, climbs and verifies.

**Measured end to end (demo seed 3, headless, one full 25 min mission):** all 6 casualties found, every pose type, each within 1.4 m, with no false positives and no crash:

| Casualty | Pose | Where | Found |
|---|---|---|---|
| C3 | sitting | field | 69 s |
| C4 | sitting | field | 95 s |
| C1 | crawling | field | 146 s |
| C5 | crawling | rubble, city | 369 s |
| C2 | waving | street, city | 420 s |
| C6 | lying | forest clearing | 1115 s |

That's 5 of 6 in the first 7 min. Over the mission:
- Average speed was 3.0 m/s, and 84% of the probability was searched.
- Five repeat sightings were recognised as casualties already found.
- The drone climbed over mapped buildings five times.
- It mapped both fires and three flood areas, and didn't report the river as flood.

Getting there took four fixes found in earlier runs: the carrot, time-based blocking, re-sighting checks at confirmation, and flying over mapped building heights, after two crashes in the city.

**Obstacle intel.** `known_obstacles` in `mission_region.json` lists 19 mapped obstacles, the generator's buildings and hero models with their heights. The bridge flies over any on the next 25 m of its path at height + 4 m, and climbs in place first when one is within 8 m of its 6 m buffer. The forward lidar remains the backup for unmapped obstacles.

**Search at this scale.** `python3 resq_mavlink/region_benchmark.py` flies the bridge's planner over the generator's own casualty placements, 16 seeds × 6 casualties, with 12 s per find for verification:

| Planner | Speed | Found by 5 min | by 10 min | by 15 min |
|---|---|---|---|---|
| Lawnmower | 5 m/s | 31% | 57% | 62% |
| Probability map, land-use prior | 3 m/s | 31% | 46% | 59% |
| **Probability map, land-use prior** | **5 m/s** | **43%** | **67%** | **81%** |
| Probability map, land-use prior | 7 m/s | 42% | 64% | 90% |

The mission flies at 5 m/s, within a 15 min budget. A "regional" planner term (a wider 10–25 m gain radius) was tried and did not help, so it is off. The planner scales to this size because gains are a small convolution: an N×N neighbour matrix would be ~680 MB here. A* routing uses a coarser grid for large areas.

## Post-disaster world

`ardu_ws/src/ardupilot_gz/ardupilot_gz_gazebo/worlds/disaster.sdf` (Gazebo world name `disaster`) is an earthquake aftermath around the same search area. From a terminal: `ros2 launch ardupilot_gz_bringup s500_quad_disaster.launch.py` (same as `s500_quad_runway.launch.py world:=disaster`).

- **Kept identical to `runway.sdf`**: frame and GPS origin, sun/fill lights (so day/night and the shadow filter work unchanged), the casualty and heat body at (10 E, 15 N), and the payload box. `mission.json` needs no change.
- **Around the grid**: the four osrf/gazebo_models collapsed buildings (police station W, fire station N, industrial E, plus four collapsed houses) with smoke plumes. They are up to 24 m tall, so they sit outside the 40×40 m search grid plus a 4 m margin. The edge lanes still see them.
- **Inside the grid** everything is under ~2 m: rubble mounds, tilted concrete slabs, wrecked cars, a fallen telephone pole and lamp post, cinder blocks, planks, pallets, a rubble spill at the casualty's feet, and a smouldering debris pile at (21 E, 31 N) at 330–360 K for the thermal camera. The spawn point and the landing spot beside the casualty are kept clear.
- **Response staging** south-west of the spawn: ambulance, fire truck, barriers, cones.
- **Second casualty** `casualty_2` (+ `casualty_2_heat`) at (22 E, 21.5 N), lying by the debris east of the street, for *mark and continue* missions. `runway.sdf` still has one casualty.
- **Flood water** `flood_water` at about (26 E, 0.5 N) beside the wrecked pickup: three flat, glassy discs about 7 × 6 m, at 279 K for the thermal camera (below the ground's 285–289 K).
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

- `frontend/map.js` — the situational map renderer, shared by the dashboard and `report.html` (the SITREP).
- `../ros2_ws/src/resq_mavlink_bridge/resq_mavlink_bridge/search_map.py` — probability map, hazard zones, A* safe routes, rescue priorities (no ROS; tested by `resq_mavlink/search_benchmark.py`).
- `backend/server.py` — Flask app: process management, an `rclpy` node bridging ROS2 topics/services to REST + SSE, MJPEG video.
- `backend/ros_env.sh` — sources the ROS2/Gazebo environment for subprocesses, and forces the system Qt plugin path (OpenCV's bundled one otherwise leaks in and crashes Gazebo).
- `frontend/` — plain HTML/CSS/JS dashboard (no build step).
