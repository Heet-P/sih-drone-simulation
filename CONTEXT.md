# SIH 2026 — Autonomous SAR Drone: Project Context

The single source of truth for this project: what we are building, how every part works, what is already done (and measured), and what is still planned. For run instructions and deeper technical notes per feature, see `README.md` and `command_center/README.md`.

**Status legend**
- ✅ Done and tested (numbers below are measured, not estimated)
- 🟡 Done, but not yet tested on the real drone, or only partly done
- 🔲 Planned, not started

Last updated: 27 Sep 2026. Git branch: `feature/intelligent-search`. **Nothing on this branch is committed yet.**

---

## 1. Problem statement and how we answer it

> *A deployable AI-powered autonomous drone that aids search-and-rescue operations by detecting people and hazards, improving responder safety and reducing victim discovery time.*

| PS requirement | How we do it | Status |
|---|---|---|
| **Autonomous navigation** (GPS, obstacle avoidance) | ArduPilot GUIDED mode driven by our mission bridge. Bayesian search planner. Forward lidar + downward rangefinder obstacle guard. Mapped building heights (pre-disaster intel). Guarded return home. | ✅ in sim · 🔲 real hardware |
| GPS-denied navigation / SLAM | Not built | 🔲 |
| **On-device AI inference** | YOLOv8m (RGB) + YOLOv8n (thermal, trained on HIT-UAV) + a thermal-signature detector, all running locally. No cloud. | ✅ on laptop GPU · 🔲 on the drone (Pi 5 + Hailo planned) |
| **Multi-sensor fusion** (RGB, thermal, IMU, GPS) | Every RGB "person" is checked against the boresighted thermal frame: body temperature, size, skin temperature, no fire pixels. Detections are geo-projected with the drone's GPS + attitude at the frame's capture time. Flood = cold in thermal **and** flat in RGB. | ✅ in sim |
| **Hazard classification** (fire, smoke, flood, debris, unstable structures) | Fire / hot spots and flood water from thermal (+ RGB). Collapsed structures from mission intel. River vs flood from the known-water map. | ✅ fire, flood, structures · 🔲 smoke, landslides, chemical leaks, electrical lines |
| **Geo-tagged mapping** | Live situational map: probability heatmap, flight trail, casualties, hazard zones with safety buffers, A* safe ground routes, pre-disaster basemap | ✅ |
| **Emergency alerting + prioritised rescue** | Alert feed with severity and chime. Casualties ranked by nearby threats (fire/flood/structure), movement and route time. Recommended action per casualty. | ✅ |
| **Situational reports** | Printable SITREP page (save as PDF), generated from the drone's own data | ✅ |
| **Offline resilience** | Everything runs locally (no internet, no cloud, no external fonts/CDNs). The dashboard only needs the local link. | ✅ design · 🔲 comms relay / store-and-forward |
| **Command-center dashboard** | Web dashboard: live AI feed, thermal + split view, live map, drone status, forward-lidar profile, rescue priorities, alerts, logs, SITREP | ✅ |

---

## 2. System architecture

### 2.1 Simulation (working now)

```
 ┌──────────────── Gazebo Harmonic (world: runway / disaster / region) ───────────────┐
 │  S500 quad model: RGB cam (640x480, 10 Hz), thermal cam (L16 radiometric, 10 Hz),  │
 │  forward 3D lidar (48x16, 40 m), downward rangefinder, IMU, GPS, payload box       │
 │  Actors (casualties, 5 poses) + heat bodies · dogs · fires · flood · buildings     │
 └───────────┬──────────────────────────────────────┬──────────────────────────────────┘
             │ JSON sensor/servo (lockstep)          │ ros_gz_bridge (images, lidar, clock)
     ArduPilot SITL (ArduCopter 4.8)                 ▼
             │ MAVLink (MAVProxy → UDP 14550)   ROS 2 Humble topics
             ▼                                        │
   mavlink_bridge (ROS 2 node) ◄──── /detections/* ── person_detector.py (YOLO + fusion + hazards)
   mission state machine, planner,                    ▲
   obstacle guard, hazard map, routes                 │ /camera/image, /thermal/image
             │ /resq/map (JSON 2 Hz), services        
             ▼
   command_center/backend/server.py (Flask + rclpy) ──► browser dashboard (http://localhost:5000)
```

### 2.2 Real drone (target)

```
 Pixhawk 2.4.8 (ArduCopter, Pixhawk1-1M build)
   ├─ RC receiver ─── your RC transmitter (always the override)
   ├─ GPS/compass, power module (6200 mAh), 4x A2212 + 8" props on S500 frame
   ├─ TELEM1 ── SiK telemetry radio ~~~ radio on laptop USB ── MAVProxy ── bridge / Mission Planner / preflight
   └─ TELEM2 ── (phase 2) Raspberry Pi 5 ── RGB camera (+ thermal later) ── detector (Hailo-8L)
                                              └─ WiFi ── laptop dashboard
```

- **Phase 1 (now):** everything runs on the laptop, with MAVLink over the telemetry radio. First test: the **hop test**.
- **Phase 2:** bridge + detector on the Pi 5 (on-device AI). The laptop only shows the dashboard.

---

## 3. Components in detail

### 3.1 Simulation stack ✅

| Piece | What / where | Notes |
|---|---|---|
| Gazebo Harmonic | `ardu_ws/src/ardupilot_gz/` (launch, worlds), `ardu_ws/src/ardupilot_gazebo/` (models, ArduPilot plugin) | Launched by `s500_quad_runway.launch.py world:=<name>`. RViz off by default (`RESQ_RVIZ=1` to open it). |
| ArduPilot SITL | ArduCopter 4.8.0, JSON interface, lockstep | Same firmware family as the real Pixhawk |
| ROS 2 Humble | `ros2_ws/src/resq_mavlink_bridge` | Two workspaces; `command_center/backend/ros_env.sh` sources both |
| S500 model | `ardu_ws/src/ardupilot_gazebo/models/s500_quad/model.sdf` | Nadir RGB + thermal (boresighted, HFOV 1.2 rad, 640×480, **10 Hz**; 20 Hz made the Gazebo window stall). Forward lidar + downward rangefinder. Detachable payload. |
| GPU rendering | `ros_env.sh` forces NVIDIA EGL/GLX | Headless rendering had fallen back to the CPU (`dri2` warning) |
| ROS bridge config | `ardupilot_gz_bringup/config/s500_bridge.yaml` | Images, lidar points, rangefinder, clock, IMU, GPS, battery |

**Performance (measured, full mission):**
- Small world: 0.92× real time with the Gazebo window. GPU 36% mean / 63% peak.
- Region world: 1.00× headless (16% GPU); ~0.96× with the Gazebo window (46% GPU, a fixed cost of the window itself).

### 3.2 Worlds

| World | Size | Content | Status |
|---|---|---|---|
| `runway` | flat | 1 casualty, simple test | ✅ |
| `disaster` | 40×40 m search area | Earthquake aftermath. Collapsed buildings ring the area; rubble, cars, poles inside. **2 casualties**, 2 dogs (thermal distractors), smouldering debris (fire), flood pool, smoke plumes, staging area. | ✅ |
| `region` | **200×200 m** | Generated. City (NE), forest (NW), village (SW), river + flood (SE), launch pad at the centre. 6 seeded casualties in 5 poses. | ✅ |

**Region world generator:** `worldgen/generate_region.py`
- The whole landscape is **one static model of merged meshes**: 167 trees in one OBJ, all procedural buildings in another, ground/roads/fields in a third. About 6,500 triangles total. Generated tileable textures.
- No collision except the ground (keeps physics cheap).
- Fuel "hero" models, render-checked in Harmonic: apartment, office, post office, 3× Indian house, 2× collapsed house, bus, cars, truss bridge. Fuel House 1/2/3 render black and are not used.
- **Casualties are seeded**: `./start.sh region <seed>`. They are placed only on open ground the camera can see (rubble, street, lane, field, clearing, riverbank). Poses come from a separate random stream (lying 35%, crawling 15%, sitting 20%, waving 15%, walking 15%). Movers pace a clear path.
- Heat bodies per pose (skin 309 K, clothing 305 K). `worldgen/heat_follower.py` moves the heat of walking/crawling casualties with their actors, from the sim clock, at 10 Hz.
- It also writes the mission intel `resq_mavlink/mission_region.json`, containing only pre-disaster information:
  - building footprints + heights (`known_obstacles`)
  - the river outline (`known_water`)
  - a land-use prior raster (`prior_region.npy`)
  - the basemap PNG
  - the staging point

  Ground truth goes to `region_truth.json` (scoring only; the drone never reads it). Mover routes go to `region_actors.json`.
- **Demo seed: 3.** All 6 casualties are clearly visible to the detector (seeds 2–7 had 4–5 of 6).

### 3.3 Perception — `ardu_ws/person_detector.py` ✅ (sim)

**Person detection**
- **Day:** COCO YOLOv8m on RGB, run on the frame + 90°/180°/270° rotations (people lying from above face any way). Confidence ≥ 0.6.
- **Night:** YOLOv8n trained on HIT-UAV (drone thermal) on white-hot 283–313 K frames. Confidence ≥ 0.45.
- **Day also runs the thermal model and the thermal-signature detector**, merged with RGB.
- **Thermal-signature detector:** a person-sized warm region with a skin-temperature peak (≥ 308 K) and no fire pixels. It is the only route to **sitting** people.

**Fusion checks on every box, using the thermal frame:**
1. Body temperature: 97th percentile 303–316 K.
2. Fire rejection: the warm region must not reach 323 K (smouldering ash is 305 K but touches 330–360 K embers).
3. Size / skin: a warm region ≥ 1.1 m = lying or crawling. A compact one (0.35–1.1 m) counts only if its peak is ≥ 308 K (exposed skin). The dog's fur is at most 307 K and rejects as an animal. *This is a 2 K margin in simulation; it needs validating with real thermal footage.*

**Detection by pose, from above at 8 m:**

| Pose | Found by |
|---|---|
| Lying | RGB 0.92 + thermal model + signature |
| Crawling | RGB 0.91 + thermal model + signature |
| Walking | thermal model + signature |
| Waving | thermal model + signature |
| Sitting | signature only |
| Dog | rejected |

**Hazards (1 Hz scan of every thermal frame, day and night)**
- Fire / hot spot: > 323 K.
- Flood: colder than 283.5 K and larger than 1.5 m², plus RGB "flat and textureless" by day.
- Published on `/detections/hazards` (JSON with image offsets).

**Performance**
- GPU, capped at **5 analyses/s in FP16** (`quantize=16`): 0.05 s per analysis, which leaves the GPU for Gazebo.
- Live frames show the latest boxes (no freeze-frame when inference is fast).
- Output is JPEG `/detections/debug_image/compressed`. Raw ROS images cost ~93 ms per frame in Python, which had capped the feed at 3.4 fps.

### 3.4 Mission bridge — `ros2_ws/src/resq_mavlink_bridge/resq_mavlink_bridge/mavlink_bridge.py` ✅ (sim)

**Mission state machine**

`STARTING → ARMING → TAKEOFF → CLIMBING → SEARCHING ⇄ VERIFYING → MARKING → … → RETURNING → LANDING → COMPLETE`

Plus `HOVERING` (hop test), `ABORTED` (crash / arming failure) and `PILOT` (override).

**Mission types**
- `search` (default): full SAR mission.
- `hop` (hop test): take off to 20 m (configurable, 3–40 m), hover 10 s, then LAND straight down. **No horizontal command is ever sent.** Horizontal drift in sim: ≤ 1 cm.

**Search planners**
- `bayes` (default): the probability-map planner (§3.5). Flies to the most probability per second of flight. Replans every 1 s with 25% hysteresis.
- **Carrot:** it aims 2 s of flight past the target so ArduPilot doesn't brake at every hop. Average speed went from 1.47 → ~3 m/s at a commanded 5 m/s.
- `grid`: lawnmower, for comparison.
- **Livelock guard:** 30 s at one target with no progress writes that area off.

**Verification**
- Stop, fly over the candidate, hover.
- Confirm when ≥ 8 frames are analysed with a person in ≥ 20% of them (fusion has already removed animals and shadows).
- Fast-reject when < 8% of 25+ frames.
- Tracks the latest 3 sightings (for movers). Duplicate check at confirmation.
- Drone-shadow rejection by sun geometry (day).

**On confirm**
- `continue`: drop the aid kit 2.5 m beside the first casualty, mark later ones, keep searching.
- `land`: land beside the casualty.

**Triage:** a casualty whose sightings drift > 2.5 m is flagged **moving (responsive)** and ranked below still ones. *Only walkers are detectable; crawlers are too slow to separate from ~1 m jitter.*

**Hazard map:** hazard sightings are projected with the pose at capture time and merged (fire 8 m, flood 5 m). A zone is confirmed after 2 sightings. Flood detections inside the known river are ignored.

**Obstacle safety (lessons from 2 crashes in the region city)**
- **Obstacle guard** (`obstacle_guard.py`): levels the lidar points with roll/pitch and checks a 5 m corridor ahead (look-ahead 10–35 m, growing with speed).
  - Climbs to top + 4 m.
  - **Blocked** only if the needed climb can't be done in the time to reach the obstacle (2 m/s climb).
  - Tall walls beyond the lidar's view block within stopping distance.
  - Holds altitude 8 s. Won't descend onto roofs (rangefinder).
- **Mapped obstacles:** flies at building height + 4 m along the next 25 m of path, and climbs in place first when close.
- **Caution:** max 2 m/s and no carrot while an obstacle still needs climbing.
- **Brake-back hold:** the hold point is 1 s of travel *behind* the drone.
- **Crash / in-flight disarm detection:** critical alert, mission `ABORTED`.
- **Guarded return home** (instead of RTL at a fixed height).

**Pilot override** ✅ tested in sim: if the flight mode changes away from what the bridge set (e.g. the RC switch to LOITER), the bridge stops commanding immediately, shows **PILOT OVERRIDE**, and never switches back.

**Other safety:** arming timeout 20 s. The Pixhawk's STATUSTEXT (PreArm, failsafe, EKF…) is forwarded to the dashboard alerts.

**Aid drop:** Gazebo detach in sim. On the real drone, a servo via `RESQ_PAYLOAD_SERVO="<output>:<pwm>"` (e.g. `9:1900` = AUX1). There is also a `/resq/payload/drop` service.

**Configuration:**
- `mission.json` / `mission_region.json`: area, altitude, speed, planner, on-confirm, POS target, time budget, priors, hazards, obstacles, water, basemap.
- `RESQ_MISSION_FILE` overrides the mission file.
- Dashboard overrides via the latched `/resq/mission/config`.

### 3.5 Planning library — `search_map.py` ✅

- **ProbabilityMap** (Bayesian search theory): 2 m cells.
  - The prior comes from Gaussian zones or a raster (the region's land-use prior).
  - Each analysed frame multiplies the probability of the cells in the central 80% of the footprint by (1 − POD). POD is 0.7 RGB / 0.6 thermal, scaled by (8/alt)² above 8 m, and a look counts only once per second.
  - Gains use a small convolution (scales to 200 m; a dense matrix would need ~680 MB).
  - Probability of success = prior mass searched.
- **HazardMap:** merge distances per type; a zone is confirmed at 2 sightings; intel zones are pre-confirmed.
- **safe_route:** A* around hazard buffers (fire 4 m, flood 1.5 m, structure 3 m). Grid 1 m (coarser for big areas).
- **assess_victims:** threat score (fire < 15 m, flood < 10 m, structure < 8 m), moving −10, then route ETA at 1 m/s on foot → priority P1…Pn + recommended action.

### 3.6 Command center ✅

- **Backend** `command_center/backend/server.py` (Flask + rclpy):
  - Process manager (sim, bridge, feed, heat follower, drone link).
  - REST API + SSE: status, logs, map.
  - MJPEG streams: AI feed, raw thermal.
  - World generation per seed.
  - Day/night lighting via Gazebo services.
  - Real-drone mode (`RESQ_REAL=1`).
  - `RESQ_PORT` for a second instance.
- **Dashboard** `command_center/frontend/` (plain HTML/JS/CSS, no build step, works offline):
  - **Home:** AI feed with AI / THERMAL / SPLIT (draggable diagonal) views, HUD (alt/speed/heading/stage), compass, LIVE badge, alert toasts with chime, full screen, reticle. Mini map with zoom/follow. Drone status (altitude, speed, battery, satellites). Forward-lidar side profile. Environment and camera pickers. Mission strip (stage, probability of success, casualty and hazard chips). Action bar.
  - **Map:** full map (basemap, heatmap, trail, footprint, target, candidates, rejected, casualties with priority badges, hazards with buffers, safe routes, staging), rescue-priority cards, hazard list, alert log.
  - **Telemetry:** every value + process status + console.
  - **Settings:** world, casualty seed, mission type (search / hop test), hop altitude, planner, on-confirm.
  - **Log drawer** (Logs button or `L`): Events + Console with source filters and search.
  - **SITREP** (`/report`): KPIs, map, priorities table with coordinates, hazards, mission log; print to PDF.
  - Real-drone mode: *Connect Drone*, sim-only settings hidden.
- **`start.sh`:**
  - `./start.sh [disaster|runway|region [seed]|real [device] [baud]]`
  - Stops any previous run first (other `start.sh`, leftover processes, port 5000).
  - Real mode does not start the detector unless `RESQ_CAMERA=1`.

### 3.7 Tools ✅

| Tool | Purpose |
|---|---|
| `resq_mavlink/search_benchmark.py` | Monte Carlo: probability planner vs lawnmower, 40 m area |
| `resq_mavlink/region_benchmark.py` | Same over the region world, using the generator's real seeded casualties |
| `resq_mavlink/preflight.py` | Real-drone pre-flight check, props off (§4.4) |
| `worldgen/generate_region.py`, `worldgen/heat_follower.py` | Region world generation, moving heat |

---

## 4. Real drone integration

### 4.1 Hardware (as known)

| Item | Status |
|---|---|
| Frame | S500 quad, A2212 motors, 8" props, 6200 mAh battery |
| Flight controller | Pixhawk 2.4.8 (clone): flash the **ArduCopter "Pixhawk1-1M"** build (1 MB flash limit) |
| RC | Transmitter + receiver: **always the override** (keep LOITER + STABILIZE on the mode switch) |
| Telemetry | SiK radio pair (USB on the laptop) |
| Companion computer | **Raspberry Pi 5, running Pi OS** |
| Cameras | RGB camera. **No thermal camera yet.** |
| Lidar / rangefinder | None yet (optional) |

### 4.2 Phases

| Phase | Goal | Status |
|---|---|---|
| 0 | Real-drone software path (link mode, hop test, pilot override, preflight, servo drop) | ✅ built and tested against the simulated Pixhawk through the real-mode path |
| 1a | Bench, props OFF: `./start.sh real`, `preflight.py` all PASS, dashboard telemetry, arming, servo | 🔲 needs the drone |
| 1b | **Hop test**, open field, you on the RC: 20 m up, hover, land in place | 🔲 |
| 1c | Camera on the laptop: stream RGB from the Pi over WiFi; detector on the laptop; detection check by carrying the camera at ~8 m over a person lying on grass | 🔲 |
| 2 | On-device AI: bridge + detector on the Pi 5. ROS 2 via Docker (Pi OS has no native Humble) or Ubuntu 24.04. Detector: YOLOv8n (CPU) or the **Raspberry Pi AI Kit (Hailo-8L)** | 🔲 |
| 3 | Small autonomous search (20×20 m, volunteer / mannequin, empty field, obstacle guard off) | 🔲 |
| 4 | Thermal camera (radiometric: InfiRay P2 Pro / Topdon TC001 / FLIR Lepton 3.5 + PureThermal): enables night mode and the fusion checks | 🔲 |
| 5 | Optional forward lidar / rangefinder (TFmini-class downward for AGL first) | 🔲 |

### 4.3 How real mode works (Phase 0 ✅)

1. `./start.sh real` (defaults `/dev/ttyUSB0 57600`), or `./start.sh real /dev/ttyACM0 57600`.
2. **Connect Drone** starts MAVProxy on the radio, split to UDP **14550** (bridge, same as sim), **14551** (Mission Planner / QGroundControl as a safety monitor; in QGC add a UDP link on 14551), **14552** (preflight check).
3. Settings → **Mission type: Hop test**, altitude 20 → **Start Hop Test**.
4. Flip the RC mode switch at any time: the bridge stops (PILOT OVERRIDE).

### 4.4 Pre-flight checklist (`python3 resq_mavlink/preflight.py`)

It checks the link, ArduPilot firmware, disarmed state, GPS (3D fix, ≥ 8 satellites, HDOP < 1.5), sensor health, EKF, battery (V/cell), RC receiver, geofence, RTL altitude, RC-loss failsafe, battery failsafe, arming checks, a takeover mode on the switch, and PreArm messages.

**Parameters to set in Mission Planner before flying (the simulated Pixhawk's defaults failed these):**
- `FENCE_ENABLE=1`, `FENCE_TYPE=3`, `FENCE_ALT_MAX=30`, `FENCE_RADIUS=50`, `FENCE_ACTION=1`
- `BATT_MONITOR=4`, `BATT_LOW_VOLT≈3.6×cells`, `BATT_FS_LOW_ACT=2`
- `FS_THR_ENABLE=1`
- RTL altitude 10–30 m

### 4.5 Still to build for the real drone 🔲

- A shared **camera config file** (HFOV, resolution, mount) used by both the bridge and the detector. Today these are constants matching the sim camera.
- A Pi camera publisher (`/camera/image`) and ROS 2 on the Pi (Docker).
- The detector on the Pi (YOLOv8n export / Hailo) and a re-measurement of accuracy.
- A time sync (chrony) between Pi and laptop if frames are stamped on the Pi.
- A thermal camera adapter (each vendor's radiometric format → Kelvin).
- **Legal (India, Drone Rules 2021):** ~1.5 kg = small category. Register on Digital Sky (UIN), fly in green zones below 400 ft. Check current rules.

---

## 5. Measured results

| What | Result |
|---|---|
| Disaster world, full mission (GPU) | Both casualties found (T+19 s, T+34 s, ≤ 0.5 m error); fire and flood mapped; no false positives after fusion; mission 153 s search + landing |
| Dogs vs people | Before fusion: 2 dogs confirmed as casualties. After: 0 (120 RGB detections rejected as "likely an animal") |
| Planner benchmark, 40 m area | Probability map vs lawnmower median time-to-find: **51% faster** (intel-driven placement), 19% faster (random placement) |
| Region benchmark, 16 seeds × 6 casualties | Land-use map planner at 5 m/s found 43/67/81% by 5/10/15 min; lawnmower 31/57/62% |
| **Region, demo seed 3, full 25 min mission** | **All 6 found** (sitting, sitting, crawling, crawling in rubble, waving in a street, lying in a forest clearing); 5 of 6 in 7 min; errors 0.2–1.4 m; no false positives, no crash |
| Feed | 3.4 → ~6 fps AI feed at 10 Hz cameras (16 fps at 20 Hz headless); detector 0.05 s/analysis |
| Sim speed | Small world 0.92× with the Gazebo window; region 1.00× headless, ~0.96× with the window |
| Hop test (sim, and real-mode path) | 20 m in ~12 s, 10 s hover, LAND, disarmed; ≤ 1 cm drift |
| Pilot override (sim) | Detected ~2.5 s after the mode switch; bridge stopped; mode never forced back |

---

## 6. Known limitations (be honest with judges)

- **Nadir-only camera:** upright people are small from above. Sitting people are found only by thermal signature. A tilted gimbal would help.
- **Skin-temperature rule** (person vs dog) relies on a 2 K sim margin; unvalidated on real footage.
- **The "moving" triage flag** works for walkers only (crawlers are within projection jitter).
- **City streets between tall blocks** are searched from above the buildings (lower detection per look); the price of not crashing.
- **The forward-only lidar** can't see sideways or backwards. Mapped building heights cover known buildings; unmapped tall obstacles are the residual risk.
- **Benchmarks are optimistic:** they assume constant speed; real flights brake, climb and verify (region: benchmark ~80% by 15 min vs 2–5 of 6 in real sim runs before the latest fixes).
- **No GPS-denied navigation / SLAM, no comms relay** yet.
- **Sim-only pieces** (world generator, heat follower, day/night lighting) don't exist on the real drone.

---

## 7. Backlog — ideas discussed, not started 🔲

Roughly in priority order:
1. **Real drone phases 1–3** (§4.2): bench → hop test → camera → on-device AI → small search.
2. **Thermal camera** on the real drone (unlocks night mode and fusion).
3. **Video for the SIH submission:** cold open on a thermal lock, real drone → digital twin match cut, split-screen mission, "it says no" rejections (shadow, dog, cone), mid-mission day/night switch, payload drop, SITREP finale. Remotion for titles and captions.
4. **Remote triage** (challenge and response): hover, play a voice prompt (Hindi + local language), classify the response with pose estimation → START tags.
5. **Finding people cameras can't see:** an ESP32 sniffing phone WiFi probe / BLE RSSI; locate buried victims by multilateration; fuse with thermal/RGB.
6. **Comms resilience:** show the comms radius, store-and-forward ("data mule") return, or a second drone as a relay (`multiagent.launch.py` exists).
7. **More seeds / benchmark in the real sim** (headless batch runs), and tuning the region search speed/altitude further.
8. Smoke / landslide / electrical-line hazard classifiers.
9. GPS-denied navigation (visual odometry / SLAM).
10. A gimbal / oblique camera for upright people.

---

## 8. How to run

```bash
./start.sh                  # disaster world (40 m, 2 casualties)
./start.sh region 3         # 200 m region, demo seed 3 (6 casualties, 5 poses)
./start.sh region           # region, random placement
./start.sh real             # real drone via telemetry radio (/dev/ttyUSB0 57600)
RESQ_RVIZ=1 ./start.sh      # also open RViz
python3 resq_mavlink/preflight.py                 # with ./start.sh real running, props OFF
python3 resq_mavlink/search_benchmark.py          # planner benchmark (40 m)
python3 resq_mavlink/region_benchmark.py          # planner benchmark (region)
python3 worldgen/generate_region.py --scenery     # rebuild region scenery + intel
```

Dashboard: http://localhost:5000. Press **Start Search & Rescue** (or **Start Hop Test**). Ctrl+C in the terminal stops everything.

GPU note: PyTorch is the CUDA build (`torch 2.11.0+cu128`). Keep **numpy 1.26.4**, because numpy 2 breaks ROS Humble's `cv_bridge`. Never `pip install --force-reinstall` without `"numpy<2"`.

---

## 9. File map

| Path | Role |
|---|---|
| `start.sh` | One-command launcher, cleanup of previous runs, real mode |
| `command_center/backend/server.py`, `ros_env.sh` | Dashboard backend, process manager, ROS env (NVIDIA rendering) |
| `command_center/frontend/index.html`, `app.js`, `style.css` | Dashboard |
| `command_center/frontend/map.js` | Map renderer (dashboard + SITREP) |
| `command_center/frontend/report.html` | SITREP |
| `command_center/frontend/basemaps/region.png` | Pre-disaster basemap |
| `command_center/README.md` | Detailed technical notes and measurements per feature |
| `ros2_ws/src/resq_mavlink_bridge/resq_mavlink_bridge/mavlink_bridge.py` | Mission bridge (state machine, safety, map publishing) |
| `…/search_map.py` | Probability map, hazard map, A*, victim priorities |
| `…/obstacle_guard.py` | Lidar obstacle guard |
| `ardu_ws/person_detector.py` | Detector: YOLO RGB + thermal, fusion, signature, hazards |
| `ardu_ws/yolov8m.pt`, `ardu_ws/thermal_best.pt` | Models (COCO; HIT-UAV thermal) |
| `ardu_ws/src/ardupilot_gazebo/models/s500_quad/model.sdf` | Drone model and sensors |
| `ardu_ws/src/ardupilot_gazebo/models/region_scenery/` | Generated region landscape |
| `ardu_ws/src/ardupilot_gz/ardupilot_gz_gazebo/worlds/` | `runway.sdf`, `disaster.sdf`, `region.sdf` (generated) |
| `ardu_ws/src/ardupilot_gz/ardupilot_gz_bringup/config/s500_bridge.yaml` | Gazebo ↔ ROS topics |
| `worldgen/generate_region.py`, `heat_follower.py` | Region generator, moving heat |
| `resq_mavlink/mission.json` | Disaster-world mission + intel |
| `resq_mavlink/mission_region.json`, `prior_region.npy` | Region mission intel (generated) |
| `resq_mavlink/region_truth.json`, `region_actors.json` | Ground truth (scoring only), mover routes |
| `resq_mavlink/preflight.py` | Real-drone pre-flight check |
| `resq_mavlink/search_benchmark.py`, `region_benchmark.py` | Planner benchmarks |
