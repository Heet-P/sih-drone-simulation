import json
import math
import os
import subprocess
import time
from collections import deque

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray
from std_srvs.srv import SetBool
from std_srvs.srv import Trigger

from pymavlink import mavutil

from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2

import numpy as np

from resq_mavlink_bridge.obstacle_guard import ObstacleGuard
from resq_mavlink_bridge.search_map import (
    HazardMap, ProbabilityMap, assess_victims, footprint_corners,
)

# RESQ_MISSION_FILE overrides it, e.g. for a test mission or another world.
MISSION_FILE = os.environ.get(
    'RESQ_MISSION_FILE', '/home/heet/Desktop/SIH-Drone-Simulation/resq_mavlink/mission.json')
EARTH_RADIUS_M = 6371000.0

# Search grid defaults, overridable from mission.json's "search" block.
# The mission.json "target" is the search AREA center (last known
# position), not an exact person location.
DEFAULT_SEARCH_WIDTH_M = 40.0
DEFAULT_SEARCH_HEIGHT_M = 40.0
DEFAULT_SEARCH_ALTITUDE_M = 8.0
# 8m is where the detector was measured to be reliable (0.88-0.96
# confidence on rendered frames at 640px on CPU); it gets unreliable
# above ~10m.

# Re-asserted every SPEED_REASSERT_S: on this ArduPilot version the
# first position target after takeoff switches Guided into its
# position-control submode, and pva_control_start() resets the speed
# limit to WP_SPD (10 m/s here) - a one-off DO_CHANGE_SPEED is lost.
SEARCH_SPEED_MS = 2.0
SPEED_REASSERT_S = 2.0
LANE_OVERLAP = 0.3
WAYPOINT_RADIUS_M = 1.5

# Detect -> stop -> verify. Inference is ~1.3s per frame on CPU, so a
# moving drone gets maybe one look at the person per pass; on the
# first candidate the drone flies over it and hovers until the
# detector's multi-frame confirmation arrives.
VERIFY_TIMEOUT_S = 30.0
VERIFY_MAX_S = 90.0
VERIFY_SAMPLES = 2
# A candidate is only rejected on real negative evidence: at least this
# many frames analysed while hovering over it, mostly with no person.
# (Previously a timeout alone rejected it - on a slow machine where few
# frames got analysed, that blacklisted the real person's location.)
VERIFY_MIN_FRAMES = 3
# Fast rejection when the detector is fast (GPU, ~10 analysed frames a
# second): after VERIFY_FAST_MIN_S overhead with at least
# VERIFY_FAST_FRAMES analysed and a person in under VERIFY_FAST_HIT_RATIO
# of them, the evidence is already overwhelming - no need to wait out
# VERIFY_TIMEOUT_S (seen: 0/312 frames and still hovering).
# Confirmation: at least VERIFY_CONFIRM_MIN_FRAMES analysed while hovering
# overhead, with a person in at least VERIFY_CONFIRM_RATIO of them. A ratio
# can't tell a dog from a person by itself (before fusion a dog scored 12/40
# frames; a real casualty in a busy run 35/121), and the detector's short
# streak needs more than ~50% recall to build up, so it isn't required.
# What separates them is person_detector.py's RGB + thermal fusion, which
# drops animal and shadow boxes before they're counted: fused, the dog
# candidates scored 0/46 and 0/38 frames.
VERIFY_CONFIRM_MIN_FRAMES = 8
VERIFY_CONFIRM_RATIO = 0.2
VERIFY_FAST_MIN_S = 4.0
VERIFY_TRACK_N = 3
# The mean of the first and last VERIFY_TRACK_N sightings of a casualty
# further apart than this = they're moving (walking): responsive, a triage
# cue. Still casualties' sightings drift up to ~1.7 m (projection jitter, sd
# ~0.7-1.0 m, measured in the region world), so less can't be trusted. A
# crawler (0.3 m/s) moves only ~0.6 m in a ~2 s verification - not
# separable from jitter, so crawlers are not flagged.
MOVING_SPREAD_M = 2.5
VERIFY_FAST_FRAMES = 25
VERIFY_FAST_HIT_RATIO = 0.08
# Detections are projected using the pose at capture time (pose_history
# covers 20s), so age doesn't hurt accuracy; this only drops truly stale
# ones. Was 3s, which is about the inference time on a busy machine
# (2.9s measured with the Gazebo GUI + RViz open) - so nearly every
# verification sample was thrown away.
OFFSET_FRESH_S = 8.0
REJECTED_RADIUS_M = 4.0

# Land beside the casualty, not on them.
LANDING_STANDOFF_M = 2.5
ARRIVAL_RADIUS_M = 1.0

# Camera intrinsics, must match the camera sensor in
# ardupilot_gazebo/models/s500_quad/model.sdf.
CAMERA_HFOV_RAD = 1.2
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480

# Sun direction from the "sun" light in runway.sdf, world ENU
# (x=east, y=north, z=up). Used to predict where the drone's own
# shadow falls so it can never be accepted as a candidate.
SUN_DIRECTION_ENU = (-0.5, 0.1, -0.9)
SHADOW_REJECT_RADIUS_M = 2.5

# Probability-map search (planner "bayes"). POD = chance one analysed
# detector frame finds a person inside the central part of its
# footprint. Frames closer together than OBSERVE_MIN_GAP_S aren't
# independent looks (thermal runs at ~7 fps), so they count once.
PROB_CELL_M = 2.0
POD_PER_FRAME = {'day': 0.7, 'night': 0.6}
OBSERVE_MIN_GAP_S = 1.0
# While hovering to verify, keep this much probability around a
# pending detection: it isn't "searched" until verification decides.
DETECTION_EXCLUDE_M = 3.0
REPLAN_S = 1.0
REPLAN_HYSTERESIS = 1.25
TARGET_REACHED_M = 2.0
CARROT_S = 2.0
# Near an obstacle the drone has to climb over, fly at most this fast and
# don't aim past the target (a 5 m/s approach met an 18 m building only 9 m
# away, couldn't brake in time and crashed into it).
CAUTION_SPEED_MS = 2.0
# Mapped obstacles (mission.json known_obstacles): fly over them at
# height + OBSTACLE_CLEARANCE_M whenever the next INTEL_LOOKAHEAD_M of the
# path passes within their radius + INTEL_MARGIN_M; when that buffer is
# within INTEL_CLOSE_M, climb in place before going on.
OBSTACLE_CLEARANCE_M = 4.0
INTEL_LOOKAHEAD_M = 25.0
INTEL_MARGIN_M = 6.0
INTEL_CLOSE_M = 8.0
INTEL_CLIMB_SLACK_M = 1.0
# Blocked: aim this many seconds of travel BEHIND the drone, so it stops
# short instead of where "hold here" leaves it after braking (~5 m at 5 m/s).
BRAKE_BACK_S = 1.0
BRAKE_BACK_MAX_M = 5.0
TARGET_STUCK_S = 30.0
STUCK_NEAR_M = 12.0
STUCK_WRITE_OFF_M = 5.0
STUCK_KEEP = 0.3
DEFAULT_TARGET_POS = 0.9
DEFAULT_MAX_SEARCH_S = 900.0
# A confirmed casualty accounts for the probability around them, and new
# detections this close to one are the same person.
VICTIM_RADIUS_M = 4.0
# "continue" mode: hover over the drop point this long after release.
MARK_HOLD_S = 3.0
PAYLOAD_DETACH_TOPIC = '/model/s500_quad/payload/detach'
MAP_PUBLISH_S = 0.5
FLIGHT_STAGES = ('SEARCHING', 'VERIFYING', 'MARKING', 'APPROACH', 'RETURNING', 'HOVERING')
# Pilot override: in these stages the flight mode must stay what the bridge
# last set. If the pilot flips the RC mode switch (or anything else changes
# the mode) the bridge stops commanding at once and never switches back.
# MODE_GRACE_S covers the heartbeat lag after the bridge's own mode changes.
PILOT_WATCH_STAGES = ('ARMING', 'CLIMBING', 'HOVERING', 'SEARCHING', 'VERIFYING', 'MARKING',
                      'APPROACH', 'RETURNING', 'LANDING')
MODE_GRACE_S = 3.0
ARM_TIMEOUT_S = 20.0
# Hop test (mission_type "hop"): take off straight up, hover, land in place.
# No horizontal command is ever sent. The first mission for a real drone.
HOP_ALTITUDE_M = 20.0
HOP_HOLD_S = 10.0
# The Pixhawk's own text messages (PreArm failures, failsafes, crash
# check...) forwarded to the dashboard's alerts, at warning or worse, plus
# these at any severity.
STATUSTEXT_KEYWORDS = ('PreArm', 'Arm', 'Fence', 'failsafe', 'Failsafe', 'Crash', 'EKF', 'GPS', 'Battery')
# Water detected within this distance of a known (pre-disaster) water body
# is that body, not a flood.
KNOWN_WATER_MARGIN_M = 3.0
# Obstacle-aware altitude (obstacle_guard.py). Must match the
# obstacle_lidar's top vertical beam in s500_quad/model.sdf.
LIDAR_TOP_BEAM_RAD = 0.35
LIDAR_FRESH_S = 1.0
# Looks from above the search altitude (while flying over an obstacle)
# still count, with POD scaled by (search alt / alt)^2 - a person covers
# that many fewer pixels. Above this factor they aren't counted at all.
OBSERVE_MAX_ALT_FACTOR = 3.5  # up to 28 m: flying over city blocks at building height + 4 m
# Guarded return home, then land (instead of ArduPilot's RTL, which
# flies straight at a fixed height and knows nothing about obstacles).
HOME_REACHED_M = 1.5
ROUTE_REFRESH_S = 2.0
TRAIL_STEP_M = 0.5


def offset_latlon(lat, lon, north_m, east_m):
    d_lat = (north_m / EARTH_RADIUS_M) * (180.0 / math.pi)
    d_lon = (east_m / (EARTH_RADIUS_M * math.cos(math.radians(lat)))) * (180.0 / math.pi)
    return lat + d_lat, lon + d_lon


def north_east_between(lat1, lon1, lat2, lon2):
    """Flat-earth north/east metres from point 1 to point 2."""
    north = (lat2 - lat1) * (math.pi / 180.0) * EARTH_RADIUS_M
    east = (lon2 - lon1) * (math.pi / 180.0) * EARTH_RADIUS_M * math.cos(math.radians(lat1))
    return north, east


def distance_m(lat1, lon1, lat2, lon2):
    north, east = north_east_between(lat1, lon1, lat2, lon2)
    return math.hypot(north, east)


def near_polygon(x, y, poly, margin):
    """True if (x, y) is inside the polygon [[x, y], ...] or within margin of its edge."""
    inside = False
    j = len(poly) - 1
    for i in range(len(poly)):
        (xi, yi), (xj, yj) = poly[i], poly[j]
        if (yi > y) != (yj > y) and x < (xj - xi) * (y - yi) / (yj - yi + 1e-12) + xi:
            inside = not inside
        j = i
    if inside:
        return True
    for i in range(len(poly)):
        (x0, y0), (x1, y1) = poly[i], poly[(i + 1) % len(poly)]
        dx, dy = x1 - x0, y1 - y0
        t = max(0.0, min(1.0, ((x - x0) * dx + (y - y0) * dy) / (dx * dx + dy * dy + 1e-12)))
        if math.hypot(x - (x0 + t * dx), y - (y0 + t * dy)) <= margin:
            return True
    return False


def camera_ray_to_ground(offset_x, offset_y, alt, roll, pitch, yaw):
    """North/east metres from the drone to where a pixel's ray hits
    flat ground. Camera is rigidly mounted pointing straight down,
    image-up = vehicle nose, image-right = vehicle right (verified by
    rendering the sensor in Gazebo). Offsets are normalized
    (pixel - center) / image size. Returns None for rays that don't
    reach the ground."""
    focal = (CAMERA_WIDTH / 2.0) / math.tan(CAMERA_HFOV_RAD / 2.0)
    forward = -(offset_y * CAMERA_HEIGHT) / focal
    right = (offset_x * CAMERA_WIDTH) / focal
    down = 1.0

    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    # body (FRD) -> NED, ZYX Euler (ArduPilot convention)
    n = (cy * cp) * forward + (cy * sp * sr - sy * cr) * right + (cy * sp * cr + sy * sr) * down
    e = (sy * cp) * forward + (sy * sp * sr + cy * cr) * right + (sy * sp * cr - cy * sr) * down
    d = (-sp) * forward + (cp * sr) * right + (cp * cr) * down
    if d < 0.05:
        return None
    t = alt / d
    return n * t, e * t


def build_lawnmower(center_lat, center_lon, width, height, spacing, start_lat, start_lon):
    """Boustrophedon waypoints over a width (east) x height (north)
    box, lanes running north-south, starting at the corner nearest
    the given start position."""
    lanes = max(1, math.ceil(width / spacing))
    if lanes == 1:
        lane_east = [0.0]
    else:
        first = -width / 2.0 + spacing / 2.0
        last = width / 2.0 - spacing / 2.0
        lane_east = [first + i * (last - first) / (lanes - 1) for i in range(lanes)]

    start_north, start_east = north_east_between(center_lat, center_lon, start_lat, start_lon)
    if start_east > 0:
        lane_east.reverse()
    ends = [-height / 2.0, height / 2.0]
    if start_north > 0:
        ends.reverse()

    waypoints = []
    for i, east in enumerate(lane_east):
        a, b = ends if i % 2 == 0 else ends[::-1]
        waypoints.append(offset_latlon(center_lat, center_lon, a, east))
        waypoints.append(offset_latlon(center_lat, center_lon, b, east))
    return waypoints


class MavlinkBridge(Node):

    def __init__(self):
        super().__init__('mavlink_bridge')

        self.get_logger().info('Connecting to ArduPilot SITL...')
        self.vehicle = mavutil.mavlink_connection('udp:127.0.0.1:14550')
        self.get_logger().info('Waiting for heartbeat...')
        self.vehicle.wait_heartbeat()
        self.get_logger().info(
            f'Connected to ArduPilot! System ID: {self.vehicle.target_system}, '
            f'Component ID: {self.vehicle.target_component}'
        )
        self.request_message_rate(mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 10)
        self.request_message_rate(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 10)
        self.request_message_rate(mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT, 2)

        # Telemetry
        self.latitude = 0.0
        self.longitude = 0.0
        self.altitude = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.armed = False
        self.mode = 'UNKNOWN'
        self.battery_percentage = -1
        self.battery_voltage = 0.0
        self.ground_speed = 0.0
        self.vel_n = self.vel_e = 0.0
        self.satellites = 0
        self.gps_fix = 0  # MAVLink GPS_FIX_TYPE: 3 = 3D fix
        # (wall time, lat, lon, alt, roll, pitch, yaw) for looking up
        # where the drone was when a detection's frame was captured.
        self.pose_history = deque(maxlen=200)

        # Mission
        self.mission_running = False
        self.mission_stage = 'IDLE'
        self.stage_started = time.time()
        self.last_command_time = 0.0
        self.last_speed_time = 0.0

        self.search_center_lat = 0.0
        self.search_center_lon = 0.0
        self.search_width = DEFAULT_SEARCH_WIDTH_M
        self.search_height = DEFAULT_SEARCH_HEIGHT_M
        self.search_altitude = DEFAULT_SEARCH_ALTITUDE_M
        self.search_speed = SEARCH_SPEED_MS
        self.known_water = []   # pre-disaster water outlines (local E/N polygons)
        self.basemap = None     # pre-disaster map image for the dashboard
        self.waypoints = []
        self.waypoint_index = 0
        self.home_lat = None
        self.home_lon = None

        # Detection
        self.person_confirmed = False
        self.last_offset = None  # (captured wall time, x, y, conf)
        self.candidate = None  # [lat, lon]
        self.settled_at = None
        self.verify_samples = []
        self.verify_frames = 0
        self.verify_hits = 0
        self.rejected = []
        self.sensor_mode = 'day'
        self.landing_lat = None
        self.landing_lon = None

        # Situational map. Local frame: metres east/north of `origin`,
        # the first GPS fix (the drone on its launch pad).
        self.origin = None
        self.config = {}
        self.config_override = {}
        self.mission_id = None
        self.mission_t0 = None
        self.search_t0 = None
        self.pmap = None
        self.hazards = HazardMap()
        self.victims = []
        self.events = deque(maxlen=60)
        self.events_total = 0  # events ever raised, so clients can tell which are new
        self.trail = []
        self.staging = (0.0, 0.0)
        self.plan_target = None  # (e, n)
        self.stuck_target = None
        self.stuck_since = 0.0
        self.last_plan_time = 0.0
        self.last_observe_capture = 0.0
        self.last_detector_frame = 0.0
        self.verify_best_conf = 0.0
        self.mark_point = None  # (lat, lon) where the kit is dropped
        self.marked_at = None
        self.payload_dropped = False
        self.routes_dirty = True
        self.last_route_time = 0.0

        # Obstacle-aware altitude
        self.guard = ObstacleGuard(LIDAR_TOP_BEAM_RAD)
        self.guard_raised = False
        self.guard_blocked = False
        self.caution = False
        self.expected_mode = None
        self.mode_set_at = 0.0
        self.last_statustext = {}
        self.known_obstacles = []
        self.last_intel_event = 0.0
        self.lidar_warned = False
        self.last_blocked_event = 0.0

        self.position_publisher = self.create_publisher(String, '/resq/drone/position', 10)
        self.state_publisher = self.create_publisher(String, '/resq/drone/state', 10)

        self.create_service(SetBool, '/resq/mission/start', self.start_mission)
        self.create_service(Trigger, '/resq/mission/land', self.land_now)
        self.create_service(Trigger, '/resq/payload/drop', self.payload_service)

        self.create_subscription(Bool, '/detections/person_confirmed', self._on_person_confirmed, 10)
        self.create_subscription(PointStamped, '/detections/person_offset', self._on_person_offset, 10)
        self.create_subscription(Detection2DArray, '/detections/persons', self._on_detection_frame, 10)
        self.create_subscription(
            String, '/resq/sensor_mode', self._on_sensor_mode,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.create_subscription(
            String, '/resq/mission/config', self._on_mission_config,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.create_subscription(String, '/detections/hazards', self._on_hazards, 10)
        self.create_subscription(PointCloud2, '/obstacle_lidar/points', self._on_lidar, qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/down_range', self._on_down_range, qos_profile_sensor_data)
        self.map_publisher = self.create_publisher(String, '/resq/map', 10)

        self.create_timer(0.1, self.update)
        self.create_timer(MAP_PUBLISH_S, self.publish_map)
        self.get_logger().info('RESQ-MESH MAVLink bridge online. Mission service: /resq/mission/start')

    # ---------------------------------------------------------------
    # Main loop, telemetry, publishing
    # ---------------------------------------------------------------

    def update(self):
        self.read_telemetry()
        self.pose_history.append((
            time.time(), self.latitude, self.longitude, self.altitude,
            self.roll, self.pitch, self.yaw,
        ))
        self.publish_position()
        self.publish_state()
        self.record_trail()
        self.run_mission()

    # ---------------------------------------------------------------
    # Local map frame
    # ---------------------------------------------------------------

    def to_local(self, lat, lon):
        north, east = north_east_between(self.origin[0], self.origin[1], lat, lon)
        return east, north

    def to_global(self, east, north):
        return offset_latlon(self.origin[0], self.origin[1], north, east)

    def mission_time(self):
        return time.time() - self.mission_t0 if self.mission_t0 else 0.0

    def event(self, level, text):
        """Alert for the dashboard's alert feed (and the log)."""
        self.events.append({'t': round(self.mission_time(), 1), 'level': level, 'text': text})
        self.events_total += 1
        # Separate call sites: rclpy won't let one call site change severity.
        if level in ('warn', 'critical'):
            self.get_logger().warn(f'[{level.upper()}] {text}')
        else:
            self.get_logger().info(f'[{level.upper()}] {text}')

    def record_trail(self):
        if self.origin is None or not self.armed or self.altitude < 1.0:
            return
        e, n = self.to_local(self.latitude, self.longitude)
        if not self.trail or math.hypot(e - self.trail[-1][0], n - self.trail[-1][1]) >= TRAIL_STEP_M:
            self.trail.append([round(e, 2), round(n, 2)])
            if len(self.trail) > 4000:
                self.trail = self.trail[::2]

    def read_telemetry(self):
        while True:
            message = self.vehicle.recv_match(blocking=False)
            if message is None:
                break
            message_type = message.get_type()
            if message_type == 'GLOBAL_POSITION_INT':
                self.latitude = message.lat / 1e7
                self.longitude = message.lon / 1e7
                self.altitude = message.relative_alt / 1000.0
                self.ground_speed = math.hypot(message.vx, message.vy) / 100.0
                self.vel_n, self.vel_e = message.vx / 100.0, message.vy / 100.0
                if self.origin is None and message.lat != 0:
                    self.origin = (self.latitude, self.longitude)
                    self.get_logger().info(
                        f'Map origin (launch point): {self.latitude:.7f}, {self.longitude:.7f}'
                    )
            elif message_type == 'GPS_RAW_INT':
                self.satellites = message.satellites_visible
                self.gps_fix = message.fix_type
            elif message_type == 'ATTITUDE':
                self.roll = message.roll
                self.pitch = message.pitch
                self.yaw = message.yaw
            elif message_type == 'HEARTBEAT':
                if message.get_srcComponent() != mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1:
                    continue
                self.armed = bool(message.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                self.mode = mavutil.mode_string_v10(message)
            elif message_type == 'STATUSTEXT':
                self.on_statustext(message)
            elif message_type == 'BATTERY_STATUS':
                self.battery_percentage = message.battery_remaining
                voltage = message.voltages[0]
                if voltage != 65535:
                    self.battery_voltage = voltage / 1000.0

    def on_statustext(self, message):
        text = message.text.strip() if isinstance(message.text, str) else str(message.text)
        if not text:
            return
        important = message.severity <= mavutil.mavlink.MAV_SEVERITY_WARNING
        if not (important or any(k in text for k in STATUSTEXT_KEYWORDS)):
            return
        now = time.time()
        if self.last_statustext.get(text, 0.0) > now - 10.0:
            return  # the Pixhawk repeats PreArm messages every few seconds
        self.last_statustext[text] = now
        self.event('warn' if important else 'info', f'Pixhawk: {text}')

    def publish_position(self):
        msg = String()
        msg.data = (
            f'Latitude: {self.latitude:.7f}, '
            f'Longitude: {self.longitude:.7f}, '
            f'Altitude: {self.altitude:.2f} m'
        )
        self.position_publisher.publish(msg)

    def publish_state(self):
        msg = String()
        msg.data = (
            f'Armed: {self.armed}, '
            f'Mode: {self.mode}, '
            f'Battery: {self.battery_percentage}%, '
            f'Voltage: {self.battery_voltage:.2f} V, '
            f'Altitude: {self.altitude:.2f} m, '
            f'Mission: {self.mission_stage}, '
            f'Search: {min(self.waypoint_index, len(self.waypoints))}/{len(self.waypoints)}'
        )
        self.state_publisher.publish(msg)

    # ---------------------------------------------------------------
    # Detection input
    # ---------------------------------------------------------------

    def _on_sensor_mode(self, msg):
        mode = msg.data.strip().lower()
        if mode in ('day', 'night') and mode != self.sensor_mode:
            self.get_logger().info(f'Sensor mode -> {mode}')
            self.sensor_mode = mode

    def _on_mission_config(self, msg):
        try:
            override = json.loads(msg.data)
        except ValueError:
            self.get_logger().warn(f'Ignoring bad mission config: {msg.data!r}')
            return
        self.config_override = {k: v for k, v in override.items() if v not in (None, '')}
        self.get_logger().info(f'Mission config from command center: {self.config_override}')

    def _on_detection_frame(self, msg):
        """One message per frame the detector analysed (empty = no person).
        Counted while hovering over a candidate, as evidence for/against,
        and used to update the probability map: every analysed frame
        is a look at the ground under it."""
        self.last_detector_frame = time.time()
        if self.mission_stage == 'VERIFYING' and self.settled_at is not None:
            self.verify_frames += 1
            if msg.detections:
                self.verify_hits += 1
        self.observe_frame(msg)

    def observe_frame(self, msg):
        if self.pmap is None or self.mission_stage not in ('SEARCHING', 'VERIFYING', 'MARKING'):
            return
        # Stamped with the wall-clock capture time by person_detector.py.
        captured = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if captured - self.last_observe_capture < OBSERVE_MIN_GAP_S:
            return
        pose = self.pose_at(captured)
        if pose is None or abs(pose[0] - captured) > 1.0:
            return
        _, lat, lon, alt, roll, pitch, yaw = pose
        if alt < self.search_altitude * 0.7 or alt > self.search_altitude * OBSERVE_MAX_ALT_FACTOR:
            return
        self.last_observe_capture = captured
        # Cells around anything detected in this frame aren't searched
        # yet - verification decides those.
        keep = []
        for det in msg.detections:
            ox = (det.bbox.center.position.x - CAMERA_WIDTH / 2.0) / CAMERA_WIDTH
            oy = (det.bbox.center.position.y - CAMERA_HEIGHT / 2.0) / CAMERA_HEIGHT
            ground = camera_ray_to_ground(ox, oy, alt, roll, pitch, yaw)
            if ground is not None:
                keep.append(self.to_local(*offset_latlon(lat, lon, ground[0], ground[1])))
        before = self.pmap.p.copy()
        e, n = self.to_local(lat, lon)
        pod = POD_PER_FRAME.get(self.sensor_mode, 0.6) * min(1.0, (self.search_altitude / alt) ** 2)
        self.pmap.observe(
            e, n, alt, yaw, pod,
            CAMERA_HFOV_RAD, CAMERA_WIDTH, CAMERA_HEIGHT,
        )
        for ke, kn in keep:
            mask = (self.pmap.ee - ke) ** 2 + (self.pmap.nn - kn) ** 2 <= DETECTION_EXCLUDE_M ** 2
            self.pmap.p[mask] = before[mask]

    def _on_hazards(self, msg):
        """Hazard sightings from the detector (normalized image offsets),
        projected to the ground with the pose at capture time and merged
        into geo-tagged hazard zones."""
        if self.origin is None or self.altitude < 3.0:
            return
        try:
            data = json.loads(msg.data)
        except ValueError:
            return
        pose = self.pose_at(data.get('stamp', 0.0))
        if pose is None:
            return
        _, lat, lon, alt, roll, pitch, yaw = pose
        focal = (CAMERA_WIDTH / 2.0) / math.tan(CAMERA_HFOV_RAD / 2.0)
        for hz in data.get('hazards', []):
            ground = camera_ray_to_ground(hz['cx'], hz['cy'], max(alt, 1.0), roll, pitch, yaw)
            if ground is None:
                continue
            e, n = self.to_local(*offset_latlon(lat, lon, ground[0], ground[1]))
            radius = hz.get('r_px', 0.0) * alt / focal
            if hz['type'] == 'flood' and any(near_polygon(e, n, poly, KNOWN_WATER_MARGIN_M) for poly in self.known_water):
                continue  # the river/lake on the pre-disaster map, not new flooding
            zone, newly = self.hazards.add(
                hz['type'], e, n, radius, round(self.mission_time(), 1),
                peak_k=hz.get('peak_k'), conf=hz.get('conf'), sensors=hz.get('sensors', ()),
            )
            if newly:
                self.routes_dirty = True
                temp = f", {zone['peak_k'] - 273.15:.0f} °C" if zone.get('peak_k') else ''
                via = ' + '.join(s.upper() for s in zone['sensors'])
                self.event(
                    'critical' if zone['type'] == 'fire' else 'warn',
                    f"HAZARD {zone['id']}: {zone['label']} at E{zone['e']:+.0f} N{zone['n']:+.0f} m, "
                    f"~{zone['radius']:.1f} m across{temp} ({via})",
                )

    def _on_person_confirmed(self, msg):
        self.person_confirmed = msg.data

    def _on_person_offset(self, msg):
        captured = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.last_offset = (captured, msg.point.x, msg.point.y, msg.point.z)

    def pose_at(self, wall_time):
        if not self.pose_history:
            return None
        return min(self.pose_history, key=lambda p: abs(p[0] - wall_time))

    def project_offset(self, offset):
        """Ground lat/lon of a detection, using the drone's pose at the
        moment its frame was captured. Returns (lat, lon) or None;
        rejects the drone's own shadow."""
        captured, ox, oy, conf = offset
        pose = self.pose_at(captured)
        if pose is None:
            return None
        _, lat, lon, alt, roll, pitch, yaw = pose
        ground = camera_ray_to_ground(ox, oy, max(alt, 1.0), roll, pitch, yaw)
        if ground is None:
            return None
        spot = offset_latlon(lat, lon, ground[0], ground[1])
        shadow_dist = self.distance_to_own_shadow(spot, pose)
        self.get_logger().info(
            f'detection conf={conf:.2f} px=({ox:+.2f},{oy:+.2f}) -> ground '
            f'N{ground[0]:+.1f} E{ground[1]:+.1f} m from drone (alt {alt:.1f}, '
            f'roll {math.degrees(roll):+.0f}, pitch {math.degrees(pitch):+.0f}, '
            f'hdg {math.degrees(yaw) % 360:.0f}); own shadow {shadow_dist:.1f} m away'
        )
        if self.sensor_mode == 'day' and shadow_dist < SHADOW_REJECT_RADIUS_M:
            # No sun at night, so no shadow - and skipping this avoids
            # rejecting a real person who happens to be at that spot.
            self.get_logger().info("  -> rejected: the drone's own shadow")
            return None
        return spot

    @staticmethod
    def distance_to_own_shadow(spot, pose):
        """Distance from a ground point to where the drone's shadow was
        at that pose - the SAME pose the detection was projected from,
        not the current one (the drone may have moved metres since)."""
        _, lat, lon, alt, _, _, _ = pose
        sx, sy, sz = SUN_DIRECTION_ENU
        shadow = offset_latlon(lat, lon, alt * sy / -sz, alt * sx / -sz)
        return distance_m(spot[0], spot[1], shadow[0], shadow[1])

    # ---------------------------------------------------------------
    # Obstacle-aware altitude
    # ---------------------------------------------------------------

    def _on_lidar(self, msg):
        offsets = {f.name: f.offset for f in msg.fields}
        if not {'x', 'y', 'z'} <= offsets.keys() or msg.point_step == 0:
            return
        raw = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(-1, msg.point_step)
        pts = np.column_stack([
            raw[:, offsets[c]:offsets[c] + 4].copy().view(np.float32).ravel() for c in ('x', 'y', 'z')
        ])
        pts = pts[np.all(np.isfinite(pts), axis=1)]
        rng = np.linalg.norm(pts, axis=1)
        pts = pts[(rng > 0.6) & (rng < 39.5)]
        self.guard.update_scan(pts, self.altitude, self.roll, self.pitch, self.ground_speed, time.time())

    def _on_down_range(self, msg):
        r = msg.ranges[0] if msg.ranges else float('inf')
        self.guard.update_range(r if msg.range_min <= r <= msg.range_max else None)

    def lidar_live(self):
        return time.time() - self.guard.last_scan < LIDAR_FRESH_S

    def flight_altitude(self):
        """Search altitude, raised by the obstacle guard for whatever is
        ahead of or below the drone (when the lidar is live)."""
        if not self.lidar_live():
            if self.mission_running and not self.lidar_warned and self.mission_stage == 'SEARCHING':
                self.lidar_warned = True
                self.event('warn', 'Obstacle lidar offline: flying at a fixed search altitude.')
            return self.search_altitude
        return self.guard.command_altitude(self.search_altitude, self.altitude, time.time())

    def guarded_target(self, lat, lon):
        """Fly toward (lat, lon) at the obstacle-safe altitude. If an
        obstacle blocks the path at the drone's level, hold position and
        climb until the lidar sees over it."""
        alt = self.flight_altitude()
        # Mapped obstacles first (pre-disaster building heights), lidar as
        # backup: the forward lidar can't see sideways or backwards.
        intel_alt, intel_close, intel_what = self.intel_altitude(lat, lon)
        alt = max(alt, intel_alt)
        climb_first = intel_close and self.altitude < intel_alt - INTEL_CLIMB_SLACK_M
        if climb_first and time.time() - self.last_intel_event > 8.0:
            self.last_intel_event = time.time()
            self.event('info', f'Mapped {intel_what} ahead: climbing to {intel_alt:.0f} m before going on.')
        lidar_blocked = self.lidar_live() and self.guard.blocked
        blocked = lidar_blocked or climb_first
        # Hysteresis, so small climbs over huts and rubble don't flood the
        # alerts (the region world has dozens): announced from 3 m above the
        # search altitude until back within 0.5 m. Smaller adjustments still
        # show on the dashboard's Forward lidar card.
        margin = 0.5 if self.guard_raised else 3.0
        raised = alt > self.search_altitude + margin
        g = self.guard
        now = time.time()
        if lidar_blocked and not self.guard_blocked and now - self.last_blocked_event > 5.0:
            self.last_blocked_event = now
            ahead = f'{g.ahead_m:.0f} m ahead' if g.ahead_m is not None else 'ahead'
            self.event('warn', f'Obstacle at flight level {ahead}: holding position and climbing.')
        if raised and not self.guard_raised:
            what = f'{g.top_m:.1f} m tall' if g.top_m is not None else 'below'
            where = f', {g.ahead_m:.0f} m ahead' if g.ahead_m is not None else ''
            self.event('info', f'Obstacle ({what}{where}): climbing to {alt:.1f} m.')
        elif not raised and self.guard_raised:
            self.event('info', f'Path clear: back to search altitude {self.search_altitude:.0f} m.')
        self.guard_blocked, self.guard_raised = lidar_blocked, raised
        if blocked:
            back_n, back_e = -self.vel_n * BRAKE_BACK_S, -self.vel_e * BRAKE_BACK_S
            d = math.hypot(back_n, back_e)
            if d > BRAKE_BACK_MAX_M:
                back_n, back_e = back_n * BRAKE_BACK_MAX_M / d, back_e * BRAKE_BACK_MAX_M / d
            lat, lon = offset_latlon(self.latitude, self.longitude, back_n, back_e)
        self.send_gps_target(lat, lon, alt)
        self.update_caution(intel_alt)

    def intel_altitude(self, lat, lon):
        """Altitude needed to clear the mapped obstacles along the next
        INTEL_LOOKAHEAD_M of the path to (lat, lon): (altitude, an obstacle is
        close, its label). Close = the drone is within INTEL_CLOSE_M of the
        obstacle's buffer, so it must climb before going on."""
        if not self.known_obstacles or self.origin is None:
            return 0.0, False, ''
        e, n = self.to_local(self.latitude, self.longitude)
        te, tn = self.to_local(lat, lon)
        de, dn = te - e, tn - n
        dist = math.hypot(de, dn)
        if dist > INTEL_LOOKAHEAD_M:
            de, dn = de * INTEL_LOOKAHEAD_M / dist, dn * INTEL_LOOKAHEAD_M / dist
        seg2 = de * de + dn * dn
        need, close, what = 0.0, False, ''
        for ob in self.known_obstacles:
            oe, on_ = ob['east_m'], ob['north_m']
            t = 0.0 if seg2 < 1e-9 else max(0.0, min(1.0, ((oe - e) * de + (on_ - n) * dn) / seg2))
            d_path = math.hypot(oe - (e + t * de), on_ - (n + t * dn))
            buffer = ob['radius_m'] + INTEL_MARGIN_M
            if d_path < buffer and ob['height_m'] + OBSTACLE_CLEARANCE_M > need:
                need = ob['height_m'] + OBSTACLE_CLEARANCE_M
                what = f"{ob['label'].lower()} ({ob['height_m']:.0f} m)"
                close = math.hypot(oe - e, on_ - n) < buffer + INTEL_CLOSE_M
        return need, close, what

    def update_caution(self, intel_alt):
        """Slow down while an obstacle ahead (seen or mapped) still needs climbing over."""
        g = self.guard
        caution = (self.lidar_live() and g.ahead_m is not None and (g.blocked or g.required_fwd > self.altitude + 0.5)) \
            or intel_alt > self.altitude + 0.5
        if caution != self.caution:
            self.caution = caution
            self.last_speed_time = 0.0  # re-send the speed limit now

    def current_speed_limit(self):
        return min(self.search_speed, CAUTION_SPEED_MS) if self.caution else self.search_speed

    def hold_search_speed(self):
        now = time.time()
        if now - self.last_speed_time >= SPEED_REASSERT_S:
            self.last_speed_time = now
            self.set_speed(self.current_speed_limit())

    def fresh_offset(self, not_before=0.0):
        if self.last_offset is None:
            return None
        captured = self.last_offset[0]
        if time.time() - captured > OFFSET_FRESH_S or captured < not_before:
            return None
        return self.last_offset

    # ---------------------------------------------------------------
    # Services
    # ---------------------------------------------------------------

    def start_mission(self, request, response):
        if not request.data:
            response.success = True
            response.message = 'Mission start request ignored.'
            return response
        if self.mission_running:
            response.success = False
            response.message = 'A mission is already running.'
            return response

        try:
            with open(MISSION_FILE, 'r') as file:
                mission = json.load(file)
            search = mission.get('search', {})
            self.search_center_lat = mission['target']['latitude']
            self.search_center_lon = mission['target']['longitude']
            self.search_width = float(search.get('width_m', DEFAULT_SEARCH_WIDTH_M))
            self.search_height = float(search.get('height_m', DEFAULT_SEARCH_HEIGHT_M))
            self.search_altitude = float(search.get('altitude_m', DEFAULT_SEARCH_ALTITUDE_M))
            mission_id = mission.get('mission_id', 'UNKNOWN')
            config = {
                'planner': search.get('planner', 'bayes'),
                'on_confirm': search.get('on_confirm', 'continue'),
                'target_pos': float(search.get('target_pos', DEFAULT_TARGET_POS)),
                'max_search_s': float(search.get('max_search_s', DEFAULT_MAX_SEARCH_S)),
                'speed_ms': float(search.get('speed_ms', SEARCH_SPEED_MS)),
                'mission_type': 'search',
                'hop_altitude_m': HOP_ALTITUDE_M,
                'hop_hold_s': HOP_HOLD_S,
            }
            config.update(self.config_override)
            if config['planner'] not in ('bayes', 'grid'):
                raise ValueError(f"planner must be 'bayes' or 'grid', not {config['planner']!r}")
            if config['on_confirm'] not in ('continue', 'land'):
                raise ValueError(f"on_confirm must be 'continue' or 'land', not {config['on_confirm']!r}")
            if config['mission_type'] not in ('search', 'hop'):
                raise ValueError(f"mission_type must be 'search' or 'hop', not {config['mission_type']!r}")
            config['hop_altitude_m'] = min(40.0, max(3.0, float(config['hop_altitude_m'])))
            config['hop_hold_s'] = min(120.0, max(0.0, float(config['hop_hold_s'])))
            if self.origin is None:
                raise ValueError('no GPS fix yet - wait for the sim to finish starting')
            staging = mission.get('staging', {})
            staging = (float(staging.get('east_m', 0.0)), float(staging.get('north_m', 0.0)))
            center = self.to_local(self.search_center_lat, self.search_center_lon)
            prior_map = None
            if mission.get('prior_map'):
                pm = mission['prior_map']
                path = pm['file'] if os.path.isabs(pm['file']) else os.path.join(os.path.dirname(MISSION_FILE), pm['file'])
                prior_map = (np.load(path), float(pm['e0']), float(pm['n0']), float(pm['cell_m']))
            pmap = ProbabilityMap(
                center, self.search_width, self.search_height, PROB_CELL_M,
                zones=mission.get('prior_zones', []), prior_map=prior_map,
            )
            hazards = HazardMap()
            for known in mission.get('known_hazards', []):
                hazards.add_known(
                    known['type'], float(known['east_m']), float(known['north_m']),
                    float(known['radius_m']), known.get('label'),
                )
        except Exception as error:  # noqa: BLE001 - report any bad mission file to the caller
            self.get_logger().error(f'Failed to load mission: {error}')
            response.success = False
            response.message = str(error)
            return response

        self.get_logger().info(
            f'Loaded mission {mission_id}: search {self.search_width:.0f}x'
            f'{self.search_height:.0f} m around {self.search_center_lat:.7f}, '
            f'{self.search_center_lon:.7f} at {self.search_altitude:.0f} m'
        )
        self.home_lat, self.home_lon = self.latitude, self.longitude
        self.waypoints = []
        self.waypoint_index = 0
        self.person_confirmed = False
        self.last_offset = None
        self.candidate = None
        self.rejected = []
        self.mission_id = mission_id
        self.config = config
        self.search_speed = config['speed_ms']
        self.known_water = mission.get('known_water', [])
        self.known_obstacles = mission.get('known_obstacles', [])
        self.basemap = mission.get('basemap')
        self.pmap = pmap
        self.hazards = hazards
        self.staging = staging
        self.victims = []
        self.events.clear()
        self.events_total = 0
        self.trail = []
        self.plan_target = None
        self.mark_point = None
        self.routes_dirty = True
        self.mission_t0 = time.time()
        self.search_t0 = None
        self.lidar_warned = False
        self.guard_raised = self.guard_blocked = False
        self.mission_running = True
        if config['mission_type'] == 'hop':
            self.search_altitude = config['hop_altitude_m']
            self.event('info', f"Hop test started: take off to {self.search_altitude:.0f} m, hover "
                               f"{config['hop_hold_s']:.0f} s, land in place. Flip the RC mode switch "
                               'at any time to take over.')
        else:
            planner = 'probability-map (Bayesian)' if config['planner'] == 'bayes' else 'lawnmower grid'
            after = 'mark, drop aid kit and keep searching' if config['on_confirm'] == 'continue' else 'land beside them'
            self.event('info', f'Mission {mission_id} started: {planner} search; on each casualty: {after}.')
        self.expected_mode = None
        self.set_stage('STARTING')

        response.success = True
        response.message = f'Mission {mission_id} started.'
        return response

    def land_now(self, request, response):
        self.get_logger().info('Manual land requested.')
        self.set_land_mode()
        self.mission_running = True
        self.set_stage('LANDING')
        response.success = True
        response.message = 'Landing now.'
        return response

    # ---------------------------------------------------------------
    # Mission state machine
    # ---------------------------------------------------------------

    def set_stage(self, stage):
        self.get_logger().info(f'Mission stage: {self.mission_stage} -> {stage}')
        self.mission_stage = stage
        self.stage_started = time.time()
        self.last_command_time = 0.0

    def every(self, seconds):
        """True at most once per `seconds` within a stage - used to
        re-send commands the vehicle may have ignored, without
        flooding the link at the 10Hz loop rate."""
        now = time.time()
        if now - self.last_command_time >= seconds:
            self.last_command_time = now
            return True
        return False

    def run_mission(self):
        if not self.mission_running:
            return
        stage = self.mission_stage
        elapsed = time.time() - self.stage_started

        if (stage in PILOT_WATCH_STAGES and self.expected_mode and self.mode != self.expected_mode
                and time.time() - self.mode_set_at > MODE_GRACE_S):
            self.event('critical', f'PILOT OVERRIDE: flight mode changed to {self.mode} during {stage}. '
                                   'The bridge has stopped sending commands - fly manually.')
            self.set_stage('PILOT')
            self.mission_running = False
            return

        if stage == 'STARTING':
            self.set_guided_mode()
            self.set_stage('ARMING')

        elif stage == 'ARMING':
            if self.armed:
                self.get_logger().info('Vehicle armed!')
                self.set_stage('TAKEOFF')
            elif elapsed > ARM_TIMEOUT_S:
                self.event('critical', f'Could not arm within {ARM_TIMEOUT_S:.0f} s (see the Pixhawk messages above). '
                                       'Mission cancelled.')
                self.set_stage('ABORTED')
                self.mission_running = False
            elif self.every(1.0):
                self.arm_vehicle()

        elif stage == 'TAKEOFF':
            self.get_logger().info(f'Taking off to {self.search_altitude} m...')
            self.takeoff(self.search_altitude)
            self.set_stage('CLIMBING')

        elif stage == 'CLIMBING':
            if not self.armed:
                # ArduCopter auto-disarms if the takeoff doesn't happen
                # within a few seconds of arming - start over.
                self.get_logger().warn('Disarmed before climbing, retrying.')
                self.set_stage('STARTING')
            elif self.altitude >= self.search_altitude * 0.95:
                if self.config.get('mission_type') == 'hop':
                    self.event('info', f'Hop test: reached {self.altitude:.1f} m, hovering for '
                                       f"{self.config['hop_hold_s']:.0f} s.")
                    self.set_stage('HOVERING')
                else:
                    self.get_logger().info('Search altitude reached.')
                    self.begin_search()
            elif self.altitude < 0.5 and elapsed > 4.0 and self.every(3.0):
                self.get_logger().warn('Not climbing, re-sending takeoff.')
                self.takeoff(self.search_altitude)

        elif stage in FLIGHT_STAGES and not self.armed and elapsed > 3.0:
            # Disarmed in the air or on impact (ArduPilot's crash check):
            # stop, say so, don't keep "searching" with a dead drone.
            self.event('critical', f'DRONE DISARMED DURING {stage} at altitude {self.altitude:.1f} m '
                                   f'(crash or failsafe). Mission aborted.')
            self.set_stage('ABORTED')
            self.mission_running = False

        elif stage == 'HOVERING':
            # Hop test: GUIDED holds position by itself after the takeoff;
            # nothing is sent, so the drone cannot move sideways.
            if elapsed >= self.config['hop_hold_s']:
                self.event('info', 'Hop test: landing straight down (LAND mode).')
                self.set_land_mode()
                self.set_stage('LANDING')

        elif stage == 'SEARCHING':
            if self.config.get('planner') == 'grid':
                self.run_search()
            else:
                self.run_bayes_search()

        elif stage == 'VERIFYING':
            self.run_verify(elapsed)

        elif stage == 'MARKING':
            self.run_marking()

        elif stage == 'APPROACH':
            self.guarded_target(self.landing_lat, self.landing_lon)
            self.hold_search_speed()
            if distance_m(self.latitude, self.longitude, self.landing_lat, self.landing_lon) < ARRIVAL_RADIUS_M:
                self.get_logger().info('Beside the casualty. Landing.')
                self.set_land_mode()
                self.set_stage('LANDING')

        elif stage == 'RETURNING':
            self.guarded_target(self.home_lat, self.home_lon)
            self.hold_search_speed()
            if distance_m(self.latitude, self.longitude, self.home_lat, self.home_lon) < HOME_REACHED_M \
                    and not self.guard_blocked:
                self.get_logger().info('Over the launch point. Landing.')
                self.set_land_mode()
                self.set_stage('LANDING')

        elif stage == 'LANDING':
            if self.altitude <= 0.5 and not self.armed:
                if self.config.get('mission_type') == 'hop':
                    self.event('info', 'Hop test complete: landed and disarmed.')
                else:
                    self.event('info', f'Landed. Mission complete: {len(self.victims)} casualt'
                               f"{'y' if len(self.victims) == 1 else 'ies'} located.")
                self.set_stage('COMPLETE')
                self.mission_running = False

    def begin_search(self):
        self.search_t0 = time.time()
        self.last_speed_time = 0.0
        if self.config.get('planner') != 'grid':
            self.plan_target = None
            self.get_logger().info(
                f'Probability-map search: {self.pmap.nx}x{self.pmap.ny} cells, '
                f"stop at {self.config['target_pos']:.0%} probability of success"
            )
            self.set_stage('SEARCHING')
            return
        focal = (CAMERA_WIDTH / 2.0) / math.tan(CAMERA_HFOV_RAD / 2.0)
        # Narrowest ground footprint (image height), so lane spacing
        # is safe whichever way the drone is yawed.
        footprint = self.search_altitude * CAMERA_HEIGHT / focal
        spacing = footprint * (1.0 - LANE_OVERLAP)
        self.waypoints = build_lawnmower(
            self.search_center_lat, self.search_center_lon,
            self.search_width, self.search_height, spacing,
            self.latitude, self.longitude,
        )
        self.waypoint_index = 0
        self.get_logger().info(
            f'Grid search: {len(self.waypoints) // 2} lanes, {spacing:.1f} m apart, '
            f'{len(self.waypoints)} waypoints'
        )
        self.set_stage('SEARCHING')

    def finish_search(self, reason):
        n = len(self.victims)
        self.event(
            'info',
            f'Search finished ({reason}): {n} casualt{"y" if n == 1 else "ies"} located, '
            f'probability of success {self.pmap.pos:.0%}. Returning to launch.',
        )
        self.plan_target = None
        self.last_speed_time = 0.0
        self.set_stage('RETURNING')

    def check_for_candidate(self):
        """Switch to VERIFYING if the detector has a new candidate. Returns True if it did."""
        offset = self.fresh_offset()
        if offset is None:
            return False
        spot = self.project_offset(offset)
        self.last_offset = None
        if spot is None:
            return False
        if any(distance_m(spot[0], spot[1], r[0], r[1]) < REJECTED_RADIUS_M for r in self.rejected):
            self.get_logger().info('  -> ignored: within a previously rejected candidate area')
            return False
        for v in self.victims:
            if distance_m(spot[0], spot[1], v['lat'], v['lon']) < VICTIM_RADIUS_M:
                self.get_logger().info(f"  -> ignored: casualty {v['id']}, already located")
                return False
        self.get_logger().info(
            f'Candidate person (conf {offset[3]:.2f}) near {spot[0]:.7f}, {spot[1]:.7f}. '
            'Stopping to verify.'
        )
        self.candidate = list(spot)
        self.settled_at = None
        self.verify_samples = []
        self.verify_frames = 0
        self.verify_hits = 0
        self.verify_best_conf = offset[3]
        self.set_stage('VERIFYING')
        return True

    def run_search(self):
        if self.waypoint_index >= len(self.waypoints):
            self.finish_search('grid fully covered')
            return
        if self.check_for_candidate():
            return

        lat, lon = self.waypoints[self.waypoint_index]
        self.guarded_target(lat, lon)
        self.hold_search_speed()
        if distance_m(self.latitude, self.longitude, lat, lon) < WAYPOINT_RADIUS_M:
            self.waypoint_index += 1

    def run_bayes_search(self):
        """Fly to wherever the most casualty probability can be searched
        away per second; the map drains as detector frames come in."""
        if self.pmap.pos >= self.config['target_pos']:
            self.finish_search(f"reached {self.config['target_pos']:.0%} probability of success")
            return
        if time.time() - self.search_t0 > self.config['max_search_s']:
            self.finish_search('time limit')
            return
        if self.check_for_candidate():
            return

        now = time.time()
        e, n = self.to_local(self.latitude, self.longitude)
        reached = self.plan_target is not None and \
            math.hypot(e - self.plan_target[0], n - self.plan_target[1]) < TARGET_REACHED_M
        if self.plan_target is None or reached or now - self.last_plan_time >= REPLAN_S:
            self.last_plan_time = now
            best, score = self.pmap.best_target(e, n, self.search_speed)
            if self.plan_target is None or reached:
                self.plan_target = best
            else:
                current = self.pmap.score_at(self.plan_target, e, n, self.search_speed)
                if score > current * REPLAN_HYSTERESIS:
                    self.plan_target = best
        # Livelock guard: a target the drone can't actually search - e.g.
        # beside a tall building it has to overfly too high for its looks
        # to count - never drains, so the planner would pick it forever
        # (seen: 10 min circling the region's office block). After
        # TARGET_STUCK_S on one target, write most of its probability off
        # and move on.
        near = math.hypot(e - self.plan_target[0], n - self.plan_target[1]) < STUCK_NEAR_M
        if self.plan_target != self.stuck_target or not near:
            # Only time spent AT the target counts: getting to a far one
            # can take longer than TARGET_STUCK_S.
            self.stuck_target, self.stuck_since = self.plan_target, now
        elif now - self.stuck_since > TARGET_STUCK_S:
            te, tn = self.plan_target
            self.pmap.remove_disk(te, tn, STUCK_WRITE_OFF_M, keep=STUCK_KEEP)
            self.event('info', f'Skipping an area it cannot see well (E{te:+.0f} N{tn:+.0f}): '
                               f'no progress for {TARGET_STUCK_S:.0f} s.')
            self.plan_target = None
            self.stuck_target = None
            return
        lat, lon = self.to_global(*self.carrot(e, n, self.plan_target))
        self.guarded_target(lat, lon)

    def carrot(self, e, n, target):
        """A point CARROT_S of flight beyond the planner's target, on the
        same line. ArduPilot brakes to a stop at every position target, and
        the planner's targets are only metres apart: commanded at 5 m/s the
        drone averaged 1.47 m/s over a region mission. Aiming past the
        target lets it fly through at speed; the planner picks the next
        target as it arrives (TARGET_REACHED_M) and the carrot moves on."""
        de, dn = target[0] - e, target[1] - n
        dist = math.hypot(de, dn)
        reach = CARROT_S * self.search_speed
        if self.caution or dist < 1e-3 or dist >= reach:
            return target
        return e + de / dist * reach, n + dn / dist * reach
        self.hold_search_speed()

    def run_verify(self, elapsed):
        self.guarded_target(self.candidate[0], self.candidate[1])
        self.hold_search_speed()

        if self.settled_at is None:
            if distance_m(self.latitude, self.longitude, *self.candidate) < ARRIVAL_RADIUS_M:
                self.settled_at = time.time()
        else:
            # Only frames captured after arriving overhead count as
            # verification - the hover gives a steady, centered view.
            offset = self.fresh_offset(not_before=self.settled_at)
            if offset is not None:
                spot = self.project_offset(offset)
                self.last_offset = None
                if spot is not None:
                    self.verify_samples.append(spot)
                    self.verify_best_conf = max(self.verify_best_conf, offset[3])
                    # Follow the latest sightings, not all of them: a walking
                    # or crawling casualty would drag an all-time average behind.
                    recent = self.verify_samples[-VERIFY_TRACK_N:]
                    n = len(recent)
                    self.candidate = [sum(s[0] for s in recent) / n, sum(s[1] for s in recent) / n]

            majority = (
                self.verify_frames >= VERIFY_CONFIRM_MIN_FRAMES
                and self.verify_hits >= VERIFY_CONFIRM_RATIO * self.verify_frames
            )
            if len(self.verify_samples) >= VERIFY_SAMPLES and majority:
                self.confirm_victim()
                return

        fast_reject = (
            self.settled_at is not None
            and time.time() - self.settled_at >= VERIFY_FAST_MIN_S
            and self.verify_frames >= VERIFY_FAST_FRAMES
            and self.verify_hits < VERIFY_FAST_HIT_RATIO * self.verify_frames
        )
        if elapsed > VERIFY_TIMEOUT_S or fast_reject:
            enough_looks = self.verify_frames >= VERIFY_MIN_FRAMES
            mostly_empty = self.verify_hits < VERIFY_CONFIRM_RATIO * self.verify_frames
            if enough_looks and mostly_empty:
                self.event(
                    'info',
                    f'Candidate rejected: person seen in only {self.verify_hits}/{self.verify_frames} '
                    'frames while hovering over it. Resuming search.',
                )
                self.rejected.append(tuple(self.candidate))
            elif elapsed < VERIFY_MAX_S:
                if self.every(10.0):
                    self.get_logger().info(
                        f'Still verifying: person in {self.verify_hits}/{self.verify_frames} '
                        'analysed frames so far, waiting for more.'
                    )
                return
            else:
                self.get_logger().warn('Verification ran out of time; resuming search (not blacklisted).')
            self.candidate = None
            self.plan_target = None
            self.set_stage('SEARCHING')

    def confirm_victim(self):
        person_lat, person_lon = self.candidate
        # The same person again? A first sighting can project several metres
        # off (e.g. from high over city blocks) and slip past the check in
        # check_for_candidate, then verification converges onto someone
        # already found (seen: one casualty confirmed three times).
        for v in self.victims:
            if distance_m(person_lat, person_lon, v['lat'], v['lon']) < VICTIM_RADIUS_M:
                self.get_logger().info(f"Verified candidate is {v['id']} again (already located); resuming search.")
                self.candidate = None
                self.plan_target = None
                self.set_stage('SEARCHING')
                return
        e, n = self.to_local(person_lat, person_lon)
        k = VERIFY_TRACK_N
        head, tail = self.verify_samples[:k], self.verify_samples[-k:]
        first = (sum(p[0] for p in head) / len(head), sum(p[1] for p in head) / len(head))
        last = (sum(p[0] for p in tail) / len(tail), sum(p[1] for p in tail) / len(tail))
        moving = len(self.verify_samples) >= 2 * k and distance_m(first[0], first[1], last[0], last[1]) > MOVING_SPREAD_M
        victim = {
            'id': f'V{len(self.victims) + 1}',
            'moving': moving,
            'lat': person_lat, 'lon': person_lon, 'e': round(e, 2), 'n': round(n, 2),
            'conf': round(self.verify_best_conf, 2),
            'sensor': 'thermal' if self.sensor_mode == 'night' else 'rgb',
            'evidence': f'{self.verify_hits}/{self.verify_frames} frames',
            'found_s': round(self.mission_time(), 1),
            'aid_dropped': False,
        }
        self.victims.append(victim)
        self.routes_dirty = True
        self.pmap.remove_disk(e, n, VICTIM_RADIUS_M)
        self.event(
            'critical',
            f"CASUALTY {victim['id']} CONFIRMED at E{e:+.1f} N{n:+.1f} m "
            f"({person_lat:.6f}, {person_lon:.6f}), {victim['sensor'].upper()} conf {victim['conf']:.2f}, "
            f"{'MOVING (responsive)' if moving else 'not moving'}, "
            f"{self.mission_time():.0f} s into the mission",
        )

        # Stand-off point on the side facing the launch point, for the
        # landing or the aid-kit drop.
        ref_lat = self.home_lat if self.home_lat is not None else self.latitude
        ref_lon = self.home_lon if self.home_lon is not None else self.longitude
        north, east = north_east_between(person_lat, person_lon, ref_lat, ref_lon)
        dist = math.hypot(north, east)
        if dist < 0.1:
            north, east, dist = 1.0, 0.0, 1.0
        standoff = offset_latlon(
            person_lat, person_lon,
            LANDING_STANDOFF_M * north / dist, LANDING_STANDOFF_M * east / dist,
        )
        self.candidate = None
        if self.config.get('on_confirm') == 'land':
            self.landing_lat, self.landing_lon = standoff
            self.set_stage('APPROACH')
        else:
            self.mark_point = standoff
            self.marked_at = None
            self.set_stage('MARKING')

    def run_marking(self):
        """Fly to the stand-off point, drop the aid kit (once - there is
        one on board), hold briefly, then resume the search."""
        self.guarded_target(self.mark_point[0], self.mark_point[1])
        self.hold_search_speed()
        if self.marked_at is None:
            if distance_m(self.latitude, self.longitude, *self.mark_point) < ARRIVAL_RADIUS_M:
                self.marked_at = time.time()
                victim = self.victims[-1]
                if not self.payload_dropped:
                    self.drop_payload()
                    victim['aid_dropped'] = True
                    self.event('info', f"Aid kit dropped {LANDING_STANDOFF_M:.1f} m from {victim['id']}.")
                else:
                    self.event('info', f"{victim['id']} marked (aid kit already used).")
        elif time.time() - self.marked_at >= MARK_HOLD_S:
            self.mark_point = None
            self.plan_target = None
            self.event('info', 'Resuming search for further casualties.')
            self.set_stage('SEARCHING')

    def payload_service(self, request, response):
        self.drop_payload()
        response.success = self.payload_dropped
        response.message = 'Payload released.' if self.payload_dropped else 'Payload release failed.'
        return response

    def drop_payload(self):
        # Real drone: RESQ_PAYLOAD_SERVO="<servo output>:<release PWM>"
        # (e.g. "9:1900", AUX1 on a Pixhawk 2.4.8) moves the release servo.
        servo = os.environ.get('RESQ_PAYLOAD_SERVO')
        if servo:
            channel, pwm = (int(v) for v in servo.split(':'))
            self.vehicle.mav.command_long_send(
                self.vehicle.target_system, self.vehicle.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0, channel, pwm, 0, 0, 0, 0, 0)
            self.payload_dropped = True
            self.get_logger().info(f'Payload servo {channel} -> {pwm} us')
            return
        # Simulator: the same Gazebo topic the command center's Drop Payload button uses.
        try:
            subprocess.Popen(
                ['gz', 'topic', '-t', PAYLOAD_DETACH_TOPIC, '-m', 'gz.msgs.Empty', '-p', ''],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            )
            self.payload_dropped = True
        except OSError as error:
            self.get_logger().error(f'Payload release failed: {error}')

    # ---------------------------------------------------------------
    # Situational map for the command center
    # ---------------------------------------------------------------

    def route_bounds(self):
        pts = [self.staging] + [(v['e'], v['n']) for v in self.victims]
        pm = self.pmap
        e_min = min([pm.e0] + [p[0] for p in pts]) - 8.0
        n_min = min([pm.n0] + [p[1] for p in pts]) - 8.0
        e_max = max([pm.e0 + pm.width] + [p[0] for p in pts]) + 8.0
        n_max = max([pm.n0 + pm.height] + [p[1] for p in pts]) + 8.0
        return e_min, n_min, e_max, n_max

    def publish_map(self):
        if self.origin is None:
            return
        now = time.time()
        if self.victims and self.pmap is not None and (
                self.routes_dirty or now - self.last_route_time >= ROUTE_REFRESH_S * 5):
            assess_victims(self.victims, self.hazards.confirmed(), self.staging, self.route_bounds())
            self.routes_dirty = False
            self.last_route_time = now
        e, n = self.to_local(self.latitude, self.longitude)
        data = {
            'mission_id': self.mission_id,
            'origin': list(self.origin),
            't': round(self.mission_time(), 1),
            'stage': self.mission_stage,
            'sensor': self.sensor_mode,
            'config': self.config,
            'drone': {
                'e': round(e, 2), 'n': round(n, 2), 'alt': round(self.altitude, 2),
                'yaw': round(self.yaw, 3), 'armed': self.armed,
                'speed': round(self.ground_speed, 2), 'sats': self.satellites, 'gps_fix': self.gps_fix,
                'battery': self.battery_percentage, 'mode': self.mode,
            },
            'footprint': [
                [round(c[0], 2), round(c[1], 2)] for c in footprint_corners(
                    e, n, max(self.altitude, 0.5), self.yaw, CAMERA_HFOV_RAD, CAMERA_WIDTH, CAMERA_HEIGHT)
            ] if self.altitude > 1.0 else None,
            'trail': self.trail,
            'staging': list(self.staging),
            'detector_live': now - self.last_detector_frame < 5.0,
            'payload_dropped': self.payload_dropped,
            'victims': self.victims,
            'hazards': self.hazards.confirmed(),
            'events': list(self.events),
            'obstacle': dict(self.guard.to_dict(self.flight_altitude(), self.search_altitude),
                             lidar_live=self.lidar_live()),
            'events_total': self.events_total,
            'basemap': self.basemap,
        }
        if self.pmap is not None:
            data['prob'] = self.pmap.to_dict()
            data['pos'] = round(self.pmap.pos, 4)
        if self.plan_target is not None and self.mission_stage == 'SEARCHING':
            data['target'] = [round(self.plan_target[0], 2), round(self.plan_target[1], 2)]
        if self.config.get('planner') == 'grid' and self.waypoints:
            data['waypoints'] = [
                [round(c, 2) for c in self.to_local(*wp)] for wp in self.waypoints[self.waypoint_index:]
            ]
        if self.candidate is not None:
            data['candidate'] = [round(c, 2) for c in self.to_local(*self.candidate)]
        data['rejected'] = [[round(c, 2) for c in self.to_local(*r)] for r in self.rejected]
        self.map_publisher.publish(String(data=json.dumps(data)))

    # ---------------------------------------------------------------
    # Vehicle commands
    # ---------------------------------------------------------------

    def request_message_rate(self, message_id, hz):
        self.vehicle.mav.command_long_send(
            self.vehicle.target_system, self.vehicle.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            message_id, int(1e6 / hz), 0, 0, 0, 0, 0,
        )

    def set_mode(self, name):
        mode_id = self.vehicle.mode_mapping().get(name)
        if mode_id is None:
            self.get_logger().error(f'{name} mode is not available.')
            return
        self.expected_mode = name
        self.mode_set_at = time.time()
        self.vehicle.mav.set_mode_send(
            self.vehicle.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id,
        )

    def set_guided_mode(self):
        self.set_mode('GUIDED')

    def set_land_mode(self):
        self.set_mode('LAND')

    def set_rtl_mode(self):
        self.set_mode('RTL')

    def arm_vehicle(self):
        self.vehicle.mav.command_long_send(
            self.vehicle.target_system, self.vehicle.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            1, 0, 0, 0, 0, 0, 0,
        )

    def takeoff(self, altitude):
        self.vehicle.mav.command_long_send(
            self.vehicle.target_system, self.vehicle.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
            0, 0, 0, 0, 0, 0, altitude,
        )

    def set_speed(self, speed_ms):
        self.vehicle.mav.command_long_send(
            self.vehicle.target_system, self.vehicle.target_component,
            mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, 0,
            1, speed_ms, -1, 0, 0, 0, 0,
        )

    def send_gps_target(self, target_lat, target_lon, altitude):
        self.vehicle.mav.set_position_target_global_int_send(
            0,
            self.vehicle.target_system,
            self.vehicle.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            0b0000111111111000,
            int(target_lat * 1e7),
            int(target_lon * 1e7),
            altitude,
            0, 0, 0,
            0, 0, 0,
            0, 0,
        )


def main(args=None):
    rclpy.init(args=args)
    node = MavlinkBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
