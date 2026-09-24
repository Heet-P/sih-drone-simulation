import json
import math
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

MISSION_FILE = '/home/heet/Desktop/SIH-Drone-Simulation/resq_mavlink/mission.json'
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

        self.position_publisher = self.create_publisher(String, '/resq/drone/position', 10)
        self.state_publisher = self.create_publisher(String, '/resq/drone/state', 10)

        self.create_service(SetBool, '/resq/mission/start', self.start_mission)
        self.create_service(Trigger, '/resq/mission/land', self.land_now)

        self.create_subscription(Bool, '/detections/person_confirmed', self._on_person_confirmed, 10)
        self.create_subscription(PointStamped, '/detections/person_offset', self._on_person_offset, 10)
        self.create_subscription(Detection2DArray, '/detections/persons', self._on_detection_frame, 10)
        self.create_subscription(
            String, '/resq/sensor_mode', self._on_sensor_mode,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )

        self.create_timer(0.1, self.update)
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
        self.run_mission()

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
            elif message_type == 'ATTITUDE':
                self.roll = message.roll
                self.pitch = message.pitch
                self.yaw = message.yaw
            elif message_type == 'HEARTBEAT':
                if message.get_srcComponent() != mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1:
                    continue
                self.armed = bool(message.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                self.mode = mavutil.mode_string_v10(message)
            elif message_type == 'BATTERY_STATUS':
                self.battery_percentage = message.battery_remaining
                voltage = message.voltages[0]
                if voltage != 65535:
                    self.battery_voltage = voltage / 1000.0

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

    def _on_detection_frame(self, msg):
        """One message per frame the detector analysed (empty = no person).
        Counted while hovering over a candidate, as evidence for/against."""
        if self.mission_stage == 'VERIFYING' and self.settled_at is not None:
            self.verify_frames += 1
            if msg.detections:
                self.verify_hits += 1

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

    def hold_search_speed(self):
        now = time.time()
        if now - self.last_speed_time >= SPEED_REASSERT_S:
            self.last_speed_time = now
            self.set_speed(SEARCH_SPEED_MS)

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
        self.mission_running = True
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

        if stage == 'STARTING':
            self.set_guided_mode()
            self.set_stage('ARMING')

        elif stage == 'ARMING':
            if self.armed:
                self.get_logger().info('Vehicle armed!')
                self.set_stage('TAKEOFF')
            elif self.every(1.0):
                if self.mode != 'GUIDED':
                    self.set_guided_mode()
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
                self.get_logger().info('Search altitude reached.')
                self.begin_search()
            elif self.altitude < 0.5 and elapsed > 4.0 and self.every(3.0):
                self.get_logger().warn('Not climbing, re-sending takeoff.')
                self.takeoff(self.search_altitude)

        elif stage == 'SEARCHING':
            self.run_search()

        elif stage == 'VERIFYING':
            self.run_verify(elapsed)

        elif stage == 'APPROACH':
            self.send_gps_target(self.landing_lat, self.landing_lon, self.search_altitude)
            self.hold_search_speed()
            if distance_m(self.latitude, self.longitude, self.landing_lat, self.landing_lon) < ARRIVAL_RADIUS_M:
                self.get_logger().info('Beside the casualty. Landing.')
                self.set_land_mode()
                self.set_stage('LANDING')

        elif stage in ('LANDING', 'RETURNING'):
            if self.altitude <= 0.5 and not self.armed:
                self.get_logger().info('Landed. Mission complete!')
                self.set_stage('COMPLETE')
                self.mission_running = False

    def begin_search(self):
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
        self.last_speed_time = 0.0
        self.set_stage('SEARCHING')

    def run_search(self):
        if self.waypoint_index >= len(self.waypoints):
            self.get_logger().warn('Search area covered, no person confirmed. Returning to launch.')
            self.set_rtl_mode()
            self.set_stage('RETURNING')
            return

        offset = self.fresh_offset()
        if offset is not None:
            spot = self.project_offset(offset)
            self.last_offset = None
            if spot is None:
                pass
            elif any(distance_m(spot[0], spot[1], r[0], r[1]) < REJECTED_RADIUS_M for r in self.rejected):
                self.get_logger().info('  -> ignored: within a previously rejected candidate area')
            else:
                self.get_logger().info(
                    f'Candidate person (conf {offset[3]:.2f}) near {spot[0]:.7f}, {spot[1]:.7f}. '
                    'Stopping to verify.'
                )
                self.candidate = list(spot)
                self.settled_at = None
                self.verify_samples = []
                self.verify_frames = 0
                self.verify_hits = 0
                self.set_stage('VERIFYING')
                return

        lat, lon = self.waypoints[self.waypoint_index]
        self.send_gps_target(lat, lon, self.search_altitude)
        self.hold_search_speed()
        if distance_m(self.latitude, self.longitude, lat, lon) < WAYPOINT_RADIUS_M:
            self.waypoint_index += 1

    def run_verify(self, elapsed):
        self.send_gps_target(self.candidate[0], self.candidate[1], self.search_altitude)
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
                    n = len(self.verify_samples)
                    self.candidate = [
                        sum(s[0] for s in self.verify_samples) / n,
                        sum(s[1] for s in self.verify_samples) / n,
                    ]

            if self.person_confirmed and len(self.verify_samples) >= VERIFY_SAMPLES:
                self.confirm_and_approach()
                return

        if elapsed > VERIFY_TIMEOUT_S:
            enough_looks = self.verify_frames >= VERIFY_MIN_FRAMES
            mostly_empty = self.verify_hits * 2 < self.verify_frames
            if enough_looks and mostly_empty:
                self.get_logger().warn(
                    f'Candidate rejected: person seen in {self.verify_hits}/{self.verify_frames} '
                    'frames while hovering over it. Resuming search.'
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
            self.set_stage('SEARCHING')

    def confirm_and_approach(self):
        person_lat, person_lon = self.candidate
        self.get_logger().info(f'PERSON CONFIRMED at {person_lat:.7f}, {person_lon:.7f}')
        ref_lat = self.home_lat if self.home_lat is not None else self.latitude
        ref_lon = self.home_lon if self.home_lon is not None else self.longitude
        north, east = north_east_between(person_lat, person_lon, ref_lat, ref_lon)
        dist = math.hypot(north, east)
        if dist < 0.1:
            north, east, dist = 1.0, 0.0, 1.0
        self.landing_lat, self.landing_lon = offset_latlon(
            person_lat, person_lon,
            LANDING_STANDOFF_M * north / dist, LANDING_STANDOFF_M * east / dist,
        )
        self.set_stage('APPROACH')

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
