#!/usr/bin/env python3
"""
Multi-leg mission for the SIH SAR drone project.
Sequence: home -> waypoint 1 (land) -> takeoff -> waypoint 2 (land) ->
takeoff -> home (land).
Uses ArduPilot's native AP_DDS interface (ardupilot_msgs), against the
versioned /ap/v1/ namespace confirmed live on this machine.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from ardupilot_msgs.srv import ArmMotors, ModeSwitch, Takeoff
from std_srvs.srv import Trigger
from ardupilot_msgs.msg import GlobalPosition
from geographic_msgs.msg import GeoPoseStamped

GUIDED_MODE = 4    # confirmed: ArduCopter GUIDED = 4
LAND_MODE = 9      # ArduCopter LAND = 9 (stable, long-standing mode number)

CRUISE_ALT_M = 20.0        # meters above home, for each leg
ARRIVAL_RADIUS_M = 3.0     # how close counts as "reached" laterally
LANDED_THRESHOLD_M = 0.5   # how close to home altitude counts as "landed"

EARTH_RADIUS_M = 6371000.0

IGNORE_VX = 8
IGNORE_VY = 16
IGNORE_VZ = 32
IGNORE_AFX = 64
IGNORE_AFY = 128
IGNORE_AFZ = 256
IGNORE_YAW = 1024
IGNORE_YAW_RATE = 2048
POSITION_ONLY_MASK = (
    IGNORE_VX | IGNORE_VY | IGNORE_VZ |
    IGNORE_AFX | IGNORE_AFY | IGNORE_AFZ |
    IGNORE_YAW | IGNORE_YAW_RATE
)

# Each leg: (name, north_offset_m, east_offset_m), measured from HOME,
# not from the previous leg. Same numbers are reused by the marker
# script below so the labels line up with where the drone actually goes.
LEGS = [
    ("Waypoint 1", 150.0, 50.0),
    ("Waypoint 2", 150.0, 300.0),
    ("Home", 0.0, 0.0),
]


def offset_latlon(lat, lon, north_m, east_m):
    d_lat = (north_m / EARTH_RADIUS_M) * (180.0 / math.pi)
    d_lon = (east_m / (EARTH_RADIUS_M * math.cos(math.radians(lat)))) * (180.0 / math.pi)
    return lat + d_lat, lon + d_lon


def flat_earth_distance_m(lat1, lon1, lat2, lon2):
    d_lat = (lat2 - lat1) * 111320.0
    d_lon = (lon2 - lon1) * 111320.0 * math.cos(math.radians(lat1))
    return math.sqrt(d_lat ** 2 + d_lon ** 2)


class WaypointMission(Node):
    def __init__(self):
        super().__init__('waypoint_mission')

        self.arm_client = self.create_client(ArmMotors, '/ap/v1/arm_motors')
        self.mode_client = self.create_client(ModeSwitch, '/ap/v1/mode_switch')
        self.takeoff_client = self.create_client(Takeoff, '/ap/v1/experimental/takeoff')
        self.prearm_client = self.create_client(Trigger, '/ap/v1/prearm_check')

        self.gps_pub = self.create_publisher(GlobalPosition, '/ap/v1/cmd_gps_pose', 10)

        self.current_lat = None
        self.current_lon = None
        self.current_alt = None
        self.home_lat = None
        self.home_lon = None
        self.home_alt = None

        self.create_subscription(
            GeoPoseStamped, '/ap/v1/geopose/filtered', self._geopose_cb,
            qos_profile_sensor_data
        )

    def _geopose_cb(self, msg):
        self.current_lat = msg.pose.position.latitude
        self.current_lon = msg.pose.position.longitude
        self.current_alt = msg.pose.position.altitude
        if self.home_lat is None:
            self.home_lat = self.current_lat
            self.home_lon = self.current_lon
            self.home_alt = self.current_alt
            self.get_logger().info(
                f'Home recorded: {self.home_lat:.6f}, {self.home_lon:.6f}, '
                f'{self.home_alt:.1f} m AMSL'
            )

    def wait_for_services(self):
        for client, name in [
            (self.arm_client, 'arm_motors'),
            (self.mode_client, 'mode_switch'),
            (self.takeoff_client, 'takeoff'),
        ]:
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'Waiting for {name} service...')

    def wait_for_home(self):
        self.get_logger().info('Waiting for first position fix...')
        while self.home_lat is None:
            rclpy.spin_once(self, timeout_sec=0.5)

    def wait_for_prearm(self, timeout_sec=30.0):
        self.get_logger().info('Waiting for pre-arm checks to pass...')
        deadline = self.get_clock().now().nanoseconds + int(timeout_sec * 1e9)
        req = Trigger.Request()
        while self.get_clock().now().nanoseconds < deadline:
            future = self.prearm_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            result = future.result()
            if result is not None and result.success:
                self.get_logger().info('Pre-arm checks passed.')
                return True
            msg = result.message if result is not None else 'no response'
            self.get_logger().info(f'Pre-arm not ready yet: {msg}')
            rclpy.spin_once(self, timeout_sec=1.0)
        self.get_logger().error('Pre-arm checks did not pass within timeout.')
        return False

    def call_mode_switch(self, mode):
        req = ModeSwitch.Request()
        req.mode = mode
        future = self.mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        self.get_logger().info(f'mode_switch({mode}) -> status={result.status}')
        return result.status

    def call_arm(self, arm):
        req = ArmMotors.Request()
        req.arm = arm
        future = self.arm_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        self.get_logger().info(f'arm_motors({arm}) -> result={result.result}')
        return result.result

    def call_takeoff(self, alt):
        req = Takeoff.Request()
        req.alt = alt
        future = self.takeoff_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        self.get_logger().info(f'takeoff({alt}) -> status={result.status}')
        return result.status

    def wait_until_altitude(self, target_alt, tolerance=1.0):
        self.get_logger().info(f'Waiting to reach {target_alt} m above home...')
        while True:
            rclpy.spin_once(self, timeout_sec=0.5)
            if self.current_alt is None:
                continue
            rel_alt = self.current_alt - self.home_alt
            if rel_alt >= target_alt - tolerance:
                self.get_logger().info(f'Reached altitude: {rel_alt:.1f} m above home')
                return

    def wait_until_landed(self):
        self.get_logger().info('Waiting to touch down...')
        while True:
            rclpy.spin_once(self, timeout_sec=0.5)
            if self.current_alt is None:
                continue
            rel_alt = self.current_alt - self.home_alt
            if rel_alt <= LANDED_THRESHOLD_M:
                self.get_logger().info(f'Landed: {rel_alt:.1f} m above home')
                return

    def publish_waypoint(self, lat, lon, alt_above_home):
        msg = GlobalPosition()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.coordinate_frame = GlobalPosition.FRAME_GLOBAL_REL_ALT
        msg.type_mask = POSITION_ONLY_MASK
        msg.latitude = lat
        msg.longitude = lon
        msg.altitude = alt_above_home
        self.gps_pub.publish(msg)
        self.get_logger().info(f'Published waypoint: {lat:.6f}, {lon:.6f}, {alt_above_home} m')

    def wait_until_arrived(self, target_lat, target_lon):
        self.get_logger().info('Waiting to reach waypoint...')
        while True:
            rclpy.spin_once(self, timeout_sec=0.5)
            if self.current_lat is None:
                continue
            dist = flat_earth_distance_m(
                self.current_lat, self.current_lon, target_lat, target_lon
            )
            if dist <= ARRIVAL_RADIUS_M:
                self.get_logger().info(f'Arrived, distance={dist:.1f} m')
                return

    def takeoff_leg(self):
        if not self.call_mode_switch(GUIDED_MODE):
            self.get_logger().error('Failed to switch to GUIDED. Aborting.')
            return False
        if not self.call_arm(True):
            self.get_logger().error('Failed to arm. Aborting.')
            return False
        if not self.call_takeoff(CRUISE_ALT_M):
            self.get_logger().error('Takeoff request failed. Aborting.')
            return False
        self.wait_until_altitude(CRUISE_ALT_M)
        return True

    def land_leg(self):
        if not self.call_mode_switch(LAND_MODE):
            self.get_logger().error('Failed to switch to LAND. Aborting.')
            return False
        self.wait_until_landed()
        self.call_arm(False)
        return True

    def run_mission(self):
        self.wait_for_services()
        self.wait_for_home()
        if not self.wait_for_prearm():
            return

        for name, north_m, east_m in LEGS:
            self.get_logger().info(f'--- Leg: {name} ---')
            if not self.takeoff_leg():
                return
            target_lat, target_lon = offset_latlon(
                self.home_lat, self.home_lon, north_m, east_m
            )
            self.publish_waypoint(target_lat, target_lon, CRUISE_ALT_M)
            self.wait_until_arrived(target_lat, target_lon)
            if not self.land_leg():
                return

        self.get_logger().info('Mission complete. Landed at home, disarmed.')


def main():
    rclpy.init()
    node = WaypointMission()
    try:
        node.run_mission()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
