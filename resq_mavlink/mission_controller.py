import json
import math
import time

from pymavlink import mavutil


CONNECTION = "udp:127.0.0.1:14550"
MISSION_FILE = "mission.json"


def load_mission():
    print("Loading mission...")

    with open(MISSION_FILE, "r") as file:
        mission = json.load(file)

    target = mission["target"]

    print(f"Mission ID: {mission['mission_id']}")
    print(f"Target latitude:  {target['latitude']}")
    print(f"Target longitude: {target['longitude']}")
    print(f"Target altitude:  {target['altitude']} m")

    return mission


def connect_vehicle():
    print("\nConnecting to ArduPilot SITL...")

    vehicle = mavutil.mavlink_connection(CONNECTION)

    print("Waiting for heartbeat...")
    vehicle.wait_heartbeat()

    print("Connected!")
    print(f"System ID: {vehicle.target_system}")
    print(f"Component ID: {vehicle.target_component}")

    return vehicle


def set_mode(vehicle, mode):
    print(f"\nSetting {mode} mode...")

    mode_id = vehicle.mode_mapping().get(mode)

    if mode_id is None:
        raise RuntimeError(f"{mode} mode is not available")

    vehicle.mav.set_mode_send(
        vehicle.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id,
    )

    time.sleep(2)

    print(f"{mode} mode requested")


def arm(vehicle):
    print("\nArming vehicle...")

    vehicle.mav.command_long_send(
        vehicle.target_system,
        vehicle.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1,
        0,
        0,
        0,
        0,
        0,
        0,
    )

    vehicle.motors_armed_wait()

    print("Vehicle armed!")


def takeoff(vehicle, altitude):
    print(f"\nTaking off to {altitude} m...")

    vehicle.mav.command_long_send(
        vehicle.target_system,
        vehicle.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        altitude,
    )

    while True:
        message = vehicle.recv_match(
            type="GLOBAL_POSITION_INT",
            blocking=True,
            timeout=2,
        )

        if message is None:
            continue

        current_altitude = message.relative_alt / 1000.0

        print(f"Altitude: {current_altitude:.2f} m")

        if current_altitude >= altitude * 0.90:
            print("Target altitude reached!")
            break

        time.sleep(1)


def send_position_target(vehicle, latitude, longitude, altitude):
    print("\nSending GPS target...")

    vehicle.mav.set_position_target_global_int_send(
        0,
        vehicle.target_system,
        vehicle.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        0b110111111000,
        int(latitude * 1e7),
        int(longitude * 1e7),
        altitude,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
    )

    print("GPS target sent!")


def calculate_distance(lat1, lon1, lat2, lon2):
    earth_radius = 6371000

    lat1 = math.radians(lat1)
    lon1 = math.radians(lon1)
    lat2 = math.radians(lat2)
    lon2 = math.radians(lon2)

    dlat = lat2 - lat1
    dlon = lon2 - lon1

    a = (
        math.sin(dlat / 2) ** 2
        + math.cos(lat1)
        * math.cos(lat2)
        * math.sin(dlon / 2) ** 2
    )

    return earth_radius * 2 * math.atan2(
        math.sqrt(a),
        math.sqrt(1 - a),
    )


def fly_to_target(vehicle, target):
    target_lat = target["latitude"]
    target_lon = target["longitude"]
    target_alt = target["altitude"]

    print("\nFlying to target...")

    while True:
        send_position_target(
            vehicle,
            target_lat,
            target_lon,
            target_alt,
        )

        message = vehicle.recv_match(
            type="GLOBAL_POSITION_INT",
            blocking=True,
            timeout=2,
        )

        if message is None:
            continue

        current_lat = message.lat / 1e7
        current_lon = message.lon / 1e7
        current_alt = message.relative_alt / 1000.0

        distance = calculate_distance(
            current_lat,
            current_lon,
            target_lat,
            target_lon,
        )

        print(
            f"Position: {current_lat:.7f}, {current_lon:.7f} | "
            f"Altitude: {current_alt:.2f} m | "
            f"Distance: {distance:.2f} m"
        )

        if distance < 2.0:
            print("\nTarget reached!")
            break

        time.sleep(1)


def land(vehicle):
    print("\nLanding...")

    set_mode(vehicle, "LAND")

    print("LAND mode requested")


def main():
    mission = load_mission()

    vehicle = connect_vehicle()

    set_mode(vehicle, "GUIDED")

    arm(vehicle)

    takeoff(
        vehicle,
        mission["target"]["altitude"],
    )

    fly_to_target(
        vehicle,
        mission["target"],
    )

    print("\nHolding position for 5 seconds...")
    time.sleep(5)

    land(vehicle)

    print("\nRESQ-MESH mission complete!")


if __name__ == "__main__":
    main()
