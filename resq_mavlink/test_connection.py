from pymavlink import mavutil

print("Connecting to ArduPilot SITL...")

connection = mavutil.mavlink_connection(
    "udp:127.0.0.1:14550"
)

print("Waiting for heartbeat...")

connection.wait_heartbeat()

print("Connected!")
print(f"System ID: {connection.target_system}")
print(f"Component ID: {connection.target_component}")

print("Getting vehicle position...")

message = connection.recv_match(
    type="GLOBAL_POSITION_INT",
    blocking=True
)

latitude = message.lat / 1e7
longitude = message.lon / 1e7
altitude = message.relative_alt / 1000.0

print(f"Latitude:  {latitude}")
print(f"Longitude: {longitude}")
print(f"Altitude:  {altitude} m")
