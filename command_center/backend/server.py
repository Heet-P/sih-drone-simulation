#!/usr/bin/env python3
"""
SIH-Drone Command Center backend.

Runs real subprocesses for the Gazebo/ArduPilot sim, the MAVLink/ROS2
mission bridge, and the YOLO person detector; bridges ROS2 topics and
services to a small REST/SSE/MJPEG API for the command_center/frontend
web page.
"""
import json
import os
import re
import signal
import subprocess
import threading
import time
from collections import deque

import cv2
import rclpy
from cv_bridge import CvBridge
from flask import Flask, Response, request, send_from_directory
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger
from vision_msgs.msg import Detection2DArray

HOME = os.path.expanduser("~")
BACKEND_DIR = os.path.dirname(os.path.abspath(__file__))
FRONTEND_DIR = os.path.join(os.path.dirname(BACKEND_DIR), "frontend")
PROJECT_ROOT = os.path.dirname(os.path.dirname(BACKEND_DIR))  # SIH-Drone-Simulation
ROS_ENV = os.path.join(BACKEND_DIR, "ros_env.sh")

# Worlds in ardupilot_gz_gazebo/worlds the dashboard can launch. Both share
# the frame, casualty and lights that the bridge, detector and lighting
# below rely on; only the Gazebo world name differs.
WORLDS = {
    "disaster": "Post-disaster zone",
    "runway": "Runway (flat)",
}
DEFAULT_WORLD = "disaster"


def sim_launch_cmd(world):
    return [
        ROS_ENV, "ros2", "launch", "ardupilot_gz_bringup",
        "s500_quad_runway.launch.py", f"world:={world}", "rviz:=true", "use_gz_tf:=true",
    ]


current_world = DEFAULT_WORLD
BRIDGE_CMD = [ROS_ENV, "ros2", "run", "resq_mavlink_bridge", "mavlink_bridge"]
FEED_CMD = [ROS_ENV, "python3", os.path.join(PROJECT_ROOT, "ardu_ws", "person_detector.py")]

# Day/night lighting for the worlds' "sun" and "fill" lights (identical in
# runway.sdf and disaster.sdf), applied live through Gazebo's
# /world/<world>/light_config service. That service replaces the whole
# light, so every field (direction, attenuation, ...) is sent.
SUN_BASE = (
    'type: DIRECTIONAL pose {{ position {{ z: 10 }} }} direction {{ x: -0.5 y: 0.1 z: -0.9 }} '
    'range: 1000 attenuation_constant: 0.9 attenuation_linear: 0.01 attenuation_quadratic: 0.001 '
    'intensity: 1 '
)
FILL_BASE = 'type: DIRECTIONAL pose {{ position {{ z: 10 }} }} direction {{ z: -1 }} range: 1000 intensity: 1 '
LIGHTING = {
    # Matches the worlds' defaults.
    "day": [
        ("sun", SUN_BASE + 'diffuse {{ r: 0.7 g: 0.7 b: 0.7 a: 1 }} specular {{ r: 0.8 g: 0.8 b: 0.8 a: 1 }} cast_shadows: true'),
        ("fill", FILL_BASE + 'diffuse {{ r: 0.4 g: 0.4 b: 0.4 a: 1 }} specular {{ a: 1 }} cast_shadows: false'),
    ],
    # Sun becomes dim bluish moonlight, fill off.
    "night": [
        ("sun", SUN_BASE + 'diffuse {{ r: 0.05 g: 0.06 b: 0.09 a: 1 }} specular {{ a: 1 }} cast_shadows: false'),
        ("fill", FILL_BASE + 'diffuse {{ a: 1 }} specular {{ a: 1 }} cast_shadows: false'),
    ],
}
SENSOR_MODE_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def apply_lighting(mode):
    """Switch the sim's lights; returns (ok, message)."""
    for name, fields in LIGHTING[mode]:
        req = f'name: "{name}" ' + fields.format()
        try:
            result = subprocess.run(
                [ROS_ENV, "gz", "service", "-s", f"/world/{current_world}/light_config",
                 "--reqtype", "gz.msgs.Light",
                 "--reptype", "gz.msgs.Boolean", "--timeout", "3000", "--req", req],
                capture_output=True, text=True, timeout=10,
            )
        except subprocess.TimeoutExpired:
            return False, f"light service timed out ({name})"
        if result.returncode != 0 or "true" not in result.stdout:
            return False, f"light '{name}' not updated: {(result.stdout + result.stderr).strip()[:200]}"
    return True, f"lighting set to {mode}"

# Same processes as the pkill sequence run by hand between sim runs,
# but specific: a bare "gz" or "ardupilot" pattern (as in the manual
# version) kills ANY process whose command line merely contains that
# text, e.g. an editor with an ArduPilot source file open.
KILL_PATTERNS = [
    "ros2 launch ardupilot_gz_bringup", "gz sim", "arducopter", "rviz2",
    "robot_state_publisher", "parameter_bridge", "micro_ros_agent", "mavproxy.py",
    "resq_mavlink_bridge", "person_detector.py",
]

STATE_RE = re.compile(
    r"Armed:\s*(?P<armed>\w+),\s*Mode:\s*(?P<mode>\S+),\s*"
    r"Battery:\s*(?P<battery_pct>-?\d+)%,\s*Voltage:\s*(?P<voltage>[\d.]+)\s*V,\s*"
    r"Altitude:\s*(?P<altitude>-?[\d.]+)\s*m,\s*Mission:\s*(?P<mission_stage>\w+)"
    r"(?:,\s*Search:\s*(?P<search_done>\d+)/(?P<search_total>\d+))?"
)
POSITION_RE = re.compile(
    r"Latitude:\s*(?P<lat>-?[\d.]+),\s*Longitude:\s*(?P<lon>-?[\d.]+),\s*"
    r"Altitude:\s*(?P<altitude>-?[\d.]+)\s*m"
)


class LogHub:
    """Shared ring buffer of log lines, fanned out to SSE clients."""

    def __init__(self, maxlen=2000):
        self.lines = deque(maxlen=maxlen)
        self.total = 0
        self.cond = threading.Condition()

    def push(self, source, line):
        with self.cond:
            self.lines.append(f"[{source}] {line}")
            self.total += 1
            self.cond.notify_all()

    def stream(self):
        # Track a running line count, not a buffer index: once the ring
        # buffer is full its length stops growing, and an index-based
        # cursor would never see another line.
        seen = 0
        while True:
            with self.cond:
                self.cond.wait_for(lambda: self.total > seen, timeout=15.0)
                new_count = self.total - seen
                pending = list(self.lines)[-new_count:] if new_count else []
                seen = self.total
            if not pending:
                yield ": keepalive\n\n"
            for line in pending:
                yield f"data: {json.dumps(line)}\n\n"


log_hub = LogHub()


class ProcessManager:
    """Tracks the long-running subprocesses the GUI can start/stop."""

    def __init__(self):
        self.procs = {}
        self.lock = threading.Lock()

    def _reader(self, name, proc):
        for raw in iter(proc.stdout.readline, b""):
            line = raw.decode("utf-8", errors="replace").rstrip()
            if line:
                log_hub.push(name, line)
        log_hub.push(name, "[process exited]")

    def start(self, name, cmd):
        with self.lock:
            existing = self.procs.get(name)
            if existing and existing.poll() is None:
                return False, f"{name} is already running."
            proc = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                preexec_fn=os.setsid,
                cwd=HOME,
            )
            self.procs[name] = proc
        threading.Thread(target=self._reader, args=(name, proc), daemon=True).start()
        log_hub.push(name, f"started (pid {proc.pid})")
        return True, f"{name} launched (pid {proc.pid})."

    def stop(self, name):
        with self.lock:
            proc = self.procs.pop(name, None)
        if not proc:
            return False, f"{name} is not running."
        was_running = proc.poll() is None
        # Signal the whole group (pgid == leader pid, via setsid) even if
        # the leader already exited: e.g. `ros2 launch` dying still leaves
        # gz sim / arducopter alive in its group.
        for sig in (signal.SIGTERM, signal.SIGKILL):
            try:
                os.killpg(proc.pid, sig)
            except ProcessLookupError:
                break
            try:
                proc.wait(timeout=5)
                time.sleep(0.5)
            except subprocess.TimeoutExpired:
                pass
        return True, f"{name} stopped." if was_running else f"{name} cleaned up."

    def is_running(self, name):
        with self.lock:
            proc = self.procs.get(name)
        return bool(proc and proc.poll() is None)

    def kill_all_fallback(self):
        """Safety net for orphaned gz/ardupilot/ros2 processes, mirroring
        the pkill sequence the user runs by hand between sim runs."""
        for pattern in KILL_PATTERNS:
            subprocess.run(["pkill", "-9", "-f", pattern], stdout=subprocess.DEVNULL,
                            stderr=subprocess.DEVNULL)


procs = ProcessManager()


class RosBridge(Node):
    def __init__(self):
        super().__init__("command_center_bridge")
        self.bridge = CvBridge()
        self.frame_lock = threading.Lock()
        self.latest_jpeg = None
        self.state_lock = threading.Lock()
        self.telemetry = {
            "armed": False,
            "mode": "UNKNOWN",
            "battery_pct": -1,
            "battery_voltage": 0.0,
            "altitude": 0.0,
            "latitude": 0.0,
            "longitude": 0.0,
            "mission_stage": "IDLE",
            "search_done": 0,
            "search_total": 0,
            "persons_detected": 0,
            "person_confirmed": False,
            "payload_dropped": False,
            "sensor_mode": "day",
        }

        self.create_subscription(String, "/resq/drone/state", self._on_state, 10)
        self.create_subscription(String, "/resq/drone/position", self._on_position, 10)
        self.create_subscription(
            Detection2DArray, "/detections/persons", self._on_persons,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Bool, "/detections/person_confirmed", self._on_person_confirmed, 10,
        )
        self.create_subscription(
            Image, "/detections/debug_image", self._on_frame,
            qos_profile_sensor_data,
        )

        self.mission_start_client = self.create_client(SetBool, "/resq/mission/start")
        self.mission_land_client = self.create_client(Trigger, "/resq/mission/land")
        self.mode_pub = self.create_publisher(String, "/resq/sensor_mode", SENSOR_MODE_QOS)
        self.set_mode("day")

    def set_mode(self, mode):
        with self.state_lock:
            self.telemetry["sensor_mode"] = mode
        self.mode_pub.publish(String(data=mode))

    def get_mode(self):
        with self.state_lock:
            return self.telemetry["sensor_mode"]

    def _on_state(self, msg):
        match = STATE_RE.search(msg.data)
        if not match:
            return
        with self.state_lock:
            self.telemetry["armed"] = match.group("armed") == "True"
            self.telemetry["mode"] = match.group("mode")
            self.telemetry["battery_pct"] = int(match.group("battery_pct"))
            self.telemetry["battery_voltage"] = float(match.group("voltage"))
            self.telemetry["altitude"] = float(match.group("altitude"))
            self.telemetry["mission_stage"] = match.group("mission_stage")
            if match.group("search_total") is not None:
                self.telemetry["search_done"] = int(match.group("search_done"))
                self.telemetry["search_total"] = int(match.group("search_total"))

    def _on_position(self, msg):
        match = POSITION_RE.search(msg.data)
        if not match:
            return
        with self.state_lock:
            self.telemetry["latitude"] = float(match.group("lat"))
            self.telemetry["longitude"] = float(match.group("lon"))

    def _on_persons(self, msg):
        with self.state_lock:
            self.telemetry["persons_detected"] = len(msg.detections)

    def _on_person_confirmed(self, msg):
        with self.state_lock:
            self.telemetry["person_confirmed"] = msg.data

    def _on_frame(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:  # noqa: BLE001 - log and drop a bad frame
            self.get_logger().warn(f"frame decode failed: {exc}")
            return
        ok, jpeg = cv2.imencode(".jpg", cv_image, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if not ok:
            return
        with self.frame_lock:
            self.latest_jpeg = jpeg.tobytes()

    def get_telemetry(self):
        with self.state_lock:
            return dict(self.telemetry)

    def set_payload_dropped(self, value):
        with self.state_lock:
            self.telemetry["payload_dropped"] = value

    def get_frame(self):
        with self.frame_lock:
            return self.latest_jpeg

    def clear_frame(self):
        with self.frame_lock:
            self.latest_jpeg = None

    def call_mission_start(self, start):
        if not self.mission_start_client.wait_for_service(timeout_sec=2.0):
            return False, "mission service unavailable (is the sim launched?)"
        req = SetBool.Request()
        req.data = start
        future = self.mission_start_client.call_async(req)
        return self._wait_future(future)

    def call_mission_land(self):
        if not self.mission_land_client.wait_for_service(timeout_sec=2.0):
            return False, "land service unavailable (is the sim launched?)"
        req = Trigger.Request()
        future = self.mission_land_client.call_async(req)
        return self._wait_future(future)

    @staticmethod
    def _wait_future(future, timeout_sec=10.0):
        deadline = time.time() + timeout_sec
        while not future.done() and time.time() < deadline:
            time.sleep(0.05)
        if not future.done():
            return False, "service call timed out"
        result = future.result()
        if result is None:
            return False, "service call failed"
        return bool(result.success), result.message


ros_bridge = None


def start_ros_bridge():
    global ros_bridge
    rclpy.init()
    ros_bridge = RosBridge()
    executor_thread = threading.Thread(
        target=rclpy.spin, args=(ros_bridge,), daemon=True
    )
    executor_thread.start()


app = Flask(__name__, static_folder=None)


@app.get("/")
def index():
    return send_from_directory(FRONTEND_DIR, "index.html")


@app.get("/<path:filename>")
def static_files(filename):
    return send_from_directory(FRONTEND_DIR, filename)


def _json_ok(message, **extra):
    return {"ok": True, "message": message, **extra}


def _json_err(message, **extra):
    return {"ok": False, "message": message, **extra}, 400


@app.post("/api/sim/launch")
def sim_launch():
    global current_world
    world = str((request.get_json(silent=True) or {}).get("world", DEFAULT_WORLD)).lower()
    if world not in WORLDS:
        return _json_err(f"unknown world '{world}' (expected one of: {', '.join(WORLDS)})")
    if procs.is_running("sim"):
        return _json_err(f"sim already running ({current_world}); stop it first.")
    current_world = world
    ros_bridge.clear_frame()
    ros_bridge.set_payload_dropped(False)
    ok1, msg1 = procs.start("sim", sim_launch_cmd(world))
    ok2, msg2 = procs.start("bridge", BRIDGE_CMD)
    if ok1:
        threading.Thread(target=_reapply_lighting_when_ready, daemon=True).start()
    if not (ok1 or ok2):
        return _json_err(f"{msg1} {msg2}")
    return _json_ok(f"{msg1} {msg2}")


@app.post("/api/sim/stop")
def sim_stop():
    procs.stop("bridge")
    procs.stop("sim")
    procs.stop("feed")
    procs.kill_all_fallback()
    ros_bridge.clear_frame()
    return _json_ok("Simulation stack stopped.")


@app.post("/api/feed/start")
def feed_start():
    ok, msg = procs.start("feed", FEED_CMD)
    return _json_ok(msg) if ok else _json_err(msg)


@app.post("/api/feed/stop")
def feed_stop():
    ok, msg = procs.stop("feed")
    ros_bridge.clear_frame()
    return _json_ok(msg) if ok else _json_err(msg)


@app.post("/api/mission/start")
def mission_start():
    ok, msg = ros_bridge.call_mission_start(True)
    return _json_ok(msg) if ok else _json_err(msg)


@app.post("/api/mission/land")
def mission_land():
    ok, msg = ros_bridge.call_mission_land()
    return _json_ok(msg) if ok else _json_err(msg)


def _reapply_lighting_when_ready(timeout_s=180):
    """A fresh sim starts with the world's daytime lights; if night mode
    is selected, re-apply it as soon as Gazebo's light service is up."""
    deadline = time.time() + timeout_s
    while time.time() < deadline and procs.is_running("sim"):
        mode = ros_bridge.get_mode()
        if mode == "day":
            return
        ok, msg = apply_lighting(mode)
        if ok:
            log_hub.push("mode", f"{msg} (re-applied after launch)")
            return
        time.sleep(3)


@app.post("/api/mode")
def set_mode():
    mode = str((request.get_json(silent=True) or {}).get("mode", "")).lower()
    if mode not in LIGHTING:
        return _json_err("mode must be 'day' or 'night'")
    ros_bridge.set_mode(mode)
    log_hub.push("mode", f"sensor mode -> {mode} ({'thermal' if mode == 'night' else 'RGB'})")
    if not procs.is_running("sim"):
        return _json_ok(f"Mode set to {mode}; lighting will apply when the sim launches.", mode=mode)
    ok, msg = apply_lighting(mode)
    log_hub.push("mode", msg)
    if not ok:
        return _json_err(f"Sensor switched to {mode}, but {msg}", mode=mode)
    return _json_ok(f"Switched to {mode} mode.", mode=mode)


@app.post("/api/payload/drop")
def payload_drop():
    if not procs.is_running("sim"):
        return _json_err("sim is not running - nothing to drop.")
    cmd = [
        ROS_ENV, "gz", "topic", "-t", "/model/s500_quad/payload/detach",
        "-m", "gz.msgs.Empty", "-p", "",
    ]
    try:
        result = subprocess.run(cmd, capture_output=True, timeout=5, text=True)
    except subprocess.TimeoutExpired:
        return _json_err("gz topic call timed out (is the sim launched?)")
    if result.returncode != 0:
        return _json_err(f"drop failed: {result.stderr.strip() or result.stdout.strip()}")
    ros_bridge.set_payload_dropped(True)
    log_hub.push("payload", "release triggered")
    return _json_ok("Payload released.")


@app.post("/api/emergency_stop")
def emergency_stop():
    for name in ("bridge", "sim", "feed"):
        procs.stop(name)
    procs.kill_all_fallback()
    ros_bridge.clear_frame()
    return _json_ok("Emergency stop complete.")


@app.get("/api/status")
def status_stream():
    def generate():
        while True:
            payload = ros_bridge.get_telemetry()
            payload["sim_running"] = procs.is_running("sim")
            payload["bridge_running"] = procs.is_running("bridge")
            payload["feed_running"] = procs.is_running("feed")
            payload["world"] = current_world
            if not payload["feed_running"]:
                # Last values from a stopped detector are stale, not live.
                payload["persons_detected"] = 0
                payload["person_confirmed"] = False
            yield f"data: {json.dumps(payload)}\n\n"
            time.sleep(0.5)

    return Response(generate(), mimetype="text/event-stream")


@app.get("/api/logs")
def logs_stream():
    return Response(log_hub.stream(), mimetype="text/event-stream")


@app.get("/video_feed")
def video_feed():
    def generate():
        boundary = b"--frame"
        placeholder = _placeholder_jpeg()
        while True:
            frame = ros_bridge.get_frame() or placeholder
            yield (
                boundary + b"\r\n"
                b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n"
            )
            time.sleep(0.1)

    return Response(
        generate(), mimetype="multipart/x-mixed-replace; boundary=frame"
    )


def _placeholder_jpeg():
    import numpy as np

    img = np.zeros((360, 640, 3), dtype="uint8")
    cv2.putText(
        img, "Waiting for feed...", (140, 190),
        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (120, 120, 120), 2, cv2.LINE_AA,
    )
    ok, jpeg = cv2.imencode(".jpg", img)
    return jpeg.tobytes() if ok else b""


if __name__ == "__main__":
    start_ros_bridge()
    app.run(host="0.0.0.0", port=5000, threaded=True)
