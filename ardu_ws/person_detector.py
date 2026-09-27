#!/usr/bin/env python3
"""
Person detection node, v4: day (RGB) and night (thermal) modes, switched
live via the latched /resq/sensor_mode topic ("day" / "night").

Day: COCO-trained YOLOv8m on /camera/image. The previous VisDrone model
scored the lying casualty at most 0.31 at any altitude from 4-15m (it's
trained on upright pedestrians seen obliquely); from directly above, a
face-up person looks like an ordinary full-body photo of a person rotated
in-plane, which a COCO model detects well once the frame is also tried at
90/180/270 degrees. Measured on rendered frames at 8m: 0.88-0.96.

Night: the user's YOLOv8n trained on HIT-UAV (drone thermal infrared;
classes Person/Car/Bicycle/OtherVehicle) on /thermal/image. The sim's
thermal camera outputs 16-bit radiometric frames (0.01 K per unit); they
are mapped to white-hot 8-bit grey over a fixed temperature window, like a
real thermal camera's output, before inference. Measured on rendered
frames of the casualty's heat signature at 6-20m: 0.51-0.73, and 0.00 on
an empty scene; ~0.15s per frame on CPU.

The drone's X-shaped shadow can score as a "person" in daylight (0.6-0.85
seen in flight); mavlink_bridge.py filters it using the sun direction and
the drone's pose. There is no shadow at night.

Thermal detections are also checked against the frame's temperatures:
a box is kept only if its hottest pixels are at body temperature (see
BODY_TEMP_RANGE_K), which rejects cones, embers and other warm clutter,
and the warm region it sits on must be person-sized (MIN_BODY_LENGTH_M),
which rejects animals. By day the same two checks run on every RGB "person"
box against the boresighted thermal frame (RGB + thermal fusion): the COCO
model also calls the dogs "person", the thermal camera shows they're too small.

Hazards: every thermal frame (at HAZARD_SCAN_S, in day and night mode -
the thermal camera is always on) is scanned for fire / hot spots (above
FIRE_K) and flood water (large regions colder than FLOOD_MAX_K). By day a
cold region is only called water if the boresighted RGB camera agrees it
is flat and textureless like a water surface (multi-sensor fusion); at
night the thermal signature alone decides. Sightings go out on
/detections/hazards as JSON with image offsets; mavlink_bridge.py
projects them to the ground and maps them. (The command center shows the
raw thermal camera next to this feed in its split view.)

Inference runs in a background thread on whatever frame is newest. The
debug image is re-published at camera rate with a text banner; boxes are
drawn only on the frame they were computed from (a brief freeze-frame),
since inference lags the live camera.
"""
import json
import math
import os
import re
import threading
import time

import cv2
import numpy as np
import rclpy
import torch
from builtin_interfaces.msg import Time
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Bool, String
from ultralytics import YOLO
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

HERE = os.path.dirname(os.path.abspath(__file__))

MODES = {
    "day": {
        "label": "RGB",
        "topic": "/camera/image",
        "model": os.path.join(HERE, "yolov8m.pt"),
        "person_class": 0,        # COCO "person"
        "confidence": 0.6,
        "imgsz": 640,
    },
    "night": {
        "label": "THERMAL",
        "topic": "/thermal/image",
        "model": os.path.join(HERE, "thermal_best.pt"),
        "person_class": 0,        # HIT-UAV "Person"
        "confidence": 0.45,       # measured 0.51-0.73 on the casualty, 0.00 on empty ground
        "imgsz": 640,
    },
}

# White-hot mapping for the thermal camera: ground is ~288 K, body 310 K.
THERMAL_WINDOW_K = (283.0, 313.0)
THERMAL_UNITS_PER_K = 100.0  # matches <resolution>0.01</resolution> in model.sdf

# Radiometric check on thermal detections. The HIT-UAV model also fires on
# small warm or saturated blobs (traffic cones, embers of burning debris,
# car parts: 37 false hits up to 0.77 over the disaster world), but the
# camera reports real temperatures, so a box is kept only if its hottest
# pixels are at human skin/clothing temperature. The check uses a high
# percentile rather than the max so a few hot pixels at the edge of a box
# don't decide it.
BODY_TEMP_RANGE_K = (303.0, 316.0)
BODY_TEMP_PERCENTILE = 97

# Size check on thermal detections. The thermal model also calls animals
# "Person" (a dog's fur surface, ~302-307 K, passes the temperature
# check), and it often boxes only part of a body, so the box itself says
# little about size. Instead, measure the whole warm region (> WARM_K)
# the detection sits on, and require its long axis to be at least
# MIN_BODY_LENGTH_M, using the drone's altitude (camera points straight
# down). Rendered at 8 m: casualty 1.79 m; sitting dogs 0.74-0.93 m. An
# adult curled up tighter than 1.1 m would also be rejected. Skipped while
# the altitude is unknown or under MIN_SIZE_ALT_M.
WARM_K = 300.0
MIN_BODY_LENGTH_M = 1.1
MIN_SIZE_ALT_M = 3.0
# Upright people (sitting, standing, walking) are compact from above -
# 0.73-0.85 m, the same size as the dogs (0.72-0.93 m) - so size can't tell
# them apart. Exposed skin can: a person's face and hands are ~309 K, a
# dog's fur surface at most 307 K (dog model: 302-307 K). A compact warm
# region is a person only if its peak reaches SKIN_K. (A 2 K margin in
# this simulation; real thermal footage would need to confirm it.)
SKIN_K = 308.0
MIN_UPRIGHT_M = 0.35
# Thermal-signature detector: any warm region with a skin-temperature peak
# (and no fire-temperature pixels) of person size becomes a candidate, even
# when neither YOLO model recognises the shape - the only route to people
# sitting down, which both models miss from above. Candidates still need
# the bridge's hover verification.
SIGNATURE_CONF = 0.55
SIGNATURE_MAX_M = 2.5
CAMERA_HFOV_RAD = 1.2  # must match the thermal camera in s500_quad/model.sdf
ALTITUDE_RE = re.compile(r"Altitude: (-?[0-9.]+) m")

# Hazard scan on the radiometric thermal frames. FIRE_K (50 C) is well
# above body temperature (BODY_TEMP_RANGE_K) and sun-warmed ground; the
# flood threshold sits below the ground's ~285-289 K spread (see the
# <atmosphere> note in disaster.sdf). Areas are in square metres on the
# ground, from the drone's altitude.
HAZARD_SCAN_S = 1.0
FIRE_K = 323.0
MIN_FIRE_AREA_M2 = 0.05
FLOOD_MAX_K = 283.5
MIN_FLOOD_AREA_M2 = 1.5
# Water in the RGB frame: a flat surface, so its mean |Laplacian| is well
# below the frame's (rubble, gravel and debris are all high-texture).
WATER_TEXTURE_RATIO = 0.5
RGB_PAIR_MAX_S = 0.5
HAZARD_BANNER_S = 2.5

# Day-mode fusion: an RGB "person" box must also pass the thermal checks
# above, using the thermal frame captured within this much of it.
FUSION_MAX_SKEW_S = 0.3

# Live-box mode: when inference is this fast (GPU: ~0.05 s), the latest
# boxes are drawn on the live frames as they stream (they are at most a
# frame or two old, a few pixels off) instead of freezing the video on the
# analysed frame, which with a detection on nearly every frame made the
# feed look stuck. Slow CPU inference keeps the freeze-frame.
LIVE_BOX_MAX_LATENCY_S = 0.25
LIVE_BOX_MAX_AGE_S = 0.5

# GPU budget. Uncapped, the detector analysed ~12 frames/s (4 rotations
# each) and kept the laptop GPU 63% busy on average, 95% at peaks, on top of
# Gazebo's rendering (24% alone): the Gazebo window then couldn't draw on
# time and the drone looked like it stopped and jumped. 5 analyses/s is one
# every 0.4 m at search speed, and verification still gets its 8 frames in
# under 2 s; live frames keep showing the latest boxes in between. FP16
# roughly halves the GPU time per analysis on this class of GPU
# (Ultralytics' quantize=16; its older half=True alias is deprecated).
MAX_INFERENCE_HZ = 5.0
USE_FP16 = torch.cuda.is_available()

# Consecutive processed frames with a person before publishing
# person_confirmed. Leaky counter (a miss decrements rather than
# resets), so one dropped frame doesn't restart confirmation.
CONFIRM_FRAMES = 3

ROTATIONS = [None, cv2.ROTATE_90_CLOCKWISE, cv2.ROTATE_180, cv2.ROTATE_90_COUNTERCLOCKWISE]

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def unrotate_box(box, rotation, width, height):
    """Map an (x1, y1, x2, y2) box from a rotated frame back to the original."""
    x1, y1, x2, y2 = box
    if rotation is None:
        pts = [(x1, y1), (x2, y2)]
    elif rotation == cv2.ROTATE_90_CLOCKWISE:
        pts = [(y1, height - 1 - x1), (y2, height - 1 - x2)]
    elif rotation == cv2.ROTATE_180:
        pts = [(width - 1 - x1, height - 1 - y1), (width - 1 - x2, height - 1 - y2)]
    else:
        pts = [(width - 1 - y1, x1), (width - 1 - y2, x2)]
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    return min(xs), min(ys), max(xs), max(ys)


def iou(a, b):
    ix = max(0.0, min(a[2], b[2]) - max(a[0], b[0]))
    iy = max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
    inter = ix * iy
    union = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / union if union > 0 else 0.0


def merge(detections, iou_threshold=0.4):
    """Greedy NMS across the rotated passes: keep the most confident box per person."""
    kept = []
    for det in sorted(detections, key=lambda d: d[1], reverse=True):
        if all(iou(det[0], k[0]) < iou_threshold for k in kept):
            kept.append(det)
    return kept


def body_temperature_ok(kelvin, box):
    """True if the hottest pixels in box (x1, y1, x2, y2) are at body temperature."""
    h, w = kelvin.shape
    x1, y1 = max(int(box[0]), 0), max(int(box[1]), 0)
    x2, y2 = min(int(np.ceil(box[2])), w), min(int(np.ceil(box[3])), h)
    if x2 <= x1 or y2 <= y1:
        return False, 0.0
    hot = float(np.percentile(kelvin[y1:y2, x1:x2], BODY_TEMP_PERCENTILE))
    return BODY_TEMP_RANGE_K[0] <= hot <= BODY_TEMP_RANGE_K[1], hot


def warm_region(kelvin, box):
    """Mask of the warm region(s) (> WARM_K) that the box sits on."""
    warm = cv2.morphologyEx((kelvin > WARM_K).astype(np.uint8), cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    count, labels = cv2.connectedComponents(warm)
    h, w = kelvin.shape
    x1, y1 = max(int(box[0]), 0), max(int(box[1]), 0)
    x2, y2 = min(int(np.ceil(box[2])), w), min(int(np.ceil(box[3])), h)
    ids = np.unique(labels[y1:y2, x1:x2])
    ids = ids[ids > 0]
    return np.isin(labels, ids) if len(ids) else None


def warm_region_length(kelvin, box, altitude, region=None):
    """Long axis, in metres, of the warm region (> WARM_K) under box."""
    region = warm_region(kelvin, box) if region is None else region
    if region is None:
        return 0.0
    points = np.column_stack(np.nonzero(region))[:, ::-1].astype(np.float32)
    (_, _), (a, b), _ = cv2.minAreaRect(points)
    focal = (kelvin.shape[1] / 2.0) / math.tan(CAMERA_HFOV_RAD / 2.0)
    return max(a, b) * altitude / focal


def thermal_to_grey(raw16):
    lo, hi = THERMAL_WINDOW_K
    kelvin = raw16.astype(np.float32) / THERMAL_UNITS_PER_K
    return np.clip((kelvin - lo) / (hi - lo) * 255.0, 0, 255).astype(np.uint8)


class PersonDetector(Node):
    def __init__(self):
        super().__init__("person_detector")

        self.bridge = CvBridge()
        self.models = {}
        for mode, cfg in MODES.items():
            self.get_logger().info(f"Loading {mode} model {cfg['model']} ...")
            self.models[mode] = YOLO(cfg["model"])
        self.get_logger().info("Models loaded (day: COCO person, night: HIT-UAV thermal person).")

        self.detections_pub = self.create_publisher(Detection2DArray, "/detections/persons", 10)
        # JPEG, not a raw Image: filling a raw 640x480 Image's data from
        # Python takes ~93 ms per frame in rclpy, which capped the dashboard
        # feed at ~3-4 fps; encoding + publishing a JPEG takes a few ms.
        self.debug_image_pub = self.create_publisher(CompressedImage, "/detections/debug_image/compressed", 10)
        self.confirmed_pub = self.create_publisher(Bool, "/detections/person_confirmed", 10)
        self.offset_pub = self.create_publisher(PointStamped, "/detections/person_offset", 10)
        self.hazards_pub = self.create_publisher(String, "/detections/hazards", 10)

        self.lock = threading.Lock()
        self.mode = "day"
        self.latest = None  # (mode, msg, model_input, display, (kelvin, altitude) or None, wall time)
        self.frame_seq = 0
        self.last_detection = None  # (wall time, best confidence)
        self.freeze_until = 0.0
        self.live_boxes = None  # (wall time, boxes), in live-box mode
        self.confirm_streak = 0
        self.altitude = None  # metres above home, from the mission bridge
        self.latest_rgb = None  # (msg, wall time), for pairing with thermal
        self.last_hazard_scan = 0.0
        self.latest_thermal = None  # (msg, wall time)
        self.hazard_banner = None  # (wall time, text)

        self.create_subscription(String, "/resq/sensor_mode", self.mode_cb, LATCHED)
        self.create_subscription(String, "/resq/drone/position", self.position_cb, 10)
        for mode, cfg in MODES.items():
            self.create_subscription(
                Image, cfg["topic"], lambda msg, m=mode: self.image_cb(m, msg), qos_profile_sensor_data
            )
        threading.Thread(target=self.inference_loop, daemon=True).start()

    def position_cb(self, msg):
        match = ALTITUDE_RE.search(msg.data)
        if match:
            self.altitude = float(match.group(1))

    def mode_cb(self, msg):
        mode = msg.data.strip().lower()
        if mode not in MODES:
            self.get_logger().warn(f"ignoring unknown sensor mode '{msg.data}'")
            return
        with self.lock:
            if mode == self.mode:
                return
            self.mode = mode
            self.latest = None
            self.last_detection = None
            self.freeze_until = 0.0
            self.confirm_streak = 0
        self.get_logger().info(f"sensor mode -> {mode} ({MODES[mode]['label']})")

    def image_cb(self, mode, msg):
        # Both cameras always stream: thermal feeds the hazard scan in
        # either mode, and RGB is paired with it to confirm water.
        if mode == "night":
            self.thermal_side_tasks(msg)
        else:
            self.latest_rgb = (msg, time.time())
        if mode != self.mode:
            return
        if mode == "night":
            raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            kelvin = (raw.astype(np.float32) / THERMAL_UNITS_PER_K, self.altitude)
            grey = thermal_to_grey(raw)
            model_input = cv2.cvtColor(grey, cv2.COLOR_GRAY2BGR)
            display = cv2.applyColorMap(grey, cv2.COLORMAP_INFERNO)
        else:
            model_input = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            display = model_input
            kelvin = None
        with self.lock:
            if mode != self.mode:
                return
            self.latest = (mode, msg, model_input, display, kelvin, time.time())
            self.frame_seq += 1
            frozen = time.time() < self.freeze_until
            last = self.last_detection
            live = self.live_boxes
        if frozen:
            return
        boxes = live[1] if live is not None and time.time() - live[0] < LIVE_BOX_MAX_AGE_S else []
        banner = f"{MODES[mode]['label']}"
        if boxes:
            banner += " | PERSON DETECTED"
        elif last is not None and time.time() - last[0] < 5.0:
            banner += f" | last detection: person {last[1]:.2f}, {time.time() - last[0]:.1f}s ago"
        self.publish_debug(msg, display, boxes, banner)

    def thermal_side_tasks(self, msg):
        now = time.time()
        # Kept for fusion: day-mode RGB detections are checked against it.
        self.latest_thermal = (msg, now)
        if now - self.last_hazard_scan < HAZARD_SCAN_S:
            return
        raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        self.last_hazard_scan = now
        try:
            self.scan_hazards(raw.astype(np.float32) / THERMAL_UNITS_PER_K, now)
        except Exception as exc:  # noqa: BLE001 - keep the node alive on a bad frame
            self.get_logger().error(f"hazard scan failed: {exc}")

    def scan_hazards(self, kelvin, captured_at):
        altitude = self.altitude
        if altitude is None or altitude < MIN_SIZE_ALT_M:
            return
        h, w = kelvin.shape
        focal = (w / 2.0) / math.tan(CAMERA_HFOV_RAD / 2.0)
        px_per_m2 = (focal / altitude) ** 2
        found = []

        hot = cv2.morphologyEx((kelvin >= FIRE_K).astype(np.uint8), cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
        count, labels, stats, centroids = cv2.connectedComponentsWithStats(hot)
        for i in range(1, count):
            area = stats[i, cv2.CC_STAT_AREA]
            if area / px_per_m2 < MIN_FIRE_AREA_M2:
                continue
            peak = float(kelvin[labels == i].max())
            found.append({
                "type": "fire", "cx": centroids[i][0] / w - 0.5, "cy": centroids[i][1] / h - 0.5,
                "r_px": 0.5 * math.hypot(stats[i, cv2.CC_STAT_WIDTH], stats[i, cv2.CC_STAT_HEIGHT]),
                "peak_k": round(peak, 1), "conf": round(min(1.0, 0.6 + (peak - FIRE_K) / 50.0), 2),
                "sensors": ["thermal"], "area_m2": round(area / px_per_m2, 2),
            })

        cold = (kelvin <= FLOOD_MAX_K).astype(np.uint8)
        cold = cv2.morphologyEx(cold, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        cold = cv2.morphologyEx(cold, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
        count, labels, stats, centroids = cv2.connectedComponentsWithStats(cold)
        texture = None
        rgb = self.latest_rgb
        if self.mode == "day" and rgb is not None and abs(rgb[1] - captured_at) < RGB_PAIR_MAX_S:
            grey = cv2.cvtColor(self.bridge.imgmsg_to_cv2(rgb[0], desired_encoding="bgr8"), cv2.COLOR_BGR2GRAY)
            texture = np.abs(cv2.Laplacian(cv2.GaussianBlur(grey, (3, 3), 0), cv2.CV_32F))
        for i in range(1, count):
            area = stats[i, cv2.CC_STAT_AREA]
            if area / px_per_m2 < MIN_FLOOD_AREA_M2:
                continue
            region = labels == i
            sensors, conf = ["thermal"], 0.7
            if texture is not None:
                ratio = float(texture[region].mean() / max(float(texture.mean()), 1e-3))
                if ratio > WATER_TEXTURE_RATIO:
                    self.get_logger().info(
                        f"cold region {area / px_per_m2:.1f} m2 rejected as water: RGB texture ratio {ratio:.2f}"
                    )
                    continue
                sensors, conf = ["thermal", "rgb"], 0.95
            found.append({
                "type": "flood", "cx": centroids[i][0] / w - 0.5, "cy": centroids[i][1] / h - 0.5,
                "r_px": 0.5 * math.hypot(stats[i, cv2.CC_STAT_WIDTH], stats[i, cv2.CC_STAT_HEIGHT]),
                "peak_k": round(float(kelvin[region].min()), 1), "conf": conf,
                "sensors": sensors, "area_m2": round(area / px_per_m2, 2),
            })

        if not found:
            return
        self.hazards_pub.publish(String(data=json.dumps(
            {"stamp": captured_at, "alt": altitude, "hazards": found}
        )))
        parts = []
        for hz in found:
            if hz["type"] == "fire":
                parts.append(f"FIRE {hz['peak_k'] - 273.15:.0f}C")
            else:
                parts.append(f"FLOOD WATER {hz['area_m2']:.0f}m2 ({'+'.join(s.upper() for s in hz['sensors'])})")
        self.hazard_banner = (time.time(), "HAZARD: " + ", ".join(parts))

    def publish_debug(self, msg, frame, boxes, banner=None):
        # Boxes are only ever drawn on the exact frame they were computed
        # from. Inference lags the live camera, so drawing them on later
        # frames puts them metres away from the person once the drone has
        # moved; instead the analysed frame is shown briefly as a
        # freeze-frame, and live frames just get a text banner.
        annotated = frame.copy()
        for (x1, y1, x2, y2), conf in boxes:
            cv2.rectangle(annotated, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            cv2.putText(annotated, f"person {conf:.2f}", (int(x1), max(int(y1) - 6, 12)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
        # Status and hazard text sit bottom-centre, clear of the command
        # center's overlays (view switcher, HUD, live chip, compass).
        lines = []
        hazard = self.hazard_banner
        if hazard is not None and time.time() - hazard[0] < HAZARD_BANNER_S:
            lines.append((hazard[1], (0, 140, 255)))
        if banner:
            lines.append((banner, (0, 255, 0)))
        y = annotated.shape[0] - 10
        for text, colour in reversed(lines):
            (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)
            x = (annotated.shape[1] - tw) // 2
            cv2.rectangle(annotated, (x - 6, y - th - 6), (x + tw + 6, y + 5), (0, 0, 0), -1)
            cv2.putText(annotated, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.45, colour, 1, cv2.LINE_AA)
            y -= th + 14
        ok, jpeg = cv2.imencode(".jpg", annotated, [cv2.IMWRITE_JPEG_QUALITY, 85])
        if not ok:
            return
        out = CompressedImage()
        out.header = msg.header
        out.format = "jpeg"
        out.data = jpeg.tobytes()
        self.debug_image_pub.publish(out)

    def detect(self, mode, frame):
        cfg = MODES[mode]
        height, width = frame.shape[:2]
        batch = [frame if r is None else cv2.rotate(frame, r) for r in ROTATIONS]
        results = self.models[mode](batch, verbose=False, conf=cfg["confidence"],
                                    imgsz=cfg["imgsz"], classes=[cfg["person_class"]],
                                    quantize=16 if USE_FP16 else None)
        found = []
        for rotation, result in zip(ROTATIONS, results):
            for b in result.boxes:
                box = unrotate_box(b.xyxy[0].tolist(), rotation, width, height)
                found.append((box, float(b.conf[0])))
        return merge(found)

    def filter_thermal(self, boxes, thermal, what="thermal detection"):
        """Temperature check, then size check (see BODY_TEMP_RANGE_K, MIN_BODY_LENGTH_M).
        Used on the thermal model's own boxes at night, and on the RGB
        model's boxes by day (the cameras are boresighted, so a box covers
        the same ground in both frames): multi-sensor fusion."""
        kelvin, altitude = thermal
        kept = []
        for box, conf in boxes:
            ok, hot = body_temperature_ok(kelvin, box)
            if not ok:
                self.get_logger().info(
                    f"[{self.mode}] rejected {what} conf={conf:.2f}: hottest pixels {hot:.1f} K, "
                    f"outside body range {BODY_TEMP_RANGE_K[0]:.0f}-{BODY_TEMP_RANGE_K[1]:.0f} K"
                )
                continue
            region = warm_region(kelvin, box)
            # Smouldering debris: its ash is at body temperature (~305 K),
            # but it's one warm region with embers at fire temperature. A
            # person's warm region never contains pixels that hot.
            if region is not None:
                peak = float(kelvin[region].max())
                if peak >= FIRE_K:
                    self.get_logger().info(
                        f"[{self.mode}] rejected {what} conf={conf:.2f}: its warm region reaches {peak:.0f} K "
                        f"(fire / embers), not a person"
                    )
                    continue
            if altitude is not None and altitude >= MIN_SIZE_ALT_M:
                length = warm_region_length(kelvin, box, altitude, region)
                skin = region is not None and float(kelvin[region].max()) >= SKIN_K
                if length < MIN_BODY_LENGTH_M and not (length >= MIN_UPRIGHT_M and skin):
                    peak = float(kelvin[region].max()) if region is not None else 0.0
                    self.get_logger().info(
                        f"[{self.mode}] rejected {what} conf={conf:.2f}: warm region {length:.2f} m long "
                        f"at {altitude:.1f} m, too short to be lying and no exposed skin "
                        f"(peak {peak:.1f} K < {SKIN_K:.0f} K) - likely an animal"
                    )
                    continue
            kept.append((box, conf))
        return kept

    def signature_boxes(self, thermal):
        """Person-sized warm regions with a skin-temperature peak (see SKIN_K)."""
        kelvin, altitude = thermal
        if altitude is None or altitude < MIN_SIZE_ALT_M:
            return []
        warm = cv2.morphologyEx((kelvin > WARM_K).astype(np.uint8), cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
        count, labels, stats, _ = cv2.connectedComponentsWithStats(warm)
        focal = (kelvin.shape[1] / 2.0) / math.tan(CAMERA_HFOV_RAD / 2.0)
        boxes = []
        for i in range(1, count):
            x, y, w, h, area = stats[i]
            if max(w, h) * altitude / focal < MIN_UPRIGHT_M:
                continue
            region = labels == i
            peak = float(kelvin[region].max())
            if not (SKIN_K <= peak < FIRE_K):
                continue
            if warm_region_length(kelvin, (x, y, x + w, y + h), altitude, region) > SIGNATURE_MAX_M:
                continue
            pad = 3
            boxes.append(((float(x - pad), float(y - pad), float(x + w + pad), float(y + h + pad)), SIGNATURE_CONF))
        return boxes

    def thermal_for(self, captured_at):
        """(kelvin, altitude) from the thermal frame taken with an RGB
        frame, or None if there isn't one close enough in time."""
        latest = self.latest_thermal
        if latest is None or abs(latest[1] - captured_at) > FUSION_MAX_SKEW_S:
            return None
        raw = self.bridge.imgmsg_to_cv2(latest[0], desired_encoding="passthrough")
        return raw.astype(np.float32) / THERMAL_UNITS_PER_K, self.altitude

    def inference_loop(self):
        last_seq = 0
        next_allowed = 0.0
        stats_start, processed, busy = time.time(), 0, 0.0
        while rclpy.ok():
            if time.time() - stats_start >= 10.0 and processed:
                self.get_logger().info(
                    f"[{self.mode}] processed {processed} frames in {time.time() - stats_start:.0f}s, "
                    f"avg inference {busy / processed:.2f}s"
                )
                stats_start, processed, busy = time.time(), 0, 0.0
            with self.lock:
                seq, latest = self.frame_seq, self.latest
            if latest is None or seq == last_seq:
                time.sleep(0.05)
                continue
            # Rate cap: leave the GPU to Gazebo's own rendering in between.
            wait = next_allowed - time.time()
            if wait > 0:
                time.sleep(wait)
                continue
            next_allowed = time.time() + 1.0 / MAX_INFERENCE_HZ
            last_seq = seq
            mode, msg, model_input, display, kelvin, captured_at = latest

            started = time.time()
            try:
                boxes = self.detect(mode, model_input)
                if kelvin is not None:
                    # Night: thermal model + thermal signatures, checked.
                    boxes = self.filter_thermal(boxes, kelvin)
                    boxes = merge(boxes + self.filter_thermal(self.signature_boxes(kelvin), kelvin, "thermal signature"))
                else:
                    # Day: the RGB model calls dogs "person" too, and misses
                    # upright people from above; the boresighted thermal
                    # camera checks its boxes and adds its own (thermal
                    # model + signatures), all through the same checks.
                    thermal = self.thermal_for(captured_at)
                    if thermal is not None:
                        boxes = self.filter_thermal(boxes, thermal, "RGB detection (thermal check)")
                        grey = np.clip((thermal[0] - THERMAL_WINDOW_K[0]) / (THERMAL_WINDOW_K[1] - THERMAL_WINDOW_K[0])
                                       * 255.0, 0, 255).astype(np.uint8)
                        t_boxes = self.detect("night", cv2.cvtColor(grey, cv2.COLOR_GRAY2BGR))
                        t_boxes = self.filter_thermal(t_boxes, thermal, "thermal-model detection")
                        s_boxes = self.filter_thermal(self.signature_boxes(thermal), thermal, "thermal signature")
                        boxes = merge(boxes + t_boxes + s_boxes)
            except Exception as exc:  # noqa: BLE001 - keep the node alive on a bad frame
                self.get_logger().error(f"inference failed: {exc}")
                continue
            latency = time.time() - started
            busy += latency
            processed += 1
            live_mode = latency < LIVE_BOX_MAX_LATENCY_S

            with self.lock:
                if mode != self.mode:
                    continue  # switched sensors mid-inference; this result is stale
                self.live_boxes = (time.time(), boxes) if live_mode else None
                if boxes:
                    self.last_detection = (time.time(), boxes[0][1])
                    if not live_mode:
                        self.freeze_until = time.time() + 0.8
            if boxes and not live_mode:
                self.publish_debug(msg, display, boxes, f"{MODES[mode]['label']} | DETECTION (analysed frame)")
            self.publish_results(mode, msg, model_input, boxes, captured_at)

    def publish_results(self, mode, msg, frame, boxes, captured_at):
        height, width = frame.shape[:2]

        # Published every processed frame, empty when nothing is found,
        # so subscribers' counts drop back to zero instead of sticking.
        array = Detection2DArray()
        array.header.frame_id = msg.header.frame_id
        # Wall-clock capture time, like person_offset below: the bridge
        # looks up the drone's pose for this frame to update its map.
        sec = int(captured_at)
        array.header.stamp = Time(sec=sec, nanosec=int((captured_at - sec) * 1e9))
        for box, conf in boxes:
            x1, y1, x2, y2 = (float(v) for v in box)  # ROS fields must be float
            conf = float(conf)
            det = Detection2D()
            det.header = msg.header
            det.bbox.center.position.x = (x1 + x2) / 2.0
            det.bbox.center.position.y = (y1 + y2) / 2.0
            det.bbox.size_x = x2 - x1
            det.bbox.size_y = y2 - y1
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = "person"
            hyp.hypothesis.score = conf
            det.results.append(hyp)
            array.detections.append(det)
        self.detections_pub.publish(array)

        if boxes:
            (x1, y1, x2, y2), conf = boxes[0]
            offset = PointStamped()
            # WALL-CLOCK time the frame arrived, not sim time: the
            # consumer (mavlink_bridge) keeps a wall-clock history of
            # the drone's pose and looks up where it was when this
            # frame was taken, since inference lags behind.
            sec = int(captured_at)
            offset.header.stamp = Time(sec=sec, nanosec=int((captured_at - sec) * 1e9))
            offset.header.frame_id = "camera_normalized"
            offset.point.x = ((x1 + x2) / 2.0 - width / 2.0) / width
            offset.point.y = ((y1 + y2) / 2.0 - height / 2.0) / height
            offset.point.z = conf
            self.offset_pub.publish(offset)
            self.confirm_streak = min(self.confirm_streak + 1, CONFIRM_FRAMES)
            self.get_logger().info(
                f"[{mode}] person conf={conf:.2f} offset=({offset.point.x:+.2f},{offset.point.y:+.2f})"
            )
        else:
            self.confirm_streak = max(self.confirm_streak - 1, 0)

        self.confirmed_pub.publish(Bool(data=self.confirm_streak >= CONFIRM_FRAMES))


def main():
    rclpy.init()
    node = PersonDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
