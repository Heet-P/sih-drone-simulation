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
BODY_TEMP_RANGE_K), which rejects cones, embers and other warm clutter.

Inference runs in a background thread on whatever frame is newest. The
debug image is re-published at camera rate with a text banner; boxes are
drawn only on the frame they were computed from (a brief freeze-frame),
since inference lags the live camera.
"""
import os
import threading
import time

import cv2
import numpy as np
import rclpy
from builtin_interfaces.msg import Time
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import Image
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
        self.debug_image_pub = self.create_publisher(Image, "/detections/debug_image", 10)
        self.confirmed_pub = self.create_publisher(Bool, "/detections/person_confirmed", 10)
        self.offset_pub = self.create_publisher(PointStamped, "/detections/person_offset", 10)

        self.lock = threading.Lock()
        self.mode = "day"
        self.latest = None  # (mode, msg, model_input, display, kelvin or None, wall time)
        self.frame_seq = 0
        self.last_detection = None  # (wall time, best confidence)
        self.freeze_until = 0.0
        self.confirm_streak = 0

        self.create_subscription(String, "/resq/sensor_mode", self.mode_cb, LATCHED)
        for mode, cfg in MODES.items():
            self.create_subscription(
                Image, cfg["topic"], lambda msg, m=mode: self.image_cb(m, msg), qos_profile_sensor_data
            )
        threading.Thread(target=self.inference_loop, daemon=True).start()

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
        if mode != self.mode:
            return
        if mode == "night":
            raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            kelvin = raw.astype(np.float32) / THERMAL_UNITS_PER_K
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
        if frozen:
            return
        banner = f"{MODES[mode]['label']}"
        if last is not None and time.time() - last[0] < 5.0:
            banner += f" | last detection: person {last[1]:.2f}, {time.time() - last[0]:.1f}s ago"
        self.publish_debug(msg, display, [], banner)

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
        if banner:
            cv2.putText(annotated, banner, (8, annotated.shape[0] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
        # Built by hand: cv_bridge.cv2_to_imgmsg raises KeyError on this
        # machine (ROS cv_bridge vs. the pip-installed OpenCV).
        out = Image()
        out.header = msg.header
        out.height, out.width = annotated.shape[:2]
        out.encoding = "bgr8"
        out.is_bigendian = 0
        out.step = annotated.shape[1] * 3
        out.data = annotated.tobytes()
        self.debug_image_pub.publish(out)

    def detect(self, mode, frame):
        cfg = MODES[mode]
        height, width = frame.shape[:2]
        batch = [frame if r is None else cv2.rotate(frame, r) for r in ROTATIONS]
        results = self.models[mode](batch, verbose=False, conf=cfg["confidence"],
                                    imgsz=cfg["imgsz"], classes=[cfg["person_class"]])
        found = []
        for rotation, result in zip(ROTATIONS, results):
            for b in result.boxes:
                box = unrotate_box(b.xyxy[0].tolist(), rotation, width, height)
                found.append((box, float(b.conf[0])))
        return merge(found)

    def filter_by_temperature(self, boxes, kelvin):
        kept = []
        for box, conf in boxes:
            ok, hot = body_temperature_ok(kelvin, box)
            if ok:
                kept.append((box, conf))
            else:
                self.get_logger().info(
                    f"[night] rejected thermal detection conf={conf:.2f}: hottest pixels {hot:.1f} K, "
                    f"outside body range {BODY_TEMP_RANGE_K[0]:.0f}-{BODY_TEMP_RANGE_K[1]:.0f} K"
                )
        return kept

    def inference_loop(self):
        last_seq = 0
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
            last_seq = seq
            mode, msg, model_input, display, kelvin, captured_at = latest

            started = time.time()
            try:
                boxes = self.detect(mode, model_input)
                if kelvin is not None:
                    boxes = self.filter_by_temperature(boxes, kelvin)
            except Exception as exc:  # noqa: BLE001 - keep the node alive on a bad frame
                self.get_logger().error(f"inference failed: {exc}")
                continue
            busy += time.time() - started
            processed += 1

            with self.lock:
                if mode != self.mode:
                    continue  # switched sensors mid-inference; this result is stale
                if boxes:
                    self.last_detection = (time.time(), boxes[0][1])
                    self.freeze_until = time.time() + 0.8
            if boxes:
                self.publish_debug(msg, display, boxes, f"{MODES[mode]['label']} | DETECTION (analysed frame)")
            self.publish_results(mode, msg, model_input, boxes, captured_at)

    def publish_results(self, mode, msg, frame, boxes, captured_at):
        height, width = frame.shape[:2]

        # Published every processed frame, empty when nothing is found,
        # so subscribers' counts drop back to zero instead of sticking.
        array = Detection2DArray()
        array.header = msg.header
        for (x1, y1, x2, y2), conf in boxes:
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
