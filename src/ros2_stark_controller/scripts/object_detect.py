#!/usr/bin/env python3
"""
object_detect.py — detects one or more named target objects (e.g. the
robot's own hand wearing a blue medical glove, a teal ball, a red pen
standing in for a vial of blood) in the Intel RealSense D435i's RGB
stream by color, and estimates each one's 3D position (in the camera's
optical frame) using the depth image + camera intrinsics.

STATUS (updated 2026-07-23 once the camera was confirmed running via
`cd cam_ws/ && . install/local_setup.bash && ros2 launch realsense2_camera
rs_launch.py`): real topic names below are now CONFIRMED from an actual
`ros2 topic list` on the robot — this wrapper version double-namespaces
as /camera/camera/... (not the single /camera/... some realsense-ros
versions use). CONFIRMED WORKING ON HARDWARE 2026-07-23 (single-object
mode, before the multi-object rewrite below): found a calibrated orange
sticky note reliably in the color image, with stable depth/3D output
once pointed at the raw (unaligned) depth topic. Still unconfirmed /
things to check before trusting a real run:

  1. DEPTH ALIGNMENT — the captured topic list only had
     /camera/camera/depth/image_rect_raw (raw depth), no
     aligned_depth_to_color topic. back_project() assumes depth and color
     share the same pixel grid, which is only true for ALIGNED depth —
     unaligned depth (what the 2026-07-23 hardware test actually used, via
     --depth-topic override) gives a real but slightly offset 3D position,
     not a wrong-in-principle one, since the D435i's depth and color
     sensors are physically only a few cm apart. The --depth-topic default
     below (.../aligned_depth_to_color/image_raw) is a GUESSED name for
     what this topic will be called once alignment is enabled — NOT
     confirmed to exist yet. Before relying on this for anything needing
     precision, relaunch with alignment on:
       ros2 launch realsense2_camera rs_launch.py --show-args | grep -i align
       ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true
     (confirm the exact launch arg name via --show-args first — the name
     above is a reasonable guess based on this wrapper's naming style,
     not verified), then re-check `ros2 topic list | grep -i camera` for
     the real aligned-depth topic name and pass it via --depth-topic if
     it differs from the default.
  2. cv_bridge/cv2 ARE importable here (confirmed 2026-07-23 — the script
     got past all imports to reach argparse validation on the first real
     run). However cv_bridge.cv2_to_imgmsg() is BROKEN on this robot —
     see bgr8_to_imgmsg() below for the confirmed root cause and the
     workaround already applied.
  3. Depth encoding/scale — assumes the realsense2_camera default, 16-bit
     unsigned millimeters (--depth-scale 0.001 converts to meters). Not
     yet rigorously sanity-checked against a known real-world distance,
     though the 2026-07-23 hardware run produced a stable, plausible-looking
     ~0.65m reading.
  4. --object color ranges are NEVER guessed defaults — always required
     explicitly, one per target (see below). A fabricated "plausible-
     looking" default could appear to work while actually detecting
     nothing real. Use --calibrate (one object at a time) to find real
     values against the actual objects/lighting.

Detection method: HSV color thresholding + largest-contour centroid per
named target, not a trained model. Deliberately dependency-light (no
model file, no GPU, nothing beyond opencv/cv_bridge that the ROS image
pipeline needs anyway) since the target objects are simple, distinctly-
colored items against a plain desk.

Multiple objects, one process: pass --object repeatedly, once per target,
each with its own name and HSV range:
  --object NAME:Hlo,Slo,Vlo:Hhi,Shi,Vhi
Each named object gets its own 3D-position topic
(--output-topic-prefix/NAME, e.g. /detected_object_position/teal_ball) and
is drawn in its own color (cycled from a small palette) with its name
labeled on ONE shared debug image (--debug-topic) — so you can watch all
targets at once in a single rqt_image_view window rather than juggling
several script instances/windows. One example use case this was built
for: detecting the robot's own gripper (wearing a colored glove as a
visual marker) alongside separate target objects in the same frame, e.g.
to know where its own end-effector is in the camera view without relying
on precise forward kinematics.

This node only DETECTS and REPORTS 3D positions — it does not move the
arm. Arm motion is still blocked on the missing unitree_hg workspace (see
current.md, session 8) and is a separate, higher-stakes task to wire up
once that's resolved (though session 13 found /arm_sdk and /api/arm/*
topics already active on the robot, which may offer a way around that
blocker — unconfirmed, worth a follow-up).

Modes:
  --calibrate     live HSV trackbar tuning window (needs ssh -X, same as
                  rqt_plot/slider_control.py) — move the sliders until
                  only ONE target object is white in the mask view (do
                  this once per object), then use those H/S/V values as
                  one --object entry for a real detection run below.
  (default)       runs detection for every --object given, publishes each
                  one's geometry_msgs/PointStamped on
                  {--output-topic-prefix}/{name} and one shared annotated
                  debug image on --debug-topic (default
                  /object_detect/debug_image, viewable via
                  `ros2 run rqt_image_view rqt_image_view` over ssh -X).

Usage:
  python3 object_detect.py --calibrate
  python3 object_detect.py \\
      --object glove:100,80,80:130,255,255 \\
      --object teal_ball:85,100,100:100,255,255 \\
      --object red_pen:0,120,80:8,255,255
"""

import argparse
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge

MIN_CONTOUR_AREA = 200  # px^2 — filters out small color-noise blobs
LOG_THROTTLE_S = 0.5    # don't flood the log every frame at ~30Hz per object

# Cycled per --object, in order given, so each target gets a visually
# distinct debug overlay color. BGR tuples.
PALETTE = [
    (0, 255, 0),      # green
    (0, 165, 255),    # orange
    (255, 0, 255),    # magenta
    (0, 255, 255),    # yellow
    (255, 255, 0),    # cyan
    (255, 0, 0),      # blue
]


def _parse_hsv_triple(value: str):
    parts = value.split(',')
    if len(parts) != 3:
        raise argparse.ArgumentTypeError(f"expected 'H,S,V', got {value!r}")
    try:
        return tuple(int(p) for p in parts)
    except ValueError:
        raise argparse.ArgumentTypeError(f"expected three integers 'H,S,V', got {value!r}")


def parse_object(value: str):
    """argparse type for --object: "name:Hlo,Slo,Vlo:Hhi,Shi,Vhi" ->
    (name, hsv_low, hsv_high). Used with action='append' so --object can
    be given multiple times, one per target to detect simultaneously."""
    parts = value.split(':')
    if len(parts) != 3:
        raise argparse.ArgumentTypeError(
            f"expected 'name:H,S,V:H,S,V', got {value!r}")
    name, low_s, high_s = parts
    name = name.strip()
    if not name:
        raise argparse.ArgumentTypeError(f"empty object name in {value!r}")
    return name, _parse_hsv_triple(low_s), _parse_hsv_triple(high_s)


class ObjectDetector(Node):
    def __init__(self, args):
        super().__init__('object_detect')
        self.args = args
        self.bridge = CvBridge()
        self.latest_depth = None   # raw depth Image msg, cached
        self.latest_info = None    # CameraInfo msg, cached
        self._last_log_t = {name: 0.0 for name, _, _ in args.objects}

        self.create_subscription(Image, args.depth_topic, self._on_depth, 5)
        self.create_subscription(CameraInfo, args.info_topic, self._on_info, 5)
        self.create_subscription(Image, args.color_topic, self._on_color, 5)

        self.pub_points = {
            name: self.create_publisher(PointStamped, f"{args.output_topic_prefix}/{name}", 5)
            for name, _, _ in args.objects
        }
        self.pub_debug = self.create_publisher(Image, args.debug_topic, 5)

        names = ', '.join(name for name, _, _ in args.objects)
        self.get_logger().info(
            f"Watching {args.color_topic} for objects: {names}. "
            f"Publishing detections under {args.output_topic_prefix}/<name>, "
            f"debug image to {args.debug_topic}.")

    def _on_depth(self, msg):
        self.latest_depth = msg

    def _on_info(self, msg):
        self.latest_info = msg

    def _on_color(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        debug = frame.copy()

        for i, (name, hsv_low, hsv_high) in enumerate(self.args.objects):
            detection, contour = find_target(frame, hsv_low, hsv_high)
            if detection is None:
                continue

            color = PALETTE[i % len(PALETTE)]
            draw_detection(debug, detection, contour, color, name)

            point = self._back_project(*detection)
            if point is not None:
                self._publish_point(name, point, msg.header)
                self._log_throttled(
                    name, f"{name}: pixel={detection} -> "
                    f"camera-frame xyz(m)=({point[0]:.3f}, {point[1]:.3f}, {point[2]:.3f})")
            else:
                self._log_throttled(
                    name, f"{name}: found in color image at {detection} but no "
                    f"valid depth there yet (missing depth/info, or a depth hole)")

        debug_msg = bgr8_to_imgmsg(debug, msg.header)
        self.pub_debug.publish(debug_msg)

    def _back_project(self, u, v):
        """Pixel (u,v) in the color image + the cached depth/intrinsics ->
        (x, y, z) in meters, in the camera's optical frame. Returns None if
        depth/camera_info aren't available yet, or the depth at/near this
        pixel is invalid (a common RealSense hole on edges/reflective
        surfaces)."""
        if self.latest_depth is None or self.latest_info is None:
            return None
        depth_img = self.bridge.imgmsg_to_cv2(self.latest_depth, desired_encoding='passthrough')
        return back_project(u, v, depth_img, self.latest_info.k, self.args.depth_scale)

    def _publish_point(self, name, point, header):
        msg = PointStamped()
        msg.header = header
        msg.point.x, msg.point.y, msg.point.z = point
        self.pub_points[name].publish(msg)

    def _log_throttled(self, name, text):
        now = time.monotonic()
        if now - self._last_log_t[name] >= LOG_THROTTLE_S:
            self.get_logger().info(text)
            self._last_log_t[name] = now


def find_target(frame, hsv_low, hsv_high):
    """Core per-object detection logic, pulled out of the ROS callback so
    it can be exercised standalone (e.g. against a synthetic test image)
    without rclpy/cv_bridge. Returns ((u, v), contour) for the largest
    blob above MIN_CONTOUR_AREA matching the given HSV range, or
    (None, None) if nothing matches."""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array(hsv_low), np.array(hsv_high))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None, None

    largest = max(contours, key=cv2.contourArea)
    if cv2.contourArea(largest) < MIN_CONTOUR_AREA:
        return None, None

    m = cv2.moments(largest)
    if m['m00'] <= 0:
        return None, None

    u = int(m['m10'] / m['m00'])
    v = int(m['m01'] / m['m00'])
    return (u, v), largest


def draw_detection(debug, detection, contour, color, label):
    """Draws one object's contour, centroid, and name label onto a shared
    debug frame — called once per detected object per frame so multiple
    targets can be shown together in one image."""
    cv2.drawContours(debug, [contour], -1, color, 2)
    cv2.circle(debug, detection, 6, color, -1)
    cv2.putText(debug, label, (detection[0] + 10, detection[1] - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)


def bgr8_to_imgmsg(frame, header):
    """Manual replacement for cv_bridge.CvBridge.cv2_to_imgmsg(frame,
    encoding='bgr8'). CONFIRMED ON HARDWARE 2026-07-23: cv_bridge's
    cv2_to_imgmsg throws `KeyError: 16` on this robot — cv2 here is a
    pip-installed opencv-python under ~/.local/lib/python3.8/
    site-packages/cv2 (confirmed via an earlier Qt-plugin error message),
    not the apt/ROS python3-opencv cv_bridge was built against, and their
    internal type-numbering doesn't match. Only this write direction
    (cv2 -> Image) is affected — imgmsg_to_cv2 (Image -> cv2), used
    elsewhere in this file for the color/depth reads, uses a different
    cv_bridge code path and works fine, so it's left as-is. This function
    just fills in the Image message fields directly, sidestepping
    cv_bridge's broken type lookup for bgr8 specifically."""
    msg = Image()
    msg.header = header
    msg.height, msg.width = frame.shape[:2]
    msg.encoding = 'bgr8'
    msg.is_bigendian = 0
    msg.step = frame.shape[1] * frame.shape[2]
    msg.data = frame.tobytes()
    return msg


def back_project(u, v, depth_img, k, depth_scale):
    """Pixel (u,v) + a depth image + camera intrinsics (row-major 3x3, as
    in sensor_msgs/CameraInfo.k) -> (x, y, z) in meters, pinhole model.
    Pulled out of ObjectDetector so it can be unit-tested standalone.
    Returns None if (u,v) is out of bounds or has no valid depth nearby."""
    h, w = depth_img.shape[:2]
    if not (0 <= v < h and 0 <= u < w):
        return None

    # Median over a small patch instead of one pixel — more robust to
    # single-pixel depth noise/holes right at the centroid.
    r = 3
    patch = depth_img[max(0, v - r):v + r + 1, max(0, u - r):u + r + 1].astype(np.float32)
    valid = patch[patch > 0]
    if valid.size == 0:
        return None
    z = float(np.median(valid)) * depth_scale

    fx, fy, cx, cy = k[0], k[4], k[2], k[5]
    x = (u - cx) * z / fx
    y = (v - cy) * z / fy
    return (x, y, z)


def run_calibrate(args):
    """Live HSV trackbar tuning window — needs ssh -X, same as
    rqt_plot/slider_control.py. Calibrates ONE object at a time (run this
    once per target). Move sliders until only that object shows up white
    in the mask window, then use those H/S/V values as one --object entry
    (name:low:high) for a real detection run."""
    rclpy.init()
    node = rclpy.create_node('object_detect_calibrate')
    bridge = CvBridge()
    state = {'frame': None}

    def on_color(msg):
        state['frame'] = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    node.create_subscription(Image, args.color_topic, on_color, 5)

    cv2.namedWindow('calibrate')
    for name, default, maxval in [
        ('H low', 0, 179), ('S low', 0, 255), ('V low', 0, 255),
        ('H high', 179, 179), ('S high', 255, 255), ('V high', 255, 255),
    ]:
        cv2.createTrackbar(name, 'calibrate', default, maxval, lambda _v: None)

    print("Move the sliders until only ONE target object is white in the "
          "mask window (calibrate one object at a time). Current H,S,V "
          "low/high print below on every frame, ready to use as an "
          "--object NAME:low:high entry. Press 'q' in the image window to quit.")

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            if state['frame'] is None:
                continue
            frame = state['frame']
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            low = tuple(cv2.getTrackbarPos(n, 'calibrate') for n in ('H low', 'S low', 'V low'))
            high = tuple(cv2.getTrackbarPos(n, 'calibrate') for n in ('H high', 'S high', 'V high'))
            mask = cv2.inRange(hsv, np.array(low), np.array(high))
            masked = cv2.bitwise_and(frame, frame, mask=mask)
            combined = np.hstack([frame, cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR), masked])
            cv2.imshow('calibrate', combined)
            low_s = ','.join(map(str, low))
            high_s = ','.join(map(str, high))
            print(f"\rlow={low} high={high}   (--object NAME:{low_s}:{high_s})",
                  end='', flush=True)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        print()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--calibrate', action='store_true',
                         help='live HSV trackbar tuning window instead of running detection '
                              '(one object at a time)')
    parser.add_argument('--object', dest='objects', action='append', type=parse_object,
                         default=[],
                         help='required (at least one) unless --calibrate: '
                              '"name:H,S,V:H,S,V" — repeat for multiple objects, e.g. '
                              '--object glove:100,80,80:130,255,255 '
                              '--object teal_ball:85,100,100:100,255,255. '
                              'No default color range is provided on purpose (see module '
                              'docstring) — use --calibrate to find real values.')
    parser.add_argument('--color-topic', type=str, default='/camera/camera/color/image_raw',
                         help='confirmed via ros2 topic list 2026-07-23')
    parser.add_argument('--depth-topic', type=str,
                         default='/camera/camera/aligned_depth_to_color/image_raw',
                         help='GUESSED name, NOT confirmed to exist — the 2026-07-23 topic '
                              'list only had raw (unaligned) depth; this assumes an aligned '
                              'topic appears once align_depth is enabled at launch (see '
                              'module docstring). Override once the real name is confirmed, '
                              'or pass /camera/camera/depth/image_rect_raw to get real '
                              '(unaligned, slightly offset) depth right now.')
    parser.add_argument('--info-topic', type=str, default='/camera/camera/color/camera_info',
                         help='confirmed via ros2 topic list 2026-07-23')
    parser.add_argument('--depth-scale', type=float, default=0.001,
                         help='multiply raw depth units by this to get meters (default '
                              'assumes 16-bit unsigned millimeters, the realsense2_camera '
                              'default — verify once real data is flowing)')
    parser.add_argument('--output-topic-prefix', type=str, default='/detected_object_position',
                         help='each --object NAME publishes its geometry_msgs/PointStamped on '
                              '{prefix}/{NAME}, e.g. /detected_object_position/teal_ball')
    parser.add_argument('--debug-topic', type=str, default='/object_detect/debug_image',
                         help='ONE shared annotated image with every detected object drawn '
                              'in its own color and labeled by name. View with: '
                              'ros2 run rqt_image_view rqt_image_view')
    args = parser.parse_args()

    if args.calibrate:
        run_calibrate(args)
        return

    if not args.objects:
        parser.error(
            "at least one --object is required unless --calibrate is given (see "
            "module docstring — no default color range is provided on purpose, since a "
            "guessed default could look like it's working while detecting nothing real)")

    rclpy.init()
    node = ObjectDetector(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
