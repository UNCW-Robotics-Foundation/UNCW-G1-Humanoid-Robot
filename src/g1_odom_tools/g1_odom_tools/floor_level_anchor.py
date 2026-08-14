import math

import numpy as np
import rclpy

from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster


def quat_to_z(v):
    """Quaternion (x,y,z,w) of the smallest rotation taking v to +z."""
    n = math.sqrt(v[0]**2 + v[1]**2 + v[2]**2)
    ux, uy, uz = v[0] / n, v[1] / n, v[2] / n
    ax, ay = uy, -ux
    sin_t = math.sqrt(ax * ax + ay * ay)
    cos_t = uz
    if sin_t < 1e-9:
        return (0.0, 0.0, 0.0, 1.0)
    theta = math.atan2(sin_t, cos_t)
    s2 = math.sin(theta / 2.0)
    return (ax / sin_t * s2, ay / sin_t * s2, 0.0, math.cos(theta / 2.0))


class FloorLevelAnchor(Node):
    """Anchors FAST_LIO's world (lio_odom) to a floor-level odom frame.

    Gravity-based leveling is limited by accelerometer bias (1-2 deg of
    fake tilt), so this measures the thing we actually care about: the
    floor. It collects a few /cloud_registered scans (lio_odom frame),
    keeps points in a height band where the floor must be (the lidar
    starts ~1.2 m above it), fits a plane by least squares with
    iterative outlier rejection, and publishes the static
    odom->lio_odom transform that maps that plane to exactly z=0:

        rotation    = plane normal -> +z
        translation = -(normal . centroid)  (= measured lidar height)

    By construction the floor is level and at z=0 in odom. Runs once at
    startup (before Nav2 launches at t=25s) and then stops — the anchor
    must never jump mid-run. Falls back to identity + z_fallback if no
    acceptable plane is found in time.
    """

    def __init__(self):
        super().__init__('floor_level_anchor')
        self.declare_parameter('scans_to_collect', 5)
        self.declare_parameter('floor_z_min', -1.6)   # band below lidar
        self.declare_parameter('floor_z_max', -0.8)
        self.declare_parameter('radius_max', 4.0)
        self.declare_parameter('dist_thresh', 0.04)   # plane inlier dist
        self.declare_parameter('min_inliers', 400)
        self.declare_parameter('max_tilt_deg', 15.0)
        self.declare_parameter('fallback_after_sec', 20.0)
        self.declare_parameter('z_fallback', 1.31)

        self.scans_to_collect = int(self.get_parameter('scans_to_collect').value)
        self.floor_z_min = self.get_parameter('floor_z_min').value
        self.floor_z_max = self.get_parameter('floor_z_max').value
        self.radius_max = self.get_parameter('radius_max').value
        self.dist_thresh = self.get_parameter('dist_thresh').value
        self.min_inliers = int(self.get_parameter('min_inliers').value)
        self.max_tilt_deg = self.get_parameter('max_tilt_deg').value

        self.chunks = []
        self.scan_count = 0
        self.done = False

        self.static_tf = StaticTransformBroadcaster(self)
        self.sub = self.create_subscription(
            PointCloud2, '/cloud_registered', self.cloud_callback, 10)
        self.fallback_timer = self.create_timer(
            self.get_parameter('fallback_after_sec').value, self.fallback)
        self.get_logger().info('floor_level_anchor up, waiting for /cloud_registered...')

    def publish_anchor(self, q, tz):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'lio_odom'
        t.transform.translation.z = float(tz)
        t.transform.rotation.x = float(q[0])
        t.transform.rotation.y = float(q[1])
        t.transform.rotation.z = float(q[2])
        t.transform.rotation.w = float(q[3])
        self.static_tf.sendTransform(t)
        self.done = True
        self.fallback_timer.cancel()
        self.destroy_subscription(self.sub)

    def cloud_callback(self, msg):
        if self.done:
            return
        arr = point_cloud2.read_points(
            msg, field_names=('x', 'y', 'z'), skip_nans=True)
        pts = np.column_stack((arr['x'], arr['y'], arr['z']))
        if pts.shape[0] == 0:
            return
        r2 = pts[:, 0]**2 + pts[:, 1]**2
        band = ((pts[:, 2] > self.floor_z_min) & (pts[:, 2] < self.floor_z_max)
                & (r2 < self.radius_max**2))
        self.chunks.append(pts[band])
        self.scan_count += 1
        if self.scan_count >= self.scans_to_collect:
            self.try_fit()

    def restart_collection(self, why):
        self.get_logger().warn('floor fit rejected (%s), retrying...' % why)
        self.chunks = []
        self.scan_count = 0

    def try_fit(self):
        pts = np.vstack(self.chunks)
        if pts.shape[0] < self.min_inliers:
            self.restart_collection('only %d candidate points' % pts.shape[0])
            return

        # Least-squares plane with iterative outlier rejection: normal =
        # eigenvector of the smallest eigenvalue of the covariance.
        for _ in range(4):
            centroid = pts.mean(axis=0)
            q = pts - centroid
            _, vecs = np.linalg.eigh(q.T @ q)
            normal = vecs[:, 0]
            if normal[2] < 0:
                normal = -normal
            dist = q @ normal
            keep = np.abs(dist) < self.dist_thresh
            if keep.sum() < self.min_inliers:
                self.restart_collection('only %d inliers' % keep.sum())
                return
            pts = pts[keep]

        tilt = math.degrees(math.acos(min(1.0, max(-1.0, normal[2]))))
        if tilt > self.max_tilt_deg:
            self.restart_collection('implausible tilt %.1f deg' % tilt)
            return
        # After leveling, the plane sits at z = normal . centroid, so
        # lifting by its negation puts the floor at exactly 0. That lift
        # is also the lidar's true height above the floor.
        height = -float(normal @ centroid)
        if not 0.8 < height < 1.6:
            self.restart_collection('implausible lidar height %.2f m' % height)
            return

        self.publish_anchor(quat_to_z(normal), height)
        self.get_logger().info(
            'floor locked: tilt %.2f deg, lidar %.3f m above floor, '
            '%d inliers — floor is now z=0 in odom' %
            (tilt, height, pts.shape[0]))

    def fallback(self):
        self.fallback_timer.cancel()
        if self.done:
            return
        self.publish_anchor((0.0, 0.0, 0.0, 1.0),
                            self.get_parameter('z_fallback').value)
        self.get_logger().error(
            'no acceptable floor plane found — using identity anchor at '
            'z=%.2f. Expect tilt; check /cloud_registered.' %
            self.get_parameter('z_fallback').value)


def main():
    rclpy.init()
    node = FloorLevelAnchor()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
