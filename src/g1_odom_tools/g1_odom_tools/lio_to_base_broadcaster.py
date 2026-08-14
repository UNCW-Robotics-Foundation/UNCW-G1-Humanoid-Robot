import rclpy

from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


def quat_rotate(q, v):
    """Rotate vector v = (x, y, z) by quaternion q = (x, y, z, w)."""
    qx, qy, qz, qw = q
    vx, vy, vz = v
    # t = 2 * (q_vec x v)
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    # v' = v + w * t + (q_vec x t)
    rx = vx + qw * tx + (qy * tz - qz * ty)
    ry = vy + qw * ty + (qz * tx - qx * tz)
    rz = vz + qw * tz + (qx * ty - qy * tx)
    return (rx, ry, rz)


class LioToBaseBroadcaster(Node):
    """Turns FAST_LIO's head-frame odometry into the odom->base_link TF.

    FAST_LIO tracks lio_body — the rectified IMU frame inside the G1's
    head, posed in lio_odom (FAST_LIO's world). Nav2 needs base_link
    (this URDF's ground-level floating base). This node composes the
    FAST_LIO pose with the fixed head->base offset and broadcasts
    base_link as a child of lio_odom:

        T_lio_odom_base = T_lio_odom_lio_body * T_lio_body_base

    imu_frameflip anchors lio_odom under a gravity-level odom frame
    (static TF with the measured startup tilt + z lift), so the chain
    odom -> lio_odom -> base_link comes out level with the floor near
    z=0. Also republishes the composed pose as Odometry on /odom.
    """

    def __init__(self):
        super().__init__('lio_to_base_broadcaster')

        # base_link origin expressed in the lio_body frame. This URDF's
        # base_link is a GROUND-LEVEL floating base (pelvis is fixed
        # 0.85 m above it); the lidar sits 1.31 m above base_link:
        # 0.85 (base->pelvis) + 0.044 (waist) + 0.416 (torso->mid360).
        self.declare_parameter('offset_x', 0.0)
        self.declare_parameter('offset_y', 0.0)
        self.declare_parameter('offset_z', -1.310)

        self.offset = (
            self.get_parameter('offset_x').value,
            self.get_parameter('offset_y').value,
            self.get_parameter('offset_z').value,
        )

        self.tf_broadcaster = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.sub = self.create_subscription(
            Odometry, '/Odometry', self.odom_callback, 10)

    def odom_callback(self, msg):
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        q = (o.x, o.y, o.z, o.w)

        # Carry the fixed lio_body->base_link offset into lio_odom.
        dx, dy, dz = quat_rotate(q, self.offset)
        bx = p.x + dx
        by = p.y + dy
        bz = p.z + dz

        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'lio_odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = bx
        t.transform.translation.y = by
        t.transform.translation.z = bz
        # No rotation between lio_body and base_link (both upright,
        # x forward) — reuse FAST_LIO's orientation as-is.
        t.transform.rotation = o
        self.tf_broadcaster.sendTransform(t)

        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = 'lio_odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = bx
        odom.pose.pose.position.y = by
        odom.pose.pose.position.z = bz
        odom.pose.pose.orientation = o
        odom.pose.covariance = msg.pose.covariance
        odom.twist = msg.twist
        self.odom_pub.publish(odom)


def main():
    rclpy.init()
    node = LioToBaseBroadcaster()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
