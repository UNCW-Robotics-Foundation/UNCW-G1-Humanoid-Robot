import math

import rclpy

from rclpy.node import Node
from sensor_msgs.msg import Imu
from rclpy.qos import qos_profile_sensor_data


class ImuFrameflip(Node):
    """Rectifies the head IMU into the fixed lidar-body frame.

    The MID360 (and its internal IMU) is mounted in the G1's head upside
    down AND pitched 23 deg nose-down. This applies the fixed rotation

        v_out = R_y(23 deg) * R_roll180 * v_raw

    Rigid geometry lives in three places that must all agree — here, in
    fast_lio_mid360.yaml (extrinsic_R/T), and in the URDF mid360_joint.
    Do not tune one alone. Startup lean/tilt is NOT handled here: the
    floor_level_anchor node absorbs it in the odom->lio_odom TF.
    """

    def __init__(self):
        super().__init__('imu_frameflip')
        self.declare_parameter('pitch_down_deg', 23.0)
        b = math.radians(self.get_parameter('pitch_down_deg').value)
        c, s = math.cos(b), math.sin(b)
        # R_y(b) @ diag(1,-1,-1), rows of the combined matrix:
        self.rot = ((c, 0.0, -s),
                    (0.0, -1.0, 0.0),
                    (-s, 0.0, -c))

        # Unitree publishes sensor data best-effort; the default reliable
        # QoS would silently receive nothing.
        self.sub = self.create_subscription(
            Imu,
            '/utlidar/imu_livox_mid360',
            self.imu_callback,
            qos_profile_sensor_data)
        self.pub = self.create_publisher(Imu, '/imu/upright', 10)
        self.get_logger().info(
            'imu_frameflip up: %.1f deg fixed correction' % math.degrees(b))

    def rotate(self, v):
        r = self.rot
        x = r[0][0] * v.x + r[0][1] * v.y + r[0][2] * v.z
        y = r[1][0] * v.x + r[1][1] * v.y + r[1][2] * v.z
        z = r[2][0] * v.x + r[2][1] * v.y + r[2][2] * v.z
        v.x, v.y, v.z = x, y, z

    def imu_callback(self, msg):
        # Timestamp stays untouched — FAST_LIO needs original sensor time.
        self.rotate(msg.linear_acceleration)
        self.rotate(msg.angular_velocity)
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = ImuFrameflip()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
