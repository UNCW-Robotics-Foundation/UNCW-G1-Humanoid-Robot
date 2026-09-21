#!/usr/bin/env python3
"""
Geomagic Touch pen node that reads pen pose and publishes it to a topic that the inverse
kinematics solver will use. Since main node is de-coupled with a Ruckig generator, robot movements
may lag behind pen movements depending on main node speed limits.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import Joy


class G1TelePenNode(Node):
    def __init__(self):
        super().__init__('g1_tele_pen_node')

        self.dt = 1.0 / 10.0

        self.tele_start = False

        self.current_pen_x = 0.0
        self.current_pen_y = 0.0
        self.current_pen_z = 0.0
        self.init_x = 0.0
        self.init_y = 0.0
        self.init_z = 0.0

        # ---- ROS interfaces ----
        self.target_sub = self.create_subscription(
            PoseStamped, 'phantom/pose', self.pen_callback, 10)

        self.joy_sub = self.create_subscription(
                    Joy, 'joy', self.joy_callback, 10)

        self.point_pub = self.create_publisher(
            JointTrajectoryPoint, 'joint/trajectory_point', 10)

        self.timer = self.create_timer(self.dt, self.update_loop)

    def pen_callback(self, msg: PoseStamped):
        self.current_pen_x = msg.pose.position.x
        self.current_pen_y = msg.pose.position.y
        self.current_pen_z = msg.pose.position.z

    def joy_callback(self, msg: Joy):
        if (msg.buttons[6] == 1) and (not self.tele_start): # Start (3 lines)
            self.init_x = self.current_pen_x
            self.init_y = self.current_pen_y
            self.init_z = self.current_pen_z
            self.tele_start = True
            self.get_logger().info(f"Pen Teleoperation Enabled! ")

    def update_loop(self):
        if not self.tele_start:
            return

        point = JointTrajectoryPoint()
        point.positions = [(self.current_pen_y - self.init_y), (self.current_pen_x - self.init_x) * -1, (self.current_pen_z - self.init_z)]
        self.point_pub.publish(point)


def main(args=None):
    rclpy.init(args=args)
    node = G1TelePenNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()