#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from tf2_ros import TransformException
from sensor_msgs.msg import Joy
from g1_msgs.msg import G1Plan, G1Point
import csv


class PathToCsvNode(Node):
    def __init__(self):
        super().__init__('path_to_csv_node')

        # ---- Parameters (tune to your joint's actual limits) ----
        self.declare_parameter('control_frequency_hz', 10.0)

        control_freq = self.get_parameter('control_frequency_hz').value
        self.dt = 1.0 / control_freq

        self.start_flag = False
        self.stop_flag = False
        self.field_names = ["x", "y", "z"]
        self.data = []

        # ---- ROS interfaces ----

        self.joy_sub = self.create_subscription(
            Joy, 'joy', self.joy_callback, 10)

        self.timer = self.create_timer(self.dt, self.update_loop)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def joy_callback(self, msg: Joy):
        if (msg.buttons[9] == 1) and (not self.start_flag):
            self.start_flag = True
            self.get_logger().info(f"Recording Data...")
        if (msg.buttons[10] == 1 and (not self.stop_flag)):
            self.start_flag = False
            self.stop_flag = True
            self.get_logger().info(f"Recording Stopped! ")

    def update_loop(self):
        try:
            t_ee = self.tf_buffer.lookup_transform(
                'pelvis',
                'left_ee',
                rclpy.time.Time())

        except TransformException as ex:
            self.get_logger().info(
                f'Could not find transform: {ex}')
            return

        if self.start_flag:
            self.data.append([t_ee.transform.translation.x, t_ee.transform.translation.y, t_ee.transform.translation.z])
        elif self.stop_flag:
            self.get_logger().info(f"Writing to file... ")
            with open("g1_path.csv", "w", newline="", encoding="utf-8") as file:
                writer = csv.writer(file)
                writer.writerow(self.field_names)
                writer.writerows(self.data)
            self.get_logger().info(f"Writing Complete! ")
            self.stop_flag = False


    def log_target_errors(self, t_ee, t_w):
        tmp_dx = abs(self.plan[self.current_point][0] - t_ee.transform.translation.x)
        tmp_dy = abs(self.plan[self.current_point][1] - t_ee.transform.translation.y)
        tmp_dz = abs(self.plan[self.current_point][2] - t_ee.transform.translation.z)
        self.get_logger().info(f'Pin end Effector to target error deltas: {tmp_dx} {tmp_dy} {tmp_dz}')

        tmp_dx = abs(self.plan[self.current_point][0] - t_w.transform.translation.x)
        tmp_dy = abs(self.plan[self.current_point][1] - t_w.transform.translation.y)
        tmp_dz = abs(self.plan[self.current_point][2] - t_w.transform.translation.z)
        self.get_logger().info(f'TF wrist to target error deltas: {tmp_dx} {tmp_dy} {tmp_dz}')

        tmp_dx = abs(t_w.transform.translation.x - t_ee.transform.translation.x)
        tmp_dy = abs(t_w.transform.translation.y - t_ee.transform.translation.y)
        tmp_dz = abs(t_w.transform.translation.z - t_ee.transform.translation.z)
        self.get_logger().info(f'Pin ee to TF w error deltas: {tmp_dx} {tmp_dy} {tmp_dz}')

    def check_point(self, t_ee):
        tmp_dx = abs(self.plan[self.current_point][0] - t_ee.transform.translation.x)
        tmp_dy = abs(self.plan[self.current_point][1] - t_ee.transform.translation.y)
        tmp_dz = abs(self.plan[self.current_point][2] - t_ee.transform.translation.z)

        if (tmp_dx < 0.004) and (tmp_dy < 0.004) and (tmp_dz < 0.005):
            self._close_to_point = True
        else:
            self._close_to_point = False


def main(args=None):
    rclpy.init(args=args)
    node = PathToCsvNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()