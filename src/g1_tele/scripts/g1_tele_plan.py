#!/usr/bin/env python3
"""
Single-joint jerk-limited trajectory generator using Ruckig, integrated with ROS 2.

Subscribes to a target position on 'joint/target_position' (std_msgs/Float64)
and streams jerk-limited trajectory points on 'joint/trajectory_point'
(trajectory_msgs/JointTrajectoryPoint) at a fixed control-loop rate.

Install ruckig first:
    pip install ruckig

Run:
    ros2 run <your_package> ruckig_joint_trajectory_node
    # or directly:
    python3 ruckig_joint_trajectory_node.py

Send a target:
    ros2 topic pub --once /joint/target_position geometry_msgs/msg/Point "{x: 0.1, y: 0.0, z: 0.0}"
"""

import rclpy
from rclpy.node import Node
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from tf2_ros import TransformException
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, Bool
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import Joy
from g1_msgs.msg import G1Plan, G1Point

from ruckig import InputParameter, OutputParameter, Result, Ruckig


class RuckigJointTrajectoryNode(Node):
    def __init__(self):
        super().__init__('ruckig_joint_trajectory_node')

        # ---- Parameters (tune to your joint's actual limits) ----
        self.declare_parameter('joint_name', 'joint_1')
        self.declare_parameter('control_frequency_hz', 10.0)
        self.declare_parameter('max_velocity', [0.03, 0.03, 0.03])        # rad/s
        self.declare_parameter('max_acceleration', [0.06, 0.06, 0.06])    # rad/s^2
        self.declare_parameter('max_jerk', [2.0, 2.0, 2.0])             # rad/s^3

        self.joint_name = self.get_parameter('joint_name').value
        control_freq = self.get_parameter('control_frequency_hz').value
        self.dt = 1.0 / control_freq

        # ---- Ruckig setup: 1 degree of freedom ----
        self.otg = Ruckig(3, self.dt)
        self.inp = InputParameter(3)
        self.out = OutputParameter(3)

        self.inp.current_position = [0.0, 0.0, 0.0]
        self.inp.current_velocity = [0.0, 0.0, 0.0]
        self.inp.current_acceleration = [0.0, 0.0, 0.0]

        self.inp.target_position = [0.0, 0.0, 0.0]
        self.inp.target_velocity = [0.0, 0.0, 0.0]
        self.inp.target_acceleration = [0.0, 0.0, 0.0]

        self.inp.max_velocity = list(self.get_parameter('max_velocity').value)
        self.inp.max_acceleration = list(self.get_parameter('max_acceleration').value)
        self.inp.max_jerk = list(self.get_parameter('max_jerk').value)

        self._has_plan = False
        self._init_frame = True
        self._ruckig_state = False

        self.plan = []
        self.plan_msg = G1Plan()
        self.current_point = 0

        # ---- ROS interfaces ----
        self.point_sub = self.create_subscription(
            PointStamped, 'clicked_point', self.target_callback, 10)

        self.joy_sub = self.create_subscription(
            Joy, 'joy', self.joy_callback, 10)

        self.ruckig_sub = self.create_subscription(
            Bool, 'ruckig_state', self.ruckig_callback, 10)

        self.point_pub = self.create_publisher(
            JointTrajectoryPoint, 'joint/trajectory_point', 10)

        self.dbg_plan_pub = self.create_publisher(
            PointCloud2, 'joint/plan_dbg', 10)

        self.plan_pub = self.create_publisher(
            G1Plan, 'joint/plan', 10)

        self.timer = self.create_timer(self.dt, self.update_loop)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info(
            f"Ruckig trajectory generator running for '{self.joint_name}' "
            f"at {control_freq:.1f} Hz. Waiting fr points...")

    def target_callback(self, msg: PointStamped):
        self.plan.append([msg.point.x, msg.point.y, msg.point.z + .15])
        tmp_point = G1Point()
        tmp_point.x = msg.point.x
        tmp_point.y = msg.point.y
        tmp_point.z = msg.point.z + .15
        self.plan_msg.plan.append(tmp_point)
        # self.inp.target_position = [msg.x, msg.y, msg.z]
        # self.inp.target_velocity = [0.0, 0.0, 0.0]
        # self.inp.target_acceleration = [0.0, 0.0, 0.0]
        self.get_logger().info(f'New plan point: x={msg.point.x:.4f}, y={msg.point.y:.4f}, z={msg.point.z+.15:.4f}')

    def joy_callback(self, msg: Joy):
        if (msg.buttons[6] == 1) and (not self._has_plan):
            # self.init_x = self.current_pen_x
            # self.init_y = self.current_pen_y
            # self.init_z = self.current_pen_z
            self.inp.target_position = [self.plan[self.current_point][0], self.plan[self.current_point][1], self.plan[self.current_point][2]]
            self.inp.target_velocity = [0.0, 0.0, 0.0]
            self.inp.target_acceleration = [0.0, 0.0, 0.0]
            self._has_plan = True
            self.get_logger().info(f"Plan Started! ")
        if (msg.buttons[4] == 1):
            self.plan_pub.publish(self.plan_msg)

    def ruckig_callback(self, msg: Bool):
        self._ruckig_state = msg.data

    def update_loop(self):
        try:
            if self._init_frame:
                t = self.tf_buffer.lookup_transform(
                    'pelvis',
                    'left_ee',
                    rclpy.time.Time())

                self._init_frame = False
                self.inp.current_position = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
                self.get_logger().info(f"Wrist Frame Recorded!")
            else:
                t = self.tf_buffer.lookup_transform(
                    'pelvis',
                    'left_ee',
                    rclpy.time.Time())

        except TransformException as ex:
            self.get_logger().info(
                f'Could not find transform: {ex}')
            return
        tmp_header = Header()
        tmp_header.stamp = self.get_clock().now().to_msg()
        tmp_header.frame_id = 'pelvis'
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]
        pc2_msg = point_cloud2.create_cloud(tmp_header, fields, self.plan)
        self.dbg_plan_pub.publish(pc2_msg)
        if not self._has_plan:
            return

        result = self.otg.update(self.inp, self.out)

        point = JointTrajectoryPoint()
        point.positions = list(self.out.new_position)
        point.velocities = list(self.out.new_velocity)
        point.accelerations = list(self.out.new_acceleration)
        self.point_pub.publish(point)

        # Feed this cycle's output back in as next cycle's current state.
        # This is the standard Ruckig "online" pattern.
        self.out.pass_to_input(self.inp)

        if result == Result.Finished and self.current_point >= len(self.plan) - 1:
            self._has_plan = False
            self.get_logger().info(f'Point {self.current_point} completed.')
            self.log_target_errors(t)
            self.get_logger().info('Plan finished.')
        elif result == Result.Finished and self.current_point < len(self.plan) - 1 and not self._ruckig_state:
            self.get_logger().info(f'Point {self.current_point} completed.')
            self.log_target_errors(t)
            self.current_point += 1
            self.inp.target_position = [self.plan[self.current_point][0], self.plan[self.current_point][1], self.plan[self.current_point][2]]
            self.inp.target_velocity = [0.0, 0.0, 0.0]
            self.inp.target_acceleration = [0.0, 0.0, 0.0]
        elif result == Result.Finished and self.current_point < len(self.plan) - 1 and self._ruckig_state:
            self.get_logger().info(f'Waiting on main ruckig...')
        elif result == Result.Error:
            self.get_logger().error(
                'Ruckig returned an error; check limits and target values.')

    def log_target_errors(self, t):
        tmp_dx = abs(self.plan[self.current_point][0] - t.transform.translation.x)
        tmp_dy = abs(self.plan[self.current_point][1] - t.transform.translation.y)
        tmp_dz = abs(self.plan[self.current_point][2] - t.transform.translation.z)
        self.get_logger().info(f'Error deltas: {tmp_dx} {tmp_dy} {tmp_dz}')


def main(args=None):
    rclpy.init(args=args)
    node = RuckigJointTrajectoryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()