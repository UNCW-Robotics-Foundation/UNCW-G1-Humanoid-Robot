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
    ros2 topic pub --once /joint/target_position geometry_msgs/msg/Point "{x: 0.3, y: -0.1, z: 0.5}"
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from trajectory_msgs.msg import JointTrajectoryPoint
from unitree_hg.msg import LowCmd
from sensor_msgs.msg import Joy

from ruckig import InputParameter, OutputParameter, Result, Ruckig


class RuckigJointTrajectoryNode(Node):
    def __init__(self):
        super().__init__('ruckig_joint_trajectory_node')

        # ---- Parameters (tune to your joint's actual limits) ----
        self.declare_parameter('joint_name', 'joint_1')
        self.declare_parameter('control_frequency_hz', 30.0)
        self.declare_parameter('max_velocity', [0.1, 0.1, 0.1])        # rad/s
        self.declare_parameter('max_acceleration', [0.2, 0.2, 0.2])    # rad/s^2
        self.declare_parameter('max_jerk', [3.0, 3.0, 3.0])             # rad/s^3

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

        self._has_target = False

        self.target_x = 0.0
        self.target_y = 0.0

        # ---- ROS interfaces ----
        self.target_sub = self.create_subscription(
            Point, 'joint/target_position', self.target_callback, 10)

        # self.joy_sub = self.create_subscription(
        #             Joy, 'joy', self.joy_callback, 10)

        self.point_pub = self.create_publisher(
            JointTrajectoryPoint, 'joint/trajectory_point', 10)

        self.cmd_pub = self.create_publisher(
                    LowCmd, 'joint/cmd', 10)

        self.timer = self.create_timer(self.dt, self.update_loop)

        self.get_logger().info(
            f"Ruckig trajectory generator running for '{self.joint_name}' "
            f"at {control_freq:.1f} Hz")

    def target_callback(self, msg: Point):
        # Update the target; Ruckig will replan from the current in-flight
        # state on the very next update() call, so this is safe mid-motion.
        self.inp.target_position = [msg.x, msg.y, msg.z]
        self.inp.target_velocity = [0.0, 0.0, 0.0]
        self.inp.target_acceleration = [0.0, 0.0, 0.0]
        self._has_target = True
        self.get_logger().info(f'New target position: x={msg.x:.4f}, y={msg.y:.4f}, z={msg.z:.4f}')

    def joy_callback(self, msg: Joy):
        # Update the target; Ruckig will replan from the current in-flight
        # state on the very next update() call, so this is safe mid-motion.
        self.target_x += msg.axes[1] * 0.002
        self.target_y += msg.axes[0] * 0.002
        self.inp.target_position = [self.target_x, self.target_y, 0.0]
        self.inp.target_velocity = [0.0, 0.0, 0.0]
        self.inp.target_acceleration = [0.0, 0.0, 0.0]
        self._has_target = True
        #self.get_logger().info(f'New target position: x={msg.x:.4f}, y={msg.y:.4f}, z={msg.z:.4f}')

    def update_loop(self):
        if not self._has_target:
            return

        result = self.otg.update(self.inp, self.out)

        point = JointTrajectoryPoint()
        point.positions = list(self.out.new_position)
        point.velocities = list(self.out.new_velocity)
        point.accelerations = list(self.out.new_acceleration)
        self.point_pub.publish(point)

        # cmd = LowCmd()
        # cmd.mode_machine = 5
        # for i in range(29):
        #     if i == 15:
        #         cmd.motor_cmd[i].q = self.out.new_position[0]
        #         cmd.motor_cmd[i].dq = self.out.new_velocity[0]
        #     else :
        #         cmd.motor_cmd[i].dq = 0.0
        #         cmd.motor_cmd[i].dq = 0.0
        #     cmd.motor_cmd[i].tau = 0.0
        #     cmd.motor_cmd[i].kp = 60.0
        #     cmd.motor_cmd[i].kd = 1.5
        #     cmd.motor_cmd[i].mode = 1
        # self.cmd_pub.publish(cmd)

        # Feed this cycle's output back in as next cycle's current state.
        # This is the standard Ruckig "online" pattern.
        self.out.pass_to_input(self.inp)

        if result == Result.Finished:
            self._has_target = False
            self.get_logger().info('Trajectory finished.')
        elif result == Result.Error:
            self.get_logger().error(
                'Ruckig returned an error; check limits and target values.')


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