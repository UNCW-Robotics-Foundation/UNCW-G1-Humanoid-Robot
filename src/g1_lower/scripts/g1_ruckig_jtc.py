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
    ros2 topic pub --once /joint/target_position std_msgs/msg/Float64 "{data: 1.57}"
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from trajectory_msgs.msg import JointTrajectoryPoint
from unitree_hg.msg import LowCmd

from ruckig import InputParameter, OutputParameter, Result, Ruckig


class RuckigJointTrajectoryNode(Node):
    def __init__(self):
        super().__init__('ruckig_joint_trajectory_node')

        # ---- Parameters (tune to your joint's actual limits) ----
        self.declare_parameter('joint_name', 'joint_1')
        self.declare_parameter('control_frequency_hz', 100.0)
        self.declare_parameter('max_velocity', 1.0)        # rad/s
        self.declare_parameter('max_acceleration', 2.0)    # rad/s^2
        self.declare_parameter('max_jerk', 5.0)             # rad/s^3

        self.joint_name = self.get_parameter('joint_name').value
        control_freq = self.get_parameter('control_frequency_hz').value
        self.dt = 1.0 / control_freq

        # ---- Ruckig setup: 1 degree of freedom ----
        self.otg = Ruckig(1, self.dt)
        self.inp = InputParameter(1)
        self.out = OutputParameter(1)

        self.inp.current_position = [0.0]
        self.inp.current_velocity = [0.0]
        self.inp.current_acceleration = [0.0]

        self.inp.target_position = [0.0]
        self.inp.target_velocity = [0.0]
        self.inp.target_acceleration = [0.0]

        self.inp.max_velocity = [self.get_parameter('max_velocity').value]
        self.inp.max_acceleration = [self.get_parameter('max_acceleration').value]
        self.inp.max_jerk = [self.get_parameter('max_jerk').value]

        self._has_target = False

        # ---- ROS interfaces ----
        self.target_sub = self.create_subscription(
            Float64, 'joint/target_position', self.target_callback, 10)

        self.point_pub = self.create_publisher(
            JointTrajectoryPoint, 'joint/trajectory_point', 10)

        self.cmd_pub = self.create_publisher(
                    LowCmd, 'joint/cmd', 10)

        self.timer = self.create_timer(self.dt, self.update_loop)

        self.get_logger().info(
            f"Ruckig trajectory generator running for '{self.joint_name}' "
            f"at {control_freq:.1f} Hz")

    def target_callback(self, msg: Float64):
        # Update the target; Ruckig will replan from the current in-flight
        # state on the very next update() call, so this is safe mid-motion.
        self.inp.target_position = [msg.data]
        self.inp.target_velocity = [0.0]
        self.inp.target_acceleration = [0.0]
        self._has_target = True
        self.get_logger().info(f'New target position: {msg.data:.4f} rad')

    def update_loop(self):
        if not self._has_target:
            return

        result = self.otg.update(self.inp, self.out)

        point = JointTrajectoryPoint()
        point.positions = [self.out.new_position[0]]
        point.velocities = [self.out.new_velocity[0]]
        point.accelerations = [self.out.new_acceleration[0]]
        self.point_pub.publish(point)

        cmd = LowCmd()
        cmd.mode_machine = 5
        for i in range(29):
            if i == 15:
                cmd.motor_cmd[i].q = self.out.new_position[0]
                cmd.motor_cmd[i].dq = self.out.new_velocity[0]
            else :
                cmd.motor_cmd[i].dq = 0.0
                cmd.motor_cmd[i].dq = 0.0
            cmd.motor_cmd[i].tau = 0.0
            cmd.motor_cmd[i].kp = 60.0
            cmd.motor_cmd[i].kd = 1.5
            cmd.motor_cmd[i].mode = 1
        self.cmd_pub.publish(cmd)

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