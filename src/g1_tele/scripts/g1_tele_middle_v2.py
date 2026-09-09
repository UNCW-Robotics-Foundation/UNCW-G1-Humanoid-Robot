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
from geometry_msgs.msg import Point
from trajectory_msgs.msg import JointTrajectoryPoint
from unitree_hg.msg import LowCmd
from sensor_msgs.msg import Joy
from g1_msgs.msg import ArmStates
from g1_msgs.msg import MotorState
import pinocchio as pin
import numpy as np

from ruckig import InputParameter, OutputParameter, Result, Ruckig

"""
16-22 left arm joints
39-45 right arm joints
"""

class RuckigJointTrajectoryNode(Node):
    def __init__(self):
        super().__init__('ruckig_joint_trajectory_node')

        # ---- Parameters (tune to your joint's actual limits) ----
        self.declare_parameter('joint_name', 'joint_1')
        self.declare_parameter('control_frequency_hz', 200.0)
        self.declare_parameter('max_velocity', [0.03, 0.03, 0.03])        # rad/s
        self.declare_parameter('max_acceleration', [0.06, 0.06, 0.06])    # rad/s^2
        self.declare_parameter('max_jerk', [2.0, 2.0, 2.0])             # rad/s^3

        self.joint_name = self.get_parameter('joint_name').value
        control_freq = self.get_parameter('control_frequency_hz').value
        self.dt = 1.0 / control_freq

        urdf_path = "/home/temo/ik_ws/UNCW-G1-Humanoid-Robot/src/g1_ik/assets/g1_v2/g1_29dof_mode_15_brainco_hand.urdf"
        self.model = pin.buildModelFromUrdf(urdf_path)
        self.data = self.model.createData()
        self.model.gravity.linear = np.array([0.0, 0.0, -9.81])
        self.q = np.zeros(self.model.nq)
        self.v = np.zeros(self.model.nv)
        self.a = np.zeros(self.model.nv)

        self.otg = Ruckig(7, self.dt)
        self.inp = InputParameter(7)
        self.out = OutputParameter(7)

        self.inp.current_position = [0.0] * 7
        self.inp.current_velocity = [0.0] * 7
        self.inp.current_acceleration = [0.0] * 7

        self.inp.target_position = [0.0] * 7
        self.inp.target_velocity = [0.0] * 7
        self.inp.target_acceleration = [0.0] * 7

        self.inp.max_velocity = [0.1] * 7
        self.inp.max_acceleration = [0.2] * 7
        self.inp.max_jerk = [3.0] * 7

        self._has_target = False
        self.record_arms = False

        self.target_x = 0.0
        self.target_y = 0.0

        # ---- ROS interfaces ----
        self.target_sub = self.create_subscription(
            ArmStates, 'joint/ik_sol', self.target_callback, 10)

        self.current_arms_sub = self.create_subscription(
                    ArmStates, 'arm_joints', self.current_arms_callback, 10)

        # self.joy_sub = self.create_subscription(
        #             Joy, 'joy', self.joy_callback, 10)

        self.point_pub = self.create_publisher(
            ArmStates, 'ik_sol', 10)

        self.timer = self.create_timer(self.dt, self.update_loop)

        self.get_logger().info(
            f"Ruckig trajectory generator running for '{self.joint_name}' "
            f"at {control_freq:.1f} Hz")

    def current_arms_callback(self, msg: ArmStates):
        tmp_arms = msg
        if not self.record_arms:
            self.record_arms = True
            self.inp.current_velocity = [0.0] * 7
            self.inp.current_acceleration = [0.0] * 7
            for i in range(7):
                self.inp.current_position[i] = tmp_arms.motor_states[i].q
            self.get_logger().info(f'Current arms recorded')


    def target_callback(self, msg: ArmStates):
        tmp_target = msg
        self.inp.target_position = [q.q for q in tmp_target.motor_states[:7]]
        #for i in range(7):
            #self.get_logger().info(f'Target {i} = {tmp_target.motor_states[i].q}')
            # self.inp.target_position[i] = tmp_target.motor_states[i].q
            # self.get_logger().info(f'Target {i} = {self.inp.target_position[i]}')

        self.inp.target_velocity = [0.0] * 7
        self.inp.target_acceleration = [0.0] * 7
        self._has_target = True
        #self.get_logger().info(f'Target Recieved')
        #self.get_logger().info(f'New target position: x={msg.x:.4f}, y={msg.y:.4f}, z={msg.z:.4f}')

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
        if (not self._has_target) or (not self.record_arms):
            return

        result = self.otg.update(self.inp, self.out)

        tmp_target = []
        tmp_list = list(self.out.new_position) + [0.0] * 7
        tmp_list_v = list(self.out.new_velocity)
        tmp_list_a = list(self.out.new_acceleration)
        for i in range(7):
            self.q[i + 16] = tmp_list[i]
            self.v[i + 16] = tmp_list_v[i]
            self.a[i + 16] = tmp_list_a[i]

        tau_ff = pin.rnea(self.model, self.data, self.q, self.v, self.a)
        target = ArmStates()
        for i in range(14):
            tmp_motor = MotorState()
            if i < 7:
                tmp_motor.q = tmp_list[i]
                tmp_motor.dq = tau_ff[i + 16]
            else:
                tmp_motor.q = 0.0
                tmp_motor.dq = 0.0
            tmp_target.append(tmp_motor)
        target.motor_states = tmp_target
        self.point_pub.publish(target)

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
            self.record_arms = False
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