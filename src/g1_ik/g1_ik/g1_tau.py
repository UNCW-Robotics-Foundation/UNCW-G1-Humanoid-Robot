import rclpy
from rclpy.node import Node
from g1_msgs.msg import ArmStates, MotorState, RuckigData
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from tf2_ros import TransformException, TransformBroadcaster
import numpy as np
from g1_ik.robot_arm_ik_v2 import G1_29_ArmIK
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TransformStamped
from trajectory_msgs.msg import JointTrajectoryPoint
import logging_mp
from scipy.spatial.transform import Rotation as R
logger_mp = logging_mp.getLogger(__name__)

class TauGenerator(Node):

    def __init__(self):
        super().__init__('tau_generator')
        
        # self.subscription = self.create_subscription(
        #     ArmStates,
        #     'joints/ruckig',
        #     self.listener_callback,
        #     10)

        self.arm_sub = self.create_subscription(
            ArmStates,
            'arm_joints',
            self.listener_callback,
            10)

        self.ruckig_sub = self.create_subscription(
            RuckigData,
            'ruckig_future',
            self.ruckig_callback,
            10)

        self.tau_pub = self.create_publisher(
            ArmStates,
            'joints/tau',
            10)


        self.callback_flag = False
        self.callback_flag2 = False

        self.current_arms = ArmStates()
        self.arm_ik = G1_29_ArmIK()

        self.empty_arm = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.ruck_pos = []
        self.ruck_vel = []
        self.ruck_accel = []

        self.timer = self.create_timer(0.004, self.on_timers)

    def listener_callback(self, msg):
        #self.get_logger().info('I heard: ' + str(msg.motor_states[0].q))
        self.current_arms = msg
        self.callback_flag = True

    def ruckig_callback(self, msg: RuckigData):
        self.ruck_pos = np.array(self.empty_arm + list(msg.position))
        self.ruck_vel = np.array(self.empty_arm + list(msg.velocity))
        self.ruck_accel = np.array(self.empty_arm + list(msg.acceleration))
        self.callback_flag2 = True

    def on_timer(self):
        if not self.callback_flag:
            return
        # arms = []
        # for joint in self.current_arms.motor_states:
        #     arms.append(joint.q)
        sol_tau = self.arm_ik.get_tau(np.array([joint.q for joint in self.current_arms.motor_states]))
        new_arms = ArmStates()
        tmp_arms = []
        for i in range(14):
            tmp_motor = MotorState()
            tmp_motor.q = 0.0
            tmp_motor.dq = sol_tau[i]
            tmp_arms.append(tmp_motor)
        new_arms.motor_states = tmp_arms
        self.tau_pub.publish(new_arms)

    def on_timer2(self):
        if not self.callback_flag2:
            return
        # arms = []
        # for joint in self.current_arms.motor_states:
        #     arms.append(joint.q)
        sol_tau = self.arm_ik.get_tau_full(self.ruck_pos, self.ruck_vel, self.ruck_accel)
        new_arms = ArmStates()
        tmp_arms = []
        for i in range(14):
            tmp_motor = MotorState()
            tmp_motor.q = 0.0
            tmp_motor.dq = sol_tau[i]
            tmp_arms.append(tmp_motor)
        new_arms.motor_states = tmp_arms
        self.tau_pub.publish(new_arms)

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = TauGenerator()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
