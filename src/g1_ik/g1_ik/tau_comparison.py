import rclpy
from rclpy.node import Node
from g1_msgs.msg import ArmStates, MotorState, DebugData
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

        self.callback_flag = False

        self.current_arms = ArmStates()
        self.arm_ik = G1_29_ArmIK()

        #self.timer = self.create_timer(0.004, self.on_timer)

        q = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.05, 0.0098, 0.004566, 0.0612, 0.0, 0.0102, 0.00218])
        self.tau_compare(np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.8789, -0.6009, -0.60218, 0.04506, -0.0001, 0.009006, 0.0009]))
        self.tau_compare(np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.88, -0.60266, -0.6, 0.673, 0.35276, 0.296, 0.3708]))
        self.tau_compare(np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.575, -0.10587, -0.2367, 0.6745, 0.353, 0.295, 0.37076]))
        self.tau_compare(np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1392, -1.0034, -0.8997, -0.416, -0.005776, 0.0061599, 0.00608]))

        # g_base = self.arm_ik.gravity_at(q)
        # g_test = self.arm_ik.gravity_at(q, 'right_wrist_yaw_link', extra_mass=0.05)

        # roll_idx, elbow_idx = self.arm_ik.reduced_robot.model.getJointId('right_shoulder_roll_joint')-1, self.arm_ik.reduced_robot.model.getJointId('right_elbow_joint')-1
        # print("roll change:", g_test[roll_idx]-g_base[roll_idx])
        # print("elbow change:", g_test[elbow_idx]-g_base[elbow_idx])

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

    def tau_compare(self, q):
        g_base = self.arm_ik.gravity_at(q)
        g_test = self.arm_ik.gravity_at(q, 'right_wrist_yaw_link', extra_mass=0.05)

        roll_idx, elbow_idx = self.arm_ik.reduced_robot.model.getJointId('right_shoulder_roll_joint')-1, self.arm_ik.reduced_robot.model.getJointId('right_elbow_joint')-1
        print("roll change:", g_test[roll_idx]-g_base[roll_idx])
        print("elbow change:", g_test[elbow_idx]-g_base[elbow_idx])

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
