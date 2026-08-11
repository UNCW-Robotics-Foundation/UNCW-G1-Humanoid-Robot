import rclpy
from rclpy.node import Node
from g1_msgs.msg import ArmStates
from g1_msgs.msg import MotorState
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from tf2_ros import TransformException
import numpy as np
from g1_ik.robot_arm_ik import G1_29_ArmIK
from sensor_msgs.msg import Joy
import logging_mp
logger_mp = logging_mp.getLogger(__name__)


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        
        self.subscription = self.create_subscription(
            ArmStates,
            'arm_joints',
            self.listener_callback,
            10)
        self.joy_sub = self.create_subscription(
                    Joy,
                    'joy',
                    self.joy_callback,
                    10)
        self.joy_sub
        self.subscription  # prevent unused variable warning
        self.ik_pub = self.create_publisher(
                    ArmStates,
                    'ik_sol',
                    10)
        self.ik_pub

        self.matrix = np.array([
            [0.993, -0.014, 0.119, 0.191 + 0.06],
            [0.014, 1.0, 0.007, 0.151 + 0.037],
            [-0.119, -0.005, 0.993, 0.073 - 0.2],
            [0.0, 0.0, 0.0, 1.0]
        ])
        # self.matrix = np.array([
        #     [0.991, 0.012, -0.133, -0.201],
        #     [-0.012, 1.0, -0.003, -0.128],
        #     [0.132, 0.004, 0.991, -0.097],
        #     [0.0, 0.0, 0.0, 1.0]
        # ])
        self.matrix_default = np.array([
            [0.993, 0.008, 0.119, 0.192],
            [-0.008, 1.0, 0.0, -0.148],
            [-0.119, -0.001, 0.993, 0.075],
            [0.0, 0.0, 0.0, 1.0]
        ])
        self.count = 0
        self.t_x = 0.0
        self.t_y = 0.0
        self.t_z = 0.0
        self.frame_flag = False
        self.robot_flag = False
        self.btn_flag = False
        self.current_arms = ArmStates()
        self.arm_ik = G1_29_ArmIK()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(0.1, self.on_timer)

    def listener_callback(self, msg):
        #self.get_logger().info('I heard: ' + str(msg.motor_states[0].q))
        self.current_arms = msg
        self.robot_flag = True

    def joy_callback(self, msg):
        if (self.btn_flag):
            press_counter = 0
            msg_btn_size = len(msg.buttons)
            for i in range(msg_btn_size):
                if (msg.buttons[i] == 1):
                    break
                press_counter = press_counter + 1

            if (press_counter == msg_btn_size):
                self.btn_flag = False;

        elif (msg.buttons[11] == 1):   # d-pad up
            self.matrix[0, 3] += 0.01
            self.btn_flag = True
        elif (msg.buttons[12] == 1):   # d-pad down
            self.matrix[0, 3] += -0.01
            self.btn_flag = True
        elif (msg.buttons[13] == 1):   # d-pad left
            self.matrix[2, 3] += 0.01
            self.btn_flag = True
        elif (msg.buttons[14] == 1):   # d-pad right
            self.matrix[2, 3] += -0.01
            self.btn_flag = True
        elif (msg.buttons[0] == 1):   # A
            self.matrix[1, 3] += 0.01
            self.btn_flag = True
        elif (msg.buttons[3] == 1):   # Y
            self.matrix[1, 3] += -0.01
            self.btn_flag = True
        else:
            return

    def on_timer(self):
        try:
            #if self.count < 1:
            if self.count > 1:
                t = self.tf_buffer.lookup_transform(
                    'world',
                    'target',
                    rclpy.time.Time())
                self.get_logger().info('found transform')
                # self.matrix = np.array([
                # [1.0, 0.0, 0.0, t.transform.translation.x],
                # [0.0, 1.0, 0.0, t.transform.translation.y],
                # [0.0, 0.0, 1.0, t.transform.translation.z],
                # [0.0, 0.0, 0.0, 1.0]
                # ])
                # self.matrix = np.array([
                #             [0.138, 0.261, 0.955, 0.094 + t.transform.translation.x],
                #             [-0.007, 0.965, -0.262, 0.114 + t.transform.translation.y],
                #             [-0.990, 0.029, 0.135, -0.077 + t.transform.translation.z],
                #             [0.0, 0.0, 0.0, 1.0]
                # ])
                #self.matrix = self.matrix_default
                self.frame_flag = True
                self.count += 1

            #if self.frame_flag and self.robot_flag:
            if self.robot_flag:
                sol_q, sol_tauff  = self.arm_ik.solve_ik(self.matrix, self.matrix_default, np.array([ joint.q for joint in self.current_arms.motor_states]), np.array([ joint.dq for joint in self.current_arms.motor_states]))
                new_arms = ArmStates()
                tmp_arms = []
                for i in range(14):
                    tmp_motor = MotorState()
                    tmp_motor.q = sol_q[i]
                    tmp_motor.dq = sol_tauff[i]
                    tmp_arms.append(tmp_motor)
                new_arms.motor_states = tmp_arms
                #self.get_logger().info('publishing ik solutions')
                self.ik_pub.publish(new_arms)
                #print(self.matrix)

        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {'test'} to {'left_hand_ref'}: {ex}')
            return


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()