import rclpy
from rclpy.node import Node
from g1_msgs.msg import ArmStates
from g1_msgs.msg import MotorState
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from tf2_ros import TransformException
import numpy as np
from g1_ik.robot_arm_ik import G1_29_ArmIK


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        
        self.subscription = self.create_subscription(
            ArmStates,
            'arm_joints',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.ik_pub = self.create_publisher(
                    ArmStates,
                    'ik_sol',
                    10)
        self.ik_pub

        self.matrix = np.array([
            [1.0, 0.0, 0.0, 1.0],
            [0.0, 1.0, 0.0, 2.0],
            [0.0, 0.0, 1.0, 3.0],
            [0.0, 0.0, 0.0, 1.0]
        ])
        self.matrix_default = np.array([
                    [1.0, 0.0, 0.0, 0.0],
                    [0.0, 1.0, 0.0, 0.0],
                    [0.0, 0.0, 1.0, 0.0],
                    [0.0, 0.0, 0.0, 1.0]
                ])
        self.count = 0
        self.frame_flag = False
        self.robot_flag = False
        self.current_arms = ArmStates()
        self.arm_ik = G1_29_ArmIK()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(1.0, self.on_timer)

    def listener_callback(self, msg):
        #self.get_logger().info('I heard: ' + str(msg.motor_states[0].q))
        self.current_arms = msg
        self.robot_flag = True

    def on_timer(self):
        try:
            t = self.tf_buffer.lookup_transform(
                'left_hand_ref',
                'test',
                rclpy.time.Time())
            self.get_logger().info('found transform')
            self.matrix = np.array([
            [1.0, 0.0, 0.0, t.transform.translation.x],
            [0.0, 1.0, 0.0, t.transform.translation.y],
            [0.0, 0.0, 1.0, t.transform.translation.z],
            [0.0, 0.0, 0.0, 1.0]
            ])
            self.frame_flag = True

            if self.count < 1:
                if self.frame_flag and self.robot_flag:
                    self.count += 1
                    sol_q, sol_tauff  = self.arm_ik.solve_ik(self.matrix, self.matrix_default, np.array([ joint.q for joint in self.current_arms.motor_states[:6]]), np.array([ joint.q for joint in self.current_arms.motor_states[6:]]))
                    new_arms = ArmStates()
                    tmp_arms = []
                    for i in range(14):
                        tmp_motor = MotorState()
                        tmp_motor.q = sol_q[i]
                        tmp_motor.dq = sol_tauff[i]
                        tmp_arms.append(tmp_motor)
                    new_arms.motor_states = tmp_arms
                    self.ik_pub.publish(new_arms)

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