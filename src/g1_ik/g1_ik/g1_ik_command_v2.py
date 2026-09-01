import rclpy
from rclpy.node import Node
from g1_msgs.msg import ArmStates
from g1_msgs.msg import MotorState
from g1_msgs.msg import DebugData
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from tf2_ros import TransformException
import numpy as np
from g1_ik.robot_arm_ik import G1_29_ArmIK
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Point
import logging_mp
logger_mp = logging_mp.getLogger(__name__)

from ruckig import InputParameter, OutputParameter, Result, Ruckig
from scipy.spatial.transform import Rotation as R

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
        self.target_sub = self.create_subscription(
                    Point,
                    'joint/target_position',
                    self.target_callback,
                    10)
        self.ik_pub = self.create_publisher(
                    ArmStates,
                    'ik_sol',
                    10)
        self.debug_pub = self.create_publisher(
                    DebugData,
                    'ik_debug',
                    10)

        self.initial_x = 0.207
        self.initial_y = 0.129
        self.initial_z = 0.067
        self.matrix = np.array([
            [0.984, 0.091, 0.156, self.initial_x],
            [-0.090, 0.996, -0.014, self.initial_y],
            [-0.156, 0.000, 0.988, self.initial_z],
            [0.0, 0.0, 0.0, 1.0]
        ])
        # self.matrix = np.array([
        #     [0.991, 0.012, -0.133, -0.201],
        #     [-0.012, 1.0, -0.003, -0.128],
        #     [0.132, 0.004, 0.991, -0.097],
        #     [0.0, 0.0, 0.0, 1.0]
        # ])
        self.matrix_default = np.array([
            [0.982, 0.108, 0.155, 0.177],
            [-0.105, 0.994, -0.024, -0.168],
            [-0.157, 0.007, 0.988, 0.066],
            [0.0, 0.0, 0.0, 1.0]
        ])
        self.count = 0
        self.t_x = 0.0
        self.t_y = 0.0
        self.t_z = 0.0
        self.robot_flag = False
        self.btn_flag = False
        self.current_arms = ArmStates()
        self.arm_ik = G1_29_ArmIK()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(0.1, self.on_timer)

        self.otg = Ruckig(7, 0.1)
        self.inp = InputParameter(7)
        self.out = OutputParameter(7)

        self.inp.current_velocity = [0.0] * 7
        self.inp.current_acceleration = [0.0] * 7

        self.inp.target_velocity = [0.0] * 7
        self.inp.target_acceleration = [0.0] * 7

        self.inp.max_velocity = [0.5] * 7
        self.inp.max_acceleration = [1.0] * 7
        self.inp.max_jerk = [5.0] * 7

        self._has_target = False
        self.ik_flag = False
        self.get_frame = True
        self.is_busy = False

    def listener_callback(self, msg):
        #self.get_logger().info('I heard: ' + str(msg.motor_states[0].q))
        self.current_arms = msg
        self.robot_flag = True

    def target_callback(self, msg):
            if self.is_busy:
                return
            self.matrix[0, 3] = msg.x + self.initial_x
            self.matrix[1, 3] = msg.y + self.initial_y
            self.matrix[2, 3] = msg.z + self.initial_z
            self.ik_flag = True
            #self.get_frame = True
            self.is_busy = True

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
            # self.matrix[0, 3] += 0.01
            self.btn_flag = True
        elif (msg.buttons[12] == 1):   # d-pad down
            # self.matrix[0, 3] += -0.01
            self.btn_flag = True
        elif (msg.buttons[13] == 1):   # d-pad left
            # self.matrix[1, 3] += 0.01
            self.btn_flag = True
        elif (msg.buttons[14] == 1):   # d-pad right
            # self.matrix[1, 3] += -0.01
            self.btn_flag = True
        elif (msg.buttons[0] == 1):   # A
            # self.matrix[2, 3] += -0.01
            self.btn_flag = True
        elif (msg.buttons[3] == 1):   # Y
            # self.matrix[2, 3] += 0.01
            self.btn_flag = True
        elif (msg.buttons[1] == 1):   # B
            self.matrix[0, 3] = self.initial_x
            self.matrix[1, 3] = self.initial_y
            self.matrix[2, 3] = self.initial_z
            self.btn_flag = True
        else:
            return

    def on_timer(self):
        try:
            if self.get_frame:
                t = self.tf_buffer.lookup_transform(
                    'pelvis',
                    'left_wrist_yaw_link',
                    rclpy.time.Time())
                quat = [t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w]
                rotation = R.from_quat(quat)
                rotation_matrix = rotation.as_matrix()
                self.initial_x = t.transform.translation.x
                self.initial_y = t.transform.translation.y
                self.initial_z = t.transform.translation.z

                self.matrix = np.array([
                    [rotation_matrix[0, 0], rotation_matrix[0, 1], rotation_matrix[0, 2], self.initial_x],
                    [rotation_matrix[1, 0], rotation_matrix[1, 1], rotation_matrix[1, 2], self.initial_y],
                    [rotation_matrix[2, 0], rotation_matrix[2, 1], rotation_matrix[2, 2], self.initial_z],
                    [0.0, 0.0, 0.0, 1.0]
                ])
                self.get_frame = False

            if self.robot_flag and self.ik_flag:
                #sol_q, sol_tauff  = self.arm_ik.solve_ik(self.matrix, self.matrix_default, np.array([ joint.q for joint in self.current_arms.motor_states]), np.array([ joint.dq for joint in self.current_arms.motor_states]))
                sol_q, sol_tauff  = self.arm_ik.solve_ik(self.matrix, self.matrix_default, np.array([ joint.q for joint in self.current_arms.motor_states]), np.array([ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]))

                # new_arms = ArmStates()
                # tmp_arms = []
                # for i in range(14):
                #     tmp_motor = MotorState()
                #     tmp_motor.q = sol_q[i]
                #     tmp_motor.dq = sol_tauff[i]
                #     tmp_arms.append(tmp_motor)
                # new_arms.motor_states = tmp_arms
                # self.ik_pub.publish(new_arms)

                self.inp.current_position = [ joint.q for joint in self.current_arms.motor_states[:7]]
                self.inp.target_position = [ t for t in sol_q[:7]]
                self.ik_flag = False
                self._has_target = True

        except TransformException as ex:
            self.get_logger().info(
                f'Could not find transform: {ex}')
            return

        if self._has_target:
            result = self.otg.update(self.inp, self.out)
            
            # point = JointTrajectoryPoint()
            # point.positions = list(self.out.new_position)
            # point.velocities = list(self.out.new_velocity)
            # point.accelerations = list(self.out.new_acceleration)
            # self.point_pub.publish(point)
            new_arms = ArmStates()
            tmp_arms = []
            tmp_list = list(self.out.new_position)
            tmp_list = tmp_list + [0.0] * 7
            for i in range(14):
                tmp_motor = MotorState()
                tmp_motor.q = tmp_list[i]
                tmp_motor.dq = 0.0
                tmp_arms.append(tmp_motor)
            new_arms.motor_states = tmp_arms
            self.ik_pub.publish(new_arms)

            self.out.pass_to_input(self.inp)

            if result == Result.Finished:
                self._has_target = False
                self.is_busy = False
                self.get_logger().info('Trajectory finished.')
            elif result == Result.Error:
                self.get_logger().error(
                    'Ruckig returned an error; check limits and target values.')


        # dbgData = DebugData()
        # dbgData.target_tx = self.matrix[0, 3]
        # dbgData.target_ty = self.matrix[1, 3]
        # dbgData.target_tz = self.matrix[2, 3]
        # dbgData.target_qx = 0.0035137;
        # dbgData.target_qy = 0.0783057;
        # dbgData.target_qz = -0.0454273;
        # dbgData.target_qw = 0.9958877;

        # dbgData.actual_tx = t.transform.translation.x
        # dbgData.actual_ty = t.transform.translation.y
        # dbgData.actual_tz = t.transform.translation.z
        # dbgData.actual_qx = t.transform.rotation.x
        # dbgData.actual_qy = t.transform.rotation.y
        # dbgData.actual_qz = t.transform.rotation.z
        # dbgData.actual_qw = t.transform.rotation.w

        # dbgData.delta_tx = self.abs_helper(dbgData.target_tx, dbgData.actual_tx)
        # dbgData.delta_ty = self.abs_helper(dbgData.target_ty, dbgData.actual_ty)
        # dbgData.delta_tz = self.abs_helper(dbgData.target_tz, dbgData.actual_tz)
        # dbgData.delta_qx = self.abs_helper(dbgData.target_qx, dbgData.actual_qx)
        # dbgData.delta_qy = self.abs_helper(dbgData.target_qy, dbgData.actual_qy)
        # dbgData.delta_qz = self.abs_helper(dbgData.target_qz, dbgData.actual_qz)
        # dbgData.delta_qw = self.abs_helper(dbgData.target_qw, dbgData.actual_qw)

        # self.debug_pub.publish(dbgData)

    def abs_helper(self, x, y):
        if (x > y):
            return x - y
        else :
            return y - x

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