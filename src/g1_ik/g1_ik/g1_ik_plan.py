import rclpy
from rclpy.node import Node
from g1_msgs.msg import ArmStates, MotorState, DebugData
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from tf2_ros import TransformException, TransformBroadcaster
import numpy as np
from g1_ik.robot_arm_ik_v3 import G1_29_ArmIK
from sensor_msgs.msg import Joy
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import TransformStamped
import logging_mp
from scipy.spatial.transform import Rotation as R
logger_mp = logging_mp.getLogger(__name__)

real_l = np.array([
    [0.984, 0.091, 0.156, 0.207],
    [-0.090, 0.996, -0.014, 0.129],
    [-0.156, 0.000, 0.988, 0.067],
    [0.0, 0.0, 0.0, 1.0]
])
real_r = np.array([
    [0.982, 0.108, 0.155, 0.177],
    [-0.105, 0.994, -0.024, -0.168],
    [-0.157, 0.007, 0.988, 0.066],
    [0.0, 0.0, 0.0, 1.0]
])

sim_l = np.array([
    [0.955, -0.002, 0.295, 0.185],
    [0.003, 1.000, -0.005, 0.150],
    [-0.295, 0.000, 0.955, 0.061],
    [0.0, 0.0, 0.0, 1.0]
])

sim_r = np.array([
    [0.962, 0.014, 0.271, 0.186],
    [-0.013, 1.000, -0.008, -0.150],
    [-0.271, 0.004, 0.962, 0.062],
    [0.0, 0.0, 0.0, 1.0]
])

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
        self.traj_sub = self.create_subscription(
                    JointTrajectoryPoint,
                    'joint/trajectory_point',
                    self.traj_callback,
                    10)
        self.joy_sub
        self.traj_sub
        self.subscription  # prevent unused variable warning
        self.ik_pub = self.create_publisher(
                    ArmStates,
                    'ik_sol',
                    10)
        self.ik_pub
        self.debug_pub = self.create_publisher(
                    DebugData,
                    'ik_debug',
                    10)
        self.debug_pub

        self.matrix = sim_l
        self.matrix_default = sim_r
        self.initial_x = sim_l[0, 3]
        self.initial_y = sim_l[1, 3]
        self.initial_z = sim_l[2, 3]

        self.count = 0

        self.frame_flag = False
        self.robot_flag = False
        self.btn_flag = False
        #self.traj_flag = False
        self.get_pin_fk = True

        self.current_arms = ArmStates()
        self.current_traj = JointTrajectoryPoint()
        self.arm_ik = G1_29_ArmIK()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.timer = self.create_timer(0.1, self.on_timer)

    def listener_callback(self, msg):
        #self.get_logger().info('I heard: ' + str(msg.motor_states[0].q))
        self.current_arms = msg
        if self.count < 5:
            #print("init joint[0] q:", self.current_arms.motor_states[0].q)
            self.count += 1
        else:
            self.robot_flag = True

    def traj_callback(self, msg):
            self.current_traj = msg
            self.matrix[0, 3] = self.current_traj.positions[0]
            self.matrix[1, 3] = self.current_traj.positions[1]
            self.matrix[2, 3] = self.current_traj.positions[2]
            #self.traj_flag = True

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

    def set_matrix(self, t, r, arm):
        if arm == 0:
            self.initial_x = t[0]
            self.initial_y = t[1]
            self.initial_z = t[2]

            for i in range(3):
                self.matrix[i, 3] = t[i]
                for j in range(3):
                    self.matrix[i, j] = r[i, j]
        else:
            for i in range(3):
                self.matrix_default[i, 3] = t[i]
                for j in range(3):
                    self.matrix_default[i, j] = r[i, j]

    def on_timer(self):
        try:
            # t = self.tf_buffer.lookup_transform(
            #     'pelvis',
            #     'left_wrist_yaw_link',
            #     rclpy.time.Time())

            if self.robot_flag:
                pin_t, pin_r, pin_q = self.arm_ik.get_fk_l(np.array([ joint.q for joint in self.current_arms.motor_states]))
                print("Translation:", pin_t)
                print("Rotation:")
                print(pin_r)
                print(pin_q)
                print()

                frame = TransformStamped()
                frame.header.stamp = self.get_clock().now().to_msg()
                frame.header.frame_id = 'pelvis'
                frame.child_frame_id = 'left_ee'
                frame.transform.translation.x = pin_t[0]
                frame.transform.translation.y = pin_t[1]
                frame.transform.translation.z = pin_t[2]
                frame.transform.rotation.x = pin_q[0]
                frame.transform.rotation.y = pin_q[1]
                frame.transform.rotation.z = pin_q[2]
                frame.transform.rotation.w = pin_q[3]
                self.tf_broadcaster.sendTransform(frame)
                
                if self.get_pin_fk:
                    self.get_pin_fk = False
                    self.set_matrix(pin_t, pin_r, 0)
                    r_pin_t, r_pin_r = self.arm_ik.get_fk_r(np.array([ joint.q for joint in self.current_arms.motor_states]))
                    self.set_matrix(r_pin_t, r_pin_r, 1)
                #time_start = time.time()
                sol_q, sol_tauff  = self.arm_ik.solve_ik(self.matrix, self.matrix_default, np.array([ joint.q for joint in self.current_arms.motor_states]), np.array([ joint.dq for joint in self.current_arms.motor_states]))
                #sol_q, sol_tauff  = self.arm_ik.solve_ik(self.matrix, self.matrix_default, np.array([ joint.q for joint in self.current_arms.motor_states]), np.array([ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]))
                #time_end = time.time()
                #self.get_logger().info("IK Solve Time: " + str((time_end - time_start))
                
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
                if self.count < 15:
                    self.count += 1
                    #print("solution", self.count - 6, " before publishing:", sol_q[0])
                else:
                    self.ik_pub.publish(new_arms)
                    if self.count < 25:
                        #print("solution", self.count - 15, " after publishing:", sol_q[0])
                        self.count += 1

        except TransformException as ex:
            self.get_logger().info(
                f'Could not find transform: {ex}')
            return

        tmp_r = R.from_matrix(self.matrix[0:3, 0:3])
        target_q = tmp_r.as_quat()
        dbgData = DebugData()
        dbgData.target_tx = self.matrix[0, 3]
        dbgData.target_ty = self.matrix[1, 3]
        dbgData.target_tz = self.matrix[2, 3]
        dbgData.target_qx = target_q[0];
        dbgData.target_qy = target_q[1];
        dbgData.target_qz = target_q[2];
        dbgData.target_qw = target_q[3];

        # dbgData.actual_tx = t.transform.translation.x
        # dbgData.actual_ty = t.transform.translation.y
        # dbgData.actual_tz = t.transform.translation.z
        # print(dbgData.actual_tz)
        # dbgData.actual_qx = t.transform.rotation.x
        # dbgData.actual_qy = t.transform.rotation.y
        # dbgData.actual_qz = t.transform.rotation.z
        # dbgData.actual_qw = t.transform.rotation.w

        dbgData.actual_tx = pin_t[0]
        dbgData.actual_ty = pin_t[1]
        dbgData.actual_tz = pin_t[2]
        # print(dbgData.actual_tz)
        dbgData.actual_qx = pin_q[0]
        dbgData.actual_qy = pin_q[1]
        dbgData.actual_qz = pin_q[2]
        dbgData.actual_qw = pin_q[3]

        dbgData.delta_tx = self.abs_helper(dbgData.target_tx, dbgData.actual_tx)
        dbgData.delta_ty = self.abs_helper(dbgData.target_ty, dbgData.actual_ty)
        dbgData.delta_tz = self.abs_helper(dbgData.target_tz, dbgData.actual_tz)
        dbgData.delta_qx = self.abs_helper(dbgData.target_qx, dbgData.actual_qx)
        dbgData.delta_qy = self.abs_helper(dbgData.target_qy, dbgData.actual_qy)
        dbgData.delta_qz = self.abs_helper(dbgData.target_qz, dbgData.actual_qz)
        dbgData.delta_qw = self.abs_helper(dbgData.target_qw, dbgData.actual_qw)

        self.debug_pub.publish(dbgData)

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
