#!/usr/bin/env python3
"""
G1 arm plan node that generates a path to each point using Ruckig. Subscribes to /clicked_point 
from Rviz2 for points. Subscribes to /joy for commands. Publishes path to /joint/trajectory_point.
Publishes plan pointcloud data to /joint/plan_dbg. Publishes plan points to /joint/plan.
"""

import rclpy
from rclpy.node import Node
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from tf2_ros import TransformException
from geometry_msgs.msg import PointStamped, Point
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import Joy
from g1_msgs.msg import G1Plan, G1Point

from ruckig import InputParameter, OutputParameter, Result, Ruckig


class RuckigPlanNode(Node):
    def __init__(self):
        super().__init__('ruckig_plan_node')

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

        self._got_point = False
        self._init_frame = True
        self._close_to_point = False
        self.print_wait_once = True
        self.return_signal = False
        self.grasp_flag = False
        self.record_initial_pos = True

        self.demo_once_flag = True

        self.plan = []
        self.current_point = 0
        self.last_button = 99
        self.initial_pos = []

        # ---- ROS interfaces ----
        self.point_sub = self.create_subscription(
            PointStamped, 'clicked_point', self.target_callback, 10)

        self.joy_sub = self.create_subscription(
            Joy, 'joy', self.joy_callback, 10)

        self.hand_sub = self.create_subscription(
            Point, 'demo/hand', self.hand_callback, 10)

        self.point_pub = self.create_publisher(
            JointTrajectoryPoint, 'joint/trajectory_point', 10)

        self.dbg_plan_pub = self.create_publisher(
            PointCloud2, 'joint/plan_dbg', 10)

        self.plan_pub = self.create_publisher(
            G1Plan, 'joint/plan', 10)
        self.hand_pub = self.create_publisher(
            Point, 'demo/point', 10)

        self.timer = self.create_timer(self.dt, self.update_loop)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info(
            f"Ruckig trajectory generator running for '{self.joint_name}' "
            f"at {control_freq:.1f} Hz. Waiting fr points...")

    def target_callback(self, msg: PointStamped):
        z_offset = 0.0
        self.plan.append([msg.point.x, msg.point.y, msg.point.z + z_offset])
        # self.plan.append([msg.point.x - .1, msg.point.y, msg.point.z + .15])

        self.inp.target_position = [msg.point.x, msg.point.y, msg.point.z + z_offset]
        self.inp.target_velocity = [0.0, 0.0, 0.0]
        self.inp.target_acceleration = [0.0, 0.0, 0.0]
        self._got_point = True
        self.get_logger().info(f'New point: x={msg.point.x:.4f}, y={msg.point.y:.4f}, z={msg.point.z + z_offset:.4f}')

    def joy_callback(self, msg: Joy):
        if (self.last_button != 99):
            if (msg.buttons[self.last_button] == 1):
                return
            else:
                self.last_button = 99
        if (msg.buttons[6] == 1):  # Start (3 lines)
            self.return_signal = True
            self.last_button = 6
        if (msg.buttons[4] == 1):  
            self.last_button = 4                         # Select (2 windows)
            return

    def hand_callback(self, msg: Point):
        if msg.x == 1.0:
            self.return_signal = True

    def update_loop(self):
        try:
            t_ee = self.tf_buffer.lookup_transform(
                'pelvis',
                'left_ee',
                rclpy.time.Time())

            t_w = self.tf_buffer.lookup_transform(
                'pelvis',
                'left_wrist_yaw_link',
                rclpy.time.Time())

            if self._init_frame:
                self._init_frame = False
                self.initial_pos = [t_ee.transform.translation.x, t_ee.transform.translation.y, t_ee.transform.translation.z]
                self.inp.current_position = self.initial_pos
                self.get_logger().info(f"Wrist Frame Recorded!")

        except TransformException as ex:
            self.get_logger().info(
                f'Could not find transform: {ex}')
            return

        # poincloud2 object creation
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

        if self.return_signal:
            self.inp.target_position = self.initial_pos
            self._got_point = True

        # Rest of controller waits until a plan is confirmed
        if not self._got_point:
            return

        self.check_point(t_ee)

        # Ruckig update
        result = self.otg.update(self.inp, self.out)

        # Ruckig data to publishing topic data
        point = JointTrajectoryPoint()
        point.positions = list(self.out.new_position)
        point.velocities = list(self.out.new_velocity)
        point.accelerations = list(self.out.new_acceleration)
        self.point_pub.publish(point)

        # Feed this cycle's output back in as next cycle's current state.
        # This is the standard Ruckig "online" pattern.
        self.out.pass_to_input(self.inp)

        # Logic for handling plan with Ruckig and logging real to target error. After the plan finishes,
        # the plan can be restarted with the correct controller input.
        if result == Result.Finished and not self._close_to_point:
            # Node would skip points since the main node is de-coupled. This ensures this node will
            # not move onto the next one until the robot gets close enough.
            if self.print_wait_once:
                self.get_logger().info(f'Waiting on main ruckig...')
                self.print_wait_once = False
        elif result == Result.Finished:
            self._got_point = False
            self.get_logger().info(f'Point completed.')
            # self.log_target_errors(t_ee, t_w)
            # self.get_logger().info('Plan finished.')
            self.grasp_flag = True
            self.current_point = 0

            if self.demo_once_flag:
                self.demo_once_flag = False
                # self.return_signal = True
            #     tmp_point = Point()
            #     tmp_point.x = 1.0
            #     self.hand_pub.publish(tmp_point)

        elif result == Result.Error:
            self.get_logger().error(
                'Ruckig returned an error; check limits and target values.')

    '''
    Helper function for logging the deltas between the pinocchio end effector and target, the 
    wrist and target, and the end effector and wrist. The end effector and wrist are compared
    because the end effector is linked to the wrist, and the offset is changed to 0 to compare 
    pinocchio data with something that is known.
    '''
    def log_target_errors(self, t_ee, t_w):
        tmp_dx = abs(self.plan[self.current_point][0] - t_ee.transform.translation.x)
        tmp_dy = abs(self.plan[self.current_point][1] - t_ee.transform.translation.y)
        tmp_dz = abs(self.plan[self.current_point][2] - t_ee.transform.translation.z)
        self.get_logger().info(f'Pin end effector to target error deltas: {tmp_dx} {tmp_dy} {tmp_dz}')

        tmp_dx = abs(self.plan[self.current_point][0] - t_w.transform.translation.x)
        tmp_dy = abs(self.plan[self.current_point][1] - t_w.transform.translation.y)
        tmp_dz = abs(self.plan[self.current_point][2] - t_w.transform.translation.z)
        self.get_logger().info(f'TF wrist to target error deltas: {tmp_dx} {tmp_dy} {tmp_dz}')

        tmp_dx = abs(t_w.transform.translation.x - t_ee.transform.translation.x)
        tmp_dy = abs(t_w.transform.translation.y - t_ee.transform.translation.y)
        tmp_dz = abs(t_w.transform.translation.z - t_ee.transform.translation.z)
        self.get_logger().info(f'Pin ee to TF w error deltas: {tmp_dx} {tmp_dy} {tmp_dz}')
        print()

    '''
    Helper function for determining if the robot is close enough to a point and moving on to the
    next. The values 0.004 and 0.005 were chosen after observing results without this function and
    further testing. Setting a value too low would cause the plan node to never continue.
    '''
    def check_point(self, t_ee):
        tmp_dx = abs(self.plan[self.current_point][0] - t_ee.transform.translation.x)
        tmp_dy = abs(self.plan[self.current_point][1] - t_ee.transform.translation.y)
        tmp_dz = abs(self.plan[self.current_point][2] - t_ee.transform.translation.z)

        if (tmp_dx < 0.008) and (tmp_dy < 0.008) and (tmp_dz < 0.008):
            self._close_to_point = True
        else:
            self._close_to_point = False


def main(args=None):
    rclpy.init(args=args)
    node = RuckigPlanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()