import rclpy
from rclpy.node import Node
from unitree_hg.msg import LowCmd
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from g1_msgs.msg import DebugData
import math

class LowCmdGuiRosNode(Node):
    def __init__(self):
        super().__init__('low_cmd_gui_node')
        self.cmd_pub = self.create_publisher(
                    LowCmd,
                    'gui_lowcmd',
                    10)
        self.cmd_pub
        self.dbg_pub = self.create_publisher(
                            DebugData,
                            'gui_dbg',
                            10)
        self.dbg_pub

        self.cmd = LowCmd()
        self.to_frame = "left_wrist_pitch_link"
        self.from_frame = "left_wrist_yaw_link"
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.rpy = 2
        self.motor = 21

        self.cmd.mode_machine = 5
        for i in range(29):
            self.cmd.motor_cmd[i].dq = 0.0
            self.cmd.motor_cmd[i].tau = 0.0
            self.cmd.motor_cmd[i].kp = 60.0
            self.cmd.motor_cmd[i].kd = 1.5
            self.cmd.motor_cmd[i].mode = 1

        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        self.cmd_pub.publish(self.cmd)

        try:
            t = self.tf_buffer.lookup_transform(self.to_frame, self.from_frame, rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(f'Could not transform {self.to_frame} to {self.from_frame}: {ex}')
            return

        dbg_msg = DebugData()
        qx = t.transform.rotation.x
        qy = t.transform.rotation.y
        qz = t.transform.rotation.z
        qw = t.transform.rotation.w

        match self.rpy:
            case 0:
                dbg_msg.target_rr = self.cmd.motor_cmd[self.motor].q
                dbg_msg.actual_rr = self.quat_to_roll(qx, qy, qz, qw)
                dbg_msg.delta_rr = self.delta_helper(dbg_msg.target_rr, dbg_msg.actual_rr)
            case 1:
                dbg_msg.target_rp = self.cmd.motor_cmd[self.motor].q
                dbg_msg.actual_rp = self.quat_to_pitch(qx, qy, qz, qw)
                dbg_msg.delta_rp = self.delta_helper(dbg_msg.target_rp, dbg_msg.actual_rp)
            case 2:
                dbg_msg.target_ry = self.cmd.motor_cmd[self.motor].q
                dbg_msg.actual_ry = self.quat_to_yaw(qx, qy, qz, qw)
                dbg_msg.delta_ry = self.delta_helper(dbg_msg.target_ry, dbg_msg.actual_ry)

        self.dbg_pub.publish(dbg_msg)
            

    def quat_to_roll(self, x, y, z, w):
        t0 = 2 * (w * x + y * z)
        t1 = 1 - 2 *(x * x + y * y)
        roll = math.atan2(t0, t1)
        return roll

    def quat_to_pitch(self, x, y, z, w):
        t2 = 2 * (w * y - x * z)
        t2 = 1 if t2 > 1 else t2
        t2 = -1 if t2 < -1 else t2
        pitch = math.asin(t2)
        return pitch

    def quat_to_yaw(self, x, y, z, w):
        t3 = 2 * (w * z + y * x)
        t4 = 1 - 2 *(z * z + y * y)
        yaw = math.atan2(t3, t4)
        return yaw

    def delta_helper(self, x, y):
        if x > y:
            return x - y
        else:
            return y - x
        
        

def main():
    rclpy.init()
    low_cmd_gui_node = LowCmdGuiRosNode()

    try:
        rclpy.spin(low_cmd_gui_node)
    except KeyboardInterrupt:
        pass

    low_cmd_gui_node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()