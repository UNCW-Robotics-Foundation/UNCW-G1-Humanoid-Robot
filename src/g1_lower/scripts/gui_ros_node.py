import rclpy
from rclpy.node import Node
from unitree_hg.msg import LowCmd

class LowCmdGuiRosNode(Node):
    def __init__(self):
        super().__init__('low_cmd_gui_node')
        self.cmd_pub = self.create_publisher(
                    LowCmd,
                    'gui_lowcmd',
                    10)
        self.cmd_pub

        self.cmd = LowCmd()
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