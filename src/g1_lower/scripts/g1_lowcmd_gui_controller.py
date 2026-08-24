import rclpy
from rclpy.node import Node
from unitree_hg.msg import LowCmd
from g1_joints_gui import MainWindow
from PyQt6.QtWidgets import QApplication, QMainWindow
from PyQt6.QtCore import QTimer
import sys

initial_x = 0.207
initial_y = 0.129
initial_z = 0.067

class LowCmdGuiPublisherController(Node, QMainWindow):

    def __init__(self):
        super().__init__('low_cmd_gui_publisher')
        QMainWindow.__init__(self)
        self.gui_window = MainWindow()
        self.gui_window.setupGui(self)
        

        self.cmd_pub = self.create_publisher(
                    LowCmd,
                    'gui_lowcmd',
                    10)
        self.cmd_pub

        #self.control_timer = self.create_timer(0.5, self.control_loop)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.spin_ros)
        self.timer.start(100)

    def control_loop(self):
        tmp_cmd = LowCmd()
        count = 0
        for data in self.gui_window.current_data:
            tmp_cmd.motor_cmd[count].q = data
            count += 1
        
        self.cmd_pub.publish(tmp_cmd)
        self.get_logger().info('published command!')

    def spin_ros(self):
        tmp_cmd = LowCmd()
        count = 0
        for data in self.gui_window.current_data:
            tmp_cmd.motor_cmd[count].q = data
            count += 1
        
        self.cmd_pub.publish(tmp_cmd)
        self.get_logger().info('SPINNING!')
        rclpy.spin_once(self, timeout_sec=0.1)




def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    low_cmd_gui_publisher_controller = LowCmdGuiPublisherController()
    low_cmd_gui_publisher_controller.show()

    #rclpy.spin(low_cmd_gui_publisher_controller)

    low_cmd_gui_publisher_controller.destroy_node()
    exit_code = app.exec()
    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()