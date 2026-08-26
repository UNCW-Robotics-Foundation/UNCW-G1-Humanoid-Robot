from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget, QScrollArea
from joint_slider import JointSlider
from gui_ros_node import LowCmdGuiRosNode

import sys
import signal
import threading
import rclpy

'''

Creates a Qt gui window with custom joint sliders and a simple ROS node. Each joint slider is 
connected to its corresponding joint in the lowcmd message of the ROS node, so it is able to 
update in real time. Only changes the joint position (q). Maximums and minimums were taken from
the urdf, but they are ~0.1 less than what is actually stated in the urdf.

'''
class LowCmdGui(QMainWindow):
    def __init__(self, low_cmd_gui_ros_node):
        super().__init__()

        # Initializing objects adn values
        self.low_cmd_gui_ros_node = low_cmd_gui_ros_node
        self.running = True
        joint_names = [                 # Manually setting joint names, maximums, and minimums
            "left_hip_pitch_joint",
            "left_hip_roll_joint",
            "left_hip_yaw_joint",
            "left_knee_joint",
            "left_ankle_pitch_joint",
            "left_ankle_roll_joint",
            "right_hip_pitch_joint",
            "right_hip_roll_joint",
            "right_hip_yaw_joint",
            "right_knee_joint",
            "right_ankle_pitch_joint",
            "right_ankle_roll_joint",
            "waist_yaw_joint",
            "waist_roll_joint", 
            "waist_pitch_joint", 
            "left_shoulder_pitch_joint",
            "left_shoulder_roll_joint",
            "left_shoulder_yaw_joint",
            "left_elbow_joint",
            "left_wrist_roll_joint",
            "left_wrist_pitch_joint", 
            "left_wrist_yaw_joint",
            "right_shoulder_pitch_joint",
            "right_shoulder_roll_joint",
            "right_shoulder_yaw_joint",
            "right_elbow_joint",
            "right_wrist_roll_joint",
            "right_wrist_pitch_joint",
            "right_wrist_yaw_joint"
        ]
        joints_max = [2.7, 2.8, 2.6, 2.7, 0.4, 0.2, 2.7, 0.4, 2.6, 2.7, 0.4, 0.2, 2.5, 0.4, 0.4, 2.5, 2.1, 2.5, 1.9, 1.8, 1.5, 1.5, 2.5, 1.4, 2.5, 1.9, 1.8, 1.5, 1.5]
        joints_min = [-2.4, -0.4, -2.6, -0.06, -0.7, -0.2, -2.4, -2.8, -2.6, -0.06, -0.7, -0.2, -2.5, -0.4, -0.4, -2.9, -1.4, -2.5, -0.9, -1.8, -1.5, -1.5, -2.9, -2.1, -2.5, -0.9, -1.8, -1.5, -1.5]

        # Initializing gui objects
        self.setWindowTitle("My App")
        self.scroll_area = QScrollArea()
        layout = QVBoxLayout()
        widget = QWidget()

        # Creating and adding custom sliders for every joint
        for i in range(29):
             j_slider = JointSlider(jName=joint_names[i], maximum=int(joints_max[i] * 1000), minimum=int(joints_min[i] * 1000), jId=i)
             j_slider.moved_slider.connect(self.update_cmd)
             layout.addWidget(j_slider)

        # Setting gui object configurations
        widget.setLayout(layout)
        self.scroll_area.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOn)
        self.scroll_area.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.scroll_area.setWidgetResizable(True)
        self.scroll_area.setWidget(widget)

        # Set the central widget of the Window.
        self.setCentralWidget(self.scroll_area)
        self.setGeometry(600, 100, 320, 300)
        self.setMinimumWidth(320)


    '''
    The custom joint slider has its own signal that returns a list with the first value being its
    assigned id, and the second value being the slider's current value. That signal is connected to this
    function everytime it activates
    '''
    def update_cmd(self, x):
           self.low_cmd_gui_ros_node.cmds.motor_cmd[x[0]].q = x[1]

    '''
    This replaces the closeEvent function from a QMainWindow. I'm pretty sure this is just a signal,
    and this does'nt handle actually terminating the object. It gets repurposed to flip a flag.
    '''
    def closeEvent(self, a0):
          self.running = False

    '''
    A simple loop for spinning the ros node
    '''
    def loop(self):
          while self.running:
                rclpy.spin_once(self.low_cmd_gui_ros_node, timeout_sec=0.1)

    
def main():
    rclpy.init()

    app = QApplication(sys.argv)
    l_cmd_gui = LowCmdGui(LowCmdGuiRosNode())
    l_cmd_gui.show()

    # Use a python thread for running the ros loop. In a previous attempt, I used a QT timer for the loop,
    # but it resulted in a lower framerate gui. I'm not sure if it was the implemntation, or if a qt timer
    # is just worse performance-wise compared to a python thread.
    threading.Thread(target=l_cmd_gui.loop).start()
    signal.signal(signal.SIGINT, signal.SIG_DFL)    # Allows cmd line ctrl+c to terminate program
    sys.exit(app.exec() if hasattr(app, 'exec') else app.exec_())
       

if __name__ == "__main__":
      main()