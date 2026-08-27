#!/usr/bin/env python3

from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget, QScrollArea
from g1_lower.joint_slider import JointSlider
from g1_lower.gui_ros_node import LowCmdGuiRosNode

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
link_names = [                 # Manually setting joint names, maximums, and minimums
                    "left_shoulder_pitch_link",
                    "left_shoulder_roll_link",
                    "left_shoulder_yaw_link",
                    "left_elbow_link",
                    "left_wrist_roll_link",
                    "left_wrist_pitch_link", 
                    "left_wrist_yaw_link",
                    "right_shoulder_pitch_link",
                    "right_shoulder_roll_link",
                    "right_shoulder_yaw_link",
                    "right_elbow_link",
                    "right_wrist_roll_link",
                    "right_wrist_pitch_link",
                    "right_wrist_yaw_link"
                ]
link_rpy = [1, 0, 2, 1, 0, 1, 2, 1, 0, 2, 1, 0, 1, 2]

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
           slider_id = x[0]
           slider_value = x[1]
           adjusted_id = slider_id - 15
           self.low_cmd_gui_ros_node.cmd.motor_cmd[slider_id].q = slider_value

           if slider_id >= 15:
                self.low_cmd_gui_ros_node.motor = slider_id
                self.low_cmd_gui_ros_node.rpy = link_rpy[adjusted_id]
                self.low_cmd_gui_ros_node.from_frame = link_names[adjusted_id]
                if (slider_id == 15) or (slider_id == 22):
                     self.low_cmd_gui_ros_node.to_frame = "torso_link"
                else :
                     self.low_cmd_gui_ros_node.to_frame = link_names[adjusted_id - 1]

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
