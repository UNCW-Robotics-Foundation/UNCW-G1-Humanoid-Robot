from PyQt6.QtCore import QSize, Qt
from PyQt6.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget, QPushButton, QSlider, QLabel, QScrollArea
from joint_slider import JointSlider
from PyQt6 import QtWidgets

import sys
import signal

signal.signal(signal.SIGINT, signal.SIG_DFL)

# class MainWindow(QMainWindow):
#     def __init__(self):
#         super().__init__()
#     # def setupGui(self, window)

#         joint_names = [
#             "left_hip_pitch_joint",
#             "left_hip_roll_joint",
#             "left_hip_yaw_joint",
#             "left_knee_joint",
#             "left_ankle_pitch_joint",
#             "left_ankle_roll_joint",
#             "right_hip_pitch_joint",
#             "right_hip_roll_joint",
#             "right_hip_yaw_joint",
#             "right_knee_joint",
#             "right_ankle_pitch_joint",
#             "right_ankle_roll_joint",
#             "waist_yaw_joint",
#             "waist_roll_joint", 
#             "waist_pitch_joint", 
#             "left_shoulder_pitch_joint",
#             "left_shoulder_roll_joint",
#             "left_shoulder_yaw_joint",
#             "left_elbow_joint",
#             "left_wrist_roll_joint",
#             "left_wrist_pitch_joint", 
#             "left_wrist_yaw_joint",
#             "right_shoulder_pitch_joint",
#             "right_shoulder_roll_joint",
#             "right_shoulder_yaw_joint",
#             "right_elbow_joint",
#             "right_wrist_roll_joint",
#             "right_wrist_pitch_joint",
#             "right_wrist_yaw_joint"
#         ]
#         joints_max = [2.7, 2.8, 2.6, 2.7, 0.4, 0.2, 2.7, 0.4, 2.6, 2.7, 0.4, 0.2, 2.5, 0.4, 0.4, 2.5, 2.1, 2.5, 1.9, 1.8, 1.5, 1.5, 2.5, 1.4, 2.5, 1.9, 1.8, 1.5, 1.5]
#         joints_min = [-2.4, -0.4, -2.6, -0.06, -0.7, -0.2, -2.4, -2.8, -2.6, -0.06, -0.7, -0.2, -2.5, -0.4, -0.4, -2.9, -1.4, -2.5, -0.9, -1.8, -1.5, -1.5, -2.9, -2.1, -2.5, -0.9, -1.8, -1.5, -1.5]
#         j_params = []

#         self.setWindowTitle("My App")
#         self.scroll_area = QScrollArea()
#         layout = QVBoxLayout()
#         self.labels = []
#         self.current_data = []

#         for i in range(len(joint_names)):
#             j_params.append([joint_names[i], int(joints_max[i] * 1000), int(joints_min[i] * 1000)])
#             tmp_label = QLabel(joint_names[i])
#             self.labels.append(tmp_label)
#             layout.addWidget(tmp_label)
#             self.current_data.append(0.0)

#         count = 0
#         for i in j_params:
#              j_slider = JointSlider(jName=i[0], maximum=i[1], minimum=i[2], jId=count)
#              j_slider.moved_slider.connect(self.btn_pressed_id)
#              count += 1
#              layout.addWidget(j_slider)

#         widget = QWidget()
#         widget.setLayout(layout)

#         self.scroll_area.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOn)
#         self.scroll_area.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
#         self.scroll_area.setWidgetResizable(True)
#         self.scroll_area.setWidget(widget)

#         # Set the central widget of the Window.
#         self.setCentralWidget(self.scroll_area)
#         self.setGeometry(600, 100, 320, 300)
#         self.setMinimumWidth(320)

#     def btn_pressed_id(self, x):
#                 self.labels[x[0]].setText(str(x[1]))
#                 self.current_data[x[0]] = x[1]

    

# app = QApplication(sys.argv)


# window = MainWindow()
# window.show()

# app.exec()

class MainWindow(object):
    # def __init__(self):
    #     super().__init__()
    def setupGui(self, window):
        window.setGeometry(600, 100, 320, 300)
        window.setMinimumWidth(320)

        joint_names = [
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
        j_params = []

        #self.setWindowTitle("My App")
        self.scroll_area = QScrollArea()
        layout = QVBoxLayout()
        self.labels = []
        self.current_data = []

        for i in range(len(joint_names)):
            j_params.append([joint_names[i], int(joints_max[i] * 1000), int(joints_min[i] * 1000)])
            tmp_label = QLabel(joint_names[i])
            self.labels.append(tmp_label)
            layout.addWidget(tmp_label)
            self.current_data.append(0.0)

        count = 0
        for i in j_params:
             j_slider = JointSlider(jName=i[0], maximum=i[1], minimum=i[2], jId=count)
             j_slider.moved_slider.connect(self.btn_pressed_id)
             count += 1
             layout.addWidget(j_slider)

        self.widget = QWidget(window)
        self.widget.setLayout(layout)

        self.scroll_area.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOn)
        self.scroll_area.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.scroll_area.setWidgetResizable(True)
        self.scroll_area.setWidget(self.widget)

        window.setCentralWidget(self.scroll_area)
        #window.setCentralWidget(self.widget)

    def btn_pressed_id(self, x):
                self.labels[x[0]].setText(str(x[1]))
                self.current_data[x[0]] = x[1]

if __name__ == "__main__":
      app = QtWidgets.QApplication(sys.argv)
      window = QtWidgets.QMainWindow()
      gui = MainWindow()
      gui.setupGui(window)
      #window.show()
      sys.exit(app.exec())
