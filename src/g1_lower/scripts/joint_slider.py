from PyQt6 import QtCore, QtGui, QtWidgets
from PyQt6.QtCore import Qt

# class SliderContainer(QtWidgets.QWidget):

#     def __init__(self, joints, *args, **kwargs):
#             super().__init__(*args, **kwargs)

#             layout = QtWidgets.QVBoxLayout()

#             self.current_values = []
#             self.joint_list = joints
#             for joint in self.joint_list:
#                 self.current_values.append(0.0)
#                 layout.addWidget(JointSlider(jName=joint[0], maximum=joint[1], minimum=joint[2]))

#             self.setLayout(layout)

class CustomSlider(QtWidgets.QSlider):
    def wheelEvent(self, e):
        e.ignore()

class JointSlider(QtWidgets.QWidget):
    """
    Custom Qt Widget for individual g1 joints
    """

    moved_slider = QtCore.pyqtSignal(list)

    def __init__(self, maximum=1000, minimum=-1000, jName='tmp', jId=0, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.setFixedSize(300, 80)
        self.maximum = maximum * 0.001
        self.minimum = minimum * 0.001
        self.current_val = 0.0
        self.id = jId

        self.middle_value = (self.minimum  + self.maximum) / 2
        self.middle_distance = ((self.minimum * -1) + self.maximum) / 2

        layout_main = QtWidgets.QVBoxLayout()
        #layout_current = QtWidgets.QHBoxLayout()
        layout_bar = QtWidgets.QHBoxLayout()

        self.name_label = QtWidgets.QLabel(jName)
        self.current_label = QtWidgets.QLabel('0.000')
        self.slider = CustomSlider(Qt.Orientation.Horizontal)
        self.min_label = QtWidgets.QLabel(format(minimum * .001, '.1f'))
        self.max_label = QtWidgets.QLabel(format(maximum * .001, '.1f'))
        self.spacer = QtWidgets.QSpacerItem(0, 0, QtWidgets.QSizePolicy.Policy.Minimum, QtWidgets.QSizePolicy.Policy.Expanding)
        self.current_label.setStyleSheet(" padding-left:" + str(((self.current_val - self.middle_value) / (self.middle_distance)) * 110 + 120) + "px ")

        self.slider.setMaximum(maximum)
        self.slider.setMinimum(minimum)
        self.slider.setSingleStep(0)
        self.slider.setPageStep(0)
        self.slider.sliderMoved.connect(self.slider_moved)

        layout_main.addWidget(self.name_label, alignment=Qt.AlignmentFlag.AlignCenter)
        #layout_main.addLayout(layout_current)
        #layout_current.addWidget(self.current_label)
        layout_main.addWidget(self.current_label)
        layout_main.addLayout(layout_bar)
        layout_bar.addWidget(self.min_label)
        layout_bar.addWidget(self.slider)
        layout_bar.addWidget(self.max_label)

        self.setLayout(layout_main)

    def slider_moved(self, x):
        self.current_val = x * .001
        self.current_label.setStyleSheet(" padding-left:" + str(((self.current_val - self.middle_value) / (self.middle_distance)) * 110 + 120) + "px ")
        self.moved_slider.emit([self.id, self.current_val])
        if (self.current_val == self.maximum or self.current_val == self.minimum):
            self.current_label.setText(format(self.current_val, '.1f'))
        else:
            self.current_label.setText(format(self.current_val, '.3f'))
