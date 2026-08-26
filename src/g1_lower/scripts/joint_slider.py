from PyQt6 import QtCore, QtWidgets
from PyQt6.QtCore import Qt

# Removes ability to move slider with mousewheel
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
        self.slider.setSingleStep(0)    # Removes ability to move slider with keyboard arrows
        self.slider.setPageStep(0)      # Removes ability to move slider when clicking left or right of slider
        self.slider.sliderMoved.connect(self.slider_moved)

        layout_main.addWidget(self.name_label, alignment=Qt.AlignmentFlag.AlignCenter)
        layout_main.addWidget(self.current_label)
        layout_main.addLayout(layout_bar)
        layout_bar.addWidget(self.min_label)
        layout_bar.addWidget(self.slider)
        layout_bar.addWidget(self.max_label)

        self.setLayout(layout_main)

    '''
    Updates the value of the current label, moves the current label to match the position of the slider, and
    emits a signal with the custom widget's id value and current slider value.
    '''
    def slider_moved(self, x):
        self.current_val = x * .001
        self.current_label.setStyleSheet(" padding-left:" + str(((self.current_val - self.middle_value) / (self.middle_distance)) * 110 + 120) + "px ")
        self.moved_slider.emit([self.id, self.current_val])
        if (self.current_val == self.maximum or self.current_val == self.minimum):
            self.current_label.setText(format(self.current_val, '.1f'))
        else:
            self.current_label.setText(format(self.current_val, '.3f'))
