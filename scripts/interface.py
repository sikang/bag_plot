#!/usr/bin/python

import time
import sys
import bag_reader
import numpy as np
from PyQt5 import QtWidgets, QtCore
from PyQt5.QtWidgets import QApplication, QWidget

import matplotlib
matplotlib.use('Qt5Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas

import mouse_interface

data = {}
topics_selected = []
topics_has_read = []
variables_selected = []

class Example(QWidget):

    def __init__(self):
        super(Example, self).__init__()
        self.initUI()

    def initUI(self):
        has_type = []
        frame_left = {}
        vbox_left = {}
        label_left = {}
        topic_type_dict = bag_reader.read_bag(sys.argv[1])

        for msg_type in topic_type_dict.values():
            if bag_reader.msg_types.count(msg_type) and not has_type.count(msg_type):
                has_type.append(msg_type)

        splitter_left = QtWidgets.QSplitter(QtCore.Qt.Vertical)

        for msg_type in has_type:
            frame_left[msg_type] = QtWidgets.QFrame()
            frame_left[msg_type].setFrameShape(QtWidgets.QFrame.StyledPanel)
            vbox_left[msg_type] = QtWidgets.QVBoxLayout()
            label_left[msg_type] = QtWidgets.QLabel()
            label_left[msg_type].setText(msg_type)
            label_left[msg_type].adjustSize()
            vbox_left[msg_type].addWidget(label_left[msg_type])
            for topic in topic_type_dict.keys():
              if topic_type_dict[topic] == msg_type:
                  cb = QtWidgets.QCheckBox(topic)
                  cb.stateChanged.connect(self.addTopic)
                  vbox_left[msg_type].addWidget(cb)
                  #cb.toggle()

            vbox_left[msg_type].addStretch(1)
            frame_left[msg_type].setLayout(vbox_left[msg_type])
            splitter_left.addWidget(frame_left[msg_type])

        frame_left_var = QtWidgets.QFrame()
        frame_left_var.setFrameShape(QtWidgets.QFrame.StyledPanel)

        vbox_left_var = QtWidgets.QVBoxLayout()
        label_left_var = QtWidgets.QLabel()
        label_left_var.setText('Variables')
        label_left_var.adjustSize()
        vbox_left_var.addWidget(label_left_var)

        for var in bag_reader.var_types:
            cb = QtWidgets.QCheckBox(var)
            cb.stateChanged.connect(self.addVar)
            vbox_left_var.addWidget(cb)
        vbox_left_var.addStretch(0)
        frame_left_var.setLayout(vbox_left_var)
        splitter_left.addWidget(frame_left_var)

        frame_left_control = QtWidgets.QFrame()
        frame_left_control.setFrameShape(QtWidgets.QFrame.StyledPanel)

        vbox_left_control = QtWidgets.QVBoxLayout()
        load_btn = QtWidgets.QPushButton('Load')
        clear_btn = QtWidgets.QPushButton('Clear')
        load_btn.clicked.connect(self.buttonClicked)
        clear_btn.clicked.connect(self.buttonClearClicked)
        vbox_left_control.addWidget(load_btn)
        vbox_left_control.addWidget(clear_btn)
        vbox_left_control.addStretch(1)
        frame_left_control.setLayout(vbox_left_control)
        splitter_left.addWidget(frame_left_control)


        splitter_v = QtWidgets.QSplitter(QtCore.Qt.Horizontal)

        frame_right = QtWidgets.QFrame()
        frame_right.setFrameShape(QtWidgets.QFrame.StyledPanel)

        splitter_v.addWidget(splitter_left)
        splitter_v.addWidget(frame_right)

        hbox = QtWidgets.QHBoxLayout()
        hbox.addWidget(splitter_v)

        self.figure = plt.figure()
        self.canvas = FigureCanvas(self.figure)
        vbox_right = QtWidgets.QVBoxLayout()
        vbox_right.addWidget(self.canvas)
        frame_right.setLayout(vbox_right)

        self.setLayout(hbox)
        self.resize(1000, 800)
        self.center()


        self.show()


    def topics_to_read(self):
        topics_to_read = []
        for topic in topics_selected:
            if not topics_has_read.count(topic):
                topics_has_read.append(topic)
                if not topics_to_read.count(topic):
                   topics_to_read.append(topic)
        return topics_to_read


    def plot(self):
        new_data = bag_reader.read_msg(self.topics_to_read())
        for key in new_data.keys():
            if not key in data:
                data[key] = new_data[key]

        mouse_interface.plot_data(data, variables_selected, topics_selected)
        self.canvas.draw()

    def addTopic(self, state):
        topic = str(self.sender().text())
        if state == QtCore.Qt.Checked:
            if not topics_selected.count(topic):
                topics_selected.append(topic)
        else:
            if topics_selected.count(topic):
                topics_selected.remove(topic)

    def addVar(self, state):
        var = str(self.sender().text())
        if state == QtCore.Qt.Checked:
            if not variables_selected.count(var):
                variables_selected.append(var)
        else:
            if variables_selected.count(var):
                variables_selected.remove(var)

    def buttonClearClicked(self):
        mouse_interface.clear()


    def buttonClicked(self):
        self.plot()

    def keyPressEvent(self, e):

        if e.key() == QtCore.Qt.Key_Escape:
            self.close()
        elif e.key() == QtCore.Qt.Key_Shift:
            mouse_interface.shift_hold = True
        else:
            mouse_interface.shift_hold = False

    def keyReleaseEvent(self, e):
        if e.key() == QtCore.Qt.Key_Shift:
            mouse_interface.shift_hold = False

    def center(self):

        qr = self.frameGeometry()
        cp = QtWidgets.QDesktopWidget().availableGeometry().center()
        qr.moveCenter(cp)
        self.move(qr.topLeft())

def main():
    app = QApplication(sys.argv)
    if(len(sys.argv) < 2):
        print('Usage:',sys.argv[0],' <bag_file>')
        sys.exit(1)
    ex = Example()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
