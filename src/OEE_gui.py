#!/usr/bin/python

# Import required Python code.
import roslib
roslib.load_manifest('legosort')
import rospy
import sys
from PyQt4 import QtCore, QtGui
from OEE import Ui_Form
import signal
from datetime import time, datetime

class oee_gui():
	"""docstring for oee_gui"""
	def __init__(self):
		rospy.init_node('oee_gui')
		signal.signal(signal.SIGINT, signal.SIG_DFL)
		app = QtGui.QApplication(sys.argv)
		Form = QtGui.QWidget()
		self.ui = Ui_Form()
		self.ui.setupUi(Form)
		Form.show()

		# Do callbacks here
		timer_value = float(rospy.get_param('~timer_value', 2))
		rospy.Timer(rospy.rostime.Duration(timer_value), self.timer_callback)

		# No need for rospy.spin() in python when we have this. (I think)
		sys.exit(app.exec_())
	
	# Setter for the OEE value
	def set_OEE(self, value):
		self.ui.oee_output.setText(value)

	def timer_callback(self, event):
		self.set_OEE(str(datetime.time(datetime.now()).second))

# Main loop
if __name__ == '__main__':
    try:
        oee_gui();
    except rospy.ROSInterruptException: pass