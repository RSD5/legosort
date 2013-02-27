#!/usr/bin/python

# Import required Python code.
import roslib
roslib.load_manifest('legosort')
import rospy
import sys

class oee_gui():
	"""docstring for oee_gui"""
	def __init__(self):
		rospy.init_node('oee_gui')

		timer_value = float(rospy.get_param('~timer_value', 2))

		rospy.Timer(rospy.rostime.Duration(timer_value), self.timer_callback)
		rospy.loginfo("Localization is on")
		rospy.spin()

	def timer_callback(self, event):
		rospy.loginfo("Timed")

# Main loop
if __name__ == '__main__':
    try:
        oee_gui();
    except rospy.ROSInterruptException: pass