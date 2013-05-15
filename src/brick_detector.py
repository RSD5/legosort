#!/usr/bin/python

from legosort.msg import *
import rospy

class brick_detector():
	def __init__(self):
		rospy.init_node('brick_detector')

		# Parameters
		self.brick_listen_topic = str(rospy.get_param('~raw_bricks_topic', 'all_bricks_topic'))
		self.brick_publish_topic = str(rospy.get_param('~brick_publish_topic', 'new_bricks_topic'))
		self.debug = int(rospy.get_param('~debug', 0))
		self.brick_size_tolerance = float(rospy.get_param('~brick_size_tolerance', 0.1))
		self.brick_angle_tolerance = float(rospy.get_param('~brick_angle_tolerance', 0.1))
		
		# Setup listeners
		rospy.Subscriber(self.brick_listen_topic, RawBrick, self.detector)

		# Setup publishers
		self.brick_pub = rospy.Publisher(self.brick_publish_topic, RawBrick)

		# Setup timers
		# Clean old bricks from self.bricks
		#rospy.Timer(rospy.rostime.Duration(timer_value), self.timer_callback)

		# List for known bricks
		self.bricks = []

		if self.debug:
			print "Messy Server Node initialized"
			print "\tMessy Order Server IP:", self.messy_ip
			# print "\tMax Orders:", self.max_orders
			# print "\tOrder update interval:", self.order_update_interval
			print '\tMax order requests:', self.max_order_requests

		rospy.spin()
	
	def detector(self, brick):
		# print 'Brick at angle', data.angle

		if len(self.bricks) > 0:
			new = True
			for old_brick in self.bricks:
				if self.is_brick_the_same(brick, old_brick):
					new = False
					break
			if new:
				# Brick new
				self.bricks.append(brick)
				print '1 New brick', brick.color
				self.brick_pub.publish(brick)	

		else:
			# Brick new
			self.bricks.append(brick)
			print '1 New brick', brick.color
			self.brick_pub.publish(brick)

	def is_brick_the_same(self, brick1, brick2):
		if brick1.color != brick2.color:
			return False
		if abs(brick1.width - brick2.width) > self.brick_size_tolerance:
			return False
		if abs(brick1.height - brick2.height) > self.brick_size_tolerance:
			return False
		if abs(brick1.angle - brick2.angle) > self.brick_angle_tolerance:
			return False
		return True



# Main loop
if __name__ == '__main__':
    try:
        brick_detector();
    except rospy.ROSInterruptException: pass