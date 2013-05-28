#!/usr/bin/python

# Import required Python code.
import roslib
roslib.load_manifest('legosort')
import rospy
from legosort.srv import *
from legosort.msg import *
import time


class legosort_controller():
	"""docstring for messy_node"""
	def __init__(self):
		rospy.init_node('legosort_controller')

		# Parameters
		self.messy_ip = str(rospy.get_param('~messy_ip', '192.168.10.100'))
		self.max_orders = int(rospy.get_param('~max_orders', 5))
		self.debug = int(rospy.get_param('~debug', 5))
		self.order_update_interval = int(rospy.get_param('~order_update_interval', 5))
		self.max_order_requests = int(rospy.get_param('~max_order_requests', 5))
		new_bricks_topic = str(rospy.get_param('~new_bricks_topic', '/Vision_NS/new_bricks_topic'))


		# Initialize data
		self.working_orders = []
		self.completed_orders = []
		self.fetching_order = 0

		# Development only -- Remove in production
		self.order_finishing_counter = 0

		# Bricks data
		self.coming_bricks = []

		# Do callbacks here
		# rospy.Timer(rospy.rostime.Duration(self.order_update_interval), self.order_update) # Order update timer
		# rospy.Timer(rospy.rostime.Duration(self.order_update_interval), self.order_update) # Order update timer
		
		# Setup publishers
		self.logger = rospy.Publisher('CellLog', LogEntry)
		self.plc_pub = rospy.Publisher('plc', rsd)

		if self.debug:
			print "Legosort controller initializing"
			print "\tMessy Order Server IP:", self.messy_ip
			print "\tMax Orders:", self.max_orders
			print "\tOrder update interval:", self.order_update_interval
			print '\tMax order requests:', self.max_order_requests

		print 'Sleeping to wait for the rest of the system...'
		sleep = 5
		while sleep > 1:
			print str(sleep) + '...'
			time.sleep(1)
			sleep = sleep - 1

		print 'Starting Conveyer...'
		msg = rsd()
		msg.plccommand = 'start'
		self.plc_pub.publish(msg)
		print 'Message send.'

		rospy.spin()

	def get_one_order(self):
		print 'Hep!'


	def order_update(self, event):
		self.logger.publish('mEvent', 'mTime', 'mCellID', 'mComment') 
		if self.order_finishing_counter > 5:
			self.order_finishing_counter = 0
			finished_order = self.working_orders.pop(0)
			rospy.wait_for_service('finish_order')
			try:
				finish_order = rospy.ServiceProxy('finish_order', FinishOrder)
				order = finish_order(str(finished_order['id']), str(finished_order['status']), str(finished_order['ticket']), str(finished_order['time']), finished_order['yellow_bricks'], finished_order['yellow_bricks_packed'], finished_order['blue_bricks'], finished_order['blue_bricks_packed'], finished_order['red_bricks'], finished_order['red_bricks_packed'])
				if order.status == 1:
					print "Order fulfilled."
					finished_order['fulfilled'] = 1
				else:
					print "Order not fulfilled!!!!!"
					finished_order['fulfilled'] = 0
			except rospy.ServiceException, e:
				print "Service call failed: %s"%e
		else:
			self.order_finishing_counter = self.order_finishing_counter + 1
	
		if self.fetching_order == 0:

			orders_to_fetch = self.max_orders - len(self.working_orders)
	
			if self.debug:
				print "Order update:"
				print "\t" + str(len(self.working_orders)), "orders being worked on"
				print '\t' + str(orders_to_fetch), 'orders to fetch'
	
			# Should we fetch an order?
			if orders_to_fetch > 0:

				rospy.wait_for_service('get_order')
				self.fetching_order = 1

				try:
					get_order = rospy.ServiceProxy('get_order', GetOrder)
					order = get_order()
					self.working_orders.append({"id": order.id, "status": order.status, "ticket": order.ticket, "time": order.time, "yellow_bricks": order.yellow_bricks, "yellow_bricks_packed": order.yellow_bricks_packed, "blue_bricks": order.blue_bricks, "blue_bricks_packed": order.blue_bricks_packed, "red_bricks": order.red_bricks, "red_bricks_packed": order.red_bricks_packed})
				except rospy.ServiceException, e:
					print "Service call failed: %s"%e
				self.fetching_order = 0

# Main loop
if __name__ == '__main__':
    try:
        legosort_controller();
    except rospy.ROSInterruptException: pass