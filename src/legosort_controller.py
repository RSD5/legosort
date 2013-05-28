#!/usr/bin/python

# Import required Python code.
import roslib
roslib.load_manifest('legosort')
import rospy
from legosort.srv import *
from legosort.msg import *
import time
import datetime

class legosort_controller():
	"""docstring for messy_node"""
	def __init__(self):
		rospy.init_node('legosort_controller')

		self.robot_state = 0
		self.robot_state_counter = 0

		# Timing
		self.start_time = datetime.datetime(2013, 5, 28)

		# Parameters
		self.messy_ip = str(rospy.get_param('~messy_ip', '192.168.10.100'))
		self.max_orders = int(rospy.get_param('~max_orders', 5))
		self.debug = int(rospy.get_param('~debug', 5))
		self.order_update_interval = int(rospy.get_param('~order_update_interval', 5))
		self.max_order_requests = int(rospy.get_param('~max_order_requests', 5))
		self.new_bricks_topic = str(rospy.get_param('~new_bricks_topic', '/Vision_NS/new_bricks_topic'))
		self.brick_timer_interval = float(rospy.get_param('~brick_timer_interval', 5))
		self.brick_travel_duration = int(rospy.get_param('~brick_travel_duration', 1000))
		self.brick_travel_duration_error_max = int(rospy.get_param('~brick_travel_duration_error_max', 10))


		# Initialize data
		self.working_orders = []
		self.completed_orders = []
		self.fetching_order = 0

		# Development only -- Remove in production
		self.order_finishing_counter = 0

		# Bricks data
		self.coming_bricks = []
		self.bricks_placed = []

		# Do callbacks here
		# rospy.Timer(rospy.rostime.Duration(self.order_update_interval), self.order_update) # Order update timer
		rospy.Timer(rospy.rostime.Duration(self.brick_timer_interval), self.robot_state_machine) # Order update timer
		
		# Setup publishers
		self.logger = rospy.Publisher('CellLog', LogEntry)
		self.plc_pub = rospy.Publisher('plc', rsd)

		# Setup listeners
		rospy.Subscriber(self.new_bricks_topic, RawBrick, self.add_brick)

		if self.debug:
			print "Legosort controller initializing"
			print "\tMessy Order Server IP:", self.messy_ip
			print "\tMax Orders:", self.max_orders
			print "\tOrder update interval:", self.order_update_interval
			print '\tMax order requests:', self.max_order_requests


		
		self.start_the_conveyer()
		

		# print 'Sleeping to wait for the rest of the system...'
		# sleep = 5
		# while sleep > 1:
		# 	print str(sleep) + '...'
		# 	time.sleep(1)
		# 	sleep = sleep - 1


		rospy.spin()

	def robot_state_machine(self, arg):
		st_start_up = 0
		st_moving_to_ready = 1
		st_ready = 2
		st_grip = 2
		st_moving_to_place = 3
		st_place = 4
		st_waiting_for_grip = 5

		if self.robot_state == st_start_up:
			print 'State: start_up'
			self.robot_to_ready()
			self.robot_state = st_moving_to_ready

		if self.robot_state == st_moving_to_ready:
			print 'State: moving to ready'
			if self.robot_state_counter > 40:
				self.robot_state_counter = 0
				self.robot_state = st_ready
			else:
				self.robot_state_counter = self.robot_state_counter + 1

		if self.robot_state == st_ready:
			print 'State: ready'
			if self.check_for_brick() == True:
				self.robot_grip()
				self.robot_state = st_waiting_for_grip

		if self.robot_state == st_waiting_for_grip:
			print 'State: waiting for grip'
			if self.robot_state_counter > 5:
				self.robot_state_counter = 0
				self.robot_place()
				self.robot_state = st_moving_to_place
			else:
				self.robot_state_counter = self.robot_state_counter + 1
		
		if self.robot_state == st_moving_to_place:
			print 'State: moving to place'
			if self.robot_state_counter > 10:
				self.robot_state_counter = 0
				self.robot_state = st_place
			else:
				self.robot_state_counter = self.robot_state_counter + 1

		if self.robot_state == st_place:
			print 'State: place'
			self.robot_to_ready()
			self.robot_state = st_moving_to_ready


	def robot_to_ready(self):
		msg = rsd()
		msg.plccommand = 'robot'
		msg.boxnumber = 4
		self.plc_pub.publish(msg)

	def robot_grip(self):
		msg = rsd()
		msg.plccommand = 'robot'
		msg.boxnumber = 5
		self.plc_pub.publish(msg)

	def robot_place(self):
		msg = rsd()
		msg.plccommand = 'robot'
		msg.boxnumber = 11
		self.plc_pub.publish(msg)

	def check_for_brick(self):
		millis_now = self.millis()

		if len(self.coming_bricks) > 0:
			for i in range(len(self.coming_bricks)):
				travel = abs(millis_now - self.coming_bricks[i].time_milliseconds)
				adjusted_travel = self.brick_travel_duration + (3000 - int(self.coming_bricks[i].y_coord))
				error = abs(travel - adjusted_travel)
				if  error < self.brick_travel_duration_error_max:
					self.bricks_placed.append(self.coming_bricks[i])
					del self.coming_bricks[i]
					return True
		return False

	def brick_pos_update(self, arg):
		millis_now = self.millis()

		if len(self.coming_bricks) > 0:
			for i in range(len(self.coming_bricks)):
				travel = abs(millis_now - self.coming_bricks[i].time_milliseconds)
				adjusted_travel = self.brick_travel_duration + (3000 - int(self.coming_bricks[i].y_coord))
				error = abs(travel - adjusted_travel)
				if  error < self.brick_travel_duration_error_max:
					if self.coming_bricks[i].color == 1:
						print 'Yellow brick now!!!!!!!!!!!!!!'
					if self.coming_bricks[i].color == 2:
						print 'Blue brick now!!!!!!!!!!!!!!'
					if self.coming_bricks[i].color == 3:
						print 'Red brick now!!!!!!!!!!!!!!'
					msg = rsd()
					msg.plccommand = 'robot'
					msg.boxnumber = 5
					self.plc_pub.publish(msg)
					sleep = 3
					while sleep > 0:
						print str(sleep) + '...'
						time.sleep(1)
						sleep = sleep - 1
					msg = rsd()
					msg.plccommand = 'robot'
					msg.boxnumber = 4
					self.plc_pub.publish(msg)
					del self.coming_bricks[i]
					break


	def millis(self):
		dt = datetime.datetime.now() - self.start_time
		ms = (dt.days * 24 * 60 * 60 + dt.seconds) * 1000 + dt.microseconds / 1000.0
		return ms

	def add_brick(self, brick):
		self.coming_bricks.append(brick)

	def get_one_order(self):
		print 'Hep!'

	def start_the_conveyer(self):
		print 'Starting Conveyer...'
		msg = rsd()
		msg.plccommand = 'start'
		self.plc_pub.publish(msg)

	def stop_the_conveyer(self):
		print 'Stopping Conveyer...'
		msg = rsd()
		msg.plccommand = 'stop'
		self.plc_pub.publish(msg)

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