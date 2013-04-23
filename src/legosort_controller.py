#!/usr/bin/python

# Import required Python code.
import roslib
roslib.load_manifest('legosort')
import rospy
from messy_functions import messy_get_orders, messy_accept_order, messy_delete_order


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

		# Initialize data
		self.working_orders = []
		self.completed_orders = []

		# Development only -- Remove in production
		self.order_finishing_counter = 0

		# Do callbacks here
		rospy.Timer(rospy.rostime.Duration(self.order_update_interval), self.order_update)

		if self.debug:
			print "Legosort controller initializing"
			print "\tMessy Order Server IP:", self.messy_ip
			print "\tMax Orders:", self.max_orders
			print "\tOrder update interval:", self.order_update_interval
			print '\tMax order requests:', self.max_order_requests

		rospy.spin()

	def get_one_order(self):
		print 'Hep!'


	def order_update(self, event):

		orders_to_fetch = self.max_orders - len(self.working_orders)

		if self.debug:
			print "Order update:"
			print "\t" + str(len(self.working_orders)), "orders being worked on"
			print '\t' + str(orders_to_fetch), 'orders to fetch'

		# Should we fetch an order?
		if orders_to_fetch > 0:
			orders = self.messy_get_orders()
			picked_order = self.pick_order(orders)
			if picked_order != False:
				self.messy_accept_order(picked_order)
				if picked_order['ticket'] != 0:
					picked_order['red_bricks_packed'] = 0
					picked_order['yellow_bricks_packed'] = 0
					picked_order['blue_bricks_packed'] = 0
					self.working_orders.append(picked_order)

		if self.order_finishing_counter > 5:
			finished_order = self.working_orders.pop(0)
			self.messy_delete_order(finished_order)
			self.completed_orders.append(finished_order)
			self.order_finishing_counter = 0
		else:
			self.order_finishing_counter = self.order_finishing_counter + 1

		
		


	def pick_order(self, orders):
		# Pick an order to add

		print '\t\tPicking order.'

		# if len(self.working_orders) == 0:
		# No othere orders yet, pick the first one
		order_picked = 0

		for order in orders:
			if order['status'] == 'ready':
				order_picked = 1
				picked_order = order
				break

		if order_picked == 0:
			# print 'No applicable order found!'
			return False
		else:
			return picked_order


	def messy_delete_order(self, order):
		import requests
		from bs4 import BeautifulSoup

		print '\t\tDeleting order from server.'

		if 'ticket' in order:
			
			# Fetch XML
			r = requests.delete('http://' + self.messy_ip + '/orders/' + 'ord_' + order['id'] + '/' + order['ticket'])
	
			# Did the request work?
			if r.status_code == 200:
				order['fulfilled'] = True
				return True
			else:
				order['fulfilled'] = False
				return False
		else:
			print 'Does not have ticket'
			return False



	def messy_accept_order(self, order):
		import requests
		from bs4 import BeautifulSoup

		print '\t\tAccepting order.'

		# Fetch XML
		r = requests.put('http://' + self.messy_ip + '/orders/' + 'ord_' + order['id'])
	
		if r.status_code == 200:
	
			# Parse XML
			soup = BeautifulSoup(r.text)
			order['ticket'] = soup.find('ticket').text
	
		else:
			order['ticket'] =  0

	def messy_get_orders(self):
		import requests
		from bs4 import BeautifulSoup
		
		print '\t\tGetting orders.'

		orders = []
	
		# Fetch the XML
		r = requests.get('http://' + self.messy_ip + '/orders/' + str(self.max_order_requests))
	
		# Check if the response is allright
		if r.status_code == 200:
	
			# Parse XML
			soup = BeautifulSoup(r.text)
	
			# Fidn the orders
			order_tags = soup.find_all('order')
	
			#Extract the order URL and fetch the information
			for order in order_tags:
				orders.append(self.messy_get_order_info(order.text))
		else :
	
			# Some HTTP status other than 200 occured
			print 'Error', r.status_code
	
		# Return the orders
		return orders

	def messy_get_order_info(self, order_url):
		import requests
		from bs4 import BeautifulSoup

		order = {}
		order['id'] = order_url.split('_')[-1]
	
		# Fetch the XML
		r = requests.get(order_url)
		if r.status_code == 200:
	
			# Parse XML
			soup = BeautifulSoup(r.text)
	
			# Extract information
			order['status'] = soup.find('status').text
			order['red_bricks'] = int(soup.find('red').text)
			order['blue_bricks'] = int(soup.find('blue').text)
			order['yellow_bricks'] = int(soup.find('yellow').text)
			order['time'] = soup.find('time').text
		else :
	
			# Some HTTP status other than 200 occured
			print 'Error', r.status_code
	
		return order

# Main loop
if __name__ == '__main__':
    try:
        legosort_controller();
    except rospy.ROSInterruptException: pass