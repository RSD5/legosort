#!/usr/bin/python

from legosort.srv import *
from legosort.msg import *
import rospy

class messy_server():
	def __init__(self):
		rospy.init_node('messy_server')

		# Parameters
		self.messy_ip = str(rospy.get_param('~messy_ip', '192.168.10.100'))
		# self.max_orders = int(rospy.get_param('~max_orders', 5))
		self.debug = int(rospy.get_param('~debug', 0))
		# self.order_update_interval = int(rospy.get_param('~order_update_interval', 5))
		self.max_order_requests = int(rospy.get_param('~max_order_requests', 5))

		# Setup services
		s = rospy.Service('get_order', GetOrder, self.get_order)
		s = rospy.Service('finish_order', FinishOrder, self.finish_order)
		
		# Setup listeners
		rospy.Subscriber("CellLog", LogEntry, self.logger)

		if self.debug:
			print "Messy Server Node initialized"
			print "\tMessy Order Server IP:", self.messy_ip
			# print "\tMax Orders:", self.max_orders
			# print "\tOrder update interval:", self.order_update_interval
			print '\tMax order requests:', self.max_order_requests

		rospy.spin()
	
	def logger(self, data):
		import requests
		from bs4 import BeautifulSoup
		
		soup = BeautifulSoup("""<?xml version="1.0" encoding="utf-8"?> <log_entry> <event>PML_Idle</event> <time>2013-2-25 12:40:21</time> <cell_id>3</cell_id> <comment>Back from stop</comment> </log_entry>""")
		soup.event.string = data.event
		soup.time.string = data.time
		soup.cell_id.string = data.cell_id
		soup.comment.string = data.comment
		xml = soup.prettify()
		headers = {'content-type': 'application/xml'}
		#payload = {'xml': xml}
		r = requests.post('http://' + self.messy_ip + '/log', data=xml, headers=headers)
		
		if self.debug:
			if r.status_code == 201:
				print 'Messy: Logged state:', data.event
			else:
				print 'Messy: Log error:', str(r.status_code)
		
	
	def finish_order(self, order):
		order_to_finish = {"id": order.id, "status": order.status, "ticket": order.ticket, "time": order.time, "yellow_bricks": order.yellow_bricks, "yellow_bricks_packed": order.yellow_bricks_packed, "blue_bricks": order.blue_bricks, "blue_bricks_packed": order.blue_bricks_packed, "red_bricks": order.red_bricks, "red_bricks_packed": order.red_bricks_packed}
		status = self.messy_delete_order(order_to_finish)
		if status:
			return FinishOrderResponse(1)
		else:
			return FinishOrderResponse(0)
		
		

	def get_order(self, req):
		order_fetched = 0
		while order_fetched == 0:
			if self.debug:
				print "Getting an order from the Messy Server."
			orders = self.messy_get_orders()
			picked_order = self.pick_order(orders)
			if picked_order != False:
				self.messy_accept_order(picked_order)
				if picked_order['ticket'] != 0:
					picked_order['red_bricks_packed'] = 0
					picked_order['yellow_bricks_packed'] = 0
					picked_order['blue_bricks_packed'] = 0
					order_fetched = 1

		return GetOrderResponse(str(picked_order['id']), str(picked_order['status']), str(picked_order['ticket']), str(picked_order['time']), picked_order['yellow_bricks'], picked_order['yellow_bricks_packed'], picked_order['blue_bricks'], picked_order['blue_bricks_packed'], picked_order['red_bricks'], picked_order['red_bricks_packed'])
		
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
        messy_server();
    except rospy.ROSInterruptException: pass