#!/usr/bin/python

from legosort.msg import *
import rospy

class packml_statemachine():
	def __init__(self):
		rospy.init_node('packml_statemachine')

		# Parameters
		self.debug = int(rospy.get_param('~debug', 0))
		log_topic = str(rospy.get_param('~log_topic', 'MessyServer/CellLog'))

		# Setup services
		# Get current state-service
		#s = rospy.Service('get_order', GetOrder, self.get_order)
		
		self.initialize_state_machine()
		
		if self.debug:
			print "PackML Statemachine Node initialized"
			print '\t' + 'Logger topic:', log_topic
			print '\t' + str(len(self.states)), 'states initialized'
			print '\t' + str(len(self.commands)), 'commands initialized'

		rospy.spin()
		
	def get_state_key(self, name):
		for key in self.states:
			if self.states[key]['name'] == name:
				return key
		return false
		
	def get_command_key(self, name):
		for key in self.commands:
			if self.commands[key]['name'] == name:
				return key
		return false
		
		
	def add_allowed_commands(self, state_name, command_names):
		state_key = self.get_state_key(state_name)
		for command in command_names:
			command_key = self.get_command_key(command)
			if command_key not in self.states[state_key]['allowed_commands']:
				self.states[state_key]['allowed_commands'].append(command_key)
				
	def add_transition(self, state_name, next_state_name, command_name):
		state_key = self.get_state_key(state_name)
		next_state_key = self.get_state_key(next_state_name)
		command_key = self.get_command_key(command_name)
		transition = {'command': command_key, 'next_state': next_state_key}
		self.states[state_key]['transitions'].append(transition)
	
	def initialize_state_machine(self):
		
		# States
		self.states = {
			0: {'name': 'stopped'},
			1: {'name': 'starting'},
			2: {'name': 'idle'},
			3: {'name': 'suspending'},
			4: {'name': 'suspended'},
			5: {'name': 'un-suspending'},
			6: {'name': 'execute'},
			7: {'name': 'stopping'},
			8: {'name': 'aborting'},
			9: {'name': 'aborted'},
			10: {'name': 'holding'},
			11: {'name': 'held'},
			12: {'name': 'un-holding'},
			13: {'name': 'completing'},
			14: {'name': 'complete'},
			15: {'name': 'resetting'},
			16: {'name': 'clearing'}
		}
		
		# Commands
		self.commands = {
			0: {'name': 'start'},
			1: {'name': 'complete'},
			2: {'name': 'reset'},
			3: {'name': 'hold'},
			4: {'name': 'un-hold'},
			5: {'name': 'suspend'},
			6: {'name': 'un-suspend'},
			7: {'name': 'clear'},
			8: {'name': 'stop'},
			9: {'name': 'abort'}
		}
		
		# Allowed commands
		for key in self.states:
			self.states[key]['allowed_commands'] = []
			
		self.add_allowed_commands('stopped', ['reset', 'abort'])
		self.add_allowed_commands('starting', ['stop', 'abort'])
		self.add_allowed_commands('idle', ['start', 'stop', 'abort'])
		self.add_allowed_commands('suspending', ['stop', 'abort'])
		self.add_allowed_commands('suspended', ['un-suspend', 'stop', 'abort'])
		self.add_allowed_commands('un-suspending', ['stop', 'abort'])
		self.add_allowed_commands('execute', ['complete', 'hold', 'suspend', 'stop', 'abort'])
		self.add_allowed_commands('stopping', ['abort'])
		self.add_allowed_commands('aborting', [])
		self.add_allowed_commands('aborted', ['clear'])
		self.add_allowed_commands('holding', ['stop', 'abort'])
		self.add_allowed_commands('held', ['un-hold', 'abort'])
		self.add_allowed_commands('un-holding', ['stop', 'abort'])
		self.add_allowed_commands('completing', ['stop', 'abort'])
		self.add_allowed_commands('complete', ['reset', 'stop', 'abort'])
		self.add_allowed_commands('resetting', ['stop', 'abort'])
		self.add_allowed_commands('clearing', ['abort'])
		
		# Transitions
		for key in self.states:
			self.states[key]['transitions'] = []
			
		self.add_transition('idle', 'starting', 'start')
		self.add_transition('idle', 'stopping', 'stop')
		self.add_transition('idle', 'aborting', 'abort')
		self.add_transition('starting', 'stopping', 'stop')
		self.add_transition('starting', 'aborting', 'abort')
		self.add_transition('execute', 'completing', 'complete')
		self.add_transition('execute', 'holding', 'hold')
		self.add_transition('execute', 'suspending', 'suspend')
		self.add_transition('execute', 'stopping', 'stop')
		self.add_transition('execute', 'aborting', 'abort')
		self.add_transition('completing', 'stopping', 'stop')
		self.add_transition('completing', 'aborting', 'abort')
		self.add_transition('complete', 'stopping', 'stop')
		self.add_transition('complete', 'aborting', 'abort')
		self.add_transition('complete', 'resetting', 'reset')
		self.add_transition('resetting', 'stopping', 'stop')
		self.add_transition('resetting', 'aborting', 'abort')
		self.add_transition('holding', 'stopping', 'stop')
		self.add_transition('holding', 'aborting', 'abort')
		self.add_transition('held', 'stopping', 'stop')
		self.add_transition('held', 'aborting', 'abort')
		self.add_transition('held', 'un-holding', 'un-hold')
		self.add_transition('un-holding', 'stopping', 'stop')
		self.add_transition('un-holding', 'aborting', 'abort')
		self.add_transition('suspended', 'un-suspending', 'un-suspend')
		self.add_transition('suspended', 'stopping', 'stop')
		self.add_transition('suspended', 'aborting', 'abort')
		self.add_transition('un-suspending', 'stopping', 'stop')
		self.add_transition('un-suspending', 'aborting', 'abort')
		self.add_transition('stopping', 'aborting', 'abort')
		self.add_transition('stopped', 'resetting', 'reset')
		self.add_transition('stopped', 'aborting', 'abort')
		self.add_transition('aborted', 'clearing', 'clear')
		self.add_transition('clearing', 'aborting', 'abort')
		
		# Complete transitions: Only for -ing states (almost)
		self.states[self.get_state_key('starting')]['complete_transition'] = self.get_state_key('execute')
		self.states[self.get_state_key('completing')]['complete_transition'] = self.get_state_key('complete')
		self.states[self.get_state_key('resetting')]['complete_transition'] = self.get_state_key('idle')
		self.states[self.get_state_key('holding')]['complete_transition'] = self.get_state_key('held')
		self.states[self.get_state_key('un-holding')]['complete_transition'] = self.get_state_key('execute')
		self.states[self.get_state_key('suspending')]['complete_transition'] = self.get_state_key('suspended')
		self.states[self.get_state_key('un-suspending')]['complete_transition'] = self.get_state_key('execute')
		self.states[self.get_state_key('stopping')]['complete_transition'] = self.get_state_key('stopped')
		self.states[self.get_state_key('stopped')]['complete_transition'] = self.get_state_key('idle')
		self.states[self.get_state_key('aborting')]['complete_transition'] = self.get_state_key('aborted')
		self.states[self.get_state_key('clearing')]['complete_transition'] = self.get_state_key('stopped')


# Main loop
if __name__ == '__main__':
    try:
        packml_statemachine();
    except rospy.ROSInterruptException: pass