#!/usr/bin/python

import rospy
from control_input_aggregator.msg import *
from time import time


class Channel:
	"""
	Control Input Channel class.
	
	Helper class for the ControlInputAggregator(CIA).
	This class stores information on input channels. It processes `ControlInput` messages.
	"""
	
	refresh_rate = 0.2
	"""Time in seconds before a `ControlInput` expires"""
	
	def __init__(self):
		"""
		Initializes a new Channel class. Generates default values to store `ControlInput` messages.
		"""
		self.heading = [0, 0]
		self.speed_clamp = 1
		self.urgent = False
		self.timestamp = 0
		
	def process_message(self, message):
		"""
		Unpacks all the values from a received message and stores them.
		
		:param message: ControlInput object
		"""
		self.heading = list(message.heading)
		self.speed_clamp = message.speed_clamp
		self.urgent = message.is_urgent
		self.timestamp = time()
	
	def is_urgent(self):
		"""
		This function returns true if the previously received ControlInput was marked urgent.
		"""
		return self.urgent
	
	def is_active(self):
		"""
		This function returns true if the time since the previously received ControlInput is less than the specified `Channel.refresh_rate`.
		"""
		return time() - self.timestamp < Channel.refresh_rate


class ControlInputAggregator():
	"""
	Class for handling control input to drive motors for the rover
	"""
	def __init__(self):
		"""
		Initializes the CIA and creates a ROS Node.
		"""
		self.channels = {}
		
		rospy.init_node("control_input_aggregator")
		rospy.loginfo("Initializing CIA node...")
		self.control_input_sub = rospy.Subscriber("/control_input", ControlInput, self.handle_control_input, queue_size=30)
		
		self.control_out_pub = rospy.Publisher("/control_output", ControlInput, queue_size=1)
		
		self.r = rospy.Rate(30)
		rospy.loginfo("CIA startup complete!")
	
	def handle_control_input(self, control_input):
		"""
		This function routes `ControlInput` messages to their respective `Channel` objects and creates new `Channel` objects for new received channels.
		"""
		if control_input.channel not in self.channels:
			self.channels[control_input.channel] = Channel()
		
		self.channels[control_input.channel].process_message(control_input)
	
	def aggregate_channels(self):
		"""
		This function loops through all the active `Channel` objects and calculates the overall direction of the rover.
		Urgent `ControlInput`s will short circuit the loop and publish the response immediately.
		The final heading is normalized to be standardized for any nodes down the processing chain.
		"""
		heading = [0, 0]
		speed_clamp = 1.0
		for channel in self.channels.values():
			if channel.is_active():
				if channel.is_urgent():
					heading = channel.heading
					speed_clamp = channel.speed_clamp
					break
				heading[0] += channel.heading[0]
				heading[1] += channel.heading[1]
				if channel.speed_clamp < speed_clamp:
					speed_clamp = channel.speed_clamp
		
		heading_magnitude = ((heading[0]**2)+(heading[1]**2))**0.5
		if heading_magnitude != 0: # Normalize if the magnitude is more than 0
			heading[0] /= heading_magnitude
			heading[1] /= heading_magnitude
		self.control_out_pub.publish("aggregate", heading, speed_clamp, True)
	
	def run(self):
		"""
		This function defines the main loop behavior for the CIA. Each turn, the CIA will aggregate all `Channel`s and publish a `ControlInput` message with the calculated output.
		This loop runs at the frequency(rate) defined in the init function for this class.
		"""
		while not rospy.is_shutdown():
			self.aggregate_channels()
			self.r.sleep()


if __name__ == '__main__':
	cia = ControlInputAggregator()
	cia.run()