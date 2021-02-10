import rospy, time
from channel import *
from control_input_aggregator.msg import *

class ControlInputAggregator():
	"""
	Class for handling control input to drive motors for the rover
	"""
	def __init__(self):
		self.channels = {}
		
		rospy.init_node("control_input_aggregator")
		rospy.loginfo("Initializing CIA node...")
		self.ros_control_input_sub = rospy.Subscriber("/control_input", ControlInput, queue_size=30)
		
		
		self.r = rospy.Rate(30)
		rospy.loginfo("CIA startup complete!")
	
	def handle_control_input(self, control_input):
		if control_input.name not in self.channels:
			self.channels[control_input.name] = Channel()
		
		self.channels[control_input.name].process_message(control_input)
		
	def run(self):
		while not rospy.is_shutdown():
			self.r.sleep()

if __name__ == '__main__':
	cia = ControlInputAggregator()
	cia.run()