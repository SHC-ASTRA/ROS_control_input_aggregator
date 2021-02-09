import rospy, time
from channel import Channel
from control_input_aggregator.msg import ControlInput, CreateChannel

class ControlInputAggregator():
	"""
	Class for handling control input to drive motors for the rover
	"""
	def __init__(self):
		self.channels = {}
		self.override_channels = []
		
		rospy.loginfo("Initializing CIA node...")
		
		rospy.init_node("control_input_aggregator")
		self.ros_control_input_sub = rospy.Subscriber("/control_input", ControlInput, queue_size=30)
		
		rospy.loginfo("CIA startup complete!")