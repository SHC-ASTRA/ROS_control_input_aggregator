from time import time


class Channel:
	refresh_rate = 0.2 # how long the rover should act on a control input
	
	def __init__(self):
		self.heading = (0, 0)
		self.speed_clamp = 1
		self.is_urgent = False
		self.timestamp = 0
		
	def process_message(self, message):
		self.heading = message.heading
		self.speed_clamp = message.speed_clamp
		self.is_urgent = message.is_urgent
		self.timestamp = time()
	
	def is_urgent(self):
		return self.is_urgent
	
	def is_active(self):
		return time() - self.timestamp < Channel.refresh_rate
		
	def read(self):
		return self.heading, self.speed_clamp
	