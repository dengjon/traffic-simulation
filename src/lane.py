from typing import Optional
import unittest
from abc import ABC, abstractmethod


class Lane(ABC):
	def __init__(self, length, start, end, **settings):
		"""
		Initializes the properties of a lane.

		:param length: Length of the lane
		:param max_speed: Maximum speed allowed on the lane
		:param start: Start point of the lane
		:param end: End point of the lane
		:param left_lane: The neighboring lane to the left
		:param right_lane: The neighboring lane to the right
		:param capacity: Maximum number of vehicles that can be on the lane
		"""
		self.length = length
		self.type = ''
		self.max_speed = settings.get('max_speed', 120 / 3.6)
		self.min_speed = settings.get('min_speed', 0 / 3.6)
		self.capacity = settings.get('capacity', 1600)
		self.start = start
		self.end = end
		self.left_lane: Optional[Lane] = None
		self.right_lane: Optional[Lane] = None
		self.fleet = None


class MainLane(Lane):
	def __init__(self, length, start, end, **settings):
		super().__init__(length, start, end, **settings)
		self.type = 'Main'


class Ramp(Lane):
	def __init__(self, length, start, end, **settings):
		super().__init__(length, start, end, **settings)
		self.type = 'Ramp'


if __name__ == '__main__':
	unittest.main()
