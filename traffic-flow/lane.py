from typing import Optional
import unittest
from abc import ABC, abstractmethod


class Lane(ABC):
	def __init__(self, length, start, end, lane_type,
	             left_lane=None, right_lane=None, **settings):
		"""
		Initializes the properties of a lane.

		:param length: Length of the lane
		:param max_speed: Maximum speed allowed on the lane
		:param start: Start point of the lane
		:param end: End point of the lane
		:param lane_type: Type of the lane (e.g. 'straight', 'left-turn-only', etc.)
		:param left_lane: The neighboring lane to the left
		:param right_lane: The neighboring lane to the right
		:param capacity: Maximum number of vehicles that can be on the lane
		"""
		self.length = length
		self.max_speed = settings.get('max_speed', 120 / 3.6)
		self.min_speed = settings.get('min_speed', 0 / 3.6)
		self.capacity = settings.get('capacity', 1600)
		self.start = start
		self.end = end
		self.left_lane: Optional[Lane] = left_lane
		self.right_lane: Optional[Lane] = right_lane
		self.fleet = None
		self.type = lane_type


class MainLane(Lane):
	def __init__(self, length, start, end, lane_type,
	             left_lane=None, right_lane=None, **settings):
		super().__init__(length, start, end, lane_type,
		                 left_lane, right_lane, **settings)
		self.type = 'Main'


class Ramp(Lane):
	def __init__(self, length, start, end, lane_type,
	             left_lane=None, right_lane=None, **settings):
		super().__init__(length, start, end, lane_type,
		                 left_lane, right_lane, **settings)
		self.type = 'Ramp'


if __name__ == '__main__':
	unittest.main()
