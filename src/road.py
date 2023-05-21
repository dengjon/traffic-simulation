from typing import Optional, List
import unittest


class Lane(object):
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


class Road(List[Lane]):
	def __init__(self, *args, **kwargs):
		super().__init__(*args, **kwargs)

	def append(self, __lane: Lane) -> None:
		if len(self) > 0:
			__lane.left_lane = self[-1]
			self[-1].right_lane = __lane
		super().append(__lane)

	def remove(self, __lane: Lane) -> None:
		if __lane.left_lane is not None:
			__lane.left_lane.right_lane = __lane.right_lane
		if __lane.right_lane is not None:
			__lane.right_lane.left_lane = __lane.left_lane
		super().remove(__lane)

	def get_lane_by_index(self, index: int) -> Optional[Lane]:
		if index < len(self):
			return self[index]
		return None

	def get_lane_by_type(self, lane_type: str) -> List[Lane]:
		lane_list = []
		for lane in self:
			if lane.type == lane_type:
				lane_list.append(lane)
		return lane_list


if __name__ == '__main__':
	unittest.main()
