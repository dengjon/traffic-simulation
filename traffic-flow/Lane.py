class Lane(object):
	"""
	定义道路的基本属性
	"""

	def __init__(self, lane_id: int, lane_len: int, lane_type: str,
	             start: int, end: int) -> None:
		self.id = lane_id
		self.len = lane_len
		self.type = lane_type
		self.start = start
		self.end = end
		self.left = None
		self.right = None
		self.platoon = None


class MainLane(Lane):
	def __init__(self, lane_id, lane_len, start, end) -> None:
		super().__init__(lane_id, lane_len, 'main', start, end)


class Ramp(Lane):
	def __init__(self, lane_id, lane_len, start, end) -> None:
		super().__init__(lane_id, lane_len, 'ramp', start, end)
		self.ramp_in = None
		self.weaving = None
		self.ramp_out = None
