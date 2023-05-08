import numpy as np
from Lane import *


class Vehicle(object):
	def __init__(self, vehicle_id: int, init_speed: int, init_lane: Lane,
	             init_pos: int, init_acc: float) -> None:
		# initial state
		self.id = vehicle_id
		self.speed = init_speed
		self.lane = init_lane
		self.pos = init_pos
		self.acc = init_acc
		# Properties to update
		self.front_veh = None
		self.after_veh = None
		# Parameters to overwrite
		self.reaction_time = 0  # reaction time
		self.len = 0  # vehicle len
		self.min_speed = 0  # speed lower bound
		self.type = ''  # vehicle type
		self.target_pos = 0  # target position to get off the lane

	def cal_acc(self):
		pass

	def lane_changing(self):
		pass


class HV(Vehicle):
	def __init__(self, vehicle_id: int, init_speed: int, init_lane: Lane,
	             init_pos: int, init_acc: float, **settings) -> None:
		super().__init__(vehicle_id, init_speed, init_lane, init_pos, init_acc)
		self.type = 'HV'
		# init params
		self.desire_dec = 0
		self.desire_speed = 0
		self.max_acc = 0
		self.jam_dist = 0
		# update attributes
		for attr, val in settings.items():
			setattr(self, attr, val)

	def cal_acc(self):
		if self.front_veh is not None:
			T = 1.5
			# Calculate acceleration
			delta_v = self.speed - self.front_veh.speed
			s = max(0.01, - self.pos + self.front_veh.pos - self.front_veh.len)
			if s < 0.02:
				# 距离太近，撞车了
				return -100
			s_star = self.jam_dist + max(0, self.speed * T + self.speed * delta_v *
			                             0.5 / (self.max_acc * self.desire_dec) ** 0.5)
			acc = self.max_acc * (1 - (self.speed / self.desire_speed) ** 4 - (s_star / s) ** 2)
		else:
			acc = self.acc
		return acc


class CAV(Vehicle):
	def __init__(self, vehicle_id: int, init_speed: int, init_lane: Lane,
	             init_pos: int, init_acc: float, **settings) -> None:
		super().__init__(vehicle_id, init_speed, init_lane, init_pos, init_acc)
		self.type = 'CAV'
		# init params
		self.desire_dec = 0
		self.desire_speed = 0
		self.max_acc = 0
		self.jam_dist = 0
		# update attributes
		for attr, val in settings.items():
			setattr(self, attr, val)


if __name__ == '__main__':
	import json

	with open("settings.json") as f:
		params = json.load(fp=f)
	params_vehicle = params['Vehicle']
	lane = Lane(1, 3000, 'main', 0, 3000)
	a = HV(1, 1, lane, 1, 0.1, **params_vehicle["HV"])
	print(a)
