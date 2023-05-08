from typing import Union, Any
from Platoon import Platoon
import numpy as np
from Lane import *
import copy


class Vehicle(object):
	def __init__(self, vehicle_id: int, init_speed: int, init_lane: Lane,
	             init_pos: int, init_acc: float, **settings) -> None:
		# initial state
		self.id = vehicle_id
		self.speed = init_speed
		self.lane = init_lane
		self.pos = init_pos
		self.acc = init_acc
		self.params = settings
		# global values
		param_global = settings["global"]
		self.safe_lc_acc = param_global["safe-lc-acc"]
		self.mobile_lc = param_global["mobile-lc"]
		self.no_lc = param_global["no-lc"]
		self.cannot_lc = param_global["cannot-lc"]
		# Properties to update
		self.front_veh = None
		self.after_veh = None
		# Parameters to overwrite
		self.reaction_time = 0  # reaction time
		self.len = 0  # vehicle len
		self.speed_min = 0  # speed lower bound
		self.type = ''  # vehicle type
		self.target_pos = 0  # target position to get off the lane

	def cal_acc(self):
		raise NotImplementedError

	def lane_changing(self):
		raise NotImplementedError

	def __find_spacing(self, target_platoon: Platoon):
		"""
		Find available spacing in the target platoon and join it
		:param target_platoon:
		:return: target vehicle to follow
		"""
		self.front_veh = None
		self.after_veh = None
		for veh in target_platoon:
			if veh.pos < self.pos:
				# already find the available spacing, cut in
				self.front_veh = veh.front_veh
				self.after_veh = veh
				return
		# no available spacing, add to the last of platoon
		target_platoon.append(self)
		return

	def __speed_match(self, target_veh) -> bool:
		"""
		测试换道前后的车辆速度是否匹配，即不需要剧烈的减速
		:param target_veh:
		:return:
		"""
		res = True
		# create fake vehicles for testing (current lane)
		ego_veh: Vehicle = copy.deepcopy(self)
		after_veh_curr: Vehicle = copy.deepcopy(self.after_veh)
		after_veh_curr.front_veh = ego_veh
		ego_veh.front_veh = self.front_veh
		# acc before lc in current lane
		res = res and self.__is_safe(after_veh_curr.cal_acc())
		# acc after lc in current lane
		after_veh_curr.front_veh = self.front_veh
		res = res and self.__is_safe(after_veh_curr.cal_acc())

		# create fake vehicles for testing (target lane)
		front_veh_target: Vehicle = copy.deepcopy(target_veh)
		ego_veh.lane = front_veh_target.lane
		ego_veh.front_veh = front_veh_target
		res = res and self.__is_safe(ego_veh.cal_acc())
		if front_veh_target.after_veh is not None:
			after_veh_target: Vehicle = copy.deepcopy(front_veh_target.after_veh)
			after_veh_target.front_veh = ego_veh
			res = res and self.__is_safe(after_veh_target.cal_acc())
		else:
			pass
		return res

	def mandatory_lc(self, target_platoon: Platoon):
		pass

	def discretionary_lc(self, target_platoon: Platoon):
		pass

	def __is_safe(self, acc) -> bool:
		"""
		make a judgement on the safety of lc behavior according to the
		 acceleration of ego vehicle and following vehicle after lane-changing.
		"""
		if acc > self.safe_lc_acc:
			return True
		else:
			return False


class HV(Vehicle):
	def __init__(self, vehicle_id: int, init_speed: int, init_lane: Lane,
	             init_pos: int, init_acc: float, **settings) -> None:
		super().__init__(vehicle_id, init_speed, init_lane, init_pos, init_acc, **settings["global"])
		self.type = 'HV'
		self.len = 4
		self.speed_min = 0
		self.reaction_time = 1.5
		self.settings = settings["HV"]

	def cal_acc(self):
		v = self.speed
		a = self.settings["max-acc"]
		b = self.settings["desire-dec"]
		s0 = self.settings["jam-dist"]
		if self.lane.type == 'ramp':
			v0 = self.settings["desire-speed-ramp"]
		else:
			v0 = self.settings["desire-speed-main"]
		if self.front_veh is not None:
			if self.front_veh.type == 'HV':
				T = self.settings["time-gap-hv"]
			elif self.front_veh.type == 'CAV':
				T = self.settings["time-gap-cav"]
			else:
				raise ValueError(-1)
			# Calculate acceleration
			delta_v = v - self.front_veh.speed
			s = max(0.01, - self.pos + self.front_veh.pos - self.front_veh.len)
			if s < 0.02:
				# 距离太近，撞车了
				return -100
			s_star = s0 + max(0, v * T + v * delta_v *
			                  0.5 / (a * b) ** 0.5)
			acc = a * (1 - (v / v0) ** 4 - (s_star / s) ** 2)
		else:
			acc = a * (1 - (v / v0) ** 4)
		return acc


class CAV(Vehicle):
	def __init__(self, vehicle_id: int, init_speed: int, init_lane: Lane,
	             init_pos: int, init_acc: float, **settings) -> None:
		super().__init__(vehicle_id, init_speed, init_lane, init_pos, init_acc)
		self.type = 'CAV'
		self.len = 4
		self.reaction_time = 0
		self.settings = settings["CAV"]

	def cal_acc(self):
		v = self.speed
		a = self.settings["max-acc"]
		b = self.settings["desire-dec"]
		s0 = self.settings["jam-dist"]
		if self.lane.id == 4:
			v0 = self.settings["desire-speed-ramp"]
		else:
			v0 = self.settings["desire-speed-main"]
		if self.front_veh is not None:
			if self.front_veh.type == 'HV':
				T = self.settings["time-gap-hv"]
			elif self.front_veh.type == 'CAV':
				T = self.settings["time-gap-cav"]
			else:
				raise ValueError(-1)
		if self.front_veh is not None:
			if self.front_veh.front_veh is not None:
				delta_v1 = self.speed - self.front_veh.speed
				delta_v2 = self.front_veh.speed - self.front_veh.front_veh.speed
				s1 = max(0.01, self.front_veh.pos - self.pos - self.front_veh.len)
				s2 = max(0.01, self.front_veh.front_veh.pos - self.front_veh.pos -
				         self.front_veh.front_veh.len)
				delta1 = abs(delta_v1) / (s1 ** 0.5)
				delta2 = abs(delta_v2) / (s2 ** 0.5)
				try:
					lambda1 = delta1 / (delta1 + delta2)
					lambda2 = delta2 / (delta1 + delta2)
				except ZeroDivisionError:
					lambda1 = 0.5
					lambda2 = 0.5
				delta_v = lambda1 * delta_v1 + lambda2 * delta_v2
				s = lambda1 * s1 + lambda2 * s2
				if (s1 < 0.02) | (s < 0.02):
					return -100
			else:
				delta_v = self.speed - self.front_veh.speed
				s = max(0.01, -self.pos + self.front_veh.pos - self.front_veh.len)
				if s < 0.02:
					return -100
			s_star = s0 + max(0, v * T + v * delta_v * 0.5 / (a * b) ** 0.5)
			acc = a * (1 - (v / v0) ** 4 - (s_star / s) ** 2)
		else:
			acc = a * (1 - (v / v0) ** 4)
		return acc
