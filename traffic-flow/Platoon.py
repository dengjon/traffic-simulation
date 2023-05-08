from Vehicle import *


class Platoon(object):
	def __init__(self, platoon_id: int, lane: Lane) -> None:
		self.id = platoon_id
		self.lane = lane
		# First and last vehicle in a platoon
		self.first_veh = None
		self.last_veh = None
		self.platoon_len = 0
		self.error_route = 0

	def append(self, veh: Vehicle):
		"""
		append new vehicle into the platoon
		:param veh: target vehicle to be added
		:return: None
		"""
		if self.platoon_len > 0:
			# Add new vehicle at the tail of the platoon
			last_veh = self.last_veh
			last_veh.after_veh = veh
			# Update
			veh.front_veh = last_veh
			self.last_veh = veh
		else:
			self.first_veh = veh
			self.last_veh = veh
		self.platoon_len += 1

	def update_acc(self):
		for veh in self:
			veh.acc = veh.cal_acc()
		return None

	def move(self, time_step=1):
		"""
		Update the speed of vehicles and delete vehicles out of the scenario
		:param time_step: iteration time step
		"""
		for veh in self:
			# update position and speed of each vehicle
			speed_before = veh.speed
			veh.speed = max(veh.speed_min, veh.speed + veh.acc * time_step)
			veh.pos = veh.pos + speed_before * time_step + 1 / 2 * veh.acc + time_step ** 2
			if veh.pos > veh.target_pos:
				# if the vehicle reaches its destination, delete it
				if veh.front_veh is not None:
					# not the leading vehicle
					veh.front_veh.after_veh = veh.after_veh
				else:
					# is the leading vehicle
					self.first_veh = veh.after_veh
				if veh.after_veh is not None:
					# not the last vehicle
					veh.after_veh.front_veh = veh.front_veh
				else:
					# is the last vehicle
					self.last_veh = veh.front_veh
				# update the platoon length
				self.platoon_len -= 1
		if self.first_veh is not None:
			# The first vehicle reaches the end of the road
			if self.first_veh.pos > self.lane.end:
				# if it has a following vehicle
				if self.first_veh.after_veh is not None:
					self.first_veh.after_veh.front_veh = None
					self.first_veh = self.first_veh.after_veh
				else:
					# The platoon has only one vehicle
					self.first_veh = None
					self.last_veh = None
				# update platoon length
				self.platoon_len -= 1

		pass

	def get(self, index: int) -> Vehicle:
		"""
		get vehicle from the platoon via index
		:param index: vehicle number
		:return: Vehicle
		"""
		assert 0 <= index < self.platoon_len
		veh_curr = self.first_veh
		for i in range(index):
			veh_curr = veh_curr.after_veh
		return veh_curr

	def __iter__(self):
		self.count = 0
		return self

	def __next__(self):
		# make the platoon class itertable
		while self.count < self.platoon_len:
			veh_curr = self.get(self.count)
			self.count += 1
			return veh_curr
		raise StopIteration
