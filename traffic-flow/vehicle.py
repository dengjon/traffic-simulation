from models import *
import copy


class Vehicle(object):
	def __init__(self, vehicle_id: int, init_speed: int, init_lane: Lane,
	             init_pos: int, init_acc: float, **settings) -> None:
		# initial state
		self.id = vehicle_id
		self.speed = init_speed
		self.lane = init_lane
		self.position = init_pos
		self.acc = init_acc

		# Properties to update
		self.front_vehicle = None
		self.rear_vehicle = None

		# Parameters to overwrite
		self.lane_change_indicator = False
		self.type = ''

		# fetch from settings
		self.desired_speed_main = settings.get('desired_speed_main', 25)  # default desired speed in main lane
		self.desired_speed_ramp = settings.get('desired_speed_ramp', 16.66)  # default desired speed in ramp
		self.max_speed = settings.get('max_speed', 120)  # default max speed
		self.min_speed = settings.get('min_speed', 0)
		self.jam_distance = settings.get('jam_distance', 10)  # default safety distance
		self.min_gap = settings.get('min_gap', 0)
		self.max_acc = settings.get('max_acc', 2)
		self.desired_dec = settings.get('desired_dec', 3)
		self.reaction_time = settings.get('reaction_time', 0)
		self.length = settings.get('length', 5)
		self.target_pos = settings.get('target_pos', 0)

	def get_acceleration(self) -> float:
		"""
		Calculates the acceleration of the vehicle based on the Intelligent Driver Model (IDM).

		:return: The acceleration of the vehicle in meters per second squared.
		"""
		raise NotImplemented

	def update(self, time_step: float):
		"""
		Updates the position and speed of the vehicle based on the provided acceleration and time step.

		:param time_step: The time step of the simulation in seconds.
		"""
		acc = self.get_acceleration()
		self.speed += acc * time_step
		self.position += self.speed * time_step


class HV(Vehicle):
	def __init__(self, vehicle_id: int, init_speed: int, init_lane: Lane,
	             init_pos: int, init_acc: float, **settings) -> None:
		super().__init__(vehicle_id, init_speed, init_lane, init_pos, init_acc, **settings["global"])
		self.type = 'HV'

	def get_acceleration(self) -> float:
		"""
		Calculates the acceleration of the vehicle based on the Intelligent Driver Model (IDM).

		:return: The acceleration of the vehicle in meters per second squared.
		"""
		v = self.speed
		s = self.front_vehicle.position - self.position - self.front_vehicle.length if self.front_vehicle else np.inf
		v_lead = self.front_vehicle.speed if self.front_vehicle else None

		# The ego vehicle has a front vehicle
		if v_lead is not None:

			# change later
			if self.lane.type == 'Main':
				idm = IDM(self.desired_speed_main, self.reaction_time, self.max_acc, self.desired_dec,
				          self.jam_distance)
			else:
				idm = IDM(self.desired_speed_ramp, self.reaction_time, self.max_acc, self.desired_dec,
				          self.jam_distance)
			acc = idm.get_acceleration(v, v_lead, s)
		else:
			# The ego vehicle has no front vehicle
			if self.lane.type == 'Main':
				acc = self.max_acc(1 - (v / self.desired_speed_main) ** 4)
			else:
				acc = self.max_acc(1 - (v / self.desired_speed_ramp) ** 4)
		if acc > self.max_acc:
			acc = self.max_acc
		elif acc < -self.desired_dec:
			acc = -self.desired_dec
		return acc


class CAV(Vehicle):
	def __init__(self, vehicle_id: int, init_speed: int, init_lane: Lane,
	             init_pos: int, init_acc: float, **settings) -> None:
		super().__init__(vehicle_id, init_speed, init_lane, init_pos, init_acc, **settings)
		self.type = 'CAV'

	def get_acceleration(self) -> float:
		"""
		Calculate acceleration based on connected IDM car-following model
		:return:
		"""
		# TODO: Complete the car-following model in `models.py`
		pass


class Truck(Vehicle):
	def __init__(self, vehicle_id: int, init_speed: int, init_lane: Lane,
	             init_pos: int, init_acc: float, **settings) -> None:
		super().__init__(vehicle_id, init_speed, init_lane, init_pos, init_acc, **settings)
		self.type = 'Truck'
