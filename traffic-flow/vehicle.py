from models import *
import copy
from lane import Lane
from typing import Optional


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
		self.front_vehicle: Optional[Vehicle] = None
		self.rear_vehicle: Optional[Vehicle] = None
		self.platoon = None

		# Parameters to overwrite
		self.lane_change_indicator = False
		self.type = ''

		# fetch from settings
		self.desired_speed_main = settings.get('desired_speed_main', 25)  # default desired speed in main lane
		self.desired_speed_ramp = settings.get('desired_speed_ramp', 16.66)  # default desired speed in ramp
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

	def update(self, acc: float, time_step: float):
		"""
		Updates the position and speed of the vehicle based on the provided acceleration and time step.

		:param acc: The acceleration of the vehicle in meters per second squared.
		:param time_step: The time step of the simulation in seconds.
		"""
		# Update the position of the vehicle based on the current speed and time step
		self.position += self.speed * time_step + 0.5 * acc * time_step**2

		# Update the speed of the vehicle based on the provided acceleration and time step
		self.speed += acc * time_step

		# Ensure the speed is within the valid range
		if self.speed < 0:
			self.speed = 0  # Set the speed to 0 if it becomes negative
		elif self.speed > self.lane.max_speed:
			self.speed = self.lane.max_speed  # Limit the speed to the maximum speed of the lane

	def get_adjacent_lead_vehicle(self, target_lane: Lane):
		"""
		Get the lead vehicle in the target lane which the target vehicle in the current lane is going to follow
		after lane changing.

		:param target_lane: The target lane that the target vehicle is going to change into.
		:return: The lead vehicle in the target lane or None if no lead vehicle is found.
		"""

		# Initialize the lead vehicle and minimum distance to infinity and target vehicle to None
		lead_vehicle = None
		min_distance = float('inf')
		target_vehicle = None

		# Loop through all the vehicles in the target lane
		for vehicle in target_lane.fleet.vehicles:

			# Calculate the relative distance between the current vehicle and the target vehicle
			relative_dist = vehicle.position - self.position

			# If the relative distance is greater than 0 and less than the minimum distance so far
			if 0 < relative_dist < min_distance:
				# Update the minimum distance and the lead vehicle
				min_distance = relative_dist
				lead_vehicle = vehicle

		# Return the lead vehicle or None if no lead vehicle is found
		return lead_vehicle

	def move_to_lane(self, new_lane: Lane):
		"""
		Move the vehicle to a new lane

		:param new_lane: the new lane to move to
		"""
		# Remove the vehicle from its current lane
		self.lane.fleet.remove_vehicle(self)

		# Find the closest front vehicle on the new lane
		min_dist = float('inf')
		front_vehicle = None
		for vehicle in new_lane.fleet.vehicles:
			delta_dist = vehicle.position - self.position
			if 0 < delta_dist < min_dist:
				min_dist = delta_dist
				front_vehicle = vehicle

		# Add the vehicle to the new lane
		new_lane.fleet.add_vehicle(self, front_vehicle)

		# Update the vehicle's lane attribute to the new lane
		self.lane = new_lane


class HV(Vehicle):
	def __init__(self, vehicle_id: int, init_speed: int, init_lane: Lane,
	             init_pos: int, init_acc: float, **settings) -> None:
		super().__init__(vehicle_id, init_speed, init_lane, init_pos, init_acc, **settings)
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
				acc = self.max_acc * (1 - (v / self.desired_speed_main) ** 4)
			else:
				acc = self.max_acc * (1 - (v / self.desired_speed_ramp) ** 4)
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
