import numpy as np

from models import *
import copy
from road import Lane
from typing import Optional


class Vehicle(object):
	cnt = 0

	def __init__(self, init_speed: int, init_lane: Optional[Lane],
	             init_pos: int, init_acc: float, **settings) -> None:
		# initial state
		self.id = Vehicle.cnt
		self.speed = init_speed
		self.lane = init_lane
		self.position = init_pos
		self.acc = init_acc

		# Properties to update
		self.desired_speed = 25
		self.front_vehicle: Optional[Vehicle] = None
		self.rear_vehicle: Optional[Vehicle] = None
		self.platoon = None
		self.position_record = []
		self.speed_record = []
		self.acc_record = []
		self.in_platoon = False
		self.obstacle_position: float = float('inf')

		# Parameters to overwrite
		self.lane_change_indicator = False
		self.type = ''

		# fetch from settings
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
		self.position += self.speed * time_step + 0.5 * acc * time_step ** 2

		# Update the speed of the vehicle based on the provided acceleration and time step
		self.speed += acc * time_step
		self.acc = acc

		# Ensure the speed is within the valid range
		if self.speed < 0:
			self.speed = 0  # Set the speed to 0 if it becomes negative
		elif self.speed > self.lane.max_speed:
			self.speed = self.lane.max_speed  # Limit the speed to the maximum speed of the lane

		self._restore_states()

	def move_to_lane(self, front_vehicle, new_lane: Lane):
		"""
		Move the vehicle to a new lane

		:param front_vehicle: the front vehicle in the new lane
		:param new_lane: the new lane to move to
		"""
		if front_vehicle.lane != new_lane:
			# This circumstance happens when the front vehicle moves to other lanes \
			# before lane changing of ego vehicle
			# If the front vehicle is not in the new lane, get new front vehicle in the target lane
			front_vehicle = self.get_adjacent_front_vehicle(new_lane)

		# Remove the vehicle from its current lane
		self.lane.fleet.remove(self)

		# Add the vehicle to the new lane
		new_lane.fleet.append(self, front_vehicle)

		# Update the vehicle's lane attribute to the new lane
		self.lane = new_lane

	def get_adjacent_front_vehicle(self, target_lane: Lane):
		"""
		Get the front vehicle in the target lane which the target vehicle in the current lane is going to follow
		after lane changing.

		:param target_lane: The target lane that the target vehicle is going to change into.
		:return: The lead vehicle in the target lane or None if no lead vehicle is found.
		"""

		# Initialize the lead vehicle and minimum distance to infinity and target vehicle to None
		lead_vehicle = None
		min_distance = float('inf')

		# Loop through all the vehicles in the target lane
		for vehicle in target_lane.fleet:

			# Calculate the relative distance between the current vehicle and the target vehicle
			relative_dist = vehicle.position - self.position

			# If the relative distance is greater than 0 and less than the minimum distance so far
			if 0 < relative_dist < min_distance:
				# Update the minimum distance and the lead vehicle
				min_distance = relative_dist
				lead_vehicle = vehicle

		# Return the lead vehicle or None if no lead vehicle is found
		return lead_vehicle

	def _restore_states(self):
		"""
		Restore the states of the vehicle to the previous step
		"""
		self.position_record.append(self.position)
		self.speed_record.append(self.speed)
		self.acc_record.append(self.acc)

	def _get_acceleration_obstacle(self):
		"""
		Calculate the acceleration caused by obstacles, such as road merge, traffic lights, and ramps.
		The acceleration caused by obstacles is calculated based on the IDM model.
		The acceleration will be used to compare with car-following acceleration to determine the final acceleration.
		"""
		acc_obstacle = float('inf')
		# The distance between the vehicle and the obstacle can be very small, so we need to avoid zero division
		min_distance = 1  # Placeholder of the minimum distance between the vehicle and the obstacle
		v0 = 0  # The speed of the obstacle is assumed to be 0

		# Calculate the desired distance between the vehicle and the obstacle
		s_star = min_distance + max(0, self.speed * self.reaction_time + self.speed * (self.speed - v0) /
		                            (2 * np.sqrt(self.max_acc * self.desired_dec)))

		# Calculate the distance between the vehicle and the obstacle. Vehicle length is not considered here.
		obstacle_distance = self.obstacle_position - self.position

		# If the distance between the vehicle and the obstacle less than the desired distance
		if obstacle_distance < s_star:
			acc_obstacle = self.max_acc * (1 - (self.speed / self.desired_speed) ** 4 -
			                               (s_star / max(obstacle_distance, min_distance)) ** 2)

		acc_obstacle = max(acc_obstacle, -self.desired_dec)

		return acc_obstacle


class HV(Vehicle):
	def __init__(self, init_speed: int, init_lane: Optional[Lane],
	             init_pos: int, init_acc: float, **settings) -> None:
		super().__init__(init_speed, init_lane, init_pos, init_acc, **settings)
		self.type = 'HV'

	def get_acceleration(self) -> float:
		"""
		Calculates the acceleration of the vehicle based on the Intelligent Driver Model (IDM).

		:return: The acceleration of the vehicle in meters per second squared.
		"""

		v_ego = self.speed
		pos_ego = self.position

		idm = IDM(self.desired_speed, self.reaction_time, self.max_acc,
		          self.desired_dec, self.jam_distance)

		if self.front_vehicle is not None:
			v_lead = self.front_vehicle.speed
			pos_lead = self.front_vehicle.position
			veh_length_lead = self.front_vehicle.length

			acc = idm.get_acceleration(v_ego, pos_ego, v_lead, pos_lead, veh_length_lead)
		else:
			acc = idm.get_acceleration(v_ego, pos_ego)

		acc_obstacle = self._get_acceleration_obstacle()

		acc = min(acc, acc_obstacle)

		return acc


class CAV(Vehicle):
	def __init__(self, init_speed: int, init_lane: Optional[Lane],
	             init_pos: int, init_acc: float, **settings) -> None:
		super().__init__(init_speed, init_lane, init_pos, init_acc, **settings)
		self.type = 'CAV'

	def get_acceleration(self) -> float:
		"""
		Calculate acceleration based on connected IDM car-following model
		:return:
		"""
		# TODO: Complete the car-following model in `models.py`
		pass


class Truck(Vehicle):
	def __init__(self, init_speed: int, init_lane: Optional[Lane],
	             init_pos: int, init_acc: float, **settings) -> None:
		super().__init__(init_speed, init_lane, init_pos, init_acc, **settings)
		self.type = 'Truck'
