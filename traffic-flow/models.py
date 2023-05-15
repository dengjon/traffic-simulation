import math
import numpy as np
import copy
from typing import List, Union, Optional, Tuple


class IDM(object):
	def __init__(self, v0, T, a, b, s0):
		"""
		Initializes the IDM model with the specified parameters.

		Args:
			v0 (float): Desired speed in free traffic.
			T (float): Desired time headway to the vehicle in front.
			a (float): Maximum acceleration.
			b (float): Comfortable deceleration.
			s0 (float): Minimum desired net distance.
		"""
		self.v0 = v0
		self.T = T
		self.a = a
		self.b = b
		self.s0 = s0

	def get_acceleration(self, v, v_lead, s):
		"""
		Calculates the acceleration of a vehicle based on the Intelligent Driver Model (IDM).

		Args:
			v (float): Current speed of the vehicle.
			v_lead (float): Speed of the vehicle in front.
			s (float): Distance to the vehicle in front.

		Returns:
			float: Calculated acceleration.
		"""

		# Safe distance
		s_star = self.s0 + max(0, v * self.T + v * (v - v_lead) / (2 * np.sqrt(self.a * self.b)))

		# Acceleration
		acceleration = self.a * (1 - math.pow(v / self.v0, 4) - math.pow(s_star / s, 2))

		return acceleration


class ACC(object):
	def __init__(self, desired_speed=25, max_acceleration=3, min_distance=2, time_headway=1.5):
		"""
		Initializes the ACC model with default values for its parameters.

		:param desired_speed: float, desired speed of the vehicle in meters per second (default: 25 m/s)
		:param max_acceleration: float, maximum acceleration of the vehicle in meters per second squared (default: 3 m/s^2)
		:param min_distance: float, minimum distance to maintain to the vehicle ahead in meters (default: 2 m)
		:param time_headway: float, desired time headway to the vehicle ahead in seconds (default: 1.5 s)
		"""
		self.desired_speed = desired_speed
		self.max_acceleration = max_acceleration
		self.min_distance = min_distance
		self.time_headway = time_headway

	def get_acceleration(self, ego_vehicle_speed, lead_vehicle_speed, lead_vehicle_distance):
		"""
		Calculates the acceleration of an automated vehicle based on the Adaptive Cruise Control (ACC) model.

		:param ego_vehicle_speed: float, speed of the ego vehicle in meters per second
		:param lead_vehicle_speed: float, speed of the lead vehicle in meters per second
		:param lead_vehicle_distance: float, distance between the ego vehicle and the lead vehicle in meters
		:return: float, acceleration of the ego vehicle in meters per second squared
		"""
		# TODO: Complete the ACC car-following function
		pass


class MOBIL:
	def __init__(self, accel_threshold: float = 0.5, brake_threshold: float = -3.0,
	             delta: float = 0, politeness_factor: float = 0.3):
		"""
		Initializes an instance of the MOBIL (Minimizing Overall Braking Induced by Lane changes) class with the
		specified parameters.

		:param accel_threshold: The acceleration threshold for considering a vehicle to be in a "safe gap".
		:param brake_threshold: The braking threshold for considering a vehicle to be in a "safe gap".
		:param delta: MOBIL model parameter delta
		:param politeness_factor: A factor used to determine the probability of a lane change.
		"""
		self.accel_threshold = accel_threshold  # The acceleration threshold for considering a vehicle to be in a "safe gap"
		self.brake_threshold = brake_threshold  # The braking threshold for considering a vehicle to be in a "safe gap"
		self.delta = delta  # Placeholder for the MOBIL model parameter delta
		self.politeness_factor = politeness_factor  # A factor used to determine the probability of a lane change

	def can_change_lane(self, vehicle, direction: str, dt) -> bool:
		"""
		Checks whether a vehicle can change lanes based on the MOBIL (Minimizing Overall Braking Induced by Lane changes) model.

		:param vehicle: The vehicle to check for a lane change.
		:param direction: The direction in which the vehicle wants to change lanes. It must be either 'left' or 'right'.
		:param dt: The time step of the simulation.
		:return: True if the vehicle can change lanes, False otherwise.
		"""

		# Ensure that direction is valid (either 'left' or 'right')
		assert direction in ['left', 'right']

		# Get the current lane of the vehicle
		current_lane = vehicle.lane

		# Determine the lane to which the vehicle wants to change
		if direction == 'left':
			target_lane = current_lane.left_lane
		else:
			target_lane = current_lane.right_lane

		if target_lane is None:
			return False

		# Calculate the acceleration gain from the lane change
		delta_acc_lc_self, delta_acc_lc_rear = self.__calculate_acceleration_lc(vehicle, target_lane)

		# Calculate the lane change gain (how much the vehicle can benefit from the lane change)
		delta_acc_curr_rear = self.__calculate_acceleration_curr(vehicle)

		# Calculate the total incentive of the lane change
		incentive = delta_acc_lc_self + self.politeness_factor * (delta_acc_curr_rear + delta_acc_lc_rear)

		# Return whether the incentive is greater than the minimum threshold
		return incentive > self.delta * dt

	@staticmethod
	def __calculate_acceleration_lc(vehicle, adjacent_lane) -> Tuple[float, float]:
		"""
		Calculates the acceleration gain from changing lanes based on the MOBIL model.

		:param vehicle: The vehicle that is attempting to change lanes.
		:param adjacent_lane: The lane that the vehicle is attempting to switch to.
		:return: The acceleration gain from changing lanes.
		"""
		# Get the speed of the target vehicle, the vehicle immediately in front of it in the adjacent lane,
		# and the vehicle immediately behind it in the current lane.
		front_vehicle_adjacent = vehicle.get_adjacent_lead_vehicle(adjacent_lane)
		vehicle_after_lc = copy.copy(vehicle)
		delta_acc_lc_rear = 0

		if front_vehicle_adjacent is not None:
			front_vehicle = copy.copy(front_vehicle_adjacent)
			if front_vehicle.rear_vehicle is not None:
				rear_vehicle = copy.copy(front_vehicle.rear_vehicle)
				rear_vehicle.front_vehicle = vehicle_after_lc
				delta_acc_lc_rear = rear_vehicle.get_acceleration() - rear_vehicle.acc
			else:
				rear_vehicle = None
		else:
			front_vehicle = None
			rear_vehicle = None

		vehicle_after_lc.front_vehicle = front_vehicle
		vehicle_after_lc.rear_vehicle = rear_vehicle

		delta_acc_lc = vehicle_after_lc.get_acceleration() - vehicle.acc

		return delta_acc_lc, delta_acc_lc_rear

	@staticmethod
	def __calculate_acceleration_curr(vehicle) -> float:
		"""
		Calculates the gain from changing lanes to the given adjacent lane.

		:param vehicle: The vehicle that wants to change lanes.
		:return: The gain from changing lanes to the given adjacent lane.
		"""
		delta_acc_rear = 0

		if vehicle.rear_vehicle is not None:
			if vehicle.front_vehicle is not None:
				rear_vehicle = copy.copy(vehicle.front_vehicle)
				delta_acc_rear = rear_vehicle.get_acceleration() - rear_vehicle.acc

		return delta_acc_rear
