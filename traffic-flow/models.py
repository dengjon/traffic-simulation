import math
import numpy as np
from lane import Lane
from vehicle import Vehicle


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
		delta_v = ego_vehicle_speed - lead_vehicle_speed
		s_star = self.min_distance + ego_vehicle_speed * self.time_headway + ego_vehicle_speed * delta_v / (
				2 * math.sqrt(self.max_acceleration * self.min_distance))

		acceleration = self.max_acceleration * (
				1 - (ego_vehicle_speed / self.desired_speed) ** 4 - (s_star / lead_vehicle_distance) ** 2)

		return acceleration


class MOBIL(object):
	def __init__(self, politeness_factor: float = 0.5, acceleration_difference_threshold: float = 0.2):
		"""
		Initializes the MOBIL model with the given politeness factor and acceleration difference threshold.

		:param politeness_factor: a float between 0 and 1 that determines the probability of changing lanes
		                           based on the benefit to other vehicles (default: 0.5)
		:param acceleration_difference_threshold: a float that represents the minimum acceleration difference
		                                           between the current and target lanes for a lane change
		                                           (default: 0.2)
		"""
		self.politeness_factor = politeness_factor
		self.acceleration_difference_threshold = acceleration_difference_threshold

	def can_change_lane(self, vehicle: Vehicle, current_lane: Lane, adjacent_lane: Lane, dt: float) -> bool:
		"""
		Checks whether a vehicle can change lanes based on the MOBIL (Minimizing Overall Braking Induced by Lane changes) model

		:param vehicle: The vehicle to check for a lane change
		:param current_lane: The current lane of the vehicle
		:param adjacent_lane: The adjacent lane to check for the lane change
		:param dt: The time step of the simulation
		:return: True if the vehicle can change lanes, False otherwise
		"""

		# Check if adjacent lane is valid and has enough space for the vehicle to change into it
		if adjacent_lane is None or not adjacent_lane.is_space_for_vehicle(vehicle):
			return False

		# Calculate acceleration in current lane using IDM model
		current_acc = current_lane.get_vehicle_acceleration(vehicle, dt)

		# Calculate acceleration in adjacent lane using IDM model
		adjacent_acc = adjacent_lane.get_vehicle_acceleration(vehicle, dt)

		# Calculate potential gain in overall acceleration using MOBIL model
		potential_gain = self.get_potential_gain(vehicle, current_lane, adjacent_lane, dt)

		# If potential gain is greater than threshold, return True to indicate lane change
		if potential_gain > self.politeness_factor:
			return True

		# If potential gain is not greater than threshold, return False to indicate no lane change
		return False

	def get_potential_gain(self, vehicle: Vehicle, current_lane: Lane, adjacent_lane: Lane, dt: float) -> float:
		# Calculate the travel time of the vehicle in the current lane
		current_travel_time = current_lane.get_travel_time(vehicle)

		# Calculate the travel time of the vehicle in the adjacent lane
		new_travel_time = adjacent_lane.get_travel_time(vehicle)

		# Calculate the travel time of the vehicle in the current lane if it were to follow the vehicle in front of it
		if vehicle.front_vehicle is not None:
			current_following_travel_time = current_lane.get_travel_time(vehicle.front_vehicle)
		else:
			current_following_travel_time = current_travel_time

		# Calculate the travel time of the vehicle in the adjacent lane if it were to follow the vehicle in front of it
		if vehicle.front_vehicle is not None:
			new_following_travel_time = adjacent_lane.get_travel_time(vehicle.front_vehicle)
		else:
			new_following_travel_time = new_travel_time

		# Calculate the expected improvement in travel time
		expected_gain = current_travel_time - new_travel_time - self.mobil_accel_gain * (
				current_following_travel_time - new_following_travel_time)

		return expected_gain

	def get_lane_change_direction(self, vehicle: Vehicle, current_lane: Lane, left_lane: Lane, right_lane: Lane,
	                              dt: float) -> str:
		"""
		Determines the direction of the lane change (left, right, or none) based on the MOBIL model

		:param vehicle: The vehicle to check for a lane change
		:param current_lane: The current lane of the vehicle
		:param left_lane: The lane to the left of the current lane
		:param right_lane: The lane to the right of the current lane
		:param dt: The time step of the simulation
		:return: The direction of the lane change, either 'left', 'right', or 'none'
		"""
		# Check if changing lanes to the left lane is possible
		if left_lane and self.can_change_lane(vehicle, current_lane, left_lane, dt):
			return 'left'

		# Check if changing lanes to the right lane is possible
		if right_lane and self.can_change_lane(vehicle, current_lane, right_lane, dt):
			return 'right'

		return 'none'
