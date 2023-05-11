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
		# TODO: Complete the ACC car-following function
		pass


class MOBIL:
	def __init__(self, accel_threshold: float = 0.5, brake_threshold: float = -3.0, max_acc: float = 3.0,
	             max_dec: float = -8.0, politeness_factor: float = 0.3, lane_change_duration: float = 5.0):
		"""
		Initializes an instance of the MOBIL (Minimizing Overall Braking Induced by Lane changes) class with the
		specified parameters.

		:param accel_threshold: The acceleration threshold for considering a vehicle to be in a "safe gap".
		:param brake_threshold: The braking threshold for considering a vehicle to be in a "safe gap".
		:param max_acc: The maximum acceleration for the vehicle.
		:param max_dec: The maximum deceleration for the vehicle.
		:param politeness_factor: A factor used to determine the probability of a lane change.
		:param lane_change_duration: The duration of a lane change in seconds.
		"""
		self.accel_threshold = accel_threshold
		self.brake_threshold = brake_threshold
		self.max_acc = max_acc
		self.max_dec = max_dec
		self.beta = 0.5  # Placeholder for the MOBIL model parameter beta
		self.delta = 4.0  # Placeholder for the MOBIL model parameter delta
		self.politeness_factor = politeness_factor
		self.lane_change_duration = lane_change_duration

	def can_change_lane(self, vehicle: Vehicle, direction: str, dt: float) -> bool:
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

		# Calculate the acceleration gain from the lane change
		acceleration_gain = self._calculate_acceleration_gain(vehicle, target_lane, dt)

		# Calculate the lane change gain (how much the vehicle can benefit from the lane change)
		lane_change_gain = self._calculate_lane_change_gain(vehicle, current_lane, target_lane)

		# Calculate the total incentive of the lane change
		incentive = acceleration_gain + self.beta * lane_change_gain

		# Return whether the incentive is greater than the minimum threshold
		return incentive > self.delta

	def _calculate_acceleration_gain(self, vehicle: Vehicle, adjacent_lane: Lane, dt: float) -> float:
		"""
		Calculates the acceleration gain from changing lanes based on the MOBIL model.

		:param vehicle: the vehicle that is attempting to change lanes
		:param adjacent_lane: the lane that the vehicle is attempting to switch to
		:param dt: the time step for the simulation
		:return: the acceleration gain from changing lanes
		"""
		# Get the speed of the target vehicle, the vehicle immediately in front of it in the adjacent lane,
		# and the vehicle immediately behind it in the current lane.
		v_a = vehicle.speed
		front_vehicle_adjacent = vehicle.get_adjacent_lead_vehicle(adjacent_lane)
		if front_vehicle_adjacent is not None:
			v_b = front_vehicle_adjacent.speed
		else:
			v_b = v_a
		v_f = vehicle.rear_vehicle.speed

		# Calculate the difference in speed between the target vehicle and the vehicle behind it.
		delta_v = v_a - v_f

		# Calculate the acceleration of the target vehicle.
		a = vehicle.get_acceleration()

		# Calculate the desired acceleration for lane changing using the MOBIL model.
		if delta_v <= 0:
			# If the target vehicle is slower than the vehicle behind it, then the target vehicle should
			# attempt to accelerate to match the speed of the vehicle in front of it.
			a_star = self.accel_threshold * (v_b - v_a) + self.brake_threshold * (v_a - v_f)
		else:
			# If the target vehicle is faster than the vehicle behind it, then the target vehicle should
			# attempt to maintain a safe distance from the vehicle in front of it.
			a_star = self.accel_threshold * (v_b - v_a) + self.brake_threshold * (v_a - v_f - delta_v)

		# Cap the acceleration gain between the maximum acceleration and maximum deceleration values.
		acc_gain = max(min(a_star - a, self.max_acc), self.max_dec)

		# Scale the acceleration gain by the time step for the simulation.
		return acc_gain * dt

	def _calculate_lane_change_gain(self, vehicle: Vehicle, current_lane: Lane, adjacent_lane: Lane) -> float:
		"""
		Calculates the gain from changing lanes to the given adjacent lane.

		:param vehicle: The vehicle that wants to change lanes.
		:param current_lane: The lane the vehicle is currently in.
		:param adjacent_lane: The lane to consider for lane changing.
		:return: The gain from changing lanes to the given adjacent lane.
		"""
		# Check if the adjacent lane is to the left or to the right of the current lane
		if adjacent_lane == current_lane.left_lane:
			# Calculate the lane change gain for the left adjacent lane
			# by subtracting the density of right neighbors in the adjacent lane
			# from the density of left neighbors in the current lane
			left_gain = self.politeness_factor * current_lane.get_neighbor_density(vehicle, 'left') \
			            - adjacent_lane.get_neighbor_density(vehicle, 'right')
			return max(left_gain, 0)

		elif adjacent_lane == current_lane.right_lane:
			# Calculate the lane change gain for the right adjacent lane
			# by subtracting the density of left neighbors in the adjacent lane
			# from the density of right neighbors in the current lane
			right_gain = self.politeness_factor * current_lane.get_neighbor_density(vehicle, 'right') \
			             - adjacent_lane.get_neighbor_density(vehicle, 'left')
			return max(right_gain, 0)

		else:
			# Raise an error if the provided lane is not an adjacent lane
			raise ValueError("Provided lane is not an adjacent lane.")

	def _get_lane_change_direction(self, vehicle: Vehicle, current_lane: Lane, left_lane: Lane, right_lane: Lane,
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
		if left_lane and self.can_change_lane(vehicle, dt):
			return 'left'

		# Check if changing lanes to the right lane is possible
		if right_lane and self.can_change_lane(vehicle, dt):
			return 'right'

		return 'none'

	def get_braking_rate(self, vehicle: Vehicle, dt: float) -> float:
		"""
		Calculates the braking rate of a vehicle based on its current acceleration and velocity

		:param vehicle: The vehicle to calculate the braking rate for
		:param dt: The time step of the simulation
		:return: The braking rate of the vehicle
		"""
		# Calculate the current acceleration of the vehicle
		curr_acc = (vehicle.speed - vehicle.front_vehicle.speed) / dt

		# Calculate the maximum deceleration rate of the vehicle
		max_dec = min(-self.max_acc, -vehicle.speed / dt)

		# If the current acceleration is greater than the maximum deceleration rate, return 0
		if curr_acc > max_dec:
			return 0.0

		# Calculate the difference between the current acceleration and the maximum deceleration rate
		delta_acc = max_dec - curr_acc

		# Calculate the braking rate of the vehicle
		braking_rate = self.delta * delta_acc / self.max_dec

		return braking_rate

