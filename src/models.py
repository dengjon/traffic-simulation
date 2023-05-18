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

	def get_acceleration(self, v_ego: float, pos_ego: float, vehicle_length: float,
	                     v_lead: Optional[float] = None, pos_lead: Optional[float] = None) -> float:
		"""
		Calculates the acceleration of a vehicle based on the Intelligent Driver Model (IDM).

		Args:
			v_ego (float): Speed of the ego vehicle in m/s.
			pos_ego (float): Position of the ego vehicle in m.
			vehicle_length (float): Length of the ego vehicle in m.
			v_lead (float, optional): Speed of the lead vehicle in m/s.
			pos_lead (float, optional): Position of the lead vehicle in m.

		Returns:
			float: The acceleration of the ego vehicle in m/s^2.
		"""

		# The ego vehicle has a front vehicle
		if v_lead is not None:
			assert pos_lead is not None
			# desired distance between two vehicles
			s_star = self.s0 + max(0, v_ego * self.T + v_ego * (v_ego - v_lead) / (2 * np.sqrt(self.a * self.b)))

			# real distance between two vehicles minus the length of the vehicle
			real_distance = pos_lead - pos_ego - vehicle_length

			# acceleration
			acceleration = self.a * (1 - math.pow(v_ego / self.v0, 4) - math.pow(s_star / real_distance, 2))
		else:
			# The ego vehicle does not have a front vehicle
			acceleration = self.a * (1 - math.pow(v_ego / self.v0, 4))

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
	def __init__(self, accel_threshold: float = 0.5, brake_threshold: float = -2.0,
	             delta: float = 0.5, politeness_factor: float = 0.3):
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

	def get_incentive(self, vehicle, front_vehicle):
		"""
		Calculates the incentive for a lane change for the specified vehicle.
		:param vehicle: The vehicle that is attempting to change lanes.
		:param front_vehicle: The vehicle immediately in front of the vehicle in the targe lane.
		:return:
		"""
		# Calculate the acceleration gain from the lane change
		acc_lc_self, delta_acc_lc_rear = self.__calculate_acceleration_lc(vehicle, front_vehicle)

		# Calculate the lane change gain (how much the vehicle can benefit from the lane change)
		delta_acc_curr_rear = self.__calculate_acceleration_curr(vehicle)

		# Calculate the total incentive of the lane change
		delta_acc_lc_self = acc_lc_self - vehicle.get_acceleration()
		incentive = delta_acc_lc_self + self.politeness_factor * (delta_acc_curr_rear + delta_acc_lc_rear)

		return incentive

	@staticmethod
	def __calculate_acceleration_lc(v_ego: float, pos_ego: float, vehicle_length: float,
	                                v_front: Optional[float] = None, pos_front: Optional[float] = None,
	                                v_rear: Optional[float] = None, pos_rear: Optional[float] = None):
		"""
		Calculates the acceleration impacts of ego vehicle on vehicles in target lane if it were to change lanes.

		:param v_ego: The speed of the ego vehicle in m/s.
		:param pos_ego: The position of the ego vehicle in m.
		:param vehicle_length: The length of the ego vehicle in m.
		:param v_front: The speed of the vehicle immediately in front of the ego vehicle in the target lane in m/s.
		:param pos_front: The position of the vehicle immediately in front of the ego vehicle in the target lane in m.
		:param v_rear: The speed of the vehicle immediately behind the ego vehicle in the target lane in m/s.
		:param pos_rear: The position of the vehicle immediately behind the ego vehicle in the target lane in m.
		:return: The acceleration impacts of the ego vehicle on vehicles in target lane if it were to change lanes.
		"""
		delta_acc_lc_rear = 0

		idm = IDM()
		# Calculate the acceleration of the ego vehicle
		acc_ego = idm.get_acceleration(v_ego, pos_ego, vehicle_length, v_front, pos_front)

		# Calculate the acceleration of the vehicle immediately following the ego vehicle in the target lane
		if v_rear is not None and pos_rear is not None:
			acc_rear = idm.get_acceleration(v_rear, pos_rear, vehicle_length, v_front, pos_front)



			acc_rear = idm.get_acceleration(v_rear, pos_rear, vehicle_length, v_ego, pos_ego)
			delta_acc_lc_rear = acc_rear - v_rear.get_acceleration()
			delta_acc_lc_rear = min(v_rear.max_acc, delta_acc_lc_rear)


		return acc_lc, delta_acc_lc_rear

	@staticmethod
	def __calculate_acceleration_curr(vehicle) -> float:
		"""
		Calculates the gain from changing lanes to the given adjacent lane.

		:param vehicle: The vehicle that wants to change lanes.
		:return: The gain from changing lanes to the given adjacent lane.
		"""
		delta_acc_rear = 0

		if vehicle.rear_vehicle is not None:
			acc_before = vehicle.rear_vehicle.get_acceleration()
			if vehicle.front_vehicle is not None:
				rear_vehicle = copy.copy(vehicle.front_vehicle)
				delta_acc_rear = rear_vehicle.get_acceleration() - acc_before
				delta_acc_rear = min(rear_vehicle.max_acc, delta_acc_rear)
		return delta_acc_rear

	def check_lane_changing(self, vehicle, front_vehicle_adjacent):
		"""
		Checks whether a vehicle can change lanes based on the MOBIL model.
		:param vehicle: vehicle to check for a lane change
		:param front_vehicle_adjacent: vehicle in front of the vehicle in the adjacent lane
		:return: None
		"""
		# Get the speed of the target vehicle, the vehicle immediately in front of it in the adjacent lane,
		vehicle_curr = copy.copy(vehicle)

		if front_vehicle_adjacent is None:
			# vehicle has no front vehicle has no need to change lane
			return False
		else:
			if front_vehicle_adjacent.rear_vehicle is not None and \
					front_vehicle_adjacent.rear_vehicle.position > vehicle_curr.position:
				# vehicle is not the rear vehicle of the lane
				return False

			front_vehicle = copy.copy(front_vehicle_adjacent)
			vehicle_curr.front_vehicle = front_vehicle

			# check safety of front vehicle
			if vehicle_curr.get_acceleration() < self.brake_threshold:
				return False

			# check safety of rear vehicle
			if front_vehicle.rear_vehicle is not None:
				rear_vehicle = copy.copy(front_vehicle.rear_vehicle)
				rear_vehicle.front_vehicle = vehicle_curr
				if rear_vehicle.get_acceleration() < self.brake_threshold:
					return False
		return True
