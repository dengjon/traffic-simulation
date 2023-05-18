from typing import List, Optional
from vehicle import *


class Platoon(object):
	def __init__(self, lead_vehicle: Vehicle, last_vehicle: Vehicle, max_size: int):
		"""
		Initializes a new Platoon object.

		:param lead_vehicle: The leading vehicle of the platoon.
		:param last_vehicle: The rear vehicle of the platoon.
		:param max_size: The maximum number of vehicles that can be in the platoon.
		"""
		self.MIN_DISTANCE = 10  # Minimum safety distance
		self.vehicles = [lead_vehicle, last_vehicle]  # List of vehicles in the platoon
		self.num_vehicle = 2  # number of vehicles in the platoon
		self.lead_vehicle = lead_vehicle  # The leading vehicle in the platoon
		self.last_vehicle = last_vehicle  # The last vehicle in the platoon
		self.front_vehicle = lead_vehicle.front_vehicle  # The front vehicle of the platoon
		self.rear_vehicle = last_vehicle.rear_vehicle  # The rear vehicle of the platoon
		self.max_size = max_size  # The maximum number of vehicles that can be in the platoon

	def is_empty(self) -> bool:
		"""
		Returns True if the platoon has no vehicles besides the lead vehicle.
		"""
		return len(self.vehicles) >= 1

	def get_acceleration(self, dt):
		"""
		Updates the position, speed, and acceleration of each vehicle in the platoon based on the current state of the
		platoon and the road.

		:param dt: The time step to use for the update.
		"""
		# Update the acceleration, speed, and position of each vehicle in the platoon
		acc_lead = self.lead_vehicle.get_acceleration()
		acc_list = self.control()
		acc_list.insert(0, acc_lead)
		return acc_list

	def control(self) -> List[float]:
		"""Control method of the platoon"""
		# TODO: Complete the control method, or introduce control method from outside
		pass

	def add_vehicle(self, vehicle: Vehicle) -> bool:
		"""
		Attempts to add a vehicle to the platoon.

		:param vehicle: The vehicle to add.
		:return: True if the vehicle was successfully added, False otherwise.
		"""
		if not isinstance(vehicle, CAV):
			return False
		if not self.__can_join_platoon(vehicle):
			raise ValueError("Can't Join Platoon!")
		self.vehicles.append(vehicle)
		self.num_vehicle += 1
		self.last_vehicle = vehicle
		vehicle.platoon = self
		return True

	def remove_vehicle(self, vehicle: Vehicle):
		"""
		Attempts to remove a vehicle from the platoon.

		:param vehicle: The vehicle to remove.
		:return:
		"""
		if vehicle == self.lead_vehicle:
			raise ValueError("Cannot remove the front vehicle")  # cannot remove the front vehicle

		self.vehicles.remove(vehicle)
		vehicle.platoon = None  # remove the vehicle's platoon reference
		self.num_vehicle -= 1

	def split(self, vehicle: Vehicle):
		"""
		Splits the platoon into two platoons. If the member in the platoon raises a
		request to leave the platoon, the platoon will be split into two platoons.
		"""
		if vehicle == self.lead_vehicle:
			raise ValueError("Cannot split the front vehicle")

		if vehicle not in self.vehicles:
			raise ValueError("Vehicle not in the platoon")

		if vehicle == self.last_vehicle:
			vehicle.platoon = None
			self.vehicles.remove(vehicle)
			self.num_vehicle -= 1
			self.last_vehicle = self.vehicles[-1]
			return

		if vehicle == self.lead_vehicle:
			vehicle.platoon = None
			self.vehicles.remove(vehicle)
			self.num_vehicle -= 1
			self.lead_vehicle = self.vehicles[0]
			return

	def __can_join_platoon(self, vehicle: Vehicle) -> bool:
		"""
		Determines whether a vehicle can join the platoon.

		:param vehicle: The vehicle to check.
		:return: True if the vehicle can join the platoon, False otherwise.
		"""
		if vehicle.in_platoon:
			# Vehicle is already in a platoon
			return False

		if self.__is_full() or not self.vehicles:
			# Platoon is already full or empty
			return False

		if not isinstance(vehicle, CAV):
			# Only cars can join the platoon
			return False

		if vehicle.position > self.front_vehicle.position - vehicle.length - self.MIN_DISTANCE:
			# Vehicle's speed is too close to the front vehicle
			return False

		# All safety checks passed
		return True

	def __is_full(self) -> bool:
		"""
		Determines whether the platoon is full.

		:return: True if the platoon is full, False otherwise.
		"""
		return self.num_vehicle >= self.max_size
