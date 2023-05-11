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
		self.MIN_DISTANCE = 30  # Minimum safety distance
		self.vehicles = [lead_vehicle, last_vehicle]  # List of vehicles in the platoon
		self.num_vehicle = 2    # number of vehicles in the platoon
		self.lead_vehicle = lead_vehicle  # The leading vehicle in the platoon
		self.last_vehicle = last_vehicle  # The last vehicle in the platoon
		self.front_vehicle = lead_vehicle.front_vehicle     # The front vehicle of the platoon
		self.rear_vehicle = last_vehicle.rear_vehicle       # The rear vehicle of the platoon
		self.max_size = max_size  # The maximum number of vehicles that can be in the platoon

	def is_empty(self) -> bool:
		"""
		Returns True if the platoon has no vehicles besides the lead vehicle.
		"""
		return len(self.vehicles) >= 1

	def is_full(self) -> bool:
		"""
		Returns True if the platoon has reached its maximum size.
		"""
		return len(self.vehicles) >= self.max_size

	def update(self, dt: float):
		"""
		Updates the position, speed, and acceleration of each vehicle in the platoon based on the current state of the
		platoon and the road.

		:param dt: The time step to use for the update.
		"""
		# Update the acceleration, speed, and position of each vehicle in the platoon
		for i, vehicle in enumerate(self.vehicles):
			if i == 0:  # Leader vehicle
				# TODO: Lead vehicle in a platoon will behave different from following vehicles
				vehicle.acceleration = vehicle.get_acceleration()
			else:
				vehicle.acceleration = vehicle.get_acceleration()

			vehicle.speed += vehicle.acceleration * dt
			vehicle.position += vehicle.speed * dt

		# Update the front and rear vehicles
		self.lead_vehicle = self.vehicles[0]
		self.last_vehicle = self.vehicles[-1]
		"""Complete"""

	def add_vehicle(self, vehicle: Vehicle) -> bool:
		"""
		Attempts to add a vehicle to the platoon.

		:param vehicle: The vehicle to add.
		:return: True if the vehicle was successfully added, False otherwise.
		"""
		if not isinstance(vehicle, Vehicle):
			return False
		if self.is_full():
			raise OverflowError("Platoon is Full!")
		if not self.can_join_platoon(vehicle):
			raise ValueError("Can't Join Platoon!")
		self.vehicles.insert(-1, vehicle)
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
			raise ValueError("Cannot remove the front vehicle")     # cannot remove the front vehicle

		self.vehicles.remove(vehicle)
		vehicle.platoon = None  # remove the vehicle's platoon reference
		self.num_vehicle -= 1

	def can_join_platoon(self, vehicle: Vehicle) -> bool:
		"""
		Determines whether a vehicle can join the platoon.

		:param vehicle: The vehicle to check.
		:return: True if the vehicle can join the platoon, False otherwise.
		"""
		if self.is_full() or not self.vehicles:
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



