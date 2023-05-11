from vehicle import Vehicle
from typing import Optional
from models import *


class Lane:
	def __init__(self, length, max_speed, start, end, lane_type,
	             left_lane=None, right_lane=None, capacity=1600):
		# Initialize the properties of the lane
		self.length = length
		self.max_speed = max_speed
		self.capacity = capacity
		self.start = start
		self.end = end
		self.left_lane: Optional[Lane] = left_lane
		self.right_lane: Optional[Lane] = right_lane
		self.num_vehicle = 0
		self.front_vehicle: Optional[Vehicle] = None
		self.rear_vehicle: Optional[Vehicle] = None
		self.type = lane_type
		self.vehicles = []  # List to hold the vehicles currently on the lane
		self.lanes = []  # List of neighboring lanes

	def add_vehicle(self, vehicle, target_front_vehicle: Optional[Vehicle] = None):
		"""
		Adds a vehicle to the lane if there is space.

		If target_front_vehicle is None, adds the vehicle as the front vehicle in the lane.

		:param vehicle: The vehicle to add.
		:param target_front_vehicle: The front vehicle in the lane to add the vehicle behind.
		"""
		if self.front_vehicle is None:  # Lane is empty
			self.front_vehicle = vehicle
			self.rear_vehicle = vehicle
			self.num_vehicle = 1
		elif target_front_vehicle is None:  # No target front vehicle
			# Add the vehicle as the front vehicle of the lane
			self.front_vehicle.lead_vehicle = vehicle
			vehicle.last_vehicle = self.front_vehicle
			self.front_vehicle = vehicle
			self.num_vehicle += 1
		else:  # Add the vehicle behind the target front vehicle
			# Insert the new vehicle behind the target front vehicle
			target_rear_vehicle = target_front_vehicle.rear_vehicle
			if target_rear_vehicle is None:
				# If the target front vehicle has no rear vehicle, set the new vehicle as the rear vehicle
				target_front_vehicle.rear_vehicle = vehicle
				vehicle.lead_vehicle = target_front_vehicle
				self.rear_vehicle = vehicle
			else:
				# If the target front vehicle has a rear vehicle, insert the new vehicle between them
				target_rear_vehicle.front_vehicle = vehicle
				vehicle.last_vehicle = target_rear_vehicle
				target_front_vehicle.rear_vehicle = vehicle
				vehicle.lead_vehicle = target_front_vehicle
			self.num_vehicle += 1

	def remove_vehicle(self, vehicle: Vehicle):
		"""
		Removes the given vehicle from the lane.

		:param vehicle: The vehicle to remove.
		"""
		if self.front_vehicle == vehicle:  # Vehicle to remove is the front vehicle
			self.front_vehicle = vehicle.rear_vehicle
			if self.front_vehicle is not None:
				self.front_vehicle.front_vehicle = None
			else:
				self.rear_vehicle = None
		elif self.rear_vehicle == vehicle:  # Vehicle to remove is the rear vehicle
			self.rear_vehicle = vehicle.front_vehicle
			if self.rear_vehicle is not None:
				self.rear_vehicle.rear_vehicle = None
			else:
				self.front_vehicle = None
		else:  # Vehicle to remove is in the middle of the lane
			vehicle.front_vehicle.rear_vehicle = vehicle.rear_vehicle
			vehicle.rear_vehicle.front_vehicle = vehicle.front_vehicle
		self.num_vehicle -= 1

	def check_collisions(self):
		# TODO: check for collisions between vehicles in the lane
		pass

	def update_vehicles(self, dt: float):
		"""
		This function updates the position and speed of all vehicles on the road,
		 based on their acceleration calculated by the car-following model.
		This function should also handle lane changing based on the MOBIL model.
		"""
		mobil = MOBIL()
		for i, vehicle in enumerate(self.vehicles):
			# Calculate the acceleration for this vehicle based on the car-following model
			acceleration = vehicle.get_acceleration()

			# Handle lane changing based on the MOBIL model
			if mobil.can_change_lane(vehicle, 'left'):
				new_lane = self.left_lane
				if new_lane is not None:
					if self._can_move_to_lane(i, new_lane):
						self._move_to_lane(i, new_lane)

			# Update the vehicle's speed and position based on the acceleration
			vehicle.speed += acceleration * dt
			if vehicle.speed < 0:
				vehicle.speed = 0
			elif vehicle.speed > self.max_speed:
				vehicle.speed = self.max_speed
			vehicle.position += vehicle.speed

			# Wrap around if necessary
			if vehicle.position >= self.length:
				vehicle.position -= self.length

	def get_neighbor_density(self, vehicle: Vehicle, direction: str) -> float:
		"""
		Calculate the density of neighboring vehicles in the given direction.

		:param vehicle: The target vehicle.
		:param direction: The direction to check for neighboring vehicles ('left' or 'right').
		:return: The density of neighboring vehicles.
		"""
		density = 0.0

		# Calculate density of neighboring vehicles in this lane
		for v in self.vehicles:
			if v is not vehicle:
				dist = abs(v.position - vehicle.position)
				density += 1.0 / (dist * v.length)

		# Check for neighboring vehicles in the left adjacent lane
		if direction == 'left' and self.left_lane is None:
			raise ValueError('No left lane')
		else:
			for v in self.left_lane.vehicles:
				if v.position > vehicle.position:
					dist = abs(v.position - vehicle.position)
					density += 1.0 / (dist * v.length)

		# Check for neighboring vehicles in the right adjacent lane
		if direction == 'right' and self.right_lane is None:
			raise ValueError('No right lane')
		else:
			for v in self.right_lane.vehicles:
				if v.position < vehicle.position:
					dist = abs(v.position - vehicle.position)
					density += 1.0 / (dist * v.length)

		return density


if __name__ == '__main__':
	print('hello')
