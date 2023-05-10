from vehicle import Vehicle
from typing import Optional


class Lane:
	def __init__(self, length, max_speed, start, end, lane_type, capacity=1600):
		# Initialize the properties of the lane
		self.length = length
		self.max_speed = max_speed
		self.capacity = capacity
		self.start = start
		self.end = end
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
			self.front_vehicle.front_vehicle = vehicle
			vehicle.rear_vehicle = self.front_vehicle
			self.front_vehicle = vehicle
			self.num_vehicle += 1
		else:  # Add the vehicle behind the target front vehicle
			# Insert the new vehicle behind the target front vehicle
			target_rear_vehicle = target_front_vehicle.rear_vehicle
			if target_rear_vehicle is None:
				# If the target front vehicle has no rear vehicle, set the new vehicle as the rear vehicle
				target_front_vehicle.rear_vehicle = vehicle
				vehicle.front_vehicle = target_front_vehicle
				self.rear_vehicle = vehicle
			else:
				# If the target front vehicle has a rear vehicle, insert the new vehicle between them
				target_rear_vehicle.front_vehicle = vehicle
				vehicle.rear_vehicle = target_rear_vehicle
				target_front_vehicle.rear_vehicle = vehicle
				vehicle.front_vehicle = target_front_vehicle
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

	def update_vehicles(self):
		"""
		This function updates the position and speed of all vehicles on the road,
		 based on their acceleration calculated by the car-following model.
		This function should also handle lane changing based on the MOBIL model.
		"""
		pass


if __name__ == '__main__':
	print('hello')
