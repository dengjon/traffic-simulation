from vehicle import Vehicle
from typing import Optional
from models import *
import unittest


class Lane:
	def __init__(self, length, max_speed, start, end, lane_type,
	             left_lane=None, right_lane=None, capacity=1600):
		"""
		Initializes the properties of a lane.

		:param length: Length of the lane
		:param max_speed: Maximum speed allowed on the lane
		:param start: Start point of the lane
		:param end: End point of the lane
		:param lane_type: Type of the lane (e.g. 'straight', 'left-turn-only', etc.)
		:param left_lane: The neighboring lane to the left
		:param right_lane: The neighboring lane to the right
		:param capacity: Maximum number of vehicles that can be on the lane
		"""
		self.length = length
		self.max_speed = max_speed
		self.capacity = capacity
		self.start = start
		self.end = end
		self.left_lane: Optional[Lane] = left_lane
		self.right_lane: Optional[Lane] = right_lane
		self.num_vehicle = 0  # Number of vehicles currently on the lane
		self.front_vehicle: Optional[Vehicle] = None  # The vehicle at the front of the lane
		self.rear_vehicle: Optional[Vehicle] = None  # The vehicle at the back of the lane
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
			if mobil.can_change_lane(vehicle, 'left', dt):
				new_lane = self.left_lane
				if new_lane is not None:
					vehicle.move_to_lane(new_lane)

			# Update the vehicle's speed and position based on the acceleration
			vehicle.speed += acceleration * dt
			if vehicle.speed < 0:
				vehicle.speed = 0
			elif vehicle.speed > self.max_speed:
				vehicle.speed = self.max_speed
			vehicle.position += vehicle.speed

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


class LaneTestCase(unittest.TestCase):
	def test_add_vehicle(self):
		# Create test objects (lane, vehicles, and target front vehicle)
		lane1 = Lane(length=100, max_speed=50, start=(0, 0), end=(100, 0), lane_type='straight')
		lane2 = Lane(length=100, max_speed=50, start=(0, 0), end=(100, 0), lane_type='straight')
		vehicle1 = Vehicle(vehicle_id=1, init_speed=30, init_lane=lane1, init_pos=0, init_acc=0)
		vehicle2 = Vehicle(vehicle_id=2, init_speed=20, init_lane=lane2, init_pos=50, init_acc=0)
		target_front_vehicle = Vehicle(vehicle_id=3, init_speed=25, init_lane=lane1, init_pos=40, init_acc=0)

		# Test adding a vehicle with no target front vehicle
		lane1.add_vehicle(vehicle1)
		self.assertEqual(lane1.front_vehicle, vehicle1)
		self.assertEqual(lane1.rear_vehicle, vehicle1)
		self.assertEqual(lane1.num_vehicle, 1)

		# Test adding a vehicle with a target front vehicle
		lane1.add_vehicle(vehicle2, target_front_vehicle)
		self.assertEqual(target_front_vehicle.rear_vehicle, vehicle2)
		self.assertEqual(vehicle2.front_vehicle, target_front_vehicle)
		self.assertEqual(lane1.rear_vehicle, vehicle2)
		self.assertEqual(lane1.num_vehicle, 2)


if __name__ == '__main__':
	unittest.main()
