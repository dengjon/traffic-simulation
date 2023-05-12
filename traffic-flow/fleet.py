from vehicle import *
from models import *
from platoon import *
import unittest


class Fleet(object):
	def __init__(self):
		"""
		Initializes a fleet of vehicles.
		"""
		self.front_vehicle = None  # Placeholder for the front vehicle
		self.rear_vehicle = None  # Placeholder for the rear vehicle
		self.vehicles = []  # List to store the vehicles in the fleet

	def add_vehicle(self, vehicle: Vehicle, target_front_vehicle: Optional[Vehicle] = None):
		"""
		Adds a vehicle to the fleet if there is space.

		If target_front_vehicle is None, adds the vehicle as the front vehicle in the fleet.

		:param vehicle: The vehicle to add.
		:param target_front_vehicle: The front vehicle in the fleet to add the vehicle behind.
		"""
		if len(self.vehicles) == 0:  # Fleet is empty
			self.vehicles.append(vehicle)
			self.front_vehicle = vehicle
			self.rear_vehicle = vehicle
			return

		if target_front_vehicle is None:  # No target front vehicle
			# Add the vehicle as the front vehicle of the fleet
			self.vehicles.insert(0, vehicle)
			vehicle.lane.fleet.remove_vehicle(vehicle)

			self.front_vehicle.lead_vehicle = vehicle  # Placeholder for setting the lead vehicle property
			vehicle.last_vehicle = self.front_vehicle  # Placeholder for setting the last vehicle property
			self.front_vehicle = vehicle  # Update the front vehicle
		else:  # Add the vehicle behind the target front vehicle
			# Insert the new vehicle behind the target front vehicle
			index = self.vehicles.index(target_front_vehicle)
			self.vehicles.insert(index + 1, vehicle)
			vehicle.lane.fleet.remove_vehicle(vehicle)

			target_rear_vehicle = target_front_vehicle.rear_vehicle  # Placeholder for the target front vehicle's rear vehicle
			target_front_vehicle.rear_vehicle = vehicle  # Update the rear vehicle of the target front vehicle
			vehicle.front_vehicle = target_front_vehicle  # Update the front vehicle of the added vehicle
			if target_rear_vehicle is None:
				# If the target front vehicle has no rear vehicle, set the new vehicle as the rear vehicle
				self.rear_vehicle = vehicle
			else:
				# If the target front vehicle has a rear vehicle, insert the new vehicle between them
				target_rear_vehicle.front_vehicle = vehicle
				vehicle.rear_vehicle = target_rear_vehicle  # Update the rear vehicle of the added vehicle

	def remove_vehicle(self, vehicle: Vehicle):
		"""
		Removes the given vehicle from the fleet.

		:param vehicle: The vehicle to remove.
		"""
		self.vehicles.remove(vehicle)  # Remove the vehicle from the fleet

		if self.front_vehicle == vehicle:  # Vehicle to remove is the front vehicle
			self.front_vehicle = vehicle.rear_vehicle  # Update the front vehicle
			if self.front_vehicle is not None:
				self.front_vehicle.front_vehicle = None  # Update the front vehicle's front vehicle
			else:
				self.rear_vehicle = None  # No more vehicles in the fleet, update the rear vehicle
		elif self.rear_vehicle == vehicle:  # Vehicle to remove is the rear vehicle
			self.rear_vehicle = vehicle.front_vehicle  # Update the rear vehicle
			if self.rear_vehicle is not None:
				self.rear_vehicle.rear_vehicle = None  # Update the rear vehicle's rear vehicle
			else:
				self.front_vehicle = None  # No more vehicles in the fleet, update the front vehicle
		else:  # Vehicle to remove is in the middle of the fleet
			vehicle.front_vehicle.rear_vehicle = vehicle.rear_vehicle  # Update the front vehicle's rear vehicle
			vehicle.rear_vehicle.front_vehicle = vehicle.front_vehicle  # Update the rear vehicle's front vehicle

	def init_vehicle(self, vehicle: Vehicle):
		"""
		Initializes the fleet with vehicles.

		:param vehicle: The first vehicle to add to the fleet.
		"""
		# If the fleet is empty, set the given vehicle as the front and rear vehicle
		if len(self.vehicles) == 0:
			self.front_vehicle = vehicle
			self.rear_vehicle = vehicle
		else:
			# Add the vehicle behind the current rear vehicle
			self.rear_vehicle.rear_vehicle = vehicle
			vehicle.front_vehicle = self.rear_vehicle
			self.rear_vehicle = vehicle

		self.vehicles.append(vehicle)  # Add the vehicle to the list of vehicles in the fleet

	def update_vehicles(self, dt: float):
		"""
		Updates the position and speed of all vehicles in the fleet based on the car-following model and lane changing.

		:param dt: The time step for the update.
		"""
		mobil = MOBIL()  # Create an instance of the MOBIL model

		for i, vehicle in enumerate(self.vehicles):
			# Calculate the acceleration for this vehicle based on the car-following model
			acceleration = vehicle.get_acceleration()  # Placeholder for calculating the acceleration

			# Handle lane changing based on the MOBIL model
			if mobil.can_change_lane(vehicle, 'left', dt):  # Placeholder for determining if lane change is possible
				new_lane = vehicle.lane.left_lane  # Placeholder for getting the left neighboring lane
				if new_lane is not None:
					vehicle.move_to_lane(new_lane)  # Placeholder for moving the vehicle to the new lane

			# Update the vehicle's speed and position based on the acceleration
			vehicle.speed += acceleration * dt  # Update the vehicle's speed
			if vehicle.speed < 0:
				vehicle.speed = 0  # Ensure the speed is non-negative
			elif vehicle.speed > vehicle.lane.max_speed:
				vehicle.speed = vehicle.lane.max_speed  # Limit the speed to the maximum speed of the lane
			vehicle.position += vehicle.speed  # Update the vehicle's position


class __TestFleet(unittest.TestCase):

	def setUp(self):
		self.fleet1 = Fleet()  # Create a Fleet instance for testing
		self.fleet2 = Fleet()  # Create a Fleet instance for testing

	def test_add_vehicle(self):
		lane1 = Lane(length=100, max_speed=50, start=(0, 0), end=(100, 0), lane_type='straight')
		lane1.fleet = self.fleet1
		lane2 = Lane(length=100, max_speed=50, start=(0, 0), end=(100, 0), lane_type='straight')
		lane2.fleet = self.fleet2
		vehicle1 = Vehicle(vehicle_id=1, init_speed=30, init_lane=lane1, init_pos=0, init_acc=0)
		vehicle2 = Vehicle(vehicle_id=2, init_speed=20, init_lane=lane2, init_pos=50, init_acc=0)

		self.fleet1.init_vehicle(vehicle1)  # Add the first vehicle to the fleet
		self.fleet2.init_vehicle(vehicle2)  # Add the first vehicle to the fleet
		self.assertEqual(len(self.fleet1.vehicles), 1)  # Check that the vehicle was added
		self.assertEqual(len(self.fleet2.vehicles), 1)  # Check that the vehicle was added

		self.fleet1.add_vehicle(vehicle2,
		                        target_front_vehicle=vehicle1)  # Add the second vehicle behind the first vehicle
		self.assertEqual(len(self.fleet1.vehicles), 2)  # Check that both vehicles were added
		self.assertEqual(len(self.fleet2.vehicles), 0)
		self.assertEqual(self.fleet1.vehicles[-1], vehicle2)  # Check that the second vehicle is in the correct position

	def test_remove_vehicle(self):
		lane1 = Lane(length=100, max_speed=50, start=(0, 0), end=(100, 0), lane_type='straight')
		lane1.fleet = self.fleet1
		lane2 = Lane(length=100, max_speed=50, start=(0, 0), end=(100, 0), lane_type='straight')
		lane2.fleet = self.fleet2
		vehicle1 = Vehicle(vehicle_id=1, init_speed=30, init_lane=lane1, init_pos=0, init_acc=0)
		vehicle2 = Vehicle(vehicle_id=2, init_speed=20, init_lane=lane2, init_pos=50, init_acc=0)
		self.fleet1.init_vehicle(vehicle1)  # Add the vehicles to the fleet
		self.fleet2.init_vehicle(vehicle2)  # Add the vehicles to the fleet
		self.fleet1.add_vehicle(vehicle2)

		self.fleet1.remove_vehicle(vehicle1)  # Remove the first vehicle from the fleet
		self.assertEqual(len(self.fleet1.vehicles), 1)  # Check that the vehicle was removed
		self.assertEqual(self.fleet1.vehicles[-1], vehicle2)  # Check that the remaining vehicle is in the fleet

	def test_update_vehicles(self):
		import json
		with open('settings.json') as fp:
			settings = json.load(fp)
			settings_vehicle = settings["Vehicle"]
		lane = Lane(length=100, max_speed=50, start=(0, 0), end=(100, 0), lane_type='straight')
		lane.fleet = self.fleet1
		vehicle = HV(vehicle_id=1, init_speed=30, init_lane=lane, init_pos=0, init_acc=0, **settings_vehicle)
		vehicle.speed = 10
		vehicle.position = 0

		self.fleet1.add_vehicle(vehicle)  # Add the vehicle to the fleet

		dt = 0.1  # Time step for the update
		self.fleet1.update_vehicles(dt)  # Update the vehicles in the fleet


if __name__ == '__main__':
	unittest.main()
