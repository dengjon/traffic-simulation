import unittest
from vehicle import *
from models import *
from platoon import *
from lane import *
from fleet import *
import json


class __TestFleet(unittest.TestCase):

	def setUp(self):
		self.fleet1 = Fleet()  # Create a Fleet instance for testing
		self.fleet2 = Fleet()  # Create a Fleet instance for testing

	def test_add_vehicle(self):
		lane1 = MainLane(length=300, max_speed=30, start=0, end=3000)
		lane1.fleet = self.fleet1
		lane2 = MainLane(length=300, max_speed=30, start=0, end=3000)
		lane2.fleet = self.fleet2
		vehicle1 = Vehicle(vehicle_id=1, init_speed=30, init_lane=lane1, init_pos=50, init_acc=0)
		vehicle2 = Vehicle(vehicle_id=2, init_speed=20, init_lane=lane2, init_pos=0, init_acc=0)

		self.fleet1.add_vehicle(vehicle1)  # Add the first vehicle to the fleet
		self.fleet2.add_vehicle(vehicle2)  # Add the first vehicle to the fleet
		self.assertEqual(len(self.fleet1.vehicles), 1)  # Check that the vehicle was added
		self.assertEqual(len(self.fleet2.vehicles), 1)  # Check that the vehicle was added

		self.fleet1.change_lane(vehicle2, lane1)  # Add the second vehicle behind the first vehicle
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
		self.fleet1.add_vehicle(vehicle1)  # Add the vehicles to the fleet
		self.fleet2.add_vehicle(vehicle2)  # Add the vehicles to the fleet
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
		vehicle = HV(vehicle_id=1, init_speed=30, init_lane=lane, init_pos=0, init_acc=0, **settings_vehicle["HV"])
		vehicle.speed = 10
		vehicle.position = 0

		self.fleet1.add_vehicle(vehicle)  # Add the vehicle to the fleet

		dt = 0.1  # Time step for the update
		self.fleet1.update_vehicles(dt)  # Update the vehicles in the fleet


class VehicleTest(unittest.TestCase):
	def setUp(self) -> None:
		self.lane1 = MainLane(length=100, max_speed=50, start=(0, 0), end=(100, 0))
		self.lane2 = MainLane(length=100, max_speed=50, start=(0, 0), end=(100, 0))
		self.fleet1 = Fleet()  # Create a Fleet instance for testing
		self.fleet2 = Fleet()  # Create a Fleet instance for testing
		self.lane1.fleet = self.fleet1
		self.lane2.fleet = self.fleet2

	def test_get_acceleration(self):
		with open('settings.json') as fp:
			settings = json.load(fp)
			settings_vehicle = settings["Vehicle"]
		# Create an instance of the Vehicle class
		vehicle1 = HV(vehicle_id=1, init_speed=25, init_lane=self.lane1, init_pos=50, init_acc=0, **settings_vehicle["HV"])
		vehicle2 = HV(vehicle_id=2, init_speed=20, init_lane=self.lane1, init_pos=0, init_acc=0, **settings_vehicle["HV"])
		self.fleet1.add_vehicle(vehicle1)
		self.fleet1.add_vehicle(vehicle2, vehicle1)

		# Call the get_acceleration() method
		acceleration = vehicle2.get_acceleration()

		# Perform assertions to check if the acceleration is calculated correctly
		self.assertTrue(acceleration > 0)

	def test_update(self):
		# Create an instance of the Vehicle class
		with open('settings.json') as fp:
			settings = json.load(fp)
			settings_vehicle = settings["Vehicle"]
		# Create an instance of the Vehicle class
		vehicle1 = HV(vehicle_id=1, init_speed=30, init_lane=self.lane1, init_pos=100, init_acc=0,
		              **settings_vehicle["HV"])
		vehicle2 = HV(vehicle_id=2, init_speed=20, init_lane=self.lane1, init_pos=0, init_acc=0,
		              **settings_vehicle["HV"])
		self.fleet1.add_vehicle(vehicle1)
		self.fleet1.add_vehicle(vehicle2, vehicle1)

		# Call the update() method with a time step
		dt = 1
		acc_list = [vehicle1.get_acceleration(), vehicle2.get_acceleration()]
		vehicle1.update(acc_list[0], dt)
		vehicle2.update(acc_list[1], dt)

		# Perform assertions to check if the position and speed are updated correctly
		delta_position = vehicle1.position - vehicle2.position
		self.assertTrue(delta_position > 100)

	def test_get_adjacent_lead_vehicle(self):
		with open('settings.json') as fp:
			settings = json.load(fp)
			settings_vehicle = settings["Vehicle"]
		# Create an instance of the Vehicle class
		vehicle1 = HV(vehicle_id=1, init_speed=25, init_lane=self.lane1, init_pos=100, init_acc=0,
		              **settings_vehicle["HV"])
		vehicle2 = HV(vehicle_id=2, init_speed=20, init_lane=self.lane1, init_pos=0, init_acc=0,
		              **settings_vehicle["HV"])
		self.fleet1.add_vehicle(vehicle1)
		self.fleet1.add_vehicle(vehicle2, vehicle1)

		# Call the get_adjacent_lead_vehicle() method with a target lane
		target_lane = self.lane2
		lead_vehicle = vehicle1.get_adjacent_lead_vehicle(target_lane)

		# Perform assertions to check if the correct lead vehicle is returned
		self.assertIsNone(lead_vehicle)

	def test_move_to_lane(self):
		# Create an instance of the Vehicle class
		vehicle1 = HV(vehicle_id=1, init_speed=30, init_lane=self.lane1, init_pos=10, init_acc=0)
		vehicle2 = HV(vehicle_id=2, init_speed=20, init_lane=self.lane1, init_pos=0, init_acc=0)
		self.fleet1.add_vehicle(vehicle1)
		self.fleet1.add_vehicle(vehicle2, vehicle1)

		# Call the move_to_lane() method with the new lane
		vehicle1.move_to_lane(self.lane2)

		# Perform assertions to check if the vehicle is correctly moved to the new lane
		self.assertEqual(vehicle1.lane, self.lane2)
		# Perform additional assertions if needed


class MOBILTests(unittest.TestCase):

	def setUp(self) -> None:
		self.lane1 = MainLane(length=3000, max_speed=50, start=0, end=3000)
		self.lane2 = MainLane(length=3000, max_speed=50, start=0, end=3000)
		self.lane1.right_lane = self.lane2
		self.lane2.left_lane = self.lane1

		self.lane1.fleet = Fleet()
		self.lane2.fleet = Fleet()

	def test_can_change_lane(self):
		mobil = MOBIL(accel_threshold=0.5, brake_threshold=-3.0, delta=0.1, politeness_factor=0.3)

		with open('settings.json') as fp:
			settings = json.load(fp)
			settings_vehicle = settings["Vehicle"]
		# Create an instance of the Vehicle class
		vehicle1 = HV(init_speed=20, init_lane=self.lane1, init_pos=1000, init_acc=0, **settings_vehicle["HV"])
		vehicle2 = HV(init_speed=20, init_lane=self.lane2, init_pos=0, init_acc=0, **settings_vehicle["HV"])
		self.lane1.fleet.add_vehicle(vehicle1)
		self.lane2.fleet.add_vehicle(vehicle2)

		dt = 1

		# Test case: Vehicle1 can change to the right lane
		self.assertTrue(mobil.can_change_lane(vehicle1, 'right', dt))

		# Test case: Vehicle2 can change to the right lane
		self.assertTrue(mobil.can_change_lane(vehicle2, 'left', dt))

		# Test case: Vehicle1 cannot change lanes without neighboring lanes
		self.lane1.right_lane = None
		self.assertFalse(mobil.can_change_lane(vehicle1, 'left', dt))
		self.assertFalse(mobil.can_change_lane(vehicle1, 'right', dt))

# Add more test methods for other methods in the MOBIL class


if __name__ == '__main__':
	unittest.main()
