import unittest
from vehicle import *
from models import *
from platoon import *
from lane import *
from fleet import *
import json
import utils

NUM_VEHICLES = 5
INIT_SPEED = 20


def initialize():
	with open('settings.json', 'r') as f:
		settings = json.load(f)

	# Generate and initialize lanes
	lane_list = utils.generate_scenario()

	# Generate and initialize vehicles
	num_vehicle_each_lane = NUM_VEHICLES
	headway = 200
	init_speed = INIT_SPEED
	position_first_vehicle = 2000
	for i in range(num_vehicle_each_lane):
		for lane_curr in lane_list:
			last_vehicle = lane_curr.fleet.rear_vehicle
			if last_vehicle is not None:
				position = last_vehicle.position - headway - last_vehicle.length
			else:
				position = position_first_vehicle
			vehicle_curr = HV(init_speed, lane_curr, position, init_acc=0, **settings['Vehicle']['HV'])
			lane_curr.fleet.add_vehicle(vehicle_curr, last_vehicle)
			position_first_vehicle -= 100
	return lane_list


class TestFleet(unittest.TestCase):

	def setUp(self):
		lane_list = initialize()
		self.fleet1 = lane_list[0].fleet
		self.fleet2 = lane_list[1].fleet

	def test_add_vehicle(self):
		self.assertEqual(len(self.fleet1.vehicles), NUM_VEHICLES)  # Check that the vehicle was added

		front_vehicle_list_right = self.fleet1.get_front_vehicle_list(self.fleet2.lane)
		self.assertIsNone(front_vehicle_list_right[0])

		# Add the second vehicle behind the first vehicle
		self.fleet1.get_lane_change_intention(None, front_vehicle_list_right)
		self.fleet1.change_lane()
		self.assertEqual(len(self.fleet1.vehicles), NUM_VEHICLES)  # Check that both vehicles were added
		self.assertEqual(len(self.fleet2.vehicles), NUM_VEHICLES)  # Check that both vehicles were added

	def test_remove_vehicle(self):
		vehicle = self.fleet2.vehicles[1]
		self.fleet2.remove_vehicle(self.fleet2.front_vehicle)  # Remove the first vehicle from the fleet
		self.assertEqual(len(self.fleet2.vehicles), NUM_VEHICLES - 1)  # Check that the vehicle was removed
		self.assertEqual(self.fleet2.front_vehicle, vehicle)  # Check that the remaining vehicle is in the fleet


class VehicleTest(unittest.TestCase):
	def setUp(self) -> None:
		lane_list = initialize()
		self.fleet1 = lane_list[0].fleet
		self.fleet2 = lane_list[1].fleet

	def test_get_acceleration(self):
		vehicle = self.fleet1.rear_vehicle
		acceleration = vehicle.get_acceleration()

		# Perform assertions to check if the acceleration is calculated correctly
		self.assertTrue(acceleration > 0)

		# Test acceleration influenced by obstacle
		vehicle.obstacle_position = vehicle.position + 100
		acc_obstacle = vehicle.get_acceleration()
		self.assertLess(acc_obstacle, acceleration)

	def test_update(self):
		dt = 1
		acc_list = []
		delta_position_before = self.fleet1.vehicles[0].position - self.fleet1.vehicles[1].position
		for vehicle in self.fleet1.vehicles:
			acc_list.append(vehicle.get_acceleration())

		for i, vehicle in enumerate(self.fleet1.vehicles):
			vehicle.update(acc_list[i], dt)

		# Perform assertions to check if the position and speed are updated correctly
		delta_position_after = self.fleet1.vehicles[0].position - self.fleet1.vehicles[1].position
		self.assertTrue(delta_position_after > delta_position_before)

	def test_move_to_lane(self):
		vehicle = self.fleet1.vehicles[1]
		front_vehicle = vehicle.get_adjacent_front_vehicle(self.fleet2.lane)
		# Call the move_to_lane() method with the new lane
		vehicle.move_to_lane(front_vehicle, self.fleet2.lane)

		# Perform assertions to check if the vehicle is correctly moved to the new lane
		self.assertEqual(vehicle.lane, self.fleet2.lane)


if __name__ == '__main__':
	unittest.main()
