import unittest
from vehicle import *
from models import *
from road import *
from structure import Fleet, Platoon
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
			fleet_curr: Fleet = lane_curr.fleet
			if len(fleet_curr) > 0:
				last_vehicle = fleet_curr[-1]
				position = last_vehicle.position - headway - last_vehicle.length
			else:
				position = position_first_vehicle
			vehicle_curr = HV(init_speed, lane_curr, position, init_acc=0, **settings['Vehicle']['HV'])
			lane_curr.fleet.append(vehicle_curr)
			position_first_vehicle -= 100
	return lane_list


# class TestFleet(unittest.TestCase):
#
# 	def setUp(self):
# 		lane_list = initialize()
# 		self.fleet1 = lane_list[0].fleet
# 		self.fleet2 = lane_list[1].fleet
#
# 	def test_get_acceleration(self):
# 		acc_list = self.fleet1.get_acceleration(1)
# 		self.assertEqual(len(acc_list), NUM_VEHICLES)


class TestVehicle(unittest.TestCase):
	def setUp(self) -> None:
		lane_list = initialize()
		self.fleet1 = lane_list[0].fleet
		self.fleet2 = lane_list[1].fleet

	def test_get_acceleration(self):
		vehicle = self.fleet1[-1]
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
		delta_position_before = self.fleet1[0].position - self.fleet1[1].position
		for vehicle in self.fleet1:
			acc_list.append(vehicle.get_acceleration())

		for i, vehicle in enumerate(self.fleet1):
			vehicle.update(acc_list[i], dt)

		# Perform assertions to check if the position and speed are updated correctly
		delta_position_after = self.fleet1[0].position - self.fleet1[1].position
		self.assertTrue(delta_position_after > delta_position_before)

	def test_move_to_lane(self):
		vehicle = self.fleet1[1]
		front_vehicle = vehicle.get_adjacent_front_vehicle(self.fleet2.lane)
		# Call the move_to_lane() method with the new lane
		vehicle.move_to_lane(front_vehicle, self.fleet2.lane)

		# Perform assertions to check if the vehicle is correctly moved to the new lane
		self.assertEqual(vehicle.lane, self.fleet2.lane)


# class TestPlatoon(unittest.TestCase):
# 	def setUp(self) -> None:
# 		lane_list = initialize()
# 		self.fleet1 = lane_list[0].fleet
# 		self.fleet2 = lane_list[1].fleet
#
# 	# def test_get_acceleration(self):
# 	# 	platoon = Platoon(self.fleet1[:3])
# 	# 	acceleration = platoon.get_acceleration(1)
# 	#
# 	# 	# Perform assertions to check if the acceleration is calculated correctly
# 	# 	self.assertTrue(len(acceleration) == 3)
#
# 	def test_split(self):
# 		platoon = Platoon(self.fleet1[:])
# 		platoon_new = platoon.split(2)
# 		self.assertEqual(len(platoon), 2)
# 		self.assertEqual(len(platoon_new), 3)
#
# 	def test_extend(self):
# 		platoon1 = Platoon(self.fleet1[:3])
# 		platoon2 = Platoon(self.fleet2[3:])
# 		platoon1.extend(platoon2)
#
# 		self.assertEqual(len(platoon1), 5)


if __name__ == '__main__':
	unittest.main()
