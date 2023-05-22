import random
import road
from models import *
from structure import Fleet, Platoon
from road import *
from vehicle import *
import numpy as np
from typing import List, Optional, Tuple, Union, Any


def generate_scenario():
	# Generate and initialize lanes
	lane_list = []

	# Example lane creation
	lane1 = MainLane(length=3500, start=-500, end=3500, lane_type='Main')
	lane2 = MainLane(length=3500, start=-500, end=3500, lane_type='Main')
	lane3 = MainLane(length=3500, start=-500, end=3500, lane_type='Main')

	# Set neighboring lanes
	lane1.right_lane = lane2  # Set the neighboring lane to the right of lane1
	lane2.left_lane = lane1  # Set the neighboring lane to the left of lane2
	lane2.right_lane = lane3  # Set the neighboring lane to the right of lane2
	lane3.left_lane = lane2  # Set the neighboring lane to the left of lane3

	lane_list.append(lane1)  # Add lane1 to the lane list
	lane_list.append(lane2)  # Add lane2 to the lane list
	lane_list.append(lane3)  # Add lane3 to the lane list

	# Generate and initialize fleets
	for lane_curr in lane_list:
		lane_curr.fleet = Fleet()  # Create a fleet for each lane
		lane_curr.fleet.lane = lane_curr  # Set the lane of the fleet

	return lane_list


def generate_vehicles():
	"""generate vehicles by specific distribution and 'safety_check()' """
	# TODO: generate vehicles based on specific distribution

	pass


def generate_vehicle_main(lane_list: List[Lane], configs: dict, permeability: float = 0):
	"""
	Generate vehicles on main lane
	:param configs: settings of vehicle
	:param lane_list:  list of main lane
	:param permeability: permeability of generating vehicles
	:return:
	"""
	can_add_list = []
	lambda_param = 1
	speed = 15
	headway = np.random.exponential(1 / lambda_param) * 50
	headway = np.clip(headway, 20, 100)
	if random.random() < permeability:
		vehicle_curr = CAV(speed, None, lane_list[0].start, 0, **configs['CAV'])
	else:
		vehicle_curr = HV(speed, None, lane_list[0].start, 0, **configs['HV'])

	for lane_curr in lane_list:
		vehicle_curr.lane = lane_curr
		if lane_curr.type == 'Main':
			last_vehicle: Optional[Vehicle] = lane_curr.fleet.rear_vehicle
			if last_vehicle is None:
				# If there is no vehicle in the lane, set the position to 0
				vehicle_curr.position = 0
				can_add_list.append(lane_curr)
			else:
				vehicle_curr.position = last_vehicle.position - headway
				if vehicle_curr.position < lane_curr.start:
					continue
				if safety_check(last_vehicle, vehicle_curr):
					can_add_list.append(lane_curr)
		else:
			pass

	# If there is no lane that can add vehicle, return None
	if len(can_add_list) == 0:
		return None
	else:
		# Add vehicle to the lane
		lane_curr = random.choice(can_add_list)
		if lane_curr.fleet.rear_vehicle is None:
			# If there is no vehicle in the lane, set the position to 0
			vehicle_curr.position = 0
			lane_curr.fleet.add_vehicle(vehicle_curr)
		else:
			last_vehicle = lane_curr.fleet.rear_vehicle
			vehicle_curr.position = last_vehicle.position - headway
			lane_curr.fleet.add_vehicle(vehicle_curr, last_vehicle)


def generate_vehicle_ramp(lane_list: List[Lane], configs, permeability):
	"""
	Generate vehicles on ramp
	:param lane_list:
	:param configs:
	:param permeability:
	:return:
	"""
	pass


def safety_check(front_vehicle: Vehicle, rear_vehicle: Vehicle):
	"""
	Checks the safety between vehicles in case of initial collisions.

	:param front_vehicle: The front vehicle.
	:param rear_vehicle: The rear vehicle.
	:return: True if the vehicles are safe, False otherwise.
	"""
	is_safe = True  # Assume the vehicles are safe initially

	if front_vehicle is not None and rear_vehicle is not None:
		# Calculate the acceleration of the rear vehicle and the deceleration limit
		acceleration = rear_vehicle.get_acceleration()
		deceleration_limit = -rear_vehicle.desired_dec

		# Check if the acceleration exceeds the deceleration limit
		if acceleration < deceleration_limit:
			is_safe = False

		gap = front_vehicle.position - rear_vehicle.position - front_vehicle.length

		# Check if the gap between the vehicles is smaller than the rear vehicle's jam distance
		if gap < rear_vehicle.jam_distance:
			is_safe = False

	return is_safe


class Simulator:
	def __init__(self, configs: dict, permeability: float = 0):
		self.lane_list = generate_scenario()
		self.permeability = permeability
		self.road = Road(self.lane_list)
		self.configs = configs

	def step(self):
		"""
		Step the simulator forward by 1 time step.
		"""
		pass

	def get_adjacent_vehicle(self, fleet: Fleet, direction: str) \
			-> tuple[list[int], list[int]]:
		"""
		Get the front vehicles and rear vehicles in adjacent lane of vehicles in current lane.
		One vehicle in current lane has one front vehicle in adjacent lane.

		:param fleet: The fleet of vehicles.
		:param direction: The direction of adjacent lane.
		:return: The front vehicle of the vehicle.
		"""
		assert direction in ['left', 'right']
		lane_curr = self.road.get_lane_by_index(fleet.lane_id)

		if direction == 'left':
			adjacent_lane = lane_curr.left_lane
		else:
			adjacent_lane = lane_curr.right_lane

		if adjacent_lane is None:
			raise ValueError('No adjacent lane in the direction of {}.'.format(direction))

		front_vehicle_list = []
		rear_vehicle_list = []
		fleet_adj = adjacent_lane.fleet
		for vehicle_curr in fleet:
			# find the front vehicle of vehicle_curr in the target lane according to the vehicle's position
			front_vehicle_curr = None
			rear_vehicle_curr = None
			for i, vehicle_target in enumerate(fleet_adj):
				delta_dist = vehicle_target.position - vehicle_curr.position
				if delta_dist > 0:
					front_vehicle_curr = vehicle_target
				else:
					rear_vehicle_curr = vehicle_target
					break

			front_vehicle_list.append(front_vehicle_curr)
			rear_vehicle_list.append(rear_vehicle_curr)
		return front_vehicle_list, rear_vehicle_list

	def get_lc_intention(self, fleet: Fleet, front_veh_list_left: Optional[List[Vehicle]] = None,
	                     front_veh_list_right: Optional[List[Vehicle]] = None,
	                     rear_veh_list_left: Optional[List[Vehicle]] = None,
	                     rear_veh_list_right: Optional[List[Vehicle]] = None):
		"""
		Get the lane change intention of the vehicles in the fleet.

		:param fleet: The fleet of vehicles.
		:param front_veh_list_left: A list of the front vehicles in the left adjacent lanes.
		:param front_veh_list_right: A list of the front vehicles in the right adjacent lanes.
		:param rear_veh_list_left: A list of the rear vehicles in the left adjacent lanes.
		:param rear_veh_list_right: A list of the rear vehicles in the right adjacent lanes.
		:return:
		"""
		lane_curr = self.road.get_lane_by_index(fleet.lane_id)
		left_lane = lane_curr.left_lane
		right_lane = lane_curr.right_lane

		if left_lane is None and right_lane is None:
			raise ValueError('At least one of the adjacent lanes should exist.')

		if left_lane is not None and right_lane is not None:
			for i, vehicle in enumerate(fleet):
				if vehicle.platoon is not None:
					# if the vehicle is in a platoon, it will not change lane
					continue

				incentive_left = vehicle.get_lc_incentive(vehicle, front_veh_list_left[i], rear_veh_list_left[i])
				incentive_right = vehicle.get_lc_incentive(vehicle, front_veh_list_right[i], rear_veh_list_right[i])

				if incentive_left > incentive_right:
					if incentive_left > vehicle.lc_threshold:
						vehicle.lc_intention = 'left'
						vehicle.lc_front_vehicle = front_veh_list_left[i]
				else:
					if incentive_right > vehicle.lc_threshold:
						vehicle.lc_intention = 'right'
						vehicle.lc_front_vehicle = front_veh_list_right[i]

		elif front_veh_list_left is not None:
			for i, vehicle in enumerate(fleet):
				if vehicle.platoon is not None:
					# if the vehicle is in a platoon, it will not change lane
					continue
				incentive_left = vehicle.get_lc_incentive(vehicle, front_veh_list_left[i], rear_veh_list_left[i])
				if incentive_left > vehicle.lc_threshold:
					vehicle.lc_intention = 'left'
					vehicle.lc_front_vehicle = front_veh_list_left[i]

		elif front_veh_list_right is not None:
			for i, vehicle in enumerate(fleet):
				if vehicle.platoon is not None:
					# if the vehicle is in a platoon, it will not change lane
					continue

				incentive_right = vehicle.get_lc_incentive(vehicle, front_veh_list_right[i], rear_veh_list_right[i])
				if incentive_right > vehicle.lc_threshold:
					vehicle.lc_intention = 'right'
					vehicle.lc_front_vehicle = front_veh_list_right[i]

	def change_lane(self, fleet: Fleet):
		"""
		Change lane according to the lane change intention of the vehicles.

		:param fleet: The fleet of vehicles.
		"""
		lane_left = self.road.get_lane_by_index(fleet.lane_id).left_lane
		lane_right = self.road.get_lane_by_index(fleet.lane_id).right_lane

		for vehicle in fleet:
			if vehicle.lc_intention is not None:
				if vehicle.lc_intention == 'left':
					index = lane_left.fleet.index(vehicle.lc_front_vehicle)
					lane_left.fleet.insert(index + 1, vehicle)
					fleet.remove(vehicle)
					vehicle.lane = lane_left
				elif vehicle.lc_intention == 'right':
					index = lane_right.fleet.index(vehicle.lc_front_vehicle)
					lane_right.fleet.insert(index + 1, vehicle)
					fleet.remove(vehicle)
					vehicle.lane = lane_right
