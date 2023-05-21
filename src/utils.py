import random
import road
from structure import Fleet, Platoon
from road import *
from vehicle import *
import numpy as np
from typing import List, Optional, Tuple, Union


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
