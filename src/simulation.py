# coding=UTF-8
import lane
from platoon import *
from lane import *
from vehicle import *
from fleet import *
from tqdm import tqdm
import numpy as np
import matplotlib.pyplot as plt
import utils


def main():
	init_speed = 20
	num_epochs = 1000
	dt = 0.1

	# Generate and initialize lanes
	lane_list = utils.generate_scenario()

	# Generate and initialize vehicles
	for i in range(1000):
		utils.generate_vehicle_main(lane_list, settings['Vehicle'])

	# Run simulation
	for epoch in tqdm(range(num_epochs)):
		for lane_curr in lane_list:
			front_veh_list_left = lane_curr.fleet.get_front_vehicle_list(lane_curr.left_lane)
			front_veh_list_right = lane_curr.fleet.get_front_vehicle_list(lane_curr.right_lane)
			lane_curr.fleet.get_lane_change_intention(front_veh_list_left, front_veh_list_right)

		for lane_curr in lane_list:
			lane_curr.fleet.change_lane()

		for lane_curr in lane_list:
			lane_curr.fleet.update_vehicles(dt)


if __name__ == '__main__':
	import json

	with open('settings.json', 'r') as f:
		settings = json.load(f)
	main()
