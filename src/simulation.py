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
import os


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
	vehicle_total_list = []
	for lane_curr in lane_list:
		for vehicle in lane_curr.fleet.vehicles:
			vehicle_total_list.append(vehicle)

	position_record = np.zeros((num_epochs, len(vehicle_total_list)))
	velocity_record = np.zeros((num_epochs, len(vehicle_total_list)))
	acceleration_record = np.zeros((num_epochs, len(vehicle_total_list)))
	gap_record = np.zeros((num_epochs, len(vehicle_total_list)))

	for epoch in tqdm(range(num_epochs)):
		for lane_curr in lane_list:
			front_veh_list_left = lane_curr.fleet.get_front_vehicle_list(lane_curr.left_lane)
			front_veh_list_right = lane_curr.fleet.get_front_vehicle_list(lane_curr.right_lane)
			lane_curr.fleet.get_lane_change_intention(front_veh_list_left, front_veh_list_right)

		for lane_curr in lane_list:
			lane_curr.fleet.change_lane()

		for lane_curr in lane_list:
			lane_curr.fleet.update_vehicles(dt)

		for i, vehicle in enumerate(vehicle_total_list):
			position_record[epoch, i] = vehicle.position
			velocity_record[epoch, i] = vehicle.speed
			acceleration_record[epoch, i] = vehicle.acc
			if vehicle.front_vehicle is not None:
				gap_record[epoch, i] = vehicle.front_vehicle.position - vehicle.position - vehicle.length

	# plot position, speed, and acceleration of each vehicle in separate axes
	fig, axs = plt.subplots(4, 1, figsize=(10, 12))
	for i, vehicle in enumerate(vehicle_total_list):
		axs[0].plot(position_record[:, i])
		axs[1].plot(velocity_record[:, i])
		axs[2].plot(acceleration_record[:, i])
		axs[3].plot(gap_record[:, i])
	axs[0].set_title('Position')
	axs[1].set_title('Velocity')
	axs[2].set_title('Acceleration')
	axs[3].set_title('Gap')

	# save figure as png in 'figure/' directory without white space
	# if figure directory does not exist, create one
	if not os.path.exists('figures/'):
		os.makedirs('figures/')
	plt.savefig('figures/position.png', bbox_inches='tight')

	plt.show()


if __name__ == '__main__':
	import json

	with open('settings.json', 'r') as f:
		settings = json.load(f)
	main()
