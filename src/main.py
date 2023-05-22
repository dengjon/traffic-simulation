# Created by Dengjiang Sun, sdengjon@gmail.com at 20:22 2023/5/22 using PyCharm
from utils import Simulator
from tqdm import tqdm
import numpy as np
import os
import matplotlib.pyplot as plt


def main():
	num_epochs = 1000
	simulator = Simulator()

	road = simulator.road
	vehicle_total_list = []
	for lane_curr in road:
		for vehicle in lane_curr.fleet:
			vehicle_total_list.append(vehicle)

	position_record = np.zeros((num_epochs, len(vehicle_total_list)))
	velocity_record = np.zeros((num_epochs, len(vehicle_total_list)))
	acceleration_record = np.zeros((num_epochs, len(vehicle_total_list)))
	gap_record = np.zeros((num_epochs, len(vehicle_total_list)))

	for epoch in tqdm(range(num_epochs)):
		simulator.step()
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
	main()
