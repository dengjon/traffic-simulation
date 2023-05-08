import Lane
from Platoon import *
from Lane import *
from Vehicle import *
from tqdm import tqdm
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import os
import json


def generate_scenario(params: dict, is_ramp=False):
	lane_num = params["Scenario"]["num_lane"]
	lane_len = params["Scenario"]["lane_len"]
	lane_start = params["Scenario"]["lane_start"]
	lane_end = params["Scenario"]["lane_end"]
	# initialize lanes
	lane_list = []
	for i in range(lane_num):
		lane_list.append(MainLane(i, lane_len, lane_start, lane_end))
	for i in range(lane_num):
		if i == 0 and lane_num > 1:
			lane_list[i].right = lane_list[i + 1]
		elif i == lane_num - 1:
			lane_list[i].left = lane_list[i - 1]
		else:
			lane_list[i].left = lane_list[i - 1]
			lane_list[i].right = lane_list[i + 1]

	# initialize platoon
	platoon_list = []
	for i in range(lane_num):
		platoon_list.append(Platoon(i, lane_list[i]))
		lane_list[i].platoon = platoon_list[i]
	return lane_list, platoon_list


def generate_platoon(size: int, platoon: Platoon, params: dict):
	init_speed = 25 + np.random.random((size, 1))
	init_pos = np.linspace(550, 0, size)
	init_acc = np.zeros((size, 1))
	for i in range(size):
		veh_curr = HV(i, init_speed[i], platoon.lane, init_pos[i], init_acc[i], **params["HV"])
		platoon.append(veh_curr)
	return platoon


def main():
	with open("settings.json") as f:
		params = json.load(fp=f)
	# parameters
	num_epochs = params["Simulation"]["num_epochs"]
	time_step = 1
	platoon_len = params["Simulation"]["len_platoon"]
	# generate lane and platoon
	lane_list, platoon_list = generate_scenario(params)
	platoon = generate_platoon(platoon_len, platoon_list[0], params["Vehicle"])
	# preparation
	acc_lead = np.zeros(num_epochs)
	acc_lead[100:102] = -8      # abrupt disturbance
	# simulation
	pos_record = np.zeros((num_epochs, platoon_len))
	speed_record = np.zeros((num_epochs, platoon_len))
	acc_record = np.zeros((num_epochs, platoon_len))
	for i in tqdm(range(int(num_epochs / time_step))):
		platoon.first_veh.acc = acc_lead[i]
		platoon.update_acc()
		platoon.move()
		pos_record[i, :], speed_record[i, :], acc_record[i, :] = platoon.output_state()
	# calculate relative metrics
	relative_spacing = pos_record[:, :-1] - pos_record[:, 1:]
	relative_speed = speed_record[:, :-1] - speed_record[:, 1:]
	relative_acc = acc_record[:, :-1] - acc_record[:, 1:]

	fig, ax = plt.subplots(3, 1, figsize=(8, 8))
	lb, ub = 0, 500
	for i in range(platoon_len):
		ax[0].plot(pos_record[lb:ub, i], label='vehicle' + str(i))
		ax[1].plot(speed_record[lb:ub, i], label='vehicle' + str(i))
		ax[2].plot(acc_record[lb:ub, i], label='vehicle' + str(i))
	ax[0].set(xlabel='Time (sec)', ylabel='Position (m)')
	ax[1].set(xlabel='Time (sec)', ylabel='Speed (m/s)')
	ax[2].set(xlabel='Time (sec)', ylabel='Acceleration (m/s^2)')
	plt.legend()
	plt.savefig(os.path.join('figures', 'figure1.svg'))

	fig, ax = plt.subplots(3, 1, figsize=(8, 8))
	for i in range(platoon_len - 1):
		ax[0].plot(relative_spacing[lb:ub, i], label='vehicle' + str(i))
		ax[1].plot(relative_speed[lb:ub, i], label='vehicle' + str(i))
		ax[2].plot(relative_acc[lb:ub, i], label='vehicle' + str(i))
	ax[0].set(xlabel='Time (sec)', ylabel='Relative Spacing (m)')
	ax[1].set(xlabel='Time (sec)', ylabel='Relative Speed (m/s)')
	ax[2].set(xlabel='Time (sec)', ylabel='Relative Acc (m/s^2)')
	plt.legend()
	plt.savefig(os.path.join('figures', 'figure2.svg'))

	# plt.show()


if __name__ == '__main__':
	main()
