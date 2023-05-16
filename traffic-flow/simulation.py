# coding=UTF-8
import random

import lane
from platoon import *
from lane import *
from vehicle import *
from fleet import *
from tqdm import tqdm
import numpy as np
import pandas as pd
from typing import List, Optional, Tuple, Union
import matplotlib.pyplot as plt
import utils


def main(settgings: dict):
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
			lane_curr.fleet.get_lane_change_intention(dt)
		for lane_curr in lane_list:
			lane_curr.fleet.update_vehicles(dt)


if __name__ == '__main__':
	import json

	with open('settings.json', 'r') as f:
		settings = json.load(f)
	main(settings)
