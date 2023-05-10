# coding=UTF-8
import random

import lane
from Platoon import *
from lane import *
from vehicle import *
from tqdm import tqdm
import numpy as np
import pandas as pd


def generate_scenario():
	lane_len = 3500
	lane_start = -500
	lane_end = 3000
	# initialize lanes
	lane_list = []
	for i in range(3):
		lane_list.append(Lane.MainLane(i, lane_len, lane_start, lane_end))
	for i in range(3):
		if i == 0:
			lane_list[i].right = lane_list[i + 1]
		elif i == 2:
			lane_list[i].left = lane_list[i - 1]
		else:
			lane_list[i].left = lane_list[i - 1]
			lane_list[i].right = lane_list[i + 1]

	# initialize platoon
	platoon_list = []
	for i in range(3):
		platoon_list.append(Platoon(i, lane_list[i]))
		lane_list[i].platoon = platoon_list[i]
	return lane_list, platoon_list
