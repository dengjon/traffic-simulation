from vehicle import *
from models import *
from platoon import *


class Fleet(object):
	def __init__(self, lane=None):
		"""
		Initializes a fleet of vehicles.
		"""
		self.front_vehicle = None  # Placeholder for the front vehicle
		self.lane = lane  # Placeholder for the lane the fleet is in
		self.rear_vehicle = None  # Placeholder for the rear vehicle
		self.vehicles = []  # List to store the vehicles in the fleet
		self.lc_vehicle_list = []  # List to store the vehicles that want to change lanes
		self.lc_direction_list = []  # List to store the direction of the lane change for each vehicle

	def add_vehicle(self, vehicle: Vehicle, front_vehicle: Optional[Vehicle] = None):
		"""
		Adds a vehicle to the fleet if there is space.

		If target_front_vehicle is None, adds the vehicle as the front vehicle in the fleet.

		:param front_vehicle: The vehicle to add the new vehicle behind.
		:param vehicle: The vehicle to add.
		:return:
		"""
		if len(self.vehicles) == 0:  # Fleet is empty
			self.vehicles.append(vehicle)
			self.front_vehicle = vehicle
			self.rear_vehicle = vehicle
			vehicle.lane = self.lane
			return

		target_front_vehicle = front_vehicle
		vehicle.lane = self.lane  # Update the lane of the vehicle.
		if target_front_vehicle is None:  # No target front vehicle
			# Add the vehicle as the front vehicle of the fleet
			self.vehicles.insert(0, vehicle)

			self.front_vehicle.front_vehicle = vehicle  # Placeholder for setting the front vehicle property
			vehicle.last_vehicle = self.front_vehicle  # Placeholder for setting the last vehicle property
			self.front_vehicle = vehicle  # Update the front vehicle
		else:  # Add the vehicle behind the target front vehicle
			# Insert the new vehicle behind the target front vehicle
			index = self.vehicles.index(target_front_vehicle)
			self.vehicles.insert(index + 1, vehicle)

			target_rear_vehicle = target_front_vehicle.rear_vehicle  # Placeholder for the target front vehicle's rear vehicle
			target_front_vehicle.rear_vehicle = vehicle  # Update the rear vehicle of the target front vehicle
			vehicle.front_vehicle = target_front_vehicle  # Update the front vehicle of the added vehicle
			if target_rear_vehicle is None:
				# If the target front vehicle has no rear vehicle, set the new vehicle as the rear vehicle
				self.rear_vehicle = vehicle
			else:
				# If the target front vehicle has a rear vehicle, insert the new vehicle between them
				target_rear_vehicle.front_vehicle = vehicle
				vehicle.rear_vehicle = target_rear_vehicle  # Update the rear vehicle of the added vehicle

	def remove_vehicle(self, vehicle: Vehicle):
		"""
		Removes the given vehicle from the fleet.

		:param vehicle: The vehicle to remove.
		"""
		self.vehicles.remove(vehicle)  # Remove the vehicle from the fleet

		if self.front_vehicle == vehicle:  # Vehicle to remove is the front vehicle
			if vehicle.rear_vehicle is not None:
				self.front_vehicle = vehicle.rear_vehicle  # Update the front vehicle
				return  # No more vehicles in the fleet, return
			else:
				self.front_vehicle = None  # No more vehicles in the fleet, update the front vehicle
				self.rear_vehicle = None
				return  # No more vehicles in the fleet, return

		elif self.rear_vehicle == vehicle:  # Vehicle to remove is the rear vehicle
			if vehicle.front_vehicle is not None:
				self.rear_vehicle = vehicle.front_vehicle  # Update the rear vehicle
				return  # No more vehicles in the fleet, return
			else:
				self.front_vehicle = None
				self.rear_vehicle = None
				return  # No more vehicles in the fleet, return
		else:  # Vehicle to remove is in the middle of the fleet
			if vehicle.front_vehicle is not None:
				# Update the front vehicle of the vehicle behind the vehicle to remove
				vehicle.front_vehicle.rear_vehicle = vehicle.rear_vehicle

			if vehicle.rear_vehicle is not None:
				# Update the rear vehicle of the vehicle in front of the vehicle to remove
				vehicle.rear_vehicle.front_vehicle = vehicle.front_vehicle

	def get_lane_change_intention(self, front_veh_list_lc_left: Optional[List[Vehicle]] = None,
	                              front_veh_list_lc_right: Optional[List[Vehicle]] = None):
		"""
		Updates the lane change intention of the vehicles in the fleet.
		:param front_veh_list_lc_left: A list of the front vehicles in the adjacent lanes.
		:param front_veh_list_lc_right: A list of the front vehicles in the adjacent lanes.
		:return:
		"""
		mobil = MOBIL()  # Create an instance of the MOBIL model
		if front_veh_list_lc_left is None and front_veh_list_lc_right is None:
			# If no adjacent lane, return
			return

		if front_veh_list_lc_left is not None and front_veh_list_lc_right is not None:
			for i, vehicle in enumerate(self.vehicles):
				front_vehicle_curr_left = front_veh_list_lc_left[i]  # the front vehicle in the left lane
				front_vehicle_curr_right = front_veh_list_lc_right[i]  # the front vehicle in the right lane
				if mobil.check_safety(vehicle, front_vehicle_curr_left) and \
						mobil.check_safety(vehicle, front_vehicle_curr_right):
					incentive_left = mobil.get_incentive(vehicle,
					                                     front_vehicle_curr_left)  # Get the incentive to change lanes
					incentive_right = mobil.get_incentive(vehicle,
					                                      front_vehicle_curr_right)
					if incentive_left > incentive_right:
						if incentive_left > mobil.delta:
							# Add the vehicle to the list of vehicles that want to change lanes
							self.lc_vehicle_list.append(vehicle)
							self.lc_direction_list.append('left')
					else:
						if incentive_right > mobil.delta:
							# Add the vehicle to the list of vehicles that want to change lanes
							self.lc_vehicle_list.append(vehicle)
							self.lc_direction_list.append('right')

				elif mobil.check_safety(vehicle, front_vehicle_curr_left):
					incentive_left = mobil.get_incentive(vehicle,
					                                     front_vehicle_curr_left)
					if incentive_left > mobil.delta:
						# Add the vehicle to the list of vehicles that want to change lanes
						self.lc_vehicle_list.append(vehicle)
						self.lc_direction_list.append('left')

				elif mobil.check_safety(vehicle, front_vehicle_curr_right):
					incentive_right = mobil.get_incentive(vehicle,
					                                      front_vehicle_curr_right)
					if incentive_right > mobil.delta:
						# Add the vehicle to the list of vehicles that want to change lanes
						self.lc_vehicle_list.append(vehicle)
						self.lc_direction_list.append('right')

				else:
					continue
		elif front_veh_list_lc_left is not None:
			for i, vehicle in enumerate(self.vehicles):
				front_vehicle_curr_left = front_veh_list_lc_left[i]
				if mobil.check_safety(vehicle, front_vehicle_curr_left):
					incentive_left = mobil.get_incentive(vehicle,
					                                     front_vehicle_curr_left)
					if incentive_left > mobil.delta:
						# Add the vehicle to the list of vehicles that want to change lanes
						self.lc_vehicle_list.append(vehicle)
						self.lc_direction_list.append('left')
		elif front_veh_list_lc_right is not None:
			for i, vehicle in enumerate(self.vehicles):
				front_vehicle_curr_right = front_veh_list_lc_right[i]
				if mobil.check_safety(vehicle, front_vehicle_curr_right):
					incentive_right = mobil.get_incentive(vehicle,
					                                      front_vehicle_curr_right)
					if incentive_right > mobil.delta:
						# Add the vehicle to the list of vehicles that want to change lanes
						self.lc_vehicle_list.append(vehicle)
						self.lc_direction_list.append('right')

	def change_lane(self):
		"""
		Changes the lane of the vehicles in the fleet.
		:return:
		"""
		for i, vehicle in enumerate(self.lc_vehicle_list):
			if self.lc_direction_list[i] == 'left':
				vehicle.move_to_lane(self.lane.left_lane)
			elif self.lc_direction_list[i] == 'right':
				vehicle.move_to_lane(self.lane.right_lane)

	def update_vehicles(self, dt: float):
		"""
		Updates the position and speed of all vehicles in the fleet based on the car-following model and lane changing.

		:param dt: The time step for the update.
		"""
		acc_list = []  # List to store the calculated accelerations for each vehicle

		for i, vehicle in enumerate(self.vehicles):
			acc_list.append(vehicle.get_acceleration())  # Placeholder for calculating the acceleration

		for i, vehicle in enumerate(self.vehicles):
			# Update the vehicle's speed and position based on the acceleration
			acceleration = acc_list[i]
			vehicle.update(acceleration, dt)

	def get_front_vehicle_list(self, target_lane):
		"""
		Returns a list of the front vehicles in the target lane.

		:param target_lane: The target lane.
		:return: A list of the front vehicles in the target lane.
		"""
		if target_lane is None:
			return None

		front_vehicle_list = []
		start = 0
		for vehicle_curr in self.vehicles:
			# find the front vehicle of vehicle_curr in the target lane according to the vehicle's position
			front_vehicle_curr = None
			for i, vehicle_target in enumerate(target_lane.fleet.vehicles[start:]):
				if vehicle_curr.position < vehicle_target.position:
					# If the vehicle is in front of the target vehicle, set the target vehicle to be the front vehicle
					front_vehicle_curr = vehicle_target
					# Update the start index for the next vehicle
					start = i
					break
			if front_vehicle_curr is None:
				# If no front vehicle, set the front vehicle to be the first vehicle in the target lane
				start = 0
			front_vehicle_list.append(front_vehicle_curr)

		return front_vehicle_list

	def get_states(self, state_type: float = 'position'):
		"""
		Returns a list of the states of the vehicles in the fleet.

		:param state_type: The type of state to return.
		:return: A list of the states of the vehicles in the fleet.
		"""
		assert state_type in ['position', 'speed', 'acceleration', 'gap']
		state_list = []
		# Iterate through the vehicles in the fleet and add the state to the list
		for vehicle in self.vehicles:
			if state_type == 'position':
				state_list.append(vehicle.position)
			elif state_type == 'speed':
				state_list.append(vehicle.speed)
			elif state_type == 'acceleration':
				state_list.append(vehicle.acc)
			elif state_type == 'gap':
				# Calculate the gap between the vehicle and the front vehicle
				if vehicle.front_vehicle is not None:
					delta_loc = vehicle.front_vehicle.position - vehicle.position
					gap = delta_loc - vehicle.length
					state_list.append(gap)
				else:
					state_list.append(-1)
		return state_list
