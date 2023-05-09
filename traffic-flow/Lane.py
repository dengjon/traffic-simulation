from Vehicle import Vehicle


class Lane:
	def __init__(self, length, max_speed, capacity, start, end, lane_type):
		# Initialize the properties of the lane
		self.length = length
		self.max_speed = max_speed
		self.capacity = capacity
		self.start = start
		self.end = end
		self.type = lane_type
		self.vehicles = []  # List to hold the vehicles currently on the lane
		self.lanes = []  # List of neighboring lanes

	def add_vehicle(self, vehicle):
		# Add a vehicle to the lane if there is space
		if len(self.vehicles) < self.capacity:
			self.vehicles.append(vehicle)
			vehicle.lane_index = len(self.lanes)  # Assign the vehicle to the current lane
			vehicle.lanes.append(self)  # Add the lane to the list of lanes the vehicle is on
		else:
			print("Lane is full")

	def remove_vehicle(self, vehicle):
		# Remove a vehicle from the lane
		self.vehicles.remove(vehicle)

	def update_vehicles(self, dt):
		# Sort the vehicles in the lane by their positions
		self.vehicles.sort(key=lambda v: v.position)

		# Update the lanes of the vehicles
		for vehicle in self.vehicles:
			vehicle.update_lane_index(self)

		# Update the positions of the vehicles based on the MOBIL model
		for i, vehicle in enumerate(self.vehicles):
			left_lane = self.lanes[vehicle.lane_index - 1] if vehicle.lane_index > 0 else None
			right_lane = self.lanes[vehicle.lane_index + 1] if vehicle.lane_index < len(vehicle.lanes) - 1 else None

			# Update the acceleration of the vehicle based on the MOBIL model
			a_left, a_right = vehicle.acceleration_left_right(left_lane, right_lane)
			if a_left >= vehicle.desired_dec and (a_right <= a_left or right_lane is None):
				delta_a_left = a_left - a_right if right_lane is not None else 0
				if delta_a_left > vehicle.min_lateral_distance:
					# Move the vehicle to the left lane
					vehicle.lane_index -= 1
					self.vehicles.remove(vehicle)
					left_lane.add_vehicle(vehicle)

			elif a_right >= vehicle.desired_dec and (a_left <= a_right or left_lane is None):
				delta_a_right = a_right - a_left if left_lane is not None else 0
				if delta_a_right > vehicle.min_lateral_distance:
					# Move the vehicle to the right lane
					vehicle.lane_index += 1
					self.vehicles.remove(vehicle)
					right_lane.add_vehicle(vehicle)

		# Update the positions of the vehicles based on their speeds
		for vehicle in self.vehicles:
			vehicle.position += vehicle.speed * dt

	def distance_to_front_vehicle(self, vehicle):
		# Calculate the distance to the vehicle in front of the given vehicle on the lane
		distance_to_front = self.length
		for v in self.vehicles[vehicle.lane_index + 1:]:
			if v.position > vehicle.position and v.position - vehicle.position < distance_to_front:
				distance_to_front = v.position - vehicle.position
		return distance_to_front

	def front_rear_vehicle(self, vehicle):
		"""
		Find the front and rear vehicles of a given vehicle in the lane.

		Args:
			vehicle (Vehicle): The vehicle to find the front and rear vehicles of.

		Returns:
			A tuple (front_vehicle, rear_vehicle) of the front and rear vehicles of the given vehicle in the lane, or
			(None, None) if the given vehicle is the only vehicle in the lane.
		"""
		front_vehicle = None
		rear_vehicle = None
		for v in self.vehicles:
			if v != vehicle:
				if v.position < vehicle.position and (rear_vehicle is None or v.position > rear_vehicle.position):
					rear_vehicle = v
				elif v.position > vehicle.position and (front_vehicle is None or v.position < front_vehicle.position):
					front_vehicle = v
		return front_vehicle, rear_vehicle


if __name__ == '__main__':
	print('hello')
