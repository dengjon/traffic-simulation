from vehicle import *
from models import *
from platoon import *


class Fleet(object):
	def __init__(self):
		"""
		Initializes a fleet of vehicles.
		"""
		self.front_vehicle = None  # Placeholder for the front vehicle
		self.rear_vehicle = None  # Placeholder for the rear vehicle
		self.vehicles = []  # List to store the vehicles in the fleet

	def add_vehicle(self, vehicle: Vehicle, target_front_vehicle: Optional[Vehicle] = None):
		"""
		Adds a vehicle to the fleet if there is space.

		If target_front_vehicle is None, adds the vehicle as the front vehicle in the fleet.

		:param vehicle: The vehicle to add.
		:param target_front_vehicle: The front vehicle in the fleet to add the vehicle behind.
		"""
		if len(self.vehicles) == 0:  # Fleet is empty
			self.vehicles.append(vehicle)
			self.front_vehicle = vehicle
			self.rear_vehicle = vehicle
			return

		if target_front_vehicle is None:  # No target front vehicle
			# Add the vehicle as the front vehicle of the fleet
			self.vehicles.insert(0, vehicle)

			self.front_vehicle.lead_vehicle = vehicle  # Placeholder for setting the lead vehicle property
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
			self.front_vehicle = vehicle.rear_vehicle  # Update the front vehicle
			if self.front_vehicle is not None:
				self.front_vehicle.front_vehicle = None  # Update the front vehicle's front vehicle
			else:
				self.rear_vehicle = None  # No more vehicles in the fleet, update the rear vehicle
		elif self.rear_vehicle == vehicle:  # Vehicle to remove is the rear vehicle
			self.rear_vehicle = vehicle.front_vehicle  # Update the rear vehicle
			if self.rear_vehicle is not None:
				self.rear_vehicle.rear_vehicle = None  # Update the rear vehicle's rear vehicle
			else:
				self.front_vehicle = None  # No more vehicles in the fleet, update the front vehicle
		else:  # Vehicle to remove is in the middle of the fleet
			vehicle.front_vehicle.rear_vehicle = vehicle.rear_vehicle  # Update the front vehicle's rear vehicle
			vehicle.rear_vehicle.front_vehicle = vehicle.front_vehicle  # Update the rear vehicle's front vehicle

	def change_lane(self, vehicle: Vehicle, target_front_vehicle: Optional[Vehicle] = None):
		"""
		Adds a vehicle to the fleet if there is space for lane changing.

		If target_front_vehicle is None, adds the vehicle as the front vehicle in the fleet.

		:param vehicle: The vehicle to add.
		:param target_front_vehicle: The front vehicle in the fleet to add the vehicle behind.
		"""
		if target_front_vehicle is None:
			# Add the vehicle to the current lane's fleet as the front vehicle
			self.add_vehicle(vehicle)

			# Remove the vehicle from its current lane's fleet
			vehicle.lane.fleet.remove_vehicle(vehicle)
		else:
			# Add the vehicle to the current lane's fleet behind the target front vehicle
			self.add_vehicle(vehicle, target_front_vehicle)

			# Remove the vehicle from its current lane's fleet
			vehicle.lane.fleet.remove_vehicle(vehicle)

	def update_vehicles(self, dt: float):
		"""
		Updates the position and speed of all vehicles in the fleet based on the car-following model and lane changing.

		:param dt: The time step for the update.
		"""
		mobil = MOBIL()  # Create an instance of the MOBIL model
		acc_list = []  # List to store the calculated accelerations for each vehicle

		for i, vehicle in enumerate(self.vehicles):
			# Handle lane changing based on the MOBIL model
			new_lane = None
			if mobil.can_change_lane(vehicle, 'left', dt):  # Placeholder for determining if lane change is possible
				new_lane = vehicle.lane.left_lane  # Placeholder for getting the left neighboring lane
			elif mobil.can_change_lane(vehicle, 'right', dt):
				new_lane = vehicle.lane.right_lane

			if new_lane is not None:
				vehicle.move_to_lane(new_lane)  # Placeholder for moving the vehicle to the new lane
			else:
				# Calculate the acceleration for this vehicle based on the car-following model
				acc_list.append(vehicle.get_acceleration())  # Placeholder for calculating the acceleration

		for i, vehicle in enumerate(self.vehicles):
			# Update the vehicle's speed and position based on the acceleration
			acceleration = acc_list[i]
			vehicle.update(acceleration, dt)


