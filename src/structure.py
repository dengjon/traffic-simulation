from vehicle import Vehicle
from typing import List, Optional


class VehicleList(List[Vehicle]):
	def __init__(self, *args, **kwargs):
		super().__init__(*args, **kwargs)

	def append(self, __vehicle: Vehicle) -> None:
		if len(self) > 1:
			self[-1].rear_vehicle = __vehicle
			__vehicle.front_vehicle = self[-1]
		super().append(__vehicle)

	def insert(self, __index: int, __vehicle: Vehicle) -> None:
		if len(self) == 0:
			raise Exception("Empty list cannot insert")

		if 0 < __index < len(self):
			self[__index - 1].rear_vehicle = __vehicle
			__vehicle.front_vehicle = self[__index - 1]
			self[__index].front_vehicle = __vehicle
			__vehicle.rear_vehicle = self[__index]
		else:
			raise Exception("Index out of range")
		super().insert(__index, __vehicle)

	def remove(self, __vehicle: Vehicle) -> None:
		"""
		remove vehicles from list
		:param __vehicle: Vehicle to be removed
		:return:
		"""
		if __vehicle in self:
			if __vehicle.front_vehicle is not None:
				# if __vehicle is not the first vehicle
				__vehicle.front_vehicle.rear_vehicle = __vehicle.rear_vehicle
			if __vehicle.rear_vehicle is not None:
				# if __vehicle is not the last vehicle
				__vehicle.rear_vehicle.front_vehicle = __vehicle.front_vehicle
			super().remove(__vehicle)
		else:
			raise Exception("Vehicle not in list")

	def get_vehicle_by_id(self, veh_id: int) -> Optional[Vehicle]:
		for vehicle in self:
			if vehicle.id == veh_id:
				return vehicle
		return None

	def get_vehicle_by_index(self, index: int) -> Optional[Vehicle]:
		if index < len(self):
			return self[index]
		return None

	def get_vehicle_by_type(self, veh_type: str) -> List[Vehicle]:
		vehicle_list = []
		for vehicle in self:
			if vehicle.type == veh_type:
				vehicle_list.append(vehicle)
		return vehicle_list


class Fleet(VehicleList):
	def __init__(self, *args, **kwargs):
		super().__init__(*args, **kwargs)

	def get_acceleration(self, dt: float) -> List[float]:
		"""
		Updates the position and speed of all vehicles in the fleet based on the car-following model and lane changing.

		:param dt: The time step for the update.
		"""
		acc_list = []  # list of acceleration of all vehicles
		for vehicle in self:
			if vehicle.platoon is None:
				# if the vehicle is not in a platoon
				acc_list.append(vehicle.get_acceleration())
			else:
				# if the vehicle is in a platoon
				platoon = vehicle.platoon
				# get acceleration of the platoon once
				if vehicle in platoon[1:]:
					pass
				else:
					acc_list.extend(vehicle.platoon.get_acceleration(dt))

		return acc_list

	def update(self, dt: float) -> None:
		"""
		Updates the position and speed of all vehicles in the fleet based on the car-following model and lane changing.

		:param dt: The time step for the update.
		"""
		acc_list = self.get_acceleration(dt)

		for i, vehicle in enumerate(self):
			vehicle.update(dt, acc_list[i])


class Platoon(VehicleList):
	def __init__(self, *args, **kwargs):
		super().__init__(*args, **kwargs)
		self.max_size = 0

	def append(self, __vehicle: Vehicle) -> None:
		"""
		Append a vehicle to the platoon
		:param __vehicle:
		:return:
		"""
		__vehicle.platoon = self
		if 0 <= len(self) <= self.max_size:
			# if the platoon is not full, add the vehicle to the platoon
			super().append(__vehicle)
		else:
			# if the platoon is full, raise an exception
			raise Exception("Platoon is full")

	def extend(self, __platoon: 'Platoon') -> None:
		"""
		Extend the current platoon with another platoon, i.e., combine two platoons
		:param __platoon:
		:return:
		"""
		# check if the platoon is empty
		if self.is_empty():
			raise Exception("Empty list cannot extend")
		if __platoon.is_empty() == 0:
			raise Exception("Empty list cannot extend")
		# Create link between two platoons
		self[-1].rear_vehicle = __platoon[0]
		__platoon[0].front_vehicle = self[-1]

		for vehicle in __platoon:
			vehicle.platoon = self
		super().extend(__platoon)

	def split(self, __index: int) -> 'Platoon':
		"""
		Split the platoon into two platoons
		:param __index: index of the vehicle to split
		:return: the new platoon
		"""
		# check if the platoon is empty
		if len(self) == 0:
			raise Exception("Empty list cannot split")
		if 0 < __index < len(self):
			# Create a new platoon
			new_platoon = Platoon(self[__index:])

			# delete the vehicles in the new platoon from the old platoon
			for vehicle in new_platoon:
				self.remove(vehicle)
			return new_platoon
		else:
			raise Exception("Index out of range")

	def remove(self, __vehicle: Vehicle) -> None:
		"""
		remove vehicles from list
		:param __vehicle: Vehicle to be removed
		:return:
		"""
		if __vehicle in self:
			__vehicle.platoon = None
			super().remove(__vehicle)
		else:
			raise Exception("Vehicle not in list")

	def pop(self, __index: int = ...) -> Vehicle:
		"""
		remove vehicles from list by __index
		:param __index: index of the vehicle to be removed
		:return:
		"""
		__vehicle = super().pop(__index)
		__vehicle.platoon = None
		return __vehicle

	def get_acceleration(self, dt: float) -> List[float]:
		"""
		Updates the position and speed of all vehicles in the platoon based on the car-following model and lane changing.

		:param dt: The time step for the update.
		"""
		# get acceleration of the first vehicle
		acc_list = [self[0].get_acceleration()]

		# get acceleration of the rest vehicles
		acc_rest = self.control(dt)

		# combine the two acceleration lists
		acc_list.extend(acc_rest)
		return acc_list

	def control(self, dt: float = 1) -> List[float]:
		"""Control method of the platoon"""
		# Placeholder for control method, currently replaced with IDM model
		acc_list = []
		if not self.is_empty():
			for vehicle in self[1:]:
				acc_list.append(vehicle.get_acceleration())
		else:
			raise Exception("Platoon is empty")

		return acc_list

	def is_empty(self) -> bool:
		"""Check if the platoon is empty"""
		return len(self) == 0
