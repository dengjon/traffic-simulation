# Introduction
This repository is created for beginners to implement traffic flow simulation and the main part of this repository is under development.

*Warning*: This repository is **under development !!!**. If you unfortunately find it, just go away and **don't waste time on it**!

# Code structure

`src/`: Multi-lane traffic flow simulation with IDM car-following model and MOBIL lane-changing model.

# Dev log

**TODO**

1. Modify the lane-changing part. Search the fleet once for each iteration.
2. Complete platoon control in traffic flow simulation.

File structure：

- `models.py`：car-following and lane-changing functions

  1. `IDM`: car-following functions for human drivers

  - Attributes: 
  - Functions:
    `get_acceleration`: calculates the acceleration of a vehicle based on the Intelligent Driver Model (IDM)

  2. `ACC`: car-following functions for automated vehicles
     `get_acceleration`: calculates the acceleration of an automated vehicle based on the Adaptive Cruise Control (ACC) model.

  3. `MOBIL`: lane-changing functions for human drivers

     - Attributes:

     - Functions:
       1. `can_change_lane`: checks whether a vehicle can change lanes based on the MOBIL (Minimizing Overall Braking Induced by Lane changes) model
       2. `get_lane_change_direction`: determines the direction of the lane change (left, right, or none) based on the MOBIL model

- `vehicle.py`:  abstract classes of various vehicles
  - `Vehicle`: parent class of various vehicles
    - Attributes: `length`, `lane`, `position`, `speed`, and `type` .
    - Functions:
      `update`: updates the vehicle's position and speed based on the acceleration calculated by the car-following model
  - `HV`: human-driving vehicles (Inherits from `Vehicle`).
    - Functions:
      `get_acceleration`: calculates the acceleration of a human-driven vehicle based on the car-following model (IDM or ACC).
  - `AV`: automated vehicles
  - `CAV`: connected and automated vehicles

- `road.py`: road class to accommodate vehicles
  - Attributes:
    - `lane_list`: list to accommodate different types of lane
  - Functions:

- `lane.py`: lane class to represent different types of lane
  - Attributes:
    - `lane_type`: type of lane
  - Functions:
    - `add_vehicle`: adds a vehicle to the road at the specified position and lane.
    - `remove_vehicle`: removes a vehicle from the road.
    - `check_collisions`: check for collisions between vehicles
    - `update`: updates the position and speed of all vehicles on the road based on their acceleration calculated by the car-following model. This function should also handle lane changing based on the MOBIL model.

- `platoon.py`: vehicle platoon

  - **Attributes:**

    - `leader`: The leading vehicle of the platoon.
    - `rear_vehicle`: The trailing vehicle of the platoon.
    - `num_vehicles`: The number of vehicles in the platoon.

  - **Methods:**
    - `add_vehicle(vehicle)`: Adds a new vehicle to the platoon.
    - `remove_vehicle(vehicle)`: Removes a vehicle from the platoon.
    - `split_platoon(vehicle)`: Splits the platoon into two separate platoons at the given vehicle.
    

- `simulation.py`: Implement the simulation
  
  - `generate_scenario`: generate lanes, fleet and initialize their relationships.
  - `generate_vehicle_main`: generate vehicles on main lane based on specific distributions of headway and safety check.
    - What distribution: negative exponential distribution?
    - Put vehicles on the "warm up" lane to make them enter the main lane at a steady state.
  - `generate_vehicle_ramp`: same as the last one, on ramp
  - `safety_check`: judge the safety between vehicles by location and acceleration while generating them.
