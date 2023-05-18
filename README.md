# Introduction
This repository is created for beginners to implement traffic flow simulation and the main part of this repository is under development.

*Warning* ⚠️: This repository is **under development !!!**. If you unfortunately find it, just go away and **don't waste time on it**!

# TODO

1. Modify the lane-changing part. Search the fleet once for each iteration to improve speed. ✅
   - Problem: Lane-changing vehicles still have to search the fleet for their front vehicles.
   - 💡：Input the front vehicle and target lane simultaneously. ✅
2. Implement ramp lane, where vehicles can come in from outside and get off from main lane.
   - Forbid lane-changing behavior: 
     - 💡: Set the left and right lane of ramp as `None` 
   - Lane for acceleration and deceleration  beside the main lane
   - Maximum speed limit: Easy
   - Maximum length limit: Easy
   - Mandatory lane-changing behavior at weaving area
     - 💡: Add a function to calculate the vehicles' maximum acceleration, which is based on the comparison of vehicles' car-following acceleration and acceleration caused by their destination. For example, a vehicle requiring exit at off-ramp will gradually slow down while approaching the off-ramp and eventually stop waiting for lane-changing opportunities.
3. Complete platoon control in traffic flow simulation.
   - Vehicle in a platoon can not change lane
   - Vehicle can request to get off a platoon
   - Small platoons can combine to a bigger one

# Code structure

`src/`: Multi-lane traffic flow simulation with IDM car-following model and MOBIL lane-changing model.

**File structure**：

- `models.py`：car-following and lane-changing functions
  - `IDM`: car-following functions for human drivers
    - Attributes: 
    - `get_acceleration`: calculates the acceleration of a vehicle based on the Intelligent Driver Model (IDM)
  - `ACC`: car-following functions for automated vehicles (TODO)
    - `get_acceleration`: calculates the acceleration of an automated vehicle based on the Adaptive Cruise Control (ACC) model.
  - `MOBIL`: lane-changing functions for human drivers
  - `get_incentive`: calculate vehicles' lane-changing incentives based on the MOBIL (Minimizing Overall Braking Induced by Lane changes) model
  - `__calculate_accleration_lc`: Protected function. Calculate acceleration of following vehicle on target lane after lane-changing behavior. Create pseudo vehicles and calculate acceleration with vehicles' `get_acceleration()` method.
  - `__calculate_acceleration_curr`: Protected function. Calculate acceleration of following vehicle on current lane.
- `vehicle.py`:  classes of various vehicles
  - `Vehicle`: parent class of various vehicles
    - `get_acceleration`: Calculate vehicle's accleration based on different car-following models.
    - `update`: updates the vehicle's position and speed based on the acceleration calculated by the car-following model
    - `move_to_lane`: move the ego vehicle to target lane.
    - `__get_adjacent_lead_vehicle`: Protected funcion. Get the front vehicle in the target lane which the ego vehicle intends to change to.
    - `__restore_states`: Protected function. Restore the states of vehicles at each iteration to make it convenient to extract for plots.
  - `HV`: human-driving vehicles (Inherits from `Vehicle`).
    - `get_acceleration`: **overwrite**. calculates the acceleration of a human-driven vehicle based on the car-following model (IDM or ACC).
  - `AV`: automated vehicles
  - `CAV`: connected and automated vehicles
- `lane.py`: lane class to represent different types of lane
  - `Lane`: Parent class of differen lane types
  - `MainLane`: Main road
  - `Ramp`: In ramp or off ramp
- `platoon.py`: vehicle platoon (To be implemented in the future)
  - `add_vehicle(vehicle)`: Adds a new vehicle to the platoon.
  - `remove_vehicle(vehicle)`: Removes a vehicle from the platoon.
  - `split_platoon(vehicle)`: Splits the platoon into two separate platoons at the given vehicle.
- `utils.py`: functions used in simulation.
  - `generate_scenario`: generate lanes, fleet and initialize their relationships.
  - `generate_vehicle_main`: generate vehicles on main lane based on specific distributions of headway and safety check.
    - What distribution: negative exponential distribution?
    - Put vehicles on the "warm up" lane to make them enter the main lane at a steady state.
  - `generate_vehicle_ramp`: same as the last one, on ramp
  - `safety_check`: judge the safety between vehicles by location and acceleration while generating them.
- `test.py`: Test file to check functions of different classes with `unittest` 
