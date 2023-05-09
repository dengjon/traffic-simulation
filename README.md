# Introduction
This repository is created for beginners to implement traffic flow simulation and the main part of this repository is under development.

# Code structure

`platoon-control`: Single-lane platoon simulation with IDM.

`traffic flow/`: Multi-lane traffic flow simulation with IDM car-following model and MOBIL lane-changing model.



# Dev log

如果车辆要换道，那么需要判断当前车道上前后车辆是否满足条件，同时还需要判断目标车道上的前后车辆是否满足条件。

换道行为应该属于`fleet`而不是属于车辆本身。如何进行判断？

Vehicle应该包含的方法：





**ChatGPT magic code **

Simuate the multi-lane traffic flow with Python. The code is based on Object-Oriented programming and seprate the code into different files. The requirements are given below:

1. Add the packages needed in the code automatically.
2. Define <Vehicle> class will be writen in <Vehicle.py> file. Vehicles update their position and speed based on IDM car-following model and MOBIL lane-changing model.
3. Define <Lane> class which will be writen in <Lane.py> file. The Lane has a start and a end and it has two types, which are "Main" and "Ramp". Vehicles could get off the Main road via off ramp.
4. Integrate the params into a <.json> file and load it in the program while using it.
5. Write a <main.py> to implement the whole process and record vehicles' speed, acceleration, and position. After the simulation, draw a plot to represent the traffic flow on the road.
6. The order of generating these codes are: settings.json, Vehicle, Lane, main

Don't just Give me the code structure. I need you to give me the completed code, which could implement the simulation without other efforts.
