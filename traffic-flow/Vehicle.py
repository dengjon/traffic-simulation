from typing import Union, Any
from Platoon import Platoon
import numpy as np
from Lane import *
import copy
import math


class Vehicle(object):
    def __init__(self, vehicle_id: int, init_speed: int, init_lane: Lane,
                 init_pos: int, init_acc: float, **settings) -> None:
        # initial state
        self.id = vehicle_id
        self.speed = init_speed
        self.lane = init_lane
        self.position = init_pos
        self.acc = init_acc

        # Properties to update
        self.front_vehicle = None
        self.rear_vehicle = None

        # Parameters to overwrite
        self.lane_change_indicator = False
        self.type = ''

        # fetch from settings
        self.desired_speed_main = settings.get('desired_speed_main', 25)  # default desired speed in main lane
        self.desired_speed_ramp = settings.get('desired_speed_ramp', 16.66)  # default desired speed in ramp
        self.max_speed = settings.get('max_speed', 120)  # default max speed
        self.min_speed = settings.get('min_speed', 0)
        self.jam_distance = settings.get('jam_distance', 10)  # default safety distance
        self.min_gap = settings.get('min_gap', 0)
        self.max_acc = settings.get('max_acc', 2)
        self.desired_dec = settings.get('desired_dec', 3)
        self.reaction_time = settings.get('reaction_time', 0)
        self.len = settings.get('len', 5)
        self.target_pos = settings.get('target_pos', 0)

    def car_following(self):
        """
        Compute the acceleration of the vehicle based on the Intelligent Driver Model (IDM).
        """
        if self.front_vehicle is None:
            s_star = self.jam_distance
        else:
            s_star = self.jam_distance + self.speed * self.reaction_time + self.speed * (
                        self.speed - self.front_vehicle.speed) / (2 * math.sqrt(self.max_acc * self.desired_dec))

        d_front = self.front_vehicle.position - self.position if self.front_vehicle is not None else float('inf')

        a = self.max_acc * (1 - (self.speed / self.desired_speed_main) ** 4 - (
                    s_star / max(self.min_gap, min(self.jam_distance, d_front))) ** 2)

        return a

    def update(self, target_lane: Lane, target_acc: float):
        """
        Update the location and lane of the vehicle.
        """
        self.acc = target_acc

        # Update position based on speed and acceleration
        self.position += self.speed + 0.5 * self.acc

        # Update lane
        if self.lane != target_lane:
            self.lane.remove_vehicle(self)
            target_lane.add_vehicle(self)
            self.lane = target_lane

        # Update front and rear vehicles
        self.front_vehicle, self.rear_vehicle = self.lane.front_rear_vehicle(self)

        # Update speed based on acceleration
        self.speed += self.acc

        # Enforce speed limits
        self.speed = min(self.speed, self.max_speed)
        self.speed = max(self.speed, self.min_speed)


class HV(Vehicle):
    def __init__(self, vehicle_id: int, init_speed: int, init_lane: Lane,
                 init_pos: int, init_acc: float, **settings) -> None:
        super().__init__(vehicle_id, init_speed, init_lane, init_pos, init_acc, **settings["global"])
        self.type = 'HV'

    def car_following(self):
        v = self.speed
        a = self.max_acc
        b = self.desired_dec
        s0 = self.jam_distance
        if self.lane.type == 'ramp':
            v0 = self.desired_speed_ramp
        else:
            v0 = self.desired_speed_main
        if self.front_vehicle is not None:
            T = self.reaction_time
            # Calculate acceleration
            delta_v = v - self.front_vehicle.speed
            s = max(0.01, - self.position + self.front_vehicle.position - self.front_vehicle.length)
            if s < 0.02:
                # 距离太近，撞车了
                return -100
            s_star = s0 + max(0, v * T + v * delta_v *
                              0.5 / (a * b) ** 0.5)
            acc = a * (1 - (v / v0) ** 4 - (s_star / s) ** 2)
        else:
            acc = a * (1 - (v / v0) ** 4)
        return acc


class CAV(Vehicle):
    def __init__(self, vehicle_id: int, init_speed: int, init_lane: Lane,
                 init_pos: int, init_acc: float, **settings) -> None:
        super().__init__(vehicle_id, init_speed, init_lane, init_pos, init_acc)
        self.type = 'CAV'
        self.len = 4
        self.reaction_time = 0
        self.settings = settings["CAV"]

    def car_following(self):
        v = self.speed
        a = self.max_acc
        b = self.desired_dec
        s0 = self.jam_distance
        if self.lane.type == 'ramp':
            v0 = self.desired_speed_ramp
        else:
            v0 = self.desired_speed_main
        if self.front_vehicle is not None:
            if self.front_vehicle.type == 'HV':
                T = self.settings["time-gap-hv"]
            elif self.front_vehicle.type == 'CAV':
                T = self.settings["time-gap-cav"]
            else:
                raise ValueError(-1)
        if self.front_vehicle is not None:
            if self.front_vehicle.front_vehicle is not None:
                delta_v1 = self.speed - self.front_vehicle.speed
                delta_v2 = self.front_vehicle.speed - self.front_vehicle.front_vehicle.speed
                s1 = max(0.01, self.front_vehicle.position - self.position - self.front_vehicle.length)
                s2 = max(0.01, self.front_vehicle.front_vehicle.position - self.front_vehicle.position -
                         self.front_vehicle.front_vehicle.length)
                delta1 = abs(delta_v1) / (s1 ** 0.5)
                delta2 = abs(delta_v2) / (s2 ** 0.5)
                try:
                    lambda1 = delta1 / (delta1 + delta2)
                    lambda2 = delta2 / (delta1 + delta2)
                except ZeroDivisionError:
                    lambda1 = 0.5
                    lambda2 = 0.5
                delta_v = lambda1 * delta_v1 + lambda2 * delta_v2
                s = lambda1 * s1 + lambda2 * s2
                if (s1 < 0.02) | (s < 0.02):
                    return -100
            else:
                delta_v = self.speed - self.front_vehicle.speed
                s = max(0.01, -self.position + self.front_vehicle.position - self.front_vehicle.length)
                if s < 0.02:
                    return -100
            s_star = s0 + max(0, v * T + v * delta_v * 0.5 / (a * b) ** 0.5)
            acc = a * (1 - (v / v0) ** 4 - (s_star / s) ** 2)
        else:
            acc = a * (1 - (v / v0) ** 4)
        return acc
