from robot_epuck import Robot
import sim as vrep  # V-rep library
import sys
import time  # used to keep track of time
import numpy as np  # array library
import math
import random
import psutil


class Modc:

    def __init__(self, sensor_dist, det_obj):
        self.num_sensors = len(sensor_dist)
        self.sensor_dist = sensor_dist  # will be provided in np.array form
        self.detection_obj = det_obj  # will be provided in np.array form
        self.danger_nr = float()  # Danger associated with no of robots connected with robot

    def no_neigh_rob(self, target_no_robots, diffusion_rate):  # An external danger # Decided diffusion rate is 0.025
        no_nei_rob = (self.detection_obj == 1).sum()  # Count number of robots connected with robot
        if target_no_robots == 2:  # Diffuse danger when current no robots == target no of robots
            self.danger_nr = (1 - diffusion_rate) * self.danger_nr
        else:
            self.danger_nr = self.danger_nr + (target_no_robots - no_nei_rob) * (diffusion_rate)
            self.danger_nr = 0.0 if self.danger_nr < 0.0 else 1 if self.danger_nr > 1 else self.danger_nr
        return self.danger_nr

    # Base on danger theory immune cell must respond to harmful signal not necessarily to foreign bodies
    # Both safe and internal danger signal will be generated from it.

    def self_nonself(self, rob_min_limit, rob_max_limit, obstacle_limit):
        for i in range(self.num_sensors):
            if self.detection_obj[i]:  # Presence of robot
                if not (rob_min_limit < self.sensor_dist[i] > rob_max_limit):  # danger produce by robot
                    self.ext_danger[i] = self.ext_danger[i] + 0.04
                    self.safe_signal[i] = self.safe_signal[i] - 0.04
                else:  # robot in safe range
                    self.ext_danger[i] = self.ext_danger[i] - 0.04
                    self.safe_signal[i] = self.safe_signal[i] + 0.04
            elif not self.detection_obj[i]:  # Presence of Obstacle
                if self.sensor_dist[i] < obstacle_limit :  # danger produce by obstacle
                    self.ext_danger[i] = self.ext_danger[i] + 0.04
                    self.safe_signal[i] = self.safe_signal[i] - 0.04
                else:  # obstacle in safe range
                    self.ext_danger[i] = self.ext_danger[i] - 0.04
                    self.safe_signal[i] = self.safe_signal[i] + 0.04
            else:  # nothing in front of sensor
                continue

