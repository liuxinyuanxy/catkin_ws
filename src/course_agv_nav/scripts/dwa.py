#!/usr/bin/env python
# -*- coding: utf-8 -*-
import math

import numpy as np

class Config:
    def __init__(self):
        self.obs_radius = 0.5
        self.dt = 0.1  # [s] Time tick for motion prediction

        self.max_speed = 0.2  # [m/s]
        self.min_speed = -0.2  # [m/s]
        self.max_accel = 1  # [m/ss]
        self.v_reso = self.max_accel * self.dt / 10.0  # [m/s]

        self.max_yawrate = 100.0 * math.pi / 180.0  # [rad/s]
        self.max_dyawrate = 100.0 * math.pi / 180.0  # [rad/ss]
        self.yawrate_reso = self.max_dyawrate * self.dt / 10.0  # [rad/s]

        self.predict_time = 1  # [s]

        self.to_goal_cost_gain = 1.0
        self.speed_cost_gain = 0.1
        self.obstacle_cost_gain = 1.0

        self.tracking_dist = self.predict_time * self.max_speed
        self.arrive_dist = 0.1


config = Config()
class DWAPlanner():
    def dist(self, state, config, ob, v, omega):
        # return 100
        x, y, theta, _, _ = state
        nearest_dist = np.inf
        for i in range(2):
            x += v * math.cos(theta) * config.dt
            y += v * math.sin(theta) * config.dt
            theta += omega * config.dt
            for (ox, oy) in zip(ob[:, 0], ob[:, 1]):
                d = math.sqrt((x - ox) ** 2 + (y - oy) ** 2)
                nearest_dist = min(nearest_dist, d - config.obs_radius)
        return nearest_dist

    def heading_cost(self, state, config, goal, v, omega):
        x, y, theta, _, _ = state
        dx = goal[0] - x
        dy = goal[1] - y
        goal_theta = math.atan2(dy, dx)
        theta += omega * config.dt
        cost = abs(goal_theta - theta)
        for i in range(10):
            x += v * math.cos(theta) * config.dt
            y += v * math.sin(theta) * config.dt
            theta += omega * config.dt
        dis = math.sqrt((goal[0] - x) ** 2 + (goal[1] - y) ** 2)
        return cost + dis

    def speed_cost(self, config, v):
        return config.max_speed - v

    def plan(self, state, goal, ob):
        x, y, theta, v, omega = state
        ans_v = 0.1
        ans_omega = 0.1
        min_cost = np.inf
        v_min = max(config.min_speed, v - config.max_accel * config.dt)
        v_max = min(config.max_speed, v + config.max_accel * config.dt)
        omega_min = max(-config.max_yawrate, omega - config.max_dyawrate * config.dt)
        omega_max = min(config.max_yawrate, omega + config.max_dyawrate * config.dt)
        for v in np.arange(v_min, v_max, config.v_reso):
            for omega in np.arange(omega_min, omega_max, config.yawrate_reso):
                dist_ = self.dist(state, config, ob, v, omega)
                if dist_ < 0:
                    continue
                if v > math.sqrt(2 * dist_ * config.max_accel):
                    continue
                if omega > math.sqrt(2 * dist_ * config.max_dyawrate):
                    continue
                dist_cost = 1 / (dist_ + 1)
                cost = self.heading_cost(state, config, goal, v, omega) * config.to_goal_cost_gain + self.speed_cost(
                    config, v) * config.speed_cost_gain + dist_cost * config.obstacle_cost_gain
                if min_cost >= cost:
                    min_cost = cost
                    ans_v = v
                    ans_omega = omega
        # print([ans_v, ans_omega])
        return [ans_v, ans_omega]
