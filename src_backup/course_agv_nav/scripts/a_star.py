#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np


class RRTNode:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None
        self.depth = 0

class RRTTree:
    def __init__(self, start_x, start_y):
        self.start = RRTNode(start_x, start_y)
        self.nodes = [self.start]

    def get_nearest_node(self, x, y):
        dists = []
        for node in self.nodes:
            dists.append(np.hypot(node.x - x, node.y - y))
        dists = np.array(dists)
        idx = np.argmin(dists)
        return self.nodes[idx]

    def add_node(self, x, y, parent: RRTNode):
        node = RRTNode(x, y)
        node.parent = parent
        node.depth = parent.depth + np.hypot(x - parent.x, y - parent.y)
        self.nodes.append(node)
        return node

    def get_path(self, target_node):
        path = []
        node = target_node
        while node is not None:
            path.append(node)
            node = node.parent
        return path[::-1]

    def nearest_k_nodes(self, x, y, k):
        dists = []
        for node in self.nodes:
            dists.append(np.hypot(node.x - x, node.y - y))
        dists = np.array(dists)
        idx = np.argsort(dists)
        return [self.nodes[i] for i in idx[:k]]

    def choose_parent(self, x, y, nodes):
        min_dist = np.inf
        min_node = None
        for node in nodes:
            dist = np.hypot(node.x - x, node.y - y) + node.depth
            if dist < min_dist:
                min_dist = dist
                min_node = node
        return min_node

class AStarPlanner:
    def __init__(self, step_size, obs_radius):
        self.obs_radius = obs_radius
        self.step_size = step_size

    def sample(self, min_x, min_y, max_x, max_y):
        return np.random.uniform(min_x, max_x), np.random.uniform(min_y, max_y)

    def steer(self, x1, y1, x2, y2):
        dist = np.hypot(x1 - x2, y1 - y2)
        if dist < self.step_size:
            return x2, y2
        else:
            return x1 + self.step_size * (x2 - x1) / dist, y1 + self.step_size * (y2 - y1) / dist

    def is_collision(self, x, y, obs_x, obs_y):
        for ox, oy in zip(obs_x, obs_y):
            if np.hypot(x - ox, y - oy) < self.obs_radius:
                return True
        return False

    def is_collision_on_path(self, start_x, start_y, target_x, target_y, obs_x, obs_y):
        for ox, oy in zip(obs_x, obs_y):
            if np.hypot(start_x - ox, start_y - oy) < self.obs_radius:
                return True
            if np.hypot(target_x - ox, target_y - oy) < self.obs_radius:
                return True
            # calculate the area of the triangle (ox, oy) (start_x, start_y) (target_x, target_y)
            area = np.abs((ox - target_x) * (start_y - target_y) - (start_x - target_x) * (oy - target_y)) / 2.0
            # calculate the distance from the point (ox, oy) to the line (start_x, start_y) (target_x, target_y)
            d = area / np.hypot(start_x - target_x, start_y - target_y)
            if d < self.obs_radius:
                return True
        return False

    def planning(self, obs_x, obs_y, start_x, start_y, target_x, target_y, min_x, min_y, max_x, max_y):
        # do the rrt
        target = RRTNode(target_x, target_y)
        tree = RRTTree(start_x, start_y)
        for i in range(10000):
            x_rand = self.sample(min_x, min_y, max_x, max_y)
            nearest_node = tree.get_nearest_node(*x_rand)
            x_new = self.steer(nearest_node.x, nearest_node.y, *x_rand)
            if not self.is_collision(*x_new, obs_x, obs_y) and not self.is_collision_on_path(nearest_node.x,
                                                                                             nearest_node.y, *x_new,
                                                                                             obs_x,
                                                                                             obs_y):
                new_node = tree.add_node(*x_new, nearest_node)
                if np.hypot(new_node.x - target_x,
                            new_node.y - target_y) < self.step_size and not self.is_collision_on_path(
                    new_node.x, new_node.y, target_x, target_y, obs_x, obs_y):
                    target = tree.add_node(target_x, target_y, new_node)
                    print("find the path")
                    path = tree.get_path(target)
                    px = [node.x for node in path]
                    py = [node.y for node in path]
                    return px, py

        print("cannot find the path")