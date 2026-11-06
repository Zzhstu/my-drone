#!/usr/bin/env python3
"""Bounded actual-flight passage memory, independent of occupancy and energy.

Only observed body motion writes visits. Planned paths never certify coverage.
All preferences are soft, and spatial state is reset after graph revisions.
"""
import math
import numpy as np


class RouteMemory:
    def __init__(self, cell_size=.8, capacity=4096):
        self.cell_size = float(cell_size)
        self.capacity = int(capacity)
        self.total_m = self.repeated_m = 0.
        self.jumps = 0
        self.reset_coordinates()

    def reset_coordinates(self):
        self.visits = {}
        self.previous = None
        self.active_cell = None
        self.heading = None
        self.capacity_limited = False

    def key(self, xy):
        return tuple(math.floor(float(x)/self.cell_size) for x in xy)

    def _enter(self, key):
        if key == self.active_cell:
            return
        if key in self.visits or len(self.visits) < self.capacity:
            self.visits[key] = min(3, self.visits.get(key, 0)+1)
        else:
            self.capacity_limited = True
        self.active_cell = key

    def observe(self, xy):
        q = np.asarray(xy, dtype=float)
        if q.shape != (2,) or not np.isfinite(q).all():
            return
        if self.previous is None:
            self.previous = q.copy()
            self._enter(self.key(q))
            return
        delta = q-self.previous
        length = float(np.linalg.norm(delta))
        if length < .2:
            return
        if length > 1.5:
            # A stale pose gap / frame jump cannot create a traversed corridor.
            self.jumps += 1
            self.previous = q.copy()
            self.active_cell = None
            self.heading = None
            return
        n = max(1, int(math.ceil(length/.15)))
        for i in range(n):
            key = self.key(self.previous+delta*((i+.5)/n))
            self._enter(key)
            self.repeated_m += length/n*int(self.visits.get(key, 0)>1)
        self._enter(self.key(q))
        self.total_m += length
        self.heading = delta/length
        self.previous = q.copy()

    def weight(self, xy):
        key = self.key(xy)
        # The vehicle's current first passage is not already a repeated leg.
        count = self.visits.get(key, 0)-int(key == self.active_cell)
        return min(2, max(0, count))

    def report(self):
        return dict(actual_sampled_travel_m=self.total_m, actual_repeated_tile_travel_m=self.repeated_m,
                    visited_tiles=len(self.visits), tile_size_m=self.cell_size,
                    capacity_limited=self.capacity_limited, rejected_pose_jumps=self.jumps,
                    scope='Actual body passage proxy, not lidar coverage; no occupancy or energy modification')


class RouteQuery:
    """Memoized costs along an EXISTING safe path tree, with a bounded cache."""
    def __init__(self, grid, parent, start, memory):
        self.grid, self.parent, self.start, self.memory = grid, parent, tuple(start), memory
        xy = np.asarray(grid.xy(start))
        self.cache = {self.start: (0., 0., xy, np.zeros(2))}
        self.capacity_limited = False

    def cost(self, target):
        target = tuple(target)
        chain = []
        node = target
        while node not in self.cache:
            if len(chain)+len(self.cache) >= 8192:
                self.capacity_limited = True
                return 0., 0., np.zeros(2), np.zeros(2)
            chain.append(node)
            if isinstance(self.parent, np.ndarray):
                index = int(self.parent[node])
                if index < 0:
                    return 0., 0., np.zeros(2), np.zeros(2)
                node = divmod(index, self.grid.shape[1])
            else:
                node = self.parent.get(node)
                if node is None:
                    return 0., 0., np.zeros(2), np.zeros(2)
        for child in reversed(chain):
            repeat, distance, prefix, _ = self.cache[node]
            a, b = np.asarray(self.grid.xy(node)), np.asarray(self.grid.xy(child))
            length = float(np.linalg.norm(b-a))
            repeat += length*self.memory.weight((a+b)*.5)
            distance += length
            if distance <= 1.2:
                prefix = b
            self.cache[child] = (repeat, distance, prefix, b-a)
            node = child
        repeat, distance, prefix, last_direction = self.cache[target]
        return repeat, distance, prefix-np.asarray(self.grid.xy(self.start)), last_direction


def reversal_cost(heading, direction, maximum_seconds):
    if heading is None:
        return 0.
    a, b = np.asarray(heading), np.asarray(direction)
    norm = float(np.linalg.norm(a)*np.linalg.norm(b))
    return maximum_seconds*max(0., -float(np.dot(a, b))/norm) if norm > 1e-6 else 0.
