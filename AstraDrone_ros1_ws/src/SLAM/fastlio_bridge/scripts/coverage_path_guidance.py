#!/usr/bin/env python3
"""Bounded coverage-region guidance over *observed safe* 2-D flight cells.

FALCON-inspired hierarchy only: global region ordering followed by local
frontier selection. This is not the FALCON implementation or 3-D exploration.
No world model, collision boxes, or simulator truth are accepted as input.
"""
import math
from collections import defaultdict

import numpy as np


class CoveragePathGuide:
    def __init__(self, region_size=2.5, max_regions=8, lookahead_weight=.6,
                 revisit_weight=.4):
        values = (region_size, lookahead_weight, revisit_weight)
        if (not all(math.isfinite(float(v)) for v in values) or
                not 1.0 <= region_size <= 8.0 or
                not 1 <= max_regions <= 16 or
                not 0.0 <= lookahead_weight <= 2.0 or
                not 0.0 <= revisit_weight <= 2.0):
            raise ValueError('Invalid coverage path guidance parameters')
        self.region_size = float(region_size)
        self.max_regions = int(max_regions)
        self.lookahead_weight = float(lookahead_weight)
        self.revisit_weight = float(revisit_weight)

    def region(self, xy):
        # Global map-metre keys remain stable when the planning grid grows.
        return tuple(int(math.floor(float(v)/self.region_size)) for v in xy)

    def select(self, grid, options, gain, distances, visited=(), clearance_weight=0.):
        """Return (safe reachable frontier cell, bounded guidance diagnostics).

        Current-to-region and region-to-region costs come from the same safe
        grid Dijkstra used by flight. Region order only guides selection; the
        ordinary planner still validates and publishes every local path.
        """
        groups = defaultdict(list)
        for row in options:
            cell = tuple(int(v) for v in row)
            if not grid.inside(cell) or not grid.safe[cell]:
                continue
            distance = float(distances[cell])
            reward = float(gain[cell])
            if not math.isfinite(distance) or not math.isfinite(reward) or reward <= 0:
                continue
            groups[self.region(grid.xy(cell))].append((cell,reward,distance))
        if not groups:
            return None, dict(candidate_regions=0,route_regions=[],reason='no_safe_reachable_frontier')

        counts = defaultdict(int)
        for cell,_ in visited:
            if grid.inside(cell):
                counts[self.region(grid.xy(cell))] += 1

        regions = []
        for key, entries in groups.items():
            entries.sort(key=lambda e: e[1]/(2.+e[2]),reverse=True)
            representative = entries[0]
            # Multiple frontiers in one region matter, but must not make a
            # densely sampled wall win solely through duplicate gain.
            reward = entries[0][1]+.2*sum(e[1] for e in entries[1:3])
            reward /= 1.+self.revisit_weight*counts[key]
            regions.append(dict(key=key,cell=representative[0],
                                reward=reward,distance=representative[2],
                                local_count=len(entries)))
        regions.sort(key=lambda r:r['reward']/(2.+r['distance']),reverse=True)
        regions = regions[:self.max_regions]

        # A bounded set of exact observed-safe searches supplies the next-leg
        # costs. Unknown cells are never inserted into the route graph.
        legs = {}
        for i,region in enumerate(regions):
            if len(regions) == 1:
                break
            other_distances,_ = grid.search(region['cell'],clearance_weight=clearance_weight)
            for j,other in enumerate(regions):
                if i != j:
                    legs[i,j] = float(other_distances[other['cell']])

        scores = []
        for i,region in enumerate(regions):
            immediate = region['reward']/(2.+region['distance'])
            second = max((other['reward']/(2.+region['distance']+legs[i,j])
                          for j,other in enumerate(regions)
                          if j != i and math.isfinite(legs[i,j])),default=0.)
            scores.append(immediate+self.lookahead_weight*second)
        first = int(np.argmax(scores))
        route = [first]
        remaining = set(range(len(regions)))-{first}
        while remaining:
            current = route[-1]
            reachable = [j for j in remaining if math.isfinite(legs[current,j])]
            if not reachable:
                break
            nxt = max(reachable,key=lambda j:regions[j]['reward']/(2.+legs[current,j]))
            route.append(nxt)
            remaining.remove(nxt)
        selected = regions[first]
        report = dict(candidate_regions=len(groups),considered_regions=len(regions),
                      region_size_m=self.region_size,
                      selected_region=list(selected['key']),
                      selected_region_score=float(scores[first]),
                      selected_region_frontiers=selected['local_count'],
                      route_regions=[list(regions[i]['key']) for i in route],
                      route_scope='observed-safe fixed-altitude regions')
        return selected['cell'],report
