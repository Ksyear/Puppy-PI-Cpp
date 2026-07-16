#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS에 의존하지 않는 frontier 탐색과 A* 경로 계획 코어."""

import heapq
import math
from collections import deque


class GridMap(object):
    """nav_msgs/OccupancyGrid의 가벼운 순수 Python 표현."""

    CARDINAL = ((1, 0), (-1, 0), (0, 1), (0, -1))
    DIAGONAL = ((1, 1), (1, -1), (-1, 1), (-1, -1))

    def __init__(self, width, height, resolution, origin_x, origin_y, data,
                 free_threshold=25, occupied_threshold=65):
        self.width = int(width)
        self.height = int(height)
        self.resolution = float(resolution)
        self.origin_x = float(origin_x)
        self.origin_y = float(origin_y)
        self.data = list(data)
        self.free_threshold = int(free_threshold)
        self.occupied_threshold = int(occupied_threshold)
        expected = self.width * self.height
        if self.width <= 0 or self.height <= 0 or len(self.data) != expected:
            raise ValueError('invalid occupancy grid dimensions')

    def in_bounds(self, cell):
        x, y = cell
        return 0 <= x < self.width and 0 <= y < self.height

    def value(self, cell):
        x, y = cell
        return self.data[y * self.width + x]

    def is_unknown(self, cell):
        return self.in_bounds(cell) and self.value(cell) < 0

    def is_free(self, cell):
        if not self.in_bounds(cell):
            return False
        value = self.value(cell)
        return 0 <= value <= self.free_threshold

    def is_occupied(self, cell):
        return self.in_bounds(cell) and self.value(cell) >= self.occupied_threshold

    def world_to_cell(self, x, y):
        return (
            int(math.floor((x - self.origin_x) / self.resolution)),
            int(math.floor((y - self.origin_y) / self.resolution)),
        )

    def cell_to_world(self, cell):
        return (
            self.origin_x + (cell[0] + 0.5) * self.resolution,
            self.origin_y + (cell[1] + 0.5) * self.resolution,
        )

    def _neighbors(self, cell, diagonal=False):
        offsets = self.CARDINAL + (self.DIAGONAL if diagonal else ())
        for dx, dy in offsets:
            nxt = (cell[0] + dx, cell[1] + dy)
            if self.in_bounds(nxt):
                yield nxt

    def inflated_obstacles(self, radius_m):
        radius_cells = max(0, int(math.ceil(float(radius_m) / self.resolution)))
        occupied = []
        for y in range(self.height):
            for x in range(self.width):
                if self.is_occupied((x, y)):
                    occupied.append((x, y))
        if radius_cells == 0:
            return set(occupied)

        blocked = set()
        radius_sq = radius_cells * radius_cells
        for ox, oy in occupied:
            for dy in range(-radius_cells, radius_cells + 1):
                for dx in range(-radius_cells, radius_cells + 1):
                    if dx * dx + dy * dy <= radius_sq:
                        cell = (ox + dx, oy + dy)
                        if self.in_bounds(cell):
                            blocked.add(cell)
        return blocked

    def frontier_cells(self, blocked=None):
        """미탐사 셀과 맞닿은 안전한 자유 셀을 반환."""
        blocked = blocked or set()
        frontiers = set()
        for y in range(self.height):
            for x in range(self.width):
                cell = (x, y)
                if cell in blocked or not self.is_free(cell):
                    continue
                if any(self.is_unknown(n) for n in self._neighbors(cell)):
                    frontiers.add(cell)
        return frontiers

    def frontier_clusters(self, blocked=None, min_size=3):
        remaining = self.frontier_cells(blocked)
        clusters = []
        while remaining:
            seed = remaining.pop()
            queue = deque([seed])
            cluster = [seed]
            while queue:
                current = queue.popleft()
                for nxt in self._neighbors(current, diagonal=True):
                    if nxt in remaining:
                        remaining.remove(nxt)
                        queue.append(nxt)
                        cluster.append(nxt)
            if len(cluster) >= int(min_size):
                clusters.append(cluster)
        return clusters

    def astar(self, start, goal, blocked=None):
        """자유 셀 위 8방향 A*. 도달 불가능하면 빈 리스트."""
        blocked = set(blocked or ())
        blocked.discard(start)
        if not self.in_bounds(start) or not self.in_bounds(goal):
            return []
        if not self.is_free(start) or not self.is_free(goal) or goal in blocked:
            return []

        def heuristic(a, b):
            return math.hypot(a[0] - b[0], a[1] - b[1])

        frontier = [(heuristic(start, goal), 0.0, start)]
        came_from = {}
        cost_so_far = {start: 0.0}
        while frontier:
            _, cost, current = heapq.heappop(frontier)
            if cost > cost_so_far.get(current, float('inf')):
                continue
            if current == goal:
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                path.reverse()
                return path

            for nxt in self._neighbors(current, diagonal=True):
                if nxt in blocked or not self.is_free(nxt):
                    continue
                dx = nxt[0] - current[0]
                dy = nxt[1] - current[1]
                if dx and dy:
                    # 막힌 모서리를 대각선으로 뚫고 지나가지 않는다.
                    side_a = (current[0] + dx, current[1])
                    side_b = (current[0], current[1] + dy)
                    if (side_a in blocked or side_b in blocked or
                            not self.is_free(side_a) or not self.is_free(side_b)):
                        continue
                new_cost = cost + (math.sqrt(2.0) if dx and dy else 1.0)
                if new_cost < cost_so_far.get(nxt, float('inf')):
                    cost_so_far[nxt] = new_cost
                    came_from[nxt] = current
                    priority = new_cost + heuristic(nxt, goal)
                    heapq.heappush(frontier, (priority, new_cost, nxt))
        return []

    def frontier_candidates(self, start, robot_radius=0.18, min_cluster_size=3,
                            max_candidates=5, blacklist=None, blacklist_radius=0.35,
                            min_frontier_distance=0.30):
        """도달 가능한 frontier 후보를 탐색 효율 순서로 반환."""
        blocked = self.inflated_obstacles(robot_radius)
        blocked.discard(start)
        blacklist = list(blacklist or ())
        blacklist_cells = float(blacklist_radius) / self.resolution

        candidates = []
        clusters = self.frontier_clusters(blocked, min_cluster_size)
        for cluster in clusters:
            ordered = sorted(
                cluster,
                key=lambda c: (c[0] - start[0]) ** 2 + (c[1] - start[1]) ** 2)
            goal = None
            path = []
            for cell in ordered:
                if math.hypot(cell[0] - start[0], cell[1] - start[1]) * \
                        self.resolution < min_frontier_distance:
                    continue
                if any(math.hypot(cell[0] - bad[0], cell[1] - bad[1]) <= blacklist_cells
                       for bad in blacklist):
                    continue
                path = self.astar(start, cell, blocked)
                if path:
                    goal = cell
                    break
            if goal is None:
                continue
            candidates.append({
                'cell': goal,
                'path': path,
                'cluster_size': len(cluster),
                'distance_m': max(0, len(path) - 1) * self.resolution,
            })

        # 넓은 미탐사 영역을 선호하되 너무 먼 목표는 피한다.
        for candidate in candidates:
            candidate['utility'] = (
                candidate['cluster_size'] * self.resolution /
                max(candidate['distance_m'], self.resolution))
        candidates.sort(key=lambda item: (-item['utility'], item['distance_m']))
        candidates = candidates[:max(1, int(max_candidates))]
        for index, candidate in enumerate(candidates):
            candidate['id'] = index
            candidate['world'] = self.cell_to_world(candidate['cell'])
        return candidates


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def choose_lookahead(path, grid, x, y, lookahead_m):
    """현재 위치에서 lookahead 이상 떨어진 첫 경로점을 선택."""
    if not path:
        return None
    closest_index = min(
        range(len(path)),
        key=lambda i: math.hypot(grid.cell_to_world(path[i])[0] - x,
                                 grid.cell_to_world(path[i])[1] - y))
    for cell in path[closest_index:]:
        wx, wy = grid.cell_to_world(cell)
        if math.hypot(wx - x, wy - y) >= lookahead_m:
            return wx, wy
    return grid.cell_to_world(path[-1])
