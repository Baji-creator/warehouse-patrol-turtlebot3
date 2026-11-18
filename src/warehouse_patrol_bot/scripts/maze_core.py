#!/usr/bin/env python
# -*- coding: utf-8 -*-

import random
import heapq

GRID = 11
CELL = 1.2
WALL = 0.12
SEED = 20261018

# 每个格子记录四面是否有墙：北、东、南、西
DIRS = [
    (-1, 0, 0, 2),
    (0, 1, 1, 3),
    (1, 0, 2, 0),
    (0, -1, 3, 1)
]

def build_maze():
    random.seed(SEED)
    walls = [[[True, True, True, True] for _ in range(GRID)]
             for _ in range(GRID)]
    visited = set()
    stack = [(0, 0)]
    visited.add((0, 0))

    while stack:
        r, c = stack[-1]
        choices = []
        for dr, dc, direction, opposite in DIRS:
            nr, nc = r + dr, c + dc
            if 0 <= nr < GRID and 0 <= nc < GRID:
                if (nr, nc) not in visited:
                    choices.append((nr, nc, direction, opposite))

        if not choices:
            stack.pop()
            continue

        nr, nc, direction, opposite = random.choice(choices)
        walls[r][c][direction] = False
        walls[nr][nc][opposite] = False
        visited.add((nr, nc))
        stack.append((nr, nc))

    # 加少量额外通路，形成环路，避免只有单一路径
    for r in range(1, GRID - 1):
        for c in range(1, GRID - 1):
            if random.random() < 0.10:
                choices = []
                if walls[r][c][1] and c + 1 < GRID:
                    choices.append((r, c, 1, r, c + 1, 3))
                if walls[r][c][2] and r + 1 < GRID:
                    choices.append((r, c, 2, r + 1, c, 0))
                if choices:
                    a = random.choice(choices)
                    _, _, d1, r2, c2, d2 = a
                    walls[r][c][d1] = False
                    walls[r2][c2][d2] = False

    return walls

def neighbors(walls, cell):
    r, c = cell
    result = []
    for dr, dc, direction, opposite in DIRS:
        nr, nc = r + dr, c + dc
        if 0 <= nr < GRID and 0 <= nc < GRID:
            if not walls[r][c][direction]:
                result.append((nr, nc))
    return result

def astar(walls, start, goal):
    frontier = [(0, start)]
    came_from = {start: None}
    cost = {start: 0}

    while frontier:
        _, current = heapq.heappop(frontier)
        if current == goal:
            break

        for nxt in neighbors(walls, current):
            new_cost = cost[current] + 1
            if nxt not in cost or new_cost < cost[nxt]:
                cost[nxt] = new_cost
                priority = new_cost + abs(goal[0] - nxt[0]) + abs(goal[1] - nxt[1])
                heapq.heappush(frontier, (priority, nxt))
                came_from[nxt] = current

    if goal not in came_from:
        return []

    path = []
    cur = goal
    while cur is not None:
        path.append(cur)
        cur = came_from[cur]
    path.reverse()
    return path

def world_xy(cell):
    r, c = cell
    half = (GRID - 1) * CELL / 2.0
    return c * CELL - half, half - r * CELL

def wall_segments(walls):
    half = GRID * CELL / 2.0
    segments = []

    # Outer boundary. Leave one opening at the start and one at the goal.
    for i in range(GRID):
        x = i * CELL - half + CELL / 2.0

        # Top boundary: entrance at cell (0, 0)
        if i != 0:
            segments.append((x, half, CELL, WALL, 0.0))

        # Bottom boundary: exit at cell (GRID-1, GRID-1)
        if i != GRID - 1:
            segments.append((x, -half, CELL, WALL, 0.0))

        y = half - i * CELL - CELL / 2.0
        segments.append((-half, y, WALL, CELL, 0.0))
        segments.append((half, y, WALL, CELL, 0.0))

    # Internal walls
    for r in range(GRID):
        for c in range(GRID):
            x, y = world_xy((r, c))
            if c < GRID - 1 and walls[r][c][1]:
                segments.append((x + CELL / 2.0, y, WALL, CELL, 0.0))
            if r < GRID - 1 and walls[r][c][2]:
                segments.append((x, y - CELL / 2.0, CELL, WALL, 0.0))

    return segments
