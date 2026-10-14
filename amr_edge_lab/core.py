"""ROS-independent planning, depth projection and mission-state rules.

The offline A* planner is a teaching baseline. ROS demos use Nav2 instead.
Distances are metres; grid cells are (column, row), with +Y pointing up.
"""

from dataclasses import dataclass
from enum import Enum
import heapq
import math


@dataclass(frozen=True)
class Grid:
    width: int
    height: int
    resolution: float
    occupied: frozenset

    def inside(self, cell):
        return 0 <= cell[0] < self.width and 0 <= cell[1] < self.height

    def cell(self, x, y):
        return math.floor(x / self.resolution), math.floor(y / self.resolution)

    def centre(self, cell):
        return (cell[0] + 0.5) * self.resolution, (cell[1] + 0.5) * self.resolution

    def inflated(self, radius):
        if not math.isfinite(radius) or radius < 0:
            raise ValueError("inflation radius must be finite and nonnegative")
        cells = set(self.occupied)
        reach = math.ceil(radius / self.resolution)
        for x, y in self.occupied:
            for dy in range(-reach, reach + 1):
                for dx in range(-reach, reach + 1):
                    candidate = x + dx, y + dy
                    if self.inside(candidate) and math.hypot(dx, dy) * self.resolution <= radius + 1e-9:
                        cells.add(candidate)
        return Grid(self.width, self.height, self.resolution, frozenset(cells))

    def with_obstacles(self, cells):
        return Grid(self.width, self.height, self.resolution, self.occupied | frozenset(cells))


def warehouse():
    """8 x 6 m synthetic map with a central aisle and rectangular shelves."""
    width, height, resolution = 80, 60, 0.1
    cells = {(x, y) for x in range(width) for y in range(height)
             if x in (0, width - 1) or y in (0, height - 1)}
    for x0, x1, y0, y1 in [(22, 30, 10, 22), (22, 30, 38, 50), (48, 56, 10, 22), (48, 56, 38, 50)]:
        cells.update((x, y) for x in range(x0, x1) for y in range(y0, y1))
    return Grid(width, height, resolution, frozenset(cells))


def astar(grid, start, goal):
    """8-connected A*, Euclidean admissible heuristic, no diagonal corner cutting."""
    if not grid.inside(start) or not grid.inside(goal):
        raise ValueError("start and goal must be in bounds")
    if start in grid.occupied or goal in grid.occupied:
        return []
    queue = [(math.dist(start, goal), 0.0, start)]
    cost, parent = {start: 0.0}, {}
    while queue:
        _, travelled, current = heapq.heappop(queue)
        if travelled > cost[current] + 1e-9:
            continue
        if current == goal:
            route = [goal]
            while route[-1] != start:
                route.append(parent[route[-1]])
            return route[::-1]
        x, y = current
        for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)]:
            nxt = x + dx, y + dy
            if not grid.inside(nxt) or nxt in grid.occupied:
                continue
            if dx and dy and ((x + dx, y) in grid.occupied or (x, y + dy) in grid.occupied):
                continue
            candidate = travelled + math.hypot(dx, dy)
            if candidate + 1e-9 < cost.get(nxt, math.inf):
                cost[nxt], parent[nxt] = candidate, current
                heapq.heappush(queue, (candidate + math.dist(nxt, goal), candidate, nxt))
    return []


def project_depth(depth, width, height, fx, fy, cx, cy, stride=4, near=0.2, far=5.0, camera_height=0.35):
    """32-bit float depth -> base-frame XYZ under a documented fixed extrinsic.

    Optical frame: X right, Y down, Z forward. Camera is mounted level,
    forward-facing at (0, 0, camera_height) in base_link. General mounts need TF.
    """
    if width < 1 or height < 1 or len(depth) != width * height:
        raise ValueError("depth buffer shape mismatch")
    if stride < 1 or not all(math.isfinite(v) for v in (fx, fy, cx, cy, near, far, camera_height)):
        raise ValueError("invalid projection parameters")
    if fx <= 0 or fy <= 0 or near <= 0 or far <= near:
        raise ValueError("invalid intrinsics or range")
    points = []
    for v in range(0, height, stride):
        for u in range(0, width, stride):
            z = depth[v * width + u]
            if math.isfinite(z) and near <= z <= far:
                points.append((z, -(u - cx) * z / fx, camera_height - (v - cy) * z / fy))
    return points


class State(str, Enum):
    IDLE = "IDLE"
    WAITING = "WAITING_FOR_ACCEPTANCE"
    EXECUTING = "EXECUTING"
    CANCELING = "CANCELING"
    SUCCEEDED = "SUCCEEDED"
    FAILED = "FAILED"
    CANCELED = "CANCELED"


class Mission:
    """Single outstanding goal: a cancel request is not a canceled result."""
    def __init__(self):
        self.state = State.IDLE
        self.reason = ""
        self.cancel_requested = False

    @property
    def active(self):
        return self.state in (State.WAITING, State.EXECUTING, State.CANCELING)

    def start(self):
        if self.active:
            raise RuntimeError("a goal is already outstanding")
        self.state, self.reason, self.cancel_requested = State.WAITING, "", False

    def accepted(self, accepted):
        if self.state != State.WAITING:
            raise RuntimeError("unexpected acceptance")
        if not accepted:
            self.state, self.reason = State.FAILED, "goal rejected"
        else:
            self.state = State.CANCELING if self.cancel_requested else State.EXECUTING

    def request_cancel(self):
        if not self.active:
            return False
        self.cancel_requested = True
        if self.state == State.EXECUTING:
            self.state = State.CANCELING
        return True

    def finished(self, outcome):
        if self.state not in (State.EXECUTING, State.CANCELING):
            raise RuntimeError("result without an accepted goal")
        choices = {"succeeded": State.SUCCEEDED, "canceled": State.CANCELED, "failed": State.FAILED}
        if outcome not in choices:
            raise ValueError("unknown outcome")
        self.state = choices[outcome]
        self.reason = outcome


def raycast(grid, x, y, angle, max_range=5.0, extra=()):
    blocked = grid.occupied | frozenset(extra)
    step = grid.resolution * 0.5
    for i in range(1, math.ceil(max_range / step) + 1):
        distance = min(i * step, max_range)
        cell = grid.cell(x + math.cos(angle) * distance, y + math.sin(angle) * distance)
        if not grid.inside(cell) or cell in blocked:
            return distance
    return max_range
