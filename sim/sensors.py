"""2D lidar against the same boxes and trees used in the 3D worlds."""

from __future__ import annotations

import math

from sim.worlds import Box, Tree, World


def scan_angles(count: int = 90) -> list[float]:
    """Body-frame angles, 0 is forward, from -pi to pi."""
    return [-math.pi + (i + 0.5) * (2 * math.pi) / count for i in range(count)]


def _ray_box(ox: float, oy: float, dx: float, dy: float, box: Box, max_range: float) -> float | None:
    hx, hy = box.sx * 0.5, box.sy * 0.5
    min_x, max_x = box.cx - hx, box.cx + hx
    min_y, max_y = box.cy - hy, box.cy + hy
    t_min, t_max = 0.0, max_range
    for origin, direction, lo, hi in ((ox, dx, min_x, max_x), (oy, dy, min_y, max_y)):
        if abs(direction) < 1e-9:
            if origin < lo or origin > hi:
                return None
            continue
        inv = 1.0 / direction
        t1 = (lo - origin) * inv
        t2 = (hi - origin) * inv
        if t1 > t2:
            t1, t2 = t2, t1
        t_min = max(t_min, t1)
        t_max = min(t_max, t2)
        if t_min > t_max:
            return None
    if t_min <= 0.05:
        return None
    return t_min


def _ray_circle(ox: float, oy: float, dx: float, dy: float, tree: Tree, max_range: float) -> float | None:
    fx, fy = ox - tree.x, oy - tree.y
    b = 2 * (fx * dx + fy * dy)
    c = fx * fx + fy * fy - tree.radius * tree.radius
    disc = b * b - 4 * c
    if disc < 0:
        return None
    root = math.sqrt(disc)
    t = (-b - root) * 0.5
    if t <= 0.05:
        t = (-b + root) * 0.5
    if t <= 0.05 or t > max_range:
        return None
    return t


def lidar(world: World, x: float, y: float, yaw: float, max_range: float = 12.0, count: int = 90) -> list[float]:
    ranges = []
    for angle in scan_angles(count):
        heading = yaw + angle
        dx, dy = math.cos(heading), math.sin(heading)
        hit = max_range
        for box in world.boxes:
            dist = _ray_box(x, y, dx, dy, box, max_range)
            if dist is not None and dist < hit:
                hit = dist
        for tree in world.trees:
            dist = _ray_circle(x, y, dx, dy, tree, max_range)
            if dist is not None and dist < hit:
                hit = dist
        ranges.append(hit)
    return ranges


def hits_obstacle(world: World, x: float, y: float, radius: float) -> bool:
    if x < radius or y < radius or x > world.width - radius or y > world.depth - radius:
        return True
    for box in world.boxes:
        nearest_x = min(max(x, box.cx - box.sx * 0.5), box.cx + box.sx * 0.5)
        nearest_y = min(max(y, box.cy - box.sy * 0.5), box.cy + box.sy * 0.5)
        if (x - nearest_x) ** 2 + (y - nearest_y) ** 2 < radius * radius:
            return True
    for tree in world.trees:
        if (x - tree.x) ** 2 + (y - tree.y) ** 2 < (tree.radius + radius) ** 2:
            return True
    return False
