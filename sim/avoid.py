"""Obstacle avoidance from a pose and one lidar scan.

This is the plugin. Call `Avoider.step` once per control tick.
Angles are body-frame radians, 0 = forward, positive = left.
Ranges are meters. The step returns world-frame velocity in m/s.

Behavior is the bug algorithm: drive at the goal, and when something
is in front, follow the wall on the left until the goal is clear again.
"""

from __future__ import annotations

import math


def _wrap(angle: float) -> float:
    return (angle + math.pi) % (2 * math.pi) - math.pi


class Avoider:
    def __init__(self, goal: tuple[float, float], width: float = 40.0, depth: float = 40.0):
        self.goal = goal
        self.width = width
        self.depth = depth
        self.following = False
        self.hit_distance = 0.0

    def mark_blocked(self, _x: float, _y: float) -> None:
        self.following = True

    def step(
        self,
        x: float,
        y: float,
        yaw: float,
        angles: list[float],
        ranges: list[float],
        max_range: float = 12.0,
    ) -> tuple[float, float]:
        goal_x, goal_y = self.goal
        goal_distance = math.hypot(goal_x - x, goal_y - y)
        if goal_distance < 0.6:
            return 0.0, 0.0
        goal_heading = math.atan2(goal_y - y, goal_x - x)
        front = self._cone(angles, ranges, 0.0, 0.45, max_range)

        if self.following and self._goal_is_clear(angles, ranges, yaw, goal_heading, goal_distance, max_range):
            self.following = False
        if not self.following and front < 1.1:
            self.following = True
            self.hit_distance = goal_distance

        if self.following:
            turn = self._wall_turn(angles, ranges, max_range)
            heading = yaw + turn
            speed = 0.7 if front < 0.8 else 1.3
        else:
            heading = goal_heading
            speed = 1.5 if front > 2.0 else 1.1
        vx = speed * math.cos(heading)
        vy = speed * math.sin(heading)
        margin = 0.8
        if not margin < x + vx * 0.1 < self.width - margin:
            vx = 0.0
        if not margin < y + vy * 0.1 < self.depth - margin:
            vy = 0.0
        return vx, vy

    def _goal_is_clear(
        self,
        angles: list[float],
        ranges: list[float],
        yaw: float,
        goal_heading: float,
        goal_distance: float,
        max_range: float,
    ) -> bool:
        if goal_distance > self.hit_distance - 0.5:
            return False
        bearing = _wrap(goal_heading - yaw)
        clear = self._cone(angles, ranges, bearing, 0.35, max_range)
        return clear > min(goal_distance, max_range) - 0.4

    def _wall_turn(self, angles: list[float], ranges: list[float], max_range: float) -> float:
        """Positive turn is left. Keep a wall on the left and don't hit what is ahead."""
        front = self._cone(angles, ranges, 0.0, 0.4, max_range)
        left = self._cone(angles, ranges, math.pi / 2, 0.7, max_range)
        if front < 0.9:
            return -0.7
        if left < 0.7:
            return -0.35
        if left > 1.5:
            return 0.4
        return 0.05

    def _cone(
        self,
        angles: list[float],
        ranges: list[float],
        center: float,
        half_width: float,
        max_range: float,
    ) -> float:
        picked = [
            dist
            for angle, dist in zip(angles, ranges)
            if abs(_wrap(angle - center)) <= half_width
        ]
        if not picked:
            return max_range
        return min(picked)
