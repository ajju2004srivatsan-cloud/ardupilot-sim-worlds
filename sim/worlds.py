"""Indoor maze and outdoor forest. Same layout for the Python demo, Gazebo, and Webots."""

from __future__ import annotations

import random
from dataclasses import dataclass


@dataclass(frozen=True)
class Box:
    cx: float
    cy: float
    sx: float
    sy: float
    height: float = 2.4


@dataclass(frozen=True)
class Tree:
    x: float
    y: float
    radius: float


@dataclass(frozen=True)
class World:
    name: str
    kind: str
    width: float
    depth: float
    start: tuple[float, float]
    goal: tuple[float, float]
    boxes: tuple[Box, ...]
    trees: tuple[Tree, ...]


def indoor() -> World:
    """Two baffles. The east gap is on the lower wall, the west gap is on the upper wall."""
    width, depth = 18.0, 14.0
    thick = 0.3
    height = 2.4
    boxes = (
        Box(width / 2, thick / 2, width, thick, height),
        Box(width / 2, depth - thick / 2, width, thick, height),
        Box(thick / 2, depth / 2, thick, depth, height),
        Box(width - thick / 2, depth / 2, thick, depth, height),
        Box(7.0, 5.0, 13.4, thick, height),
        Box(11.2, 9.0, 13.0, thick, height),
    )
    return World(
        name="indoor_maze",
        kind="indoor",
        width=width,
        depth=depth,
        start=(2.0, 2.0),
        goal=(15.0, 12.0),
        boxes=boxes,
        trees=(),
    )


def outdoor() -> World:
    rng = random.Random(11)
    trees: list[Tree] = []
    start = (4.0, 4.0)
    goal = (36.0, 36.0)
    while len(trees) < 26:
        x = rng.uniform(2.5, 37.5)
        y = rng.uniform(2.5, 37.5)
        radius = rng.choice((0.55, 0.7, 0.85))
        if (x - start[0]) ** 2 + (y - start[1]) ** 2 < 16:
            continue
        if (x - goal[0]) ** 2 + (y - goal[1]) ** 2 < 16:
            continue
        if any((x - t.x) ** 2 + (y - t.y) ** 2 < (t.radius + radius + 2.4) ** 2 for t in trees):
            continue
        trees.append(Tree(x, y, radius))
    return World(
        name="outdoor_forest",
        kind="outdoor",
        width=40.0,
        depth=40.0,
        start=start,
        goal=goal,
        boxes=(),
        trees=tuple(trees),
    )


def get(name: str) -> World:
    worlds = {"indoor": indoor, "outdoor": outdoor, "indoor_maze": indoor, "outdoor_forest": outdoor}
    try:
        return worlds[name]()
    except KeyError as exc:
        known = "indoor, outdoor"
        raise SystemExit(f"Unknown world {name!r}. Use {known}.") from exc
