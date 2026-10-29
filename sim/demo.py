"""Fly the avoidance plugin through the indoor maze or the outdoor forest."""

from __future__ import annotations

import argparse
import math
from pathlib import Path

from sim.avoid import Avoider
from sim.sensors import hits_obstacle, lidar, scan_angles
from sim.worlds import World, get


def run(world: World, max_seconds: float = 90.0) -> dict:
    x, y = world.start
    yaw = math.atan2(world.goal[1] - y, world.goal[0] - x)
    plugin = Avoider(world.goal, world.width, world.depth)
    angles = scan_angles()
    dt = 0.1
    radius = 0.35
    path = [(x, y)]
    closest = 99.0
    ticks = int(max_seconds / dt)
    for _ in range(ticks):
        if math.hypot(world.goal[0] - x, world.goal[1] - y) < 0.7:
            break
        ranges = lidar(world, x, y, yaw)
        closest = min(closest, min(ranges))
        vx, vy = plugin.step(x, y, yaw, angles, ranges)
        if abs(vx) + abs(vy) > 1e-6:
            yaw = math.atan2(vy, vx)
        nx, ny = x + vx * dt, y + vy * dt
        if hits_obstacle(world, nx, ny, radius):
            plugin.mark_blocked(nx, ny)
            if not hits_obstacle(world, nx, y, radius):
                ny = y
            elif not hits_obstacle(world, x, ny, radius):
                nx = x
            else:
                nx, ny = x, y
        x, y = nx, ny
        path.append((x, y))
    reached = math.hypot(world.goal[0] - x, world.goal[1] - y) < 0.7
    length = sum(math.hypot(path[i + 1][0] - path[i][0], path[i + 1][1] - path[i][1]) for i in range(len(path) - 1))
    return {
        "reached": reached,
        "seconds": round((len(path) - 1) * dt, 1),
        "length": round(length, 1),
        "closest": round(closest, 2),
        "path": path,
        "world": world,
    }


def write_svg(result: dict, path: Path) -> None:
    world: World = result["world"]
    scale = 28
    pad = 16
    width = int(world.width * scale + pad * 2)
    height = int(world.depth * scale + pad * 2)

    def xy(x: float, y: float) -> tuple[float, float]:
        return pad + x * scale, height - (pad + y * scale)

    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="#f4f1ea"/>',
    ]
    floor = "#d9dde3" if world.kind == "indoor" else "#c5d6a8"
    x0, y0 = xy(0, 0)
    x1, y1 = xy(world.width, world.depth)
    parts.append(
        f'<rect x="{min(x0, x1):.1f}" y="{min(y0, y1):.1f}" width="{world.width * scale:.1f}" height="{world.depth * scale:.1f}" fill="{floor}"/>'
    )
    for box in world.boxes:
        cx, cy = xy(box.cx, box.cy)
        parts.append(
            f'<rect x="{cx - box.sx * scale / 2:.1f}" y="{cy - box.sy * scale / 2:.1f}" width="{box.sx * scale:.1f}" height="{box.sy * scale:.1f}" fill="#eceff3" stroke="#8b93a0" stroke-width="1"/>'
        )
    for tree in world.trees:
        cx, cy = xy(tree.x, tree.y)
        parts.append(f'<circle cx="{cx:.1f}" cy="{cy:.1f}" r="{tree.radius * scale:.1f}" fill="#2f6b3a"/>')
    points = " ".join(f"{xy(px, py)[0]:.1f},{xy(px, py)[1]:.1f}" for px, py in result["path"])
    parts.append(f'<polyline points="{points}" fill="none" stroke="#1d4e89" stroke-width="2.5"/>')
    for color, point in (("#c4492c", world.start), ("#2e7d32", world.goal)):
        cx, cy = xy(*point)
        parts.append(f'<circle cx="{cx:.1f}" cy="{cy:.1f}" r="5" fill="{color}"/>')
    parts.append("</svg>")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(parts) + "\n", encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description="Run the obstacle-avoidance plugin on a built-in world.")
    parser.add_argument("world", nargs="?", choices=("indoor", "outdoor"), help="which world to fly")
    parser.add_argument("--check", action="store_true", help="run both worlds and exit non-zero if either misses the goal")
    parser.add_argument("--svg", type=Path, help="where to write a top-down picture of the path")
    args = parser.parse_args()

    names = ("indoor", "outdoor") if args.check or args.world is None else (args.world,)
    failed = False
    for name in names:
        result = run(get(name))
        status = "reached goal" if result["reached"] else "did not reach goal"
        print(
            f"{name}: {status} in {result['seconds']} s, path {result['length']} m, closest return {result['closest']} m"
        )
        if args.svg and len(names) == 1:
            write_svg(result, args.svg)
        elif args.check or args.world is None:
            write_svg(result, Path("media") / f"{name}.svg")
        failed = failed or not result["reached"]
    if failed:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
