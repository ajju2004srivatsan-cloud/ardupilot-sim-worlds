"""Webots supervisor controller. Same Avoider as the desktop demo.

Webots only provides the `controller` module while this file is running
inside Webots. From a normal shell, use `python3 -m sim.demo indoor`.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from controller import Supervisor  # noqa: E402

from sim.avoid import Avoider  # noqa: E402
from sim.sensors import scan_angles  # noqa: E402
from sim.worlds import get  # noqa: E402


def main() -> None:
    robot = Supervisor()
    timestep = int(robot.getBasicTimeStep())
    world_name = robot.getWorldPath()
    kind = "outdoor" if "outdoor" in world_name else "indoor"
    layout = get(kind)
    goal = layout.goal

    lidar = robot.getDevice("lidar")
    gps = robot.getDevice("gps")
    compass = robot.getDevice("compass")
    lidar.enable(timestep)
    gps.enable(timestep)
    compass.enable(timestep)

    node = robot.getSelf()
    translation = node.getField("translation")
    rotation = node.getField("rotation")
    plugin = Avoider(goal, layout.width, layout.depth)
    angles = scan_angles(lidar.getHorizontalResolution())
    yaw = 0.0

    while robot.step(timestep) != -1:
        pos = gps.getValues()
        x, z = pos[0], pos[2]
        if math.hypot(goal[0] - x, goal[1] - z) < 0.7:
            break
        ranges = list(lidar.getRangeImage())
        vx, vy = plugin.step(x, z, yaw, angles, ranges, lidar.getMaxRange())
        if abs(vx) + abs(vy) > 1e-6:
            yaw = math.atan2(vy, vx)
        dt = timestep / 1000.0
        translation.setSFVec3f([x + vx * dt, 1.0, z + vy * dt])
        rotation.setSFRotation([0, 1, 0, -yaw])
        # Compass is enabled so a later rate controller can use it.
        compass.getValues()


if __name__ == "__main__":
    main()
