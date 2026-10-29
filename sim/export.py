"""Write Gazebo SDF and Webots WBT files from the shared world geometry."""

from __future__ import annotations

from pathlib import Path

from sim.worlds import Box, Tree, World, indoor, outdoor


ROOT = Path(__file__).resolve().parents[1]


def _fmt(value: float) -> str:
    return f"{value:.3f}"


def _box_sdf(index: int, box: Box, color: str) -> str:
    return f"""
    <model name="wall_{index}">
      <static>true</static>
      <pose>{_fmt(box.cx)} {_fmt(box.cy)} {_fmt(box.height / 2)} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>{_fmt(box.sx)} {_fmt(box.sy)} {_fmt(box.height)}</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>{_fmt(box.sx)} {_fmt(box.sy)} {_fmt(box.height)}</size></box></geometry>
          <material><ambient>{color}</ambient><diffuse>{color}</diffuse></material>
        </visual>
      </link>
    </model>"""


def _tree_sdf(index: int, tree: Tree) -> str:
    canopy = tree.radius
    return f"""
    <model name="tree_{index}">
      <static>true</static>
      <pose>{_fmt(tree.x)} {_fmt(tree.y)} 0 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <pose>0 0 2.0 0 0 0</pose>
          <geometry><cylinder><radius>{_fmt(canopy)}</radius><length>4.0</length></cylinder></geometry>
        </collision>
        <visual name="trunk">
          <pose>0 0 1.1 0 0 0</pose>
          <geometry><cylinder><radius>0.12</radius><length>2.2</length></cylinder></geometry>
          <material><ambient>0.40 0.26 0.13 1</ambient><diffuse>0.40 0.26 0.13 1</diffuse></material>
        </visual>
        <visual name="canopy">
          <pose>0 0 2.6 0 0 0</pose>
          <geometry><sphere><radius>{_fmt(canopy)}</radius></sphere></geometry>
          <material><ambient>0.18 0.45 0.22 1</ambient><diffuse>0.18 0.45 0.22 1</diffuse></material>
        </visual>
      </link>
    </model>"""


def _quad_sdf(world: World) -> str:
    x, y = world.start
    return f"""
    <model name="scout">
      <static>true</static>
      <pose>{_fmt(x)} {_fmt(y)} 1.0 0 0 0</pose>
      <link name="base">
        <visual name="body">
          <geometry><box><size>0.36 0.36 0.10</size></box></geometry>
          <material><ambient>0.12 0.28 0.55 1</ambient><diffuse>0.12 0.28 0.55 1</diffuse></material>
        </visual>
        <sensor name="lidar" type="gpu_lidar">
          <pose>0 0 0.08 0 0 0</pose>
          <always_on>1</always_on>
          <visualize>true</visualize>
          <update_rate>10</update_rate>
          <topic>lidar</topic>
          <lidar>
            <scan>
              <horizontal>
                <samples>90</samples>
                <resolution>1</resolution>
                <min_angle>-3.14159</min_angle>
                <max_angle>3.14159</max_angle>
              </horizontal>
              <vertical>
                <samples>1</samples>
                <min_angle>0</min_angle>
                <max_angle>0</max_angle>
              </vertical>
            </scan>
            <range><min>0.15</min><max>12</max><resolution>0.02</resolution></range>
          </lidar>
        </sensor>
        <sensor name="depth" type="rgbd_camera">
          <pose>0.18 0 0.02 0 0 0</pose>
          <always_on>1</always_on>
          <visualize>true</visualize>
          <update_rate>10</update_rate>
          <topic>depth</topic>
          <camera>
            <horizontal_fov>1.047</horizontal_fov>
            <image><width>320</width><height>240</height><format>R8G8B8</format></image>
            <clip><near>0.15</near><far>12</far></clip>
          </camera>
        </sensor>
      </link>
    </model>"""


def to_sdf(world: World) -> str:
    floor = "0.78 0.80 0.84 1" if world.kind == "indoor" else "0.55 0.66 0.34 1"
    wall = "0.93 0.94 0.95 1"
    sun = "0.15 0.15 0.16 1" if world.kind == "indoor" else "0.85 0.85 0.8 1"
    ambient = "0.55 0.55 0.58 1" if world.kind == "indoor" else "0.45 0.5 0.45 1"
    pieces = [
        _box_sdf(i, box, wall) for i, box in enumerate(world.boxes)
    ] + [
        _tree_sdf(i, tree) for i, tree in enumerate(world.trees)
    ]
    gx, gy = world.goal
    body = "\n".join(pieces)
    return f"""<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="{world.name}">
    <physics name="1ms" type="ignored">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <scene>
      <ambient>{ambient}</ambient>
      <background>{sun}</background>
    </scene>
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.85 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.4 0.2 -0.9</direction>
    </light>
    <model name="ground">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>{_fmt(world.width)} {_fmt(world.depth)}</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>{_fmt(world.width)} {_fmt(world.depth)}</size></plane></geometry>
          <material><ambient>{floor}</ambient><diffuse>{floor}</diffuse></material>
        </visual>
      </link>
    </model>
    <model name="goal">
      <static>true</static>
      <pose>{_fmt(gx)} {_fmt(gy)} 0.03 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry><cylinder><radius>0.35</radius><length>0.06</length></cylinder></geometry>
          <material><ambient>0.15 0.65 0.25 1</ambient><diffuse>0.15 0.65 0.25 1</diffuse></material>
        </visual>
      </link>
    </model>
    {body}
    {_quad_sdf(world)}
  </world>
</sdf>
"""


def _solid_box(name: str, box: Box) -> str:
    return f"""
Solid {{
  translation {_fmt(box.cx)} {_fmt(box.height / 2)} {_fmt(box.cy)}
  children [
    Shape {{
      appearance Appearance {{
        material Material {{ diffuseColor 0.93 0.94 0.95 }}
      }}
      geometry Box {{ size {_fmt(box.sx)} {_fmt(box.height)} {_fmt(box.sy)} }}
    }}
  ]
  name "{name}"
  boundingObject Box {{ size {_fmt(box.sx)} {_fmt(box.height)} {_fmt(box.sy)} }}
}}"""


def _solid_tree(name: str, tree: Tree) -> str:
    return f"""
Solid {{
  translation {_fmt(tree.x)} 0 {_fmt(tree.y)}
  children [
    Shape {{
      appearance Appearance {{
        material Material {{ diffuseColor 0.40 0.26 0.13 }}
      }}
      geometry Cylinder {{ height 2.2 radius 0.12 }}
    }}
    Transform {{
      translation 0 2.6 0
      children [
        Shape {{
          appearance Appearance {{
            material Material {{ diffuseColor 0.18 0.45 0.22 }}
          }}
          geometry Sphere {{ radius {_fmt(tree.radius)} }}
        }}
      ]
    }}
  ]
  name "{name}"
  boundingObject Cylinder {{ height 4 radius {_fmt(tree.radius)} }}
}}"""


def _floor_color(world: World) -> str:
    return "0.78 0.80 0.84" if world.kind == "indoor" else "0.55 0.66 0.34"


def to_wbt(world: World) -> str:
    solids = "\n".join(_solid_box(f"wall_{i}", box) for i, box in enumerate(world.boxes))
    solids += "\n".join(_solid_tree(f"tree_{i}", tree) for i, tree in enumerate(world.trees))
    sx, sy = world.start
    gx, gy = world.goal
    title = "Indoor maze" if world.kind == "indoor" else "Outdoor forest"
    return f"""#VRML_V7.0.0 utf8
WorldInfo {{
  title "{title}"
  info [
    "Same layout as the Python avoidance demo."
  ]
  basicTimeStep 32
}}
Viewpoint {{
  orientation 1 0 0 -1.05
  position {_fmt(world.width / 2)} { _fmt(max(world.width, world.depth) * 0.9) } {_fmt(world.depth / 2)}
}}
TexturedBackground {{
}}
TexturedBackgroundLight {{
}}
Solid {{
  translation {_fmt(world.width / 2)} 0 {_fmt(world.depth / 2)}
  children [
    Shape {{
      appearance Appearance {{
        material Material {{ diffuseColor {_floor_color(world)} }}
      }}
      geometry Box {{ size {_fmt(world.width)} 0.04 {_fmt(world.depth)} }}
    }}
  ]
  name "floor"
  boundingObject Box {{ size {_fmt(world.width)} 0.04 {_fmt(world.depth)} }}
}}
DEF GOAL Solid {{
  translation {_fmt(gx)} 0.04 {_fmt(gy)}
  children [
    Shape {{
      appearance Appearance {{
        material Material {{ diffuseColor 0.15 0.65 0.25 }}
      }}
      geometry Cylinder {{ height 0.06 radius 0.35 }}
    }}
  ]
  name "goal"
}}
{solids}
DEF SCOUT Robot {{
  translation {_fmt(sx)} 1.0 {_fmt(sy)}
  children [
    Shape {{
      appearance Appearance {{
        material Material {{ diffuseColor 0.12 0.28 0.55 }}
      }}
      geometry Box {{ size 0.36 0.10 0.36 }}
    }}
    Lidar {{
      translation 0 0.08 0
      name "lidar"
      horizontalResolution 90
      fieldOfView 6.28318
      numberOfLayers 1
      minRange 0.15
      maxRange 12
    }}
    RangeFinder {{
      translation 0.18 0.02 0
      rotation 0 1 0 1.5708
      name "depth"
      fieldOfView 1.047
      width 160
      height 120
      minRange 0.15
      maxRange 12
    }}
    GPS {{
      name "gps"
    }}
    Compass {{
      name "compass"
    }}
  ]
  name "scout"
  controller "avoid_controller"
  supervisor TRUE
}}
"""


def main() -> None:
    gazebo = ROOT / "gazebo"
    webots = ROOT / "worlds"
    gazebo.mkdir(parents=True, exist_ok=True)
    webots.mkdir(parents=True, exist_ok=True)
    for world in (indoor(), outdoor()):
        (gazebo / f"{world.name}.sdf").write_text(to_sdf(world), encoding="utf-8")
        (webots / f"{world.name}.wbt").write_text(to_wbt(world), encoding="utf-8")
        print(f"wrote {world.name}")


if __name__ == "__main__":
    main()
