#!/usr/bin/env python3
"""
Applies a corridor scene with parts switched on/off, to find WHICH object causes path collisions.
Removes the four benchmark objects first, then adds the ones you ask for. Same sizes/positions as
scene_builder_variable.py (walls 0.15x0.14x0.4 at x=0.30, y=+-wall_y, z=0.2; top barrier 0.25x0.3x0.05 at
(0.40,0,0.28); bottom plate 0.5x0.5x0.02 at (0.30,0,-0.01)).

  python3 scene_builder_options.py --wall-y 0.20 --top-barrier false --bottom-plate true
  python3 scene_builder_options.py --wall-y -1 --top-barrier true  --bottom-plate false   (barrier only)
"""
import argparse, sys, time
import rclpy
from rclpy.node import Node
from moveit_msgs.srv import ApplyPlanningScene
from moveit_msgs.msg import PlanningScene, CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

def b(s): return s.lower() in ("1", "true", "yes")

ap = argparse.ArgumentParser()
ap.add_argument("--wall-y", type=float, default=-1.0)
ap.add_argument("--top-barrier", type=b, default=True)
ap.add_argument("--bottom-plate", type=b, default=True)
a = ap.parse_args()

rclpy.init()
n = Node("scene_builder_options")
cli = n.create_client(ApplyPlanningScene, "/apply_planning_scene")
if not cli.wait_for_service(timeout_sec=15.0):
    sys.exit("/apply_planning_scene not available")

def box(name, dims, pos):
    o = CollisionObject(); o.id = name; o.header.frame_id = "world"; o.operation = CollisionObject.ADD
    p = SolidPrimitive(); p.type = SolidPrimitive.BOX; p.dimensions = dims
    pose = Pose(); pose.position.x, pose.position.y, pose.position.z = pos; pose.orientation.w = 1.0
    o.primitives.append(p); o.primitive_poses.append(pose)
    return o

def apply(objs):
    sc = PlanningScene(); sc.is_diff = True
    sc.world.collision_objects = objs
    req = ApplyPlanningScene.Request(); req.scene = sc
    f = cli.call_async(req); rclpy.spin_until_future_complete(n, f, timeout_sec=10.0)
    return f.result() is not None and f.result().success

rm = []
for name in ("left_wall", "right_wall", "top_barrier", "bottom_plate"):
    o = CollisionObject(); o.id = name; o.header.frame_id = "world"; o.operation = CollisionObject.REMOVE
    rm.append(o)
print("removed old objects:", apply(rm)); time.sleep(1.0)

add = []
if a.wall_y > 0:
    add.append(box("left_wall", [0.15, 0.14, 0.4], [0.30, -a.wall_y, 0.2]))
    add.append(box("right_wall", [0.15, 0.14, 0.4], [0.30, a.wall_y, 0.2]))
if a.top_barrier:
    add.append(box("top_barrier", [0.25, 0.3, 0.05], [0.40, 0.0, 0.28]))
if a.bottom_plate:
    add.append(box("bottom_plate", [0.5, 0.5, 0.02], [0.30, 0.0, -0.01]))
print("added", [o.id for o in add], "->", apply(add) if add else "nothing"); time.sleep(1.5)
n.destroy_node(); rclpy.shutdown()