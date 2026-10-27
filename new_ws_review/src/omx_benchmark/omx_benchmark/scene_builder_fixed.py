#!/usr/bin/env python3
"""
Fixed Scene Builder with adjustable corridor width
"""
import rclpy
from rclpy.node import Node
from moveit_msgs.srv import ApplyPlanningScene
from moveit_msgs.msg import PlanningScene, CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
import time

class SceneBuilder(Node):
    def __init__(self):
        super().__init__('scene_builder')
        
        self.declare_parameter('wall_y', 0.12)
        self.wall_y = self.get_parameter('wall_y').value
        self.corridor_x = 0.30
        
        self.scene_client = self.create_client(ApplyPlanningScene, '/apply_planning_scene')
        
        self.get_logger().info('Waiting for apply_planning_scene service...')
        while not self.scene_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().info('Still waiting...')
        
        self.get_logger().info(f'🏗️ Building corridor at x={self.corridor_x}m with walls at y=±{self.wall_y}m')
    
    def create_box(self, name, dimensions, position):
        obj = CollisionObject()
        obj.id = name
        obj.header.frame_id = "world"
        obj.operation = CollisionObject.ADD
        
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = dimensions
        
        pose = Pose()
        pose.position.x = position[0]
        pose.position.y = position[1]
        pose.position.z = position[2]
        pose.orientation.w = 1.0
        
        obj.primitives.append(box)
        obj.primitive_poses.append(pose)
        return obj
    
    def build_scene(self):
        scene = PlanningScene()
        scene.is_diff = True
        
        # NO BACK WALL
        scene.world.collision_objects.append(
            self.create_box('left_wall', [0.15, 0.14, 0.4], [self.corridor_x, -self.wall_y, 0.2])
        )
        scene.world.collision_objects.append(
            self.create_box('right_wall', [0.15, 0.14, 0.4], [self.corridor_x, self.wall_y, 0.2])
        )
        scene.world.collision_objects.append(
            self.create_box('top_barrier', [0.25, 0.3, 0.05], [self.corridor_x + 0.1, 0.0, 0.28])
        )
        scene.world.collision_objects.append(
            self.create_box('bottom_plate', [0.5, 0.5, 0.02], [self.corridor_x, 0.0, -0.01])
        )
        
        return scene
    
    def apply_scene(self):
        scene = self.build_scene()
        request = ApplyPlanningScene.Request()
        request.scene = scene
        
        future = self.scene_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            self.get_logger().info('✓ Scene applied successfully!')
            return True
        return False

def main(args=None):
    rclpy.init(args=args)
    node = SceneBuilder()
    node.apply_scene()
    time.sleep(2)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()