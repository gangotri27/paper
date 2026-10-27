#!/usr/bin/env python3
"""
Scene Builder for Constrained Benchmark Environment
Creates obstacles that force the robot to use wrist roll
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
        
        # PARAMETER: Corridor width (wall y-position)
        self.declare_parameter('wall_y', 0.12)  # Default: walls at y = ±0.12m
        self.wall_y = self.get_parameter('wall_y').value
        
        # CORRIDOR X POSITION (fixed)
        self.corridor_x = 0.30  # Keep this consistent
        
        # Client for applying planning scene
        self.scene_client = self.create_client(ApplyPlanningScene, '/apply_planning_scene')
        
        self.get_logger().info('Waiting for apply_planning_scene service...')
        while not self.scene_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().info('Still waiting...')
        
        self.get_logger().info('✓ Scene builder connected')
        self.get_logger().info(f'🏗️ Building corridor at x={self.corridor_x}m with walls at y=±{self.wall_y}m (width = {2*self.wall_y:.2f}m)')
    
    def create_box(self, name, dimensions, position):
        """Create a collision box"""
        obj = CollisionObject()
        obj.id = name
        obj.header.frame_id = "world"
        obj.operation = CollisionObject.ADD
        
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = dimensions  # [x, y, z]
        
        pose = Pose()
        pose.position.x = position[0]
        pose.position.y = position[1]
        pose.position.z = position[2]
        pose.orientation.w = 1.0
        
        obj.primitives.append(box)
        obj.primitive_poses.append(pose)
        
        return obj
    
    def build_constrained_scene(self):
        """Build a scene that REQUIRES wrist roll to succeed"""
        scene = PlanningScene()
        scene.is_diff = True
        
        # NO BACK WALL - completely removed
        # This eliminates all back_wall collisions
        
        # Corridor position
        corridor_x = self.corridor_x
        wall_y = self.wall_y
        corridor_width = 2 * wall_y
        
        self.get_logger().info(f'📏 Corridor width: {corridor_width:.2f}m')
        
        # Left wall - narrower in y-direction to create corridor
        scene.world.collision_objects.append(
            self.create_box(
                'left_wall',
                [0.15, 0.14, 0.4],  # [x, y, z]
                [corridor_x, -wall_y, 0.2]
            )
        )
        
        # Right wall
        scene.world.collision_objects.append(
            self.create_box(
                'right_wall',
                [0.15, 0.14, 0.4],
                [corridor_x, wall_y, 0.2]
            )
        )
        
        # Top barrier - forces lower approach
        scene.world.collision_objects.append(
            self.create_box(
                'top_barrier',
                [0.25, 0.3, 0.05],
                [corridor_x + 0.1, 0.0, 0.28]  # Slightly forward of corridor
            )
        )
        
        # Bottom plate - simulates table surface
        scene.world.collision_objects.append(
            self.create_box(
                'bottom_plate',
                [0.5, 0.5, 0.02],
                [corridor_x, 0.0, -0.01]
            )
        )
        
        return scene
    
    def apply_scene(self):
        """Apply the scene to MoveIt"""
        self.get_logger().info('Building constrained benchmark scene...')
        scene = self.build_constrained_scene()
        
        # Apply the scene directly (no need to clear - MoveIt handles diffs)
        request = ApplyPlanningScene.Request()
        request.scene = scene
        
        future = self.scene_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            self.get_logger().info('✓ Scene applied successfully!')
            time.sleep(2.0)  # Give time for scene to propagate
            self.get_logger().info('')
            self.get_logger().info('Scene objects:')
            self.get_logger().info(f'  - Left wall: ({self.corridor_x}, -{self.wall_y}, 0.2) [0.15x0.14x0.4]')
            self.get_logger().info(f'  - Right wall: ({self.corridor_x}, {self.wall_y}, 0.2) [0.15x0.14x0.4]')
            self.get_logger().info('  - NO BACK WALL (removed)')
            self.get_logger().info(f'  - Top barrier: ({self.corridor_x+0.1}, 0.0, 0.28) [0.25x0.3x0.05]')
            self.get_logger().info(f'  - Bottom plate: ({self.corridor_x}, 0.0, -0.01) [0.5x0.5x0.02]')
            self.get_logger().info('')
            self.get_logger().info(f'Target pose: ({self.corridor_x}, 0.0, 0.15) with varying roll')
            return True
        else:
            self.get_logger().error('✗ Failed to apply scene')
            return False
    
    def verify_scene(self):
        """Print verification commands"""
        self.get_logger().info('')
        self.get_logger().info('To verify scene, run:')
        self.get_logger().info('  ros2 topic echo /planning_scene --once | grep -A 10 "collision_objects"')
        self.get_logger().info('')


def main(args=None):
    rclpy.init(args=args)
    
    node = SceneBuilder()
    success = node.apply_scene()
    
    if success:
        time.sleep(2.0)
        node.verify_scene()
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()