#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import sqlite3
import os
import time
from moveit_msgs.srv import GetPlanningScene, SetPlanningSceneDiffs
from moveit_msgs.msg import PlanningScene, PlanningSceneComponents
from geometry_msgs.msg import Pose, Point, Quaternion
from shape_msgs.msg import SolidPrimitive, Plane, Mesh
import json

class WarehouseManager(Node):
    def __init__(self):
        super().__init__('warehouse_manager')
        
        # Services
        self.get_scene_client = self.create_client(GetPlanningScene, '/get_planning_scene')
        self.set_scene_client = self.create_client(SetPlanningSceneDiffs, '/apply_planning_scene_diffs')
        
        # Wait for services
        self.get_scene_client.wait_for_service(timeout_sec=5.0)
        self.set_scene_client.wait_for_service(timeout_sec=5.0)
        
        self.db_path = None
        
    def verify_sqlite_connection(self, db_path):
        """Verify SQLite database connection"""
        self.get_logger().info(f'Verifying SQLite connection at: {db_path}')
        
        try:
            # Check if file exists
            if os.path.exists(db_path):
                size = os.path.getsize(db_path)
                self.get_logger().info(f'Database file exists. Size: {size} bytes')
            else:
                self.get_logger().warn('Database file does not exist yet')
                return False
            
            # Connect and check tables
            conn = sqlite3.connect(db_path)
            cursor = conn.cursor()
            
            # List all tables
            cursor.execute("SELECT name FROM sqlite_master WHERE type='table';")
            tables = cursor.fetchall()
            
            self.get_logger().info(f'Tables in database: {tables}')
            
            # Check for MoveIt tables
            expected_tables = ['planning_scenes', 'robot_states', 'constraints', 'warehouse_metadata']
            found_tables = [t[0] for t in tables]
            
            for table in expected_tables:
                if table in found_tables:
                    # Count rows
                    cursor.execute(f"SELECT COUNT(*) FROM {table}")
                    count = cursor.fetchone()[0]
                    self.get_logger().info(f'Table {table}: {count} rows')
            
            conn.close()
            return True
            
        except Exception as e:
            self.get_logger().error(f'SQLite verification failed: {e}')
            return False
    
    def save_current_scene(self, scene_name):
        """Save current planning scene to warehouse"""
        self.get_logger().info(f'Saving scene: {scene_name}')
        
        # Get current scene
        req = GetPlanningScene.Request()
        req.components.components = PlanningSceneComponents.WORLD_OBJECT_NAMES | \
                                    PlanningSceneComponents.WORLD_OBJECT_GEOMETRY | \
                                    PlanningSceneComponents.ROBOT_STATE
        
        future = self.get_scene_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            scene = future.result().scene
            
            # Here you would save to SQLite directly
            # For now, we'll just verify we can get the scene
            self.get_logger().info(f'Scene retrieved. World objects: {len(scene.world.collision_objects)}')
            return True
        else:
            self.get_logger().error('Failed to get planning scene')
            return False
    
    def load_scene(self, scene_name):
        """Load scene from warehouse"""
        self.get_logger().info(f'Loading scene: {scene_name}')
        # Implementation would load from SQLite and apply via SetPlanningSceneDiffs
        return True

def main(args=None):
    rclpy.init(args=args)
    node = WarehouseManager()
    
    # Test SQLite connection
    node.verify_sqlite_connection('/tmp/omx_6dof_warehouse.db')
    
    rclpy.spin_once(node, timeout_sec=2.0)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()