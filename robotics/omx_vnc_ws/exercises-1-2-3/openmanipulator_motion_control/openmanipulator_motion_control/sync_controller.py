import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import math

class SyncJointController(Node):
    def __init__(self):
        super().__init__('sync_joint_controller')
        
        # Publisher to the Group Controller (Controls both joints at once)
        self.publisher_ = self.create_publisher(
            Float64MultiArray, 
            '/custom_group_controller/commands', 
            10
        )
        
        # Timer: Run at 50Hz (Real-time-ish control loop)
        self.timer = self.create_timer(0.02, self.timer_callback)
        
        self.time_counter = 0.0
        self.get_logger().info("Custom Simultaneous Controller Started")

    def timer_callback(self):
        self.time_counter += 0.02
        
        # --- THE MATH (Logic that was in your C++ plugin) ---
        # Joint 1: Sine wave (Left/Right)
        joint1_val = 0.5 * math.sin(self.time_counter)
        
        # Joint 3: Cosine wave (Up/Down) - Simultaneous pattern
        joint3_val = 0.3 * math.cos(self.time_counter)
        
        # Create one message for the group
        msg = Float64MultiArray()
        msg.data = [joint1_val, joint3_val]
        
        self.publisher_.publish(msg)
        
        # Log occasionally
        if self.time_counter % 1.0 < 0.02:
            self.get_logger().info(f"Publishing Group Command: J1={joint1_val:.2f}, J3={joint3_val:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = SyncJointController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()