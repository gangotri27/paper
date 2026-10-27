import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class SimpleArmPublisher(Node):
    def __init__(self):
        super().__init__('simple_arm_publisher')
        self.publisher_joint1 = self.create_publisher(Float64MultiArray, '/joint1_position_controller/commands', 10)
        self.publisher_joint3 = self.create_publisher(Float64MultiArray, '/joint3_position_controller/commands', 10)
        self.timer = self.create_timer(3.0, self.timer_callback)
        self.direction = 1.0

    def timer_callback(self):
        msg1 = Float64MultiArray()
        msg3 = Float64MultiArray()

        msg1.data = [self.direction * 0.5]
        msg3.data = [self.direction * 0.3]

        self.publisher_joint1.publish(msg1)
        self.publisher_joint3.publish(msg3)

        self.get_logger().info(f"Sent joint1={msg1.data[0]}, joint3={msg3.data[0]}")

        self.direction *= -1.0  # alternate direction

def main(args=None):
    rclpy.init(args=args)
    node = SimpleArmPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
