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
        msg3.data = [self.direction * (-0.3)]

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


































# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Float64

# class BasicArmMotion(Node):
#     def __init__(self):
#         super().__init__('openmanipulator_basic_motion')

#         # Two separate publishers for base and elbow joints
#         self.base_pub = self.create_publisher(Float64, '/joint1_position_controller/commands', 10)
#         self.elbow_pub = self.create_publisher(Float64, '/joint3_position_controller/commands', 10)

#         # Timer → call every 2 seconds
#         self.timer = self.create_timer(2.0, self.move_joints)

#         # Motion states
#         self.base_left = 1.0
#         self.base_right = -1.0
#         self.elbow_up = 1.0
#         self.elbow_down = -1.0
#         self.direction = True   # toggle between directions

#         self.get_logger().info("Basic Arm Motion node started.")

#     def move_joints(self):
#         base_msg = Float64()
#         elbow_msg = Float64()

#         # Alternate directions every timer call
#         if self.direction:
#             base_msg.data = self.base_left
#             elbow_msg.data = self.elbow_up
#             self.get_logger().info("→ Moving LEFT & UP")
#         else:
#             base_msg.data = self.base_right
#             elbow_msg.data = self.elbow_down
#             self.get_logger().info("→ Moving RIGHT & DOWN")

#         # Publish the messages
#         self.base_pub.publish(base_msg)
#         self.elbow_pub.publish(elbow_msg)

#         # Toggle direction for next cycle
#         self.direction = not self.direction

# def main():
#     rclpy.init()
#     node = BasicArmMotion()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         node.get_logger().info("Shutting down.")
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()






""" #!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

TRAJ_TOPIC = "/arm_controller/joint_trajectory"
JOINTS = ["joint1","joint2","joint3","joint4"]

class Demo(Node):
    def __init__(self):
        super().__init__("omx_traj_demo")
        self.pub = self.create_publisher(JointTrajectory, TRAJ_TOPIC, 10)
        self.poses = [
            [1.57,0.0,0.0,-1.0],
            [0.0,0.0,0.0,0.0],
            [-1.57,0.0,0.0,1.0],
        ]
        self.phase = 0
        self.get_logger().info(f"Started. Publishing to {TRAJ_TOPIC} | joints={JOINTS}")
        self.timer = self.create_timer(2.0, self.tick)

    def tick(self):
        msg = JointTrajectory(joint_names=JOINTS)
        p = JointTrajectoryPoint(positions=self.poses[self.phase], time_from_start=Duration(sec=2))
        msg.points.append(p)
        self.pub.publish(msg)
        self.get_logger().info(f"Phase {self.phase} → positions {self.poses[self.phase]}")
        self.phase = (self.phase + 1) % len(self.poses)

def main():
    rclpy.init()
    node = Demo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Ctrl+C — shutting down cleanly.")
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main() """