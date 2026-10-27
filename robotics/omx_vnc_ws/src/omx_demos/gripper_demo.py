#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient
import time

class PickPlace(Node):
    def __init__(self):
        super().__init__('pick_place_node')

        # Arm publisher
        self.arm_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)

        # Gripper action client
        self.gripper_client = ActionClient(self, GripperCommand, '/gripper_controller/gripper_cmd')
        while not self.gripper_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Waiting for gripper action server...')

        # Define joint names
        self.joint_names = ["joint1", "joint2", "joint3", "joint4"]

    def send_joint_positions(self, positions, duration=2.0):
        traj = JointTrajectory()
        traj.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = int(duration)
        traj.points.append(point)
        self.arm_pub.publish(traj)
        self.get_logger().info(f'Moving arm to {positions}')
        time.sleep(duration + 0.5)  # wait for motion to complete

    def move_gripper(self, position):
        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = 1.0
        future = self.gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Gripper goal rejected!')
            return
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        self.get_logger().info(f'Gripper moved to {position}')

def main(args=None):
    rclpy.init(args=args)
    node = PickPlace()

    # Step 1: Move to home
    node.send_joint_positions([0.0, 0.0, 0.0, 0.0])

    # Step 2: Move above cube (pre-pick)
    node.send_joint_positions([0.0, 0.8, -0.6, 0.0], duration=3.0)

    # Step 3: Lower arm to cube
    node.send_joint_positions([0.0, 1.0, -0.7, 0.0], duration=3.0)

    # Step 4: Close gripper to pick cube
    node.move_gripper(0.0)  # adjust position to grip

    # Step 5: Lift cube
    node.send_joint_positions([0.0, 0.7, -0.5, 0.0], duration=3.0)

    # Step 6: Move to place position
    node.send_joint_positions([0.4, 0.6, -0.5, 0.2], duration=3.0)

    # Step 7: Open gripper to release cube
    node.move_gripper(0.015)  # adjust position to release

    # Step 8: Return home
    node.send_joint_positions([0.0, 0.0, 0.0, 0.0])

    node.get_logger().info("Pick and place completed.")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



