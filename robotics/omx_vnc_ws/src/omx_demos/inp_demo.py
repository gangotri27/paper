#!/usr/bin/env python3
import rclpy
import time 
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
            [0.0,0.0,0.0,0.0],
            [1.57,0.0,0.0,0.0],
            [0.0,0.0,0.0,0.0],
            [-1.57,0.0,0.0,0.0],
            [0.0,0.0,0.0,0.0],
            [0.0,0.0,0.0,-1.0],
            [0.0,0.0,0.0,0.0],
            [0.0,0.0,0.0,1.0],
            [0.0,0.0,0.0,0.0],
        ]
        # self.phase = 0
        # self.get_logger().info(f"Started. Publishing to {TRAJ_TOPIC} | joints={JOINTS}")
        # self.timer = self.create_timer(3.0, self.tick)

    def tick(self, pose):
        msg = JointTrajectory(joint_names=JOINTS)
        #p = JointTrajectoryPoint(positions=self.poses[self.phase], time_from_start=Duration(sec=2))
        p = JointTrajectoryPoint(positions=pose, time_from_start=Duration(sec=2))
        msg.points.append(p)
        self.pub.publish(msg)
        self.get_logger().info(f"Positions {pose}")
        # self.phase = (self.phase + 1) % len(self.poses)

def main():
    rclpy.init()
    node = Demo()
    try:
        x=True
        # rclpy.spin(node)
        while(x):
            posi = input("Enter direction to move (left , right , up , down) or (exit) : ").lower()
            arr = []
            match posi : 
                case 'left':
                    arr=node.poses[:3]
                case 'right':
                    arr=node.poses[2:5]
                case 'up':
                    arr=node.poses[4:7]
                case 'down':
                    arr=node.poses[6:]
                case 'exit':
                    x=False
                case _:
                    print("Invalid input")
            if(arr):
                for pose in (arr):
                    node.tick(pose)
                    time.sleep(3)


            
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
    main()
