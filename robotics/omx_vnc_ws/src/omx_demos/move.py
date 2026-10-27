#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from ros_gz_interfaces.srv import SetEntityPose
from ros_gz_interfaces.msg import Entity
from geometry_msgs.msg import Pose as RPose
from geometry_msgs.msg import Point, Quaternion
import time
import math

TRAJ_TOPIC = "/arm_controller/joint_trajectory"
CUBE_NAME = "cube"        # your cube model/entity name in GZ
EE_FRAME  = "ee_link"     # end effector link name

class PickPlaceDemo(Node):
    def __init__(self):
        super().__init__("pick_place_demo")

        # publisher for joint commands
        self.pub = self.create_publisher(JointTrajectory, TRAJ_TOPIC, 10)

        # service client for SetEntityPose
        self.cli = self.create_client(SetEntityPose, "/world/default/set_entity_pose")
        self.cli.wait_for_service()
        self.get_logger().info("SetEntityPose service available.")

        self.attached = False

        # timer to update cube position if attached
        self.timer = self.create_timer(0.05, self.follow_ee)

    # -------------------------------
    # METHOD: send joint trajectory
    # -------------------------------
    def goto(self, joint_positions, duration=2.0):
        msg = JointTrajectory()
        msg.joint_names = ["joint1","joint2","joint3","joint4"]

        pt = JointTrajectoryPoint()
        pt.positions = joint_positions
        pt.time_from_start.sec = int(duration)
        pt.time_from_start.nanosec = int((duration - int(duration))*1e9)

        msg.points.append(pt)
        self.pub.publish(msg)
        time.sleep(duration)

    # -------------------------------
    # METHOD: attach cube (enable follow)
    # -------------------------------
    def attach(self):
        self.attached = True
        self.get_logger().info("CUBE ATTACHED")

    # -------------------------------
    # METHOD: detach cube (stop follow)
    # -------------------------------
    def detach(self):
        self.attached = False
        self.get_logger().info("CUBE DETACHED")

    # -------------------------------
    # METHOD: continuously move cube to EE pose
    # -------------------------------
    def follow_ee(self):
        if not self.attached:
            return
        
        # get transform from map/world → ee_link
        try:
            t = self.get_world_to_ee()
        except:
            return
        
        x, y, z, qx, qy, qz, qw = t

        req = SetEntityPose.Request()
        req.entity = Entity(name=CUBE_NAME)
        req.pose = Pose(
            position=Point(x=x, y=y, z=z),
            orientation=Quaternion(x=qx, y=qy, z=qz, w=qw)
        )
        self.cli.call_async(req)

    # -------------------------------
    # METHOD: TF lookup world → ee_link
    # -------------------------------
    def get_world_to_ee(self):
        # (Fake TF because ros_gz_bridge does not expose TF)
        # So we manually compute offset relative to joint angles.

        # You can later replace this with TF2 if you want.

        # For demo, approximate the EE position:
        joint_state = [0.0, 0.0, 0.0, 0.0]  # replace if you want to read actual joints
        
        x = 0.2
        y = 0.0
        z = 0.12
        
        # Give cube a fixed rotation
        qx, qy, qz, qw = 0, 0, 0, 1
        return (x,y,z,qx,qy,qz,qw)

# -------------------------------
# MAIN DEMO LOGIC
# -------------------------------
def main():
    rclpy.init()
    node = PickPlaceDemo()

    node.get_logger().info("Starting DEMO pick & place...")

    # 1. go home
    node.goto([0.0, 0.0, 0.0, 0.0])

    # 2. move near cube
    node.goto([0.0, -0.5, 0.5, 0.0])

    # 3. ATTACH cube
    node.attach()
    time.sleep(1)

    # 4. move with cube
    node.goto([0.0, 0.7, -1.0, 0.5])

    # 5. move to place position
    node.goto([0.0, -0.3, 0.3, 0.0])

    # 6. DETACH cube
    node.detach()
    time.sleep(1)

    node.get_logger().info("DEMO finished.")
    rclpy.spin(node)

if __name__ == "__main__":
    main()

