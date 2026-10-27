#!/usr/bin/env python3
import rclpy 
from rclpy.node import Node 
from builtin_interfaces.msg import Duration 
from trajectory_msgs.msg import JointTrajectory , JointTrajectoryPoint
import time
import speech_recognition as sr 
import pyttsx3

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

    def tick(self, pose):
        msg = JointTrajectory(joint_names=JOINTS)
        p = JointTrajectoryPoint(positions=pose, time_from_start=Duration(sec=2))
        msg.points.append(p)
        self.pub.publish(msg)
        self.get_logger().info(f"Positions {pose}")

def main():
    rclpy.init()
    node = Demo()

    recognizer = sr.Recognizer()
    engine = pyttsx3.init()

    try:
        x=True
        # rclpy.spin(node)
        while(x):
            # mic_list = sr.Microphone.list_microphone_names()
            # if len(mic_list) == 0:
            #     print("No microphones found! Exiting.")
            #     exit(1)
            # else:
            #     print("Available microphones:")
            #     for i, name in enumerate(mic_list):
            #         print(f"{i}: {name}")
            with sr.Microphone() as source:
                recognizer.adjust_for_ambient_noise(source, duration=1)
                print("Say a Direction...")
                engine.say("Say a Direction...")
                engine.runAndWait()  
    
                print("Listening now...")
                audio = recognizer.listen(source, timeout=5, phrase_time_limit=3)
                #print("You said: "+recognizer.recognize_google(audio));


            try:
                direction = recognizer.recognize_google(audio)
                print("Moving towards " + direction)
                engine.say("Moving towards " + direction)
                engine.runAndWait()

                arr = []
                match direction.lower() : 
                    case 'left':
                        arr=node.poses[:3]
                    case 'right':
                        arr=node.poses[2:5]
                    case 'up':
                        arr=node.poses[4:7]
                    case 'down':
                        arr=node.poses[6:]
                    case 'exit':
                        print("Exiting Program.")
                        engine.say("Exiting Program.")
                        engine.runAndWait()
                        x=False
                    case _:
                        print("Invalid input.")
                        engine.say("Invalid input.")
                        engine.runAndWait()

            except sr.UnknownValueError as e:
                import traceback
                arr = []
                print("Sorry! I am unable to understand your voice.")
                engine.say("Sorry! I am unable to understand your voice.")
                engine.runAndWait()
                
                # Print full exception with traceback
                print("Full exception details:")
                traceback.print_exc()

            except sr.RequestError:
                arr = []
                print("Could not request results, check internet connection.")
                engine.say("Could not request results, check internet connection.")
                engine.runAndWait()
            
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