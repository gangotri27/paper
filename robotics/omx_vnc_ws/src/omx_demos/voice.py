#!/usr/bin/env python3
import os
import time
import json
import queue
import sounddevice as sd
import pyttsx3
import rclpy
from rclpy.node import Node
from vosk import Model, KaldiRecognizer
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

TRAJ_TOPIC = "/arm_controller/joint_trajectory"
JOINTS = ["joint1", "joint2", "joint3", "joint4"]

class Demo(Node):
    def __init__(self):
        super().__init__("omx_traj_demo")
        self.pub = self.create_publisher(JointTrajectory, TRAJ_TOPIC, 10)
        self.poses = {
            "right": [[-1.57, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0]],
            "left": [[1.57, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0]],
            "up": [[0.0, 0.0, 0.0, -1.0], [0.0, 0.0, 0.0, 0.0]],
            "down": [[0.0, 0.0, 0.0, 1.0], [0.0, 0.0, 0.0, 0.0]]
        }
        self.get_logger().info(f"Started. Publishing to {TRAJ_TOPIC} | joints={JOINTS}")

    def tick(self, pose):
        msg = JointTrajectory(joint_names=JOINTS)
        p = JointTrajectoryPoint(positions=pose, time_from_start=Duration(sec=2))
        msg.points.append(p)
        self.pub.publish(msg)
        self.get_logger().info(f"→ positions {pose}")

def listen_and_recognize(recognizer, q, engine):
    print("🎤 Say a command (right, left, up, down, exit)...")
    try:
        with sd.RawInputStream(samplerate=16000, blocksize=8000, dtype='int16',
                               channels=1, callback=lambda indata, frames, time, status: q.put(bytes(indata))):
            while True:
                data = q.get()
                if recognizer.AcceptWaveform(data):
                    result = json.loads(recognizer.Result())
                    if "text" in result and result["text"].strip():
                        return result["text"].strip()
    except Exception as e:
        print(f"⚠️ Audio error: {e}")
        engine.say("Audio error occurred.")
        engine.runAndWait()
        return ""

def main():
    # Initialize ROS and Node
    rclpy.init()
    node = Demo()

    # Initialize TTS engine
    engine = pyttsx3.init()
    engine.say("Voice control activated.")
    engine.runAndWait()

    # Load Vosk model
    if not os.path.exists("model"):
        print("❌ No 'model' folder found. Please download and extract a Vosk model.")
        return
    model = Model("model")
    recognizer = KaldiRecognizer(model, 16000)
    q = queue.Queue()

    try:
        while True:
            command = listen_and_recognize(recognizer, q, engine)
            if not command:
                continue

            print(f"You said: {command}")
            engine.say(f"You said {command}")
            engine.runAndWait()

            # Match commands
            if command in node.poses:
                for pose in node.poses[command]:
                    node.tick(pose)
                    engine.say(f"The arm is moving {command}.")
                    engine.runAndWait()
                    time.sleep(3)
            elif "exit" in command:
                print("Exiting safely.")
                engine.say("Exiting safely.")
                engine.runAndWait()
                break
            else:
                print("Unrecognized command.")
                engine.say("Unrecognized command.")
                engine.runAndWait()

    except KeyboardInterrupt:
        print("Ctrl+C - shutting down cleanly.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
