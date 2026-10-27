#!/usr/bin/env python3
import os
import json
import queue
import threading
import sounddevice as sd
import pyttsx3
import rclpy
from rclpy.node import Node
from vosk import Model, KaldiRecognizer
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

TRAJ_TOPIC = "/arm_controller/joint_trajectory"
JOINTS = ["joint1", "joint2", "joint3", "joint4"]

class Demo(Node):
    def __init__(self):
        super().__init__("omx_traj_demo")
        self.pub = self.create_publisher(JointTrajectory, TRAJ_TOPIC, 10)
        
        # FIXED POSES: Single flat list of floats for each command
        self.poses = {
            "right": [-1.57, 0.0, 0.0, 0.0],
            "left":  [1.57, 0.0, 0.0, 0.0],
            "up":    [0.0, 0.0, 0.0, -1.0],
            "down":  [0.0, 0.0, 0.0, 1.0]
        }
        self.get_logger().info(f"Started. Publishing to {TRAJ_TOPIC} | joints={JOINTS}")

    def tick(self, pose):
        msg = JointTrajectory(joint_names=JOINTS)
        # Now 'pose' is just [-1.57, 0.0, 0.0, 0.0] which is valid
        p = JointTrajectoryPoint(positions=pose, time_from_start=Duration(sec=2))
        msg.points.append(p)
        self.pub.publish(msg)
        self.get_logger().info(f"→ positions {pose}")

def audio_callback(indata, frames, time, status, q):
    if status:
        print(status, flush=True)
    q.put(bytes(indata))

# --- NEW FUNCTION TO FIND PULSE AUDIO ---
def get_pulse_device_id():
    """Searches available devices for the PulseAudio bridge"""
    print("🔍 Scanning audio devices...")
    device_list = sd.query_devices()
    pulse_id = None
    
    for idx, device in enumerate(device_list):
        # Look for a device that has 'pulse' in the name and is an Input device (>0 channels)
        if 'pulse' in device['name'].lower() and device['max_input_channels'] > 0:
            print(f"   ✅ Found PulseAudio at Index {idx}: {device['name']}")
            pulse_id = idx
            break
            
    if pulse_id is None:
        print("   ⚠️ No specific 'pulse' device found. Using default (might crash in WSL).")
        # Print all devices to help debug
        print(device_list)
        return None # Fallback to default
    return pulse_id

def main():
    rclpy.init()
    node = Demo()
    
    engine = pyttsx3.init()
    
    if not os.path.exists("model"):
        print("Please download the model!")
        return
    model = Model("model")
    recognizer = KaldiRecognizer(model, 16000)
    
    q = queue.Queue()

    # --- UPDATED SETUP ---
    device_id = get_pulse_device_id()
    
    print(f"🎤 Opening Microphone (Device Index: {device_id})...")
    print("Commands: Right, Left, Up, Down, STOP")
    
    try:
        # We explicitly pass `device=device_id` here
        with sd.RawInputStream(samplerate=16000, blocksize=8000, dtype='int16',
                               channels=1, device=device_id, 
                               callback=lambda i, f, t, s: audio_callback(i, f, t, s, q)):
            
            while rclpy.ok():
                try:
                    data = q.get(timeout=0.1)
                    if recognizer.AcceptWaveform(data):
                        result = json.loads(recognizer.Result())
                        text = result.get("text", "")
                        
                        if text:
                            print(f"Heard: {text}")

                            if "stop" in text or "exit" in text:
                                print("🛑 EMERGENCY STOP")
                                engine.say("Stopping")
                                engine.runAndWait()
                                break 
                            
                            found_dir = None
                            for d in node.poses.keys():
                                if d in text:
                                    found_dir = d
                                    break
                            
                            if found_dir:
                                print(f"Executing: {found_dir}")
                                engine.say(f"Moving {found_dir}")
                                engine.runAndWait() 
                                node.tick(node.poses[found_dir])

                except queue.Empty:
                    pass 
                rclpy.spin_once(node, timeout_sec=0.01)

    except KeyboardInterrupt:
        print("Shutting down")
    except Exception as e:
        print(f"❌ CRITICAL ERROR: {e}")
        print("Try running: sudo apt-get install libportaudio2")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()