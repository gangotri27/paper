#!/usr/bin/env python3
import time
import numpy as np
import sounddevice as sd
import rclpy
from rclpy.node import Node
from faster_whisper import WhisperModel
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

# --- SETTINGS (TWEAK THESE!) ---
MODEL_SIZE = "tiny.en"
VOLUME_THRESHOLD = 500  # <--- ADJUST THIS! If it triggers on air, increase to 300 or 500.
SILENCE_LIMIT = 2       # Seconds of silence to wait before stopping recording
MAX_RECORD_SECONDS = 5  # Max time to listen per command

class SmartVoiceNode(Node):
    def __init__(self):
        super().__init__("smart_voice_node")
        
        # ROS Publishers
        self.pub = self.create_publisher(JointTrajectory, "/arm_controller/joint_trajectory", 10)
        self.joints = ["joint1", "joint2", "joint3", "joint4"]
        
        # Fixed Poses
        self.poses = {
            "right": [-1.57, 0.0, 0.0, 0.0],
            "left":  [1.57, 0.0, 0.0, 0.0],
            "up":    [0.0, 0.0, 0.0, -1.0],
            "down":  [0.0, 0.0, 0.0, 1.0],
            "reset": [0.0, 0.0, 0.0, 0.0]
        }

        # Load Whisper
        self.get_logger().info(f"⏳ Loading Whisper ({MODEL_SIZE})...")
        self.model = WhisperModel(MODEL_SIZE, device="cpu", compute_type="int8")
        self.get_logger().info("✅ Robot Ready! Just speak naturally.")

    def get_pulse_id(self):
        """Finds PulseAudio in Docker/WSL"""
        for i, d in enumerate(sd.query_devices()):
            if 'pulse' in d['name'].lower() and d['max_input_channels'] > 0:
                return i
        return 0

    def move_robot(self, command):
        found_dir = None
        for direction in self.poses.keys():
            if direction in command:
                found_dir = direction
                break
        
        if found_dir:
            self.get_logger().info(f"🚀 EXECUTING: {found_dir.upper()}")
            msg = JointTrajectory(joint_names=self.joints)
            point = JointTrajectoryPoint(positions=self.poses[found_dir], time_from_start=Duration(sec=2))
            msg.points.append(point)
            self.pub.publish(msg)
        else:
            self.get_logger().info("🤷 Command recognized, but no matching move found.")

    def listen_loop(self):
        device_id = self.get_pulse_id()
        fs = 16000
        
        self.get_logger().info(f"🎤 Listening on Device {device_id}...")
        self.get_logger().info(f"📊 Threshold set to {VOLUME_THRESHOLD}. Waiting for voice...")

        with sd.InputStream(callback=None, device=device_id, channels=1, samplerate=fs, dtype='int16') as stream:
            while rclpy.ok():
                # 1. WATCHDOG: Read small chunks to check volume
                # Read 0.1 seconds of audio
                data, _ = stream.read(int(fs * 0.1)) 
                
                # Calculate loudness (Max Amplitude)
                volume = np.max(np.abs(data))
                
                # Debug print (Comment this out later if it's annoying)
                # print(f"\rVol: {volume}   ", end="", flush=True)

                if volume > VOLUME_THRESHOLD:
                    print(f"\n🔔 Voice Detected! (Vol: {volume}) - Recording...")
                    
                    # 2. RECORDING PHASE
                    # We detected voice, now capture a full buffer
                    # We record 'MAX_RECORD_SECONDS' worth of audio
                    full_recording = []
                    full_recording.append(data) # Keep the chunk that triggered it
                    
                    # Record the rest
                    rec_data, _ = stream.read(int(fs * MAX_RECORD_SECONDS))
                    full_recording.append(rec_data)
                    
                    # Merge and convert to Float32 for Whisper
                    audio_int16 = np.concatenate(full_recording)
                    audio_float = audio_int16.flatten().astype(np.float32) / 32768.0
                    
                    # 3. TRANSCRIPTION PHASE
                    print("⏳ Thinking...")
                    segments, _ = self.model.transcribe(audio_float, beam_size=5, vad_filter=True)
                    text = "".join([s.text for s in segments]).strip().lower()
                    
                    print(f"📝 You said: '{text}'")
                    
                    if text:
                        self.move_robot(text)
                    
                    print("👂 Listening again...\n")
                    
                    # Clear buffer by reading a bit to avoid echo
                    stream.read(int(fs * 0.5))

def main():
    rclpy.init()
    node = SmartVoiceNode()
    try:
        node.listen_loop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()