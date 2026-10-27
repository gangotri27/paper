#!/usr/bin/env python3
import time
import numpy as np
import sounddevice as sd
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Empty
from faster_whisper import WhisperModel

# --- CONFIGURATION ---
MODEL_SIZE = "tiny.en" 
SAMPLE_RATE = 16000
DURATION = 4.0        # Recording time
VOLUME_BOOST = 2.0    # Double the volume (Software Gain)

class AudioPromptNode(Node):
    def __init__(self):
        super().__init__("audio_prompt_node")
        
        self.publisher_ = self.create_publisher(String, "/prompt", 10)
        self.subscription_ = self.create_subscription(Empty, "/listen", self.listener_callback, 10)
        
        self.get_logger().info(f"⏳ Loading Whisper Model ({MODEL_SIZE})...")
        self.model = WhisperModel(MODEL_SIZE, device="cpu", compute_type="int8")
        self.get_logger().info("✅ Audio Node Ready! Waiting for trigger...")

    def get_pulse_device_id(self):
        """Finds the PulseAudio bridge"""
        try:
            device_list = sd.query_devices()
            for idx, device in enumerate(device_list):
                if 'pulse' in device['name'].lower() and device['max_input_channels'] > 0:
                    return idx
        except Exception:
            pass
        return 0 # Default

    def play_beep(self):
        """Generates a beep sound so you know when to speak"""
        fs = 44100
        duration = 0.2  # seconds
        f = 440.0       # sine frequency (Hz)
        t = np.arange(int(duration * fs)) / fs
        # Generate sine wave
        beep_audio = 0.5 * np.sin(2 * np.pi * f * t)
        # Play it
        try:
            sd.play(beep_audio, fs)
            sd.wait()
        except:
            pass # If output fails, ignore it

    def listener_callback(self, msg):
        self.get_logger().info("🔔 Trigger Received...")
        
        device_id = self.get_pulse_device_id()
        recording = None

        # --- FIX 1: RETRY LOGIC (Solves the -9987 Crash) ---
        for attempt in range(3):
            try:
                # Play beep ONLY on first attempt to signal user
                if attempt == 0:
                    self.play_beep() 
                    print("🔴 REC") # Visual Cue

                recording = sd.rec(int(DURATION * SAMPLE_RATE), 
                                   samplerate=SAMPLE_RATE, 
                                   channels=1, 
                                   device=device_id,
                                   dtype='float32') # Use float32 directly
                sd.wait() # Block until done
                print("⬛ STOP")
                break # Success, exit loop
            except Exception as e:
                self.get_logger().warn(f"⚠️ Mic Error (Attempt {attempt+1}): {e}")
                time.sleep(0.2) # Wait a bit and retry
        
        if recording is None:
            self.get_logger().error("❌ Failed to record audio after 3 attempts.")
            return

        # --- FIX 2: AMPLIFICATION (Solves 'Silence' errors) ---
        # Boost volume by 2x or 3x
        audio_float = recording.flatten() * VOLUME_BOOST
        
        # Check volume AFTER boost
        max_vol = np.max(np.abs(audio_float))
        if max_vol < 0.01: # Threshold for float32 (0.0 to 1.0)
            self.get_logger().warn("⚠️ Audio too silent. Did you speak?")
            return

        # --- TRANSCRIPTION ---
        self.get_logger().info("⏳ Transcribing...")
        try:
            # vad_filter=True stops it from reading "Air"
            segments, info = self.model.transcribe(audio_float, beam_size=5, vad_filter=True)
            
            detected_text = "".join([segment.text for segment in segments]).strip()
            
            # Simple clean up
            detected_text = detected_text.replace('.', '').replace('?', '').lower()

            if len(detected_text) > 1: # Ignore single letters
                self.get_logger().info(f"📝 Recognized: '{detected_text}'")
                msg_out = String()
                msg_out.data = detected_text
                self.publisher_.publish(msg_out)
            else:
                self.get_logger().info("🤷 Speech unclear.")

        except Exception as e:
            self.get_logger().error(f"Whisper Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = AudioPromptNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()


























# #!/usr/bin/env python3
# import time
# import numpy as np
# import sounddevice as sd
# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import String, Empty
# from faster_whisper import WhisperModel

# # --- CONFIGURATION ---
# MODEL_SIZE = "tiny.en" 
# DEVICE_ID = 0 # PulseAudio Device Index (Change if needed)
# SAMPLE_RATE = 16000
# DURATION = 5.0 # How long to listen after trigger

# class AudioPromptNode(Node):
#     def __init__(self):
#         super().__init__("audio_prompt_node")
        
#         # 1. Publishers and Subscribers
#         self.publisher_ = self.create_publisher(String, "/prompt", 10)
#         self.subscription_ = self.create_subscription(
#             Empty, 
#             "/listen", 
#             self.listener_callback, 
#             10
#         )
        
#         # 2. Load Model Once (at startup)
#         self.get_logger().info(f"⏳ Loading Whisper Model ({MODEL_SIZE})...")
#         self.model = WhisperModel(MODEL_SIZE, device="cpu", compute_type="int8")
#         self.get_logger().info("✅ Audio Node Ready! Waiting for '/listen' trigger...")

#     def get_pulse_device_id(self):
#         """Helper to find the PulseAudio bridge in Docker/WSL"""
#         device_list = sd.query_devices()
#         for idx, device in enumerate(device_list):
#             if 'pulse' in device['name'].lower() and device['max_input_channels'] > 0:
#                 return idx
#         return 0 # Default

#     def listener_callback(self, msg):
#         """This runs ONLY when you send a message to /listen"""
#         self.get_logger().info("🔔 Trigger Received! Listening for command...")
        
#         try:
#             # 1. Record Audio (Blocking for DURATION seconds)
#             # We record in INT16 because it's reliable in WSL
#             device_id = self.get_pulse_device_id()
#             recording = sd.rec(int(DURATION * SAMPLE_RATE), 
#                                samplerate=SAMPLE_RATE, 
#                                channels=1, 
#                                device=device_id,
#                                dtype='int16')
#             sd.wait() # Wait until recording finishes
            
#             # 2. Check Volume (Simple Safety)
#             max_vol = np.max(np.abs(recording))
#             if max_vol < 100:
#                 self.get_logger().warn("⚠️ Mic input silent. Check settings.")
#                 return

#             # 3. Convert to Float32 for Whisper
#             audio_float = recording.flatten().astype(np.float32) / 32768.0

#             # 4. Transcribe
#             self.get_logger().info("⏳ Transcribing...")
#             segments, info = self.model.transcribe(audio_float, beam_size=5)
            
#             detected_text = "".join([segment.text for segment in segments]).strip()
            
#             # 5. Publish Result
#             if detected_text:
#                 self.get_logger().info(f"📝 Recognized: '{detected_text}'")
                
#                 msg_out = String()
#                 msg_out.data = detected_text.lower()
#                 self.publisher_.publish(msg_out)
#             else:
#                 self.get_logger().info("🤷 No speech detected.")

#         except Exception as e:
#             self.get_logger().error(f"Error during recording: {e}")

# def main(args=None):
#     rclpy.init(args=args)
#     node = AudioPromptNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == "__main__":
#     main()






















# #!/usr/bin/env python3
# import os
# import queue
# import time
# import numpy as np
# import sounddevice as sd
# import pyttsx3
# import rclpy
# from rclpy.node import Node
# from faster_whisper import WhisperModel
# from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
# from builtin_interfaces.msg import Duration

# # --- CONFIGURATION ---
# MODEL_SIZE = "tiny.en" 
# TRAJ_TOPIC = "/arm_controller/joint_trajectory"
# JOINTS = ["joint1", "joint2", "joint3", "joint4"]

# class Demo(Node):
#     def __init__(self):
#         super().__init__("omx_traj_demo")
#         self.pub = self.create_publisher(JointTrajectory, TRAJ_TOPIC, 10)
#         self.poses = {
#             "right": [-1.57, 0.0, 0.0, 0.0],
#             "left":  [1.57, 0.0, 0.0, 0.0],
#             "up":    [0.0, 0.0, 0.0, -1.0],
#             "down":  [0.0, 0.0, 0.0, 1.0]
#         }
#         self.get_logger().info(f"Started. Publishing to {TRAJ_TOPIC}")

#     def tick(self, pose):
#         msg = JointTrajectory(joint_names=JOINTS)
#         p = JointTrajectoryPoint(positions=pose, time_from_start=Duration(sec=2))
#         msg.points.append(p)
#         self.pub.publish(msg)

# def get_pulse_device_id():
#     device_list = sd.query_devices()
#     for idx, device in enumerate(device_list):
#         if 'pulse' in device['name'].lower() and device['max_input_channels'] > 0:
#             return idx
#     return None

# def main():
#     rclpy.init()
#     node = Demo()
#     engine = pyttsx3.init()
    
#     print(f"⏳ Loading Whisper Model ({MODEL_SIZE})...")
#     model = WhisperModel(MODEL_SIZE, device="cpu", compute_type="int8")
#     print("✅ Model Loaded!")
    
#     # Audio settings
#     SAMPLE_RATE = 16000
#     DURATION = 4.0 
    
#     device_id = get_pulse_device_id()
#     print(f"🎤 Listening on Device {device_id}...")
#     print("Commands: 'Move Right', 'Go Up', 'Stop', 'Exit'")

#     try:
#         while rclpy.ok():
#             print(f"\n🎤 Speak now ({DURATION}s)...")
            
#             # --- FIX 1: Record in Int16 (Robust) ---
#             recording = sd.rec(int(DURATION * SAMPLE_RATE), 
#                                samplerate=SAMPLE_RATE, 
#                                channels=1, 
#                                device=device_id,
#                                dtype='int16') # Vosk style
#             sd.wait() 

#             # --- DEBUG: Check Volume ---
#             # If this number is near 0, your mic is muted or not working
#             max_vol = np.max(np.abs(recording))
#             print(f"📊 Volume Level: {max_vol}")

#             if max_vol < 100:
#                 print("⚠️ Audio too quiet or silent. Check Windows Mic settings.")
#                 continue

#             # --- FIX 2: Convert to Float32 for Whisper ---
#             # Normalize int16 (-32768 to 32767) to float32 (-1.0 to 1.0)
#             audio_float = recording.flatten().astype(np.float32) / 32768.0

#             print("⏳ Transcribing...")
            
#             # Transcribe
#             # vad_filter=True helps remove silence
#             segments, info = model.transcribe(
#                 audio_float, 
#                 beam_size=5, 
#                 vad_filter=True,  # <--- THIS IS THE MAGIC SWITCH
#                 vad_parameters=dict(min_silence_duration_ms=500) # Tweaking sensitivity
#             )
            
#             detected_text = "".join([segment.text for segment in segments]).strip().lower()
            
#             # Remove punctuation to make matching easier
#             detected_text = detected_text.replace('.', '').replace('!', '').replace('?', '')
#             print(f"📝 You said: '{detected_text}'")

#             if detected_text:
#                 if "stop" in detected_text or "exit" in detected_text:
#                     print("🛑 EMERGENCY STOP")
#                     engine.say("Stopping")
#                     engine.runAndWait()
#                     break

#                 found_dir = None
#                 for d in node.poses.keys():
#                     if d in detected_text:
#                         found_dir = d
#                         break
                
#                 if found_dir:
#                     print(f"🚀 Executing: {found_dir}")
#                     engine.say(f"Moving {found_dir}")
#                     engine.runAndWait()
#                     node.tick(node.poses[found_dir])
#                 else:
#                     print("🤷 Command not understood.")
#             else:
#                 print("🤷 (Silence or Unclear)")
            
#             rclpy.spin_once(node, timeout_sec=0.1)

#     except KeyboardInterrupt:
#         print("Shutting down")
#     except Exception as e:
#         print(f"Error: {e}")
#     finally:
#         # Proper ROS shutdown check
#         if rclpy.ok():
#             node.destroy_node()
#             rclpy.shutdown()

# if __name__ == "__main__":
#     main()









# #!/usr/bin/env python3
# import os
# import queue
# import time
# import numpy as np
# import sounddevice as sd
# import pyttsx3
# import rclpy
# from rclpy.node import Node
# from faster_whisper import WhisperModel
# from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
# from builtin_interfaces.msg import Duration

# # --- CONFIGURATION ---
# MODEL_SIZE = "tiny.en" 
# TRAJ_TOPIC = "/arm_controller/joint_trajectory"
# JOINTS = ["joint1", "joint2", "joint3", "joint4"]

# class Demo(Node):
#     def __init__(self):
#         super().__init__("omx_traj_demo")
#         self.pub = self.create_publisher(JointTrajectory, TRAJ_TOPIC, 10)
#         self.poses = {
#             "right": [-1.57, 0.0, 0.0, 0.0],
#             "left":  [1.57, 0.0, 0.0, 0.0],
#             "up":    [0.0, 0.0, 0.0, -1.0],
#             "down":  [0.0, 0.0, 0.0, 1.0]
#         }
#         self.get_logger().info(f"Started. Publishing to {TRAJ_TOPIC}")

#     def tick(self, pose):
#         msg = JointTrajectory(joint_names=JOINTS)
#         p = JointTrajectoryPoint(positions=pose, time_from_start=Duration(sec=2))
#         msg.points.append(p)
#         self.pub.publish(msg)

# def get_pulse_device_id():
#     device_list = sd.query_devices()
#     for idx, device in enumerate(device_list):
#         if 'pulse' in device['name'].lower() and device['max_input_channels'] > 0:
#             return idx
#     return None

# def main():
#     rclpy.init()
#     node = Demo()
#     engine = pyttsx3.init()
    
#     print(f"⏳ Loading Whisper Model ({MODEL_SIZE})...")
#     model = WhisperModel(MODEL_SIZE, device="cpu", compute_type="int8")
#     print("✅ Model Loaded!")
    
#     # Audio settings
#     SAMPLE_RATE = 16000
#     DURATION = 4.0 
    
#     device_id = get_pulse_device_id()
#     print(f"🎤 Listening on Device {device_id}...")
#     print("Commands: 'Move Right', 'Go Up', 'Stop', 'Exit'")

#     try:
#         while rclpy.ok():
#             print(f"\n🎤 Speak now ({DURATION}s)...")
            
#             # --- FIX 1: Record in Int16 (Robust) ---
#             recording = sd.rec(int(DURATION * SAMPLE_RATE), 
#                                samplerate=SAMPLE_RATE, 
#                                channels=1, 
#                                device=device_id,
#                                dtype='int16') # Vosk style
#             sd.wait() 

#             # --- DEBUG: Check Volume ---
#             # If this number is near 0, your mic is muted or not working
#             max_vol = np.max(np.abs(recording))
#             print(f"📊 Volume Level: {max_vol}")

#             if max_vol < 100:
#                 print("⚠️ Audio too quiet or silent. Check Windows Mic settings.")
#                 continue

#             # --- FIX 2: Convert to Float32 for Whisper ---
#             # Normalize int16 (-32768 to 32767) to float32 (-1.0 to 1.0)
#             audio_float = recording.flatten().astype(np.float32) / 32768.0

#             print("⏳ Transcribing...")
            
#             # Transcribe
#             # vad_filter=True helps remove silence
#             segments, info = model.transcribe(audio_float, beam_size=5, vad_filter=True)
            
#             detected_text = "".join([segment.text for segment in segments]).strip().lower()
            
#             # Remove punctuation to make matching easier
#             detected_text = detected_text.replace('.', '').replace('!', '').replace('?', '')
#             print(f"📝 You said: '{detected_text}'")

#             if detected_text:
#                 if "stop" in detected_text or "exit" in detected_text:
#                     print("🛑 EMERGENCY STOP")
#                     engine.say("Stopping")
#                     engine.runAndWait()
#                     break

#                 found_dir = None
#                 for d in node.poses.keys():
#                     if d in detected_text:
#                         found_dir = d
#                         break
                
#                 if found_dir:
#                     print(f"🚀 Executing: {found_dir}")
#                     engine.say(f"Moving {found_dir}")
#                     engine.runAndWait()
#                     node.tick(node.poses[found_dir])
#                 else:
#                     print("🤷 Command not understood.")
#             else:
#                 print("🤷 (Silence or Unclear)")
            
#             rclpy.spin_once(node, timeout_sec=0.1)

#     except KeyboardInterrupt:
#         print("Shutting down")
#     except Exception as e:
#         print(f"Error: {e}")
#     finally:
#         # Proper ROS shutdown check
#         if rclpy.ok():
#             node.destroy_node()
#             rclpy.shutdown()

# if __name__ == "__main__":
#     main()