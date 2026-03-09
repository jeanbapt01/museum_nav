#!/usr/bin/env python3

import rospy
import threading
import time
import json
import os
import sys
import pyaudio
from geometry_msgs.msg import Twist
from vosk import Model, KaldiRecognizer
from ctypes import *
from contextlib import contextmanager

# --- CONFIGURATION ---
LINEAR_SPEED = 0.15   # m/s
ANGULAR_SPEED = 0.5   # rad/s
CMD_TIMEOUT = 8.0     # Robot stops if no command is detected for X seconds
MODEL_PATH = "/home/user/vosk-models/vosk-model-small-en-us-0.15"  # Path to the Vosk model folder

# --- ALSA ERROR HANDLER ---
# This section interacts with the C-level sound library to suppress warnings
ERROR_HANDLER_FUNC = CFUNCTYPE(None, c_char_p, c_int, c_char_p, c_int, c_char_p)

def py_error_handler(filename, line, function, err, fmt):
    pass

c_error_handler = ERROR_HANDLER_FUNC(py_error_handler)

@contextmanager
def no_alsa_error():
    try:
        asound = cdll.LoadLibrary('libasound.so')
        asound.snd_lib_error_set_handler(c_error_handler)
        yield
        asound.snd_lib_error_set_handler(None)
    except:
        yield

class VoiceTeleopVosk:
    def __init__(self):
        # Initialize ROS Node
        rospy.init_node('voice_teleop_vosk', anonymous=True)
        
        # Publisher
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # Check Model Path
        # Handles both absolute path and path relative to the script
        script_dir = os.path.dirname(os.path.realpath(__file__))
        abs_model_path = os.path.join(script_dir, MODEL_PATH)
        
        if not os.path.exists(abs_model_path):
            print(f"[Error] Vosk model not found at: {abs_model_path}")
            print("Please download 'vosk-model-small-en-us' and rename the folder to 'model'.")
            sys.exit(1)

        print("[System] Loading Vosk Model... (this may take a moment)")
        # Redirect stderr to suppress Vosk internal logs during model loading
        with open(os.devnull, "w") as devnull:
            old_stderr = sys.stderr
            sys.stderr = devnull
            try:
                self.model = Model(abs_model_path)
            finally:
                sys.stderr = old_stderr

        self.recognizer = KaldiRecognizer(self.model, 16000)
        
        # Initialize Microphone with ALSA suppression
        print("[System] Initializing Microphone...")
        with no_alsa_error():
            self.p = pyaudio.PyAudio()
        
        # Open Audio Stream
        # 16kHz, Mono, 16bit is standard for Vosk models
        self.stream = self.p.open(format=pyaudio.paInt16, 
                                  channels=1, 
                                  rate=16000, 
                                  input=True, 
                                  frames_per_buffer=4096)
        self.stream.start_stream()
        
        # Robot State
        self.target_linear = 0.0
        self.target_angular = 0.0
        self.last_command_time = time.time()
        self.is_stopped = True
        self.twist_msg = Twist()

        rospy.on_shutdown(self.cleanup)
        
        print("\n" + "="*40)
        print("   VOICE TELEOP STARTED (OFFLINE)   ")
        print("="*40)
        print(f"Commands: FORWARD, BACKWARD, LEFT, RIGHT, STOP")
        print(f"Safety Timeout: {CMD_TIMEOUT} seconds")
        print("You can speak now.\n")

    def process_text(self, text):
        """Parses the JSON text from Vosk and updates velocity."""
        valid_command = False
        
        # Check keywords
        if 'forward' in text or 'go' in text:
            self.target_linear = LINEAR_SPEED
            self.target_angular = 0.0
            valid_command = True
        elif 'backward' in text or 'back' in text:
            self.target_linear = -LINEAR_SPEED
            self.target_angular = 0.0
            valid_command = True
        elif 'left' in text:
            self.target_linear = 0.0
            self.target_angular = ANGULAR_SPEED
            valid_command = True
        elif 'right' in text:
            self.target_linear = 0.0
            self.target_angular = -ANGULAR_SPEED
            valid_command = True
        elif 'stop' in text or 'halt' in text:
            self.target_linear = 0.0
            self.target_angular = 0.0
            valid_command = True
            print("[Command] >> STOP")

        if valid_command:
            self.last_command_time = time.time()
            self.is_stopped = (self.target_linear == 0 and self.target_angular == 0)
            if not self.is_stopped:
                print(f"[Command] >> {text.upper()}")

    def voice_listener_loop(self):
        """Background thread for audio processing."""
        while not rospy.is_shutdown():
            try:
                # Read raw audio chunk
                data = self.stream.read(4096, exception_on_overflow=False)
                
                # AcceptWaveform processes audio stream
                if self.recognizer.AcceptWaveform(data):
                    result = json.loads(self.recognizer.Result())
                    text = result.get("text", "")
                    if text:
                        self.process_text(text)
                else:
                    # Partial results (useful for debugging latency, generally ignored here)
                    # partial = json.loads(self.recognizer.PartialResult())
                    pass
            except OSError:
                # Handle stream errors gently
                pass

    def run(self):
        """Main ROS loop."""
        # Start listener thread
        voice_thread = threading.Thread(target=self.voice_listener_loop)
        voice_thread.daemon = True
        voice_thread.start()

        rate = rospy.Rate(10) # 10 Hz
        
        while not rospy.is_shutdown():
            # Safety Check: Stop robot if no command received recently
            if (time.time() - self.last_command_time > CMD_TIMEOUT) and not self.is_stopped:
                print(f"[Safety] No command for {CMD_TIMEOUT}s. Emergency Stop.")
                self.target_linear = 0.0
                self.target_angular = 0.0
                self.is_stopped = True

            # Publish
            self.twist_msg.linear.x = self.target_linear
            self.twist_msg.angular.z = self.target_angular
            self.velocity_publisher.publish(self.twist_msg)
            
            rate.sleep()

    def cleanup(self):
        """Clean shutdown of streams and publisher."""
        print("[System] Shutting down...")
        self.twist_msg.linear.x = 0.0
        self.twist_msg.angular.z = 0.0
        self.velocity_publisher.publish(self.twist_msg)
        
        try:
            self.stream.stop_stream()
            self.stream.close()
            self.p.terminate()
        except:
            pass

if __name__ == '__main__':
    try:
        controller = VoiceTeleopVosk()
        controller.run()
    except rospy.ROSInterruptException:
        pass

