#!/usr/bin/env python3

import rospy
import threading
import time
import json
import os
import sys
import yaml
import pyaudio
import actionlib
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from vosk import Model, KaldiRecognizer
from ctypes import *
from contextlib import contextmanager

# --- CONFIGURATION ---
LINEAR_SPEED  = 0.15
ANGULAR_SPEED = 0.5
CMD_TIMEOUT   = 8.0
MODEL_PATH    = "/home/user/vosk-models/vosk-model-small-en-us-0.15"
POINTS_FILE   = os.path.expanduser("~/catkin_ws/src/museum_nav/config/points.yaml")

# --- ALSA ERROR SUPPRESSOR ---
ERROR_HANDLER_FUNC = CFUNCTYPE(None, c_char_p, c_int, c_char_p, c_int, c_char_p)
def py_error_handler(filename, line, function, err, fmt): pass
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


class VoiceNav:
    def __init__(self):
        rospy.init_node('voice_nav', anonymous=True)

        # --- Load waypoints ---
        try:
            with open(POINTS_FILE, "r") as f:
                self.points = yaml.safe_load(f)
        except IOError:
            print("[Error] Cannot find points.yaml")
            sys.exit(1)

        # Build keyword map: lowercase point name -> original key
        # e.g. "statue" -> "STATUE"
        self.point_keywords = {k.lower(): k for k in self.points.keys()}
        print(f"[System] Loaded waypoints: {list(self.points.keys())}")

        # --- ROS publishers / action client ---
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.nav_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        print("[System] Waiting for move_base action server...")
        self.nav_client.wait_for_server()
        print("[System] move_base ready.")

        # --- Robot state ---
        self.target_linear   = 0.0
        self.target_angular  = 0.0
        self.last_cmd_time   = time.time()
        self.is_stopped      = True
        self.navigating      = False   # True while a nav goal is active
        self.twist_msg       = Twist()

        # --- Vosk setup ---
        if not os.path.exists(MODEL_PATH):
            print(f"[Error] Vosk model not found at: {MODEL_PATH}")
            sys.exit(1)

        print("[System] Loading Vosk model...")
        with open(os.devnull, "w") as devnull:
            old_stderr = sys.stderr
            sys.stderr = devnull
            try:
                self.model = Model(MODEL_PATH)
            finally:
                sys.stderr = old_stderr

        self.recognizer = KaldiRecognizer(self.model, 16000)

        print("[System] Initializing microphone...")
        with no_alsa_error():
            self.p = pyaudio.PyAudio()

        self.stream = self.p.open(
            format=pyaudio.paInt16,
            channels=1,
            rate=16000,
            input=True,
            frames_per_buffer=4096
        )
        self.stream.start_stream()

        rospy.on_shutdown(self.cleanup)

        print("\n" + "="*45)
        print("        VOICE NAVIGATION STARTED        ")
        print("="*45)
        print("Teleop  : FORWARD, BACKWARD, LEFT, RIGHT, STOP")
        print("Navigate: 'to' + point name")
        print(f"Points  : {', '.join(self.points.keys())}")
        print(f"Timeout : {CMD_TIMEOUT}s\n")

    # ------------------------------------------------------------------
    # Navigation
    # ------------------------------------------------------------------

    def navigate_to(self, point_key):
        """Send a navigation goal in a separate thread so voice keeps working."""
        def _nav():
            self.navigating = True
            data = self.points[point_key]
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp    = rospy.Time.now()
            goal.target_pose.pose.position.x    = data["x"]
            goal.target_pose.pose.position.y    = data["y"]
            goal.target_pose.pose.orientation.x = data["qx"]
            goal.target_pose.pose.orientation.y = data["qy"]
            goal.target_pose.pose.orientation.z = data["qz"]
            goal.target_pose.pose.orientation.w = data["qw"]

            print(f"[Nav] Navigating to {point_key}...")
            self.nav_client.send_goal(goal)
            self.nav_client.wait_for_result()

            state = self.nav_client.get_state()
            if state == actionlib.GoalStatus.SUCCEEDED:
                print(f"[Nav] Arrived at {point_key}.")
            else:
                print(f"[Nav] Navigation to {point_key} failed or was cancelled (state={state}).")

            self.navigating = False

        t = threading.Thread(target=_nav)
        t.daemon = True
        t.start()

    def cancel_navigation(self):
        """Cancel any active navigation goal."""
        if self.navigating:
            self.nav_client.cancel_all_goals()
            self.navigating = False
            print("[Nav] Navigation cancelled.")

    # ------------------------------------------------------------------
    # Voice processing
    # ------------------------------------------------------------------

    def process_text(self, text):
        """Parse recognised text and act on it."""
        print(f"[Voice] Heard: '{text}'")

        # --- STOP (always takes priority) ---
        if 'stop' in text or 'halt' in text:
            self.cancel_navigation()
            self.target_linear  = 0.0
            self.target_angular = 0.0
            self.is_stopped     = True
            self.last_cmd_time  = time.time()
            print("[Command] >> STOP")
            return

        # --- NAVIGATION: keyword "robot" followed by a point name ---
        if 'to' in text:
            for keyword, point_key in self.point_keywords.items():
                if keyword in text:
                    # Stop any current teleop movement
                    self.target_linear  = 0.0
                    self.target_angular = 0.0
                    self.is_stopped     = True
                    self.cancel_navigation()
                    self.navigate_to(point_key)
                    self.last_cmd_time = time.time()
                    return
            print("[Voice] 'to' heard but no known point detected.")
            return

        # --- TELEOP (only when not navigating autonomously) ---
        if self.navigating:
            print("[Voice] Teleop ignored: navigation in progress. Say 'stop' to cancel.")
            return

        if 'forward' in text or 'go' in text:
            self.target_linear  =  LINEAR_SPEED
            self.target_angular =  0.0
            self.is_stopped     =  False
            self.last_cmd_time  =  time.time()
            print("[Command] >> FORWARD")
        elif 'backward' in text or 'back' in text:
            self.target_linear  = -LINEAR_SPEED
            self.target_angular =  0.0
            self.is_stopped     =  False
            self.last_cmd_time  =  time.time()
            print("[Command] >> BACKWARD")
        elif 'left' in text:
            self.target_linear  =  0.0
            self.target_angular =  ANGULAR_SPEED
            self.is_stopped     =  False
            self.last_cmd_time  =  time.time()
            print("[Command] >> LEFT")
        elif 'right' in text:
            self.target_linear  =  0.0
            self.target_angular = -ANGULAR_SPEED
            self.is_stopped     =  False
            self.last_cmd_time  =  time.time()
            print("[Command] >> RIGHT")

    def voice_listener_loop(self):
        """Background thread: reads mic and feeds Vosk."""
        while not rospy.is_shutdown():
            try:
                data = self.stream.read(4096, exception_on_overflow=False)
                if self.recognizer.AcceptWaveform(data):
                    result = json.loads(self.recognizer.Result())
                    text   = result.get("text", "").strip()
                    if text:
                        self.process_text(text)
            except OSError:
                pass

    # ------------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------------

    def run(self):
        voice_thread = threading.Thread(target=self.voice_listener_loop)
        voice_thread.daemon = True
        voice_thread.start()

        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            # Safety timeout: stop teleop if no command for CMD_TIMEOUT seconds
            if (not self.navigating
                    and not self.is_stopped
                    and time.time() - self.last_cmd_time > CMD_TIMEOUT):
                print(f"[Safety] No command for {CMD_TIMEOUT}s. Stopping.")
                self.target_linear  = 0.0
                self.target_angular = 0.0
                self.is_stopped     = True

            # Publish velocity only when not under autonomous navigation
            if not self.navigating:
                self.twist_msg.linear.x  = self.target_linear
                self.twist_msg.angular.z = self.target_angular
                self.vel_pub.publish(self.twist_msg)

            rate.sleep()

    # ------------------------------------------------------------------
    # Cleanup
    # ------------------------------------------------------------------

    def cleanup(self):
        print("[System] Shutting down...")
        self.cancel_navigation()
        self.twist_msg.linear.x  = 0.0
        self.twist_msg.angular.z = 0.0
        self.vel_pub.publish(self.twist_msg)
        try:
            self.stream.stop_stream()
            self.stream.close()
            self.p.terminate()
        except:
            pass


if __name__ == '__main__':
    try:
        controller = VoiceNav()
        controller.run()
    except rospy.ROSInterruptException:
        pass