#!/usr/bin/env python3
"""
light_nav_v2.py — Museum robot navigation
Changes vs v1:
  - Push-to-listen: press ENTER to toggle voice recognition on/off
  - Gamepad support via ROS /joy topic (PS3 / DualShock 3):
      Buttons  → navigate to destinations
      L-stick  → forward / backward
      R-stick  → rotate left / right
"""

import rospy
import threading
import time
import os
import sys
import yaml
import actionlib
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import Joy
from pocketsphinx import LiveSpeech

# ---------------------------------------------------------------------------
# CONFIGURATION
# ---------------------------------------------------------------------------
LINEAR_SPEED   = 0.15   # m/s
ANGULAR_SPEED  = 0.5    # rad/s
CMD_TIMEOUT    = 8.0    # seconds before teleop auto-stop
CONFIDENCE_THR = -6000  # log-prob threshold

POINTS_FILE  = os.path.expanduser("~/catkin_ws/src/museum_nav/config/pointsv2.yaml")
GRAMMAR_FILE = os.path.expanduser("~/catkin_ws/src/museum_nav/speech/museum.gram")
DICT_FILE    = os.path.expanduser("~/catkin_ws/src/museum_nav/speech/museum.dict")

# ---------------------------------------------------------------------------
# PS3 CONTROLLER MAPPING
#
#  Verify with: rostopic echo /joy
#
#  axes[0]  : L-stick X  (left +1, right -1)
#  axes[1]  : L-stick Y  (up   +1, down  -1)  → forward / backward
#  axes[2]  : R-stick X  (left +1, right -1)  → rotation
#  axes[3]  : R-stick Y
#
#  buttons[12] : Triangle  → SHARK
#  buttons[14] : Cross     → TURTLE
#  buttons[13] : Circle    → OCTOPUS
#  buttons[15] : Square    → HOME
#  buttons[3]  : Start     → WELCOME
#  buttons[4]  : Select    → STOP
# ---------------------------------------------------------------------------
AXIS_LINEAR   = 1    # L-stick Y
AXIS_ANGULAR  = 2    # R-stick X
AXIS_DEADZONE = 0.15

BUTTON_DEST = {
    1: "SHARK",
    4: "TURTLE",
    3: "OCTOPUS",
    10: "HOME",
     0: "WELCOME",
}
BUTTON_STOP = 11   # Select

# ---------------------------------------------------------------------------
# DESCRIPTIONS
# ---------------------------------------------------------------------------
DESCRIPTIONS = {
    "SHARK": """
╔══════════════════════════════════════════════════════════╗
║                        🦈  SHARK                        ║
╠══════════════════════════════════════════════════════════╣
║  Diet: Carnivore — fish, seals, cephalopods              ║
║                                                          ║
║  Special abilities:                                      ║
║  • Ampullae of Lorenzini: detects electric fields        ║
║    produced by prey up to 1 metre away                   ║
║  • Lateral line: senses vibrations in the water          ║
║  • Can detect one drop of blood in 100 litres of water   ║
╚══════════════════════════════════════════════════════════╝
""",
    "TURTLE": """
╔══════════════════════════════════════════════════════════╗
║                    🐢  SEA TURTLE                       ║
╠══════════════════════════════════════════════════════════╣
║  Diet: Omnivore — jellyfish, seagrass, crustaceans       ║
║                                                          ║
║  Special abilities:                                      ║
║  • Navigates using Earth's magnetic field to return      ║
║    to its birth beach to lay eggs                        ║
║  • Can hold breath for up to 7 hours while resting       ║
║  • Has existed for over 100 million years                ║
╚══════════════════════════════════════════════════════════╝
""",
    "OCTOPUS": """
╔══════════════════════════════════════════════════════════╗
║                      🐙  OCTOPUS                        ║
╠══════════════════════════════════════════════════════════╣
║  Diet: Carnivore — crabs, molluscs, small fish           ║
║                                                          ║
║  Special abilities:                                      ║
║  • Changes colour in under 1 second via chromatophores   ║
║  • Ejects ink cloud to blind predators and escape        ║
║  • 3 hearts and blue blood (copper-based haemocyanin)    ║
║  • Each arm has an independent nervous system            ║
╚══════════════════════════════════════════════════════════╝
""",
}

WELCOME_MSG = """
╔══════════════════════════════════════════════════════════╗
║                                                          ║
║        🐠  Welcome to the Marine Wildlife Museum  🐠    ║
║                                                          ║
║   VOICE  (press ENTER to toggle 🎙️ / 🔇):               ║
║     SHARK / TURTLE / OCTOPUS / HOME / WELCOME            ║
║     STOP • GO • BACK • LEFT • RIGHT                      ║
║     FASTER / SLOWER                                      ║
║                                                          ║
║   GAMEPAD (PS3):                                         ║
║     Triangle → SHARK      Cross   → TURTLE               ║
║     Circle   → OCTOPUS    Square  → HOME                 ║
║     Start    → WELCOME    Select  → STOP                 ║
║     L-stick  → Forward / Backward                        ║
║     R-stick  → Rotate Left / Right                       ║
║                                                          ║
╚══════════════════════════════════════════════════════════╝
"""


# ---------------------------------------------------------------------------
# MAIN CLASS
# ---------------------------------------------------------------------------
class VoiceNavMuseum:

    def __init__(self):
        rospy.init_node('voice_nav_museum', anonymous=True)

        # --- Load waypoints ---
        try:
            with open(POINTS_FILE, "r") as f:
                self.points = yaml.safe_load(f)
        except IOError:
            print("[Error] Cannot find points yaml file")
            sys.exit(1)
        print(f"[System] Loaded waypoints: {list(self.points.keys())}")

        # --- ROS publisher + action client ---
        self.vel_pub    = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.nav_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        print("[System] Waiting for move_base action server...")
        self.nav_client.wait_for_server()
        print("[System] move_base ready.")

        # --- Robot state ---
        self.linear_speed   = LINEAR_SPEED
        self.angular_speed  = ANGULAR_SPEED
        self.target_linear  = 0.0
        self.target_angular = 0.0
        self.last_cmd_time  = time.time()
        self.is_stopped     = True
        self.navigating     = False
        self.twist_msg      = Twist()

        # --- Voice toggle ---
        self.voice_active   = False
        self._voice_lock    = threading.Lock()

        # --- Gamepad stick state ---
        self._stick_linear  = 0.0
        self._stick_angular = 0.0

        # --- Previous button states (detect rising edge, not hold) ---
        self._prev_buttons  = []

        # --- /joy subscriber ---
        rospy.Subscriber('/joy', Joy, self._joy_callback)

        rospy.on_shutdown(self.cleanup)
        print(WELCOME_MSG)
        print("[Voice] Listening is OFF — press ENTER to toggle.")

    # -----------------------------------------------------------------------
    # Navigation
    # -----------------------------------------------------------------------

    def _send_goal(self, point_key):
        if point_key not in self.points:
            print(f"[Nav] Unknown point: {point_key}")
            return None
        data = self.points[point_key]
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id    = "map"
        goal.target_pose.header.stamp       = rospy.Time.now()
        goal.target_pose.pose.position.x    = data["x"]
        goal.target_pose.pose.position.y    = data["y"]
        goal.target_pose.pose.orientation.x = data["qx"]
        goal.target_pose.pose.orientation.y = data["qy"]
        goal.target_pose.pose.orientation.z = data["qz"]
        goal.target_pose.pose.orientation.w = data["qw"]
        self.nav_client.send_goal(goal)
        self.nav_client.wait_for_result()
        return self.nav_client.get_state()

    def navigate_to(self, point_key):
        def _nav():
            self.navigating = True
            print(f"[Nav] Navigating to {point_key}...")
            state = self._send_goal(point_key)
            if state == actionlib.GoalStatus.SUCCEEDED:
                print(f"[Nav] Arrived at {point_key}.")
                if point_key in DESCRIPTIONS:
                    print(DESCRIPTIONS[point_key])
                if point_key == "WELCOME":
                    print(WELCOME_MSG)
            else:
                print(f"[Nav] Navigation to {point_key} failed or was cancelled.")
            self.navigating = False

        threading.Thread(target=_nav, daemon=True).start()

    def cancel_navigation(self):
        if self.navigating:
            self.nav_client.cancel_all_goals()
            self.navigating = False
            print("[Nav] Navigation cancelled.")

    # -----------------------------------------------------------------------
    # Gamepad — /joy callback
    # -----------------------------------------------------------------------

    def _joy_callback(self, msg):
        axes    = msg.axes
        buttons = msg.buttons

        # --- Sticks (continuous) ---
        lin = axes[AXIS_LINEAR]  if len(axes) > AXIS_LINEAR  else 0.0
        ang = axes[AXIS_ANGULAR] if len(axes) > AXIS_ANGULAR else 0.0
        self._stick_linear  = lin if abs(lin) > AXIS_DEADZONE else 0.0
        self._stick_angular = ang if abs(ang) > AXIS_DEADZONE else 0.0

        # --- Buttons (detect rising edge only) ---
        if not self._prev_buttons:
            self._prev_buttons = list(buttons)
            return

        for idx, (prev, curr) in enumerate(zip(self._prev_buttons, buttons)):
            if prev == 0 and curr == 1:
                if idx == BUTTON_STOP:
                    self.process_command("stop", source="gamepad")
                elif idx in BUTTON_DEST:
                    self.process_command(BUTTON_DEST[idx].lower(), source="gamepad")

        self._prev_buttons = list(buttons)

    # -----------------------------------------------------------------------
    # Command dispatcher (shared by voice + gamepad)
    # -----------------------------------------------------------------------

    def process_command(self, word, score=0, source="voice"):
        if source == "voice" and score < CONFIDENCE_THR:
            print(f"[Voice] '{word}' ignored (low confidence: {score})")
            return

        print(f"[{source.upper()}] Command: '{word}'" +
              (f" (score: {score})" if source == "voice" else ""))

        # STOP
        if word == "stop":
            self.cancel_navigation()
            self.target_linear  = 0.0
            self.target_angular = 0.0
            self.is_stopped     = True
            print("[Command] >> STOP")
            return

        # DESTINATIONS
        dest_map = {
            "shark":   "SHARK",
            "turtle":  "TURTLE",
            "octopus": "OCTOPUS",
            "home":    "HOME",
            "welcome": "WELCOME",
        }
        if word in dest_map:
            if self.navigating:
                print(f"[{source.upper()}] Already navigating — STOP first.")
                return
            self.navigate_to(dest_map[word])
            return

        # VOICE TELEOP
        if source == "voice":
            if self.navigating:
                print("[Voice] Teleop ignored: navigation in progress.")
                return
            if word == "go":
                self.target_linear, self.target_angular = self.linear_speed, 0.0
                self.is_stopped = False; self.last_cmd_time = time.time()
                print("[Command] >> GO")
            elif word == "back":
                self.target_linear, self.target_angular = -self.linear_speed, 0.0
                self.is_stopped = False; self.last_cmd_time = time.time()
                print("[Command] >> BACK")
            elif word == "left":
                self.target_linear, self.target_angular = 0.0, self.angular_speed
                self.is_stopped = False; self.last_cmd_time = time.time()
                print("[Command] >> LEFT")
            elif word == "right":
                self.target_linear, self.target_angular = 0.0, -self.angular_speed
                self.is_stopped = False; self.last_cmd_time = time.time()
                print("[Command] >> RIGHT")
            elif word == "faster":
                self.linear_speed  = min(self.linear_speed  + 0.03, 0.22)
                self.angular_speed = min(self.angular_speed + 0.1,  2.0)
                print(f"[Command] >> FASTER (linear={self.linear_speed:.2f} m/s)")
            elif word == "slower":
                self.linear_speed  = max(self.linear_speed  - 0.03, 0.05)
                self.angular_speed = max(self.angular_speed - 0.1,  0.2)
                print(f"[Command] >> SLOWER (linear={self.linear_speed:.2f} m/s)")

    # -----------------------------------------------------------------------
    # Voice — ENTER toggles listening
    # -----------------------------------------------------------------------

    def keyboard_toggle_loop(self):
        while not rospy.is_shutdown():
            try:
                input()
            except EOFError:
                time.sleep(0.5)
                continue
            with self._voice_lock:
                self.voice_active = not self.voice_active
                state = "ON  🎙️" if self.voice_active else "OFF 🔇"
                print(f"[Voice] Listening {state}")

    def voice_listener_loop(self):
        speech = LiveSpeech(lm=False, jsgf=GRAMMAR_FILE, dic=DICT_FILE)
        for phrase in speech:
            if rospy.is_shutdown():
                break
            with self._voice_lock:
                active = self.voice_active
            if not active:
                continue
            word  = str(phrase).strip().lower()
            score = phrase.score()
            if word:
                self.process_command(word, score, source="voice")

    # -----------------------------------------------------------------------
    # Main loop
    # -----------------------------------------------------------------------

    def run(self):
        threading.Thread(target=self.keyboard_toggle_loop, daemon=True).start()
        threading.Thread(target=self.voice_listener_loop,  daemon=True).start()

        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            gamepad_moving = (self._stick_linear != 0.0 or self._stick_angular != 0.0)

            # Safety auto-stop for voice teleop
            if (not self.navigating
                    and not self.is_stopped
                    and not gamepad_moving
                    and time.time() - self.last_cmd_time > CMD_TIMEOUT):
                print(f"[Safety] No command for {CMD_TIMEOUT}s. Stopping.")
                self.target_linear  = 0.0
                self.target_angular = 0.0
                self.is_stopped     = True

            if not self.navigating:
                if gamepad_moving:
                    # Sticks have priority over voice teleop
                    self.twist_msg.linear.x  = self._stick_linear  * self.linear_speed
                    self.twist_msg.angular.z = self._stick_angular * self.angular_speed
                    self.last_cmd_time       = time.time()
                else:
                    self.twist_msg.linear.x  = self.target_linear
                    self.twist_msg.angular.z = self.target_angular
                self.vel_pub.publish(self.twist_msg)

            rate.sleep()

    # -----------------------------------------------------------------------
    # Cleanup
    # -----------------------------------------------------------------------

    def cleanup(self):
        print("[System] Shutting down...")
        self.cancel_navigation()
        self.twist_msg.linear.x  = 0.0
        self.twist_msg.angular.z = 0.0
        self.vel_pub.publish(self.twist_msg)


# ---------------------------------------------------------------------------
if __name__ == '__main__':
    try:
        node = VoiceNavMuseum()
        node.run()
    except rospy.ROSInterruptException:
        pass
