#!/usr/bin/env python3

import rospy
import threading
import time
import os
import sys
import yaml
import actionlib
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from pocketsphinx import LiveSpeech

# ---------------------------------------------------------------------------
# CONFIGURATION
# ---------------------------------------------------------------------------
LINEAR_SPEED   = 0.15   # m/s
ANGULAR_SPEED  = 0.5    # rad/s
CMD_TIMEOUT    = 8.0    # seconds before teleop auto-stop
CONFIDENCE_THR = -3000  # log-prob threshold — ignore recognition below this

POINTS_FILE  = os.path.expanduser("~/catkin_ws/src/museum_nav/config/points.yaml")
GRAMMAR_FILE = os.path.expanduser("~/catkin_ws/src/museum_nav/speech/museum.gram")
DICT_FILE    = os.path.expanduser("~/catkin_ws/src/museum_nav/speech/museum.dict")

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
║   I am your guide robot. Here is what you can ask me:   ║
║                                                          ║
║   • Say a destination to navigate there:                 ║
║     SHARK, TURTLE, OCTOPUS, HOME                         ║
║                                                          ║
║   • Say STOP to cancel any movement                      ║
║   • Say GO / BACK / LEFT / RIGHT to move manually        ║
║   • Say FASTER / SLOWER to adjust speed                  ║
║                                                          ║
║              Enjoy your visit !                          ║
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
            print("[Error] Cannot find points.yaml")
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

        rospy.on_shutdown(self.cleanup)
        print(WELCOME_MSG)

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
                # Show animal description if available
                if point_key in DESCRIPTIONS:
                    print(DESCRIPTIONS[point_key])
                # Show welcome message if at WELCOME point
                if point_key == "WELCOME":
                    print(WELCOME_MSG)
            else:
                print(f"[Nav] Navigation to {point_key} failed or was cancelled.")
            self.navigating = False

        t = threading.Thread(target=_nav, daemon=True)
        t.start()

    def cancel_navigation(self):
        if self.navigating:
            self.nav_client.cancel_all_goals()
            self.navigating = False
            print("[Nav] Navigation cancelled.")

    # -----------------------------------------------------------------------
    # Voice processing
    # -----------------------------------------------------------------------

    def process_command(self, word, score):
        # Confidence filter
        if score < CONFIDENCE_THR:
            print(f"[Voice] '{word}' ignored (low confidence: {score})")
            return

        print(f"[Voice] Recognised: '{word}' (score: {score})")

        # STOP — always top priority
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
                print("[Voice] Already navigating. Say 'stop' first.")
                return
            self.navigate_to(dest_map[word])
            return

        # TELEOP — only when not navigating
        if self.navigating:
            print("[Voice] Teleop ignored: navigation in progress. Say 'stop' to cancel.")
            return

        if word == "go":
            self.target_linear  =  self.linear_speed
            self.target_angular =  0.0
            self.is_stopped     =  False
            self.last_cmd_time  =  time.time()
            print("[Command] >> GO")
        elif word == "back":
            self.target_linear  = -self.linear_speed
            self.target_angular =  0.0
            self.is_stopped     =  False
            self.last_cmd_time  =  time.time()
            print("[Command] >> BACK")
        elif word == "left":
            self.target_linear  =  0.0
            self.target_angular =  self.angular_speed
            self.is_stopped     =  False
            self.last_cmd_time  =  time.time()
            print("[Command] >> LEFT")
        elif word == "right":
            self.target_linear  =  0.0
            self.target_angular = -self.angular_speed
            self.is_stopped     =  False
            self.last_cmd_time  =  time.time()
            print("[Command] >> RIGHT")
        elif word == "faster":
            self.linear_speed  = min(self.linear_speed  + 0.03, 0.22)
            self.angular_speed = min(self.angular_speed + 0.1,  2.0)
            print(f"[Command] >> FASTER (linear={self.linear_speed:.2f} m/s)")
        elif word == "slower":
            self.linear_speed  = max(self.linear_speed  - 0.03, 0.05)
            self.angular_speed = max(self.angular_speed - 0.1,  0.2)
            print(f"[Command] >> SLOWER (linear={self.linear_speed:.2f} m/s)")

    def voice_listener_loop(self):
        speech = LiveSpeech(
            lm=False,
            jsgf=GRAMMAR_FILE,
            dic=DICT_FILE,
        )
        for phrase in speech:
            if rospy.is_shutdown():
                break
            word  = str(phrase).strip().lower()
            score = phrase.score()
            if word:
                self.process_command(word, score)

    # -----------------------------------------------------------------------
    # Main loop
    # -----------------------------------------------------------------------

    def run(self):
        voice_thread = threading.Thread(target=self.voice_listener_loop, daemon=True)
        voice_thread.start()

        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            if (not self.navigating
                    and not self.is_stopped
                    and time.time() - self.last_cmd_time > CMD_TIMEOUT):
                print(f"[Safety] No command for {CMD_TIMEOUT}s. Stopping.")
                self.target_linear  = 0.0
                self.target_angular = 0.0
                self.is_stopped     = True

            if not self.navigating:
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