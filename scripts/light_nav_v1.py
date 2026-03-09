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
LINEAR_SPEED  = 0.15   # m/s
ANGULAR_SPEED = 0.5    # rad/s
CMD_TIMEOUT   = 8.0    # seconds before teleop auto-stop
FOLLOW_WAIT   = 10.0   # seconds to display each animal description

POINTS_FILE  = os.path.expanduser("~/catkin_ws/src/museum_nav/config/points.yaml")
GRAMMAR_FILE = os.path.expanduser("~/catkin_ws/src/museum_nav/speech/museum.gram")
DICT_FILE    = os.path.expanduser("~/catkin_ws/src/museum_nav/speech/museum.dict")

# Tour order for FOLLOW command
TOUR_ORDER = ["HOME", "SHARK", "WHALE", "DOLPHIN", "SEASNAIL",
              "TURTLE", "OCTOPUS", "SHRIMP", "TUNA", "HOME"]

# ---------------------------------------------------------------------------
# ANIMAL DESCRIPTIONS
# ---------------------------------------------------------------------------
DESCRIPTIONS = {
    "SHARK": """
╔══════════════════════════════════════════════════════════╗
║                         🦈  SHARK                        ║
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
    "WHALE": """
╔══════════════════════════════════════════════════════════╗
║                         🐋  WHALE                        ║
╠══════════════════════════════════════════════════════════╣
║  Diet: Krill and small fish (filter feeding)             ║
║                                                          ║
║  Special abilities:                                      ║
║  • Produces complex songs that travel thousands of km    ║
║  • Blue whale: loudest animal on Earth (180 decibels)    ║
║  • Can hold breath for up to 90 minutes                  ║
╚══════════════════════════════════════════════════════════╝
""",
    "DOLPHIN": """
╔══════════════════════════════════════════════════════════╗
║                       🐬  DOLPHIN                        ║
╠══════════════════════════════════════════════════════════╣
║  Diet: Carnivore — fish, squid, hunts in groups          ║
║                                                          ║
║  Special abilities:                                      ║
║  • Echolocation: locates prey using sound waves          ║
║  • One of the few animals able to recognise itself       ║
║    in a mirror (self-awareness)                          ║
║  • Sleeps with one brain hemisphere at a time            ║
╚══════════════════════════════════════════════════════════╝
""",
    "SEASNAIL": """
╔══════════════════════════════════════════════════════════╗
║                     🐚  SEA SNAIL                        ║
╠══════════════════════════════════════════════════════════╣
║  Diet: Herbivore or carnivore depending on species       ║
║        — algae, molluscs, small invertebrates            ║
║                                                          ║
║  Special abilities:                                      ║
║  • Shell grows in a perfect logarithmic spiral           ║
║  • Cone snails can fire a venomous harpoon-like tooth    ║
║    capable of killing a human                            ║
║  • Some species can seal their shell opening with        ║
║    an operculum to survive out of water                  ║
╚══════════════════════════════════════════════════════════╝
""",
    "TURTLE": """
╔══════════════════════════════════════════════════════════╗
║                     🐢  SEA TURTLE                       ║
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
║                       🐙  OCTOPUS                        ║
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
    "SHRIMP": """
╔══════════════════════════════════════════════════════════╗
║                       🦐  SHRIMP                         ║
╠══════════════════════════════════════════════════════════╣
║  Diet: Omnivore — algae, plankton, organic debris        ║
║                                                          ║
║  Special abilities:                                      ║
║  • Mantis shrimp can punch at 80 km/h, generating        ║
║    a shockwave that stuns or kills prey                  ║
║  • Has 16 types of photoreceptors (humans have 3)        ║
║  • Some species are bioluminescent                       ║
║  • Cleaner shrimp remove parasites from larger fish      ║
╚══════════════════════════════════════════════════════════╝
""",
    "TUNA": """
╔══════════════════════════════════════════════════════════╗
║                         🐟  TUNA                         ║
╠══════════════════════════════════════════════════════════╣
║  Diet: Carnivore — fish, squid, crustaceans              ║
║                                                          ║
║  Special abilities:                                      ║
║  • One of the fastest fish: up to 70 km/h                ║
║  • Warm-blooded: maintains body temperature above        ║
║    surrounding water for better muscle performance       ║
║  • Migrates thousands of kilometres across oceans        ║
╚══════════════════════════════════════════════════════════╝
""",
}

WELCOME_MSG = """
╔══════════════════════════════════════════════════════════╗
║                                                          ║
║        🐠  Welcome to the Marine Wildlife Museum  🐠     ║
║                                                          ║
║   I am your guide robot. Here is what you can ask me:    ║
║                                                          ║
║   • Say a destination to navigate there:                 ║
║     SHARK, WHALE, DOLPHIN, CORAL, TURTLE,                ║
║     OCTOPUS, JELLYFISH, TUNA, HOME                       ║
║                                                          ║
║   • Say FOLLOW for a full guided tour                    ║
║   • Say STOP to cancel any movement                      ║
║   • Say FORWARD / BACK / LEFT / RIGHT to move manually   ║
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
        self.linear_speed  = LINEAR_SPEED
        self.angular_speed = ANGULAR_SPEED
        self.target_linear  = 0.0
        self.target_angular = 0.0
        self.last_cmd_time  = time.time()
        self.is_stopped     = True
        self.navigating     = False
        self.follow_active  = False
        self.twist_msg      = Twist()

        rospy.on_shutdown(self.cleanup)

        print(WELCOME_MSG)

    # -----------------------------------------------------------------------
    # Navigation helpers
    # -----------------------------------------------------------------------

    def _send_goal(self, point_key):
        """Build and send a MoveBaseGoal. Blocks until result. Returns state."""
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
        """Navigate to a single point in a background thread."""
        def _nav():
            self.navigating = True
            print(f"[Nav] Navigating to {point_key}...")
            state = self._send_goal(point_key)
            if state == actionlib.GoalStatus.SUCCEEDED:
                print(f"[Nav] Arrived at {point_key}.")
                if point_key in DESCRIPTIONS:
                    print(DESCRIPTIONS[point_key])
            else:
                print(f"[Nav] Navigation to {point_key} failed or was cancelled.")
            self.navigating = False

        t = threading.Thread(target=_nav, daemon=True)
        t.start()

    def cancel_navigation(self):
        """Cancel any active goal."""
        if self.navigating or self.follow_active:
            self.nav_client.cancel_all_goals()
            self.navigating    = False
            self.follow_active = False
            print("[Nav] Navigation cancelled.")

    def follow_tour(self):
        """Full guided tour in a background thread."""
        def _tour():
            self.follow_active = True
            self.navigating    = True
            print("\n[Tour] Starting full guided tour...")

            for point_key in TOUR_ORDER:
                # Check if cancelled between stops
                if not self.follow_active:
                    print("[Tour] Tour cancelled.")
                    self.navigating = False
                    return

                if point_key == "HOME":
                    print(f"[Tour] Navigating to {point_key}...")
                    state = self._send_goal(point_key)
                    if state == actionlib.GoalStatus.SUCCEEDED:
                        if point_key == TOUR_ORDER[-1]:
                            print("[Tour] Tour complete. Back at HOME.")
                        else:
                            print(f"[Tour] At {point_key}.")
                    else:
                        print(f"[Tour] Could not reach {point_key}. Stopping tour.")
                        break
                else:
                    if not self.follow_active:
                        break
                    print(f"[Tour] Navigating to {point_key}...")
                    state = self._send_goal(point_key)
                    if state != actionlib.GoalStatus.SUCCEEDED:
                        print(f"[Tour] Could not reach {point_key}. Stopping tour.")
                        break

                    # Display description and wait
                    if point_key in DESCRIPTIONS:
                        print(DESCRIPTIONS[point_key])
                    print(f"[Tour] Waiting {int(FOLLOW_WAIT)}s for visitors to read...")
                    # Interruptible wait
                    for _ in range(int(FOLLOW_WAIT * 10)):
                        if not self.follow_active:
                            break
                        time.sleep(0.1)

            self.follow_active = False
            self.navigating    = False

        t = threading.Thread(target=_tour, daemon=True)
        t.start()

    # -----------------------------------------------------------------------
    # Voice processing
    # -----------------------------------------------------------------------

    def process_command(self, word):
        """Act on a single recognised word from Pocketsphinx."""
        print(f"[Voice] Recognised: '{word}'")

        # STOP — always top priority
        if word == "stop":
            self.cancel_navigation()
            self.target_linear  = 0.0
            self.target_angular = 0.0
            self.is_stopped     = True
            print("[Command] >> STOP")
            return

        # WELCOME
        if word == "welcome":
            print(WELCOME_MSG)
            return

        # FOLLOW — guided tour
        if word == "follow":
            if self.navigating or self.follow_active:
                print("[Voice] Already navigating. Say 'stop' first.")
                return
            self.follow_tour()
            return

        # DESTINATIONS
        dest_map = {
            "shark":    "SHARK",
            "whale":    "WHALE",
            "dolphin":  "DOLPHIN",
            "sea snail":"SEASNAIL",
            "turtle":   "TURTLE",
            "octopus":  "OCTOPUS",
            "shrimp":   "SHRIMP",
            "tuna":     "TUNA",
            "home":     "HOME",
        }
        if word in dest_map:
            if self.navigating or self.follow_active:
                print("[Voice] Already navigating. Say 'stop' first.")
                return
            self.navigate_to(dest_map[word])
            return

        # TELEOP — only when not navigating
        if self.navigating or self.follow_active:
            print("[Voice] Teleop ignored: navigation in progress. Say 'stop' to cancel.")
            return

        if word == "forward":
            self.target_linear  =  self.linear_speed
            self.target_angular =  0.0
            self.is_stopped     =  False
            self.last_cmd_time  =  time.time()
            print("[Command] >> FORWARD")
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
        """Background thread: Pocketsphinx LiveSpeech loop."""
        speech = LiveSpeech(
            lm=False,
            jsgf=GRAMMAR_FILE,
            dic=DICT_FILE,
        )
        for phrase in speech:
            if rospy.is_shutdown():
                break
            word = str(phrase).strip().lower()
            if word:
                self.process_command(word)

    # -----------------------------------------------------------------------
    # Main loop
    # -----------------------------------------------------------------------

    def run(self):
        voice_thread = threading.Thread(target=self.voice_listener_loop, daemon=True)
        voice_thread.start()

        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            # Safety timeout: stop teleop if silent for too long
            if (not self.navigating
                    and not self.follow_active
                    and not self.is_stopped
                    and time.time() - self.last_cmd_time > CMD_TIMEOUT):
                print(f"[Safety] No command for {CMD_TIMEOUT}s. Stopping.")
                self.target_linear  = 0.0
                self.target_angular = 0.0
                self.is_stopped     = True

            # Publish velocity only during manual teleop
            if not self.navigating and not self.follow_active:
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