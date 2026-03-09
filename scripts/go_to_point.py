#!/usr/bin/env python

import rospy
import yaml
import actionlib
import os
import sys
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

POINTS_FILE = os.path.expanduser("~/catkin_ws/src/museum_nav/config/points.yaml")

def move_to_goal(target_name, points_data):
    # Create the Action Client
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    
    print("Waiting for move_base action server...")
    client.wait_for_server() # Waits until the navigation stack is fully ready

    # Create the Goal
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    # Fill in coordinates from YAML
    data = points_data[target_name]
    goal.target_pose.pose.position.x = data["x"]
    goal.target_pose.pose.position.y = data["y"]
    goal.target_pose.pose.orientation.x = data["qx"]
    goal.target_pose.pose.orientation.y = data["qy"]
    goal.target_pose.pose.orientation.z = data["qz"]
    goal.target_pose.pose.orientation.w = data["qw"]

    # Send
    print(f"Moving to {target_name}...")
    client.send_goal(goal)

    # Wait for result
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        return False
    else:
        return client.get_result()

if __name__ == "__main__":
    rospy.init_node("go_to_point_action")

    # Load YAML
    try:
        with open(POINTS_FILE, "r") as f:
            points = yaml.safe_load(f)
    except IOError:
        print("Cannot find points.yaml")
        sys.exit(1)

    print("Where do you want to go?")
    for key in points.keys():
        print(f" - {key}")

    # Input
    try:
        target = raw_input("Insert here: ")
    except NameError:
        target = input("Insert here: ")

    if target in points:
        result = move_to_goal(target, points)
        if result:
            print("ARRIVED at " + target)
    else:
        print("Point not found.")


