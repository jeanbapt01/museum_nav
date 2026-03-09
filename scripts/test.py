#!/usr/bin/env python3
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import time

def test_speed():
    rospy.init_node('speed_test')
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    
    # Remplace par des coordonnées valides dans ton monde Gazebo
    goal.target_pose.pose.position.x = 2.0 
    goal.target_pose.pose.position.y = 0.5
    goal.target_pose.pose.orientation.w = 1.0

    print("Top chrono ! C'est parti.")
    start_time = time.time()
    
    client.send_goal(goal)
    client.wait_for_result()
    
    end_time = time.time()
    duration = end_time - start_time
    
    if client.get_state() == actionlib.GoalStatus.SUCCEEDED:
        print(f"Arrivé ! Temps de trajet : {duration:.2f} secondes")
    else:
        print("Échec du trajet.")

if __name__ == '__main__':
    test_speed()
