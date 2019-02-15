import argparse
import time

import numpy as np

import rospy
import actionlib
import pdc_ros_msgs.msg


def test_category_manipulation_action():
    client = actionlib.SimpleActionClient("CategoryManipulation", pdc_ros_msgs.msg.CategoryManipulationAction)

    print "waiting for server"
    client.wait_for_server()
    print "connected to server"

    goal = pdc_ros_msgs.msg.CategoryManipulationGoal()
    goal.output_dir = "mankey"
    goal.keypoint_detection_type = "mankey"
    print "goal:", goal

    print "sending goal"
    client.send_goal(goal)

    rospy.loginfo("waiting for JointTrajectory action result")
    client.wait_for_result()
    result = client.get_result()
    print "result:", result

if __name__ == "__main__":
    rospy.init_node("test_category_manipulation")
    test_category_manipulation_action()
