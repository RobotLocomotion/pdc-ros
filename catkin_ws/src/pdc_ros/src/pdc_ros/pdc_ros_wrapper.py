#! /usr/bin/env python
import os
import numpy as np

# ROS imports
import rospy
import actionlib

# pdc_ros_msgs
import pdc_ros_msgs.msg

class PDCRos(object):

	def __init__(self):
		pass

	def run(self):
		rospy.loginfo("staring PDCRos")
		self._setup_ros_actions()
		rospy.spin()

	def _setup_ros_actions(self):
		"""
		Initializes the ros actions
		"""
		self._find_best_match_action_server = actionlib.SimpleActionServer("FindBestMatch", pdc_ros_msgs.msg.FindBestMatchAction, execute_cb=self._on_find_best_match_action, auto_start=False)

		self._find_best_match_action_server.start()
		rospy.loginfo("finished setting up Actions")


	def _on_find_best_match_action(self, goal):
		print "received FindBestMatchAction request"
		return


