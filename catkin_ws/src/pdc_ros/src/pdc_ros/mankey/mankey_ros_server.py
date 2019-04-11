# system
import os
import time

# ros
import rospy
import actionlib
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from PIL import Image

# pdc_ros
from pdc_ros.utils.utils import *
import pdc_ros.utils.perception_utils as perception_utils

# pdc_ros_msgs
import pdc_ros_msgs.msg

import pdc_ros.utils.utils as pdc_ros_utils

# pdc
import dense_correspondence_manipulation.utils.utils as pdc_utils

pdc_utils.add_dense_correspondence_to_python_path()
dc_source_dir = pdc_utils.getDenseCorrespondenceSourceDir()
from dense_correspondence_manipulation.mankey_utils.mankey_client import ManKeyClient


class ManKeyROSServer(object):

    def __init__(self, config=None):
        self._config = config
        self._client = ManKeyClient(config)
        self._setup_ros_actions()

    def _setup_ros_actions(self):
        """
        Initializes the ROS actions. They get started when you call `run()`
        :return:
        :rtype:
        """

        self._action_server = actionlib.SimpleActionServer("KeypointDetection", pdc_ros_msgs.msg.KeypointDetectionAction,
                                                           execute_cb=self._on_keypoint_detection_action,
                                                           auto_start=False)

        self._save_rgbd_msg_server = actionlib.SimpleActionServer("SaveRGBD", pdc_ros_msgs.msg.KeypointDetectionAction,
                                                           execute_cb=self._on_save_rgbd_msg,
                                                           auto_start=False)

    def _on_keypoint_detection_action(self, goal):
        rospy.loginfo("\n\n-------Received KeypointDetectionAction request-------")
        start_time = time.time()

        image_data_list = []
        for msg in goal.rgbd_with_pose_list:
            image_data_list.append(perception_utils.parse_RGBD_with_pose(msg))


        if goal.output_dir == "":
            output_dir = None
        else:
            output_dir = os.path.join(pdc_utils.get_sandbox_dir(), goal.output_dir)

        rospy.loginfo("output_dir %s" %(output_dir))

        num_detected_objects, output_dir = self._client.run_on_images(image_data_list, output_dir=output_dir)

        if num_detected_objects == 0:
            msg = "Mask RCNN detected 0 objects, aborting"
            rospy.loginfo(msg)
            self._action_server.set_aborted(text=msg)
            rospy.loginfo("------Aborted KeypointDetectionAction request-------\n\n")
            return

        sandbox_dir = pdc_ros_utils.get_sandbox_dir()
        relpath_to_sandbox_dir = os.path.relpath(output_dir, sandbox_dir)

        result = pdc_ros_msgs.msg.KeypointDetectionResult()
        result.output_dir = relpath_to_sandbox_dir

        self._action_server.set_succeeded(result)

        elapsed = time.time() - start_time
        rospy.loginfo("------Completed KeypointDetectionAction request in %.2f seconds-------\n\n" %(elapsed))

    def _on_save_rgbd_msg(self, goal):
        """
        Saves RGBD to file
        :param goal:
        :type goal:
        :return:
        :rtype:
        """
        rospy.loginfo("\n\n-------Received SaveRGBDAction request-------")

        output_dir = os.path.join(pdc_utils.get_sandbox_dir(), goal.output_dir)

        assert len(goal.rgbd_with_pose_list) == 1
        msg = goal.rgbd_with_pose_list[0]
        rospy.loginfo("output_dir %s" %(output_dir))
        perception_utils.save_RGBD_with_pose(msg, output_dir)

        result = pdc_ros_msgs.msg.KeypointDetectionResult()
        self._save_rgbd_msg_server.set_succeeded(result)

        rospy.loginfo("\n\n-------Completed SaveRGBDAction request-------")

    def run(self):
        """
        Start the node
        """
        self._action_server.start()
        self._save_rgbd_msg_server.start()

    @staticmethod
    def make_default():
        return ManKeyROSServer()




