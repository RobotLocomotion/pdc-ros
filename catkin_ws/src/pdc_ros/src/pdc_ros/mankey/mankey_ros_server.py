# system
import os

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


# # shoes
# MANKEY_CONFIG_FILE = os.path.join(pdc_utils.getDenseCorrespondenceSourceDir(), 'config/mankey/mankey.yaml')


# mugs
MANKEY_CONFIG_FILE = os.path.join(pdc_utils.getDenseCorrespondenceSourceDir(), 'config/mankey/mugs.yaml')

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

    def _on_keypoint_detection_action(self, goal):
        rospy.loginfo("\n\n-------Received KeypointDetectionAction request-------")

        image_data_list = []
        for msg in goal.rgbd_with_pose_list:
            image_data_list.append(perception_utils.parse_RGBD_with_pose(msg))

        num_detected_objects, output_dir = self._client.run_on_images(image_data_list)

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
        rospy.loginfo("------Completed KeypointDetectionAction request-------\n\n")

    def run(self):
        """
        Start the node
        """
        self._action_server.start()

    @staticmethod
    def make_default():
        return ManKeyROSServer()




