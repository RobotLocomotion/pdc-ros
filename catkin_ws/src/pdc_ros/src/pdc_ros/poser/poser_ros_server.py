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

# pdc
import dense_correspondence_manipulation.utils.utils as pdc_utils
pdc_utils.add_dense_correspondence_to_python_path()
dc_source_dir = pdc_utils.getDenseCorrespondenceSourceDir()
from dense_correspondence.poser.poser_client import PoserClient



class PoserRosServer(object):

    def __init__(self, config):
        self._config = config
        self._client = PoserClient(visualize=False, config=config)
        self._client.load_network()

    def _setup_ros_actions(self):
        """
        Initializes the ROS actions
        :return:
        :rtype:
        """
        pass

    def _on_deformable_registration_action(self, goal):
        rospy.loginfo("Received DeformableRegistrationAction request")

        image_data_list = []
        for msg in goal.rgbd_with_pose_list:
            image_data_list.append(perception_utils.parse_RGBD_with_pose(msg))



        self._client.run_on_images(image_data_list)




