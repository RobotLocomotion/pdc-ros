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
from dense_correspondence_manipulation.poser.poser_client import PoserClient


# POSER_CONFIG_FILE = os.path.join(pdc_ros_utils.get_config_directory(), 'poser_ros.yaml')
POSER_CONFIG_FILE = os.path.join(pdc_utils.getDenseCorrespondenceSourceDir(), 'config', 'poser', 'poser_ros.yaml')

class PoserROSServer(object):

    def __init__(self, config):
        self._config = config
        self._client = PoserClient(visualize=False, config=config)
        self._client.load_network()
        # self._client.load_segmentation_network()
        self._setup_ros_actions()

    def _setup_ros_actions(self):
        """
        Initializes the ROS actions. They get started when you call `run()`
        :return:
        :rtype:
        """
        
        self._poser_action_server = actionlib.SimpleActionServer("Poser", pdc_ros_msgs.msg.DeformableRegistrationAction, execute_cb=self._on_deformable_registration_action, auto_start=False)
        
        
    def _on_deformable_registration_action(self, goal):
        rospy.loginfo("Received DeformableRegistrationAction request")

        image_data_list = []
        for msg in goal.rgbd_with_pose_list:
            image_data_list.append(perception_utils.parse_RGBD_with_pose(msg))


        output_dir = self._client.run_on_images(image_data_list)

        sandbox_dir = pdc_ros_utils.get_sandbox_dir()
        relpath_to_sandbox_dir = os.path.relpath(output_dir, sandbox_dir)
        
        result = pdc_ros_msgs.msg.DeformableRegistrationResult()
        result.poser_output_folder = relpath_to_sandbox_dir

        self._poser_action_server.set_succeeded(result)

    def run(self):
        """
        Start the node
        """
        self._poser_action_server.start()

    @staticmethod
    def make_default():
        config = pdc_utils.getDictFromYamlFilename(POSER_CONFIG_FILE)
        return PoserROSServer(config)




