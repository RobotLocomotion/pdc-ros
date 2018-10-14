#! /usr/bin/env python
import os
import numpy as np

# ROS imports
import rospy

from utils import *

# pdc_ros_msgs
import pdc_ros_msgs.msg

import time
import sys
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from PIL import Image as PILImage

import dense_correspondence_manipulation.utils.utils as pdc_utils
pdc_utils.add_dense_correspondence_to_python_path()
dc_source_dir = pdc_utils.getDenseCorrespondenceSourceDir()
from dense_correspondence.evaluation.evaluation import DenseCorrespondenceEvaluation
from dense_correspondence.evaluation.plotting import normalize_descriptor

import torch


NETWORK_CONFIG_FILENAME = os.path.join(get_config_directory(), 'trained_networks.yaml')
NETWORK_NAME = "shoes_consistent_M_background_0.500_3"
RGB_TOPIC = "/camera_carmine_1/rgb/image_rect_color"

class StreamingPdcRos(object):

    def __init__(self):
        self.bridge = None
        self.load_dcn_network()

    def load_dcn_network(self):
        """
        Loads the DCN.
        """
        config = pdc_utils.getDictFromYamlFilename(NETWORK_CONFIG_FILENAME)
        defaults_config = pdc_utils.get_defaults_config()
        pdc_utils.set_cuda_visible_devices([0])
        dce = DenseCorrespondenceEvaluation(config)

        self.dcn = dce.load_network_from_config(NETWORK_NAME)
        self.dcn.eval()
        self.dataset = self.dcn.load_training_dataset() # why do we need to do this?
        print "finished loading dcn"

    def run(self):
        print "new"
        rospy.loginfo("starting PDCRos")
        self._setup_ros_subscribers_publishers()
        rospy.spin()

    def rgb_image_callback(self, rgb_image_ros):
        rgb_image_numpy = self.convert_ros_to_numpy(rgb_image_ros)
        rgb_image_pil = PILImage.fromarray(rgb_image_numpy)
        rgb_tensor = self.dataset.rgb_image_to_tensor(rgb_image_pil)
        
        # these are Variables holding torch.FloatTensors, first grab the data, then convert to numpy
        res_numpy = self.dcn.forward_single_image_tensor(rgb_tensor).data.cpu().numpy()
        res_numpy = normalize_descriptor(res_numpy, self.dcn.descriptor_image_stats["mask_image"])
        res_numpy = np.clip(res_numpy, a_min = 0.0, a_max = 1.0)
        res_numpy = 255 * res_numpy
        res_numpy = res_numpy.astype(np.uint8)

        res_ros = self.convert_numpy_to_ros(res_numpy)
        self.image_pub.publish(res_ros)
        
        rospy.loginfo("Published descriptor image")

    def _setup_ros_subscribers_publishers(self):
        rospy.Subscriber(RGB_TOPIC, Image, self.rgb_image_callback, queue_size=1)
        self.image_pub = rospy.Publisher("pdc_dense_descriptors", Image, queue_size=1)

        rospy.loginfo("finished setting up subscribers and publishers")

    def convert_ros_to_numpy(self, rgb_image_ros):
        if self.bridge is None:
            self.bridge = CvBridge()

        cv_image = self.bridge.imgmsg_to_cv2(rgb_image_ros, "bgr8")
        numpy_img = cv_image[:, :, ::-1].copy() # open and convert between BGR and RGB
        print numpy_img.shape, "is image shape"
        return numpy_img 

    def convert_numpy_to_ros(self, rgb_image_numpy):
        if self.bridge is None:
            self.bridge = CvBridge()

        cv2_img = rgb_image_numpy[:, :, ::-1].copy() # open and convert between BGR and RGB 
        ros_image = self.bridge.cv2_to_imgmsg(cv2_img, encoding="bgr8")
        return ros_image