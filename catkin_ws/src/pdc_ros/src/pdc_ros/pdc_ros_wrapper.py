#! /usr/bin/env python
import os
import numpy as np

# ROS imports
import rospy
import actionlib

# pdc_ros_msgs
import pdc_ros_msgs.msg

import time
import sys
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from PIL import Image

import dense_correspondence_manipulation.utils.utils as pdc_utils
pdc_utils.add_dense_correspondence_to_python_path()
dc_source_dir = pdc_utils.getDenseCorrespondenceSourceDir()
sys.path.append(os.path.join(dc_source_dir, "dense_correspondence", "correspondence_tools"))
from dense_correspondence.evaluation.evaluation import DenseCorrespondenceEvaluation

import torch

class PDCRos(object):

    def __init__(self):
        self.bridge = None
        self.load_dcn_network()
        pass

    def load_dcn_network(self):
        """
        Loads the DCN.

        Currently just edit this function to change which
        """
        config_filename = os.path.join(pdc_utils.getDenseCorrespondenceSourceDir(), 'config', 
                               'dense_correspondence', 'evaluation', 'evaluation.yaml')
        config = pdc_utils.getDictFromYamlFilename(config_filename)
        default_config = pdc_utils.get_defaults_config()
        pdc_utils.set_cuda_visible_devices([0])
        dce = DenseCorrespondenceEvaluation(config)

        network_name = "caterpillar_standard_params_3"
        self.dcn = dce.load_network_from_config(network_name)
        self.dataset = self.dcn.load_training_dataset()
        print "finished loading dcn"


    def run(self):
        print "new"
        rospy.loginfo("staring PDCRos")
        self._setup_ros_actions()
        A = torch.rand(3)
        print A
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

        print "finding best match for, ", len(goal.rgbd_with_pose_list), " rgbd frames"

        match_found, best_match = self.compute_best_match_from_rgbd_list(goal.rgbd_with_pose_list)


        result = pdc_ros_msgs.msg.FindBestMatchResult()
        result.match_found = match_found
        self._find_best_match_action_server.set_succeeded(result)


    def compute_best_match_from_rgbd_list(self, rgbd_with_pose_list):

        best_index = 0
        best_match_norm_diff = 1e9

        for i in range(len(rgbd_with_pose_list)):

            rgb_image_ros = rgbd_with_pose_list[i].rgb_image
            rgb_image_numpy = self.convert_ros_to_numpy(rgb_image_ros)
            cv2.imshow('img_rgb_'+str(i), rgb_image_numpy[:,:,::-1].copy())
            cv2.waitKey(1000)
            cv2.destroyAllWindows()

            best_match = self.find_best_match_for_single_rgb(rgb_image_numpy, i)

            depth_image_ros = rgbd_with_pose_list[i].depth_image
            depth_image_numpy_uint16 = self.depth_image_to_numpy_uint16(depth_image_ros)

            print type(rgb_image_numpy), "is rgb type"
            print type(depth_image_numpy_uint16), "is depth image type"
            print np.max(depth_image_numpy_uint16), "is max of depth image"

        time.sleep(3)

        return True, [0,0,0]


    def find_best_match_for_single_rgb(self, rgb_image_numpy, iter):
        """
        :param rgb_image_numpy: should be R, G, B [H,W,3]
        """
        rgb_image_pil = Image.fromarray(rgb_image_numpy)
        print "rgb_image_pil", rgb_image_pil

        # compute dense descriptors
        rgb_tensor = self.dataset.rgb_image_to_tensor(rgb_image_pil)

        # these are Variables holding torch.FloatTensors, first grab the data, then convert to numpy
        res = self.dcn.forward_single_image_tensor(rgb_tensor).data.cpu().numpy()

        cv2.imshow('img_res_'+str(iter), res[:,:,::-1].copy())
        cv2.waitKey(1000)
        cv2.destroyAllWindows()

        return None


    def depth_image_to_numpy_uint16(self, depth_image_msg, bridge=None):
        if self.bridge is None:
            self.bridge = CvBridge()
        """
        Parameters:
            depth_image_msg: sensor_msgs.Image
        """

        cv_img = self.bridge.imgmsg_to_cv2(depth_image_msg, "32FC1")
        cv_img = np.array(cv_img, dtype=np.float32)
        cv_img = cv_img*1000
        cv_img = cv_img.astype(np.uint16)

        return cv_img

    def convert_ros_to_numpy(self, rgb_image_ros):
        if self.bridge is None:
            self.bridge = CvBridge()

        cv_image = self.bridge.imgmsg_to_cv2(rgb_image_ros, "bgr8")
        numpy_img = cv_image[:, :, ::-1].copy() # open and convert between BGR and 
        print numpy_img.shape, "is image shape"
        return numpy_img    	



