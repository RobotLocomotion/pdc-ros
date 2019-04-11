#! /usr/bin/env python
import os
import numpy as np

# ROS imports
import rospy
import actionlib

import pdc_ros.utils.utils as pdc_ros_utils
from pdc_ros.utils.utils import *

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


PICK_POINT_CONFIG_FILENAME = "caterpillar_tail.yaml"
# PICK_POINT_CONFIG_FILENAME = "caterpillar_right_ear.yaml"
# PICK_POINT_CONFIG_FILENAME = "shoe_tongue_consistent.yaml"




NETWORK_CONFIG_FILENAME = os.path.join(get_config_directory(), 'trained_networks.yaml')


class PDCRos(object):

    def __init__(self):
        self.bridge = None
        self.load_pick_point_config()
        self.load_dcn_network()
        self.debug_visualize = True
        self.best_match_visualize = True

    def load_pick_point_config(self):
        """
        Loads the config for the pick point. Stores the information on
        - which network to use
        - the descriptor to look for
        """
        config_filename = os.path.join(get_config_directory(), PICK_POINT_CONFIG_FILENAME)

        self.pick_point_config = pdc_utils.getDictFromYamlFilename(config_filename)

    def load_dcn_network(self):
        """
        Loads the DCN.

        Currently just edit this function to change which
        """
        config = pdc_utils.getDictFromYamlFilename(NETWORK_CONFIG_FILENAME)
        defaults_config = pdc_utils.get_defaults_config()
        pdc_utils.set_cuda_visible_devices([0])
        dce = DenseCorrespondenceEvaluation(config)

        network_name = self.pick_point_config["network_name"]
        self.dcn = dce.load_network_from_config(network_name)
        self.dataset = self.dcn.load_training_dataset() # why do we need to do this?
        print "finished loading dcn"


    def run(self):
        print "new"
        rospy.loginfo("starting PDCRos")
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

        print "finding best match for, ", len(goal.rgbd_with_pose_list), " rgbd frames"

        camera_matrix = PDCRos.camera_matrix_from_camera_info_msg(goal.camera_info)
        match_found, best_match_location = self.compute_best_match_from_rgbd_list(goal.rgbd_with_pose_list, camera_matrix)


        result = pdc_ros_msgs.msg.FindBestMatchResult()
        result.match_found = match_found

        if result.match_found:
            result.best_match_location.x = best_match_location[0]
            result.best_match_location.y = best_match_location[1]
            result.best_match_location.z = best_match_location[2]

        self._find_best_match_action_server.set_succeeded(result)


    def compute_best_match_from_rgbd_list(self, rgbd_with_pose_list, camera_matrix):

        best_index = None    # if best_index remains None, this is flag for none found
        best_index_match_uv = None

        if "threshold_norm_diff" in self.pick_point_config:
            threshold_norm_diff = self.pick_point_config["threshold_norm_diff"]
        else:
            threshold_norm_diff = 0.3

        def rescale_depth_image(img):
            max_range = 2000.0
            scaled_img = np.clip(img, 0, max_range)
            return scaled_img/max_range

        for i in range(len(rgbd_with_pose_list)):

            rgb_image_ros = rgbd_with_pose_list[i].rgb_image
            depth_image_ros = rgbd_with_pose_list[i].depth_image
            rgb_image_numpy = self.convert_ros_to_numpy(rgb_image_ros)
            depth_image_numpy = self.depth_image_to_numpy_uint16(depth_image_ros)

            # if self.debug_visualize:
            #     cv2.imshow('img_rgb_'+str(i), rgb_image_numpy[:,:,::-1].copy())
            #     cv2.moveWindow('img_rgb_'+str(i), 0, 0)
            #     cv2.moveWindow('img_rgb_'+str(i), 350, 0+i*320)

            best_match_uv, best_match_diff = self.find_best_match_for_single_rgb(rgb_image_numpy, i)

            depth_is_valid, depth = self.get_depth_value_of_best_match(depth_image_numpy, best_match_uv)

            # if self.debug_visualize:
            #     depth_img_rescaled = rescale_depth_image(depth_image_numpy)
            #     self.draw_best_match(depth_img_rescaled, best_match_uv[0], best_match_uv[1])
            #     cv2.imshow('depth_' + str(i), depth_img_rescaled)
            #     cv2.moveWindow('depth_' + str(i), 0, 0)
            #     cv2.moveWindow('depth_' + str(i), 350, 580)
            #     cv2.waitKey(1000)

            if (best_match_diff < threshold_norm_diff) and depth_is_valid:
                threshold_norm_diff = best_match_diff
                best_index = i
                best_index_match_uv = best_match_uv

            #depth_image_ros = rgbd_with_pose_list[i].depth_image
            #depth_image_numpy_uint16 = self.depth_image_to_numpy_uint16(depth_image_ros)

            print type(rgb_image_numpy), "is rgb type"
            #print type(depth_image_numpy_uint16), "is depth image type"
            #print np.max(depth_image_numpy_uint16), "is max of depth image"

        if best_index is None:
            print "\n\n"
            print "I didnt find any matches below threshold of:"
            print threshold_norm_diff
            return False, None
        else:
            print "\n\n"
            print "My best match was from img index", best_index
            print "The norm diff was", threshold_norm_diff
            print "At pixel (u,v):", best_index_match_uv
            print "Showing again"

            best_match_uv = (best_index_match_uv[0], best_index_match_uv[1])
            rgbd_pose = rgbd_with_pose_list[best_index]
            rgb_image_ros = rgbd_with_pose_list[best_index].rgb_image
            rgb_image_numpy = self.convert_ros_to_numpy(rgb_image_ros)

            depth_image_ros = rgbd_with_pose_list[best_index].depth_image
            depth_image_numpy = self.depth_image_to_numpy_uint16(depth_image_ros)

            # if self.best_match_visualize:
            #     cv2_img = rgb_image_numpy[:,:,::-1].copy()
            #     self.draw_best_match(cv2_img, best_index_match_uv[0], best_index_match_uv[1])
            #     cv2.imshow('img_rgb_best_match_labeled', cv2_img)

            #     depth_img_rescaled = rescale_depth_image(depth_image_numpy)
            #     self.draw_best_match(depth_img_rescaled, best_index_match_uv[0], best_index_match_uv[1])
            #     cv2.imshow('depth_best_match' + str(i), depth_img_rescaled)

            camera_pose = rgbd_pose.camera_pose
            best_match_location = self.compute_3D_location_of_best_match(camera_pose, camera_matrix, depth_image_numpy, best_match_uv)

            print "best match location:", best_match_location
            print "Depth Value at best match", depth_image_numpy[best_match_uv[1], best_match_uv[0]]

            

            # if self.debug_visualize:
            #     while (1):
            #         k = cv2.waitKey(33)
            #         if k == 27:  # Esc key to stop
            #             break
            #         elif k == -1:  # normally -1 returned,so don't print it
            #             continue
            #         else:
            #             print k  # else print its value

            #     cv2.destroyAllWindows()



        return True, best_match_location

    def get_depth_value_of_best_match(self, depth_image, best_match_uv):
        """
        Computes the depth value of the best match
        :param depth_image:
        :param best_match_uv:
        :return:
        """

        MAX_RANGE = 1500
        u = best_match_uv[0]
        v = best_match_uv[1]
        image_height = depth_image.shape[0]
        image_width = depth_image.shape[1]

        depth_actual = depth_image[best_match_uv[1], best_match_uv[0]]

        box_size = 11
        u_min = max(u - box_size/2, 0)
        v_min = max(v - box_size/2, 0)

        u_max = min(u + box_size/2, image_width-1)
        v_max = min(v + box_size/2, image_height-1)

        depth_cropped = depth_image[v_min:(v_max+1), u_min:(u_max+1)]

        if not (depth_cropped > 0).any():
            print "Nothing in depth cropped"
            return False, None

        depth_min = np.min(depth_cropped[depth_cropped > 0])

        print "depth_actual:", depth_actual
        print "depth_min:", depth_min

        if depth_min == 0 or depth_min > MAX_RANGE:
            print "depth INVALID"
            return False, depth_min

        print "depth VALID"
        return True, depth_min

    def compute_3D_location_of_best_match(self, camera_pose, camera_intrinsics_matrix, depth_image, best_match_uv):
        """

        :param camera_pose:
        :type : geometry_msgs/TransformStamped
        :param depth_image:
        :type : numpy image
        :param best_match_uv: tuple (u,v)
        :return:
        """
        DCE = DenseCorrespondenceEvaluation

        _, depth_mm = self.get_depth_value_of_best_match(depth_image, best_match_uv)
        depth = depth_mm/1000.0

        # 4 x 4 numpy array
        camera_to_world, _ = pdc_ros_utils.homogeneous_transform_from_transform_msg(camera_pose.transform)

        pos = DCE.compute_3d_position(best_match_uv, depth, camera_intrinsics_matrix, camera_to_world)
        return pos



    def find_best_match_for_single_rgb(self, rgb_image_numpy, img_num):
        """
        :param rgb_image_numpy: should be R, G, B [H,W,3]
        """
        rgb_image_pil = Image.fromarray(rgb_image_numpy)
        print "rgb_image_pil", rgb_image_pil

        # compute dense descriptors
        rgb_tensor = self.dataset.rgb_image_to_tensor(rgb_image_pil)

        # these are Variables holding torch.FloatTensors, first grab the data, then convert to numpy
        res = self.dcn.forward_single_image_tensor(rgb_tensor).data.cpu().numpy()

        # descriptor_target = self.get_descriptor_target_from_yaml()
        descriptor_target = np.array(self.pick_point_config["descriptor"])

        best_match_uv, best_match_diff, norm_diffs = self.dcn.find_best_match_for_descriptor(descriptor_target, res)

        print "best match diff: ", best_match_diff

        if self.debug_visualize:
            cv2_img = rgb_image_numpy[:,:,::-1].copy()
            self.draw_best_match(cv2_img, best_match_uv[0], best_match_uv[1])
            cv2.imshow('img_rgb_labeled'+str(img_num), cv2_img)
            cv2.moveWindow('img_rgb_labeled'+str(img_num), 0, 0)
            cv2.moveWindow('img_rgb_labeled'+str(img_num), 350, 0+img_num*500)
            cv2.waitKey(1000)
    
        return best_match_uv, best_match_diff

    def get_descriptor_target_from_yaml(self):
        """
        Grabs a 1-dimensional numpy array of length D from the descriptor yaml file
        """
        descriptor_filename = os.path.join(pdc_utils.getDenseCorrespondenceSourceDir(), "../config", "new_descriptor_picked.yaml")
        descriptor_dict = pdc_utils.getDictFromYamlFilename(descriptor_filename)
        descriptor_list = descriptor_dict["descriptor"]
        return np.asarray(descriptor_list)

    def draw_best_match(self, img, x, y):
        white = (255,255,255)
        black = (0,0,0)
        label_color = (0,255,0)
        cv2.circle(img,(x,y),10,label_color,1)
        cv2.circle(img,(x,y),11,white,1)
        cv2.circle(img,(x,y),12,label_color,1)
        cv2.line(img,(x,y+1),(x,y+3),white,1)
        cv2.line(img,(x+1,y),(x+3,y),white,1)
        cv2.line(img,(x,y-1),(x,y-3),white,1)
        cv2.line(img,(x-1,y),(x-3,y),white,1)


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



    @staticmethod
    def camera_matrix_from_camera_info_msg(msg):
        """
        Extracts the camera matrix from a sensor_msgs/CameraInfo ROS message
        :param msg:
        :return: np.array((3,3))
        """
        fx = msg.K[0]
        cx = msg.K[2]

        fy = msg.K[4]
        cy = msg.K[5]

        return np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])
