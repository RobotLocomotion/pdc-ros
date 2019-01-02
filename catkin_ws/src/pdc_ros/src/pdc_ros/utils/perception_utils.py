# system
import numpy as np
from PIL import Image as PILImage

# ROS
import cv_bridge
from cv_bridge import CvBridge, CvBridgeError

# pdc_ros
import pdc_ros.utils.utils as pdc_ros_utils

def rgb_img_msg_to_PIL(msg):
    """

    :param msg: sensor_msgs/Image
    :type msg:
    :return: PIL.Image in RGB encoding
    :rtype:
    """
    rgb_img_numpy = cv_bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
    rgb_img_PIL = PILImage.fromarray(rgb_img_numpy)
    return rgb_img_PIL


def depth_img_msg_to_PIL(msg):
    """

    :param msg: sensor_msgs/Image
    :type msg:
    :return: PIL.Image with single channel, uint16, depth is expressed in millimeters
    :rtype:
    """
    cv_img = cv_bridge.imgmsg_to_cv2(msg, "32FC1")
    cv_img = np.array(cv_img, dtype=np.float32)
    cv_img = cv_img * 1000
    cv_img = cv_img.astype(np.uint16)
    depth_img = PILImage.fromarray(cv_img)
    return depth_img


def parse_RGBD_with_pose(msg):
    """

    :param msg:
    :type msg: RGBDWithPose
    :return: dict with following fields

    - 'rgb': rgb image of type PIL.Image, BGR encoding
    - 'depth': rgb image of type PIL.Image
    - 'camera_to_world': itself dict of form

        camera_to_world:
          quaternion:
            w: 0.11955521666256178
            x: -0.7072223465820128
            y: 0.6767424859550267
            z: -0.16602021071908557
          translation:
            x: 0.29710354158229585
            y: -0.008081499080098517
            z: 0.8270976316822616


    :rtype:
    """
    d = dict()

    d['rgb'] = rgb_img_msg_to_PIL(msg)
    d['depth'] = depth_img_msg_to_PIL(msg)
    d['camera_to_world'] = pdc_ros_utils.homogeneous_transform_from_transform_msg(msg)

    return d

