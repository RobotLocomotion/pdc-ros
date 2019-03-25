# system
import numpy as np
import os
from PIL import Image as PILImage
import cv2


# ROS
from cv_bridge import CvBridge, CvBridgeError
import ros_numpy

# pdc_ros
import pdc_ros.utils.utils as pdc_ros_utils
import dense_correspondence_manipulation.utils.utils as pdc_utils


"""
Note, you cannot use these if running with directorPython. Importing cv2
classes leads to a segfault
"""

cv_bridge = CvBridge()

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
    :return: cv2 image with single channel, uint16, depth is expressed in millimeters
    :rtype:
    """
    cv_img = cv_bridge.imgmsg_to_cv2(msg, "32FC1")
    cv_img = np.array(cv_img, dtype=np.float32)
    cv_img = cv_img * 1000
    cv_img = cv_img.astype(np.uint16)
    return cv_img


def parse_RGBD_with_pose(msg):
    """

    :param msg:
    :type msg: RGBDWithPose
    :return: dict with following fields

    - 'rgb': rgb image of type PIL.Image, BGR encoding
    - 'depth': rgb image of type cv2 image (just numpy array)
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

    d['rgb'] = rgb_img_msg_to_PIL(msg.rgb_image)
    d['depth'] = depth_img_msg_to_PIL(msg.depth_image)
    d['camera_to_world'] = pdc_ros_utils.homogeneous_transform_from_transform_msg(msg.camera_pose.transform)[1]
    d["pointcloud_to_world"] = pdc_ros_utils.homogeneous_transform_from_transform_msg(msg.point_cloud_pose.transform)[1]

    return d


def numpy_from_pointcloud2_msg(msg):
    """

    :param msg: sensor_msgs/PointCloud2
    :type msg:
    :return:
    :rtype:
    """

    pc = ros_numpy.numpify(msg)
    num_points = msg.width * msg.height

    points = np.zeros((num_points, 3))
    points[:, 0] = pc['x'].flatten()
    points[:, 1] = pc['y'].flatten()
    points[:, 2] = pc['z'].flatten()

    return points

def save_RGBD_with_pose(msg, output_dir):
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    image_data = parse_RGBD_with_pose(msg)

    # save images to disk

    rgb_img_filename = "rgb.png"
    depth_img_filename = "depth.png"
    pointcloud_filename = "pointcloud.npy"


    # save rgb image

    data = dict()
    data['rgb_filename'] = rgb_img_filename
    data["depth_filename"] = depth_img_filename
    data["pointcloud_filename"] = pointcloud_filename
    data["camera_to_world"] = image_data["camera_to_world"]
    data["pointcloud_to_world"] = image_data["pointcloud_to_world"]

    # convert to absolute paths for savings
    rgb_img_filename = os.path.join(output_dir, rgb_img_filename)
    depth_img_filename = os.path.join(output_dir, depth_img_filename)
    pointcloud_filename = os.path.join(output_dir, pointcloud_filename)


    # save RGB
    image_data["rgb"].save(rgb_img_filename)

    # save depth
    cv2.imwrite(depth_img_filename, image_data['depth'].astype(np.uint16))

    # save pointcloud
    pointcloud = numpy_from_pointcloud2_msg(msg.point_cloud)
    np.save(pointcloud_filename, pointcloud)


    pdc_utils.saveToYaml(data, os.path.join(output_dir, "data.yaml"))






