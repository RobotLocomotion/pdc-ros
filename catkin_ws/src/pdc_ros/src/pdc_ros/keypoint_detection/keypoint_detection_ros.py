# system
from PIL import Image as PILImage
import numpy as np

# ros
import rospy
import sensor_msgs.msg

# pdc
from dense_correspondence_manipulation.keypoint_detection.keypoint_detection import KeypointDetection

# pdc-ros
from pdc_ros.utils.simple_subscriber import SimpleSubscriber
import pdc_ros.utils.utils as utils


DEBUG = True

class KeypointDetectionROS(object):

    def __init__(self, keypoint_detection, config):
        """
        :param config:
        :type config:
        :param keypoint_detection:
        :type keypoint_detection: KeypointDetection object
        """
        self._keypoint_detection = keypoint_detection
        self._config = config
        self._debug = DEBUG
        self._setup_subscribers()
        self._setup_publishers()

    def _setup_subscribers(self):
        """
        Setup the ROS subscribers to get the RGB images
        :return:
        :rtype:
        """
        rgb_image_topic = self._config['rgb_image_topic']
        self._image_subscriber = SimpleSubscriber(rgb_image_topic, sensor_msgs.msg.Image,
                                                  externalCallback=self._on_rgb_image)

        self._image_subscriber.start()

    def _setup_publishers(self):
        """
        Sets up the ROS publishers
        :return:
        :rtype:
        """
        self._keypoint_image_pub = rospy.Publisher("keypoints", sensor_msgs.msg.Image, queue_size=1)


    def _on_rgb_image(self, msg):
        """
        Callback for when an rgb message is received
        :param msg:
        :type msg:
        :return:
        :rtype:
        """

        if self._debug:
            print "received RGB image"


        rgb_img_numpy = utils.convert_ros_image_to_numpy_image(msg)
        rgb_img_PIL = PILImage.fromarray(rgb_img_numpy)

        rgb_img_tensor = self._keypoint_detection.dataset.rgb_image_to_tensor(rgb_img_PIL)

        res_numpy = self._keypoint_detection.dcn.forward_single_image_tensor(rgb_img_tensor).data.cpu().numpy()
        rgb_img_w_keypoints = np.copy(rgb_img_numpy)

        kp_detections = self._keypoint_detection._detect_keypoints(res_numpy)
        self._keypoint_detection._visualize_keypoints(rgb_img_w_keypoints, kp_detections,
                                                      copy_image=False)

        # publish results
        rgb_img_w_keypoints_ros = utils.convert_numpy_image_to_ros_image(rgb_img_w_keypoints)
        self._keypoint_image_pub.publish(rgb_img_w_keypoints_ros)


    @staticmethod
    def default_config():
        """
        Makes a default config file for this node
        :return: dict
        :rtype:
        """

        config = dict()
        config['rgb_image_topic'] = "/camera_carmine_1/rgb/image_rect_color"

        return config


    @staticmethod
    def make_default():
        """
        Makes a default object
        :return:
        :rtype:
        """
        keypoint_detection = KeypointDetection.make_default()
        config = KeypointDetectionROS.default_config()
        return KeypointDetectionROS(keypoint_detection, config)


