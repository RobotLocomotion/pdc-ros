# system
from PIL import Image as PILImage
import numpy as np
import threading
import cv2
import matplotlib.pyplot as plt
import os

# ros
import rospy
import sensor_msgs.msg

# pdc
from dense_correspondence_manipulation.keypoint_detection.keypoint_detection import KeypointDetection
import dense_correspondence_manipulation.utils.utils as dc_utils

# pdc-ros
from pdc_ros.utils.simple_subscriber import SimpleSubscriber
import pdc_ros.utils.utils as utils


from cv_bridge import CvBridge, CvBridgeError

cv_bridge = CvBridge()

DEBUG = False

DC_SOURCE_DIR = utils.getDenseCorrespondenceSourceDir()
KEYPOINT_CONFIG_FILE = os.path.join(utils.get_config_directory(), 'shoe_keypoints.yaml')
EVAL_CONFIG_FILENAME = os.path.join(utils.get_config_directory(), 'evaluation.yaml')

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
        self._initialize()
        self._setup_subscribers()
        self._setup_publishers()
        self._setup_threads()

    def _initialize(self):
        """
        Initializes a few useful class variables
        :return:
        :rtype:
        """

        # note this is in bgr8 format
        self._reference_image_w_keypoints_bgr = self._keypoint_detection._visualize_reference_image()
        self._reference_image_w_keypoints_ros = cv_bridge.cv2_to_imgmsg(self._reference_image_w_keypoints_bgr, encoding='bgr8')


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
        self._keypoint_image_pub = rospy.Publisher("~keypoints", sensor_msgs.msg.Image, queue_size=1)
        self._reference_image_pub = rospy.Publisher("~reference_image", sensor_msgs.msg.Image, queue_size=1)

    def _setup_threads(self):
        self._publish_reference_image_thread = threading.Thread(target=self._publish_reference_image)


    def _publish_reference_image(self, hz=1.0):
        """
        Publishes out the reference image. Hits the target rate by including a sleep
        :return:
        :rtype:
        """

        while True:
            if self._debug:
                print "\n--------publishing reference image----------\n"
            self._reference_image_pub.publish(self._reference_image_w_keypoints_ros)
            rospy.sleep(1.0/hz)


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

        rgb_img_numpy = cv_bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        rgb_img_PIL = PILImage.fromarray(rgb_img_numpy)
        
        
        rgb_img_tensor = self._keypoint_detection.dataset.rgb_image_to_tensor(rgb_img_PIL)

        res_numpy = self._keypoint_detection.dcn.forward_single_image_tensor(rgb_img_tensor).data.cpu().numpy()
        rgb_img_w_keypoints = np.copy(rgb_img_numpy)

        kp_detections = self._keypoint_detection._detect_keypoints(res_numpy)
        cv2_img_w_keypoints = cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self._keypoint_detection._visualize_keypoints(cv2_img_w_keypoints, kp_detections,
                                                      copy_image=False)

        # publish results
        rgb_img_w_keypoints_ros = cv_bridge.cv2_to_imgmsg(cv2_img_w_keypoints, encoding="bgr8")
        self._keypoint_image_pub.publish(rgb_img_w_keypoints_ros)



    def run(self):
        """
        Starts self._publish_reference_image_thread
        :return:
        :rtype:
        """
        self._publish_reference_image_thread.start()

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

        keypoint_config = utils.getDictFromYamlFilename(CONFIG_FILE)
        eval_config = utils.getDictFromYamlFilename(EVAL_CONFIG_FILENAME)
        keypoint_detection = KeypointDetection(keypoint_config, eval_config)

        keypoint_ros_config = KeypointDetectionROS.default_config()
        return KeypointDetectionROS(keypoint_detection, keypoint_ros_config)


