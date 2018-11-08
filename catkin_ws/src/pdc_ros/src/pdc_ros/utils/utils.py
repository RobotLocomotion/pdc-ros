#ros
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from PIL import Image as PILImage


cv_bridge = CvBridge()


def convert_ros_image_to_numpy_image(rgb_image_ros):
    """
    Converts a sensor_msgs.msg.Image type to a PIL image
    :param rgb_image_ros:
    :type rgb_image_ros: sensor_msgs.msg.Image
    :return:
    :rtype:
    """

    cv_image = cv_bridge.imgmsg_to_cv2(rgb_image_ros, "bgr8")
    numpy_img = cv_image[:, :, ::-1].copy()  # open and convert between BGR and RGB
    print numpy_img.shape, "is image shape"
    return numpy_img


def convert_numpy_image_to_ros_image(rgb_image_numpy):
    """
    Converts a numpy image [W,H,D] to a ros Image message
    :param rgb_image_numpy: numpy array [W,H,D]
    :type rgb_image_numpy:
    :return: sensor_msgs.msg.Image
    :rtype:
    """
    cv2_img = rgb_image_numpy[:, :, ::-1].copy() # open and convert between BGR and RGB
    ros_image = cv_bridge.cv2_to_imgmsg(cv2_img, encoding="bgr8")
    return ros_image
