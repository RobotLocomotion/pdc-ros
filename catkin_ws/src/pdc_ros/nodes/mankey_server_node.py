# system
import os

# ROS imports
import rospy
from pdc_ros.mankey.mankey_ros_server import ManKeyROSServer

import dense_correspondence_manipulation.utils.utils as pdc_utils

# shoes
MANKEY_CONFIG_FILE = os.path.join(pdc_utils.getDenseCorrespondenceSourceDir(), 'config/mankey/mankey.yaml')

# mugs
# MANKEY_CONFIG_FILE = os.path.join(pdc_utils.getDenseCorrespondenceSourceDir(), 'config/mankey/mugs.yaml')

CONFIG = pdc_utils.getDictFromYamlFilename(MANKEY_CONFIG_FILE)

if __name__ == "__main__":
    rospy.init_node("ManKey")
    mankey_ros = ManKeyROSServer(CONFIG)
    mankey_ros.run()
    rospy.loginfo("starting to spin")
    rospy.spin()