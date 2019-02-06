# ROS imports
import rospy

from pdc_ros.mankey.mankey_ros_server import ManKeyROSServer

if __name__ == "__main__":
    rospy.init_node("ManKey")
    mankey_ros = ManKeyROSServer.make_default()
    mankey_ros.run()
    rospy.loginfo("starting to spin")
    rospy.spin()