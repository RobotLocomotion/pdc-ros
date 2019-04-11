# ROS imports
import rospy

from pdc_ros.poser.poser_ros_server import PoserROSServer

if __name__ == "__main__":
    rospy.init_node("poser")
    poser_ros = PoserROSServer.make_default()
    poser_ros.run()
    rospy.loginfo("starting to spin")
    rospy.spin()