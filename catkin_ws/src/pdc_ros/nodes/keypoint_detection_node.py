# ROS imports
import rospy

from pdc_ros.keypoint_detection.keypoint_detection_ros import KeypointDetectionROS

if __name__ == "__main__":
    rospy.init_node("pdc_keypoint_detection")
    keypoint_detection_ros = KeypointDetectionROS.make_default()
    keypoint_detection_ros.run()

    rospy.loginfo("starting to spin")
    rospy.spin()