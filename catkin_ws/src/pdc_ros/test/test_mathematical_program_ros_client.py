# ros
import rospy
import actionlib

# pdc_ros
import pdc_ros_msgs

def test_mathematical_program_ros_client():
    category_manipulation_name = "/CategoryManipulation"
    category_manip_client = actionlib.SimpleActionClient(category_manipulation_name,
                                                            pdc_ros_msgs.msg.CategoryManipulationAction)

    goal = pdc_ros_msgs.msg.CategoryManipulationGoal()
    goal.output_dir = "mankey"
    goal.keypoint_detection_type = "mankey"

    rospy.loginfo("waiting for CategoryManip server")

    category_manip_client.wait_for_server()
    rospy.loginfo("connected to CategoryManip server")

    category_manip_client.send_goal(goal)
    category_manip_client.wait_for_result()
    result = category_manip_client.get_result()


if __name__ == "__main__":
    test_mathematical_program_ros_client()