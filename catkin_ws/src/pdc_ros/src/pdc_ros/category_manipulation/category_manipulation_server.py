import os

from threading import Event

# ros
import rospy
import actionlib

# pdc_ros_msgs
import pdc_ros_msgs.msg
import pdc_ros.utils.utils as pdc_ros_utils

# pdc
from dense_correspondence_manipulation.category_manipulation.category_manipulation import CategoryManipulationWrapper
from dense_correspondence_manipulation.keypoint_detection.keypoint_detection_type import KeypointDetectionType
import dense_correspondence_manipulation.keypoint_detection.utils as keypoint_utils
from dense_correspondence_manipulation.category_manipulation.category_manipulation_type import CategoryManipulationType
import dense_correspondence_manipulation.utils.utils as pdc_utils



IMAGE_NAME = "image_1"
CATEGORY_MANIPULATION_CONFIG_FILE = os.path.join(pdc_utils.getDenseCorrespondenceSourceDir(), 'config/category_manipulation/shoes_mankey.yaml')
CATEGORY_MANIPULATION_CONFIG = pdc_utils.getDictFromYamlFilename(CATEGORY_MANIPULATION_CONFIG_FILE)

class CategoryManipulationROSServer(object):

    def __init__(self, use_director=True, config=None, category_config=None):

        self._use_director = use_director
        self._config = config


        assert category_config is not None
        self._category_config = category_config  # mugs, shoes etc.

        self._threading_event = Event()

        if use_director:
            from dense_correspondence_manipulation.category_manipulation.category_manipulation_visualizer import CategoryManipulationVisualizer

            from director.taskrunner import TaskRunner


            self._category_manip_vis = CategoryManipulationVisualizer(category_config=self._category_config)
            self.taskRunner = TaskRunner()

        # self.setup_server()


        self._category_manipulation_type = CategoryManipulationType.from_string(self._config["manipulation_type"])



    def setup_server(self):
        """
        Make sure to do callOnThread if using in conjunction with director
        :return:
        :rtype:
        """

        # setup ROS actions etc.
        self._setup_ros_actions()

    def _setup_ros_actions(self):
        """
        Initializes the ROS actions
        :return:
        :rtype:
        """

        self._category_manipulation_action_server = actionlib.SimpleActionServer("CategoryManipulation",
                                                                 pdc_ros_msgs.msg.CategoryManipulationAction, execute_cb=self._on_category_manipulation_action,
                                                                 auto_start=False)

    def run(self, spin=True):
        """
        Start the node
        :return:
        :rtype:
        """

        print "running node"
        self.setup_server()
        self._category_manipulation_action_server.start()
        print "action server started"

        if spin:
            print "node is spinning"
            rospy.spin()



    def _on_category_manipulation_action(self, goal):
        """
        Call the category_manipulation optimization
        :return:
        :rtype:
        """

        print "\n\n-------Received Category Manipulation Action Request----------\n\n"


        # if string is not empty
        if goal.output_dir:
            output_dir = os.path.join(pdc_ros_utils.get_sandbox_dir(), goal.output_dir)
        elif keypoint_detection_type == KeypointDetectionType.POSER:
            output_dir = os.getenv("POSER_OUTPUT_DIR")
        elif keypoint_detection_type == KeypointDetectionType.MANKEY:
            output_dir = os.getenv("MANKEY_OUTPUT_DIR")

        print "output_dir:", output_dir

        goal_pose_name = self._config["goal_pose_name"]
        target_pose_dict = self._category_config['poses'][goal_pose_name]


        self._category_manipulation_wrapper = CategoryManipulationWrapper(self._category_config)

        self._solution = None
        self.THREAD_SIGNAL = False
        self._threading_event.clear()

        keypoint_detection_type = KeypointDetectionType.from_string(goal.keypoint_detection_type)
        parser = keypoint_utils.make_keypoint_result_parser(output_dir, keypoint_detection_type)
        parser.load_response()
        object_name = parser.get_unique_object_name()
        image_name = IMAGE_NAME

        def solve_function():
            if keypoint_detection_type == KeypointDetectionType.MANKEY:
                self._solution = \
                    self._category_manipulation_wrapper.run_from_mankey_output(output_dir, object_name, image_name,
                                                                               self._category_manipulation_type, target_pose_dict)
            elif keypoint_detection_type == KeypointDetectionType.POSER:
                self._solution = self._category_manipulation_wrapper.run_from_poser_output(output_dir, object_name,
                                                                                            image_name)
            else:
                raise ValueError("keypoint_detection_type must be one of `poser` or `mankey`. It was %s" %(goal.keypoint_detection_type))


            # notify the main thread that solution has been found
            self._threading_event.set()


        self.taskRunner.callOnMain(solve_function)

        print "waiting on event"
        self._threading_event.wait()

        T_goal_obs = self._solution['T_goal_obs'] # 4x4 homogeneous transform

        result = pdc_ros_msgs.msg.CategoryManipulationResult()
        result.T_goal_obs = T_goal_obs.flatten().tolist()

        self._category_manipulation_action_server.set_succeeded(result)

        # launch the visualization
        if self._use_director:

            def vis_function():
                self._category_manip_vis._clear_visualization()
                self._category_manip_vis.load_synthetic_background()
                self._category_manip_vis.visualize_result(self._solution, output_dir=output_dir, keypoint_detection_type=keypoint_detection_type)

            self.taskRunner.callOnMain(vis_function)

        print "\n\n-------Category Manipulation Action Finished----------\n\n"

    @staticmethod
    def make_shoe_default(**kwargs):
        """
        Make default server for shoes
        :return:
        :rtype:
        """

        shoe_config = CategoryManipulationWrapper.get_shoe_config()
        return CategoryManipulationROSServer(category_config=shoe_config)











