import numpy as np
import os
import functools

from threading import Event

# ros
import rospy
import actionlib

# pdc_ros_msgs
import pdc_ros_msgs.msg
import pdc_ros.utils.utils as pdc_ros_utils

# pdc
from dense_correspondence_manipulation.category_manipulation.category_manipulation import CategoryManipulation, CategoryManipulationWrapper

class CategoryManipulationROSServer(object):

    def __init__(self, use_director=True, config=None, category_config=None):

        self._use_director = use_director
        self._config = config
        self._category_config = category_config # mugs, shoes etc.
        self._threading_event = Event()

        if use_director:
            from dense_correspondence_manipulation.category_manipulation.category_manipulation_visualizer import CategoryManipulationVisualizer

            from director.taskrunner import TaskRunner


            self._category_manip_vis = CategoryManipulationVisualizer(category_config=category_config)
            self.taskRunner = TaskRunner()

        # self.setup_server()


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
        poser_output_folder = goal.poser_output_dir

        # if string is not empty
        if poser_output_folder:
            poser_output_folder = os.path.join(pdc_ros_utils.get_sandbox_dir(), goal.poser_output_dir)
        else:
            poser_output_folder = os.getenv("POSER_OUTPUT_DIR")
        #
        print "poser_output_folder:", poser_output_folder

        # import pydrake
        # print "making category manipulation wrapper"
        # category_manipulation_wrapper = CategoryManipulationWrapper(self._category_config)


        self._category_manipulation_wrapper = CategoryManipulationWrapper.make_shoe_default()

        self._solution = None
        self.THREAD_SIGNAL = False

        self._threading_event.clear()
        def solve_function():
            self._solution = self._category_manipulation_wrapper.compute_target_transform_from_poser_output_folder()
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
                self._category_manip_vis.visualize_result(self._solution, poser_output_dir=poser_output_folder)

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
        return CategoryManipulationROSServer(category_config=shoe_config, **kwargs)











