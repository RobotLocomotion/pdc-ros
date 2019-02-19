import os

from threading import Event

# ros
import rospy
import actionlib
import ros_numpy
import geometry_msgs


# pdc_ros_msgs
import pdc_ros_msgs.msg
import pdc_ros.utils.utils as pdc_ros_utils


# pdc
from dense_correspondence_manipulation.category_manipulation.category_manipulation import CategoryManipulationWrapper
from dense_correspondence_manipulation.keypoint_detection.keypoint_detection_type import KeypointDetectionType
import dense_correspondence_manipulation.keypoint_detection.utils as keypoint_utils
from dense_correspondence_manipulation.category_manipulation.category_manipulation_type import CategoryManipulationType
import dense_correspondence_manipulation.utils.utils as pdc_utils
from dense_correspondence_manipulation.category_manipulation.grasping import CategoryGraspPlanner

import director.vtkNumpy as vnp
import director.transformUtils as transformUtils
import director.filterUtils as filterUtils
from director.debugVis import DebugData



IMAGE_NAME = "image_1"

class CategoryManipulationROSServer(object):

    def __init__(self, use_director=True, config=None, category_config=None, mug_rack_config=None):

        self._use_director = use_director
        self._config = config
        self._mug_rack_config = mug_rack_config


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

        self._mug_on_rack_action_server = actionlib.SimpleActionServer("MugOnRackManipulation",
                                                                                 pdc_ros_msgs.msg.MugOnRackManipulationAction,
                                                                                 execute_cb=self._on_mug_on_rack_manipulation_action,
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
        self._mug_on_rack_action_server.start()
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

    def _on_mug_on_rack_manipulation_action(self, goal):
        """
        Runs the mug manipulation action
        :param goal:
        :type goal:
        :return:
        :rtype:
        """

        print "\n\n-------Received Mug Rack Manipulation Action Request----------\n\n"

        # if string is not empty
        keypoint_detection_type = None
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


        rack_config = self._mug_rack_config
        mug_rack_pose_name = self._config["mug_rack_pose_name"]
        rack_pose_dict = rack_config['poses'][mug_rack_pose_name]
        self._solution = None

        def solve_function():
            self._solution = \
                self._category_manipulation_wrapper.mug_on_rack(output_dir,
                                                                object_name,
                                                                image_name,
                                                                target_pose_dict,
                                                                rack_pose_dict,
                                                                rack_config)

            # notify the main thread that solution has been found
            self._threading_event.set()


        self.taskRunner.callOnMain(solve_function)

        print "waiting on event"
        self._threading_event.wait()





        result = pdc_ros_msgs.msg.MugOnRackManipulationResult()

        T_goal_obs = self._solution['T_goal_obs']  # 4x4 homogeneous transform
        result.T_goal_obs = ros_numpy.msgify(geometry_msgs.msg.Pose, T_goal_obs)

        # do the grasping
        d = DebugData()
        for rgbd_with_pose in goal.rgbd_with_pose_list:
            pointcloud_msg = rgbd_with_pose.point_cloud
            T_world_pointcloud, _ = pdc_ros_utils.homogeneous_transform_from_transform_msg(rgbd_with_pose.point_cloud_pose.transform)
            T_world_pointcloud_vtk = transformUtils.getTransformFromNumpy(T_world_pointcloud)

            pointcloud_numpy = pdc_ros_utils.numpy_from_pointcloud2_msg(pointcloud_msg)
            pointcloud_vtk = vnp.getVtkPolyDataFromNumpyPoints(pointcloud_numpy)
            pointcloud_vtk = filterUtils.transformPolyData(pointcloud_vtk, T_world_pointcloud_vtk)

            d.addPolyData(pointcloud_vtk)

        fused_cloud = d.getPolyData()
        grasp_planner = CategoryGraspPlanner()
        kp_container = self._solution["kp_container"]
        T_world_grasp_vtk = grasp_planner.plan_mug_on_rack_grasp(fused_cloud, kp_container, visualize=True, task_runner=self.taskRunner)

        T_world_grasp = transformUtils.getNumpyFromTransform(T_world_grasp_vtk)
        result.T_world_gripper_fingertip = ros_numpy.msgify(geometry_msgs.msg.Pose, T_world_grasp)
        result.T_pre_goal_obs = ros_numpy.msgify(geometry_msgs.msg.Pose, self._solution["T_pre_goal_obs"])

        print("finished making message")
        print("result", result)


        self._mug_on_rack_action_server.set_succeeded(result)

        # launch the visualization
        if self._use_director:
            def vis_function():
                self._category_manip_vis._clear_visualization()
                self._category_manip_vis.load_synthetic_background()
                self._category_manip_vis.load_mug_rack_and_side_table()
                self._category_manip_vis.visualize_result(self._solution, output_dir=output_dir,
                                                          keypoint_detection_type=keypoint_detection_type)

            self.taskRunner.callOnMain(vis_function)

        print "\n\n-------Mug Rack Manipulation Action Finished----------\n\n"

    @staticmethod
    def make_shoe_default(**kwargs):
        """
        Make default server for shoes
        :return:
        :rtype:
        """

        shoe_config = CategoryManipulationWrapper.get_shoe_config()
        return CategoryManipulationROSServer(category_config=shoe_config)











