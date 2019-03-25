# system
import os
import numpy as np
import functools
from threading import Event

# ros
import rospy
import actionlib
import ros_numpy
import geometry_msgs


# drake
from pydrake.solvers.mathematicalprogram import SolutionResult

# pdc_ros_msgs
import pdc_ros_msgs.msg
import pdc_ros.utils.utils as pdc_ros_utils
import pdc_ros.utils.director_utils as pdc_ros_director_utils


# pdc
from dense_correspondence_manipulation.category_manipulation.category_manipulation import CategoryManipulationWrapper, CategoryManipulation
from dense_correspondence_manipulation.keypoint_detection.keypoint_detection_type import KeypointDetectionType
import dense_correspondence_manipulation.keypoint_detection.utils as keypoint_utils
from dense_correspondence_manipulation.category_manipulation.category_manipulation_type import CategoryManipulationType
import dense_correspondence_manipulation.utils.utils as pdc_utils
from dense_correspondence_manipulation.category_manipulation.grasping import CategoryGraspPlanner
import dense_correspondence_manipulation.utils.director_utils as director_utils
from dense_correspondence_manipulation.category_manipulation.mug_orientation_classifier import MugOrientationClassifier, MugOrientation
from dense_correspondence_manipulation.category_manipulation.object_locator import ObjectLocator

import director.vtkNumpy as vnp
import director.transformUtils as transformUtils
import director.filterUtils as filterUtils
from director.debugVis import DebugData
import director.visualization as vis



IMAGE_NAME = "image_1"

TOP_CENTER_TARGET_LOCATION = np.array([0.6650140662059443, 0.0642841347255465, 0.020806108674832013])


class CategoryManipulationROSServer(object):

    def __init__(self, use_director=True, config=None, category_config=None, mug_rack_config=None,
                 mug_shelf_config=None, shoe_rack_config=None):

        self._use_director = use_director
        self._config = config
        self._mug_rack_config = mug_rack_config
        self._mug_shelf_config = mug_shelf_config
        self._shoe_rack_config = shoe_rack_config


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

        print("category manipulation type %s" %self._config["manipulation_type"])


    @staticmethod
    def result_file(output_dir):
        return os.path.join(output_dir, "category_manipulation_result.yaml")


    def save_configs(self, output_dir):
        if self._config is not None:
            filename = os.path.join(output_dir, "category_manipulation.yaml")
            pdc_utils.saveToYaml(self._config, filename)

        if self._category_config is not None:
            filename = os.path.join(output_dir, "category_config.yaml")
            pdc_utils.saveToYaml(self._category_config, filename)

        if self._mug_rack_config is not None:
            filename = os.path.join(output_dir, "mug_rack_config.yaml")
            pdc_utils.saveToYaml(self._mug_rack_config, filename)

        if self._mug_shelf_config is not None:
            filename = os.path.join(output_dir, "mug_shelf_config.yaml")
            pdc_utils.saveToYaml(self._mug_shelf_config, filename)

        if self._shoe_rack_config is not None:
            filename = os.path.join(output_dir, "shoe_rack_config.yaml")
            pdc_utils.saveToYaml(self._shoe_rack_config, filename)

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

        # self._mug_on_rack_action_server = actionlib.SimpleActionServer("MugOnRackManipulation",
        #                                                                          pdc_ros_msgs.msg.MugOnRackManipulationAction,
        #                                                                          execute_cb=self._on_mug_on_rack_manipulation_action,
        #                                                                          auto_start=False)

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
        Dispatch to appropriate sub-routine
        :return:
        :rtype:
        """

        # dispatch based on type in our config
        rospy.loginfo("\n\n---- Received action request, dispatching it appropriately----\n\n")
        if self._category_manipulation_type == CategoryManipulationType.SHOE_ON_TABLE:
            self.shoe_on_table(goal)
        elif self._category_manipulation_type == CategoryManipulationType.MUG_ON_TABLE_3_KEYPOINTS:
            self.mug_on_table_3_keypoints(goal)
        elif self._category_manipulation_type == CategoryManipulationType.MUG_ON_RACK:
            self.mug_on_rack(goal)
        elif self._category_manipulation_type == CategoryManipulationType.MUG_ON_SHELF_3D:
            self.mug_on_shelf_3D(goal)
        elif self._category_manipulation_type == CategoryManipulationType.SHOE_ON_RACK:
            self.shoe_on_rack(goal)
        else:
            raise ValueError("unknown manipulation type")

    def _on_mug_on_rack_manipulation_action(self, goal):
        """
        Runs the mug manipulation action
        :param goal:
        :type goal:
        :return:
        :rtype:
        """

        raise ValueError("DEPRECATED METHOD")

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


        # T_mug_rack
        goal_pose_name = self._config["goal_pose_name"]
        T_rack_mug_vtk = director_utils.transformFromPose(self._category_config['poses'][goal_pose_name])


        # T_world_rack
        rack_config = self._mug_rack_config
        mug_rack_pose_name = self._config["mug_rack_pose_name"]
        T_world_rack_vtk = director_utils.transformFromPose(rack_config['poses'][mug_rack_pose_name])
        T_world_rack = transformUtils.getNumpyFromTransform(T_world_rack_vtk)

        # get T_world_mug_model
        T_world_mug_model_vtk = transformUtils.concatenateTransforms([T_rack_mug_vtk, T_world_rack_vtk])
        T_world_mug_model = transformUtils.getNumpyFromTransform(T_world_mug_model_vtk)


        self._category_manipulation_wrapper = CategoryManipulationWrapper(self._category_config)

        self._solution = None
        self.THREAD_SIGNAL = False
        self._threading_event.clear()

        keypoint_detection_type = KeypointDetectionType.from_string(goal.keypoint_detection_type)
        parser = keypoint_utils.make_keypoint_result_parser(output_dir, keypoint_detection_type)
        parser.load_response()
        object_name = parser.get_unique_object_name()
        image_name = IMAGE_NAME



        self._solution = None

        def solve_function():
            self._solution = \
                self._category_manipulation_wrapper.mug_on_rack(output_dir, object_name, image_name, T_world_mug_model, T_world_rack, rack_config)

            # notify the main thread that solution has been found
            self._threading_event.set()


        self.taskRunner.callOnMain(solve_function)

        print "waiting on event"
        self._threading_event.wait()





        result = pdc_ros_msgs.msg.MugOnRackManipulationResult()
        result.category_manipulation_type = CategoryManipulationType.to_string(self._category_manipulation_type)

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
        print("fused_cloud.GetNumberOfPoints()", fused_cloud.GetNumberOfPoints())
        print("fused_cloud.GetNumberOfCells()", fused_cloud.GetNumberOfCells())
        grasp_planner = CategoryGraspPlanner()
        kp_container = self._solution["kp_container"]
        T_world_grasp_vtk, grasp_type = grasp_planner.plan_mug_on_rack_grasp(fused_cloud, kp_container, visualize=True, task_runner=self.taskRunner)

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


    def mug_on_table_3_keypoints(self, goal):
        """
        Dispatch to appropriate sub-routine
        :return:
        :rtype:
        """

        print "\n\n-------Received MUG_ON_TABLE_3_KEYPOINTS Manipulation Action Request----------\n\n"

        # only support MANKEY for now

        # if string is not empty
        keypoint_detection_type = KeypointDetectionType.from_string(goal.keypoint_detection_type)
        # if string is not empty
        if goal.output_dir:
            output_dir = os.path.join(pdc_ros_utils.get_sandbox_dir(), goal.output_dir)
        else:
            output_dir = os.getenv("MANKEY_OUTPUT_DIR")

        print "output_dir:", output_dir

        goal_pose_name = self._config["goal_pose_name"]
        target_pose_dict = self._category_config['poses'][goal_pose_name]
        T_goal_model_vtk = director_utils.transformFromPose(target_pose_dict)
        T_goal_model = transformUtils.getNumpyFromTransform(T_goal_model_vtk)

        self._category_manipulation_wrapper = CategoryManipulationWrapper(self._category_config)

        self._solution = None
        self.THREAD_SIGNAL = False
        self._threading_event.clear()

        keypoint_detection_type = KeypointDetectionType.from_string(goal.keypoint_detection_type)
        parser = keypoint_utils.make_keypoint_result_parser(output_dir, keypoint_detection_type)
        parser.load_response()
        object_name = parser.get_unique_object_name()
        image_name = IMAGE_NAME

        d = self._category_manipulation_wrapper.construct_keypoint_containers_from_mankey_output_dir(output_dir,
                                                                                                     object_name,
                                                                                                     image_name,
                                                                                                     T_goal_model)
        kp_container = d['kp_container']
        goal_kp_container = d['goal_kp_container']
        T_init_goal_obs = d['T_init_goal_obs']

        def solve_function():
            cm = CategoryManipulation()
            mp = cm.construct_keypoint_optimization(kp_container, goal_kp_container, T_init=T_init_goal_obs)

            solver_code = mp.Solve()
            print("solver code:", solver_code)
            sol = cm.parse_solution(mp, T_init=cm.T_init)
            sol['solver_code'] = solver_code
            sol["category_manipulation_type"] = CategoryManipulationType.MUG_ON_TABLE

            self._solution = sol
            self._threading_event.set()

        self.taskRunner.callOnMain(solve_function)

        print "waiting on event"
        self._threading_event.wait()

        T_goal_obs = self._solution['T_goal_obs']  # 4x4 homogeneous transform

        result = pdc_ros_msgs.msg.CategoryManipulationResult()
        result.category_manipulation_type = CategoryManipulationType.to_string(self._category_manipulation_type)
        result.T_goal_obs = ros_numpy.msgify(geometry_msgs.msg.Pose, T_goal_obs)

        succeeded = self._solution['solver_code'] == SolutionResult.kSolutionFound
        if succeeded:
            self._category_manipulation_action_server.set_succeeded(result)
        else:
            self._category_manipulation_action_server.set_aborted(result)

        # launch the visualization
        if self._use_director:
            def vis_function():
                self._category_manip_vis._clear_visualization()
                self._category_manip_vis.load_synthetic_background()
                self._category_manip_vis.visualize_result(self._solution, output_dir=output_dir,
                                                          keypoint_detection_type=keypoint_detection_type)

            self.taskRunner.callOnMain(vis_function)

        print "\n\n-------MUG_ON_TABLE_3_KEYPOINTS Manipulation Action Finished----------\n\n"

    def shoe_on_table(self, goal):
        """
        Run the shoe on table optimization
        :return:
        :rtype:
        """

        print "\n\n-------Received SHOE_ON_TABLE Manipulation Action Request----------\n\n"

        # only support MANKEY for now

        # if string is not empty
        keypoint_detection_type = KeypointDetectionType.from_string(goal.keypoint_detection_type)
        # if string is not empty
        if goal.output_dir:
            output_dir = os.path.join(pdc_ros_utils.get_sandbox_dir(), goal.output_dir)
        else:
            output_dir = os.getenv("MANKEY_OUTPUT_DIR")

        print "output_dir:", output_dir

        goal_pose_name = self._config["goal_pose_name"]
        target_pose_dict = self._category_config['poses'][goal_pose_name]
        T_goal_model_vtk = director_utils.transformFromPose(target_pose_dict)
        T_goal_model = transformUtils.getNumpyFromTransform(T_goal_model_vtk)

        self._category_manipulation_wrapper = CategoryManipulationWrapper(self._category_config)

        self._solution = None
        self.THREAD_SIGNAL = False
        self._threading_event.clear()

        keypoint_detection_type = KeypointDetectionType.from_string(goal.keypoint_detection_type)
        parser = keypoint_utils.make_keypoint_result_parser(output_dir, keypoint_detection_type)
        parser.load_response()
        object_name = parser.get_unique_object_name()
        image_name = IMAGE_NAME

        d = self._category_manipulation_wrapper.construct_keypoint_containers_from_mankey_output_dir(output_dir, object_name, image_name, T_goal_model)

        kp_container = d['kp_container']
        goal_kp_container = d['goal_kp_container']
        T_init_goal_obs = d['T_init_goal_obs']

        def solve_function():
            cm = CategoryManipulation()
            mp = cm.construct_keypoint_optimization(kp_container, goal_kp_container, T_init=T_init_goal_obs)

            solver_code = mp.Solve()
            print("solver code:", solver_code)
            sol = cm.parse_solution(mp, T_init=cm.T_init)
            sol['solver_code'] = solver_code
            sol["category_manipulation_type"] = CategoryManipulationType.SHOE_ON_TABLE

            self._solution = sol
            self._threading_event.set()


        self.taskRunner.callOnMain(solve_function)

        print "waiting on event"
        self._threading_event.wait()

        T_goal_obs = self._solution['T_goal_obs']  # 4x4 homogeneous transform
        T_goal_obs_vtk = transformUtils.getTransformFromNumpy(T_goal_obs)


        # encode goal pose
        result = pdc_ros_msgs.msg.CategoryManipulationResult()
        result.category_manipulation_type = CategoryManipulationType.to_string(self._category_manipulation_type)
        result.T_goal_obs = ros_numpy.msgify(geometry_msgs.msg.Pose, T_goal_obs)

        # approach pose
        xyz = (0, 0, 0.1)  # 10 cm
        quat = (1, 0, 0, 0)
        T = transformUtils.transformFromPose(xyz, quat)
        T_pre_goal_obs_vtk = transformUtils.concatenateTransforms([T_goal_obs_vtk, T])
        T_pre_goal_obs = transformUtils.getNumpyFromTransform(T_pre_goal_obs_vtk)
        self._solution['T_pre_goal_obs'] = T_pre_goal_obs
        result.T_pre_goal_obs = ros_numpy.msgify(geometry_msgs.msg.Pose, T_pre_goal_obs)
        result.T_pre_goal_obs_valid = True



        # grasp pose
        grasp_planner = CategoryGraspPlanner()
        T_world_grasp_vtk = grasp_planner.plan_shoe_vertical_grasp(kp_container)

        T_world_grasp = transformUtils.getNumpyFromTransform(T_world_grasp_vtk)
        result.T_world_gripper_fingertip = ros_numpy.msgify(geometry_msgs.msg.Pose, T_world_grasp)
        result.T_world_gripper_fingertip_valid = True
        result.gripper_width = 0.05

        succeeded = self._solution['solver_code'] == SolutionResult.kSolutionFound
        if succeeded:
            self._category_manipulation_action_server.set_succeeded(result)
        else:
            self._category_manipulation_action_server.set_aborted(result)

        # launch the visualization
        if self._use_director:
            def vis_function():
                self._category_manip_vis._clear_visualization()
                self._category_manip_vis.load_synthetic_background()
                self._category_manip_vis.visualize_result(self._solution, output_dir=output_dir,
                                                          keypoint_detection_type=keypoint_detection_type)

                vis.showFrame(T_world_grasp_vtk, "gripper fingertip", scale=0.15, parent=self._category_manip_vis._vis_container)

            self.taskRunner.callOnMain(vis_function)

        print "\n\n-------SHOE_ON_TABLE Manipulation Action Finished----------\n\n"

    def shoe_on_rack(self, goal):
        """
        Run the shoe on table optimization
        :return:
        :rtype:
        """

        print "\n\n-------Received SHOE_ON_TABLE Manipulation Action Request----------\n\n"

        # only support MANKEY for now

        # if string is not empty
        keypoint_detection_type = KeypointDetectionType.from_string(goal.keypoint_detection_type)
        # if string is not empty
        if goal.output_dir:
            output_dir = os.path.join(pdc_ros_utils.get_sandbox_dir(), goal.output_dir)
        else:
            output_dir = os.getenv("MANKEY_OUTPUT_DIR")

        print "output_dir:", output_dir


        # T_rack_model
        goal_pose_name = self._config["goal_pose_name"]
        target_pose_dict = self._category_config['poses'][goal_pose_name]
        T_rack_model_vtk = director_utils.transformFromPose(target_pose_dict)


        # T_world_rack
        rack_config = self._shoe_rack_config
        rack_pose_name = self._config["rack_pose_name"]
        T_world_rack_vtk = director_utils.transformFromPose(rack_config["poses"][rack_pose_name])

        # T_goal_model
        T_goal_model_vtk = transformUtils.concatenateTransforms([T_rack_model_vtk, T_world_rack_vtk])

        if goal.apply_T_adjust:
            print("applying T_adjust")
            T_adjust = ros_numpy.numpify(goal.T_adjust)
            T_adjust_vtk = transformUtils.getTransformFromNumpy(T_adjust)
            T_goal_model_vtk = transformUtils.concatenateTransforms([T_goal_model_vtk, T_adjust_vtk])


        T_goal_model = transformUtils.getNumpyFromTransform(T_goal_model_vtk)


        self._category_manipulation_wrapper = CategoryManipulationWrapper(self._category_config)

        self._solution = None
        self.THREAD_SIGNAL = False
        self._threading_event.clear()

        keypoint_detection_type = KeypointDetectionType.from_string(goal.keypoint_detection_type)
        parser = keypoint_utils.make_keypoint_result_parser(output_dir, keypoint_detection_type)
        parser.load_response()
        object_name = parser.get_unique_object_name()
        image_name = IMAGE_NAME

        d = self._category_manipulation_wrapper.construct_keypoint_containers_from_mankey_output_dir(output_dir, object_name, image_name, T_goal_model)

        kp_container = d['kp_container']
        goal_kp_container = d['goal_kp_container']
        T_init_goal_obs = d['T_init_goal_obs']

        # extract information about plane
        plane_normal = np.array(rack_config['plane']['normal'])
        plane_normal = T_world_rack_vtk.TransformVector(plane_normal)

        point_on_plane = np.array(rack_config['plane']["point_on_plane"])
        point_on_plane = T_world_rack_vtk.TransformPoint(point_on_plane)
        b = -np.dot(plane_normal, point_on_plane)

        plane_equation = dict()
        plane_equation['A'] = plane_normal
        plane_equation['b'] = b

        cm = CategoryManipulation(plane_equation=plane_equation)
        mp = cm.construct_keypoint_optimization(kp_container, goal_kp_container, T_init=T_init_goal_obs)

        def solve_function():


            solver_code = mp.Solve()
            print("solver code:", solver_code)
            sol = cm.parse_solution(mp, T_init=cm.T_init)
            sol['solver_code'] = solver_code
            sol["category_manipulation_type"] = CategoryManipulationType.SHOE_ON_TABLE

            self._solution = sol
            self._threading_event.set()


        self.taskRunner.callOnMain(solve_function)

        print "waiting on event"
        self._threading_event.wait()

        T_goal_obs = self._solution['T_goal_obs']  # 4x4 homogeneous transform
        T_goal_obs_vtk = transformUtils.getTransformFromNumpy(T_goal_obs)




        # encode goal pose
        result = pdc_ros_msgs.msg.CategoryManipulationResult()
        result.category_manipulation_type = CategoryManipulationType.to_string(self._category_manipulation_type)
        result.T_goal_obs = ros_numpy.msgify(geometry_msgs.msg.Pose, T_goal_obs)



        # approach pose
        xyz = (0, 0, 0.15)  # 5 cm
        quat = (1, 0, 0, 0)
        T = transformUtils.transformFromPose(xyz, quat)
        T_pre_goal_obs_vtk = transformUtils.concatenateTransforms([T_goal_obs_vtk, T])
        T_pre_goal_obs = transformUtils.getNumpyFromTransform(T_pre_goal_obs_vtk)
        self._solution['T_pre_goal_obs'] = T_pre_goal_obs
        result.T_pre_goal_obs = ros_numpy.msgify(geometry_msgs.msg.Pose, T_pre_goal_obs)
        result.T_pre_goal_obs_valid = True



        # grasp pose
        fused_cloud = pdc_ros_director_utils.vtk_poly_data_from_RGBD_with_pose_list(goal.rgbd_with_pose_list)
        grasp_planner = CategoryGraspPlanner()
        T_world_grasp_vtk = grasp_planner.plan_shoe_vertical_grasp(kp_container, poly_data=fused_cloud, taskRunner=self.taskRunner)

        T_world_grasp = transformUtils.getNumpyFromTransform(T_world_grasp_vtk)
        result.T_world_gripper_fingertip = ros_numpy.msgify(geometry_msgs.msg.Pose, T_world_grasp)
        result.T_world_gripper_fingertip_valid = True
        result.gripper_width = 0.05

        succeeded = self._solution['solver_code'] == SolutionResult.kSolutionFound
        if succeeded:
            self._category_manipulation_action_server.set_succeeded(result)
        else:
            self._category_manipulation_action_server.set_aborted(result)

        # launch the visualization
        if self._use_director:
            def vis_function():
                self._category_manip_vis._clear_visualization()
                self._category_manip_vis.load_synthetic_background()
                self._category_manip_vis.load_shoe_rack()

                self._category_manip_vis.visualize_result_mankey(self._solution, output_dir=output_dir,
                                                                 T_goal_model=T_goal_model_vtk,
                                                                 object_name=object_name, image_name=image_name)

                vis.showFrame(T_world_grasp_vtk, "gripper fingertip", scale=0.15, parent=self._category_manip_vis._vis_container)

            self.taskRunner.callOnMain(vis_function)


        # save the results to a file
        # additional things we want to save out

        yaml_dict = dict()

        yaml_dict["type"] = CategoryManipulationType.to_string(self._category_manipulation_type)
        yaml_dict["solve_succeeded"] = succeeded

        yaml_dict["T_goal_obs"] = pdc_utils.dict_from_transform(T_goal_obs)
        yaml_dict["T_pre_goal_obs"] = pdc_utils.dict_from_transform(T_pre_goal_obs)
        yaml_dict["T_world_grasp"] = pdc_utils.dict_from_transform(T_world_grasp)
        yaml_dict["gripper_width"] = result.gripper_width

        T_world_rack = transformUtils.getNumpyFromTransform(T_world_rack_vtk)
        yaml_dict["T_world_rack"] = pdc_utils.dict_from_transform(T_world_rack)



        filename = CategoryManipulationROSServer.result_file(output_dir)
        pdc_utils.saveToYaml(yaml_dict, filename)

        self.save_configs(output_dir)

        print "\n\n-------SHOE_ON_TABLE Manipulation Action Finished----------\n\n"

    def mug_on_table_rotation_invariant(self, goal):
        """
                Dispatch to appropriate sub-routine
                :return:
                :rtype:
                """

        print "\n\n-------Received MUG_ON_TABLE_ROTATION_INVARIANT Manipulation Action Request----------\n\n"

        # only support MANKEY for now

        # if string is not empty
        keypoint_detection_type = KeypointDetectionType.from_string(goal.keypoint_detection_type)
        # if string is not empty
        if goal.output_dir:
            output_dir = os.path.join(pdc_ros_utils.get_sandbox_dir(), goal.output_dir)
        else:
            output_dir = os.getenv("MANKEY_OUTPUT_DIR")

        print "output_dir:", output_dir

        goal_pose_name = self._config["goal_pose_name"]
        target_pose_dict = self._category_config['poses'][goal_pose_name]
        T_goal_model_vtk = director_utils.transformFromPose(target_pose_dict)
        T_goal_model = transformUtils.getNumpyFromTransform(T_goal_model_vtk)

        self._category_manipulation_wrapper = CategoryManipulationWrapper(self._category_config)

        self._solution = None
        self.THREAD_SIGNAL = False
        self._threading_event.clear()

        keypoint_detection_type = KeypointDetectionType.from_string(goal.keypoint_detection_type)
        parser = keypoint_utils.make_keypoint_result_parser(output_dir, keypoint_detection_type)
        parser.load_response()
        object_name = parser.get_unique_object_name()
        image_name = IMAGE_NAME

        d = self._category_manipulation_wrapper.construct_keypoint_containers_from_mankey_output_dir(output_dir,
                                                                                                     object_name,
                                                                                                     image_name,
                                                                                                     T_goal_model)

        kp_container = d['kp_container']
        goal_kp_container = d['goal_kp_container']
        T_init_goal_obs = d['T_init_goal_obs']

        def solve_function():
            cm = CategoryManipulation()
            target_vector = np.array([0, 0, 1])
            mp = cm.construct_mug_on_table_rotation_invariant(kp_container, goal_kp_container,
                                                              target_vector=target_vector, T_init=T_init_goal_obs)

            solver_code = mp.Solve()
            print("solver code:", solver_code)
            sol = cm.parse_solution(mp, T_init=cm.T_init)
            sol['solver_code'] = solver_code
            sol["category_manipulation_type"] = CategoryManipulationType.SHOE_ON_TABLE

            self._solution = sol
            self._threading_event.set()

        self.taskRunner.callOnMain(solve_function)

        print "waiting on event"
        self._threading_event.wait()

        T_goal_obs = self._solution['T_goal_obs']  # 4x4 homogeneous transform

        result = pdc_ros_msgs.msg.CategoryManipulationResult()
        result.category_manipulation_type = CategoryManipulationType.to_string(self._category_manipulation_type)
        result.T_goal_obs = ros_numpy.msgify(geometry_msgs.msg.Pose, T_goal_obs)

        succeeded = self._solution['solver_code'] == SolutionResult.kSolutionFound
        if succeeded:
            self._category_manipulation_action_server.set_succeeded(result)
        else:
            self._category_manipulation_action_server.set_aborted(result)

        # launch the visualization
        if self._use_director:
            def vis_function():
                self._category_manip_vis._clear_visualization()
                self._category_manip_vis.load_synthetic_background()
                self._category_manip_vis.visualize_result(self._solution, output_dir=output_dir,
                                                          keypoint_detection_type=keypoint_detection_type)

            self.taskRunner.callOnMain(vis_function)

        print "\n\n-------MUG_ON_TABLE_ROTATION_INVARIANT Manipulation Action Finished----------\n\n"

    def mug_on_rack(self, goal):
        """
                Runs the mug manipulation action
                :param goal:
                :type goal:
                :return:
                :rtype:
                """

        print "\n\n-------Received MUG_ON_RACK Manipulation Action Request----------\n\n"

        # if string is not empty
        keypoint_detection_type = KeypointDetectionType.from_string(goal.keypoint_detection_type)
        if goal.output_dir:
            output_dir = os.path.join(pdc_ros_utils.get_sandbox_dir(), goal.output_dir)
        elif keypoint_detection_type == KeypointDetectionType.POSER:
            output_dir = os.getenv("POSER_OUTPUT_DIR")
        elif keypoint_detection_type == KeypointDetectionType.MANKEY:
            output_dir = os.getenv("MANKEY_OUTPUT_DIR")

        print "output_dir:", output_dir

        # T_mug_rack
        goal_pose_name = self._config["goal_pose_name"]
        T_rack_mug_vtk = director_utils.transformFromPose(self._category_config['poses'][goal_pose_name])

        # T_world_rack
        rack_config = self._mug_rack_config
        mug_rack_pose_name = self._config["mug_rack_pose_name"]
        T_world_rack_vtk = director_utils.transformFromPose(rack_config['poses'][mug_rack_pose_name])
        T_world_rack = transformUtils.getNumpyFromTransform(T_world_rack_vtk)

        # get T_world_mug_model
        T_world_mug_model_vtk = transformUtils.concatenateTransforms([T_rack_mug_vtk, T_world_rack_vtk])
        T_world_mug_model = transformUtils.getNumpyFromTransform(T_world_mug_model_vtk)

        self._category_manipulation_wrapper = CategoryManipulationWrapper(self._category_config)

        self._solution = None
        self.THREAD_SIGNAL = False
        self._threading_event.clear()

        keypoint_detection_type = KeypointDetectionType.from_string(goal.keypoint_detection_type)
        parser = keypoint_utils.make_keypoint_result_parser(output_dir, keypoint_detection_type)
        parser.load_response()

        list_of_objects = parser.get_object_names()
        image_name = parser.get_image_names_for_object(list_of_objects[0])[0]

        object_locator = ObjectLocator(parser, self._category_config)
        (object_name, distance_to_target) = object_locator.locate_closest_object(image_name, "top_center",
                                                                                 TOP_CENTER_TARGET_LOCATION)


        self._solution = None

        def solve_function():
            self._solution = \
                self._category_manipulation_wrapper.mug_on_rack(output_dir, object_name, image_name, T_world_mug_model,
                                                                T_world_rack, rack_config)

            # notify the main thread that solution has been found
            self._threading_event.set()

        self.taskRunner.callOnMain(solve_function)

        print "waiting on event"
        self._threading_event.wait()

        result = pdc_ros_msgs.msg.CategoryManipulationResult()
        result.category_manipulation_type = CategoryManipulationType.to_string(self._category_manipulation_type)


        T_goal_obs = self._solution['T_goal_obs']  # 4x4 homogeneous transform
        result.T_goal_obs = ros_numpy.msgify(geometry_msgs.msg.Pose, T_goal_obs)


        fused_cloud = pdc_ros_director_utils.vtk_poly_data_from_RGBD_with_pose_list(goal.rgbd_with_pose_list)

        grasp_planner = CategoryGraspPlanner()
        kp_container = self._solution["kp_container"]
        T_world_grasp_vtk, grasp_type = grasp_planner.plan_mug_on_rack_grasp(fused_cloud, kp_container, visualize=True,
                                                                             task_runner=self.taskRunner)

        T_world_grasp = transformUtils.getNumpyFromTransform(T_world_grasp_vtk)
        result.T_world_gripper_fingertip = ros_numpy.msgify(geometry_msgs.msg.Pose, T_world_grasp)
        result.T_world_gripper_fingertip_valid = True
        result.gripper_width = 0.05

        result.T_pre_goal_obs = ros_numpy.msgify(geometry_msgs.msg.Pose, self._solution["T_pre_goal_obs"])
        result.T_pre_goal_obs_valid = True

        print("finished making message")
        print("result", result)

        succeeded = self._solution['solver_code'] == SolutionResult.kSolutionFound
        if succeeded:
            self._category_manipulation_action_server.set_succeeded(result)
        else:
            self._category_manipulation_action_server.set_aborted(result)

        # launch the visualization
        if self._use_director:
            def vis_function():
                self._category_manip_vis._clear_visualization()
                self._category_manip_vis.load_synthetic_background()
                self._category_manip_vis.load_mug_rack_and_side_table()

                self._category_manip_vis.visualize_result_mankey(self._solution, output_dir=output_dir,
                                                                 T_goal_model=T_world_mug_model_vtk,
                                                                 object_name=object_name, image_name=image_name)

            self.taskRunner.callOnMain(vis_function)

        print "\n\n-------MUG_ON_RACK Manipulation Action Finished----------\n\n"

    def mug_on_shelf_3D(self, goal):
        """

        :param goal:
        :type goal:
        :return:
        :rtype:
        """
        print "\n\n-------Received MUG_ON_SHELF_3D Manipulation Action Request----------\n\n"

        # if string is not empty
        keypoint_detection_type = KeypointDetectionType.from_string(goal.keypoint_detection_type)
        if goal.output_dir:
            output_dir = os.path.join(pdc_ros_utils.get_sandbox_dir(), goal.output_dir)
        elif keypoint_detection_type == KeypointDetectionType.POSER:
            output_dir = os.getenv("POSER_OUTPUT_DIR")
        elif keypoint_detection_type == KeypointDetectionType.MANKEY:
            output_dir = os.getenv("MANKEY_OUTPUT_DIR")

        print "output_dir:", output_dir

        # T_mug_rack
        goal_pose_name = self._config["goal_pose_name"]
        T_shelf_mug_vtk = director_utils.transformFromPose(self._category_config['poses'][goal_pose_name])

        # T_world_shelf
        shelf_config = self._mug_shelf_config
        mug_rack_pose_name = self._config["shelf_pose_name"]
        T_world_shelf_vtk = director_utils.transformFromPose(shelf_config['poses'][mug_rack_pose_name])
        T_world_shelf = transformUtils.getNumpyFromTransform(T_world_shelf_vtk)

        # get T_world_mug_model
        T_world_mug_model_vtk = transformUtils.concatenateTransforms([T_shelf_mug_vtk, T_world_shelf_vtk])
        T_world_mug_model = transformUtils.getNumpyFromTransform(T_world_mug_model_vtk)

        self._category_manipulation_wrapper = CategoryManipulationWrapper(self._category_config)



        keypoint_detection_type = KeypointDetectionType.from_string(goal.keypoint_detection_type)
        parser = keypoint_utils.make_keypoint_result_parser(output_dir, keypoint_detection_type)
        parser.load_response()

        list_of_objects = parser.get_object_names()
        image_name = parser.get_image_names_for_object(list_of_objects[0])[0]

        object_locator = ObjectLocator(parser, self._category_config)
        (object_name, distance_to_target) = object_locator.locate_closest_object(image_name, "top_center", TOP_CENTER_TARGET_LOCATION)



        d = self._category_manipulation_wrapper.construct_keypoint_containers_from_mankey_output_dir(output_dir, object_name, image_name,T_world_mug_model)

        kp_container = d['kp_container']
        goal_kp_container = d['goal_kp_container']
        T_init_goal_obs = d['T_init_goal_obs']

        self._solution = None
        self.THREAD_SIGNAL = False
        self._threading_event.clear()

        mug_orientation = MugOrientationClassifier.classify_mug_orientation(kp_container)
        print("mug_orientation: ", mug_orientation)
        grasp_planner = CategoryGraspPlanner()

        pointcloud_vtk = pdc_ros_director_utils.vtk_poly_data_from_RGBD_with_pose_list(goal.rgbd_with_pose_list)
        gripper_width = 0.1 # full width
        if mug_orientation == MugOrientation.HORIZONTAL:
            print("MugOrientation == HORIZONTAL")
            gripper_width = 0.1 # fully open
            T_W_gripper_fingertip = grasp_planner.plan_mug_horizontal_grasp(kp_container)
        elif mug_orientation == MugOrientation.UPRIGHT:
            print("MugOrientation == VERTICAL")
            T_W_gripper_fingertip, _ = grasp_planner.plan_mug_on_rack_grasp(pointcloud_vtk, kp_container,
                                                                            visualize=False, task_runner=False)
            gripper_width = 0.05
        else:
            raise ValueError("Can only handle HORIZONTAL or UPRIGHT orientations for now")



        # extract information about plane
        plane_normal = np.array(self._mug_shelf_config['plane']['normal'])
        plane_normal = T_world_shelf_vtk.TransformVector(plane_normal)

        point_on_plane = np.array(self._mug_shelf_config['plane']["point_on_plane"])
        point_on_plane = T_world_shelf_vtk.TransformPoint(point_on_plane)
        b = -np.dot(plane_normal, point_on_plane)


        plane_equation = dict()
        plane_equation['A'] = plane_normal
        plane_equation['b'] = b


        target_vector = np.array([0,0,1])
        target_vector = np.array([0, 0, 1])
        cm = CategoryManipulation(plane_equation=plane_equation)
        mp = cm.construct_mug_on_table_rotation_invariant(kp_container, goal_kp_container, target_vector, T_init=T_init_goal_obs)


        # add some additional costs if doing a horizontal grasp
        # needed for kinematics reasons
        if mug_orientation == MugOrientation.HORIZONTAL:
            # add a cost
            grasp_x_axis = np.array(T_W_gripper_fingertip.TransformVector([1,0,0]))
            grasp_x_axis = grasp_x_axis.reshape((1, 3))
            target_vector = np.array([0, 1, 0]) # could be relative to shelf pose
            target_vector = target_vector.reshape((1, 3))
            weight = 1.0
            cost_fun = functools.partial(cm.vector_l2_cost, grasp_x_axis, target_vector, weight)
            mp.AddCost(cost_fun, cm._xyz_rpy)

        elif mug_orientation == MugOrientation.UPRIGHT:
            # add a cost
            grasp_z_axis = np.array(T_W_gripper_fingertip.TransformVector([0, 0, 1]))
            grasp_z_axis = grasp_z_axis.reshape((1, 3))
            target_vector = np.array([0, 1, 0])  # could be relative to shelf pose
            target_vector = target_vector.reshape((1, 3))
            weight = 1.0
            cost_fun = functools.partial(cm.vector_l2_cost, grasp_z_axis, target_vector, weight)
            mp.AddCost(cost_fun, cm._xyz_rpy)


        def solve_function():
            solver_code = mp.Solve()

            print("solver code:", solver_code)
            sol = cm.parse_solution(mp, T_init=cm.T_init)
            sol['solver_code'] = solver_code
            sol["category_manipulation_type"] = CategoryManipulationType.MUG_ON_SHELF_3D

            self._solution = sol
            # notify the main thread that solution has been found
            self._threading_event.set()


        self.taskRunner.callOnMain(solve_function)

        print "waiting on event"
        self._threading_event.wait()

        result = pdc_ros_msgs.msg.CategoryManipulationResult()
        result.mug_orientation = MugOrientation.to_string(mug_orientation)
        result.category_manipulation_type = CategoryManipulationType.to_string(self._category_manipulation_type)

        T_goal_obs = self._solution['T_goal_obs']  # 4x4 homogeneous transform
        T_goal_obs_vtk = transformUtils.getTransformFromNumpy(T_goal_obs)
        result.T_goal_obs = ros_numpy.msgify(geometry_msgs.msg.Pose, T_goal_obs)





        T_W_gripepr_fingertip_numpy = transformUtils.getNumpyFromTransform(T_W_gripper_fingertip)
        result.T_world_gripper_fingertip = ros_numpy.msgify(geometry_msgs.msg.Pose, T_W_gripepr_fingertip_numpy)
        result.T_world_gripper_fingertip_valid = True
        result.gripper_width = gripper_width

        xyz = (0,0,0.1) # 10 cm
        quat = (1,0,0,0)
        T = transformUtils.transformFromPose(xyz, quat)
        T_pre_goal_obs_vtk = transformUtils.concatenateTransforms([T_goal_obs_vtk, T])
        T_pre_goal_obs = transformUtils.getNumpyFromTransform(T_pre_goal_obs_vtk)
        self._solution['T_pre_goal_obs'] = T_pre_goal_obs
        result.T_pre_goal_obs = ros_numpy.msgify(geometry_msgs.msg.Pose, T_pre_goal_obs)
        result.T_pre_goal_obs_valid = True

        # set finger tip width

        print("finished making message")
        print("result", result)

        succeeded = self._solution['solver_code'] == SolutionResult.kSolutionFound
        if succeeded:
            self._category_manipulation_action_server.set_succeeded(result)
        else:
            self._category_manipulation_action_server.set_aborted(result)

        # launch the visualization
        if self._use_director:
            def vis_function():
                self._category_manip_vis._clear_visualization()
                self._category_manip_vis.load_synthetic_background()
                self._category_manip_vis.load_mug_platform(T_world_shelf_vtk)
                self._category_manip_vis.visualize_result_mankey(self._solution, output_dir=output_dir, T_goal_model=T_world_mug_model_vtk, object_name=object_name, image_name=image_name)

                vis.showFrame(T_W_gripper_fingertip, "gripper fingertip frame", parent=self._category_manip_vis._vis_container, scale=0.15)

            self.taskRunner.callOnMain(vis_function)

        print "\n\n-------MUG_ON_RACK Manipulation Action Finished----------\n\n"

    @staticmethod
    def make_shoe_default(**kwargs):
        """
        Make default server for shoes
        :return:
        :rtype:
        """

        shoe_config = CategoryManipulationWrapper.get_shoe_config()
        return CategoryManipulationROSServer(category_config=shoe_config)











