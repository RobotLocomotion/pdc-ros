# ros
import ros_numpy

# pdc_ros
import pdc_ros.utils.utils as pdc_ros_utils

# director
import director.vtkNumpy as vnp
import director.transformUtils as transformUtils
import director.filterUtils as filterUtils
from director.debugVis import DebugData

def vtk_poly_data_from_RGBD_with_pose_list(rgbd_with_pose_list):
    """

    :param rgbd_with_pose_list:
    :type rgbd_with_pose_list:
    :return:
    :rtype:
    """
    d = DebugData()
    for rgbd_with_pose in rgbd_with_pose_list:

        pointcloud_msg = rgbd_with_pose.point_cloud
        T_world_pointcloud = ros_numpy.numpify(rgbd_with_pose.point_cloud_pose.transform)
        T_world_pointcloud_vtk = transformUtils.getTransformFromNumpy(T_world_pointcloud)

        pointcloud_numpy = pdc_ros_utils.numpy_from_pointcloud2_msg(pointcloud_msg)
        pointcloud_vtk = vnp.getVtkPolyDataFromNumpyPoints(pointcloud_numpy)
        pointcloud_vtk = filterUtils.transformPolyData(pointcloud_vtk, T_world_pointcloud_vtk)

        d.addPolyData(pointcloud_vtk)

    return d.getPolyData()

