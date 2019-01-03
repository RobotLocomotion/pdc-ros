# system
import os

# pdc
import dense_correspondence_manipulation.utils.utils as pdc_utils

def get_pdc_ros_source_dir():
    return os.getenv("PDC_ROS_SOURCE_DIR")

def get_pdc_source_dir():
    return os.getenv("DC_SOURCE_DIR")

def get_config_directory():
    return os.path.join(get_pdc_ros_source_dir(), "config")

def get_data_dir():
    return os.getenv("DATA_DIR")

def get_sandbox_dir():
    return os.getenv("PDC_ROS_SANDBOX_DIR")

def homogeneous_transform_from_transform_msg(msg):
    """
    Computes 4 x 4 homogeneous transform matrix from a geometry_msgs/Transform
    :param msg: geometry_msgs/Transform
    :return:
    """

    d = dict()
    pos = msg.translation
    d['translation'] = dict()
    d['translation']['x'] = pos.x
    d['translation']['y'] = pos.y
    d['translation']['z'] = pos.z

    quat = msg.rotation
    d['quaternion'] = dict()
    d['quaternion']['w'] = quat.w
    d['quaternion']['x'] = quat.x
    d['quaternion']['y'] = quat.y
    d['quaternion']['z'] = quat.z


    return pdc_utils.homogenous_transform_from_dict(d), d
