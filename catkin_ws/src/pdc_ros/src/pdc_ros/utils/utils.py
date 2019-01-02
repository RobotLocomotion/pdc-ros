# system
import os

def get_pdc_ros_source_dir():
    return os.getenv("PDC_ROS_SOURCE_DIR")

def get_pdc_source_dir():
    return os.getenv("DC_SOURCE_DIR")

def get_config_directory():
    return os.path.join(get_pdc_ros_source_dir(), "config")

def get_data_dir():
    return os.getenv("DATA_DIR")