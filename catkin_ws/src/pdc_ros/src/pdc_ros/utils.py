import os

def get_pdc_source_dir():
    return os.path.join(os.path.expanduser("~"), "pytorch-dense-correspondence")

def get_config_directory():
    return os.path.join(os.path.expanduser("~"), "config")