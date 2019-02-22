# system
import os

# ROS imports
import rospy

# pdc_ros
from pdc_ros.category_manipulation.category_manipulation_server import CategoryManipulationROSServer
import dense_correspondence_manipulation.utils.utils as pdc_utils

USE_DIRECTOR = True


###### SHOES
# CATEGORY_CONFIG_FILE = os.path.join(pdc_utils.getDenseCorrespondenceSourceDir(), 'config/category_manipulation/shoes_mankey.yaml')
# TYPE = "SHOE_ON_TABLE"


####### MUGS
CATEGORY_CONFIG_FILE = os.path.join(pdc_utils.getDenseCorrespondenceSourceDir(), 'config/category_manipulation/mug_3_keypoints.yaml')
# TYPE = "MUG_ON_TABLE_3_KEYPOINTS"
# TYPE = "MUG_ON_RACK"
# TYPE = "MUG_ON_TABLE_ROTATION_INVARIANT"
TYPE = "MUG_ON_SHELF_3D"




CATEGORY_CONFIG = pdc_utils.getDictFromYamlFilename(CATEGORY_CONFIG_FILE)

SERVER_CONFIG_FILE = os.path.join(pdc_utils.getDenseCorrespondenceSourceDir(), 'config/category_manipulation/category_manipulation_server.yaml')
SERVER_CONFIG = pdc_utils.getDictFromYamlFilename(SERVER_CONFIG_FILE)[TYPE]

MUG_RACK_CONFIG_FILE = os.path.join(pdc_utils.getDenseCorrespondenceSourceDir(),"config/category_manipulation/mug_rack.yaml")
MUG_RACK_CONFIG = pdc_utils.getDictFromYamlFilename(MUG_RACK_CONFIG_FILE)



MUG_SHELF_CONFIG_FILE = os.path.join(pdc_utils.getDenseCorrespondenceSourceDir(),"config/category_manipulation/mug_platform.yaml")
MUG_SHELF_CONFIG = pdc_utils.getDictFromYamlFilename(MUG_SHELF_CONFIG_FILE)


if __name__ == "__main__":
    rospy.init_node("category_manip")

    category_manip_server = CategoryManipulationROSServer(use_director=USE_DIRECTOR, config=SERVER_CONFIG, category_config=CATEGORY_CONFIG, mug_rack_config=MUG_RACK_CONFIG,
                                                          mug_shelf_config=MUG_SHELF_CONFIG)


    globalsDict = globals()

    if USE_DIRECTOR:
        globalsDict = globals()
        from director import mainwindowapp

        app = mainwindowapp.construct()
        app.gridObj.setProperty('Visible', True)
        app.viewOptions.setProperty('Orientation widget', True)
        app.viewOptions.setProperty('View angle', 30)
        app.sceneBrowserDock.setVisible(True)
        app.propertiesDock.setVisible(False)
        app.mainWindow.setWindowTitle('Category Manip Node')
        app.mainWindow.show()
        app.mainWindow.resize(920, 600)
        app.mainWindow.move(0, 0)

        view = app.view

        globalsDict['app'] = app
        globalsDict['view'] = view

        globalsDict['cms'] = category_manip_server
        globalsDict['cmv'] = category_manip_server._category_manip_vis


        category_manip_server.taskRunner.callOnThread(category_manip_server.run)
        app.app.start()
    else:
        category_manip_server.run(spin=True)
        # rospy.loginfo("starting to spin")
        # rospy.spin()