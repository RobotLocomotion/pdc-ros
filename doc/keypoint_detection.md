# Keypoint Detection

## Streaming Keypoint Detection
 
This section explains how to run the streaming keypoint detector.

1. Start ROS on `spartan`, make sure images are streaming.
2. Inside `pdc-ros` docker container run `use_all`.
2. Edit variables at the top of `cd ~/code/catkin_ws/src/pdc_ros/keypoint_detection_ros.py`. The main variable
to set is `KEYPOINT_DETECTION_CONFIG_FILE` which specificies what the keypoints are, and which network to use.
3. Launch keypoint detection node.

```
use_all
cd ~/code/catkin_ws/src/pdc_ros/nodes
python keypoint_detection_node.py
```

4. In another terminal launch `rviz` to visualize the results
```
use_all
cd ~/code/catkin_ws/src/pdc_ros/config
rosrun rviz rviz keypoint_vis.rviz
```
