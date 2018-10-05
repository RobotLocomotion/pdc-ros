# pdc-ros

Setup

```
git clone https://github.com/RobotLocomotion/pdc-ros.git
cd pdc-ros
git submodule update --init --recursive
cd docker
./docker_build.py
```

Run a simple node to stream descriptor images

(To configure, edit `config/trained_networks.yaml`, and then `NETWORK_NAME` and `RGB_TOPIC` at the top of `pdc_ros/src/pdc_ros/pdc_ros_simple_streaming.py`)

```
./docker_run.py
use_all
cd ..
cd catkin_ws
catkin_make
use_all
python src/pdc_ros/nodes/simple_descriptor_streaming.py 
```

Run a node to respond to ROS actions to find best descriptor matches

```
./docker_run.py
use_all
cd ..
cd catkin_ws
catkin_make
use_all
python src/pdc_ros/nodes/find_best_match_node.py 
```




