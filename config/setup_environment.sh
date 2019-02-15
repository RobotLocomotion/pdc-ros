
function use_ros()
{
	source /opt/ros/kinetic/setup.bash
    echo "using ros"
}

export -f use_ros

function use_pdc_ros()
{
    source $PDC_ROS_SOURCE_DIR/catkin_ws/devel/setup.bash
    use_pytorch_dense_correspondence
    echo "using pdc_ros"
}

export -f use_pdc_ros

function use_all()
{
	use_ros
    use_pdc_ros
    use_pytorch_dense_correspondence
    use_drake
    use_director
    echo "using pdc and ros"
}

export -f use_all