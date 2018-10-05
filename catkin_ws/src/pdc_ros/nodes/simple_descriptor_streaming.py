import rospy
from pdc_ros.pdc_ros_simple_streaming import StreamingPdcRos


if __name__ == "__main__":
	rospy.init_node("stream_pytorch_dense_correspondence")
	pdc_ros_wrapper = StreamingPdcRos()
	pdc_ros_wrapper.run()
	
