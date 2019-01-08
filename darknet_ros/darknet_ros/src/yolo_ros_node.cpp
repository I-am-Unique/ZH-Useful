/**************************************************************************
 * @author z.h
 * @date 2019.1.7
 * @usage:yolo_ros_node.cpp 
 **************************************************************************/
#include <iostream>
#include <darknet_ros/yolo_ros.hpp>

#include <ros/ros.h>

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "yolo_ros_node");

	yolo_ros *yolo_ros_ = new yolo_ros;

	yolo_ros_ -> init();
	yolo_ros_ -> doListening();
	
	delete yolo_ros_;
	return 0;
}
