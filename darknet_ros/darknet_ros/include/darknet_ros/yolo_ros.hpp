/**************************************************************************
 * @author z.h
 * @date 2019.1.7
 * @usage:yolo_ros.h 
 **************************************************************************/

#pragma once

// C++
#include <math.h>
#include <string>
#include <vector>
#include <iostream>
#include <pthread.h>
#include <thread>
#include <chrono>

// ROS
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <std_msgs/Int8.h>
#include <actionlib/server/simple_action_server.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Point.h>
#include <image_transport/image_transport.h>

// OpenCv
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <cv_bridge/cv_bridge.h>

// msgs
#include <perception_msgs/Objects.h>

// Darknet.
#ifdef GPU
#include "cuda_runtime.h"
#include "curand.h"
#include "cublas_v2.h"
#endif

extern "C" {
#include "network.h"
#include "detection_layer.h"
#include "region_layer.h"
#include "cost_layer.h"
#include "utils.h"
#include "parser.h"
#include "box.h"
#include "image.h"
#include <sys/time.h>
}

typedef struct
{
    float x, y, w, h, prob;
    int num, Class;
} RosBox_;

class yolo_ros
{
public:
	/**
	 *Constructor
	 */
	explicit yolo_ros();

    /**
     *Destructor
     */
	~yolo_ros();

	void init();

	image **load_alphabet_with_file(char *datafile);

	int sizeNetwork(network *net);

	void rememberNetwork(network *net);

	detection *avgPredictions(network *net, int *nboxes);

	int fromMatToImage(const cv::Mat& mat, const image& out);

	sensor_msgs::Image fromImageToRosImage(const image& image);

	void *detect_yolo();

	void *yolo(const cv::Mat& im);

	void cameraCallback(const sensor_msgs::CompressedImage::ConstPtr &msg);

	void doListening();


private:

	//ros
	ros::NodeHandle n;//private node handle
	ros::Publisher detection_img_pub;
	ros::Publisher objects_pub;
    ros::Subscriber rawimage_sub;

	//darknet
	network *net_;
	image buff_;
    image buffLetter_;
    image **demoAlphabet_;
    char **demoNames_;//detection object names
	int demoClasses_;
	float thresh;


    int demoTotal_;
	float *avg_;
	float *predictions_;

	// class labels.
    std::vector<std::string> classLabels_;
	int numClasses_;
	RosBox_ *roiBoxes_;

	std::string image_sub_name_;//subscribe topic

	//B Box
	std::vector<std::vector<RosBox_> > rosBoxes_;
	std::vector<int> rosBoxCounter_;
	perception_msgs::Objects objects_;
};
