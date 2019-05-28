/**
 * @file: Lidar_extrinsic_calibration.cpp
 * @author: Z.H
 * @date: 2019.05.15
 * 搞这个标定的时候真的是很多坑啊，完全没有想到一个简单的外参标定能浪费这么多时间，主要原因在于激光坐标系到车体坐标系的转换
 * 涉及到一个旋转矩阵和平移矩阵，大家可能知道，表示旋转的方式很多：旋转矩阵/向量、欧拉角、四元数，不过经常用的还是欧拉角，
 * 原因就是欧拉角比较好理解，将角度分解成绕ＸＹＺ三个互相垂直轴的角度序列组成，但是，但是，但是，在使用欧拉角表示三个轴的时侯，
 * 一定要提前说明好旋转的顺序是XYZ还是ZYX，前后要保持一致，因为不同的旋转顺序得到的结果是不一样的。比如：
 * 给定一组欧拉角角度值，yaw=45度，pitch=30度，roll=60度，按照yaw-pitch-roll(ZYX)的顺序旋转和按照roll-pitch-yaw(XYZ)的顺序旋转，
 * 最终刚体的朝向是不同的！换言之，若刚体需要按照两种不同的旋转顺序旋转到相同的朝向，所需要的欧拉角角度值则是不同的！
 * 注意：旋转角度的正负也遵循右手坐标系定则，大拇指指向轴线方向，四指方向为旋转的正方向.
 * 下面给出了几种写法，本质上都差不多，所有写法均已验证，可以保证正确性
 */
 /*------------激光点云外参标定的几种写法-----------*
 *             |x|       |x`|                  *      
 *             |y| = R * |y`| + T              *
 *             |z|       |z`|                  *
 *------------------转换公式--------------------*/
/*
法一：使用 Matrix4f
提示: 变换矩阵工作原理 :
  |-------> 变换矩阵列
    | 1 0 0 x |  \
    | 0 1 0 y |   }-> 左边是一个3阶的单位阵(无旋转)
    | 0 0 1 z |  /
    | 0 0 0 1 |    -> 这一行用不到 (这一行保持 0,0,0,1)
方法二: 使用 Affine3f;
方法三: 和法一类似，只不过把外参矩阵(4×４)分解开来写;
方法四：直接使用三角函数来构造外参矩阵
*/
//C++
#include <iostream>
#include <math.h>
#include <string>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <chrono>
#include <mutex>
#include <memory>
#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>

//ROS
#include <ros/ros.h>
#include <tf/tf.h>
#include "Eigen/Dense"


// OpenCv
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// msgs
#include "std_msgs/Header.h"
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/filters/passthrough.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/sample_consensus/method_types.h>  
#include <pcl/sample_consensus/model_types.h>  
#include <pcl/segmentation/extract_clusters.h>  
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl_conversions/pcl_conversions.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudPtr;
typedef pcl::PointCloud<pcl::PointXYZ>::ConstPtr PointCloudConstPtr;

#define PI_OVER_180 (0.0174532925)

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "test");
	std::vector<float> params = {0, 0, 0, 45.0, -30.0, 60.0}; //激光雷达到base坐标系下量测的6个参数，x,y,z,roll,pitch,yaw) :
	geometry_msgs::Point p;
	p.x = 1;
	p.y = 1;
	p.z = 1;
	Eigen::Vector4f input;
	input << p.x, p.y, p.z, 1;

	// NO.1
  Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
	float tx = params[0];
	float ty = params[1];
	float tz = params[2];
	float rx = params[3] * PI_OVER_180;
	float ry = params[4] * PI_OVER_180;
	float rz = params[5] * PI_OVER_180;

	Eigen::AngleAxisf init_rotation_x(rx, Eigen::Vector3f::UnitX());
	Eigen::AngleAxisf init_rotation_y(ry, Eigen::Vector3f::UnitY());
	Eigen::AngleAxisf init_rotation_z(rz, Eigen::Vector3f::UnitZ());

	Eigen::Translation3f init_translation(tx, ty, tz);

	Eigen::Matrix4f init_guess = //此处　init_translation　必须放在最前面乘，必须必须；
	      (init_translation * init_rotation_x * init_rotation_y * init_rotation_z).matrix();//此处统一欧拉角旋转顺序XYZ(roll-pitch-yaw)
	transform_1 = init_guess;
	std::cout << "1:" << std::endl << transform_1.matrix() << std::endl;
	Eigen::Vector4f output_1 = transform_1 * input;
	std::cout << std::endl << output_1 << std::endl;
   
   

	//NO.2
	Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();

	transform_2.translation() << params[0], params[1], params[2];
	transform_2.rotate (Eigen::AngleAxisf(params[3]*PI_OVER_180, Eigen::Vector3f::UnitX())
	                  * Eigen::AngleAxisf(params[4]*PI_OVER_180, Eigen::Vector3f::UnitY())
	                  * Eigen::AngleAxisf(params[5]*PI_OVER_180, Eigen::Vector3f::UnitZ()));//此处统一欧拉角旋转顺序XYZ(roll-pitch-yaw)
	// 打印变换矩阵
	std::cout << "2:" << std::endl << transform_2.matrix() << std::endl;
	Eigen::Vector4f output_2 = transform_2 * input;
	std::cout << std::endl << output_2 << std::endl;

  // 执行变换，并将结果保存在新创建的‎‎ transformed_cloud ‎‎中
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::transformPointCloud (*source_cloud, *transformed_cloud, transform_2);
  
	//NO.3
	Eigen::Matrix3f rot;
	Eigen::Vector3f tra, trans;
	geometry_msgs::Point output_3;

	rot = Eigen::AngleAxisf((params[3]* PI_OVER_180), Eigen::Vector3f::UnitX())
	    * Eigen::AngleAxisf((params[4]* PI_OVER_180), Eigen::Vector3f::UnitY())
	    * Eigen::AngleAxisf((params[5]* PI_OVER_180), Eigen::Vector3f::UnitZ());//此处统一欧拉角旋转顺序XYZ(roll-pitch-yaw)
	tra << p.x, p.y, p.z;
	trans = rot * tra;

	output_3.x = trans(0) + params[0];
	output_3.y = trans(1) + params[1];
	output_3.z = trans(2) + params[2];
	//打印变换矩阵
	std::cout << "3:" << std::endl << rot.matrix() << std::endl;
	std::cout << output_3 << std::endl;
    
  //NO.4
	Eigen::Vector3f pt0,pt1;
	geometry_msgs::Point output_4;
  pt0 << p.x, p.y, p.z;
  Eigen::Matrix3f rotX, rotY, rotZ;//此处统一欧拉角旋转顺序XYZ(roll-pitch-yaw)
  rotX << 1,0,0, 0,cos(params[3]* PI_OVER_180),-sin(params[3]* PI_OVER_180), 0,sin(params[3]* PI_OVER_180),cos(params[3]* PI_OVER_180);
  rotY << cos(params[4]* PI_OVER_180),0,sin(params[4]* PI_OVER_180), 0,1,0, -sin(params[4]* PI_OVER_180),0,cos(params[4]* PI_OVER_180);
  rotZ << cos(params[5]* PI_OVER_180),-sin(params[5]* PI_OVER_180),0, sin(params[5]* PI_OVER_180),cos(params[5]* PI_OVER_180),0, 0,0,1;

  Eigen::Matrix3f rot1 = rotX * rotY * rotZ;
  pt1 = rot1 * pt0; 
  output_4.x = pt1(0) + params[0];
  output_4.y = pt1(1) + params[1];
  output_4.z = pt1(2) + params[2];

　// 打印变换矩阵
	std::cout << "4:" << std::endl << rot1.matrix() << std::endl;
	std::cout << output_4 << std::endl;

	return 0;
}

//附publish点云的程序：
void publishCloud()
{
  //transform
  Eigen::Matrix4f tranfor_mat;
  std::vector<float> final_trans_vec(6, 0);
  getTransformMat(final_trans_vec, tranfor_mat);
  pcl::PointCloud<pcl::PointXYZI>::Ptr transfor_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::transformPointCloud(*g_cloud_, *transfor_cloud_ptr, tranfor_mat);
  //publish
  sensor_msgs::PointCloud2 centerMsg;
  pcl::toROSMsg(*filter_cloud_, centerMsg);
  centerMsg.header.frame_id = "rslidar";
  centerMsg.header.stamp = ros::Time::now();
  center_lidar_pub_.publish(centerMsg);

}

