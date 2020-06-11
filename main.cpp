/**
 * @file: main.cpp
 * @author: Z.H
 * @date: 2019.05.15
 */
//C++
#include <bits/stdc++.h>
#include <iostream>
#include <thread>
#include <queue>
#include <unistd.h>
#include <condition_variable>
#include <jsoncpp/json/json.h>

#include <string>
#include <vector>
#include <stdio.h>
#include <bitset>
#include <chrono>
#include <mutex>
#include <memory>
#include <time.h>
#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>
#include <stdint.h>
#include <math.h>
#include <sstream>    


//socket
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <stdio.h>
#include <errno.h>
#include <stdbool.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>


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
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include "nav_msgs/Odometry.h"
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GridCells.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"

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

//#include "yaml-cpp/yaml.h" //安装yaml-cpp参考google code 主页

using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudPtr;
typedef pcl::PointCloud<pcl::PointXYZ>::ConstPtr PointCloudConstPtr;


#define PI_OVER_180 (0.0174532925)
#define CURVE_POINT_NUM 150

//-----------------z.h------------------//
#define WHEEL_TO_TIRE 26.5  //wheel to tire
#define WHEELBASE 5.8   //wheelbase
#define CAR_WIDTH 1.25 //half of car width
#define MAX_DE_ACC 1.5  //(m/s^2) maximum de-acceleration

double global_feedback_steering_angle = 0.0;
double PREVIEW_SAFE_WIDTH = 0;
double PREVIEW_SAFE_LENGTH = 0;
double TTC_REDUCE_SPEED_TIME = 0;
double TTC_BRAKE_TIME = 0;

std::vector<double> v_angle_feedback;



//-----------------z.h------------------//


struct Point{
  double x;
  double y;
  double angle;
};

struct Curve{
	int32_t index = 0;
	Point points[CURVE_POINT_NUM];
	std::vector<Point> pointList;
};


/*------------激光点云外参标定的三种写法-----------*
 *             |x|       |x`|                  *      
 *             |y| = R * |y`| + T              *
 *             |z|       |z`|                  *
 *------------------转换公式--------------------*/
/*
//注意注意注意：旋转角度的正负也遵循右手坐标系定则，大拇指指向轴线方向，四指方向为旋转的正方向.

法一：使用 Matrix4f
提示: 变换矩阵工作原理 :
  |-------> 变换矩阵列
    | 1 0 0 x |  \
    | 0 1 0 y |   }-> 左边是一个3阶的单位阵(无旋转)
    | 0 0 1 z |  /
    | 0 0 0 1 |    -> 这一行用不到 (这一行保持 0,0,0,1)
    
Tip:这个是“手工方法”，可以完美地理解，但容易出错!
  
in_vec(也就是激光雷达到base坐标系下量测的6个参数，x,y,z,roll,pitch,yaw) : 
方法二 : 使用 Affine3f

*/




Curve createVirtualCurve() {
	Curve virtualCurve;
	double xMin = 0.0;
	double xMax = 0.0;
	double yMin = 0.0;
	double yMax = 7.5;
	double yStep = (yMax - yMin) / CURVE_POINT_NUM;
	for (size_t i=0; i<CURVE_POINT_NUM; ++i) {
		Point p;
		p.x = 0.0;
		p.y = yMin + i*yStep;
		virtualCurve.points[i] = p;
	}
	return virtualCurve;
}

void curvePublisher(ros::Publisher curve_pub_, const Curve& curve) {
  geometry_msgs::PoseArray cloud_msg;
  int i,j;
  cloud_msg.header.stamp = ros::Time::now();
  cloud_msg.header.frame_id = "base_link";
  int size = CURVE_POINT_NUM;
  cloud_msg.poses.resize(size);
  int index = 0;
  for(j = 0; j < size; ++j){
        tf::poseTFToMsg(tf::Pose(tf::createQuaternionFromYaw(curve.points[j].angle),
                                  tf::Vector3(curve.points[j].x,
                                              curve.points[j].y, 1)),
                        cloud_msg.poses[index]);
        index++;
    }
  curve_pub_.publish(cloud_msg);
  //ROS_INFO("---I publish a curve!");
}

struct RoadPoint{

  bool b_isValid = false;

  int road_id;//as matlab p1 p2 p3 ... number of Road
  int index = 0;//order number
  double longitude = 0.0;
  double latitude = 0.0;
  double courseAngle = 0.0;//heading

  int parameter1;// 0 normal point;1 traffic light point; 2 stop station point 3 enable avoid obstacle
  double parameter2 = 0.0;//curvature
  double parameter3 = 0.0;//max speed

  //lateral control use
  double lateral_dist = 0;
  double yaw_diff = 0;

};

struct LidarData{
  std::vector<uint32_t> blockedAreaIndex;
  std::vector<Point> obstacle_coordinates;
  bool b_isAlive = false;
  bool b_isValid = false;
};

class DecisionData{
public:
  std::vector<Curve> curveList;
  
  double targetSteeringAngle = 0.0;
  double targetSteeringVelocity = 0.0;
  double targetSpeed = 0.0;
  double targetThrottle = 0.0;
  double throttleIntegral = 0.0;
  double targetBrakePressure = 0.0;
  double previewDistance = 0.0;
  uint8_t targetGearPosition = 0;
  int16_t targetAccLevel = 2;
  int32_t currentState = 0;
  int32_t currentId = 0;
  int32_t currentIndex = 0;
  int32_t currentTotalIndex = 0;
  int32_t roadType = 0;
  int32_t nextId = 0;
  int32_t nextNextId = 0;

  uint32_t targetWorkMode = 0;
  uint16_t postureSensorMode = 0;
  bool b_takeOverEnable = false;
  bool b_takingOver = false;
  bool b_comingBack = false;
  bool b_redLightStopped = false;
  bool b_onRoadPoint = false;
  
  uint16_t pathNumber = 0;
  boost::posix_time::ptime takingOverTimeCounter;
  
  bool b_isValid = false;
  bool b_isAlive;

  double velocity = 0;
  double last_advisedSpeed = 0; //last speed for lidar judge
  double last_advisedSpeedTakeOver = 0; //last speed for lidar judge
  bool stationStopFlag = 0; //station stop

  double targetSpeedTest = 0;

  double current_steering_angle = 0;
 
public:
  uint32_t updateStateMachine();
};



LidarData lidarData;
DecisionData decisionData;
ros::Publisher grid_pub;

void gridCallback(const nav_msgs::OccupancyGrid::ConstPtr &grid, LidarData* lidarDataPtr){
   /*
  	map->size_x = grid->info.width;
  	map->size_y = grid->info.height;
  	map->scale = grid->info.resolution;
  	map->origin_x = grid->info.origin.position.x;
  	map->origin_y = grid->info.origin.position.y;
  	// Convert to player format
    //  The map data, in row-major order, starting with (0,0).  Occupancy
    //  probabilities are in the range [0,100].  Free is -1.
  	map->cells = (map_cell_t*)malloc(sizeof(map_cell_t)*map->size_x*map->size_y);
  	ROS_ASSERT(map->cells);
    // ROS_INFO("Created cells!\n");
  	
    // ROS_INFO("Cells Defined!\n");

    int totalNumber = grid->info.width * grid->info.height;
    // std::vector<bool> grids;
    // grids.reserve(GRID_INDEX_SIZE);
    // grids.resize(GRID_INDEX_SIZE,1);
    // ROS_INFO("grids created");
    free(map->cells);
    */
    int grid_width = (int)grid->info.width;
    int grid_height = (int)grid->info.height;
    int grid_size = (int)(grid_width * grid_height);
    double grid_resolution = grid->info.resolution;
    double temp_resolution = grid_resolution/2;
    double x_minimum = grid->info.origin.position.x;
    double y_minimum = grid->info.origin.position.y;
    ROS_INFO("Received a %d X %d map @ %.3f m/pix, Origin position x: %.3f, y: %.3f ",
              grid_width, grid_height, grid_resolution, x_minimum, y_minimum);

    lidarDataPtr->blockedAreaIndex.clear();
    lidarDataPtr->obstacle_coordinates.clear();
    
    ROS_INFO("GRID SIZE: %d",grid_size);
    if(grid_size > 0){
      for(size_t i = 0; i < grid_size; ++i)
      {
   
        lidarDataPtr->blockedAreaIndex.push_back(i);
        size_t j = i%grid_width;
        size_t k = i/grid_width;
        //std::cout << "i,j,k:" << i <<"; " << j << "; " << k << "; " <<  temp_resolution << std::endl;
        Point p_coordinate;
        p_coordinate.x = j * grid_resolution + x_minimum + temp_resolution;
        p_coordinate.y = k * grid_resolution + y_minimum + temp_resolution;
        //std::cout << "i=" <<i<< "p_coordinate:" << p_coordinate.x << "; " << p_coordinate.y << std::endl;
        lidarDataPtr->obstacle_coordinates.push_back(p_coordinate);
       
      }
    }
    ROS_INFO("blockedAreaIndex size:%ld; obstacle_coordinates:%ld",
             lidarDataPtr->blockedAreaIndex.size(), lidarDataPtr->obstacle_coordinates.size());
    

  lidarDataPtr->b_isAlive = 1;
	lidarDataPtr->b_isValid = 1;

	/*----------------------------z.h--------------------------------*/
  /*
	decisionData.velocity = 0.288996/3.6;//2.77
	decisionData.current_steering_angle = -37.375000;



	double advisedSpeed;
	double judgeSpeed = decisionData.velocity;
	
  paramsAdjustByVehicleSpeed(decisionData);

  ROS_INFO("[Z.H]now receive wheel steering angle feedback:%f degree", decisionData.current_steering_angle);
  double tire_angle_feedback = decisionData.current_steering_angle/WHEEL_TO_TIRE;
  ROS_INFO("[Z.H]now receive tire angle feedback:%f degree", tire_angle_feedback);
  double tire_angle_feedback_rad = tire_angle_feedback*M_PI/180;
  double current_v_front = decisionData.velocity;//front wheel speed ::the previous conversion has been made
  // double current_v_back = postureData.imuData.velocity;//back wheel speed
  // double current_v_front = current_v_back/(std::cos(tire_angle_feedback_rad));//front wheel speed
  
  if(std::fabs(tire_angle_feedback) >= 1)
  {
    double r_front = WHEELBASE/std::sin(tire_angle_feedback_rad);
    double r_back = r_front*(std::cos(tire_angle_feedback_rad));

    ROS_INFO("[Z.H]calculate 1.r_front: %f m  2.r_back: %f m.", r_front, r_back);
    Point circle_point;//center of circle
    circle_point.x = r_back;
    circle_point.y = -WHEELBASE;
  
    double angular_speed = current_v_front/(std::fabs(r_front));//rad/s
    ROS_INFO("[Z.H]current angular_speed:%f rad/s", angular_speed);
    double r_min = (std::fabs(r_back)) - CAR_WIDTH - PREVIEW_SAFE_WIDTH;
    double r_max = (std::fabs(r_front)) + CAR_WIDTH + PREVIEW_SAFE_WIDTH;

    std::vector<double> v_ttc;
    if(!lidarData.obstacle_coordinates.empty() && (current_v_front > 0) && (angular_speed > 0) && (r_min > 0) && (r_max > 0))
    {
        int len = lidarData.obstacle_coordinates.size();
        //std::cout << "AAA" << std::endl;
        for(size_t i = 0; i < len; ++i)
        {
            double p_x = lidarData.obstacle_coordinates[i].x;
            double p_y = lidarData.obstacle_coordinates[i].y;
            double r_temp = sqrt((p_x - circle_point.x)*(p_x - circle_point.x) + (p_y - circle_point.y)*(p_y - circle_point.y));
            //std::cout << "BBB" << std::endl;
            if(r_temp <= r_max && r_temp >= r_min){
              //std::cout << "CCC" << std::endl;
                double alpha = std::atan2(p_y + WHEELBASE, std::fabs(r_back - p_x));//rad
                if(alpha > std::fabs(tire_angle_feedback_rad)){
                    double ttc = (alpha - std::fabs(tire_angle_feedback_rad))/angular_speed;
                    v_ttc.push_back(ttc);
                }   
            }
        }

        double ttc_min = *min_element(v_ttc.begin(), v_ttc.end());
        ROS_INFO("[Z.H]when tire_angle_feedback is useful,now ttc_min: %f s",ttc_min);
        if(ttc_min <= TTC_REDUCE_SPEED_TIME)
        {
            advisedSpeed += (-0.1)*MAX_DE_ACC;
            if(advisedSpeed < 0.1){
                advisedSpeed = 0.0;
            }
        }
        if(ttc_min <= TTC_BRAKE_TIME)
            advisedSpeed = 0.0;
    }
    else
    {
        advisedSpeed = judgeSpeed;
    }
  }

  //std::fabs(tire_angle_feedback)<1.0
  else
  {
    double min_x = 0.0 - CAR_WIDTH - PREVIEW_SAFE_WIDTH;
    double max_x = 0.0 + CAR_WIDTH + PREVIEW_SAFE_WIDTH;
    double min_y = 0.0;
    double max_y = PREVIEW_SAFE_LENGTH;
    std::vector<double> v_ttc;
    if(!lidarData.obstacle_coordinates.empty() && current_v_front > 0) 
    {
        int len = lidarData.obstacle_coordinates.size();

        for(size_t i = 0; i < len; ++i)
        {
            double p_x = lidarData.obstacle_coordinates[i].x;
            double p_y = lidarData.obstacle_coordinates[i].y;
            if(p_x <= max_x && p_x >= min_x && p_y <= max_y && p_y >= min_y){
                double ttc = std::fabs(p_y)/current_v_front;
                v_ttc.push_back(ttc);
            }
        }

        double ttc_min = *min_element(v_ttc.begin(), v_ttc.end());
        ROS_INFO("[Z.H]when tire_angle_feedback is too small and is invalid, now ttc_min:%f",ttc_min);
        if(ttc_min <= TTC_REDUCE_SPEED_TIME)
        {
            advisedSpeed += (-0.1)*MAX_DE_ACC;
            if(advisedSpeed < 0.1){
                advisedSpeed = 0.0;
            }
        }
        if(ttc_min <= TTC_BRAKE_TIME)
            advisedSpeed = 0.0;
    }
    else
    {
        advisedSpeed = judgeSpeed;
    }
  }
  ROS_INFO("[Z.H]original judgeSpeed:%f km/h", judgeSpeed*3.6);
  ROS_INFO("[Z.H]NOW advisedSpeed:%f km/h", advisedSpeed*3.6);

  //limit acceleration
    //accLimiter(advisedSpeed, advisedSpeedTakeOver, decisionData);
    ROS_INFO("[Z.H]NOW after acc limiter advisedSpeed:%f km/h", advisedSpeed*3.6);

*/

   /*
	global_feedback_steering_angle = decisionData.current_steering_angle;
	ROS_INFO("[ZH]Global_feedback_steering_angle :%f ",global_feedback_steering_angle);

	double tire_angle_feedback = global_feedback_steering_angle/Tire2Wheel;
	double tire_angle_feedback_rad = tire_angle_feedback*M_PI/180;
	ROS_INFO("[ZH]tire_angle_feedback_rad:%f", tire_angle_feedback_rad);
	double current_v_back = decisionData.velocity;//back wheel speed
	double current_v_front = current_v_back/(std::cos(tire_angle_feedback_rad));//front wheel speed
	ROS_INFO("[ZH]current_v_front:%f", current_v_front);


	if(std::fabs(tire_angle_feedback)>=0.5){

	double r_front = WHEELBASE/std::sin(tire_angle_feedback_rad);
	double r_back = r_front*(std::cos(tire_angle_feedback_rad));
	ROS_INFO_STREAM("r_front:" << r_front << " r_back:" << r_back);

	Point circle_point;//center of circle
	circle_point.x = r_back;
	circle_point.y = -WHEELBASE;
	ROS_INFO_STREAM("circle_point:%f" << circle_point.x << " %f" << circle_point.y);

	double angular_speed = current_v_front/(std::fabs(r_front));//rad/s
	ROS_INFO("[ZH]Current angular_speed:%f", angular_speed);
	double r_min = (std::fabs(r_back)) - CAR_WIDTH - PREVIEW_SAFE_WIDTH;
	double r_max = (std::fabs(r_front)) + CAR_WIDTH + PREVIEW_SAFE_WIDTH;

	std::vector<double> v_ttc;
	//ROS_INFO("begin to make grid map!!!");
	nav_msgs::OccupancyGrid map_;
    
	map_.header.frame_id = "base_link";
  	map_.header.stamp = ros::Time::now();

	int map_width = 100;
	int map_height = 200;
	int map_size = map_width * map_height;
	map_.info.resolution = 0.2;
	map_.info.width = map_width;
	map_.info.height = map_height;
	map_.info.origin.position.x = -10;
	map_.info.origin.position.y = 0;//In the coordinate system conversion, the lidar coordinate system should be converted to the Car coordinate system of the local vehicle, x to the right, y to the front.
	map_.info.origin.position.z = 0;
	map_.data.resize(map_size, 0);

	if(!lidarDataPtr->obstacle_coordinates.empty()) {
	    int len = lidarDataPtr->obstacle_coordinates.size();
	    for(size_t i = 0; i < len; ++i)
	    {
	        double p_x = lidarDataPtr->obstacle_coordinates[i].x;
	        double p_y = lidarDataPtr->obstacle_coordinates[i].y;
	        double r_temp = sqrt((p_x - circle_point.x)*(p_x - circle_point.x) + (p_y - circle_point.y)*(p_y - circle_point.y));
	        ROS_INFO("[ZH]r_temp:%f", r_temp);
	        if(r_temp <= r_max && r_temp >= r_min){
	        	ROS_INFO_STREAM("p_x:" << p_x << " p_y:" << p_y);
	            double alpha = std::atan2(p_y + WHEELBASE,std::fabs(r_back - p_x));//rad
	            ROS_INFO("[ZH]alpha:%f", alpha);
	            if(alpha >= std::fabs(tire_angle_feedback_rad))
	            {
	            	double ttc = (alpha - std::fabs(tire_angle_feedback_rad))/angular_speed;
		            v_ttc.push_back(ttc);
		            int map_x = (int)((p_x-(-10))/0.2);
      					int map_y = (int)((p_y-0.0)/0.2);
      					int index = map_y * map_width + map_x;
      					if(index < map_size && index >= 0 && map_x >= 0 && map_y >= 0) {
      						map_.data[index]=100;
      					}
	            }
	            
	        }
	        else
	        {
	        	map_.data[i]=0;
	        }
	    }
	    grid_pub.publish(map_);
	    double ttc_min = *min_element(v_ttc.begin(), v_ttc.end());
	    ROS_INFO("[ZH]ttc_min:%f",ttc_min);
	    if(ttc_min <= TTC_REDUCE_SPEED_TIME)
	    {
	        advisedSpeed += (-0.1)*MAX_DE_ACC;
	        if(advisedSpeed < 0.01){
	            advisedSpeed = 0.0;
	        }
	    }
	    if(ttc_min <= TTC_BRAKE_TIME)
	        advisedSpeed = 0.0;
	}
	else
	{
	    advisedSpeed = judgeSpeed;
	}
	}

	//std::fabs(tire_angle_feedback)<2.0
	else
	{
		double min_x = 0.0 - CAR_WIDTH - PREVIEW_SAFE_WIDTH;
		double max_x = 0.0 + CAR_WIDTH + PREVIEW_SAFE_WIDTH;
		double min_y = 0.0;
		double max_y = 7.5;
		std::vector<double> v_ttc;
		if(!lidarDataPtr->obstacle_coordinates.empty()) {
		    int len = lidarDataPtr->obstacle_coordinates.size();
		    for(size_t i = 0; i < len; ++i)
		    {
		        double p_x = lidarDataPtr->obstacle_coordinates[i].x;
		        double p_y = lidarDataPtr->obstacle_coordinates[i].y;
		        if(p_x <= max_x && p_x >= min_x && p_y <= max_y && p_y >= min_y){
		            double ttc = std::fabs(p_y)/current_v_back;
		            v_ttc.push_back(ttc);
		        }
		    }
		    double ttc_min = *min_element(v_ttc.begin(), v_ttc.end());
		    ROS_INFO("ttc_min:%f",ttc_min);
		    if(ttc_min <= TTC_REDUCE_SPEED_TIME)
		    {
		        advisedSpeed += (-0.1)*MAX_DE_ACC;
		        if(advisedSpeed < 0.01){
		            advisedSpeed = 0.0;
		        }
		    }map_.header.stamp = ros::Time::now();
		    if(ttc_min <= TTC_BRAKE_TIME)
		        advisedSpeed = 0.0;
		}
		else
		{
		    advisedSpeed = judgeSpeed;
		}
	}
	ROS_INFO("[ZH]preview advisedSpeed:%f", advisedSpeed);
	/*----------------------------z.h--------------------------------*/

    
}

double getAngle (const double x0, const double y0, const double x1, const double y1)
{
  double deltaY = y1 - y0;
  double deltaX = x1 - x0;

  return std::atan2(deltaY, deltaX)*180/M_PI;
}

double getAnglebtwn(const double x1, const double y1, const double x2, const double y2, const double c_a)
{
  // >0 left <0 right
  double a1 = getAngle(x2,y2,x1,y1);
  std::cout << "a1:" << a1 << std::endl;
  a1 = 90-a1;
  if(a1<0) 
    a1+=360;
  return c_a-a1;
}

double getYawDiff (double yawVehicle, double yawRoad)
{
  if (yawVehicle > 180) {
    yawVehicle = yawVehicle - 360;
  }
  else;
  yawVehicle = -yawVehicle;

  if (yawRoad > 180) {
    yawRoad = yawRoad - 360;
  }
  else;
  yawRoad = -yawRoad;

  double ret = yawRoad - yawVehicle;

  if (ret > 180) {
    ret = ret - 360;
  }
  else if (ret < -180){
    ret = ret + 360;
  }
  else;

  return ret;
}

void gaussConvert(const double &position_x, const double &position_y, double &x1, double &y1)
{
    // Earth Ellipsoid WGS84
    double lon;
    double lat;
    int a=6378137;
    double lon0;
    double e=0.0066943799013;
    double e2=0.006739496742270;
    double deltL;
    double t;
    double n;
    double C1;
    double C2;
    double C3;
    double C4;
    double C5;
    double N;
    double X;
    double A;
    lon=(double)position_x*2*M_PI/360/10000000;
    lat=(double)position_y*2*M_PI/360/10000000;
    lon0=117*2*M_PI/360;
    deltL=lon-lon0;
    t=tan(lat);
    n=e2*cos(lat);
    C1=1+3*pow(e,2)/4+45*pow(e,4)/64+175*pow(e,6)/256+11025*pow(e,8)/16384;
    C2=3*pow(e,2)/4+15*pow(e,4)/16+525*pow(e,6)/512+2205*pow(e,8)/2048;
    C3=15*pow(e,4)/64+105*pow(e,6)/256+2205*pow(e,8)/4096;
    C4=35*pow(e,6)/512+315*pow(e,8)/2048;
    C5=315*pow(e,8)/131072;
    N=a/sqrt(1-pow(e,2)*pow(sin(lat),2));
    X=a*(1-pow(e,2))*(C1*lat-0.5*C2*sin(2*lat)+C3*sin(4*lat)/4-C4*sin(6*lat)/6+C5*sin(8*lat));
    A=1+pow(deltL,2)/12*pow(cos(lat),2)*(5-pow(t,2)+9*pow(n,2)+4*pow(n,4))+pow(deltL,4)/360*pow(cos(lat),4)*(61-58*pow(t,2)+pow(t,4));
    y1=X+0.5*N*sin(lat)*cos(lat)*pow(deltL,2)*A;//正北方向
    x1=N*cos(lat)*deltL*(1+pow(deltL,2)/6*pow(cos(lat),2)*(1-pow(t,2)+pow(n,2))+pow(deltL,4)/120*pow(cos(lat),4)*(5-18*pow(t,2)+pow(t,4)-14*pow(n,2)-58*pow(n,2)*pow(t,2)));//正东方向
}

double vel_position_y = 117.2;
double vel_position_x = 39.1;

void gaussConvert(double& x1,double& y1)
  {
    // Earth Ellipsoid WGS84
    double lon;
    double lat;
    int a=6378137;
    double lon0;
    double e=0.0066943799013;
    double e2=0.006739496742270;
    double deltL;
    double t;
    double n;
    double C1;
    double C2;
    double C3;
    double C4;
    double C5;
    double N;
    double X;
    double A;
    lon=(double)vel_position_y*2*M_PI/360/10000000;
    lat=(double)vel_position_x*2*M_PI/360/10000000;
    lon0=117*2*M_PI/360;
    deltL=lon-lon0;
    t=tan(lat);
    n=e2*cos(lat);
    C1=1+3*pow(e,2)/4+45*pow(e,4)/64+175*pow(e,6)/256+11025*pow(e,8)/16384;
    C2=3*pow(e,2)/4+15*pow(e,4)/16+525*pow(e,6)/512+2205*pow(e,8)/2048;
    C3=15*pow(e,4)/64+105*pow(e,6)/256+2205*pow(e,8)/4096;
    C4=35*pow(e,6)/512+315*pow(e,8)/2048;
    C5=315*pow(e,8)/131072;
    N=a/sqrt(1-pow(e,2)*pow(sin(lat),2));
    X=a*(1-pow(e,2))*(C1*lat-0.5*C2*sin(2*lat)+C3*sin(4*lat)/4-C4*sin(6*lat)/6+C5*sin(8*lat));
    A=1+pow(deltL,2)/12*pow(cos(lat),2)*(5-pow(t,2)+9*pow(n,2)+4*pow(n,4))+pow(deltL,4)/360*pow(cos(lat),4)*(61-58*pow(t,2)+pow(t,4));
    y1=X+0.5*N*sin(lat)*cos(lat)*pow(deltL,2)*A;//正北方向
    x1=N*cos(lat)*deltL*(1+pow(deltL,2)/6*pow(cos(lat),2)*(1-pow(t,2)+pow(n,2))+pow(deltL,4)/120*pow(cos(lat),4)*(5-18*pow(t,2)+pow(t,4)-14*pow(n,2)-58*pow(n,2)*pow(t,2)));//正东方向
  }

//模板函数：将string类型变量转换为常用的数值类型（此方法具有普遍适用性）
template <class Type>
Type stringToNum(const string& str)
{
    istringstream iss(str);
    Type num;
    iss >> num;
    return num;    
}

// template <typename T>
// void operator >> (const YAML::Node& node, T& i) {
//   i = node.as<T>();
// };
class Foo
{
public:
    int a;
    int b;
 
    Foo():a(0), b(0){}
    ~Foo(){}
 
    Foo(int a, int b)
    {
        this->a = a;
        this->b = b;
    }
    friend void print(Foo &obj);
 
    // 规定对象排序的算法：先按照 a 从小到大排序；如果 a 相等，则按照 b 从小到大排序
    bool operator<(const Foo &bar)
    {
        if (this->a < bar.a)
        {
            return true;
        }
        else if (this->a == bar.a)
        {
            return this->b < bar.b;
        }
        return false;
    }
 
    // 规定对象排序的算法：先按照 a 从大到小排序；如果 a 相等，则按照 b 从大到小排序
    bool static decrease(const Foo &foo1, const Foo &foo2)
    {
        if (foo1.a > foo2.a)
        {
            return true;
        }
        else if (foo1.a == foo2.a)
        {
            return foo1.b > foo2.b;
        }
        return false;
    }
 
    friend inline ostream & operator<<(ostream &out, Foo &foo)
    {
        out << foo.a << " " << foo.b << endl;
        return out;
    }
  private:
    int ka = 10;
};


void print(Foo &obj)
{
  std::cout << "AAA" << obj.ka << std::endl;
}



//生产者消费者模型 condition_variable
//wait_for可以指定一个时间段，在当前线程收到通知或者指定的时间 rel_time 超时之前，该线程都会处于阻塞状态。
//如果超时返回std::cv_status::timeout。如果收到了其他线程的通知，返回std::cv_status::no_timeout。
//(g_cv.wait_for(lock, std::chrono::seconds(1)) == std::cv_status::timeout)

static std::mutex g_mutex;
static std::condition_variable g_cv_produce;
static std::condition_variable g_cv_consume;

static std::queue<int> g_queue;
static int maxSize = 20;

void consumer(void)
{
    while(true) {
        sleep(1);
        std::unique_lock<std::mutex> lk(g_mutex);
        while(g_queue.size() == 0) {
            g_cv_consume.wait(lk);
        }
        // lambda 
        //g_cv_consume.wait(lk, [] {return g_queue.size() != 0;});

        std::cout << "consumer thread id: " << this_thread::get_id() << std::endl;

        std::cout << "consumer thread g_queue.front(): " << g_queue.front() << std::endl;
        g_queue.pop();
        std::cout << "consumer thread now queue size is: " << g_queue.size() << std::endl;
        g_cv_produce.notify_all();
        //lk.unlock();
    }

}

void producer(void)
{
    while(true) {
        sleep(1);
        std::unique_lock<std::mutex> lk(g_mutex);
        while(g_queue.size() == maxSize) {
            g_cv_produce.wait(lk);
        }
        // lambda
        //g_cv_produce.wait(lk, [] {return g_queue.size() != maxSize; });

        std::cout << "producer thread id: " << this_thread::get_id() << std::endl;
        std::cout << "producer thread g_queue.front(): " << g_queue.front() << std::endl;
        g_queue.push(1);
        std::cout << "producer thread now queue size is: " << g_queue.size() << std::endl;
        g_cv_consume.notify_all();
        //lk.unlock();
    }
}

class A
{
public:
  
  static int a;
  static void print(int b) {
    printf("a + b = %d\n", a + b);
  }
  const static int c = 20;
  
};

int A::a = 1;

/*
function [E, N] = UTM(lat, long)
    a=6378137; %%%meter
    e=sqrt(0.00669438);
    k0=0.9996; 
    E0=500000; N0=0; %%%meter
    Zonenum=fix(long/6)+31;
    lamda0=(Zonenum-1)*6-180+3;   %%%%degree 
    lamda0=lamda0*pi/180;    %%%%%radian
    phi=lat*pi/180;   lamda=long*pi/180;    %%%%%radian

    v=1/sqrt(1-(e*sin(phi))^2);
    A=(lamda-lamda0)*cos(phi);
    T=tan(phi)^2;
    C=(e*cos(phi))^2/(1-e^2);
    s=(1-e^2/4-3*e^4/64-5*e^6/256)*phi-...
        (3*e^2/8+3*e^4/32+45*e^6/1024)*sin(2*phi)+...
        (15*e^4/256+45*e^6/1024)*sin(4*phi)-35*e^6/3072*sin(6*phi);

    E=E0+k0*a*v*(A+(1-T+C)*A^3/6+(5-18*T+T^2)*A^5/120);
    N=N0+k0*a*(s+v*tan(phi)*(A^2/2+(5-T+9*C+4*C^2)*A^4/24+(61-58*T+T^2)*A^6/720));
end
*/
//
void gaussConvert1(const double& lon, const double& lat, 
                   double lon0, double& x, double& y) {
    double p_x_r = lon * M_PI / 180;
    double p_y_r = lat * M_PI / 180;

    double a = 6378137;
    double e = sqrt(0.00669438);
    double k = 0.9996;
    
    double p_x_z_r = lon0 * M_PI / 180;

    double d_v = 1.0 / sqrt(1 - pow((e * sin(p_y_r)), 2));
    double d_a = (p_x_r - p_x_z_r) * cos(p_y_r);
    double d_t = pow(tan(p_y_r), 2);
    double d_c = pow((e * cos(p_y_r)), 2) / (1 - pow(e, 2));

    double s1 = (1 - (pow(e, 2) / 4) - (3 * (pow(e, 4) / 64)) - (5 * (pow(e, 6) / 256))) * p_y_r;
    double s2 = ((3 * (pow(e, 2) / 8)) + (3 * (pow(e, 4) / 32)) + (45 * (pow(e, 6) / 1024))) * sin(2 * p_y_r);
    double s3 = ((15 * (pow(e, 4) / 256)) + (45 * (pow(e, 6) / 1024))) * sin(4 * p_y_r);
    double s4 = (35 * (pow(e, 6) / 3072)) * sin(6 * p_y_r);
    double s = s1 - s2 + s3 -s4;

    x = k * a * d_v * (d_a + ((1 - d_t + d_c) * pow(d_a, 3) / 6) + ((5 - 18 * d_t + pow(d_t, 2)) * pow(d_a, 5) / 120));//正东方向
    y = k * a * (s + (d_v * tan(p_y_r) * ((pow(d_a, 2) / 2) + ((5 - d_t + 9 * d_c + 4 * pow(d_c, 2)) * pow(d_a, 4) / 24) + ((61 - 58 * d_t + pow(d_t, 2)) * pow(d_a, 6) / 720))));//正北方向

   


    /*
    //earth ellipsoid WGS84
    double p_x = position_x / 10000000;//longitude
    double p_y = position_y / 10000000;//latitude

    // double east = 0;//500000;
    // double north = 0;zz

    // double p_x_z = (int)(p_x / 6) + 31;
    // double p_x_z_d = (p_x_z - 1) * 6 - 180 + 3;



    // Earth Ellipsoid WGS84
    int a = 6378137;
    double e = 0.0066943799013;
    double e2 = 0.006739496742270;
    double lon, lat, lon0, t, n, deltL;
    double C1, C2, C3, C4, C5, N, X, A;

    lon = (double)position_x*2*M_PI/360/10000000;
    lat = (double)position_y*2*M_PI/360/10000000;
    lon0 = 117*2*M_PI/360;
    deltL = lon-lon0;
    t = tan(lat);
    n = e2*cos(lat);

    C1 = 1+3*pow(e,2)/4+45*pow(e,4)/64+175*pow(e,6)/256+11025*pow(e,8)/16384;
    C2 = 3*pow(e,2)/4+15*pow(e,4)/16+525*pow(e,6)/512+2205*pow(e,8)/2048;
    C3 = 15*pow(e,4)/64+105*pow(e,6)/256+2205*pow(e,8)/4096;
    C4 = 35*pow(e,6)/512+315*pow(e,8)/2048;
    C5 = 315*pow(e,8)/131072;

    N = a/sqrt(1-pow(e,2)*pow(sin(lat),2));
    X = a*(1-pow(e,2))*(C1*lat-0.5*C2*sin(2*lat)+C3*sin(4*lat)/4-C4*sin(6*lat)/6+C5*sin(8*lat));
    A = 1+pow(deltL,2)/12*pow(cos(lat),2)*(5-pow(t,2)+9*pow(n,2)+4*pow(n,4))+pow(deltL,4)/360*pow(cos(lat),4)*(61-58*pow(t,2)+pow(t,4));
    y1 = X+0.5*N*sin(lat)*cos(lat)*pow(deltL,2)*A;//正北方向
    x1 = N*cos(lat)*deltL*(1+pow(deltL,2)/6*pow(cos(lat),2)*(1-pow(t,2)+pow(n,2))+pow(deltL,4)/120*pow(cos(lat),4)*(5-18*pow(t,2)+pow(t,4)-14*pow(n,2)-58*pow(n,2)*pow(t,2)));//正东方向
    */
}

static int kk = 0;
int main(int argc, char *argv[]) 
{
  // int numRows=10,zone=5;//层数，每层需要的空间
  // vector<vector<int>> vec(numRows, vector<int>());//初始层数，赋值
  // for (int i = 0; i < numRows; i++) {
  //     vec[i].resize(zone);
  // }




  
  /*
  string h = "hello world";
  std::cout << h.size() << std::endl;
  const char* buf = h.c_str();
  std::cout << "buf: " << buf << " " << strlen(buf) << std::endl;
  
  double out_x =0;
  double out_y =0;
  double out_x1 =0;
  double out_y1 =0;

  gaussConvert1(20,30,29, out_x,out_y);
  gaussConvert1(20,31,29, out_x1,out_y1);
  //gaussConvert(out_x,out_y);
  std::cout << setprecision(20) << out_x - out_x1 << std::endl;
  std::cout << setprecision(20) << out_y - out_y1 << std::endl;
  std::cout << setprecision(20) << atan2(out_y - out_y1 , out_x - out_x1) * 180 / M_PI << std::endl;
  */



  //std::cout << setprecision(12) << pow(2,28) << std::endl;
  //double real = ((int16_t)((int8_t)test1 << 8 | test0)) * 0.1;
  unsigned char t0 = 0x83;//0xfd;
  unsigned char t1 = 0xfe;//0xeb; 
  double r = ((uint16_t)(t0 << 3) + (uint8_t)((t1 & 0xe0) >> 4));
  double r1 = ((uint16_t)(t0 << 3) | (uint8_t)((t1 & 0xe0) >> 4));
  printf("%#X\n", (uint16_t)(t0 << 3));
  printf("%#X\n", (uint8_t)((t1 & 0xe0) >> 4));
  printf("%#X\n", ((uint16_t)(t0 << 3) + (uint8_t)((t1 & 0xe0) >> 4)));
  printf("%#X\n", ((uint16_t)(t0 << 3) | (uint8_t)((t1 & 0xe0) >> 4)));
  
  printf("r = %f\n", r);
  printf("r1 = %f\n", r1);
  


  


  time_t myt = time(NULL);
  std::cout << "myt is :"<< myt << std::endl;

  //send vehicle obstacle msg
  Json::Value json_send_object_msg;
  json_send_object_msg["Timestamp"] = Json::Value((double)myt);
  for (size_t i = 0; i < 4; ++i) {
      Json::Value json_obj;     
      json_obj["PtcType"] = Json::Value(1);
      json_obj["PtcID"] = Json::Value(2);
      json_obj["Confidence"] = Json::Value(3); 
      json_obj["CoordX"] = Json::Value(4);
      json_obj["CoordY"] = Json::Value(5);
      json_obj["CoordZ"] = Json::Value(6);
      json_obj["Length"] = Json::Value(7);
      json_obj["Width"] = Json::Value(8);
      json_obj["Height"] = Json::Value(9);
      json_send_object_msg["EventDetail"].append(json_obj);
  }      
  
  Json::StyledWriter styled_writer_object;
  std::string send_buf_object = styled_writer_object.write(json_send_object_msg);
  std::cout << styled_writer_object.write(json_send_object_msg) << std::endl << std::endl;

  //register
  // Json::Value json_register_msg;      
  // Json::Value json_temp;

  // json_temp["Name"] = Json::Value("SSM");
  // json_temp["Description"] = Json::Value("绿波车速消息");      
  
  // json_register_msg["Timestamp"] = Json::Value((double)myt);
  // json_register_msg["Regist"].append(json_temp);
  
  // Json::StyledWriter styled_writer2;
  // std::cout << styled_writer2.write(json_register_msg) << std::endl << std::endl;
  /*
  //解析json对象数组
  const Json::Value Obj1 = value["employees"];
  int inum1 = Obj1.size();
  for (int j = 0; j < inum1; j++)
  {
      cout << Obj1[j]["firstName"].asString() << endl;
      cout << Obj1[j]["lastName"].asString() << endl;
  }
  //解析json对象
  const Json::Value Obj2 = value["c++"];
  cout << Obj2["hello"].asString() << endl;

  
  {
      "employees": [
          {
              "firstName": "Bill",
              "lastName": "Gates"
          },
          {
              "firstName": "George",
              "lastName": "Bush"
          },
          {
              "firstName": "Thomas",
              "lastName": "Carter"
          }
      ],
      "c++":{
          "hello":"world"
      }
  }
  */


  //先构建一个Json对象，此Json对象中含有数组，然后把Json对象序列化成字符串，代码如下：
  Json::Value roott;
  Json::Value arrayObj;
  Json::Value item;
  for (int i=0; i<10; i++)
  {
    item["key"] = i;
    arrayObj.append(item);
  }

  roott["key1"] = "value1";
  roott["key2"] = "value2";
  roott["array"] = arrayObj;
  roott.toStyledString();
  std::string out = roott.toStyledString();
  Json::StyledWriter styled_writer1;
  std::cout << styled_writer1.write(roott) << std::endl << std::endl;
  std::cout << out << std::endl;

/*
  Json::Value json_temp;      // 临时对象，供如下代码使用
  json_temp["name"] = Json::Value("huchao");
  json_temp["age"] = Json::Value(26);

  Json::Value root;  // 表示整个 json 对象
  root["key_string"] = Json::Value("value_string");         // 新建一个 Key（名为：key_string），赋予字符串值："value_string"。
  root["key_number"] = Json::Value(12345);            // 新建一个 Key（名为：key_number），赋予数值：12345。
  root["key_boolean"] = Json::Value(false);              // 新建一个 Key（名为：key_boolean），赋予bool值：false。
  root["key_double"] = Json::Value(12.345);            // 新建一个 Key（名为：key_double），赋予 double 值：12.345。
  root["key_object"] = json_temp;                           // 新建一个 Key（名为：key_object），赋予 json::Value 对象值。
  root["key_array"].append("array_string");             // 新建一个 Key（名为：key_array），类型为数组，对第一个元素赋值为字符串："array_string"。
  root["key_array"].append(1234);                           // 为数组 key_array 赋值，对第二个元素赋值为：1234。
  Json::ValueType type = root.type();                       // 获得 root 的类型，此处为 objectValue 类型。

  Json::StyledWriter styled_writer;
  std::cout << styled_writer.write(root) << std::endl << std::endl;


  Json::Value val;
  Json::StyledWriter style_writer;

  val["name"] = "xiaoli";
  //添加一个存储为double的数字
  val["pi"]["NAME"] = "SSM";
  val["pi"]["name"] = 3.141;
  // 添加一个布尔值 
  val["happy"] = true;
  //添加一个存储为std :: string的字符串
  val["name"] = "Niels";
  std::cout << val << std::endl;  //cout会自动把json序列化，以string形式输出


  std::string send_buf = style_writer.write(val);
  //int len = send(client_sockfd, send_buf.c_str(), send_buf.size(), 0);*/



  ros::init(argc, argv, "main");
  ros::NodeHandle n;
  printf("wenjian: %s  %s  %d  ",  __FILE__,  __FUNCTION__, __LINE__);
  ROS_INFO("[Z.H] %d", __LINE__);
  ROS_INFO_STREAM("[Z.H][" << __LINE__ << "] heh\n");



  
  
 
  


  // char buf[BUFSIZ]; //数据传送的缓冲区
  // printf("Enter string to send:\n");
  // scanf("%[^\n]",buf);//%[^\n]
  // printf("the buf entered length is:%d.\n", strlen(buf));

  // int send_len = strlen(buf)+1;
  // char *send_buff = (char *)malloc(send_len);
  // memset(send_buff, 0, send_len);
  // memcpy(send_buff,buf,strlen(buf));
  // send_buff[strlen(buf)]='\0';
  // printf("%s   . %d\n", send_buff, strlen(send_buff));
  // free(send_buff);



    // std::thread thread_consumer(consumer);
    // std::thread thread_producer(producer);

    // if (thread_consumer.joinable())
    //     thread_consumer.join();
    // if (thread_producer.joinable())
    //     thread_producer.join();
    // std::cout << "flag4:" << std::endl;




  // std::string s_s;
  // while(getline(cin,s_s))
  // {
  //   std::cout << s_s.find_first_of(' ') << std::endl;
  //   std::cout << s_s.length() << " " << s_s.find_last_of(' ') << " " << s_s.length() - s_s.find_last_of(' ') - 1 << std::endl;
  //   std::cout << s_s.find(" ") << std::endl;
  //   std::cout << s_s.substr(3,5) << std::endl;
  //   std::cout << s_s.size() << std::endl;
  // }
  
  // Foo f;
  // print(f);
  // std::vector<int> vv = {77,5,45,33,1,2,8,4,99};
  // std::sort(vv.begin(), vv.end(), [](int &a, int &b)-> int{return a < b;});
  // //std::sort(vv.begin(), vv.end(), greater<int>());
  // for(auto &i : vv)
  //   std::cout << i << std::endl;
  // int a = 6;
  // int b = 8;
  // auto sum = [](int a,int b) mutable->int{return a + b;};
  // printf("sum[%d]\n",sum(a,b));

  std::cout << "II" << std::endl;
  auto ss = [](int a, int b)->float {return a + b;};
  std::cout << ss(6,8) << std::endl;
  //YAML::Node yaml = YAML::LoadFile("/home/qzkj/Desktop/catkin_test/src/test/cfg/ethernet_communication.yaml");
  //
  //std::string s1 = yaml["ethernet_host"].as<std::string>();
  //int s2 = yaml["ethernet_port"].as<int>();
  
  // yaml["ethernet_host"] >> s1;
  // yaml["ethernet_port"] >> s2;

  //std::cout << s1 << std::endl << s2 << std::endl;
  

  
  // float i1 = yaml["car_info"]["car_width"].as<float>();
  // float i2 = yaml["car_info"]["car_length"].as<float>();

  // for(YAML::const_iterator it = yaml["car_info"].begin(); it != yaml["car_info"].end(); ++it)
  // {
  //   std::cout << it->first.as<std::string>() << ":" << it->second.as<float>() << endl;
  // }

  

  /*
  //--------------------------------------------------------------------------------------//
  std::string s = "$GPFPD,0,4416.400,0.000,-0.006,-0.436,24.5250599,118.0037320,0.00,0.000,0.000,0.000,0.000,0,0,0B*69\r\n";
  //std::string s = "0.000,0,0,00*46\n$GPFPD,0,136.250,0.000,-0.042,-0.415,0.0000000,0.0000000,0.00,0.000,0.000,0.000,";
  std::cout << "size: " << s.length() << std::endl;
  s.erase(std::remove(s.begin(), s.end(), '\r'), s.end());
  std::replace(s.begin(),s.end(),'\n',',');

  std::cout << "s: " << s << std::endl << s.size() << std::endl << std::endl; 
  std::vector<string> v_s;
  size_t j = 0;
  size_t k = 0;
  for(size_t i = 0; i < s.size(); ++i)
  {
    if((s[i] == ',') && (j < s.size() - 1))
    {
      string t = s.substr(j,k);
      v_s.push_back(t);
      k = 0;
      j = i + 1;
    }
    else
    {
      k++;
    }
  }

  for(auto &i : v_s)
    std::cout << i << std::endl; 
  std::cout << std::endl;   
  if(v_s.front() == "$GPFPD")
  {
    double d_yaw = atof(const_cast<const char *>(v_s.at(3).c_str()));
    double d_lati = atof(const_cast<const char *>(v_s.at(6).c_str())) * 10000000;
    double d_long = atof(const_cast<const char *>(v_s.at(7).c_str())) * 10000000;
    double d_ve = atof(const_cast<const char *>(v_s.at(9).c_str()));//东向速度m/s
    double d_vn = atof(const_cast<const char *>(v_s.at(10).c_str()));//北向速度
    double d_spd = std::sqrt(d_ve * d_ve + d_vn * d_vn); 
    std::string d_state = v_s.at(15).substr(0,2);
    //double dt = stringToNum<double>(t); 
    std::cout << "" << std::setprecision(15) << d_yaw << "  " << d_lati << " " << d_long << " " << d_state << std::endl;
  }*/


  
  //-------------------------------------------------------------------------------//

  /*
  time_t t;    //time_t是一种类型，定义time_t类型的t
  time(&t);    //取得当前时间
  printf("%s\n",ctime(&t));// ctime(&t)将日期转为字符串并打印

  std::string i = "hello";
  std::vector<std::string> v;
  v.push_back(i);
  std::cout << "i = " << i << std::endl;

  v.push_back(std::move(i));
  std::cout << "i = " << i << std::endl;

  std::deque<int> d = {1,2,3,4};
  d.emplace_front(5);
  d.insert(d.begin() + 2, 8);
  std::cout << "--" << d.front() << " " << d.back() << " " << d.at(2) << std::endl;

  

  double current_long = -1028927.202;
  double current_lati = 2984488.370;
  double current_yaw = 270.0;
  double p_x = -4;//left - right +
  double p_y = 5;
  double l = current_long + std::cos(current_yaw*M_PI/180) * p_x + std::sin(current_yaw*M_PI/180) * p_y;
  double la = current_lati - std::sin(current_yaw*M_PI/180) * p_x + std::cos(current_yaw*M_PI/180) * p_y;
  double y = current_yaw;
  std::cout << "AAA: " << std::setprecision(15) << l << "  " << la << "  " << y << std::endl;*/
 

  /*
  int socket_fd_ = -1;
  socket_fd_ = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);
  if(socket_fd_ == -1) 
  {
    printf("socket error!!!\n");
  }

  socklen_t rcvbuflen = 0;
  socklen_t rlen = sizeof(rcvbuflen);
  getsockopt(socket_fd_, SOL_SOCKET, SO_RCVBUF, (void *)&rcvbuflen, &rlen);
  printf("default receivebuf len: %d B\n", rcvbuflen);
  rcvbuflen = 425984;//max :212992 * 2 = 425984k
  setsockopt(socket_fd_, SOL_SOCKET, SO_RCVBUF, (void *)&rcvbuflen, rlen);
  getsockopt(socket_fd_, SOL_SOCKET, SO_RCVBUF, (void *)&rcvbuflen, &rlen);
  printf("now receivebuf len: %d B\n", rcvbuflen);

  socklen_t sendbuflen = 0;
  socklen_t slen = sizeof(sendbuflen);
  getsockopt(socket_fd_, SOL_SOCKET, SO_SNDBUF, (void *)&sendbuflen, &slen);
  printf("default sendbuf len: %d B\n", sendbuflen);
  sendbuflen = 425984;//max :212992 * 2 = 425984k
  setsockopt(socket_fd_, SOL_SOCKET, SO_SNDBUF, (void *)&sendbuflen, slen);
  getsockopt(socket_fd_, SOL_SOCKET, SO_SNDBUF, (void *)&sendbuflen, &slen);
  printf("now sendbuf len: %d B\n", sendbuflen);
  */
    //receive
    unsigned char can_data[8] = {0x17, 0x8C, 0xC2, 0x0F, 0xB7, 0xA2, 0x9A, 0x3F};

    double imu_yaw_ = (((uint16_t)((can_data[1] << 8) | can_data[0])) * 0.01);
                    ROS_INFO("[Z.H][can_net][can_net_rosinfoservice.cpp][%d]parse 0x10B: imu_yaw: %.3f", 
                            __LINE__, imu_yaw_);


    // long t_latitude, t_logitude;
    // t_latitude = (int8_t)can_data[3];
    // //printf("%#X\n", t_latitude);
    // for(int i=2;i>=0;i--)
    // {
    //     printf("%#X\n", t_latitude);
    //     t_latitude <<= 8;
    //     t_latitude |= can_data[i];
    //     //printf("%#X\n", t_latitude);
    //     //printf("111\n\n");
    // }
    // //int position_x = t_latitude;
    // t_logitude = (int8_t)can_data[7];
    // for(int i=6;i>3;i--)
    // {
    //     t_logitude <<= 8;
    //     t_logitude |= can_data[i];
    // }
    // //int position_y = t_logitude;
    // ROS_INFO("[Z.H]PARSE 0x30B: t_latitude: %d, t_logitude: %d",t_latitude,t_logitude);

    double imu_lon_ = 0;
    double imu_lat_ = 0;//before gauss

    long t_latitude = (int8_t)can_data[3];
    for(int i = 2; i >= 0; i--)
    {
        t_latitude <<= 8;
        t_latitude |= can_data[i];
    }
    imu_lat_ = t_latitude;//y
    long t_logitude = (int8_t)can_data[7];
    for(int i = 6; i > 3; i--)
    {
        t_logitude <<= 8;
        t_logitude |= can_data[i];
    }
    imu_lon_ = t_logitude;//x
    ROS_INFO("[Z.H][can_net][can_net_rosinfoservice.cpp][%d]parse 0x20B: imu_lat: %f, imu_lon: %f", 
            __LINE__, imu_lat_, imu_lon_);

                    
    double lat = -39.12345;
    double lon = -116.20125;

    int32_t t_lat = lat / 0.0000001;
    int32_t t_lon = lon / 0.0000001;
    printf("t_lat = %d(%#X), t_lon = %d(%#X). \n", t_lat, t_lat, t_lon, t_lon);

    unsigned char res[8];
    char temp_data = 0;
    for(size_t j = 0; j < 4; j++)
    {
        temp_data = t_lat & 0xff;
        res[j] = temp_data;
        t_lat = t_lat >> 8;
    }

    for(size_t j = 4; j < 8; j++)
    {
        temp_data = t_lon & 0xff;
        res[j] = temp_data;
        t_lon = t_lon >> 8;
    }

    for(int i = 0; i < 8; ++i)
    printf("res[%d] = %#X\n", i, res[i]);





	  //receive 1 angle_f = -38.1 满字节，符号位在字节头部 正确已验证
    unsigned char test0 = 0x83;//0xfd;
    unsigned char test1 = 0xfe;//0xeb; 
    double real = ((int16_t)((int8_t)test1 << 8 | test0)) * 0.1;
    printf("%#X\n", (int8_t)test1);
    printf("%#X\n", (int8_t)test1 << 8);
    printf("%#X\n", ((int8_t)test1 << 8 | test0));
    printf("%#X\n", (int16_t)((int8_t)test1 << 8 | test0));


    printf("RECEIVE 1 real: %f\n", real);

    //receive 2 lane_heading = -89.9 跨字节 符号位在某个字节的中间
    unsigned char test00 = 0x80;//0x7D;
    unsigned char test11 = 0x7C;//0x7C;0x72

    //此处必须把符号位移到字节头部才能转换成有符号的，否则不对
    double reall22 = (((int8_t)test11 << 4 << 4) | test00) * 0.1;//错
    double real2 = (((int8_t)(test11 << 4) << 4) | test00) * 0.1;

    double r2 = 0.01 * ((int8_t)test11 << 8 >> 4 | (test00 >> 4));
    double r22 = 0.01 * ((int8_t)test11 << 4 | (test00 >> 4));
    printf("RECEIVE 2 r2: %f\n", r2);
    printf("RECEIVE 2 r22: %f\n", r22);

    printf("%#X\n", (test11 << 4));
    printf("%#X\n\n", (int8_t)test11);
    printf("%#X\n", (int8_t)test11 << 4);
    printf("%#X\n\n", (int8_t)(test11 << 4));
    printf("%#X\n", (int8_t)test11 << 4 << 4);
    printf("%#X\n\n", (int8_t)(test11 << 4) << 4);
    printf("%#X\n", ((int8_t)test11 << 4 << 4 | test00));
    printf("%#X\n\n", ((int8_t)(test11 << 4) << 4 | test00));
    printf("RECEIVE 2 reall22: %f\n", reall22);
    printf("RECEIVE 2 real2: %f\n", real2);



    //send 1  正确已验证
    double angle_f  = -38.1;
    short angle_i = angle_f / 0.1;

    unsigned char res00 = angle_i & 0xff;
    unsigned char res11 = (angle_i >> 8) & 0xff;
    printf("SEND 1 angle_i = %d (0x%x), res00 = 0x%x, res11 = 0x%x\n", angle_i, angle_i, res00, res11);

    //send 2  正确已验证
    unsigned char res000 = static_cast<uint8_t>(static_cast<int16_t>((angle_f)/0.1) & 0xff);
    unsigned char res111 = static_cast<uint8_t>((static_cast<int16_t>((angle_f)/0.1) >> 8) & 0xff);
    printf("SEND 2 angle_i = %d (0x%x), res000 = 0x%x, res111 = 0x%x\n", static_cast<int16_t>((angle_f)/0.1), static_cast<int16_t>((angle_f)/0.1), res000, res111);


    //send 3  正确已验证
    double lane_heading = -89.6;//-89.9
    double lateral_distance = -5.21;

    int16_t lane_heading_i = lane_heading * 10;//lane_heading / 0.1;
    std::cout << lane_heading_i << "   " << lane_heading / 0.1 << std::endl;
    int16_t lateral_distance_i = lateral_distance / 0.01;
    
    unsigned char res0 = static_cast<uint8_t>(lane_heading_i & 0xff);//static_cast<uint8_t>(static_cast<int16_t>((lane_heading)/0.1) & 0xff);
    unsigned char res1 = static_cast<uint8_t>((((lane_heading_i >> 15) & 0x01) << 3) | ((lane_heading_i >> 8) & 0x07) | ((static_cast<int16_t>((lateral_distance)/0.01) & 0x0f) << 4));
    unsigned char res2 = static_cast<uint8_t>((static_cast<int16_t>((lateral_distance)/0.01) >> 4) & 0xff);
    printf("SEND 3 lane_heading_i = %d(%#X), lateral_distance_i = %d(%#X)\nSEND 3 res0 = %#X, res1 = %#X, res2 = %#X. \n", lane_heading_i, lane_heading_i, lateral_distance_i, lateral_distance_i, res0, res1, res2);

   
    

    /*
	  size_t k = 0;
	  std::cout << "k-1=" << k - 1 << std::endl;
    std::vector<double> v_dtc_shifted;
    for(float i = 0.5; i <= 4.0; i += 0.5)
    {
      v_dtc_shifted.push_back(i);
    }
    for(size_t k = 0; k < v_dtc_shifted.size(); ++k)
    {
      std::cout << v_dtc_shifted[k] << std::endl;
    }*/

    /*
	  int n, i, j;
    bool arr[100];
    while(cin >> n)
    {
        i = 0;
        while(n > 0)
        {
            arr[i] = bool(n % 2);
            i++;
            n /= 2;
        }
        for(j=i-1;j>=0;j--)
            cout << arr[j];
        cout << endl;
    }
    return 0;

	printf("AAA: %02X", -128);
	std::cout << std::endl;*/

  /*
  for(size_t i = 1; i <= 100; ++i)
  {
    v_angle_feedback.push_back(i);
    std::cout << "i = " << i << std::endl;
    if(v_angle_feedback.size() < 10 && v_angle_feedback.size() >= 0)
    {
      global_feedback_steering_angle = i;

    }
    else
    {
      std::cout << "ZZZ: " << v_angle_feedback.size() << " begin: " << v_angle_feedback.front() << " end: " << v_angle_feedback.back() << std::endl;
      double sum = std::accumulate(v_angle_feedback.begin(), v_angle_feedback.end(), 0.0);
      std::cout << "sum: " << sum << std::endl;
      global_feedback_steering_angle = sum/v_angle_feedback.size();
      v_angle_feedback.erase(v_angle_feedback.begin());
    }
    std::cout << "global_feedback_steering_angle:" << global_feedback_steering_angle << std::endl;
    

  }*/
  // double a = M_PI/2*M_PI/180;

  // double t=tan(a);
  // double n=cos(a);
  // double C1=1+3*pow(2.0,2)/4;

  

  /*
  double lo = -1028979.573;
  double la = 2984937.989;
  double y = 359.788;

  //currentPoint
  double lo1 = -1028979.703000;
  double la1 = 2984938.770000;
  double y1 = 358.840000;


  //get currentpoint info
  double dist_diff = ((lo - lo1) * std::cos(y1*M_PI/180)) - ((la - la1) * std::sin(y1*M_PI/180));
  double yaw_diff = getYawDiff(y, y1);// unit degree

  std::cout << "get currentpoint info :" << dist_diff << "  " << yaw_diff << std::endl;
  //get takeover currentpoint info
  
  double ret_takeOver_longitude = lo1 - std::cos(y1*M_PI/180) * 4;
  double ret_takeOver_latitude = la1 + std::sin(y1*M_PI/180) * 4;

  double dist_diff_takeover = ((lo - ret_takeOver_longitude) * std::cos(y1*M_PI/180)) - ((la - ret_takeOver_latitude) * std::sin(y1*M_PI/180));
  
  std::cout << "get takeover currentpoint info :" << dist_diff_takeover << "  " << yaw_diff << std::endl;

  for(int i = 0; i < 10; ++i)
  {
    static double speed_old = 0;
    double k = 1;
    std::cout << "1. k = " << k << std::endl;
    k = 0.9 * speed_old + 0.1 * k;
    speed_old = k;
    std::cout << "2. k = " << k << std::endl;

  }*/



  /*
  unsigned int target_workmode_cmd = 2;
  unsigned int gear_cmd = 2;// 1 N 2 D 3 R
  bool fullstop_flag = true;
  bool enable_aeb_flag = true;
  bool enable_rosconfig_flag = true;//1 use ros 0 use bottom default

  printf("AAA: %02X.\n", static_cast<uint8_t>((target_workmode_cmd & 0x03) | ((gear_cmd & 0x03) << 2) | ((fullstop_flag & 0x01) << 4) | ((enable_aeb_flag & 0x01) << 5) | ((enable_rosconfig_flag & 0x01) << 7)));
  double brake_kp = 2.4;
  double brake_ki = 1.2;
  std::cout << static_cast<uint16_t>(brake_kp/0.02) << std::endl;
  std::cout << static_cast<uint16_t>(brake_ki/0.01) << std::endl;
  */


  

  /*
  double a = -0.123456678;
  std::cout << std::fabs(a) << std::endl;
  std::cout << std::abs(a) << std::endl;
  std::cout << std::tan(0/26.5*M_PI/180) << std::endl;
  double global_weight_odomheading = 0.5;
  double lastyaw = 358;
  double yaw_temp = 1;
  if(std::fabs(yaw_temp - lastyaw) > 180)
  {
    double temp_max = std::max(yaw_temp, lastyaw);

    if(temp_max == yaw_temp)
    {
      yaw_temp = yaw_temp - 360;

    }
    else
    {
      lastyaw = lastyaw - 360;
    }
  }
  double temp_out = global_weight_odomheading * (lastyaw + (((std::abs(5/3.6))*(std::tan(60/26.5*M_PI/180))*0.05/5.8)*180/M_PI)) + (1 - global_weight_odomheading) * yaw_temp;
  if(temp_out < 0)
    temp_out = temp_out + 360;
  std::cout << " yaw:" << temp_out << std::endl;
  */
  /*
	// ROS initiate
	ros::init(argc, argv, "main");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<nav_msgs::GridCells>("/gridCell", 1);
  ros::Rate loop_rate(20);

  while(ros::ok())
  {
    std::cout << "AAA: " << ros::Time::now() << std::endl;
  	nav_msgs::GridCells cells;
  	cells.header.frame_id = "rslidar";
  	cells.header.stamp = ros::Time::now();
  	cells.cell_height = 1;
  	cells.cell_width = 1;
  	cells.cells.clear();

  	geometry_msgs::Point p;
  	p.x = 0;
  	p.y = 1;
  	p.z = 0;
  	cells.cells.push_back(p);

  	p.x = 3;
  	p.y = 2;
  	p.z = 0;
  	cells.cells.push_back(p);

  	p.x = 3;
  	p.y = 7;
  	p.z = 0;
  	cells.cells.push_back(p);

  	p.x = 3.5;
  	p.y = 4.2;
  	p.z = 0;
  	cells.cells.push_back(p);

  	pub.publish(cells);
  	ros::spinOnce();
    loop_rate.sleep();

  }*/


  
  //ros::Subscriber grid_sub = n.subscribe<nav_msgs::OccupancyGrid>("gridmap", 1, boost::bind(&gridCallback, _1, &lidarData));

  //ros::spin();   


  /*

	// ROS node create
	ros::NodeHandle nh;
	LidarData lidarData;

  ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 10);
  //ros::Subscriber grid_sub = n.subscribe<nav_msgs::OccupancyGrid>("gridmap", 1, boost::bind(&gridCallback, _1, &lidarData));
  //grid_pub = n.advertise<nav_msgs::OccupancyGrid>("grid", 10);
  //ros::spin();
  ros::Rate loop_rate(500);
  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(180/180*M_PI);
  while(ros::ok()){
    //filling the odometry
        nav_msgs::Odometry odom;
        odom.header.stamp = ros::Time::now();
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";


        // position
        odom.pose.pose.position.x = 123.45;
        odom.pose.pose.position.y = 234.56;
        odom.pose.pose.position.z = 1;
        odom.pose.pose.orientation = odom_quat;



    //velocity
        odom.twist.twist.linear.x = 0.0;
        odom.twist.twist.linear.y = 0.0;
        odom.twist.twist.linear.z = 0.0;
        odom.twist.twist.angular.x = 0.0;
        odom.twist.twist.angular.y = 0.0;
        odom.twist.twist.angular.z = 0.0;

    

    // publishing the odometry and the new tf
  //  broadcaster.sendTransform(odom_trans);
        odom_pub.publish(odom);
    ros::spinOnce();
    loop_rate.sleep();
  }
  */

    


    /*
	for(int i = 0; i < 5; ++i)
	{
		double vel_position_x = 10000;
		double vel_position_y = 20000;
		// low prass filter
	    static double last_x = 0;
	    static double last_y = 0;
	    vel_position_x = filter_rate0 * last_x + (1-filter_rate0) * vel_position_x;
	    vel_position_y = filter_rate0 * last_y + (1-filter_rate0) * vel_position_y;
	    std::cout << "A:" << last_x << "--" << last_y << "--" << vel_position_x << "--" << vel_position_y << std::endl;
	    last_x = vel_position_x;
	    last_y = vel_position_y;
	    std::cout << "AA:" << last_x << "--" << last_y << "--" << vel_position_x << "--" << vel_position_y << std::endl;
	    //ROS_INFO("AAA:%lf--%lf--%ld--%ld",last_x,last_y,vel_position_x,vel_position_y);

	    static double llast_x = 0;
	    static double llast_y = 0;
	    vel_position_x = filter_rate1 * llast_x + (1-filter_rate1) * vel_position_x;
	    vel_position_y = filter_rate1 * llast_y + (1-filter_rate1) * vel_position_y;
	    std::cout << "B:" << llast_x << "--" << llast_y << "--" << vel_position_x << "--" << vel_position_y << std::endl;
	    llast_x = vel_position_x;
	    llast_y = vel_position_y;
	    std::cout << "BB:" << llast_x << "--" << llast_y << "--" << vel_position_x << "--" << vel_position_y << std::endl;
	    //ROS_INFO("BBB:%lf--%lf--%ld--%ld",llast_x,llast_y,vel_position_x,vel_position_y);

	}*/
	

	/*

	//ros::Time begin = ros::Time::now();
	time_t tt;
    time( &tt );
    tt = tt + 8*3600;  // transform the time zone
    tm* t= gmtime( &tt );
    //cout << tt << endl;

    printf("%d-%02d-%02d %02d:%02d:%02d\n",
           t->tm_year + 1900,
           t->tm_mon + 1,
           t->tm_mday,
           t->tm_hour,
           t->tm_min,
           t->tm_sec);

	ros::Publisher virtual_curve_pub_ = n.advertise<geometry_msgs::PoseArray>("virtual_curve", 1);

	while(ros::ok()){

		//ROS_INFO("Listening <<<<<<<<<<<<<<<<");

		Curve virtualCurve = createVirtualCurve();
		for(int i = 0; i < 40000; ++i)
		{
			//std::cout <<"i=" << i << " ; i%100=" << i % 100 << std::endl;
			if(i < 1000){
				curvePublisher(virtual_curve_pub_, virtualCurve);
			}
			else{
				//std::cout << "I not pub!!!---" << std::endl;
			}
		}
		//ROS_INFO("Spinning  --------------------------------\n");
		ros::spinOnce();
	}
  */
	/*
	std::cout << "ZZZ" << atan(1) << std::endl;
  std::cout << "Z:" << std::atan2(1.732, 1)*180/M_PI << std::endl;
	
    
	std::vector<int> v = {1,2,3};
	ROS_INFO("AAA:%d", v.front());
	v.erase(v.begin());
	ROS_INFO("BBB:%d", v.front());
	std::cout << INT_MAX << std::endl;
	ros::init(argc, argv, "test");
	ROS_INFO("writing sorted tentacles to file\n");
	ROS_INFO_STREAM("[ "<< 20 << "%" << "\t" << "]");
	ROS_INFO("\n\n\n");
	std::vector<float> params = {0, 0.0, 0, 0.0, 0.0, 60.0}; 
	geometry_msgs::Point p;
	p.x = 2;
	p.y = 0;
	p.z = 0.0;
	Eigen::Vector4f input;
	input << p.x, p.y, p.z, 1;

	// NO.1
  Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
	float tx = params[0];
	float ty = params[1];
	float tz = params[2];
	float rx = params[3]*PI_OVER_180;
	float ry = params[4]*PI_OVER_180;
	float rz = params[5]*PI_OVER_180;

	Eigen::AngleAxisf init_rotation_x(rx, Eigen::Vector3f::UnitX());
	Eigen::AngleAxisf init_rotation_y(ry, Eigen::Vector3f::UnitY());
	Eigen::AngleAxisf init_rotation_z(rz, Eigen::Vector3f::UnitZ());

	Eigen::Translation3f init_translation(tx, ty, tz);

	Eigen::Matrix4f init_guess =
	      (init_translation * init_rotation_x * init_rotation_y * init_rotation_z).matrix();//切记：矩阵相乘的顺序不要颠倒
	transform_1 = init_guess;
	std::cout << "1:" << std::endl << transform_1.matrix() << std::endl;
	Eigen::Vector4f output_1 = transform_1 * input;
	std::cout << std::endl << output_1 << std::endl;
   
   

	//NO.2
	Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();

	transform_2.translation() << params[0], params[1], params[2];

	// 和前面一样的旋转; Z 轴上旋转 theta 弧度
	transform_2.rotate (Eigen::AngleAxisf(params[3]*PI_OVER_180, Eigen::Vector3f::UnitX())
	                  * Eigen::AngleAxisf(params[4]*PI_OVER_180, Eigen::Vector3f::UnitY())
	                  * Eigen::AngleAxisf(params[5]*PI_OVER_180, Eigen::Vector3f::UnitZ()));

	// 打印变换矩阵
	std::cout << "2:" << std::endl << transform_2.matrix() << std::endl;
	Eigen::Vector4f output_2 = transform_2 * input;
	std::cout << std::endl << output_2 << std::endl;
  
	//NO.3
	Eigen::Matrix3f rot;
	Eigen::Vector3f tra, trans;
	geometry_msgs::Point output_3;

	rot = Eigen::AngleAxisf((params[3]* PI_OVER_180), Eigen::Vector3f::UnitX())
	    * Eigen::AngleAxisf((params[4]* PI_OVER_180), Eigen::Vector3f::UnitY())
	    * Eigen::AngleAxisf((params[5]* PI_OVER_180), Eigen::Vector3f::UnitZ());
	tra << p.x, p.y, p.z;
	trans = rot * tra;

	output_3.x = trans(0) + params[0];
	output_3.y = trans(1) + params[1];
	output_3.z = trans(2) + params[2];
	// 打印变换矩阵
	std::cout << "3:" << std::endl << rot.matrix() << std::endl;
	std::cout << output_3 << std::endl;
    
    //NO.4
	Eigen::Vector3f pt0,pt1;
	geometry_msgs::Point output_4;
  pt0 << p.x, p.y, p.z;
  Eigen::Matrix3f rotX, rotY, rotZ;
  rotX << 1,0,0, 0,cos(params[3]* PI_OVER_180),-sin(params[3]* PI_OVER_180), 0,sin(params[3]* PI_OVER_180),cos(params[3]* PI_OVER_180);
  rotY << cos(params[4]* PI_OVER_180),0,sin(params[4]* PI_OVER_180), 0,1,0, -sin(params[4]* PI_OVER_180),0,cos(params[4]* PI_OVER_180);
  rotZ << cos(params[5]* PI_OVER_180),-sin(params[5]* PI_OVER_180),0, sin(params[5]* PI_OVER_180),cos(params[5]* PI_OVER_180),0, 0,0,1;

  Eigen::Matrix3f rot1 = rotX*rotY*rotZ;
  pt1 = rot1*pt0; //H P B
  output_4.x = pt1(0) + params[0];
  output_4.y = pt1(1) + params[1];
  output_4.z = pt1(2) + params[2];

  // 打印变换矩阵
	std::cout << "4:" << std::endl << rot1.matrix() << std::endl;
	std::cout << output_4 << std::endl;

  //NO.5
  geometry_msgs::Point output_5;
  output_5.x = p.x * cos(-params[5]*PI_OVER_180) + p.y * sin(-params[5]*PI_OVER_180)- params[2];
  output_5.y = p.y * cos(-params[5]*PI_OVER_180) - p.x * sin(-params[5]*PI_OVER_180)- params[2];
  output_5.z = p.z;
  std::cout << "5:" << std::endl;
  std::cout << output_5 << std::endl;
	*/

	return 0;
}