/*
使用正态分布变换进行配准的实验 。其中room_scan1.pcd  room_scan2.pcd这些点云包含同一房间360不同视角的扫描数据
*/
//PCL激光点云 的一些操作
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/transforms.h>
#include <pcl/console/time.h>
#include <pcl/segmentation/organized_connected_component_segmentation.h>
#include <pcl/search/octree.h>
#include <pcl/search/kdtree.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/kdtree/kdtree.h>


typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<PointType> PointCloud;
typedef pcl::PointCloud<PointType>::Ptr PointCloudPtr;
typedef pcl::PointCloud<PointType>::ConstPtr PointCloudConstPtr;



#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/registration/ndt.h>      //NDT(正态分布)配准类头文件
#include <pcl/filters/approximate_voxel_grid.h>   //滤波类头文件  （使用体素网格过滤器处理的效果比较好）

#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

int main (int argc, char** argv)
{
    // 加载房间的第一次扫描点云数据作为目标
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ> ("room_scan1.pcd", *target_cloud) == -1)
    {
      PCL_ERROR ("Couldn't read file room_scan1.pcd \n");
      return (-1);
    }
    std::cout << "Loaded " << target_cloud->size () << " data points from room_scan1.pcd" << std::endl;

    // 加载从新视角得到的第二次扫描点云数据作为源点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ> ("room_scan2.pcd", *input_cloud) == -1)
    {
      PCL_ERROR ("Couldn't read file room_scan2.pcd \n");
      return (-1);
    }
    std::cout << "Loaded " << input_cloud->size () << " data points from room_scan2.pcd" << std::endl;
    //以上的代码加载了两个PCD文件得到共享指针，后续配准是完成对源点云到目标点云的参考坐标系的变换矩阵的估计，得到第二组点云变换到第一组点云坐标系下的变换矩阵
    // 将输入的扫描点云数据过滤到原始尺寸的10%以提高匹配的速度，只对源点云进行滤波，减少其数据量，而目标点云不需要滤波处理
    //因为在NDT算法中在目标点云对应的体素网格数据结构的统计计算不使用单个点，而是使用包含在每个体素单元格中的点的统计数据
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
    approximate_voxel_filter.setLeafSize (0.2, 0.2, 0.2);
    approximate_voxel_filter.setInputCloud (input_cloud);
    approximate_voxel_filter.filter (*filtered_cloud);
    std::cout << "Filtered cloud contains " << filtered_cloud->size ()
              << " data points from room_scan2.pcd" << std::endl;

    // 初始化正态分布(NDT)对象
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;

    // 根据输入数据的尺度设置NDT相关参数

    ndt.setTransformationEpsilon (0.01);   //为终止条件设置最小转换差异

    ndt.setStepSize (0.1);    //为more-thuente线搜索设置最大步长

    ndt.setResolution (1.0);   //设置NDT网格网格结构的分辨率（voxelgridcovariance）
    //以上参数在使用房间尺寸比例下运算比较好，但是如果需要处理例如一个咖啡杯子的扫描之类更小的物体，需要对参数进行很大程度的缩小

    //设置匹配迭代的最大次数，这个参数控制程序运行的最大迭代次数，一般来说这个限制值之前优化程序会在epsilon变换阀值下终止
    //添加最大迭代次数限制能够增加程序的鲁棒性阻止了它在错误的方向上运行时间过长
    ndt.setMaximumIterations (35);

    ndt.setInputSource (filtered_cloud);  //源点云
    // Setting point cloud to be aligned to.
    ndt.setInputTarget (target_cloud);  //目标点云

    // 设置使用机器人测距法得到的粗略初始变换矩阵结果
    Eigen::AngleAxisf init_rotation (0.6931, Eigen::Vector3f::UnitZ ());
    Eigen::Translation3f init_translation (1.79387, 0.720047, 0);
    Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix ();

    // 计算需要的刚体变换以便将输入的源点云匹配到目标点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    ndt.align (*output_cloud, init_guess);
     //这个地方的output_cloud不能作为最终的源点云变换，因为上面对点云进行了滤波处理
    std::cout << "Normal Distributions Transform has converged:" << ndt.hasConverged ()
              << " score: " << ndt.getFitnessScore () << std::endl;

    // 使用创建的变换对为过滤的输入点云进行变换
    pcl::transformPointCloud (*input_cloud, *output_cloud, ndt.getFinalTransformation ());

    // 保存转换后的源点云作为最终的变换输出
    pcl::io::savePCDFileASCII ("room_scan2_transformed.pcd", *output_cloud);

    // 初始化点云可视化对象
    boost::shared_ptr<pcl::visualization::PCLVisualizer>
    viewer_final (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer_final->setBackgroundColor (0, 0, 0);  //设置背景颜色为黑色

    // 对目标点云着色可视化 (red).
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
    target_color (target_cloud, 255, 0, 0);
    viewer_final->addPointCloud<pcl::PointXYZ> (target_cloud, target_color, "target cloud");
    viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                    1, "target cloud");

    // 对转换后的源点云着色 (green)可视化.
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
    output_color (output_cloud, 0, 255, 0);
    viewer_final->addPointCloud<pcl::PointXYZ> (output_cloud, output_color, "output cloud");
    viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                    1, "output cloud");

    // 启动可视化
    viewer_final->addCoordinateSystem (1.0);  //显示XYZ指示轴
    viewer_final->initCameraParameters ();   //初始化摄像头参数

    // 等待直到可视化窗口关闭
    while (!viewer_final->wasStopped ())
    {
      viewer_final->spinOnce (100);
      boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }

    return (0);
}

//////////////////////////////////////////////////////////////////////
//////////////////---参考一些写法----//////////////////////////////////
    // 加载房间的第一次扫描点云数据作为目标
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ> ("room_scan1.pcd", *target_cloud) == -1)
    {
    PCL_ERROR ("Couldn't read file room_scan1.pcd \n");
    return (-1);
    }
    std::cout << "Loaded " << target_cloud->size () << " data points from room_scan1.pcd" << std::endl;

    // 加载从新视角得到的第二次扫描点云数据作为源点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ> ("room_scan2.pcd", *input_cloud) == -1)
    {
    PCL_ERROR ("Couldn't read file room_scan2.pcd \n");
    return (-1);
    }
    std::cout << "Loaded " << input_cloud->size () << " data points from room_scan2.pcd" << std::endl;
    //以上的代码加载了两个PCD文件得到共享指针，后续配准是完成对源点云到目标点云的参考坐标系的变换矩阵的估计，得到第二组点云变换到第一组点云坐标系下的变换矩阵
  
  
  
  
    pcl::PCLPointCloud2::Ptr cloud_blob (new pcl::PCLPointCloud2), cloud_filtered_blob (new pcl::PCLPointCloud2);//申明滤波前后的点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>), cloud_p (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);

    // 读取PCD文件
    pcl::PCDReader reader;
    reader.read ("table_scene_lms400.pcd", *cloud_blob);
    //统计滤波前的点云个数
    std::cerr << "PointCloud before filtering: " << cloud_blob->width * cloud_blob->height << " data points." << std::endl;

    // 创建体素栅格下采样: 下采样的大小为1cm
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;  //体素栅格下采样对象
    sor.setInputCloud (cloud_blob);             //原始点云
    sor.setLeafSize (0.01f, 0.01f, 0.01f);    // 设置采样体素大小
    sor.filter (*cloud_filtered_blob);        //保存

    // 转换为模板点云
    pcl::fromPCLPointCloud2 (*cloud_filtered_blob, *cloud_filtered);

    std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height << " data points." << std::endl;

    // 保存下采样后的点云
    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZ> ("table_scene_lms400_downsampled.pcd", *cloud_filtered, false);
    
    
/************************************************
关于如何使用PCL在ROS 中，实现简单的数据转化
时间：2017.3.31

****************************************************/


#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/cloud_viewer.h>

    ros::Publisher pub;
  

    pcl::visualization::CloudViewer viewer("Cloud Viewer");

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
    // 创建一个输出的数据格式
    sensor_msgs::PointCloud2 output;  //ROS中点云的数据格式
    //对数据进行处理
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

    output = *input;

    pcl::fromROSMsg(output,*cloud);


     //blocks until the cloud is actually rendered
    viewer.showCloud(cloud);


    pub.publish (output);
}



int main (int argc, char** argv)
{ 


    // Initialize ROS
    ros::init (argc, argv, "my_pcl_tutorial");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);
    ros::Rate loop_rate(100);
    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);




    // Spin
    ros::spin ();
    /*
    while (!viewer.wasStopped ())
    {

    } 
    */

 
}
