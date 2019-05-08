/**
 *旋转矩阵（R），旋转向量（V）和四元数（Q）在Eigen中转换关系的总结:https://blog.csdn.net/u011092188/article/details/77430988
 *旋转矩阵（R），旋转向量（V）和四元数（Q）分别通过自身初始化自己的方式，也就是第一分部分代码对旋转矩阵（R），旋转向量（V）和四元数（Q）赋值的第一种方式。
 *旋转矩阵（3X3）:Eigen::Matrix3d
 *旋转向量（3X1）:Eigen::AngleAxisd
 *四元数（4X1）:Eigen::Quaterniond
 *平移向量（3X1）:Eigen::Vector3d
 *变换矩阵（4X4）:Eigen::Isometry3d
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
#include <Eigen/Dense>


using namespace std;
using namespace Eigen;

int main(int argc, char **argv) {


    //下面三个变量作为下面演示的中间变量

    AngleAxisd t_V(M_PI / 4, Vector3d(0, 0, 1));
    Matrix3d t_R = t_V.matrix();
    Quaterniond t_Q(t_V);


    //对旋转向量（轴角）赋值的三大种方法

    //1.使用旋转的角度和旋转轴向量（此向量为单位向量）来初始化角轴
    AngleAxisd V1(M_PI / 4, Vector3d(0, 0, 1));//以（0,0,1）为旋转轴，旋转45度
    cout << "Rotation_vector1" << endl << V1.matrix() << endl;

    //2.使用旋转矩阵转旋转向量的方式

    //2.1 使用旋转向量的fromRotationMatrix()函数来对旋转向量赋值（注意此方法为旋转向量独有,四元数没有）
    AngleAxisd V2;
    V2.fromRotationMatrix(t_R);
    cout << "Rotation_vector2" << endl << V2.matrix() << endl;

    //2.2 直接使用旋转矩阵来对旋转向量赋值
    AngleAxisd V3;
    V3 = t_R;
    cout << "Rotation_vector3" << endl << V3.matrix() << endl;

    //2.3 使用旋转矩阵来对旋转向量进行初始化
    AngleAxisd V4(t_R);
    cout << "Rotation_vector4" << endl << V4.matrix() << endl;

    //3. 使用四元数来对旋转向量进行赋值

    //3.1 直接使用四元数来对旋转向量赋值
    AngleAxisd V5;
    V5 = t_Q;
    cout << "Rotation_vector5" << endl << V5.matrix() << endl;

    //3.2 使用四元数来对旋转向量进行初始化
    AngleAxisd V6(t_Q);
    cout << "Rotation_vector6" << endl << V6.matrix() << endl;


//------------------------------------------------------

    //对四元数赋值的三大种方法（注意Eigen库中的四元数前三维是虚部,最后一维是实部）

    //1.使用旋转的角度和旋转轴向量（此向量为单位向量）来初始化四元数,即使用q=[cos(A/2),n_x*sin(A/2),n_y*sin(A/2),n_z*sin(A/2)]
    Quaterniond Q1(cos((M_PI / 4) / 2), 0 * sin((M_PI / 4) / 2), 0 * sin((M_PI / 4) / 2), 1 * sin((M_PI / 4) / 2));//以（0,0,1）为旋转轴，旋转45度
    //第一种输出四元数的方式
    cout << "Quaternion1" << endl << Q1.coeffs() << endl;

    //第二种输出四元数的方式
    cout << Q1.x() << endl << endl;
    cout << Q1.y() << endl << endl;
    cout << Q1.z() << endl << endl;
    cout << Q1.w() << endl << endl;

    //2. 使用旋转矩阵转四元數的方式

    //2.1 直接使用旋转矩阵来对旋转向量赋值
    Quaterniond Q2;
    Q2 = t_R;
    cout << "Quaternion2" << endl << Q2.coeffs() << endl;


    //2.2 使用旋转矩阵来对四元數进行初始化
    Quaterniond Q3(t_R);
    cout << "Quaternion3" << endl << Q3.coeffs() << endl;

    //3. 使用旋转向量对四元数来进行赋值

    //3.1 直接使用旋转向量对四元数来赋值
    Quaterniond Q4;
    Q4 = t_V;
    cout << "Quaternion4" << endl << Q4.coeffs() << endl;

    //3.2 使用旋转向量来对四元数进行初始化
    Quaterniond Q5(t_V);
    cout << "Quaternion5" << endl << Q5.coeffs() << endl;



//----------------------------------------------------

    //对旋转矩阵赋值的三大种方法

    //1.使用旋转矩阵的函数来初始化旋转矩阵
    Matrix3d R1=Matrix3d::Identity();
    cout << "Rotation_matrix1" << endl << R1 << endl;

    //2. 使用旋转向量转旋转矩阵来对旋转矩阵赋值

    //2.1 使用旋转向量的成员函数matrix()来对旋转矩阵赋值
    Matrix3d R2;
    R2 = t_V.matrix();
    cout << "Rotation_matrix2" << endl << R2 << endl;

    //2.2 使用旋转向量的成员函数toRotationMatrix()来对旋转矩阵赋值
    Matrix3d R3;
    R3 = t_V.toRotationMatrix();
    cout << "Rotation_matrix3" << endl << R3 << endl;

    //3. 使用四元数转旋转矩阵来对旋转矩阵赋值

    //3.1 使用四元数的成员函数matrix()来对旋转矩阵赋值
    Matrix3d R4;
    R4 = t_Q.matrix();
    cout << "Rotation_matrix4" << endl << R4 << endl;

    //3.2 使用四元数的成员函数toRotationMatrix()来对旋转矩阵赋值
    Matrix3d R5;
    R5 = t_Q.toRotationMatrix();
    cout << "Rotation_matrix5" << endl << R5 << endl;

    return 0;


}

#include <iostream>
#include <Eigen/Dense>


using namespace std;
using namespace Eigen;

int main(int argc, char **argv) {

    Eigen::Vector3f e,p;
    p.x() = 4.6;
    p.y() = 1.25;
    p.z() = 0;
    e.x() = 41.9 * M_PI / 180;   //yaw 偏航角
    e.y() = -1 * M_PI / 180;     //pitch 俯仰角 
    e.z() = -0.6 * M_PI / 180;   //roll 水平角

    Eigen::Matrix3f rot;
    rot = Eigen::AngleAxisf(e(0), Eigen::Vector3d::UnitZ())//使用旋转的角度和旋转轴向量（此向量为单位向量）来初始化角轴:AngleAxisd V1(M_PI / 4, Vector3d(0, 0, 1));//以（0,0,1）为旋转轴，旋转45度
        * Eigen::AngleAxisf(e(1), Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisf(e(2), Eigen::Vector3d::UnitX());
    
    Eigen::Matrix4f m;  //1.使用旋转矩阵的函数来初始化旋转矩阵:Matrix3d R1=Matrix3d::Identity();
    m.setIdentity();
    m.block<3, 3>(0, 0) = rot;
    m.block<3, 1>(0, 3) = p;

    Eigen::Affine3f vlp;//仿射变换
    vlp.matrix() = m;



    PointCloud pointcloud;
    pcl::fromROSMsg(*msg, pointcloud);

    PointCloudPtr base(new PointCloud);
    pcl::transformPointCloud(pointcloud, *base, vlp);
}
//这里主要介绍点云的平移与旋转，话不多说，代码如下：该代码可实现点云绕z轴旋转90°，绕x轴平移2.5，
//
#include <iostream>
 
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
 
// This function displays the help
void
showHelp(char * program_name)
{
  std::cout << std::endl;
  std::cout << "Usage: " << program_name << " cloud_filename.[pcd|ply]" << std::endl;
  std::cout << "-h:  Show this help." << std::endl;
}
// This is the main function
int
main (int argc, char** argv)
{
  // Show help
  if (pcl::console::find_switch (argc, argv, "-h") || pcl::console::find_switch (argc, argv, "--help")) {
    showHelp (argv[0]);
    return 0;
  }
  // Fetch point cloud filename in arguments | Works with PCD and PLY files
  std::vector<int> filenames;
  bool file_is_pcd = false;
 
  filenames = pcl::console::parse_file_extension_argument (argc, argv, ".ply");
 
  if (filenames.size () != 1)  {
    filenames = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");
 
    if (filenames.size () != 1) {
      showHelp (argv[0]);
      return -1;
    } else {
      file_is_pcd = true;
    }
  }
 
  // Load file | Works with PCD and PLY files
  pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
 
  if (file_is_pcd) {
    if (pcl::io::loadPCDFile (argv[filenames[0]], *source_cloud) < 0)  {
      std::cout << "Error loading point cloud " << argv[filenames[0]] << std::endl << std::endl;
      showHelp (argv[0]);
      return -1;
    }
  } else {
    if (pcl::io::loadPLYFile (argv[filenames[0]], *source_cloud) < 0)  {
      std::cout << "Error loading point cloud " << argv[filenames[0]] << std::endl << std::endl;
      showHelp (argv[0]);
      return -1;
    }
  }
  /* Reminder: how transformation matrices work :
           |-------> This column is the translation
    | 1 0 0 x |  \
    | 0 1 0 y |   }-> The identity 3x3 matrix (no rotation) on the left
    | 0 0 1 z |  /
    | 0 0 0 1 |    -> We do not use this line (and it has to stay 0,0,0,1)
 
    METHOD #1: Using a Matrix4f
    This is the "manual" method, perfect to understand but error prone !
  */
  Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
 
  // Define a rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)
  float theta = M_PI/2; // The angle of rotation in radians
  transform_1 (0,0) = cos (theta);
  transform_1 (0,1) = -sin(theta);
  transform_1 (1,0) = sin (theta);
  transform_1 (1,1) = cos (theta);
  //    (row, column)
 
  // Define a translation of 2.5 meters on the x axis.
  transform_1 (0,3) = 2.5;
 
  // Print the transformation
  printf ("Method #1: using a Matrix4f\n");
  std::cout << transform_1 << std::endl;
  /*  METHOD #2: Using a Affine3f
    This method is easier and less error prone
  */
  Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
  // Define a translation of 2.5 meters on the x axis.
  transform_2.translation() << 2.5, 0.0, 0.0;
  // The same rotation matrix as before; theta radians arround Z axis
  transform_2.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitZ()));
  // Print the transformation
  printf ("\nMethod #2: using an Affine3f\n");
  std::cout << transform_2.matrix() << std::endl;
  // Executing the transformation
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  // You can either apply transform_1 or transform_2; they are the same
  pcl::transformPointCloud (*source_cloud, *transformed_cloud, transform_2);
 
  // Visualization
  printf(  "\nPoint cloud colors :  white  = original point cloud\n"
      "                        red  = transformed point cloud\n");
  pcl::visualization::PCLVisualizer viewer ("Matrix transformation example");
 
   // Define R,G,B colors for the point cloud
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler (source_cloud, 255, 255, 255);
  // We add the point cloud to the viewer and pass the color handler
  viewer.addPointCloud (source_cloud, source_cloud_color_handler, "original_cloud");
 
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler (transformed_cloud, 230, 20, 20); // Red
  viewer.addPointCloud (transformed_cloud, transformed_cloud_color_handler, "transformed_cloud");
 
  viewer.addCoordinateSystem (1.0, "cloud", 0);
  viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud");
  //viewer.setPosition(800, 400); // Setting visualiser window position
 
  while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
    viewer.spinOnce ();
  }
 
  return 0;
}
