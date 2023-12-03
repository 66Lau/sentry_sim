#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <std_msgs/Bool.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/ModelStates.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Joy.h>
#include <visualization_msgs/Marker.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>



/*
此文件的主要作用就是控制小车的移动，并发布相关信息
订阅：
  激光雷达的点云
  地形的点云
  速度话题
发布：
  registered scan
  里程计信息
  map到sensor的tf转换


如果移植到我的系统中，只需要订阅雷达的点云数据并且发布registered scan到map frame，同时发布里程计信息，还有map到sensor的tf转换
目前我的系统中：/A/car0/odom 的父坐标系是odom，子坐标系是base_link

*/
using namespace std;

const double PI = 3.1415926;

bool use_gazebo_time = false;
double cameraOffsetZ = 0;
double sensorOffsetX = 0;
double sensorOffsetY = 0;
double vehicleHeight = 0.75;
double terrainVoxelSize = 0.05;
double groundHeightThre = 0.1;
bool adjustZ = false;
double terrainRadiusZ = 0.5;
int minTerrainPointNumZ = 10;
double smoothRateZ = 0.2;
bool adjustIncl = false;
double terrainRadiusIncl = 1.5;
int minTerrainPointNumIncl = 500;
double smoothRateIncl = 0.2;
double InclFittingThre = 0.2;
double maxIncl = 30.0;

geometry_msgs::Quaternion Odom_quat_get;
geometry_msgs::Point Odom_pose_get;

const int systemDelay = 20;
int systemInitCount = 0;
bool systemInited = false;

pcl::PointCloud<pcl::PointXYZI>::Ptr scanData(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloudIncl(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloudDwz(new pcl::PointCloud<pcl::PointXYZI>());

std::vector<int> scanInd;

ros::Time odomTime;



float vehicleX = 0;
float vehicleY = 0;
float vehicleZ = 0;
float vehicleRoll = 0;
float vehiclePitch = 0;
float vehicleYaw = 0;


float vehicleYawRate = 0;
float vehicleSpeed = 0;

float terrainZ = 0;
float terrainRoll = 0;
float terrainPitch = 0;

const int stackNum = 400;
float vehicleXStack[stackNum];
float vehicleYStack[stackNum];
float vehicleZStack[stackNum];
float vehicleRollStack[stackNum];
float vehiclePitchStack[stackNum];
float vehicleYawStack[stackNum];
float terrainRollStack[stackNum];
float terrainPitchStack[stackNum];
double odomTimeStack[stackNum];
int odomSendIDPointer = -1;
int odomRecIDPointer = 0;

pcl::VoxelGrid<pcl::PointXYZI> terrainDwzFilter;

ros::Publisher* pubScanPointer = NULL;
ros::Publisher* pubRobotSpeed = NULL;
tf::TransformListener *tf_listener_ptr;

//点云数据回调函数
void scanHandler(const sensor_msgs::PointCloud2::ConstPtr& scanIn)
{

  //延迟五次
  if (!systemInited) {
    ROS_INFO("hold on, initializing!!\n");
    systemInitCount++;
    if (systemInitCount > systemDelay) {
      systemInited = true;
      ROS_INFO("initialization done!!\n");
    }
    return;
  }

  double scanTime = scanIn->header.stamp.toSec();

  if (odomSendIDPointer < 0)
  {
    return;
  }
  while (odomTimeStack[(odomRecIDPointer + 1) % stackNum] < scanTime &&
         odomRecIDPointer != (odomSendIDPointer + 1) % stackNum)
  {
    odomRecIDPointer = (odomRecIDPointer + 1) % stackNum;
  }

  double odomRecTime = odomTime.toSec();
  float vehicleRecX = vehicleX;
  float vehicleRecY = vehicleY;
  float vehicleRecZ = vehicleZ;
  float vehicleRecRoll = vehicleRoll;
  float vehicleRecPitch = vehiclePitch;
  float vehicleRecYaw = vehicleYaw;
  float terrainRecRoll = terrainRoll;
  float terrainRecPitch = terrainPitch;

  if (use_gazebo_time)
  {
    odomRecTime = odomTimeStack[odomRecIDPointer];
    vehicleRecX = vehicleXStack[odomRecIDPointer];
    vehicleRecY = vehicleYStack[odomRecIDPointer];
    vehicleRecZ = vehicleZStack[odomRecIDPointer];
    vehicleRecRoll = vehicleRollStack[odomRecIDPointer];
    vehicleRecPitch = vehiclePitchStack[odomRecIDPointer];
    vehicleRecYaw = vehicleYawStack[odomRecIDPointer];
    terrainRecRoll = terrainRollStack[odomRecIDPointer];
    terrainRecPitch = terrainPitchStack[odomRecIDPointer];
  }

  float sinTerrainRecRoll = sin(terrainRecRoll);
  float cosTerrainRecRoll = cos(terrainRecRoll);
  float sinTerrainRecPitch = sin(terrainRecPitch);
  float cosTerrainRecPitch = cos(terrainRecPitch);

  scanData->clear();
  pcl::fromROSMsg(*scanIn, *scanData);
  pcl::removeNaNFromPointCloud(*scanData, *scanData, scanInd);




  // int scanDataSize = scanData->points.size();
  // for (int i = 0; i < scanDataSize; i++)
  // {
  //   float pointX1 = scanData->points[i].x;
  //   float pointY1 = scanData->points[i].y * cosTerrainRecRoll - scanData->points[i].z * sinTerrainRecRoll;
  //   float pointZ1 = scanData->points[i].y * sinTerrainRecRoll + scanData->points[i].z * cosTerrainRecRoll;

  //   float pointX2 = pointX1 * cosTerrainRecPitch + pointZ1 * sinTerrainRecPitch;
  //   float pointY2 = pointY1;
  //   float pointZ2 = -pointX1 * sinTerrainRecPitch + pointZ1 * cosTerrainRecPitch;

  //   float pointX3 = pointX2 + vehicleRecX;
  //   float pointY3 = pointY2 + vehicleRecY;
  //   float pointZ3 = pointZ2 + vehicleRecZ;

  //   scanData->points[i].x = pointX3;
  //   scanData->points[i].y = pointY3;
  //   scanData->points[i].z = pointZ3;
  // }

  // publish 5Hz registered scan messages
  sensor_msgs::PointCloud2 scanData2;
  pcl::toROSMsg(*scanData, scanData2);
  pcl_ros::transformPointCloud("map",*scanIn,scanData2, *tf_listener_ptr);


  scanData2.header.stamp = ros::Time().fromSec(odomRecTime);
  scanData2.header.frame_id = "map";
  pubScanPointer->publish(scanData2);


}

void terrainCloudHandler(const sensor_msgs::PointCloud2ConstPtr& terrainCloud2)
{

  if (!adjustZ && !adjustIncl)
  {
    return;
  }

  terrainCloud->clear();
  pcl::fromROSMsg(*terrainCloud2, *terrainCloud);

  pcl::PointXYZI point;
  terrainCloudIncl->clear();
  int terrainCloudSize = terrainCloud->points.size();
  double elevMean = 0;
  int elevCount = 0;
  bool terrainValid = true;
  for (int i = 0; i < terrainCloudSize; i++)
  {
    point = terrainCloud->points[i];

    float dis = sqrt((point.x - vehicleX) * (point.x - vehicleX) + (point.y - vehicleY) * (point.y - vehicleY));

    if (dis < terrainRadiusZ)
    {
      if (point.intensity < groundHeightThre)
      {
        elevMean += point.z;
        elevCount++;
      }
      else
      {
        terrainValid = false;
      }
    }

    if (dis < terrainRadiusIncl && point.intensity < groundHeightThre)
    {
      terrainCloudIncl->push_back(point);
    }
  }

  if (elevCount >= minTerrainPointNumZ)
    elevMean /= elevCount;
  else
    terrainValid = false;

  if (terrainValid && adjustZ)
  {
    terrainZ = (1.0 - smoothRateZ) * terrainZ + smoothRateZ * elevMean;
  }

  terrainCloudDwz->clear();
  terrainDwzFilter.setInputCloud(terrainCloudIncl);
  terrainDwzFilter.filter(*terrainCloudDwz);
  int terrainCloudDwzSize = terrainCloudDwz->points.size();

  if (terrainCloudDwzSize < minTerrainPointNumIncl || !terrainValid)
  {
    return;
  }

  cv::Mat matA(terrainCloudDwzSize, 2, CV_32F, cv::Scalar::all(0));
  cv::Mat matAt(2, terrainCloudDwzSize, CV_32F, cv::Scalar::all(0));
  cv::Mat matAtA(2, 2, CV_32F, cv::Scalar::all(0));
  cv::Mat matB(terrainCloudDwzSize, 1, CV_32F, cv::Scalar::all(0));
  cv::Mat matAtB(2, 1, CV_32F, cv::Scalar::all(0));
  cv::Mat matX(2, 1, CV_32F, cv::Scalar::all(0));

  int inlierNum = 0;
  matX.at<float>(0, 0) = terrainPitch;
  matX.at<float>(1, 0) = terrainRoll;
  for (int iterCount = 0; iterCount < 5; iterCount++)
  {
    int outlierCount = 0;
    for (int i = 0; i < terrainCloudDwzSize; i++)
    {
      point = terrainCloudDwz->points[i];

      matA.at<float>(i, 0) = -point.x + vehicleX;
      matA.at<float>(i, 1) = point.y - vehicleY;
      matB.at<float>(i, 0) = point.z - elevMean;

      if (fabs(matA.at<float>(i, 0) * matX.at<float>(0, 0) + matA.at<float>(i, 1) * matX.at<float>(1, 0) -
               matB.at<float>(i, 0)) > InclFittingThre &&
          iterCount > 0)
      {
        matA.at<float>(i, 0) = 0;
        matA.at<float>(i, 1) = 0;
        matB.at<float>(i, 0) = 0;
        outlierCount++;
      }
    }

    cv::transpose(matA, matAt);
    matAtA = matAt * matA;
    matAtB = matAt * matB;
    cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);

    if (inlierNum == terrainCloudDwzSize - outlierCount)
      break;
    inlierNum = terrainCloudDwzSize - outlierCount;
  }

  if (inlierNum < minTerrainPointNumIncl || fabs(matX.at<float>(0, 0)) > maxIncl * PI / 180.0 ||
      fabs(matX.at<float>(1, 0)) > maxIncl * PI / 180.0)
  {
    terrainValid = false;
  }

  if (terrainValid && adjustIncl)
  {
    terrainPitch = (1.0 - smoothRateIncl) * terrainPitch + smoothRateIncl * matX.at<float>(0, 0);
    terrainRoll = (1.0 - smoothRateIncl) * terrainRoll + smoothRateIncl * matX.at<float>(1, 0);
  }

}

void speedHandler(const geometry_msgs::TwistStamped::ConstPtr& speedIn)
{
  geometry_msgs::Twist speed_temp = speedIn->twist;

  // speed_temp.linear.x = speedIn->twist.linear.x;
  // speed_temp.linear.y = speedIn->twist.linear.y;
  // speed_temp.linear.z = speedIn->twist.linear.z;
  // speed_temp.angular.x = speedIn->twist.angular.x;
  // speed_temp.angular.y =speedIn->twist.angular.y;
  // speed_temp.angular.z =speedIn->twist.angular.z;

  pubRobotSpeed->publish(speed_temp);
}


//订阅原本的odom话题，然后进行odom的坐标系转换
void odomHandler(const nav_msgs::Odometry::ConstPtr& odomIn)
{

  Odom_quat_get = odomIn->pose.pose.orientation;
  Odom_pose_get = odomIn->pose.pose.position;

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "robotSimulator");
  ros::NodeHandle nh;
  ros::NodeHandle nhPrivate = ros::NodeHandle("~");

  nhPrivate.getParam("use_gazebo_time", use_gazebo_time);
  nhPrivate.getParam("cameraOffsetZ", cameraOffsetZ);
  nhPrivate.getParam("sensorOffsetX", sensorOffsetX);
  nhPrivate.getParam("sensorOffsetY", sensorOffsetY);
  nhPrivate.getParam("vehicleHeight", vehicleHeight);
  nhPrivate.getParam("vehicleX", vehicleX);
  nhPrivate.getParam("vehicleY", vehicleY);
  nhPrivate.getParam("vehicleZ", vehicleZ);
  nhPrivate.getParam("terrainZ", terrainZ);
  nhPrivate.getParam("vehicleYaw", vehicleYaw);
  nhPrivate.getParam("terrainVoxelSize", terrainVoxelSize);
  nhPrivate.getParam("groundHeightThre", groundHeightThre);
  nhPrivate.getParam("adjustZ", adjustZ);
  nhPrivate.getParam("terrainRadiusZ", terrainRadiusZ);
  nhPrivate.getParam("minTerrainPointNumZ", minTerrainPointNumZ);
  nhPrivate.getParam("adjustIncl", adjustIncl);
  nhPrivate.getParam("terrainRadiusIncl", terrainRadiusIncl);
  nhPrivate.getParam("minTerrainPointNumIncl", minTerrainPointNumIncl);
  nhPrivate.getParam("InclFittingThre", InclFittingThre);
  nhPrivate.getParam("maxIncl", maxIncl);

  ros::Subscriber subScan = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 2, scanHandler);

  ros::Subscriber subTerrainCloud = nh.subscribe<sensor_msgs::PointCloud2>("/terrain_map", 2, terrainCloudHandler);

  ros::Subscriber subOdom = nh.subscribe<nav_msgs::Odometry>("/A/car0/odom", 100, odomHandler);

  ros::Subscriber subSpeed = nh.subscribe<geometry_msgs::TwistStamped>("/cmd_vel", 5, speedHandler);

  ros::Publisher pubVehicleOdom = nh.advertise<nav_msgs::Odometry>("/state_estimation", 5);

  ros::Publisher pubScan = nh.advertise<sensor_msgs::PointCloud2>("/registered_scan", 2);

  ros::Publisher pubspeed = nh.advertise<geometry_msgs::Twist>("/A/car0/cmd_vel", 2);

  Odom_quat_get.x = 0;
  Odom_quat_get.y = 0;
  Odom_quat_get.z = 0;
  Odom_quat_get.w = 0;

  tf::TransformBroadcaster tfBroadcaster;
  tf::StampedTransform odomTrans;
  tf::TransformListener listener;

  nav_msgs::Odometry odomData;
  odomData.header.frame_id = "map";
  odomData.child_frame_id = "sensor";


  odomTrans.frame_id_ = "map";
  odomTrans.child_frame_id_ = "sensor";

  //使pubscan这个发布者，成为cpp文件范围内的全局指针，方便在回调函数中，收一个发一个
  pubScanPointer = &pubScan;
  pubRobotSpeed = &pubspeed;
  tf_listener_ptr = &listener;

  terrainDwzFilter.setLeafSize(terrainVoxelSize, terrainVoxelSize, terrainVoxelSize);

  ROS_INFO("\nSimulation started.\n\n");

  ros::Rate rate(200);
  bool status = ros::ok();

  while (status)
  {
    ros::spinOnce();

    tf::StampedTransform transform;
    try{
       listener.lookupTransform("map", "velodyne", ros::Time(0), transform);
        }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
        }
      

    float vehicleRecRoll = vehicleRoll;
    float vehicleRecPitch = vehiclePitch;
    float vehicleRecZ = vehicleZ;

    vehicleRoll = terrainRoll * cos(vehicleYaw) + terrainPitch * sin(vehicleYaw);
    vehiclePitch = -terrainRoll * sin(vehicleYaw) + terrainPitch * cos(vehicleYaw);
    vehicleYaw += 0.005 * vehicleYawRate;
    if (vehicleYaw > PI)
      vehicleYaw -= 2 * PI;
    else if (vehicleYaw < -PI)
      vehicleYaw += 2 * PI;

    vehicleX += 0.005 * cos(vehicleYaw) * vehicleSpeed +
                0.005 * vehicleYawRate * (-sin(vehicleYaw) * sensorOffsetX - cos(vehicleYaw) * sensorOffsetY);
    vehicleY += 0.005 * sin(vehicleYaw) * vehicleSpeed +
                0.005 * vehicleYawRate * (cos(vehicleYaw) * sensorOffsetX - sin(vehicleYaw) * sensorOffsetY);
    //vehicleZ = terrainZ + vehicleHeight;

    ros::Time odomTimeRec = odomTime;
    odomTime = ros::Time::now();
    if (odomTime == odomTimeRec) odomTime += ros::Duration(0.005);

    tf::Quaternion quat;
    tf::quaternionMsgToTF(Odom_quat_get, quat);
    double roll, pitch, yaw;//定义存储roll,pitch and yaw的容器
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw); //进行转换


    odomSendIDPointer = (odomSendIDPointer + 1) % stackNum;
    odomTimeStack[odomSendIDPointer] = odomTime.toSec();
    vehicleXStack[odomSendIDPointer] = Odom_pose_get.x;
    vehicleYStack[odomSendIDPointer] = Odom_pose_get.y;
    vehicleZStack[odomSendIDPointer] = Odom_pose_get.z;
    vehicleRollStack[odomSendIDPointer] = roll;
    vehiclePitchStack[odomSendIDPointer] = pitch;
    vehicleYawStack[odomSendIDPointer] = yaw;
    terrainRollStack[odomSendIDPointer] = roll;
    terrainPitchStack[odomSendIDPointer] = pitch;
   
    //里程计话题信息赋值并发布
    odomData.header.stamp = odomTime;
    odomData.pose.pose.orientation = Odom_quat_get;
    odomData.pose.pose.position = Odom_pose_get;

    Odom_pose_get.z = transform.getOrigin().z();
    bool quat_valid = (Odom_quat_get.x != 0 || 
                      Odom_quat_get.y != 0 || 
                      Odom_quat_get.z != 0 || 
                      Odom_quat_get.w != 0 || 
                      Odom_quat_get.w != 0) ? 1:0;
    if (quat_valid){
    //里程计发布
    pubVehicleOdom.publish(odomData);
    // 发布map到sensor的tf信息，publish 200Hz tf messages
    odomTrans.stamp_ = odomTime;
    odomTrans.setRotation(tf::Quaternion(Odom_quat_get.x, Odom_quat_get.y, Odom_quat_get.z, Odom_quat_get.w));
    odomTrans.setOrigin(tf::Vector3(Odom_pose_get.x, Odom_pose_get.y, Odom_pose_get.z));
    tfBroadcaster.sendTransform(odomTrans);
    }
    



    status = ros::ok();
    rate.sleep();
  }

  return 0;
}
