#include <string>
#include <vector>
#include <stdio.h>
#include <iostream>

#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PointStamped.h>

std::string odom_topic = "state_estimation";
std::string path_topic = "move_base1/NavfnROS/plan";
float distance_thre = 1.2;
float distance_tole = 0.6;

float vehicleX = 0, vehicleY = 0, vehicleZ = 0;
double curTime = 0, waypointTime = 0;
double way_point_temp_x = 0;
double way_point_temp_y = 0;

bool Flag_get_new_path;
bool Flag_finish_path;
bool Flag_switch_goal;

std::vector<geometry_msgs::PoseStamped> way_point_array;


void poseHandler(const nav_msgs::Odometry::ConstPtr& pose)
{
  curTime = pose->header.stamp.toSec();

  vehicleX = pose->pose.pose.position.x;
  vehicleY = pose->pose.pose.position.y;
  vehicleZ = pose->pose.pose.position.z;
}


void pathHandler(const nav_msgs::Path::ConstPtr& path)
{
  /*首先获取路径并存放在容器中遍历
   *每次都挑选距离1m远的输出
   *如果遍历完都没有一个1m远的了，直接输出最后一个
  */
 if(!path->poses.empty()){
 Flag_get_new_path = true;
 way_point_array.clear();
  std::vector<geometry_msgs::PoseStamped> path_array = path->poses;
  // 获取第一个对象
  geometry_msgs::PoseStamped first_point = *path_array.begin();
  // 获取最后一个对象
  geometry_msgs::PoseStamped last_point = *(path_array.end() - 1);
  float distance_2d = 0;
  double temp_x = vehicleX;
  double temp_y = vehicleY;

	for(const auto& pose_stamped : path_array)
	{

        distance_2d = sqrt(pow(pose_stamped.pose.position.x - temp_x, 2) + 
                            pow(pose_stamped.pose.position.y - temp_y, 2));

        if (distance_2d > distance_thre)
            {
                way_point_array.push_back(pose_stamped);
                temp_x = pose_stamped.pose.position.x;
                temp_y = pose_stamped.pose.position.y;
                ROS_INFO("choosen point: (%f, %f)", temp_x,temp_y);
            }
	}
    way_point_array.push_back(last_point);
    }
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "path2waypoint");
  ros::NodeHandle nh;
  ros::NodeHandle nhPrivate = ros::NodeHandle("~");
  nhPrivate.getParam("odom_topic", odom_topic);
  nhPrivate.getParam("path_topic", path_topic);
  nhPrivate.getParam("distance_thre", distance_thre);
  
  ros::Subscriber subPose = nh.subscribe<nav_msgs::Odometry> (odom_topic, 5, poseHandler);
  ros::Subscriber subPath = nh.subscribe<nav_msgs::Path> (path_topic, 5, pathHandler);

  ros::Publisher pubWaypoint = nh.advertise<geometry_msgs::PointStamped> ("/way_point", 5);
  geometry_msgs::PointStamped waypointMsgs;
  waypointMsgs.header.frame_id = "map";

  geometry_msgs::PoseStamped goal_point;
  int path_index;
  double goal_point_distance;

//   ros::Duration(1.0).sleep();
//   way_point_temp_x = vehicleX;
//   way_point_temp_x = vehicleY;

  ros::Rate rate(100);
  bool status = ros::ok();
  while (status) {
    ros::spinOnce();

    if(!way_point_array.empty())
    {
        if (Flag_get_new_path)
        {
            goal_point = way_point_array.front();
            path_index = 0;
            Flag_switch_goal = true;
            Flag_get_new_path = false;
        }
        else
        {
            
            goal_point_distance = sqrt(pow(goal_point.pose.position.x - vehicleX, 2) 
                                    + pow(goal_point.pose.position.y - vehicleY, 2));
            
            if((goal_point_distance < distance_tole)&&(path_index < way_point_array.size()-1))
            {
                path_index ++;
                goal_point = way_point_array.at(path_index);
                Flag_switch_goal = true;
            }
            else if(path_index == way_point_array.size()-1)
            {
                Flag_finish_path = true;
            }
            
        }
    }

    if(Flag_switch_goal)
    {
        waypointMsgs.header.stamp = ros::Time().fromSec(curTime);
        waypointMsgs.point.x = goal_point.pose.position.x;
        waypointMsgs.point.y = goal_point.pose.position.y;
        waypointMsgs.point.z = 0;
        pubWaypoint.publish(waypointMsgs);
    }



    status = ros::ok();
    rate.sleep();
  }

  return 0;
}
