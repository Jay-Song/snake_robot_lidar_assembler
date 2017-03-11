/*
 * snake_robot_lidar_assembler.cpp
 *
 *  Created on: 2017. 3. 10.
 *      Author: Crowban
 */

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/JointState.h>
#include <laser_assembler/AssembleScans2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>


bool   first_sub_ = false;
double g_max_duration_sec = 10.0;

ros::Time           g_lidar_move_start_time;

ros::Publisher      g_point_cloud_pub;
ros::Subscriber     g_lidar_turn_dir_chage_sub;

ros::ServiceClient  g_assemble_laser_client;

void assembleLaserScans(ros::Time before_time, ros::Time end_time)
{
  ros::Time now = ros::Time::now();

  laser_assembler::AssembleScans2 service;
  service.request.begin = before_time;
  service.request.end = end_time;

  if (g_assemble_laser_client.call(service))
  {
    ros::Time assemble_time = ros::Time::now();
    sensor_msgs::PointCloud2 assembler_output = service.response.cloud;
    if (assembler_output.data.size() == 0)
    {
      ROS_INFO("No scan data");
      return;
    }

    ROS_INFO("  ---  publish pointcloud data!!  ---  %f", (ros::Time::now() - assemble_time).toSec());

    g_point_cloud_pub.publish(assembler_output);
  }
}

void lidarTurnCallBack(const std_msgs::Bool::ConstPtr& msg)
{
  ros::Time now = ros::Time::now();

  // assemble laser
  if( (now.toSec() - g_lidar_move_start_time.toSec()) > g_max_duration_sec )
  {
    g_lidar_move_start_time = now;
    return;
  }

  assembleLaserScans(g_lidar_move_start_time, now);
  g_lidar_move_start_time = now;
}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "thor_lidar_assembler");
  ros::NodeHandle nh;

  g_lidar_turn_dir_chage_sub = nh.subscribe("/heroehs/snake_robot/lidar_control/direction_change", 1, &lidarTurnCallBack);
  g_point_cloud_pub          = nh.advertise<sensor_msgs::PointCloud2>("/heroehs/snake_robot/point_cloud", 0);
  g_assemble_laser_client    = nh.serviceClient<laser_assembler::AssembleScans2>("/heroehs/snake_robot/assemble_scans");


  g_lidar_move_start_time = ros::Time::now();

  ros::spin();

  return 0;
}
