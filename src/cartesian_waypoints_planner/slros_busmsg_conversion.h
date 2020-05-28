#ifndef _SLROS_BUSMSG_CONVERSION_H_
#define _SLROS_BUSMSG_CONVERSION_H_

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <ros/duration.h>
#include <ros/time.h>
#include <rosgraph_msgs/Clock.h>
#include <std_msgs/Header.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include "cartesian_waypoints_planner_types.h"
#include "slros_msgconvert_utils.h"


void convertFromBus(geometry_msgs::Point* msgPtr, SL_Bus_cartesian_waypoints_planner_geometry_msgs_Point const* busPtr);
void convertToBus(SL_Bus_cartesian_waypoints_planner_geometry_msgs_Point* busPtr, geometry_msgs::Point const* msgPtr);

void convertFromBus(geometry_msgs::Pose* msgPtr, SL_Bus_cartesian_waypoints_planner_geometry_msgs_Pose const* busPtr);
void convertToBus(SL_Bus_cartesian_waypoints_planner_geometry_msgs_Pose* busPtr, geometry_msgs::Pose const* msgPtr);

void convertFromBus(geometry_msgs::PoseStamped* msgPtr, SL_Bus_cartesian_waypoints_plann_PoseStamped_hk4udn const* busPtr);
void convertToBus(SL_Bus_cartesian_waypoints_plann_PoseStamped_hk4udn* busPtr, geometry_msgs::PoseStamped const* msgPtr);

void convertFromBus(geometry_msgs::Quaternion* msgPtr, SL_Bus_cartesian_waypoints_planner_geometry_msgs_Quaternion const* busPtr);
void convertToBus(SL_Bus_cartesian_waypoints_planner_geometry_msgs_Quaternion* busPtr, geometry_msgs::Quaternion const* msgPtr);

void convertFromBus(ros::Duration* msgPtr, SL_Bus_cartesian_waypoints_planner_ros_time_Duration const* busPtr);
void convertToBus(SL_Bus_cartesian_waypoints_planner_ros_time_Duration* busPtr, ros::Duration const* msgPtr);

void convertFromBus(ros::Time* msgPtr, SL_Bus_cartesian_waypoints_planner_ros_time_Time const* busPtr);
void convertToBus(SL_Bus_cartesian_waypoints_planner_ros_time_Time* busPtr, ros::Time const* msgPtr);

void convertFromBus(rosgraph_msgs::Clock* msgPtr, SL_Bus_cartesian_waypoints_planner_rosgraph_msgs_Clock const* busPtr);
void convertToBus(SL_Bus_cartesian_waypoints_planner_rosgraph_msgs_Clock* busPtr, rosgraph_msgs::Clock const* msgPtr);

void convertFromBus(std_msgs::Header* msgPtr, SL_Bus_cartesian_waypoints_planner_std_msgs_Header const* busPtr);
void convertToBus(SL_Bus_cartesian_waypoints_planner_std_msgs_Header* busPtr, std_msgs::Header const* msgPtr);

void convertFromBus(trajectory_msgs::JointTrajectoryPoint* msgPtr, SL_Bus_cartesian_waypoints_plann_JointTrajectoryPoint_rocwzq const* busPtr);
void convertToBus(SL_Bus_cartesian_waypoints_plann_JointTrajectoryPoint_rocwzq* busPtr, trajectory_msgs::JointTrajectoryPoint const* msgPtr);


#endif
