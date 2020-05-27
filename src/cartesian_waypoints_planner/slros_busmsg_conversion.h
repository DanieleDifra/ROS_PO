#ifndef _SLROS_BUSMSG_CONVERSION_H_
#define _SLROS_BUSMSG_CONVERSION_H_

#include <ros/ros.h>
#include <ros/duration.h>
#include <ros/time.h>
#include <rosgraph_msgs/Clock.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include "cartesian_waypoints_planner_types.h"
#include "slros_msgconvert_utils.h"


void convertFromBus(ros::Duration* msgPtr, SL_Bus_cartesian_waypoints_planner_ros_time_Duration const* busPtr);
void convertToBus(SL_Bus_cartesian_waypoints_planner_ros_time_Duration* busPtr, ros::Duration const* msgPtr);

void convertFromBus(ros::Time* msgPtr, SL_Bus_cartesian_waypoints_planner_ros_time_Time const* busPtr);
void convertToBus(SL_Bus_cartesian_waypoints_planner_ros_time_Time* busPtr, ros::Time const* msgPtr);

void convertFromBus(rosgraph_msgs::Clock* msgPtr, SL_Bus_cartesian_waypoints_planner_rosgraph_msgs_Clock const* busPtr);
void convertToBus(SL_Bus_cartesian_waypoints_planner_rosgraph_msgs_Clock* busPtr, rosgraph_msgs::Clock const* msgPtr);

void convertFromBus(trajectory_msgs::JointTrajectoryPoint* msgPtr, SL_Bus_cartesian_waypoints_plann_JointTrajectoryPoint_rocwzq const* busPtr);
void convertToBus(SL_Bus_cartesian_waypoints_plann_JointTrajectoryPoint_rocwzq* busPtr, trajectory_msgs::JointTrajectoryPoint const* msgPtr);


#endif
