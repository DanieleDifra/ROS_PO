#ifndef _SLROS_BUSMSG_CONVERSION_H_
#define _SLROS_BUSMSG_CONVERSION_H_

#include <ros/ros.h>
#include <ros/duration.h>
#include <ros/time.h>
#include <rosgraph_msgs/Clock.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Header.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include "kinsim_2link_planar_types.h"
#include "slros_msgconvert_utils.h"


void convertFromBus(ros::Duration* msgPtr, SL_Bus_kinsim_2link_planar_ros_time_Duration const* busPtr);
void convertToBus(SL_Bus_kinsim_2link_planar_ros_time_Duration* busPtr, ros::Duration const* msgPtr);

void convertFromBus(ros::Time* msgPtr, SL_Bus_kinsim_2link_planar_ros_time_Time const* busPtr);
void convertToBus(SL_Bus_kinsim_2link_planar_ros_time_Time* busPtr, ros::Time const* msgPtr);

void convertFromBus(rosgraph_msgs::Clock* msgPtr, SL_Bus_kinsim_2link_planar_rosgraph_msgs_Clock const* busPtr);
void convertToBus(SL_Bus_kinsim_2link_planar_rosgraph_msgs_Clock* busPtr, rosgraph_msgs::Clock const* msgPtr);

void convertFromBus(sensor_msgs::JointState* msgPtr, SL_Bus_kinsim_2link_planar_sensor_msgs_JointState const* busPtr);
void convertToBus(SL_Bus_kinsim_2link_planar_sensor_msgs_JointState* busPtr, sensor_msgs::JointState const* msgPtr);

void convertFromBus(std_msgs::Header* msgPtr, SL_Bus_kinsim_2link_planar_std_msgs_Header const* busPtr);
void convertToBus(SL_Bus_kinsim_2link_planar_std_msgs_Header* busPtr, std_msgs::Header const* msgPtr);

void convertFromBus(trajectory_msgs::JointTrajectoryPoint* msgPtr, SL_Bus_kinsim_2link_planar_JointTrajectoryPoint_1csgl9 const* busPtr);
void convertToBus(SL_Bus_kinsim_2link_planar_JointTrajectoryPoint_1csgl9* busPtr, trajectory_msgs::JointTrajectoryPoint const* msgPtr);


#endif
