#ifndef _SLROS_BUSMSG_CONVERSION_H_
#define _SLROS_BUSMSG_CONVERSION_H_

#include <ros/ros.h>
#include <ros/duration.h>
#include <ros/time.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Header.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/MultiArrayLayout.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include "controller_types.h"
#include "slros_msgconvert_utils.h"


void convertFromBus(ros::Duration* msgPtr, SL_Bus_controller_ros_time_Duration const* busPtr);
void convertToBus(SL_Bus_controller_ros_time_Duration* busPtr, ros::Duration const* msgPtr);

void convertFromBus(ros::Time* msgPtr, SL_Bus_controller_ros_time_Time const* busPtr);
void convertToBus(SL_Bus_controller_ros_time_Time* busPtr, ros::Time const* msgPtr);

void convertFromBus(sensor_msgs::JointState* msgPtr, SL_Bus_controller_sensor_msgs_JointState const* busPtr);
void convertToBus(SL_Bus_controller_sensor_msgs_JointState* busPtr, sensor_msgs::JointState const* msgPtr);

void convertFromBus(std_msgs::Float64MultiArray* msgPtr, SL_Bus_controller_std_msgs_Float64MultiArray const* busPtr);
void convertToBus(SL_Bus_controller_std_msgs_Float64MultiArray* busPtr, std_msgs::Float64MultiArray const* msgPtr);

void convertFromBus(std_msgs::Header* msgPtr, SL_Bus_controller_std_msgs_Header const* busPtr);
void convertToBus(SL_Bus_controller_std_msgs_Header* busPtr, std_msgs::Header const* msgPtr);

void convertFromBus(std_msgs::MultiArrayDimension* msgPtr, SL_Bus_controller_std_msgs_MultiArrayDimension const* busPtr);
void convertToBus(SL_Bus_controller_std_msgs_MultiArrayDimension* busPtr, std_msgs::MultiArrayDimension const* msgPtr);

void convertFromBus(std_msgs::MultiArrayLayout* msgPtr, SL_Bus_controller_std_msgs_MultiArrayLayout const* busPtr);
void convertToBus(SL_Bus_controller_std_msgs_MultiArrayLayout* busPtr, std_msgs::MultiArrayLayout const* msgPtr);

void convertFromBus(trajectory_msgs::JointTrajectoryPoint* msgPtr, SL_Bus_controller_trajectory_msgs_JointTrajectoryPoint const* busPtr);
void convertToBus(SL_Bus_controller_trajectory_msgs_JointTrajectoryPoint* busPtr, trajectory_msgs::JointTrajectoryPoint const* msgPtr);


#endif
