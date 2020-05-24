#ifndef _SLROS_BUSMSG_CONVERSION_H_
#define _SLROS_BUSMSG_CONVERSION_H_

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/MultiArrayLayout.h>
#include "cartesian_waypoints_planner_types.h"
#include "slros_msgconvert_utils.h"


void convertFromBus(std_msgs::Float32MultiArray* msgPtr, SL_Bus_cartesian_waypoints_plann_Float32MultiArray_iutvhs const* busPtr);
void convertToBus(SL_Bus_cartesian_waypoints_plann_Float32MultiArray_iutvhs* busPtr, std_msgs::Float32MultiArray const* msgPtr);

void convertFromBus(std_msgs::MultiArrayDimension* msgPtr, SL_Bus_cartesian_waypoints_plann_MultiArrayDimension_u6izuj const* busPtr);
void convertToBus(SL_Bus_cartesian_waypoints_plann_MultiArrayDimension_u6izuj* busPtr, std_msgs::MultiArrayDimension const* msgPtr);

void convertFromBus(std_msgs::MultiArrayLayout* msgPtr, SL_Bus_cartesian_waypoints_plann_MultiArrayLayout_ldqki3 const* busPtr);
void convertToBus(SL_Bus_cartesian_waypoints_plann_MultiArrayLayout_ldqki3* busPtr, std_msgs::MultiArrayLayout const* msgPtr);


#endif
