#ifndef _SLROS_BUSMSG_CONVERSION_H_
#define _SLROS_BUSMSG_CONVERSION_H_

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3.h>
#include <ros/time.h>
#include <rosgraph_msgs/Clock.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Header.h>
#include "forward_kinematics_types.h"
#include "slros_msgconvert_utils.h"


void convertFromBus(geometry_msgs::Point* msgPtr, SL_Bus_forward_kinematics_geometry_msgs_Point const* busPtr);
void convertToBus(SL_Bus_forward_kinematics_geometry_msgs_Point* busPtr, geometry_msgs::Point const* msgPtr);

void convertFromBus(geometry_msgs::Pose* msgPtr, SL_Bus_forward_kinematics_geometry_msgs_Pose const* busPtr);
void convertToBus(SL_Bus_forward_kinematics_geometry_msgs_Pose* busPtr, geometry_msgs::Pose const* msgPtr);

void convertFromBus(geometry_msgs::PoseStamped* msgPtr, SL_Bus_forward_kinematics_geometry_msgs_PoseStamped const* busPtr);
void convertToBus(SL_Bus_forward_kinematics_geometry_msgs_PoseStamped* busPtr, geometry_msgs::PoseStamped const* msgPtr);

void convertFromBus(geometry_msgs::Quaternion* msgPtr, SL_Bus_forward_kinematics_geometry_msgs_Quaternion const* busPtr);
void convertToBus(SL_Bus_forward_kinematics_geometry_msgs_Quaternion* busPtr, geometry_msgs::Quaternion const* msgPtr);

void convertFromBus(geometry_msgs::Twist* msgPtr, SL_Bus_forward_kinematics_geometry_msgs_Twist const* busPtr);
void convertToBus(SL_Bus_forward_kinematics_geometry_msgs_Twist* busPtr, geometry_msgs::Twist const* msgPtr);

void convertFromBus(geometry_msgs::TwistStamped* msgPtr, SL_Bus_forward_kinematics_geometry_msgs_TwistStamped const* busPtr);
void convertToBus(SL_Bus_forward_kinematics_geometry_msgs_TwistStamped* busPtr, geometry_msgs::TwistStamped const* msgPtr);

void convertFromBus(geometry_msgs::Vector3* msgPtr, SL_Bus_forward_kinematics_geometry_msgs_Vector3 const* busPtr);
void convertToBus(SL_Bus_forward_kinematics_geometry_msgs_Vector3* busPtr, geometry_msgs::Vector3 const* msgPtr);

void convertFromBus(ros::Time* msgPtr, SL_Bus_forward_kinematics_ros_time_Time const* busPtr);
void convertToBus(SL_Bus_forward_kinematics_ros_time_Time* busPtr, ros::Time const* msgPtr);

void convertFromBus(rosgraph_msgs::Clock* msgPtr, SL_Bus_forward_kinematics_rosgraph_msgs_Clock const* busPtr);
void convertToBus(SL_Bus_forward_kinematics_rosgraph_msgs_Clock* busPtr, rosgraph_msgs::Clock const* msgPtr);

void convertFromBus(sensor_msgs::JointState* msgPtr, SL_Bus_forward_kinematics_sensor_msgs_JointState const* busPtr);
void convertToBus(SL_Bus_forward_kinematics_sensor_msgs_JointState* busPtr, sensor_msgs::JointState const* msgPtr);

void convertFromBus(std_msgs::Header* msgPtr, SL_Bus_forward_kinematics_std_msgs_Header const* busPtr);
void convertToBus(SL_Bus_forward_kinematics_std_msgs_Header* busPtr, std_msgs::Header const* msgPtr);


#endif
