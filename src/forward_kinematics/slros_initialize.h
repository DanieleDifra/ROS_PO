#ifndef _SLROS_INITIALIZE_H_
#define _SLROS_INITIALIZE_H_

#include "slros_busmsg_conversion.h"
#include "slros_generic.h"

extern ros::NodeHandle * SLROSNodePtr;
extern const std::string SLROSNodeName;

// For Block forward_kinematics/Subscribe
extern SimulinkSubscriber<sensor_msgs::JointState, SL_Bus_forward_kinematics_sensor_msgs_JointState> Sub_forward_kinematics_286;

// For Block forward_kinematics/Subscribe1
extern SimulinkSubscriber<rosgraph_msgs::Clock, SL_Bus_forward_kinematics_rosgraph_msgs_Clock> Sub_forward_kinematics_287;

// For Block forward_kinematics/Publish
extern SimulinkPublisher<geometry_msgs::PoseStamped, SL_Bus_forward_kinematics_geometry_msgs_PoseStamped> Pub_forward_kinematics_282;

// For Block forward_kinematics/Publish1
extern SimulinkPublisher<geometry_msgs::TwistStamped, SL_Bus_forward_kinematics_geometry_msgs_TwistStamped> Pub_forward_kinematics_283;

void slros_node_init(int argc, char** argv);

#endif
