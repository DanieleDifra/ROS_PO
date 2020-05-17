#include "slros_initialize.h"

ros::NodeHandle * SLROSNodePtr;
const std::string SLROSNodeName = "forward_kinematics";

// For Block forward_kinematics/Subscribe
SimulinkSubscriber<sensor_msgs::JointState, SL_Bus_forward_kinematics_sensor_msgs_JointState> Sub_forward_kinematics_286;

// For Block forward_kinematics/Subscribe1
SimulinkSubscriber<rosgraph_msgs::Clock, SL_Bus_forward_kinematics_rosgraph_msgs_Clock> Sub_forward_kinematics_287;

// For Block forward_kinematics/Publish
SimulinkPublisher<geometry_msgs::PoseStamped, SL_Bus_forward_kinematics_geometry_msgs_PoseStamped> Pub_forward_kinematics_282;

// For Block forward_kinematics/Publish1
SimulinkPublisher<geometry_msgs::TwistStamped, SL_Bus_forward_kinematics_geometry_msgs_TwistStamped> Pub_forward_kinematics_283;

void slros_node_init(int argc, char** argv)
{
  ros::init(argc, argv, SLROSNodeName);
  SLROSNodePtr = new ros::NodeHandle();
}

