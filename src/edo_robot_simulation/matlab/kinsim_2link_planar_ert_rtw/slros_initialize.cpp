#include "slros_initialize.h"

ros::NodeHandle * SLROSNodePtr;
const std::string SLROSNodeName = "kinsim_2link_planar";

// For Block kinsim_2link_planar/Subscribe
SimulinkSubscriber<trajectory_msgs::JointTrajectoryPoint, SL_Bus_kinsim_2link_planar_JointTrajectoryPoint_1csgl9> Sub_kinsim_2link_planar_16;

// For Block kinsim_2link_planar/Publish
SimulinkPublisher<sensor_msgs::JointState, SL_Bus_kinsim_2link_planar_sensor_msgs_JointState> Pub_kinsim_2link_planar_22;

// For Block kinsim_2link_planar/Publish1
SimulinkPublisher<rosgraph_msgs::Clock, SL_Bus_kinsim_2link_planar_rosgraph_msgs_Clock> Pub_kinsim_2link_planar_50;

// For Block kinsim_2link_planar/Publish2
SimulinkPublisher<sensor_msgs::JointState, SL_Bus_kinsim_2link_planar_sensor_msgs_JointState> Pub_kinsim_2link_planar_79;

// For Block kinsim_2link_planar/Subsystem/Get Parameter
SimulinkParameterGetter<real64_T, double> ParamGet_kinsim_2link_planar_61;

// For Block kinsim_2link_planar/Subsystem/Get Parameter1
SimulinkParameterGetter<real64_T, double> ParamGet_kinsim_2link_planar_65;

// For Block kinsim_2link_planar/Subsystem/Get Parameter2
SimulinkParameterGetter<real64_T, double> ParamGet_kinsim_2link_planar_85;

// For Block kinsim_2link_planar/Subsystem/Get Parameter3
SimulinkParameterGetter<real64_T, double> ParamGet_kinsim_2link_planar_86;

// For Block kinsim_2link_planar/Subsystem/Get Parameter4
SimulinkParameterGetter<real64_T, double> ParamGet_kinsim_2link_planar_87;

// For Block kinsim_2link_planar/Subsystem/Get Parameter5
SimulinkParameterGetter<real64_T, double> ParamGet_kinsim_2link_planar_88;

void slros_node_init(int argc, char** argv)
{
  ros::init(argc, argv, SLROSNodeName);
  SLROSNodePtr = new ros::NodeHandle();
}

