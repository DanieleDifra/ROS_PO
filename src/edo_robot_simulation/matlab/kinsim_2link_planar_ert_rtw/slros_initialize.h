#ifndef _SLROS_INITIALIZE_H_
#define _SLROS_INITIALIZE_H_

#include "slros_busmsg_conversion.h"
#include "slros_generic.h"

extern ros::NodeHandle * SLROSNodePtr;
extern const std::string SLROSNodeName;

// For Block kinsim_2link_planar/Subscribe
extern SimulinkSubscriber<trajectory_msgs::JointTrajectoryPoint, SL_Bus_kinsim_2link_planar_JointTrajectoryPoint_1csgl9> Sub_kinsim_2link_planar_16;

// For Block kinsim_2link_planar/Publish
extern SimulinkPublisher<sensor_msgs::JointState, SL_Bus_kinsim_2link_planar_sensor_msgs_JointState> Pub_kinsim_2link_planar_22;

// For Block kinsim_2link_planar/Publish1
extern SimulinkPublisher<rosgraph_msgs::Clock, SL_Bus_kinsim_2link_planar_rosgraph_msgs_Clock> Pub_kinsim_2link_planar_50;

// For Block kinsim_2link_planar/Publish2
extern SimulinkPublisher<sensor_msgs::JointState, SL_Bus_kinsim_2link_planar_sensor_msgs_JointState> Pub_kinsim_2link_planar_79;

// For Block kinsim_2link_planar/Subsystem/Get Parameter
extern SimulinkParameterGetter<real64_T, double> ParamGet_kinsim_2link_planar_61;

// For Block kinsim_2link_planar/Subsystem/Get Parameter1
extern SimulinkParameterGetter<real64_T, double> ParamGet_kinsim_2link_planar_65;

// For Block kinsim_2link_planar/Subsystem/Get Parameter2
extern SimulinkParameterGetter<real64_T, double> ParamGet_kinsim_2link_planar_85;

// For Block kinsim_2link_planar/Subsystem/Get Parameter3
extern SimulinkParameterGetter<real64_T, double> ParamGet_kinsim_2link_planar_86;

// For Block kinsim_2link_planar/Subsystem/Get Parameter4
extern SimulinkParameterGetter<real64_T, double> ParamGet_kinsim_2link_planar_87;

// For Block kinsim_2link_planar/Subsystem/Get Parameter5
extern SimulinkParameterGetter<real64_T, double> ParamGet_kinsim_2link_planar_88;

void slros_node_init(int argc, char** argv);

#endif
