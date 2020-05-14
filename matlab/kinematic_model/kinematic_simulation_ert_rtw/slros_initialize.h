#ifndef _SLROS_INITIALIZE_H_
#define _SLROS_INITIALIZE_H_

#include "slros_busmsg_conversion.h"
#include "slros_generic.h"

extern ros::NodeHandle * SLROSNodePtr;
extern const std::string SLROSNodeName;

// For Block kinematic_simulation/Subscribe
extern SimulinkSubscriber<trajectory_msgs::JointTrajectoryPoint, SL_Bus_kinematic_simulation_JointTrajectoryPoint_qcxomb> Sub_kinematic_simulation_16;

// For Block kinematic_simulation/Publish
extern SimulinkPublisher<sensor_msgs::JointState, SL_Bus_kinematic_simulation_sensor_msgs_JointState> Pub_kinematic_simulation_22;

// For Block kinematic_simulation/Publish1
extern SimulinkPublisher<rosgraph_msgs::Clock, SL_Bus_kinematic_simulation_rosgraph_msgs_Clock> Pub_kinematic_simulation_50;

// For Block kinematic_simulation/Publish2
extern SimulinkPublisher<geometry_msgs::PoseStamped, SL_Bus_kinematic_simulation_geometry_msgs_PoseStamped> Pub_kinematic_simulation_79;

// For Block kinematic_simulation/Publish3
extern SimulinkPublisher<geometry_msgs::TwistStamped, SL_Bus_kinematic_simulation_geometry_msgs_TwistStamped> Pub_kinematic_simulation_101;

// For Block kinematic_simulation/Subsystem/Get Parameter
extern SimulinkParameterGetter<real64_T, double> ParamGet_kinematic_simulation_61;

// For Block kinematic_simulation/Subsystem/Get Parameter1
extern SimulinkParameterGetter<real64_T, double> ParamGet_kinematic_simulation_65;

// For Block kinematic_simulation/Subsystem/Get Parameter2
extern SimulinkParameterGetter<real64_T, double> ParamGet_kinematic_simulation_85;

// For Block kinematic_simulation/Subsystem/Get Parameter3
extern SimulinkParameterGetter<real64_T, double> ParamGet_kinematic_simulation_86;

// For Block kinematic_simulation/Subsystem/Get Parameter4
extern SimulinkParameterGetter<real64_T, double> ParamGet_kinematic_simulation_87;

// For Block kinematic_simulation/Subsystem/Get Parameter5
extern SimulinkParameterGetter<real64_T, double> ParamGet_kinematic_simulation_88;

void slros_node_init(int argc, char** argv);

#endif
