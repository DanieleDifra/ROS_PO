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
extern SimulinkPublisher<rosgraph_msgs::Clock, SL_Bus_kinematic_simulation_rosgraph_msgs_Clock> Pub_kinematic_simulation_201;

// For Block kinematic_simulation/Joint_State_Msg_Creator/Get Joint Names/Get Parameter
extern SimulinkParameterArrayGetter<char_T, std::string> ParamGet_kinematic_simulation_205;

// For Block kinematic_simulation/Joint_State_Msg_Creator/Get Joint Names/Get Parameter1
extern SimulinkParameterArrayGetter<char_T, std::string> ParamGet_kinematic_simulation_184;

// For Block kinematic_simulation/Joint_State_Msg_Creator/Get Joint Names/Get Parameter2
extern SimulinkParameterArrayGetter<char_T, std::string> ParamGet_kinematic_simulation_185;

// For Block kinematic_simulation/Joint_State_Msg_Creator/Get Joint Names/Get Parameter3
extern SimulinkParameterArrayGetter<char_T, std::string> ParamGet_kinematic_simulation_186;

// For Block kinematic_simulation/Joint_State_Msg_Creator/Get Joint Names/Get Parameter4
extern SimulinkParameterArrayGetter<char_T, std::string> ParamGet_kinematic_simulation_187;

// For Block kinematic_simulation/Joint_State_Msg_Creator/Get Joint Names/Get Parameter5
extern SimulinkParameterArrayGetter<char_T, std::string> ParamGet_kinematic_simulation_188;

// For Block kinematic_simulation/Robot Kinematic Model/Subsystem/Get Parameter
extern SimulinkParameterGetter<real64_T, double> ParamGet_kinematic_simulation_158;

// For Block kinematic_simulation/Robot Kinematic Model/Subsystem/Get Parameter1
extern SimulinkParameterGetter<real64_T, double> ParamGet_kinematic_simulation_159;

// For Block kinematic_simulation/Robot Kinematic Model/Subsystem/Get Parameter2
extern SimulinkParameterGetter<real64_T, double> ParamGet_kinematic_simulation_160;

// For Block kinematic_simulation/Robot Kinematic Model/Subsystem/Get Parameter3
extern SimulinkParameterGetter<real64_T, double> ParamGet_kinematic_simulation_161;

// For Block kinematic_simulation/Robot Kinematic Model/Subsystem/Get Parameter4
extern SimulinkParameterGetter<real64_T, double> ParamGet_kinematic_simulation_162;

// For Block kinematic_simulation/Robot Kinematic Model/Subsystem/Get Parameter5
extern SimulinkParameterGetter<real64_T, double> ParamGet_kinematic_simulation_163;

void slros_node_init(int argc, char** argv);

#endif
