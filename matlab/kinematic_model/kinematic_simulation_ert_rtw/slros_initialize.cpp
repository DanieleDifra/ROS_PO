#include "slros_initialize.h"

ros::NodeHandle * SLROSNodePtr;
const std::string SLROSNodeName = "kinematic_simulation";

// For Block kinematic_simulation/Subscribe
SimulinkSubscriber<trajectory_msgs::JointTrajectoryPoint, SL_Bus_kinematic_simulation_JointTrajectoryPoint_qcxomb> Sub_kinematic_simulation_16;

// For Block kinematic_simulation/Publish
SimulinkPublisher<sensor_msgs::JointState, SL_Bus_kinematic_simulation_sensor_msgs_JointState> Pub_kinematic_simulation_22;

// For Block kinematic_simulation/Publish1
SimulinkPublisher<rosgraph_msgs::Clock, SL_Bus_kinematic_simulation_rosgraph_msgs_Clock> Pub_kinematic_simulation_201;

// For Block kinematic_simulation/Joint_State_Msg_Creator/Get Joint Names/Get Parameter
SimulinkParameterArrayGetter<char_T, std::string> ParamGet_kinematic_simulation_205;

// For Block kinematic_simulation/Joint_State_Msg_Creator/Get Joint Names/Get Parameter1
SimulinkParameterArrayGetter<char_T, std::string> ParamGet_kinematic_simulation_184;

// For Block kinematic_simulation/Joint_State_Msg_Creator/Get Joint Names/Get Parameter2
SimulinkParameterArrayGetter<char_T, std::string> ParamGet_kinematic_simulation_185;

// For Block kinematic_simulation/Joint_State_Msg_Creator/Get Joint Names/Get Parameter3
SimulinkParameterArrayGetter<char_T, std::string> ParamGet_kinematic_simulation_186;

// For Block kinematic_simulation/Joint_State_Msg_Creator/Get Joint Names/Get Parameter4
SimulinkParameterArrayGetter<char_T, std::string> ParamGet_kinematic_simulation_187;

// For Block kinematic_simulation/Joint_State_Msg_Creator/Get Joint Names/Get Parameter5
SimulinkParameterArrayGetter<char_T, std::string> ParamGet_kinematic_simulation_188;

// For Block kinematic_simulation/Robot Kinematic Model/Subsystem/Get Parameter
SimulinkParameterGetter<real64_T, double> ParamGet_kinematic_simulation_158;

// For Block kinematic_simulation/Robot Kinematic Model/Subsystem/Get Parameter1
SimulinkParameterGetter<real64_T, double> ParamGet_kinematic_simulation_159;

// For Block kinematic_simulation/Robot Kinematic Model/Subsystem/Get Parameter2
SimulinkParameterGetter<real64_T, double> ParamGet_kinematic_simulation_160;

// For Block kinematic_simulation/Robot Kinematic Model/Subsystem/Get Parameter3
SimulinkParameterGetter<real64_T, double> ParamGet_kinematic_simulation_161;

// For Block kinematic_simulation/Robot Kinematic Model/Subsystem/Get Parameter4
SimulinkParameterGetter<real64_T, double> ParamGet_kinematic_simulation_162;

// For Block kinematic_simulation/Robot Kinematic Model/Subsystem/Get Parameter5
SimulinkParameterGetter<real64_T, double> ParamGet_kinematic_simulation_163;

void slros_node_init(int argc, char** argv)
{
  ros::init(argc, argv, SLROSNodeName);
  SLROSNodePtr = new ros::NodeHandle();
}

