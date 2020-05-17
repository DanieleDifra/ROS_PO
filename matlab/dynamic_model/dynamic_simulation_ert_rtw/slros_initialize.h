#ifndef _SLROS_INITIALIZE_H_
#define _SLROS_INITIALIZE_H_

#include "slros_busmsg_conversion.h"
#include "slros_generic.h"

extern ros::NodeHandle * SLROSNodePtr;
extern const std::string SLROSNodeName;

// For Block dynamic_simulation/Subscribe
extern SimulinkSubscriber<std_msgs::Float64MultiArray, SL_Bus_dynamic_simulation_std_msgs_Float64MultiArray> Sub_dynamic_simulation_160;

// For Block dynamic_simulation/Publish
extern SimulinkPublisher<sensor_msgs::JointState, SL_Bus_dynamic_simulation_sensor_msgs_JointState> Pub_dynamic_simulation_157;

// For Block dynamic_simulation/Publish1
extern SimulinkPublisher<rosgraph_msgs::Clock, SL_Bus_dynamic_simulation_rosgraph_msgs_Clock> Pub_dynamic_simulation_158;

// For Block dynamic_simulation/Joint_State_Msg_Creator/Get Joint Names/Get Parameter
extern SimulinkParameterArrayGetter<char_T, std::string> ParamGet_dynamic_simulation_224;

// For Block dynamic_simulation/Joint_State_Msg_Creator/Get Joint Names/Get Parameter1
extern SimulinkParameterArrayGetter<char_T, std::string> ParamGet_dynamic_simulation_228;

// For Block dynamic_simulation/Joint_State_Msg_Creator/Get Joint Names/Get Parameter2
extern SimulinkParameterArrayGetter<char_T, std::string> ParamGet_dynamic_simulation_229;

// For Block dynamic_simulation/Joint_State_Msg_Creator/Get Joint Names/Get Parameter3
extern SimulinkParameterArrayGetter<char_T, std::string> ParamGet_dynamic_simulation_230;

// For Block dynamic_simulation/Joint_State_Msg_Creator/Get Joint Names/Get Parameter4
extern SimulinkParameterArrayGetter<char_T, std::string> ParamGet_dynamic_simulation_231;

// For Block dynamic_simulation/Joint_State_Msg_Creator/Get Joint Names/Get Parameter5
extern SimulinkParameterArrayGetter<char_T, std::string> ParamGet_dynamic_simulation_232;

// For Block dynamic_simulation/Robot Dynamic Model/Subsystem/Get Parameter1
extern SimulinkParameterGetter<real64_T, double> ParamGet_dynamic_simulation_132;

// For Block dynamic_simulation/Robot Dynamic Model/Subsystem/Get Parameter10
extern SimulinkParameterGetter<real64_T, double> ParamGet_dynamic_simulation_133;

// For Block dynamic_simulation/Robot Dynamic Model/Subsystem/Get Parameter11
extern SimulinkParameterGetter<real64_T, double> ParamGet_dynamic_simulation_134;

// For Block dynamic_simulation/Robot Dynamic Model/Subsystem/Get Parameter12
extern SimulinkParameterGetter<real64_T, double> ParamGet_dynamic_simulation_194;

// For Block dynamic_simulation/Robot Dynamic Model/Subsystem/Get Parameter2
extern SimulinkParameterGetter<real64_T, double> ParamGet_dynamic_simulation_135;

// For Block dynamic_simulation/Robot Dynamic Model/Subsystem/Get Parameter3
extern SimulinkParameterGetter<real64_T, double> ParamGet_dynamic_simulation_136;

// For Block dynamic_simulation/Robot Dynamic Model/Subsystem/Get Parameter4
extern SimulinkParameterGetter<real64_T, double> ParamGet_dynamic_simulation_137;

// For Block dynamic_simulation/Robot Dynamic Model/Subsystem/Get Parameter5
extern SimulinkParameterGetter<real64_T, double> ParamGet_dynamic_simulation_138;

// For Block dynamic_simulation/Robot Dynamic Model/Subsystem/Get Parameter6
extern SimulinkParameterGetter<real64_T, double> ParamGet_dynamic_simulation_139;

// For Block dynamic_simulation/Robot Dynamic Model/Subsystem/Get Parameter7
extern SimulinkParameterGetter<real64_T, double> ParamGet_dynamic_simulation_140;

// For Block dynamic_simulation/Robot Dynamic Model/Subsystem/Get Parameter8
extern SimulinkParameterGetter<real64_T, double> ParamGet_dynamic_simulation_141;

// For Block dynamic_simulation/Robot Dynamic Model/Subsystem/Get Parameter9
extern SimulinkParameterGetter<real64_T, double> ParamGet_dynamic_simulation_142;

void slros_node_init(int argc, char** argv);

#endif
