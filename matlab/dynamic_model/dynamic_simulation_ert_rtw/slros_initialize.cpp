#include "slros_initialize.h"

ros::NodeHandle * SLROSNodePtr;
const std::string SLROSNodeName = "dynamic_simulation";

// For Block dynamic_simulation/Subscribe
SimulinkSubscriber<std_msgs::Float64MultiArray, SL_Bus_dynamic_simulation_std_msgs_Float64MultiArray> Sub_dynamic_simulation_160;

// For Block dynamic_simulation/Publish
SimulinkPublisher<sensor_msgs::JointState, SL_Bus_dynamic_simulation_sensor_msgs_JointState> Pub_dynamic_simulation_157;

// For Block dynamic_simulation/Publish1
SimulinkPublisher<rosgraph_msgs::Clock, SL_Bus_dynamic_simulation_rosgraph_msgs_Clock> Pub_dynamic_simulation_158;

// For Block dynamic_simulation/Publish2
SimulinkPublisher<geometry_msgs::PoseStamped, SL_Bus_dynamic_simulation_geometry_msgs_PoseStamped> Pub_dynamic_simulation_178;

// For Block dynamic_simulation/Publish3
SimulinkPublisher<geometry_msgs::TwistStamped, SL_Bus_dynamic_simulation_geometry_msgs_TwistStamped> Pub_dynamic_simulation_179;

// For Block dynamic_simulation/Joint_State_Msg_Creator/Get Joint Names/Get Parameter
SimulinkParameterArrayGetter<char_T, std::string> ParamGet_dynamic_simulation_224;

// For Block dynamic_simulation/Joint_State_Msg_Creator/Get Joint Names/Get Parameter1
SimulinkParameterArrayGetter<char_T, std::string> ParamGet_dynamic_simulation_228;

// For Block dynamic_simulation/Joint_State_Msg_Creator/Get Joint Names/Get Parameter2
SimulinkParameterArrayGetter<char_T, std::string> ParamGet_dynamic_simulation_229;

// For Block dynamic_simulation/Joint_State_Msg_Creator/Get Joint Names/Get Parameter3
SimulinkParameterArrayGetter<char_T, std::string> ParamGet_dynamic_simulation_230;

// For Block dynamic_simulation/Joint_State_Msg_Creator/Get Joint Names/Get Parameter4
SimulinkParameterArrayGetter<char_T, std::string> ParamGet_dynamic_simulation_231;

// For Block dynamic_simulation/Joint_State_Msg_Creator/Get Joint Names/Get Parameter5
SimulinkParameterArrayGetter<char_T, std::string> ParamGet_dynamic_simulation_232;

// For Block dynamic_simulation/Robot Dynamic Model/Subsystem/Get Parameter1
SimulinkParameterGetter<real64_T, double> ParamGet_dynamic_simulation_132;

// For Block dynamic_simulation/Robot Dynamic Model/Subsystem/Get Parameter10
SimulinkParameterGetter<real64_T, double> ParamGet_dynamic_simulation_133;

// For Block dynamic_simulation/Robot Dynamic Model/Subsystem/Get Parameter11
SimulinkParameterGetter<real64_T, double> ParamGet_dynamic_simulation_134;

// For Block dynamic_simulation/Robot Dynamic Model/Subsystem/Get Parameter12
SimulinkParameterGetter<real64_T, double> ParamGet_dynamic_simulation_194;

// For Block dynamic_simulation/Robot Dynamic Model/Subsystem/Get Parameter2
SimulinkParameterGetter<real64_T, double> ParamGet_dynamic_simulation_135;

// For Block dynamic_simulation/Robot Dynamic Model/Subsystem/Get Parameter3
SimulinkParameterGetter<real64_T, double> ParamGet_dynamic_simulation_136;

// For Block dynamic_simulation/Robot Dynamic Model/Subsystem/Get Parameter4
SimulinkParameterGetter<real64_T, double> ParamGet_dynamic_simulation_137;

// For Block dynamic_simulation/Robot Dynamic Model/Subsystem/Get Parameter5
SimulinkParameterGetter<real64_T, double> ParamGet_dynamic_simulation_138;

// For Block dynamic_simulation/Robot Dynamic Model/Subsystem/Get Parameter6
SimulinkParameterGetter<real64_T, double> ParamGet_dynamic_simulation_139;

// For Block dynamic_simulation/Robot Dynamic Model/Subsystem/Get Parameter7
SimulinkParameterGetter<real64_T, double> ParamGet_dynamic_simulation_140;

// For Block dynamic_simulation/Robot Dynamic Model/Subsystem/Get Parameter8
SimulinkParameterGetter<real64_T, double> ParamGet_dynamic_simulation_141;

// For Block dynamic_simulation/Robot Dynamic Model/Subsystem/Get Parameter9
SimulinkParameterGetter<real64_T, double> ParamGet_dynamic_simulation_142;

void slros_node_init(int argc, char** argv)
{
  ros::init(argc, argv, SLROSNodeName);
  SLROSNodePtr = new ros::NodeHandle();
}

