#include "slros_initialize.h"

ros::NodeHandle * SLROSNodePtr;
const std::string SLROSNodeName = "controller_joint";

// For Block controller_joint/Subscribe
SimulinkSubscriber<sensor_msgs::JointState, SL_Bus_controller_joint_sensor_msgs_JointState> Sub_controller_joint_5;

// For Block controller_joint/Publish
SimulinkPublisher<std_msgs::Float64MultiArray, SL_Bus_controller_joint_std_msgs_Float64MultiArray> Pub_controller_joint_4;

// For Block controller_joint/Subsystem/Get Parameter
SimulinkParameterGetter<real64_T, double> ParamGet_controller_joint_93;

// For Block controller_joint/Subsystem/Get Parameter1
SimulinkParameterGetter<real64_T, double> ParamGet_controller_joint_94;

// For Block controller_joint/Subsystem/Get Parameter2
SimulinkParameterGetter<real64_T, double> ParamGet_controller_joint_95;

// For Block controller_joint/Subsystem/Get Parameter3
SimulinkParameterGetter<real64_T, double> ParamGet_controller_joint_96;

// For Block controller_joint/Subsystem/Get Parameter4
SimulinkParameterGetter<real64_T, double> ParamGet_controller_joint_97;

// For Block controller_joint/Subsystem/Get Parameter5
SimulinkParameterGetter<real64_T, double> ParamGet_controller_joint_98;

// For Block controller_joint/Subsystem1/Get Parameter
SimulinkParameterGetter<real64_T, double> ParamGet_controller_joint_103;

// For Block controller_joint/Subsystem1/Get Parameter1
SimulinkParameterGetter<real64_T, double> ParamGet_controller_joint_104;

// For Block controller_joint/Subsystem1/Get Parameter2
SimulinkParameterGetter<real64_T, double> ParamGet_controller_joint_105;

// For Block controller_joint/Subsystem1/Get Parameter3
SimulinkParameterGetter<real64_T, double> ParamGet_controller_joint_106;

// For Block controller_joint/Subsystem1/Get Parameter4
SimulinkParameterGetter<real64_T, double> ParamGet_controller_joint_107;

// For Block controller_joint/Subsystem1/Get Parameter5
SimulinkParameterGetter<real64_T, double> ParamGet_controller_joint_108;

// For Block controller_joint/Subsystem2/Get Parameter
SimulinkParameterGetter<real64_T, double> ParamGet_controller_joint_115;

// For Block controller_joint/Subsystem2/Get Parameter1
SimulinkParameterGetter<real64_T, double> ParamGet_controller_joint_116;

// For Block controller_joint/Subsystem2/Get Parameter2
SimulinkParameterGetter<real64_T, double> ParamGet_controller_joint_117;

// For Block controller_joint/Subsystem2/Get Parameter3
SimulinkParameterGetter<real64_T, double> ParamGet_controller_joint_118;

// For Block controller_joint/Subsystem2/Get Parameter4
SimulinkParameterGetter<real64_T, double> ParamGet_controller_joint_119;

// For Block controller_joint/Subsystem2/Get Parameter5
SimulinkParameterGetter<real64_T, double> ParamGet_controller_joint_120;

void slros_node_init(int argc, char** argv)
{
  ros::init(argc, argv, SLROSNodeName);
  SLROSNodePtr = new ros::NodeHandle();
}

