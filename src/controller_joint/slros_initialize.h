#ifndef _SLROS_INITIALIZE_H_
#define _SLROS_INITIALIZE_H_

#include "slros_busmsg_conversion.h"
#include "slros_generic.h"

extern ros::NodeHandle * SLROSNodePtr;
extern const std::string SLROSNodeName;

// For Block controller_joint/Subscribe
extern SimulinkSubscriber<sensor_msgs::JointState, SL_Bus_controller_joint_sensor_msgs_JointState> Sub_controller_joint_5;

// For Block controller_joint/Subscribe1
extern SimulinkSubscriber<rosgraph_msgs::Clock, SL_Bus_controller_joint_rosgraph_msgs_Clock> Sub_controller_joint_22;

// For Block controller_joint/Publish
extern SimulinkPublisher<std_msgs::Float64MultiArray, SL_Bus_controller_joint_std_msgs_Float64MultiArray> Pub_controller_joint_4;

// For Block controller_joint/Get Parameter
extern SimulinkParameterGetter<real64_T, double> ParamGet_controller_joint_55;

// For Block controller_joint/Subsystem/Get Parameter
extern SimulinkParameterGetter<real64_T, double> ParamGet_controller_joint_93;

// For Block controller_joint/Subsystem/Get Parameter1
extern SimulinkParameterGetter<real64_T, double> ParamGet_controller_joint_94;

// For Block controller_joint/Subsystem/Get Parameter2
extern SimulinkParameterGetter<real64_T, double> ParamGet_controller_joint_95;

// For Block controller_joint/Subsystem/Get Parameter3
extern SimulinkParameterGetter<real64_T, double> ParamGet_controller_joint_96;

// For Block controller_joint/Subsystem/Get Parameter4
extern SimulinkParameterGetter<real64_T, double> ParamGet_controller_joint_97;

// For Block controller_joint/Subsystem/Get Parameter5
extern SimulinkParameterGetter<real64_T, double> ParamGet_controller_joint_98;

// For Block controller_joint/Subsystem1/Get Parameter
extern SimulinkParameterGetter<real64_T, double> ParamGet_controller_joint_103;

// For Block controller_joint/Subsystem1/Get Parameter1
extern SimulinkParameterGetter<real64_T, double> ParamGet_controller_joint_104;

// For Block controller_joint/Subsystem1/Get Parameter2
extern SimulinkParameterGetter<real64_T, double> ParamGet_controller_joint_105;

// For Block controller_joint/Subsystem1/Get Parameter3
extern SimulinkParameterGetter<real64_T, double> ParamGet_controller_joint_106;

// For Block controller_joint/Subsystem1/Get Parameter4
extern SimulinkParameterGetter<real64_T, double> ParamGet_controller_joint_107;

// For Block controller_joint/Subsystem1/Get Parameter5
extern SimulinkParameterGetter<real64_T, double> ParamGet_controller_joint_108;

// For Block controller_joint/Subsystem2/Get Parameter
extern SimulinkParameterGetter<real64_T, double> ParamGet_controller_joint_115;

// For Block controller_joint/Subsystem2/Get Parameter1
extern SimulinkParameterGetter<real64_T, double> ParamGet_controller_joint_116;

// For Block controller_joint/Subsystem2/Get Parameter2
extern SimulinkParameterGetter<real64_T, double> ParamGet_controller_joint_117;

// For Block controller_joint/Subsystem2/Get Parameter3
extern SimulinkParameterGetter<real64_T, double> ParamGet_controller_joint_118;

// For Block controller_joint/Subsystem2/Get Parameter4
extern SimulinkParameterGetter<real64_T, double> ParamGet_controller_joint_119;

// For Block controller_joint/Subsystem2/Get Parameter5
extern SimulinkParameterGetter<real64_T, double> ParamGet_controller_joint_120;

// For Block controller_joint/Subsystem3/Get Parameter
extern SimulinkParameterGetter<real64_T, double> ParamGet_controller_joint_127;

// For Block controller_joint/Subsystem3/Get Parameter1
extern SimulinkParameterGetter<real64_T, double> ParamGet_controller_joint_128;

// For Block controller_joint/Subsystem3/Get Parameter2
extern SimulinkParameterGetter<real64_T, double> ParamGet_controller_joint_129;

// For Block controller_joint/Subsystem3/Get Parameter3
extern SimulinkParameterGetter<real64_T, double> ParamGet_controller_joint_130;

// For Block controller_joint/Subsystem3/Get Parameter4
extern SimulinkParameterGetter<real64_T, double> ParamGet_controller_joint_131;

// For Block controller_joint/Subsystem3/Get Parameter5
extern SimulinkParameterGetter<real64_T, double> ParamGet_controller_joint_132;

void slros_node_init(int argc, char** argv);

#endif
