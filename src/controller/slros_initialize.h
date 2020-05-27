#ifndef _SLROS_INITIALIZE_H_
#define _SLROS_INITIALIZE_H_

#include "slros_busmsg_conversion.h"
#include "slros_generic.h"

extern ros::NodeHandle * SLROSNodePtr;
extern const std::string SLROSNodeName;

// For Block controller/Subscribe
extern SimulinkSubscriber<sensor_msgs::JointState, SL_Bus_controller_sensor_msgs_JointState> Sub_controller_5;

// For Block controller/Subscribe1
extern SimulinkSubscriber<trajectory_msgs::JointTrajectoryPoint, SL_Bus_controller_trajectory_msgs_JointTrajectoryPoint> Sub_controller_136;

// For Block controller/Publish
extern SimulinkPublisher<std_msgs::Float64MultiArray, SL_Bus_controller_std_msgs_Float64MultiArray> Pub_controller_4;

// For Block controller/Subsystem/Get Parameter
extern SimulinkParameterGetter<real64_T, double> ParamGet_controller_93;

// For Block controller/Subsystem/Get Parameter1
extern SimulinkParameterGetter<real64_T, double> ParamGet_controller_94;

// For Block controller/Subsystem/Get Parameter2
extern SimulinkParameterGetter<real64_T, double> ParamGet_controller_95;

// For Block controller/Subsystem/Get Parameter3
extern SimulinkParameterGetter<real64_T, double> ParamGet_controller_96;

// For Block controller/Subsystem/Get Parameter4
extern SimulinkParameterGetter<real64_T, double> ParamGet_controller_97;

// For Block controller/Subsystem/Get Parameter5
extern SimulinkParameterGetter<real64_T, double> ParamGet_controller_98;

// For Block controller/Subsystem1/Get Parameter
extern SimulinkParameterGetter<real64_T, double> ParamGet_controller_103;

// For Block controller/Subsystem1/Get Parameter1
extern SimulinkParameterGetter<real64_T, double> ParamGet_controller_104;

// For Block controller/Subsystem1/Get Parameter2
extern SimulinkParameterGetter<real64_T, double> ParamGet_controller_105;

// For Block controller/Subsystem1/Get Parameter3
extern SimulinkParameterGetter<real64_T, double> ParamGet_controller_106;

// For Block controller/Subsystem1/Get Parameter4
extern SimulinkParameterGetter<real64_T, double> ParamGet_controller_107;

// For Block controller/Subsystem1/Get Parameter5
extern SimulinkParameterGetter<real64_T, double> ParamGet_controller_108;

void slros_node_init(int argc, char** argv);

#endif
