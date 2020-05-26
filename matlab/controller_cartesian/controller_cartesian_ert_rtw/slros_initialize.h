#ifndef _SLROS_INITIALIZE_H_
#define _SLROS_INITIALIZE_H_

#include "slros_busmsg_conversion.h"
#include "slros_generic.h"

extern ros::NodeHandle * SLROSNodePtr;
extern const std::string SLROSNodeName;

// For Block controller_cartesian/Subscribe
extern SimulinkSubscriber<sensor_msgs::JointState, SL_Bus_controller_cartesian_sensor_msgs_JointState> Sub_controller_cartesian_18;

// For Block controller_cartesian/Subscribe1
extern SimulinkSubscriber<rosgraph_msgs::Clock, SL_Bus_controller_cartesian_rosgraph_msgs_Clock> Sub_controller_cartesian_19;

// For Block controller_cartesian/Subscribe2
extern SimulinkSubscriber<std_msgs::Float32MultiArray, SL_Bus_controller_cartesian_std_msgs_Float32MultiArray> Sub_controller_cartesian_20;

// For Block controller_cartesian/Publish
extern SimulinkPublisher<std_msgs::Float64MultiArray, SL_Bus_controller_cartesian_std_msgs_Float64MultiArray> Pub_controller_cartesian_15;

// For Block controller_cartesian/Get Parameter
extern SimulinkParameterGetter<real64_T, double> ParamGet_controller_cartesian_6;

// For Block controller_cartesian/Get Parameter1
extern SimulinkParameterGetter<real64_T, double> ParamGet_controller_cartesian_7;

// For Block controller_cartesian/Get Parameter2
extern SimulinkParameterGetter<real64_T, double> ParamGet_controller_cartesian_8;

// For Block controller_cartesian/Subsystem/Get Parameter
extern SimulinkParameterGetter<real64_T, double> ParamGet_controller_cartesian_22;

// For Block controller_cartesian/Subsystem/Get Parameter1
extern SimulinkParameterGetter<real64_T, double> ParamGet_controller_cartesian_23;

// For Block controller_cartesian/Subsystem/Get Parameter2
extern SimulinkParameterGetter<real64_T, double> ParamGet_controller_cartesian_24;

// For Block controller_cartesian/Subsystem/Get Parameter3
extern SimulinkParameterGetter<real64_T, double> ParamGet_controller_cartesian_25;

// For Block controller_cartesian/Subsystem/Get Parameter4
extern SimulinkParameterGetter<real64_T, double> ParamGet_controller_cartesian_26;

// For Block controller_cartesian/Subsystem/Get Parameter5
extern SimulinkParameterGetter<real64_T, double> ParamGet_controller_cartesian_27;

// For Block controller_cartesian/Subsystem1/Get Parameter
extern SimulinkParameterGetter<real64_T, double> ParamGet_controller_cartesian_31;

// For Block controller_cartesian/Subsystem1/Get Parameter1
extern SimulinkParameterGetter<real64_T, double> ParamGet_controller_cartesian_32;

// For Block controller_cartesian/Subsystem1/Get Parameter2
extern SimulinkParameterGetter<real64_T, double> ParamGet_controller_cartesian_33;

// For Block controller_cartesian/Subsystem1/Get Parameter3
extern SimulinkParameterGetter<real64_T, double> ParamGet_controller_cartesian_34;

// For Block controller_cartesian/Subsystem1/Get Parameter4
extern SimulinkParameterGetter<real64_T, double> ParamGet_controller_cartesian_35;

// For Block controller_cartesian/Subsystem1/Get Parameter5
extern SimulinkParameterGetter<real64_T, double> ParamGet_controller_cartesian_36;

void slros_node_init(int argc, char** argv);

#endif
