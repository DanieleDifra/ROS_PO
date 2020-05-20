#ifndef _SLROS_INITIALIZE_H_
#define _SLROS_INITIALIZE_H_

#include "slros_busmsg_conversion.h"
#include "slros_generic.h"

extern ros::NodeHandle * SLROSNodePtr;
extern const std::string SLROSNodeName;

// For Block cartesian_trajectory_planner/Subscribe
extern SimulinkSubscriber<rosgraph_msgs::Clock, SL_Bus_cartesian_trajectory_planner_rosgraph_msgs_Clock> Sub_cartesian_trajectory_planner_280;

// For Block cartesian_trajectory_planner/Publish
extern SimulinkPublisher<trajectory_msgs::JointTrajectoryPoint, SL_Bus_cartesian_trajectory_plan_JointTrajectoryPoint_1eqsod> Pub_cartesian_trajectory_planner_278;

// For Block cartesian_trajectory_planner/Get Parameter
extern SimulinkParameterGetter<real64_T, double> ParamGet_cartesian_trajectory_planner_271;

// For Block cartesian_trajectory_planner/Get Parameter1
extern SimulinkParameterGetter<real64_T, double> ParamGet_cartesian_trajectory_planner_272;

// For Block cartesian_trajectory_planner/Subsystem/Get Parameter
extern SimulinkParameterGetter<real64_T, double> ParamGet_cartesian_trajectory_planner_283;

// For Block cartesian_trajectory_planner/Subsystem/Get Parameter1
extern SimulinkParameterGetter<real64_T, double> ParamGet_cartesian_trajectory_planner_284;

// For Block cartesian_trajectory_planner/Subsystem/Subsystem/Get Parameter
extern SimulinkParameterGetter<real64_T, double> ParamGet_cartesian_trajectory_planner_288;

// For Block cartesian_trajectory_planner/Subsystem/Subsystem/Get Parameter1
extern SimulinkParameterGetter<real64_T, double> ParamGet_cartesian_trajectory_planner_289;

// For Block cartesian_trajectory_planner/Subsystem/Subsystem/Get Parameter2
extern SimulinkParameterGetter<real64_T, double> ParamGet_cartesian_trajectory_planner_290;

// For Block cartesian_trajectory_planner/Subsystem/Subsystem/Get Parameter3
extern SimulinkParameterGetter<real64_T, double> ParamGet_cartesian_trajectory_planner_291;

// For Block cartesian_trajectory_planner/Subsystem/Subsystem/Get Parameter4
extern SimulinkParameterGetter<real64_T, double> ParamGet_cartesian_trajectory_planner_292;

// For Block cartesian_trajectory_planner/Subsystem/Subsystem/Get Parameter5
extern SimulinkParameterGetter<real64_T, double> ParamGet_cartesian_trajectory_planner_293;

// For Block cartesian_trajectory_planner/Subsystem/Subsystem/Get Parameter6
extern SimulinkParameterGetter<real64_T, double> ParamGet_cartesian_trajectory_planner_294;

// For Block cartesian_trajectory_planner/Subsystem/Subsystem1/Get Parameter
extern SimulinkParameterGetter<real64_T, double> ParamGet_cartesian_trajectory_planner_300;

// For Block cartesian_trajectory_planner/Subsystem/Subsystem1/Get Parameter1
extern SimulinkParameterGetter<real64_T, double> ParamGet_cartesian_trajectory_planner_301;

// For Block cartesian_trajectory_planner/Subsystem/Subsystem1/Get Parameter2
extern SimulinkParameterGetter<real64_T, double> ParamGet_cartesian_trajectory_planner_302;

// For Block cartesian_trajectory_planner/Subsystem/Subsystem1/Get Parameter3
extern SimulinkParameterGetter<real64_T, double> ParamGet_cartesian_trajectory_planner_303;

// For Block cartesian_trajectory_planner/Subsystem/Subsystem1/Get Parameter4
extern SimulinkParameterGetter<real64_T, double> ParamGet_cartesian_trajectory_planner_304;

// For Block cartesian_trajectory_planner/Subsystem/Subsystem1/Get Parameter5
extern SimulinkParameterGetter<real64_T, double> ParamGet_cartesian_trajectory_planner_305;

// For Block cartesian_trajectory_planner/Subsystem/Subsystem1/Get Parameter6
extern SimulinkParameterGetter<real64_T, double> ParamGet_cartesian_trajectory_planner_306;

// For Block cartesian_trajectory_planner/Subsystem1/Get Parameter
extern SimulinkParameterGetter<real64_T, double> ParamGet_cartesian_trajectory_planner_312;

// For Block cartesian_trajectory_planner/Subsystem1/Get Parameter1
extern SimulinkParameterGetter<real64_T, double> ParamGet_cartesian_trajectory_planner_313;

// For Block cartesian_trajectory_planner/Subsystem1/Get Parameter2
extern SimulinkParameterGetter<real64_T, double> ParamGet_cartesian_trajectory_planner_314;

// For Block cartesian_trajectory_planner/Subsystem1/Get Parameter3
extern SimulinkParameterGetter<real64_T, double> ParamGet_cartesian_trajectory_planner_315;

// For Block cartesian_trajectory_planner/Subsystem1/Get Parameter4
extern SimulinkParameterGetter<real64_T, double> ParamGet_cartesian_trajectory_planner_316;

// For Block cartesian_trajectory_planner/Subsystem1/Get Parameter5
extern SimulinkParameterGetter<real64_T, double> ParamGet_cartesian_trajectory_planner_317;

void slros_node_init(int argc, char** argv);

#endif
