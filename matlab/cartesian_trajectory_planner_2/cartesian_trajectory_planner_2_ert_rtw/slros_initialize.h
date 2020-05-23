#ifndef _SLROS_INITIALIZE_H_
#define _SLROS_INITIALIZE_H_

#include "slros_busmsg_conversion.h"
#include "slros_generic.h"

extern ros::NodeHandle * SLROSNodePtr;
extern const std::string SLROSNodeName;

// For Block cartesian_trajectory_planner_2/Subscribe
extern SimulinkSubscriber<rosgraph_msgs::Clock, SL_Bus_cartesian_trajectory_planner_2_rosgraph_msgs_Clock> Sub_cartesian_trajectory_planner_2_267;

// For Block cartesian_trajectory_planner_2/Publish
extern SimulinkPublisher<trajectory_msgs::JointTrajectoryPoint, SL_Bus_cartesian_trajectory_plan_JointTrajectoryPoint_na5m06> Pub_cartesian_trajectory_planner_2_266;

// For Block cartesian_trajectory_planner_2/Subsystem/Get Parameter
extern SimulinkParameterGetter<real64_T, double> ParamGet_cartesian_trajectory_planner_2_270;

// For Block cartesian_trajectory_planner_2/Subsystem/Get Parameter2
extern SimulinkParameterGetter<real64_T, double> ParamGet_cartesian_trajectory_planner_2_271;

// For Block cartesian_trajectory_planner_2/Subsystem/Get Parameter3
extern SimulinkParameterGetter<real64_T, double> ParamGet_cartesian_trajectory_planner_2_272;

// For Block cartesian_trajectory_planner_2/Subsystem/Subsystem/Get Parameter
extern SimulinkParameterGetter<real64_T, double> ParamGet_cartesian_trajectory_planner_2_275;

// For Block cartesian_trajectory_planner_2/Subsystem/Subsystem/Get Parameter1
extern SimulinkParameterGetter<real64_T, double> ParamGet_cartesian_trajectory_planner_2_276;

// For Block cartesian_trajectory_planner_2/Subsystem/Subsystem/Get Parameter2
extern SimulinkParameterGetter<real64_T, double> ParamGet_cartesian_trajectory_planner_2_277;

// For Block cartesian_trajectory_planner_2/Subsystem/Subsystem/Get Parameter3
extern SimulinkParameterGetter<real64_T, double> ParamGet_cartesian_trajectory_planner_2_278;

// For Block cartesian_trajectory_planner_2/Subsystem/Subsystem/Get Parameter4
extern SimulinkParameterGetter<real64_T, double> ParamGet_cartesian_trajectory_planner_2_279;

// For Block cartesian_trajectory_planner_2/Subsystem/Subsystem/Get Parameter5
extern SimulinkParameterGetter<real64_T, double> ParamGet_cartesian_trajectory_planner_2_280;

// For Block cartesian_trajectory_planner_2/Subsystem/Subsystem/Get Parameter6
extern SimulinkParameterGetter<real64_T, double> ParamGet_cartesian_trajectory_planner_2_281;

// For Block cartesian_trajectory_planner_2/Subsystem/Subsystem1/Get Parameter
extern SimulinkParameterGetter<real64_T, double> ParamGet_cartesian_trajectory_planner_2_287;

// For Block cartesian_trajectory_planner_2/Subsystem/Subsystem1/Get Parameter1
extern SimulinkParameterGetter<real64_T, double> ParamGet_cartesian_trajectory_planner_2_288;

// For Block cartesian_trajectory_planner_2/Subsystem/Subsystem1/Get Parameter2
extern SimulinkParameterGetter<real64_T, double> ParamGet_cartesian_trajectory_planner_2_289;

// For Block cartesian_trajectory_planner_2/Subsystem/Subsystem1/Get Parameter3
extern SimulinkParameterGetter<real64_T, double> ParamGet_cartesian_trajectory_planner_2_290;

// For Block cartesian_trajectory_planner_2/Subsystem/Subsystem1/Get Parameter4
extern SimulinkParameterGetter<real64_T, double> ParamGet_cartesian_trajectory_planner_2_291;

// For Block cartesian_trajectory_planner_2/Subsystem/Subsystem1/Get Parameter5
extern SimulinkParameterGetter<real64_T, double> ParamGet_cartesian_trajectory_planner_2_292;

// For Block cartesian_trajectory_planner_2/Subsystem/Subsystem1/Get Parameter6
extern SimulinkParameterGetter<real64_T, double> ParamGet_cartesian_trajectory_planner_2_293;

// For Block cartesian_trajectory_planner_2/weights/Get Parameter
extern SimulinkParameterGetter<real64_T, double> ParamGet_cartesian_trajectory_planner_2_299;

// For Block cartesian_trajectory_planner_2/weights/Get Parameter1
extern SimulinkParameterGetter<real64_T, double> ParamGet_cartesian_trajectory_planner_2_300;

// For Block cartesian_trajectory_planner_2/weights/Get Parameter2
extern SimulinkParameterGetter<real64_T, double> ParamGet_cartesian_trajectory_planner_2_301;

// For Block cartesian_trajectory_planner_2/weights/Get Parameter3
extern SimulinkParameterGetter<real64_T, double> ParamGet_cartesian_trajectory_planner_2_302;

// For Block cartesian_trajectory_planner_2/weights/Get Parameter4
extern SimulinkParameterGetter<real64_T, double> ParamGet_cartesian_trajectory_planner_2_303;

// For Block cartesian_trajectory_planner_2/weights/Get Parameter5
extern SimulinkParameterGetter<real64_T, double> ParamGet_cartesian_trajectory_planner_2_304;

// For Block cartesian_trajectory_planner_2/set_initial_conditions/Set Parameter
extern SimulinkParameterSetter<real64_T, double> ParamSet_cartesian_trajectory_planner_2_325;

// For Block cartesian_trajectory_planner_2/set_initial_conditions/Set Parameter1
extern SimulinkParameterSetter<real64_T, double> ParamSet_cartesian_trajectory_planner_2_326;

// For Block cartesian_trajectory_planner_2/set_initial_conditions/Set Parameter2
extern SimulinkParameterSetter<real64_T, double> ParamSet_cartesian_trajectory_planner_2_327;

// For Block cartesian_trajectory_planner_2/set_initial_conditions/Set Parameter3
extern SimulinkParameterSetter<real64_T, double> ParamSet_cartesian_trajectory_planner_2_328;

// For Block cartesian_trajectory_planner_2/set_initial_conditions/Set Parameter4
extern SimulinkParameterSetter<real64_T, double> ParamSet_cartesian_trajectory_planner_2_329;

// For Block cartesian_trajectory_planner_2/set_initial_conditions/Set Parameter5
extern SimulinkParameterSetter<real64_T, double> ParamSet_cartesian_trajectory_planner_2_330;

void slros_node_init(int argc, char** argv);

#endif
