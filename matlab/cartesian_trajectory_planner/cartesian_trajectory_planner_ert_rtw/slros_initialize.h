#ifndef _SLROS_INITIALIZE_H_
#define _SLROS_INITIALIZE_H_

#include "slros_busmsg_conversion.h"
#include "slros_generic.h"

extern ros::NodeHandle * SLROSNodePtr;
extern const std::string SLROSNodeName;

// For Block cartesian_trajectory_planner/Subscribe
extern SimulinkSubscriber<rosgraph_msgs::Clock, SL_Bus_cartesian_trajectory_planner_rosgraph_msgs_Clock> Sub_cartesian_trajectory_planner_269;

// For Block cartesian_trajectory_planner/Publish
extern SimulinkPublisher<trajectory_msgs::JointTrajectoryPoint, SL_Bus_cartesian_trajectory_plan_JointTrajectoryPoint_1eqsod> Pub_cartesian_trajectory_planner_267;

// For Block cartesian_trajectory_planner/Subsystem/Get Parameter
extern SimulinkParameterGetter<real64_T, double> ParamGet_cartesian_trajectory_planner_272;

// For Block cartesian_trajectory_planner/Subsystem/Get Parameter1
extern SimulinkParameterGetter<real64_T, double> ParamGet_cartesian_trajectory_planner_273;

// For Block cartesian_trajectory_planner/Subsystem/Subsystem/Get Parameter
extern SimulinkParameterGetter<real64_T, double> ParamGet_cartesian_trajectory_planner_277;

// For Block cartesian_trajectory_planner/Subsystem/Subsystem/Get Parameter1
extern SimulinkParameterGetter<real64_T, double> ParamGet_cartesian_trajectory_planner_278;

// For Block cartesian_trajectory_planner/Subsystem/Subsystem/Get Parameter2
extern SimulinkParameterGetter<real64_T, double> ParamGet_cartesian_trajectory_planner_279;

// For Block cartesian_trajectory_planner/Subsystem/Subsystem/Get Parameter3
extern SimulinkParameterGetter<real64_T, double> ParamGet_cartesian_trajectory_planner_280;

// For Block cartesian_trajectory_planner/Subsystem/Subsystem/Get Parameter4
extern SimulinkParameterGetter<real64_T, double> ParamGet_cartesian_trajectory_planner_281;

// For Block cartesian_trajectory_planner/Subsystem/Subsystem/Get Parameter5
extern SimulinkParameterGetter<real64_T, double> ParamGet_cartesian_trajectory_planner_282;

// For Block cartesian_trajectory_planner/Subsystem/Subsystem/Get Parameter6
extern SimulinkParameterGetter<real64_T, double> ParamGet_cartesian_trajectory_planner_283;

// For Block cartesian_trajectory_planner/Subsystem/Subsystem1/Get Parameter
extern SimulinkParameterGetter<real64_T, double> ParamGet_cartesian_trajectory_planner_289;

// For Block cartesian_trajectory_planner/Subsystem/Subsystem1/Get Parameter1
extern SimulinkParameterGetter<real64_T, double> ParamGet_cartesian_trajectory_planner_290;

// For Block cartesian_trajectory_planner/Subsystem/Subsystem1/Get Parameter2
extern SimulinkParameterGetter<real64_T, double> ParamGet_cartesian_trajectory_planner_291;

// For Block cartesian_trajectory_planner/Subsystem/Subsystem1/Get Parameter3
extern SimulinkParameterGetter<real64_T, double> ParamGet_cartesian_trajectory_planner_292;

// For Block cartesian_trajectory_planner/Subsystem/Subsystem1/Get Parameter4
extern SimulinkParameterGetter<real64_T, double> ParamGet_cartesian_trajectory_planner_293;

// For Block cartesian_trajectory_planner/Subsystem/Subsystem1/Get Parameter5
extern SimulinkParameterGetter<real64_T, double> ParamGet_cartesian_trajectory_planner_294;

// For Block cartesian_trajectory_planner/Subsystem/Subsystem1/Get Parameter6
extern SimulinkParameterGetter<real64_T, double> ParamGet_cartesian_trajectory_planner_295;

// For Block cartesian_trajectory_planner/Subsystem1/Get Parameter
extern SimulinkParameterGetter<real64_T, double> ParamGet_cartesian_trajectory_planner_301;

// For Block cartesian_trajectory_planner/Subsystem1/Get Parameter1
extern SimulinkParameterGetter<real64_T, double> ParamGet_cartesian_trajectory_planner_302;

// For Block cartesian_trajectory_planner/Subsystem1/Get Parameter2
extern SimulinkParameterGetter<real64_T, double> ParamGet_cartesian_trajectory_planner_303;

// For Block cartesian_trajectory_planner/Subsystem1/Get Parameter3
extern SimulinkParameterGetter<real64_T, double> ParamGet_cartesian_trajectory_planner_304;

// For Block cartesian_trajectory_planner/Subsystem1/Get Parameter4
extern SimulinkParameterGetter<real64_T, double> ParamGet_cartesian_trajectory_planner_305;

// For Block cartesian_trajectory_planner/Subsystem1/Get Parameter5
extern SimulinkParameterGetter<real64_T, double> ParamGet_cartesian_trajectory_planner_306;

void slros_node_init(int argc, char** argv);

#endif
