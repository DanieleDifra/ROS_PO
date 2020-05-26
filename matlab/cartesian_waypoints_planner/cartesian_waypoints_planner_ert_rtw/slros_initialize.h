#ifndef _SLROS_INITIALIZE_H_
#define _SLROS_INITIALIZE_H_

#include "slros_busmsg_conversion.h"
#include "slros_generic.h"

extern ros::NodeHandle * SLROSNodePtr;
extern const std::string SLROSNodeName;

// For Block cartesian_waypoints_planner/Publish
extern SimulinkPublisher<std_msgs::Float32MultiArray, SL_Bus_cartesian_waypoints_plann_Float32MultiArray_iutvhs> Pub_cartesian_waypoints_planner_344;

// For Block cartesian_waypoints_planner/Subsystem/Get Parameter
extern SimulinkParameterGetter<int32_T, int> ParamGet_cartesian_waypoints_planner_346;

// For Block cartesian_waypoints_planner/Subsystem/Subsystem/Get Parameter
extern SimulinkParameterGetter<real64_T, double> ParamGet_cartesian_waypoints_planner_288;

// For Block cartesian_waypoints_planner/Subsystem/Subsystem/Get Parameter1
extern SimulinkParameterGetter<real64_T, double> ParamGet_cartesian_waypoints_planner_289;

// For Block cartesian_waypoints_planner/Subsystem/Subsystem/Get Parameter2
extern SimulinkParameterGetter<real64_T, double> ParamGet_cartesian_waypoints_planner_290;

// For Block cartesian_waypoints_planner/Subsystem/Subsystem/Get Parameter3
extern SimulinkParameterGetter<real64_T, double> ParamGet_cartesian_waypoints_planner_291;

// For Block cartesian_waypoints_planner/Subsystem/Subsystem/Get Parameter4
extern SimulinkParameterGetter<real64_T, double> ParamGet_cartesian_waypoints_planner_292;

// For Block cartesian_waypoints_planner/Subsystem/Subsystem/Get Parameter5
extern SimulinkParameterGetter<real64_T, double> ParamGet_cartesian_waypoints_planner_293;

// For Block cartesian_waypoints_planner/Subsystem/Subsystem/Get Parameter6
extern SimulinkParameterGetter<real64_T, double> ParamGet_cartesian_waypoints_planner_294;

// For Block cartesian_waypoints_planner/Subsystem/Subsystem1/Get Parameter
extern SimulinkParameterGetter<real64_T, double> ParamGet_cartesian_waypoints_planner_300;

// For Block cartesian_waypoints_planner/Subsystem/Subsystem1/Get Parameter1
extern SimulinkParameterGetter<real64_T, double> ParamGet_cartesian_waypoints_planner_301;

// For Block cartesian_waypoints_planner/Subsystem/Subsystem1/Get Parameter2
extern SimulinkParameterGetter<real64_T, double> ParamGet_cartesian_waypoints_planner_302;

// For Block cartesian_waypoints_planner/Subsystem/Subsystem1/Get Parameter3
extern SimulinkParameterGetter<real64_T, double> ParamGet_cartesian_waypoints_planner_303;

// For Block cartesian_waypoints_planner/Subsystem/Subsystem1/Get Parameter4
extern SimulinkParameterGetter<real64_T, double> ParamGet_cartesian_waypoints_planner_304;

// For Block cartesian_waypoints_planner/Subsystem/Subsystem1/Get Parameter5
extern SimulinkParameterGetter<real64_T, double> ParamGet_cartesian_waypoints_planner_305;

// For Block cartesian_waypoints_planner/Subsystem/Subsystem1/Get Parameter6
extern SimulinkParameterGetter<real64_T, double> ParamGet_cartesian_waypoints_planner_306;

// For Block cartesian_waypoints_planner/Subsystem1/Get Parameter
extern SimulinkParameterGetter<real64_T, double> ParamGet_cartesian_waypoints_planner_312;

// For Block cartesian_waypoints_planner/Subsystem1/Get Parameter1
extern SimulinkParameterGetter<real64_T, double> ParamGet_cartesian_waypoints_planner_313;

// For Block cartesian_waypoints_planner/Subsystem1/Get Parameter2
extern SimulinkParameterGetter<real64_T, double> ParamGet_cartesian_waypoints_planner_314;

// For Block cartesian_waypoints_planner/Subsystem1/Get Parameter3
extern SimulinkParameterGetter<real64_T, double> ParamGet_cartesian_waypoints_planner_315;

// For Block cartesian_waypoints_planner/Subsystem1/Get Parameter4
extern SimulinkParameterGetter<real64_T, double> ParamGet_cartesian_waypoints_planner_316;

// For Block cartesian_waypoints_planner/Subsystem1/Get Parameter5
extern SimulinkParameterGetter<real64_T, double> ParamGet_cartesian_waypoints_planner_317;

// For Block cartesian_waypoints_planner/Subsystem2/Set Parameter
extern SimulinkParameterSetter<real64_T, double> ParamSet_cartesian_waypoints_planner_364;

// For Block cartesian_waypoints_planner/Subsystem2/Set Parameter1
extern SimulinkParameterSetter<real64_T, double> ParamSet_cartesian_waypoints_planner_365;

// For Block cartesian_waypoints_planner/Subsystem2/Set Parameter2
extern SimulinkParameterSetter<real64_T, double> ParamSet_cartesian_waypoints_planner_366;

// For Block cartesian_waypoints_planner/Subsystem2/Set Parameter3
extern SimulinkParameterSetter<real64_T, double> ParamSet_cartesian_waypoints_planner_371;

// For Block cartesian_waypoints_planner/Subsystem2/Set Parameter4
extern SimulinkParameterSetter<real64_T, double> ParamSet_cartesian_waypoints_planner_372;

// For Block cartesian_waypoints_planner/Subsystem2/Set Parameter5
extern SimulinkParameterSetter<real64_T, double> ParamSet_cartesian_waypoints_planner_373;

void slros_node_init(int argc, char** argv);

#endif
