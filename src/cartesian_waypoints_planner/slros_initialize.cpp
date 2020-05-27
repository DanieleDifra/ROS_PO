#include "slros_initialize.h"

ros::NodeHandle * SLROSNodePtr;
const std::string SLROSNodeName = "cartesian_waypoints_planner";

// For Block cartesian_waypoints_planner/Subscribe1
SimulinkSubscriber<rosgraph_msgs::Clock, SL_Bus_cartesian_waypoints_planner_rosgraph_msgs_Clock> Sub_cartesian_waypoints_planner_384;

// For Block cartesian_waypoints_planner/Publish
SimulinkPublisher<trajectory_msgs::JointTrajectoryPoint, SL_Bus_cartesian_waypoints_plann_JointTrajectoryPoint_rocwzq> Pub_cartesian_waypoints_planner_392;

// For Block cartesian_waypoints_planner/Get Parameter
SimulinkParameterGetter<real64_T, double> ParamGet_cartesian_waypoints_planner_378;

// For Block cartesian_waypoints_planner/Get Parameter1
SimulinkParameterGetter<real64_T, double> ParamGet_cartesian_waypoints_planner_379;

// For Block cartesian_waypoints_planner/Get Parameter2
SimulinkParameterGetter<real64_T, double> ParamGet_cartesian_waypoints_planner_380;

// For Block cartesian_waypoints_planner/Subsystem/Get Parameter
SimulinkParameterGetter<int32_T, int> ParamGet_cartesian_waypoints_planner_346;

// For Block cartesian_waypoints_planner/Subsystem/Subsystem/Get Parameter
SimulinkParameterGetter<real64_T, double> ParamGet_cartesian_waypoints_planner_288;

// For Block cartesian_waypoints_planner/Subsystem/Subsystem/Get Parameter1
SimulinkParameterGetter<real64_T, double> ParamGet_cartesian_waypoints_planner_289;

// For Block cartesian_waypoints_planner/Subsystem/Subsystem/Get Parameter2
SimulinkParameterGetter<real64_T, double> ParamGet_cartesian_waypoints_planner_290;

// For Block cartesian_waypoints_planner/Subsystem/Subsystem/Get Parameter3
SimulinkParameterGetter<real64_T, double> ParamGet_cartesian_waypoints_planner_291;

// For Block cartesian_waypoints_planner/Subsystem/Subsystem/Get Parameter4
SimulinkParameterGetter<real64_T, double> ParamGet_cartesian_waypoints_planner_292;

// For Block cartesian_waypoints_planner/Subsystem/Subsystem/Get Parameter5
SimulinkParameterGetter<real64_T, double> ParamGet_cartesian_waypoints_planner_293;

// For Block cartesian_waypoints_planner/Subsystem/Subsystem/Get Parameter6
SimulinkParameterGetter<real64_T, double> ParamGet_cartesian_waypoints_planner_294;

// For Block cartesian_waypoints_planner/Subsystem/Subsystem1/Get Parameter
SimulinkParameterGetter<real64_T, double> ParamGet_cartesian_waypoints_planner_300;

// For Block cartesian_waypoints_planner/Subsystem/Subsystem1/Get Parameter1
SimulinkParameterGetter<real64_T, double> ParamGet_cartesian_waypoints_planner_301;

// For Block cartesian_waypoints_planner/Subsystem/Subsystem1/Get Parameter2
SimulinkParameterGetter<real64_T, double> ParamGet_cartesian_waypoints_planner_302;

// For Block cartesian_waypoints_planner/Subsystem/Subsystem1/Get Parameter3
SimulinkParameterGetter<real64_T, double> ParamGet_cartesian_waypoints_planner_303;

// For Block cartesian_waypoints_planner/Subsystem/Subsystem1/Get Parameter4
SimulinkParameterGetter<real64_T, double> ParamGet_cartesian_waypoints_planner_304;

// For Block cartesian_waypoints_planner/Subsystem/Subsystem1/Get Parameter5
SimulinkParameterGetter<real64_T, double> ParamGet_cartesian_waypoints_planner_305;

// For Block cartesian_waypoints_planner/Subsystem/Subsystem1/Get Parameter6
SimulinkParameterGetter<real64_T, double> ParamGet_cartesian_waypoints_planner_306;

// For Block cartesian_waypoints_planner/Subsystem1/Get Parameter
SimulinkParameterGetter<real64_T, double> ParamGet_cartesian_waypoints_planner_312;

// For Block cartesian_waypoints_planner/Subsystem1/Get Parameter1
SimulinkParameterGetter<real64_T, double> ParamGet_cartesian_waypoints_planner_313;

// For Block cartesian_waypoints_planner/Subsystem1/Get Parameter2
SimulinkParameterGetter<real64_T, double> ParamGet_cartesian_waypoints_planner_314;

// For Block cartesian_waypoints_planner/Subsystem1/Get Parameter3
SimulinkParameterGetter<real64_T, double> ParamGet_cartesian_waypoints_planner_315;

// For Block cartesian_waypoints_planner/Subsystem1/Get Parameter4
SimulinkParameterGetter<real64_T, double> ParamGet_cartesian_waypoints_planner_316;

// For Block cartesian_waypoints_planner/Subsystem1/Get Parameter5
SimulinkParameterGetter<real64_T, double> ParamGet_cartesian_waypoints_planner_317;

// For Block cartesian_waypoints_planner/Subsystem2/Set Parameter
SimulinkParameterSetter<real64_T, double> ParamSet_cartesian_waypoints_planner_364;

// For Block cartesian_waypoints_planner/Subsystem2/Set Parameter1
SimulinkParameterSetter<real64_T, double> ParamSet_cartesian_waypoints_planner_365;

// For Block cartesian_waypoints_planner/Subsystem2/Set Parameter2
SimulinkParameterSetter<real64_T, double> ParamSet_cartesian_waypoints_planner_366;

// For Block cartesian_waypoints_planner/Subsystem2/Set Parameter3
SimulinkParameterSetter<real64_T, double> ParamSet_cartesian_waypoints_planner_371;

// For Block cartesian_waypoints_planner/Subsystem2/Set Parameter4
SimulinkParameterSetter<real64_T, double> ParamSet_cartesian_waypoints_planner_372;

// For Block cartesian_waypoints_planner/Subsystem2/Set Parameter5
SimulinkParameterSetter<real64_T, double> ParamSet_cartesian_waypoints_planner_373;

void slros_node_init(int argc, char** argv)
{
  ros::init(argc, argv, SLROSNodeName);
  SLROSNodePtr = new ros::NodeHandle();
}

