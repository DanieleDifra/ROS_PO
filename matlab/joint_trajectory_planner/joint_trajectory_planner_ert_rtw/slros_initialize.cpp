#include "slros_initialize.h"

ros::NodeHandle * SLROSNodePtr;
const std::string SLROSNodeName = "joint_trajectory_planner";

// For Block joint_trajectory_planner/Subscribe
SimulinkSubscriber<rosgraph_msgs::Clock, SL_Bus_joint_trajectory_planner_rosgraph_msgs_Clock> Sub_joint_trajectory_planner_15;

// For Block joint_trajectory_planner/Publish
SimulinkPublisher<trajectory_msgs::JointTrajectoryPoint, SL_Bus_joint_trajectory_planner_JointTrajectoryPoint_y0wep> Pub_joint_trajectory_planner_5;

// For Block joint_trajectory_planner/Get Parameters/Get Parameter
SimulinkParameterGetter<real64_T, double> ParamGet_joint_trajectory_planner_26;

// For Block joint_trajectory_planner/Get Parameters/Get Parameter1
SimulinkParameterGetter<real64_T, double> ParamGet_joint_trajectory_planner_27;

// For Block joint_trajectory_planner/Get Parameters/Get Parameter2
SimulinkParameterGetter<real64_T, double> ParamGet_joint_trajectory_planner_28;

// For Block joint_trajectory_planner/Get Parameters/Get Parameter3
SimulinkParameterGetter<real64_T, double> ParamGet_joint_trajectory_planner_49;

// For Block joint_trajectory_planner/Get Parameters/Get Parameter4
SimulinkParameterGetter<int32_T, int> ParamGet_joint_trajectory_planner_50;

// For Block joint_trajectory_planner/Get Parameters/Subsystem/Get Parameter
SimulinkParameterGetter<real64_T, double> ParamGet_joint_trajectory_planner_34;

// For Block joint_trajectory_planner/Get Parameters/Subsystem/Get Parameter1
SimulinkParameterGetter<real64_T, double> ParamGet_joint_trajectory_planner_35;

// For Block joint_trajectory_planner/Get Parameters/Subsystem/Get Parameter2
SimulinkParameterGetter<real64_T, double> ParamGet_joint_trajectory_planner_36;

// For Block joint_trajectory_planner/Get Parameters/Subsystem/Get Parameter3
SimulinkParameterGetter<real64_T, double> ParamGet_joint_trajectory_planner_37;

// For Block joint_trajectory_planner/Get Parameters/Subsystem/Get Parameter4
SimulinkParameterGetter<real64_T, double> ParamGet_joint_trajectory_planner_38;

// For Block joint_trajectory_planner/Get Parameters/Subsystem/Get Parameter5
SimulinkParameterGetter<real64_T, double> ParamGet_joint_trajectory_planner_39;

// For Block joint_trajectory_planner/Get Parameters/Subsystem1/Get Parameter
SimulinkParameterGetter<real64_T, double> ParamGet_joint_trajectory_planner_41;

// For Block joint_trajectory_planner/Get Parameters/Subsystem1/Get Parameter1
SimulinkParameterGetter<real64_T, double> ParamGet_joint_trajectory_planner_42;

// For Block joint_trajectory_planner/Get Parameters/Subsystem1/Get Parameter2
SimulinkParameterGetter<real64_T, double> ParamGet_joint_trajectory_planner_43;

// For Block joint_trajectory_planner/Get Parameters/Subsystem1/Get Parameter3
SimulinkParameterGetter<real64_T, double> ParamGet_joint_trajectory_planner_44;

// For Block joint_trajectory_planner/Get Parameters/Subsystem1/Get Parameter4
SimulinkParameterGetter<real64_T, double> ParamGet_joint_trajectory_planner_45;

// For Block joint_trajectory_planner/Get Parameters/Subsystem1/Get Parameter5
SimulinkParameterGetter<real64_T, double> ParamGet_joint_trajectory_planner_46;

void slros_node_init(int argc, char** argv)
{
  ros::init(argc, argv, SLROSNodeName);
  SLROSNodePtr = new ros::NodeHandle();
}

