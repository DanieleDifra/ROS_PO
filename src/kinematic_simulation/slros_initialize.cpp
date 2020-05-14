#include "slros_initialize.h"

ros::NodeHandle * SLROSNodePtr;
const std::string SLROSNodeName = "kinematic_simulation";

// For Block kinematic_simulation/Subscribe
SimulinkSubscriber<trajectory_msgs::JointTrajectoryPoint, SL_Bus_kinematic_simulation_JointTrajectoryPoint_qcxomb> Sub_kinematic_simulation_16;

// For Block kinematic_simulation/Publish
SimulinkPublisher<sensor_msgs::JointState, SL_Bus_kinematic_simulation_sensor_msgs_JointState> Pub_kinematic_simulation_22;

// For Block kinematic_simulation/Publish1
SimulinkPublisher<rosgraph_msgs::Clock, SL_Bus_kinematic_simulation_rosgraph_msgs_Clock> Pub_kinematic_simulation_50;

// For Block kinematic_simulation/Publish2
SimulinkPublisher<geometry_msgs::PoseStamped, SL_Bus_kinematic_simulation_geometry_msgs_PoseStamped> Pub_kinematic_simulation_79;

// For Block kinematic_simulation/Publish3
SimulinkPublisher<geometry_msgs::TwistStamped, SL_Bus_kinematic_simulation_geometry_msgs_TwistStamped> Pub_kinematic_simulation_101;

// For Block kinematic_simulation/Subsystem/Get Parameter
SimulinkParameterGetter<real64_T, double> ParamGet_kinematic_simulation_61;

// For Block kinematic_simulation/Subsystem/Get Parameter1
SimulinkParameterGetter<real64_T, double> ParamGet_kinematic_simulation_65;

// For Block kinematic_simulation/Subsystem/Get Parameter2
SimulinkParameterGetter<real64_T, double> ParamGet_kinematic_simulation_85;

// For Block kinematic_simulation/Subsystem/Get Parameter3
SimulinkParameterGetter<real64_T, double> ParamGet_kinematic_simulation_86;

// For Block kinematic_simulation/Subsystem/Get Parameter4
SimulinkParameterGetter<real64_T, double> ParamGet_kinematic_simulation_87;

// For Block kinematic_simulation/Subsystem/Get Parameter5
SimulinkParameterGetter<real64_T, double> ParamGet_kinematic_simulation_88;

void slros_node_init(int argc, char** argv)
{
  ros::init(argc, argv, SLROSNodeName);
  SLROSNodePtr = new ros::NodeHandle();
}

