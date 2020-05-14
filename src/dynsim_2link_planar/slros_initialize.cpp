#include "slros_initialize.h"

ros::NodeHandle * SLROSNodePtr;
const std::string SLROSNodeName = "dynsim_2link_planar";

// For Block dynsim_2link_planar/Subscribe
SimulinkSubscriber<std_msgs::Float64MultiArray, SL_Bus_dynsim_2link_planar_std_msgs_Float64MultiArray> Sub_dynsim_2link_planar_160;

// For Block dynsim_2link_planar/Publish
SimulinkPublisher<sensor_msgs::JointState, SL_Bus_dynsim_2link_planar_sensor_msgs_JointState> Pub_dynsim_2link_planar_157;

// For Block dynsim_2link_planar/Publish1
SimulinkPublisher<rosgraph_msgs::Clock, SL_Bus_dynsim_2link_planar_rosgraph_msgs_Clock> Pub_dynsim_2link_planar_158;

// For Block dynsim_2link_planar/Publish2
SimulinkPublisher<geometry_msgs::PoseStamped, SL_Bus_dynsim_2link_planar_geometry_msgs_PoseStamped> Pub_dynsim_2link_planar_178;

// For Block dynsim_2link_planar/Publish3
SimulinkPublisher<geometry_msgs::TwistStamped, SL_Bus_dynsim_2link_planar_geometry_msgs_TwistStamped> Pub_dynsim_2link_planar_179;

// For Block dynsim_2link_planar/2link_planar robot dynamic model/Subsystem/Get Parameter
SimulinkParameterGetter<real64_T, double> ParamGet_dynsim_2link_planar_131;

// For Block dynsim_2link_planar/2link_planar robot dynamic model/Subsystem/Get Parameter1
SimulinkParameterGetter<real64_T, double> ParamGet_dynsim_2link_planar_132;

// For Block dynsim_2link_planar/2link_planar robot dynamic model/Subsystem/Get Parameter10
SimulinkParameterGetter<real64_T, double> ParamGet_dynsim_2link_planar_133;

// For Block dynsim_2link_planar/2link_planar robot dynamic model/Subsystem/Get Parameter11
SimulinkParameterGetter<real64_T, double> ParamGet_dynsim_2link_planar_134;

// For Block dynsim_2link_planar/2link_planar robot dynamic model/Subsystem/Get Parameter2
SimulinkParameterGetter<real64_T, double> ParamGet_dynsim_2link_planar_135;

// For Block dynsim_2link_planar/2link_planar robot dynamic model/Subsystem/Get Parameter3
SimulinkParameterGetter<real64_T, double> ParamGet_dynsim_2link_planar_136;

// For Block dynsim_2link_planar/2link_planar robot dynamic model/Subsystem/Get Parameter4
SimulinkParameterGetter<real64_T, double> ParamGet_dynsim_2link_planar_137;

// For Block dynsim_2link_planar/2link_planar robot dynamic model/Subsystem/Get Parameter5
SimulinkParameterGetter<real64_T, double> ParamGet_dynsim_2link_planar_138;

// For Block dynsim_2link_planar/2link_planar robot dynamic model/Subsystem/Get Parameter6
SimulinkParameterGetter<real64_T, double> ParamGet_dynsim_2link_planar_139;

// For Block dynsim_2link_planar/2link_planar robot dynamic model/Subsystem/Get Parameter7
SimulinkParameterGetter<real64_T, double> ParamGet_dynsim_2link_planar_140;

// For Block dynsim_2link_planar/2link_planar robot dynamic model/Subsystem/Get Parameter8
SimulinkParameterGetter<real64_T, double> ParamGet_dynsim_2link_planar_141;

// For Block dynsim_2link_planar/2link_planar robot dynamic model/Subsystem/Get Parameter9
SimulinkParameterGetter<real64_T, double> ParamGet_dynsim_2link_planar_142;

void slros_node_init(int argc, char** argv)
{
  ros::init(argc, argv, SLROSNodeName);
  SLROSNodePtr = new ros::NodeHandle();
}
