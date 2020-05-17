#include "slros_busmsg_conversion.h"


// Conversions between SL_Bus_joint_trajectory_planner_ros_time_Duration and ros::Duration

void convertFromBus(ros::Duration* msgPtr, SL_Bus_joint_trajectory_planner_ros_time_Duration const* busPtr)
{
  const std::string rosMessageType("ros_time/Duration");

  msgPtr->sec =  busPtr->Sec;
  msgPtr->nsec =  busPtr->Nsec;
}

void convertToBus(SL_Bus_joint_trajectory_planner_ros_time_Duration* busPtr, ros::Duration const* msgPtr)
{
  const std::string rosMessageType("ros_time/Duration");

  busPtr->Sec =  msgPtr->sec;
  busPtr->Nsec =  msgPtr->nsec;
}


// Conversions between SL_Bus_joint_trajectory_planner_ros_time_Time and ros::Time

void convertFromBus(ros::Time* msgPtr, SL_Bus_joint_trajectory_planner_ros_time_Time const* busPtr)
{
  const std::string rosMessageType("ros_time/Time");

  msgPtr->sec =  busPtr->Sec;
  msgPtr->nsec =  busPtr->Nsec;
}

void convertToBus(SL_Bus_joint_trajectory_planner_ros_time_Time* busPtr, ros::Time const* msgPtr)
{
  const std::string rosMessageType("ros_time/Time");

  busPtr->Sec =  msgPtr->sec;
  busPtr->Nsec =  msgPtr->nsec;
}


// Conversions between SL_Bus_joint_trajectory_planner_rosgraph_msgs_Clock and rosgraph_msgs::Clock

void convertFromBus(rosgraph_msgs::Clock* msgPtr, SL_Bus_joint_trajectory_planner_rosgraph_msgs_Clock const* busPtr)
{
  const std::string rosMessageType("rosgraph_msgs/Clock");

  convertFromBus(&msgPtr->clock, &busPtr->Clock_);
}

void convertToBus(SL_Bus_joint_trajectory_planner_rosgraph_msgs_Clock* busPtr, rosgraph_msgs::Clock const* msgPtr)
{
  const std::string rosMessageType("rosgraph_msgs/Clock");

  convertToBus(&busPtr->Clock_, &msgPtr->clock);
}


// Conversions between SL_Bus_joint_trajectory_planner_JointTrajectoryPoint_y0wep and trajectory_msgs::JointTrajectoryPoint

void convertFromBus(trajectory_msgs::JointTrajectoryPoint* msgPtr, SL_Bus_joint_trajectory_planner_JointTrajectoryPoint_y0wep const* busPtr)
{
  const std::string rosMessageType("trajectory_msgs/JointTrajectoryPoint");

  convertFromBusVariablePrimitiveArray(msgPtr->accelerations, busPtr->Accelerations, busPtr->Accelerations_SL_Info);
  convertFromBusVariablePrimitiveArray(msgPtr->effort, busPtr->Effort, busPtr->Effort_SL_Info);
  convertFromBusVariablePrimitiveArray(msgPtr->positions, busPtr->Positions, busPtr->Positions_SL_Info);
  convertFromBus(&msgPtr->time_from_start, &busPtr->TimeFromStart);
  convertFromBusVariablePrimitiveArray(msgPtr->velocities, busPtr->Velocities, busPtr->Velocities_SL_Info);
}

void convertToBus(SL_Bus_joint_trajectory_planner_JointTrajectoryPoint_y0wep* busPtr, trajectory_msgs::JointTrajectoryPoint const* msgPtr)
{
  const std::string rosMessageType("trajectory_msgs/JointTrajectoryPoint");

  convertToBusVariablePrimitiveArray(busPtr->Accelerations, busPtr->Accelerations_SL_Info, msgPtr->accelerations, slros::EnabledWarning(rosMessageType, "accelerations"));
  convertToBusVariablePrimitiveArray(busPtr->Effort, busPtr->Effort_SL_Info, msgPtr->effort, slros::EnabledWarning(rosMessageType, "effort"));
  convertToBusVariablePrimitiveArray(busPtr->Positions, busPtr->Positions_SL_Info, msgPtr->positions, slros::EnabledWarning(rosMessageType, "positions"));
  convertToBus(&busPtr->TimeFromStart, &msgPtr->time_from_start);
  convertToBusVariablePrimitiveArray(busPtr->Velocities, busPtr->Velocities_SL_Info, msgPtr->velocities, slros::EnabledWarning(rosMessageType, "velocities"));
}

