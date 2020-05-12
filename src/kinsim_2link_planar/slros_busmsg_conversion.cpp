#include "slros_busmsg_conversion.h"


// Conversions between SL_Bus_kinsim_2link_planar_ros_time_Duration and ros::Duration

void convertFromBus(ros::Duration* msgPtr, SL_Bus_kinsim_2link_planar_ros_time_Duration const* busPtr)
{
  const std::string rosMessageType("ros_time/Duration");

  msgPtr->sec =  busPtr->Sec;
  msgPtr->nsec =  busPtr->Nsec;
}

void convertToBus(SL_Bus_kinsim_2link_planar_ros_time_Duration* busPtr, ros::Duration const* msgPtr)
{
  const std::string rosMessageType("ros_time/Duration");

  busPtr->Sec =  msgPtr->sec;
  busPtr->Nsec =  msgPtr->nsec;
}


// Conversions between SL_Bus_kinsim_2link_planar_ros_time_Time and ros::Time

void convertFromBus(ros::Time* msgPtr, SL_Bus_kinsim_2link_planar_ros_time_Time const* busPtr)
{
  const std::string rosMessageType("ros_time/Time");

  msgPtr->sec =  busPtr->Sec;
  msgPtr->nsec =  busPtr->Nsec;
}

void convertToBus(SL_Bus_kinsim_2link_planar_ros_time_Time* busPtr, ros::Time const* msgPtr)
{
  const std::string rosMessageType("ros_time/Time");

  busPtr->Sec =  msgPtr->sec;
  busPtr->Nsec =  msgPtr->nsec;
}


// Conversions between SL_Bus_kinsim_2link_planar_rosgraph_msgs_Clock and rosgraph_msgs::Clock

void convertFromBus(rosgraph_msgs::Clock* msgPtr, SL_Bus_kinsim_2link_planar_rosgraph_msgs_Clock const* busPtr)
{
  const std::string rosMessageType("rosgraph_msgs/Clock");

  convertFromBus(&msgPtr->clock, &busPtr->Clock_);
}

void convertToBus(SL_Bus_kinsim_2link_planar_rosgraph_msgs_Clock* busPtr, rosgraph_msgs::Clock const* msgPtr)
{
  const std::string rosMessageType("rosgraph_msgs/Clock");

  convertToBus(&busPtr->Clock_, &msgPtr->clock);
}


// Conversions between SL_Bus_kinsim_2link_planar_sensor_msgs_JointState and sensor_msgs::JointState

void convertFromBus(sensor_msgs::JointState* msgPtr, SL_Bus_kinsim_2link_planar_sensor_msgs_JointState const* busPtr)
{
  const std::string rosMessageType("sensor_msgs/JointState");

  convertFromBusVariablePrimitiveArray(msgPtr->effort, busPtr->Effort, busPtr->Effort_SL_Info);
  convertFromBus(&msgPtr->header, &busPtr->Header);
  convertFromBusVariableStringArray(msgPtr->name, busPtr->Name, busPtr->Name_SL_Info);
  convertFromBusVariablePrimitiveArray(msgPtr->position, busPtr->Position, busPtr->Position_SL_Info);
  convertFromBusVariablePrimitiveArray(msgPtr->velocity, busPtr->Velocity, busPtr->Velocity_SL_Info);
}

void convertToBus(SL_Bus_kinsim_2link_planar_sensor_msgs_JointState* busPtr, sensor_msgs::JointState const* msgPtr)
{
  const std::string rosMessageType("sensor_msgs/JointState");

  convertToBusVariablePrimitiveArray(busPtr->Effort, busPtr->Effort_SL_Info, msgPtr->effort, slros::EnabledWarning(rosMessageType, "effort"));
  convertToBus(&busPtr->Header, &msgPtr->header);
  convertToBusVariableStringArray(busPtr->Name, busPtr->Name_SL_Info, msgPtr->name, slros::EnabledWarning(rosMessageType, "name"));
  convertToBusVariablePrimitiveArray(busPtr->Position, busPtr->Position_SL_Info, msgPtr->position, slros::EnabledWarning(rosMessageType, "position"));
  convertToBusVariablePrimitiveArray(busPtr->Velocity, busPtr->Velocity_SL_Info, msgPtr->velocity, slros::EnabledWarning(rosMessageType, "velocity"));
}


// Conversions between SL_Bus_kinsim_2link_planar_std_msgs_Header and std_msgs::Header

void convertFromBus(std_msgs::Header* msgPtr, SL_Bus_kinsim_2link_planar_std_msgs_Header const* busPtr)
{
  const std::string rosMessageType("std_msgs/Header");

  convertFromBusVariablePrimitiveArray(msgPtr->frame_id, busPtr->FrameId, busPtr->FrameId_SL_Info);
  msgPtr->seq =  busPtr->Seq;
  convertFromBus(&msgPtr->stamp, &busPtr->Stamp);
}

void convertToBus(SL_Bus_kinsim_2link_planar_std_msgs_Header* busPtr, std_msgs::Header const* msgPtr)
{
  const std::string rosMessageType("std_msgs/Header");

  convertToBusVariablePrimitiveArray(busPtr->FrameId, busPtr->FrameId_SL_Info, msgPtr->frame_id, slros::EnabledWarning(rosMessageType, "frame_id"));
  busPtr->Seq =  msgPtr->seq;
  convertToBus(&busPtr->Stamp, &msgPtr->stamp);
}


// Conversions between SL_Bus_kinsim_2link_planar_JointTrajectoryPoint_1csgl9 and trajectory_msgs::JointTrajectoryPoint

void convertFromBus(trajectory_msgs::JointTrajectoryPoint* msgPtr, SL_Bus_kinsim_2link_planar_JointTrajectoryPoint_1csgl9 const* busPtr)
{
  const std::string rosMessageType("trajectory_msgs/JointTrajectoryPoint");

  convertFromBusVariablePrimitiveArray(msgPtr->accelerations, busPtr->Accelerations, busPtr->Accelerations_SL_Info);
  convertFromBusVariablePrimitiveArray(msgPtr->effort, busPtr->Effort, busPtr->Effort_SL_Info);
  convertFromBusVariablePrimitiveArray(msgPtr->positions, busPtr->Positions, busPtr->Positions_SL_Info);
  convertFromBus(&msgPtr->time_from_start, &busPtr->TimeFromStart);
  convertFromBusVariablePrimitiveArray(msgPtr->velocities, busPtr->Velocities, busPtr->Velocities_SL_Info);
}

void convertToBus(SL_Bus_kinsim_2link_planar_JointTrajectoryPoint_1csgl9* busPtr, trajectory_msgs::JointTrajectoryPoint const* msgPtr)
{
  const std::string rosMessageType("trajectory_msgs/JointTrajectoryPoint");

  convertToBusVariablePrimitiveArray(busPtr->Accelerations, busPtr->Accelerations_SL_Info, msgPtr->accelerations, slros::EnabledWarning(rosMessageType, "accelerations"));
  convertToBusVariablePrimitiveArray(busPtr->Effort, busPtr->Effort_SL_Info, msgPtr->effort, slros::EnabledWarning(rosMessageType, "effort"));
  convertToBusVariablePrimitiveArray(busPtr->Positions, busPtr->Positions_SL_Info, msgPtr->positions, slros::EnabledWarning(rosMessageType, "positions"));
  convertToBus(&busPtr->TimeFromStart, &msgPtr->time_from_start);
  convertToBusVariablePrimitiveArray(busPtr->Velocities, busPtr->Velocities_SL_Info, msgPtr->velocities, slros::EnabledWarning(rosMessageType, "velocities"));
}

