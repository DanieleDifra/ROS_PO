#include "slros_busmsg_conversion.h"


// Conversions between SL_Bus_cartesian_waypoints_planner_geometry_msgs_Point and geometry_msgs::Point

void convertFromBus(geometry_msgs::Point* msgPtr, SL_Bus_cartesian_waypoints_planner_geometry_msgs_Point const* busPtr)
{
  const std::string rosMessageType("geometry_msgs/Point");

  msgPtr->x =  busPtr->X;
  msgPtr->y =  busPtr->Y;
  msgPtr->z =  busPtr->Z;
}

void convertToBus(SL_Bus_cartesian_waypoints_planner_geometry_msgs_Point* busPtr, geometry_msgs::Point const* msgPtr)
{
  const std::string rosMessageType("geometry_msgs/Point");

  busPtr->X =  msgPtr->x;
  busPtr->Y =  msgPtr->y;
  busPtr->Z =  msgPtr->z;
}


// Conversions between SL_Bus_cartesian_waypoints_planner_geometry_msgs_Pose and geometry_msgs::Pose

void convertFromBus(geometry_msgs::Pose* msgPtr, SL_Bus_cartesian_waypoints_planner_geometry_msgs_Pose const* busPtr)
{
  const std::string rosMessageType("geometry_msgs/Pose");

  convertFromBus(&msgPtr->orientation, &busPtr->Orientation);
  convertFromBus(&msgPtr->position, &busPtr->Position);
}

void convertToBus(SL_Bus_cartesian_waypoints_planner_geometry_msgs_Pose* busPtr, geometry_msgs::Pose const* msgPtr)
{
  const std::string rosMessageType("geometry_msgs/Pose");

  convertToBus(&busPtr->Orientation, &msgPtr->orientation);
  convertToBus(&busPtr->Position, &msgPtr->position);
}


// Conversions between SL_Bus_cartesian_waypoints_plann_PoseStamped_hk4udn and geometry_msgs::PoseStamped

void convertFromBus(geometry_msgs::PoseStamped* msgPtr, SL_Bus_cartesian_waypoints_plann_PoseStamped_hk4udn const* busPtr)
{
  const std::string rosMessageType("geometry_msgs/PoseStamped");

  convertFromBus(&msgPtr->header, &busPtr->Header);
  convertFromBus(&msgPtr->pose, &busPtr->Pose);
}

void convertToBus(SL_Bus_cartesian_waypoints_plann_PoseStamped_hk4udn* busPtr, geometry_msgs::PoseStamped const* msgPtr)
{
  const std::string rosMessageType("geometry_msgs/PoseStamped");

  convertToBus(&busPtr->Header, &msgPtr->header);
  convertToBus(&busPtr->Pose, &msgPtr->pose);
}


// Conversions between SL_Bus_cartesian_waypoints_planner_geometry_msgs_Quaternion and geometry_msgs::Quaternion

void convertFromBus(geometry_msgs::Quaternion* msgPtr, SL_Bus_cartesian_waypoints_planner_geometry_msgs_Quaternion const* busPtr)
{
  const std::string rosMessageType("geometry_msgs/Quaternion");

  msgPtr->w =  busPtr->W;
  msgPtr->x =  busPtr->X;
  msgPtr->y =  busPtr->Y;
  msgPtr->z =  busPtr->Z;
}

void convertToBus(SL_Bus_cartesian_waypoints_planner_geometry_msgs_Quaternion* busPtr, geometry_msgs::Quaternion const* msgPtr)
{
  const std::string rosMessageType("geometry_msgs/Quaternion");

  busPtr->W =  msgPtr->w;
  busPtr->X =  msgPtr->x;
  busPtr->Y =  msgPtr->y;
  busPtr->Z =  msgPtr->z;
}


// Conversions between SL_Bus_cartesian_waypoints_planner_ros_time_Duration and ros::Duration

void convertFromBus(ros::Duration* msgPtr, SL_Bus_cartesian_waypoints_planner_ros_time_Duration const* busPtr)
{
  const std::string rosMessageType("ros_time/Duration");

  msgPtr->sec =  busPtr->Sec;
  msgPtr->nsec =  busPtr->Nsec;
}

void convertToBus(SL_Bus_cartesian_waypoints_planner_ros_time_Duration* busPtr, ros::Duration const* msgPtr)
{
  const std::string rosMessageType("ros_time/Duration");

  busPtr->Sec =  msgPtr->sec;
  busPtr->Nsec =  msgPtr->nsec;
}


// Conversions between SL_Bus_cartesian_waypoints_planner_ros_time_Time and ros::Time

void convertFromBus(ros::Time* msgPtr, SL_Bus_cartesian_waypoints_planner_ros_time_Time const* busPtr)
{
  const std::string rosMessageType("ros_time/Time");

  msgPtr->sec =  busPtr->Sec;
  msgPtr->nsec =  busPtr->Nsec;
}

void convertToBus(SL_Bus_cartesian_waypoints_planner_ros_time_Time* busPtr, ros::Time const* msgPtr)
{
  const std::string rosMessageType("ros_time/Time");

  busPtr->Sec =  msgPtr->sec;
  busPtr->Nsec =  msgPtr->nsec;
}


// Conversions between SL_Bus_cartesian_waypoints_planner_rosgraph_msgs_Clock and rosgraph_msgs::Clock

void convertFromBus(rosgraph_msgs::Clock* msgPtr, SL_Bus_cartesian_waypoints_planner_rosgraph_msgs_Clock const* busPtr)
{
  const std::string rosMessageType("rosgraph_msgs/Clock");

  convertFromBus(&msgPtr->clock, &busPtr->Clock_);
}

void convertToBus(SL_Bus_cartesian_waypoints_planner_rosgraph_msgs_Clock* busPtr, rosgraph_msgs::Clock const* msgPtr)
{
  const std::string rosMessageType("rosgraph_msgs/Clock");

  convertToBus(&busPtr->Clock_, &msgPtr->clock);
}


// Conversions between SL_Bus_cartesian_waypoints_planner_std_msgs_Header and std_msgs::Header

void convertFromBus(std_msgs::Header* msgPtr, SL_Bus_cartesian_waypoints_planner_std_msgs_Header const* busPtr)
{
  const std::string rosMessageType("std_msgs/Header");

  convertFromBusVariablePrimitiveArray(msgPtr->frame_id, busPtr->FrameId, busPtr->FrameId_SL_Info);
  msgPtr->seq =  busPtr->Seq;
  convertFromBus(&msgPtr->stamp, &busPtr->Stamp);
}

void convertToBus(SL_Bus_cartesian_waypoints_planner_std_msgs_Header* busPtr, std_msgs::Header const* msgPtr)
{
  const std::string rosMessageType("std_msgs/Header");

  convertToBusVariablePrimitiveArray(busPtr->FrameId, busPtr->FrameId_SL_Info, msgPtr->frame_id, slros::EnabledWarning(rosMessageType, "frame_id"));
  busPtr->Seq =  msgPtr->seq;
  convertToBus(&busPtr->Stamp, &msgPtr->stamp);
}


// Conversions between SL_Bus_cartesian_waypoints_plann_JointTrajectoryPoint_rocwzq and trajectory_msgs::JointTrajectoryPoint

void convertFromBus(trajectory_msgs::JointTrajectoryPoint* msgPtr, SL_Bus_cartesian_waypoints_plann_JointTrajectoryPoint_rocwzq const* busPtr)
{
  const std::string rosMessageType("trajectory_msgs/JointTrajectoryPoint");

  convertFromBusVariablePrimitiveArray(msgPtr->accelerations, busPtr->Accelerations, busPtr->Accelerations_SL_Info);
  convertFromBusVariablePrimitiveArray(msgPtr->effort, busPtr->Effort, busPtr->Effort_SL_Info);
  convertFromBusVariablePrimitiveArray(msgPtr->positions, busPtr->Positions, busPtr->Positions_SL_Info);
  convertFromBus(&msgPtr->time_from_start, &busPtr->TimeFromStart);
  convertFromBusVariablePrimitiveArray(msgPtr->velocities, busPtr->Velocities, busPtr->Velocities_SL_Info);
}

void convertToBus(SL_Bus_cartesian_waypoints_plann_JointTrajectoryPoint_rocwzq* busPtr, trajectory_msgs::JointTrajectoryPoint const* msgPtr)
{
  const std::string rosMessageType("trajectory_msgs/JointTrajectoryPoint");

  convertToBusVariablePrimitiveArray(busPtr->Accelerations, busPtr->Accelerations_SL_Info, msgPtr->accelerations, slros::EnabledWarning(rosMessageType, "accelerations"));
  convertToBusVariablePrimitiveArray(busPtr->Effort, busPtr->Effort_SL_Info, msgPtr->effort, slros::EnabledWarning(rosMessageType, "effort"));
  convertToBusVariablePrimitiveArray(busPtr->Positions, busPtr->Positions_SL_Info, msgPtr->positions, slros::EnabledWarning(rosMessageType, "positions"));
  convertToBus(&busPtr->TimeFromStart, &msgPtr->time_from_start);
  convertToBusVariablePrimitiveArray(busPtr->Velocities, busPtr->Velocities_SL_Info, msgPtr->velocities, slros::EnabledWarning(rosMessageType, "velocities"));
}

