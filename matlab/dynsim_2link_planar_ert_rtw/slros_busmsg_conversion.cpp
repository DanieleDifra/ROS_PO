#include "slros_busmsg_conversion.h"


// Conversions between SL_Bus_dynsim_2link_planar_geometry_msgs_Point and geometry_msgs::Point

void convertFromBus(geometry_msgs::Point* msgPtr, SL_Bus_dynsim_2link_planar_geometry_msgs_Point const* busPtr)
{
  const std::string rosMessageType("geometry_msgs/Point");

  msgPtr->x =  busPtr->X;
  msgPtr->y =  busPtr->Y;
  msgPtr->z =  busPtr->Z;
}

void convertToBus(SL_Bus_dynsim_2link_planar_geometry_msgs_Point* busPtr, geometry_msgs::Point const* msgPtr)
{
  const std::string rosMessageType("geometry_msgs/Point");

  busPtr->X =  msgPtr->x;
  busPtr->Y =  msgPtr->y;
  busPtr->Z =  msgPtr->z;
}


// Conversions between SL_Bus_dynsim_2link_planar_geometry_msgs_Pose and geometry_msgs::Pose

void convertFromBus(geometry_msgs::Pose* msgPtr, SL_Bus_dynsim_2link_planar_geometry_msgs_Pose const* busPtr)
{
  const std::string rosMessageType("geometry_msgs/Pose");

  convertFromBus(&msgPtr->orientation, &busPtr->Orientation);
  convertFromBus(&msgPtr->position, &busPtr->Position);
}

void convertToBus(SL_Bus_dynsim_2link_planar_geometry_msgs_Pose* busPtr, geometry_msgs::Pose const* msgPtr)
{
  const std::string rosMessageType("geometry_msgs/Pose");

  convertToBus(&busPtr->Orientation, &msgPtr->orientation);
  convertToBus(&busPtr->Position, &msgPtr->position);
}


// Conversions between SL_Bus_dynsim_2link_planar_geometry_msgs_PoseStamped and geometry_msgs::PoseStamped

void convertFromBus(geometry_msgs::PoseStamped* msgPtr, SL_Bus_dynsim_2link_planar_geometry_msgs_PoseStamped const* busPtr)
{
  const std::string rosMessageType("geometry_msgs/PoseStamped");

  convertFromBus(&msgPtr->header, &busPtr->Header);
  convertFromBus(&msgPtr->pose, &busPtr->Pose);
}

void convertToBus(SL_Bus_dynsim_2link_planar_geometry_msgs_PoseStamped* busPtr, geometry_msgs::PoseStamped const* msgPtr)
{
  const std::string rosMessageType("geometry_msgs/PoseStamped");

  convertToBus(&busPtr->Header, &msgPtr->header);
  convertToBus(&busPtr->Pose, &msgPtr->pose);
}


// Conversions between SL_Bus_dynsim_2link_planar_geometry_msgs_Quaternion and geometry_msgs::Quaternion

void convertFromBus(geometry_msgs::Quaternion* msgPtr, SL_Bus_dynsim_2link_planar_geometry_msgs_Quaternion const* busPtr)
{
  const std::string rosMessageType("geometry_msgs/Quaternion");

  msgPtr->w =  busPtr->W;
  msgPtr->x =  busPtr->X;
  msgPtr->y =  busPtr->Y;
  msgPtr->z =  busPtr->Z;
}

void convertToBus(SL_Bus_dynsim_2link_planar_geometry_msgs_Quaternion* busPtr, geometry_msgs::Quaternion const* msgPtr)
{
  const std::string rosMessageType("geometry_msgs/Quaternion");

  busPtr->W =  msgPtr->w;
  busPtr->X =  msgPtr->x;
  busPtr->Y =  msgPtr->y;
  busPtr->Z =  msgPtr->z;
}


// Conversions between SL_Bus_dynsim_2link_planar_geometry_msgs_Twist and geometry_msgs::Twist

void convertFromBus(geometry_msgs::Twist* msgPtr, SL_Bus_dynsim_2link_planar_geometry_msgs_Twist const* busPtr)
{
  const std::string rosMessageType("geometry_msgs/Twist");

  convertFromBus(&msgPtr->angular, &busPtr->Angular);
  convertFromBus(&msgPtr->linear, &busPtr->Linear);
}

void convertToBus(SL_Bus_dynsim_2link_planar_geometry_msgs_Twist* busPtr, geometry_msgs::Twist const* msgPtr)
{
  const std::string rosMessageType("geometry_msgs/Twist");

  convertToBus(&busPtr->Angular, &msgPtr->angular);
  convertToBus(&busPtr->Linear, &msgPtr->linear);
}


// Conversions between SL_Bus_dynsim_2link_planar_geometry_msgs_TwistStamped and geometry_msgs::TwistStamped

void convertFromBus(geometry_msgs::TwistStamped* msgPtr, SL_Bus_dynsim_2link_planar_geometry_msgs_TwistStamped const* busPtr)
{
  const std::string rosMessageType("geometry_msgs/TwistStamped");

  convertFromBus(&msgPtr->header, &busPtr->Header);
  convertFromBus(&msgPtr->twist, &busPtr->Twist);
}

void convertToBus(SL_Bus_dynsim_2link_planar_geometry_msgs_TwistStamped* busPtr, geometry_msgs::TwistStamped const* msgPtr)
{
  const std::string rosMessageType("geometry_msgs/TwistStamped");

  convertToBus(&busPtr->Header, &msgPtr->header);
  convertToBus(&busPtr->Twist, &msgPtr->twist);
}


// Conversions between SL_Bus_dynsim_2link_planar_geometry_msgs_Vector3 and geometry_msgs::Vector3

void convertFromBus(geometry_msgs::Vector3* msgPtr, SL_Bus_dynsim_2link_planar_geometry_msgs_Vector3 const* busPtr)
{
  const std::string rosMessageType("geometry_msgs/Vector3");

  msgPtr->x =  busPtr->X;
  msgPtr->y =  busPtr->Y;
  msgPtr->z =  busPtr->Z;
}

void convertToBus(SL_Bus_dynsim_2link_planar_geometry_msgs_Vector3* busPtr, geometry_msgs::Vector3 const* msgPtr)
{
  const std::string rosMessageType("geometry_msgs/Vector3");

  busPtr->X =  msgPtr->x;
  busPtr->Y =  msgPtr->y;
  busPtr->Z =  msgPtr->z;
}


// Conversions between SL_Bus_dynsim_2link_planar_ros_time_Time and ros::Time

void convertFromBus(ros::Time* msgPtr, SL_Bus_dynsim_2link_planar_ros_time_Time const* busPtr)
{
  const std::string rosMessageType("ros_time/Time");

  msgPtr->sec =  busPtr->Sec;
  msgPtr->nsec =  busPtr->Nsec;
}

void convertToBus(SL_Bus_dynsim_2link_planar_ros_time_Time* busPtr, ros::Time const* msgPtr)
{
  const std::string rosMessageType("ros_time/Time");

  busPtr->Sec =  msgPtr->sec;
  busPtr->Nsec =  msgPtr->nsec;
}


// Conversions between SL_Bus_dynsim_2link_planar_rosgraph_msgs_Clock and rosgraph_msgs::Clock

void convertFromBus(rosgraph_msgs::Clock* msgPtr, SL_Bus_dynsim_2link_planar_rosgraph_msgs_Clock const* busPtr)
{
  const std::string rosMessageType("rosgraph_msgs/Clock");

  convertFromBus(&msgPtr->clock, &busPtr->Clock_);
}

void convertToBus(SL_Bus_dynsim_2link_planar_rosgraph_msgs_Clock* busPtr, rosgraph_msgs::Clock const* msgPtr)
{
  const std::string rosMessageType("rosgraph_msgs/Clock");

  convertToBus(&busPtr->Clock_, &msgPtr->clock);
}


// Conversions between SL_Bus_dynsim_2link_planar_sensor_msgs_JointState and sensor_msgs::JointState

void convertFromBus(sensor_msgs::JointState* msgPtr, SL_Bus_dynsim_2link_planar_sensor_msgs_JointState const* busPtr)
{
  const std::string rosMessageType("sensor_msgs/JointState");

  convertFromBusVariablePrimitiveArray(msgPtr->effort, busPtr->Effort, busPtr->Effort_SL_Info);
  convertFromBus(&msgPtr->header, &busPtr->Header);
  convertFromBusVariableStringArray(msgPtr->name, busPtr->Name, busPtr->Name_SL_Info);
  convertFromBusVariablePrimitiveArray(msgPtr->position, busPtr->Position, busPtr->Position_SL_Info);
  convertFromBusVariablePrimitiveArray(msgPtr->velocity, busPtr->Velocity, busPtr->Velocity_SL_Info);
}

void convertToBus(SL_Bus_dynsim_2link_planar_sensor_msgs_JointState* busPtr, sensor_msgs::JointState const* msgPtr)
{
  const std::string rosMessageType("sensor_msgs/JointState");

  convertToBusVariablePrimitiveArray(busPtr->Effort, busPtr->Effort_SL_Info, msgPtr->effort, slros::EnabledWarning(rosMessageType, "effort"));
  convertToBus(&busPtr->Header, &msgPtr->header);
  convertToBusVariableStringArray(busPtr->Name, busPtr->Name_SL_Info, msgPtr->name, slros::EnabledWarning(rosMessageType, "name"));
  convertToBusVariablePrimitiveArray(busPtr->Position, busPtr->Position_SL_Info, msgPtr->position, slros::EnabledWarning(rosMessageType, "position"));
  convertToBusVariablePrimitiveArray(busPtr->Velocity, busPtr->Velocity_SL_Info, msgPtr->velocity, slros::EnabledWarning(rosMessageType, "velocity"));
}


// Conversions between SL_Bus_dynsim_2link_planar_std_msgs_Float64MultiArray and std_msgs::Float64MultiArray

void convertFromBus(std_msgs::Float64MultiArray* msgPtr, SL_Bus_dynsim_2link_planar_std_msgs_Float64MultiArray const* busPtr)
{
  const std::string rosMessageType("std_msgs/Float64MultiArray");

  convertFromBusVariablePrimitiveArray(msgPtr->data, busPtr->Data, busPtr->Data_SL_Info);
  convertFromBus(&msgPtr->layout, &busPtr->Layout);
}

void convertToBus(SL_Bus_dynsim_2link_planar_std_msgs_Float64MultiArray* busPtr, std_msgs::Float64MultiArray const* msgPtr)
{
  const std::string rosMessageType("std_msgs/Float64MultiArray");

  convertToBusVariablePrimitiveArray(busPtr->Data, busPtr->Data_SL_Info, msgPtr->data, slros::EnabledWarning(rosMessageType, "data"));
  convertToBus(&busPtr->Layout, &msgPtr->layout);
}


// Conversions between SL_Bus_dynsim_2link_planar_std_msgs_Header and std_msgs::Header

void convertFromBus(std_msgs::Header* msgPtr, SL_Bus_dynsim_2link_planar_std_msgs_Header const* busPtr)
{
  const std::string rosMessageType("std_msgs/Header");

  convertFromBusVariablePrimitiveArray(msgPtr->frame_id, busPtr->FrameId, busPtr->FrameId_SL_Info);
  msgPtr->seq =  busPtr->Seq;
  convertFromBus(&msgPtr->stamp, &busPtr->Stamp);
}

void convertToBus(SL_Bus_dynsim_2link_planar_std_msgs_Header* busPtr, std_msgs::Header const* msgPtr)
{
  const std::string rosMessageType("std_msgs/Header");

  convertToBusVariablePrimitiveArray(busPtr->FrameId, busPtr->FrameId_SL_Info, msgPtr->frame_id, slros::EnabledWarning(rosMessageType, "frame_id"));
  busPtr->Seq =  msgPtr->seq;
  convertToBus(&busPtr->Stamp, &msgPtr->stamp);
}


// Conversions between SL_Bus_dynsim_2link_planar_std_msgs_MultiArrayDimension and std_msgs::MultiArrayDimension

void convertFromBus(std_msgs::MultiArrayDimension* msgPtr, SL_Bus_dynsim_2link_planar_std_msgs_MultiArrayDimension const* busPtr)
{
  const std::string rosMessageType("std_msgs/MultiArrayDimension");

  convertFromBusVariablePrimitiveArray(msgPtr->label, busPtr->Label, busPtr->Label_SL_Info);
  msgPtr->size =  busPtr->Size;
  msgPtr->stride =  busPtr->Stride;
}

void convertToBus(SL_Bus_dynsim_2link_planar_std_msgs_MultiArrayDimension* busPtr, std_msgs::MultiArrayDimension const* msgPtr)
{
  const std::string rosMessageType("std_msgs/MultiArrayDimension");

  convertToBusVariablePrimitiveArray(busPtr->Label, busPtr->Label_SL_Info, msgPtr->label, slros::EnabledWarning(rosMessageType, "label"));
  busPtr->Size =  msgPtr->size;
  busPtr->Stride =  msgPtr->stride;
}


// Conversions between SL_Bus_dynsim_2link_planar_std_msgs_MultiArrayLayout and std_msgs::MultiArrayLayout

void convertFromBus(std_msgs::MultiArrayLayout* msgPtr, SL_Bus_dynsim_2link_planar_std_msgs_MultiArrayLayout const* busPtr)
{
  const std::string rosMessageType("std_msgs/MultiArrayLayout");

  msgPtr->data_offset =  busPtr->DataOffset;
  convertFromBusVariableNestedArray(msgPtr->dim, busPtr->Dim, busPtr->Dim_SL_Info);
}

void convertToBus(SL_Bus_dynsim_2link_planar_std_msgs_MultiArrayLayout* busPtr, std_msgs::MultiArrayLayout const* msgPtr)
{
  const std::string rosMessageType("std_msgs/MultiArrayLayout");

  busPtr->DataOffset =  msgPtr->data_offset;
  convertToBusVariableNestedArray(busPtr->Dim, busPtr->Dim_SL_Info, msgPtr->dim, slros::EnabledWarning(rosMessageType, "dim"));
}

