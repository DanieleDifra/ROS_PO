#include "slros_busmsg_conversion.h"


// Conversions between SL_Bus_controller_cartesian_ros_time_Time and ros::Time

void convertFromBus(ros::Time* msgPtr, SL_Bus_controller_cartesian_ros_time_Time const* busPtr)
{
  const std::string rosMessageType("ros_time/Time");

  msgPtr->sec =  busPtr->Sec;
  msgPtr->nsec =  busPtr->Nsec;
}

void convertToBus(SL_Bus_controller_cartesian_ros_time_Time* busPtr, ros::Time const* msgPtr)
{
  const std::string rosMessageType("ros_time/Time");

  busPtr->Sec =  msgPtr->sec;
  busPtr->Nsec =  msgPtr->nsec;
}


// Conversions between SL_Bus_controller_cartesian_rosgraph_msgs_Clock and rosgraph_msgs::Clock

void convertFromBus(rosgraph_msgs::Clock* msgPtr, SL_Bus_controller_cartesian_rosgraph_msgs_Clock const* busPtr)
{
  const std::string rosMessageType("rosgraph_msgs/Clock");

  convertFromBus(&msgPtr->clock, &busPtr->Clock_);
}

void convertToBus(SL_Bus_controller_cartesian_rosgraph_msgs_Clock* busPtr, rosgraph_msgs::Clock const* msgPtr)
{
  const std::string rosMessageType("rosgraph_msgs/Clock");

  convertToBus(&busPtr->Clock_, &msgPtr->clock);
}


// Conversions between SL_Bus_controller_cartesian_sensor_msgs_JointState and sensor_msgs::JointState

void convertFromBus(sensor_msgs::JointState* msgPtr, SL_Bus_controller_cartesian_sensor_msgs_JointState const* busPtr)
{
  const std::string rosMessageType("sensor_msgs/JointState");

  convertFromBusVariablePrimitiveArray(msgPtr->effort, busPtr->Effort, busPtr->Effort_SL_Info);
  convertFromBus(&msgPtr->header, &busPtr->Header);
  convertFromBusVariableStringArray(msgPtr->name, busPtr->Name, busPtr->Name_SL_Info);
  convertFromBusVariablePrimitiveArray(msgPtr->position, busPtr->Position, busPtr->Position_SL_Info);
  convertFromBusVariablePrimitiveArray(msgPtr->velocity, busPtr->Velocity, busPtr->Velocity_SL_Info);
}

void convertToBus(SL_Bus_controller_cartesian_sensor_msgs_JointState* busPtr, sensor_msgs::JointState const* msgPtr)
{
  const std::string rosMessageType("sensor_msgs/JointState");

  convertToBusVariablePrimitiveArray(busPtr->Effort, busPtr->Effort_SL_Info, msgPtr->effort, slros::EnabledWarning(rosMessageType, "effort"));
  convertToBus(&busPtr->Header, &msgPtr->header);
  convertToBusVariableStringArray(busPtr->Name, busPtr->Name_SL_Info, msgPtr->name, slros::EnabledWarning(rosMessageType, "name"));
  convertToBusVariablePrimitiveArray(busPtr->Position, busPtr->Position_SL_Info, msgPtr->position, slros::EnabledWarning(rosMessageType, "position"));
  convertToBusVariablePrimitiveArray(busPtr->Velocity, busPtr->Velocity_SL_Info, msgPtr->velocity, slros::EnabledWarning(rosMessageType, "velocity"));
}


// Conversions between SL_Bus_controller_cartesian_std_msgs_Float32MultiArray and std_msgs::Float32MultiArray

void convertFromBus(std_msgs::Float32MultiArray* msgPtr, SL_Bus_controller_cartesian_std_msgs_Float32MultiArray const* busPtr)
{
  const std::string rosMessageType("std_msgs/Float32MultiArray");

  convertFromBusVariablePrimitiveArray(msgPtr->data, busPtr->Data, busPtr->Data_SL_Info);
  convertFromBus(&msgPtr->layout, &busPtr->Layout);
}

void convertToBus(SL_Bus_controller_cartesian_std_msgs_Float32MultiArray* busPtr, std_msgs::Float32MultiArray const* msgPtr)
{
  const std::string rosMessageType("std_msgs/Float32MultiArray");

  convertToBusVariablePrimitiveArray(busPtr->Data, busPtr->Data_SL_Info, msgPtr->data, slros::EnabledWarning(rosMessageType, "data"));
  convertToBus(&busPtr->Layout, &msgPtr->layout);
}


// Conversions between SL_Bus_controller_cartesian_std_msgs_Float64MultiArray and std_msgs::Float64MultiArray

void convertFromBus(std_msgs::Float64MultiArray* msgPtr, SL_Bus_controller_cartesian_std_msgs_Float64MultiArray const* busPtr)
{
  const std::string rosMessageType("std_msgs/Float64MultiArray");

  convertFromBusVariablePrimitiveArray(msgPtr->data, busPtr->Data, busPtr->Data_SL_Info);
  convertFromBus(&msgPtr->layout, &busPtr->Layout);
}

void convertToBus(SL_Bus_controller_cartesian_std_msgs_Float64MultiArray* busPtr, std_msgs::Float64MultiArray const* msgPtr)
{
  const std::string rosMessageType("std_msgs/Float64MultiArray");

  convertToBusVariablePrimitiveArray(busPtr->Data, busPtr->Data_SL_Info, msgPtr->data, slros::EnabledWarning(rosMessageType, "data"));
  convertToBus(&busPtr->Layout, &msgPtr->layout);
}


// Conversions between SL_Bus_controller_cartesian_std_msgs_Header and std_msgs::Header

void convertFromBus(std_msgs::Header* msgPtr, SL_Bus_controller_cartesian_std_msgs_Header const* busPtr)
{
  const std::string rosMessageType("std_msgs/Header");

  convertFromBusVariablePrimitiveArray(msgPtr->frame_id, busPtr->FrameId, busPtr->FrameId_SL_Info);
  msgPtr->seq =  busPtr->Seq;
  convertFromBus(&msgPtr->stamp, &busPtr->Stamp);
}

void convertToBus(SL_Bus_controller_cartesian_std_msgs_Header* busPtr, std_msgs::Header const* msgPtr)
{
  const std::string rosMessageType("std_msgs/Header");

  convertToBusVariablePrimitiveArray(busPtr->FrameId, busPtr->FrameId_SL_Info, msgPtr->frame_id, slros::EnabledWarning(rosMessageType, "frame_id"));
  busPtr->Seq =  msgPtr->seq;
  convertToBus(&busPtr->Stamp, &msgPtr->stamp);
}


// Conversions between SL_Bus_controller_cartesian_std_msgs_MultiArrayDimension and std_msgs::MultiArrayDimension

void convertFromBus(std_msgs::MultiArrayDimension* msgPtr, SL_Bus_controller_cartesian_std_msgs_MultiArrayDimension const* busPtr)
{
  const std::string rosMessageType("std_msgs/MultiArrayDimension");

  convertFromBusVariablePrimitiveArray(msgPtr->label, busPtr->Label, busPtr->Label_SL_Info);
  msgPtr->size =  busPtr->Size;
  msgPtr->stride =  busPtr->Stride;
}

void convertToBus(SL_Bus_controller_cartesian_std_msgs_MultiArrayDimension* busPtr, std_msgs::MultiArrayDimension const* msgPtr)
{
  const std::string rosMessageType("std_msgs/MultiArrayDimension");

  convertToBusVariablePrimitiveArray(busPtr->Label, busPtr->Label_SL_Info, msgPtr->label, slros::EnabledWarning(rosMessageType, "label"));
  busPtr->Size =  msgPtr->size;
  busPtr->Stride =  msgPtr->stride;
}


// Conversions between SL_Bus_controller_cartesian_std_msgs_MultiArrayLayout and std_msgs::MultiArrayLayout

void convertFromBus(std_msgs::MultiArrayLayout* msgPtr, SL_Bus_controller_cartesian_std_msgs_MultiArrayLayout const* busPtr)
{
  const std::string rosMessageType("std_msgs/MultiArrayLayout");

  msgPtr->data_offset =  busPtr->DataOffset;
  convertFromBusVariableNestedArray(msgPtr->dim, busPtr->Dim, busPtr->Dim_SL_Info);
}

void convertToBus(SL_Bus_controller_cartesian_std_msgs_MultiArrayLayout* busPtr, std_msgs::MultiArrayLayout const* msgPtr)
{
  const std::string rosMessageType("std_msgs/MultiArrayLayout");

  busPtr->DataOffset =  msgPtr->data_offset;
  convertToBusVariableNestedArray(busPtr->Dim, busPtr->Dim_SL_Info, msgPtr->dim, slros::EnabledWarning(rosMessageType, "dim"));
}

