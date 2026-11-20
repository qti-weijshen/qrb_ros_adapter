// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_ADAPTER_ROS_IPC_BRIDGE_FLOW_BRIDGE_IMU_ROS_CONVERTER_HPP
#define QRB_ROS_ADAPTER_ROS_IPC_BRIDGE_FLOW_BRIDGE_IMU_ROS_CONVERTER_HPP

#include <sensor_msgs/msg/imu.hpp>

#include "qrb/contract/sensor/v1/imu.pb.h"
#include "qrb_ros_adapter/ros_ipc_bridge/flow_bridge/ros_conversion.hpp"

namespace qrb::adapter
{

using ImuRosMessage = sensor_msgs::msg::Imu;
using ImuRosContract = ::qrb::contract::sensor::v1::Imu;

template <>
struct RosConversion<ImuRosMessage, ImuRosContract>
{
  using RosType = ImuRosMessage;
  using ContractType = ImuRosContract;

  static ConversionResult RosToContract(const RosType & source, ContractType & target);
  static ConversionResult ContractToRos(const ContractType & source, RosType & target);
};

using ImuRosConversion = RosConversion<ImuRosMessage, ImuRosContract>;

}  // namespace qrb::adapter

#endif  // QRB_ROS_ADAPTER_ROS_IPC_BRIDGE_FLOW_BRIDGE_IMU_ROS_CONVERTER_HPP
