// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_adapter/ros_ipc_bridge/flow_bridge/imu_ros_converter.hpp"

#include <array>
#include <cstddef>

#include "qrb_ros_adapter/ipc_runtime/contract_validation.hpp"
#include "qrb_ros_adapter/ros_ipc_bridge/flow_bridge/ros_contract_header.hpp"

namespace qrb::adapter
{

namespace
{

void AddCovariance(const std::array<double, 9> & source,
    google::protobuf::RepeatedField<double> & target)
{
  for (double value : source)
    target.Add(value);
}

/** @brief An empty Contract covariance reaches ROS as the all-zero "unknown" matrix. */
void CopyCovariance(const google::protobuf::RepeatedField<double> & source,
    std::array<double, 9> & target)
{
  target.fill(0.0);
  for (int i = 0; i < source.size(); ++i) {
    target[static_cast<std::size_t>(i)] = source.Get(i);
  }
}

}  // namespace

ConversionResult ImuRosConversion::RosToContract(const ImuRosMessage & source,
    ImuRosContract & target)
{
  target.Clear();
  HeaderToContract(source.header, *target.mutable_header());

  auto * orientation = target.mutable_orientation();
  orientation->set_x(source.orientation.x);
  orientation->set_y(source.orientation.y);
  orientation->set_z(source.orientation.z);
  orientation->set_w(source.orientation.w);
  AddCovariance(source.orientation_covariance, *target.mutable_orientation_covariance());

  auto * angular = target.mutable_angular_velocity();
  angular->set_x(source.angular_velocity.x);
  angular->set_y(source.angular_velocity.y);
  angular->set_z(source.angular_velocity.z);
  AddCovariance(source.angular_velocity_covariance, *target.mutable_angular_velocity_covariance());

  auto * linear = target.mutable_linear_acceleration();
  linear->set_x(source.linear_acceleration.x);
  linear->set_y(source.linear_acceleration.y);
  linear->set_z(source.linear_acceleration.z);
  AddCovariance(
      source.linear_acceleration_covariance, *target.mutable_linear_acceleration_covariance());

  return ValidateImu(target);
}

ConversionResult ImuRosConversion::ContractToRos(const ImuRosContract & source,
    ImuRosMessage & target)
{
  ConversionResult validation = ValidateImu(source);
  if (!validation)
    return validation;

  HeaderToRos(source.header(), target.header);
  target.orientation.x = source.orientation().x();
  target.orientation.y = source.orientation().y();
  target.orientation.z = source.orientation().z();
  target.orientation.w = source.orientation().w();
  CopyCovariance(source.orientation_covariance(), target.orientation_covariance);
  target.angular_velocity.x = source.angular_velocity().x();
  target.angular_velocity.y = source.angular_velocity().y();
  target.angular_velocity.z = source.angular_velocity().z();
  CopyCovariance(source.angular_velocity_covariance(), target.angular_velocity_covariance);
  target.linear_acceleration.x = source.linear_acceleration().x();
  target.linear_acceleration.y = source.linear_acceleration().y();
  target.linear_acceleration.z = source.linear_acceleration().z();
  CopyCovariance(source.linear_acceleration_covariance(), target.linear_acceleration_covariance);
  return ConversionResult::Success();
}

}  // namespace qrb::adapter
