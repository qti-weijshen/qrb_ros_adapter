// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_adapter/hcm_ipc_adapter/flow_client/imu_hcm_converter.hpp"

#include <cstddef>

#include "qrb_ros_adapter/ipc_runtime/contract_time.hpp"
#include "qrb_ros_adapter/ipc_runtime/contract_validation.hpp"

namespace qrb::adapter
{

namespace
{

/** @brief An empty Contract covariance is the ROS "unknown" all-zero matrix. */
void CopyCovariance(const google::protobuf::RepeatedField<double> & source, double (&target)[9])
{
  for (double & value : target) {
    value = 0.0;
  }
  for (int i = 0; i < source.size(); ++i) {
    target[static_cast<std::size_t>(i)] = source.Get(i);
  }
}

void AddCovariance(const double (&source)[9], google::protobuf::RepeatedField<double> & target)
{
  for (double value : source) {
    target.Add(value);
  }
}

}  // namespace

template <>
ConversionResult ConvertContractToHcm<ImuHcmContract, ImuHcmDescriptor>(const ImuHcmContract & imu,
    const DmaBufView & /*payload*/,
    ImuHcmDescriptor & descriptor)
{
  ConversionResult validation = ValidateImu(imu);
  if (!validation)
    return validation;
  if (imu.header().frame_id().size() > QRB::Memory::QRB_FRAME_ID_MAX_LEN)
    return ConversionResult::Failure("frame_id is longer than the HCM Imu descriptor holds");

  std::uint64_t timestampNs = 0;
  ConversionResult time = ContractTimeToNanos(imu.header().stamp(), timestampNs);
  if (!time)
    return time;
  if (!AssignHcmString(imu.header().frame_id(), descriptor.frameId))
    return ConversionResult::Failure("frame_id does not fit the HCM Imu descriptor");
  descriptor.timestampNs = timestampNs;

  descriptor.orientation.x = imu.orientation().x();
  descriptor.orientation.y = imu.orientation().y();
  descriptor.orientation.z = imu.orientation().z();
  descriptor.orientation.w = imu.orientation().w();
  CopyCovariance(imu.orientation_covariance(), descriptor.orientation_covariance);
  descriptor.angular_velocity.x = imu.angular_velocity().x();
  descriptor.angular_velocity.y = imu.angular_velocity().y();
  descriptor.angular_velocity.z = imu.angular_velocity().z();
  CopyCovariance(imu.angular_velocity_covariance(), descriptor.angular_velocity_covariance);
  descriptor.linear_acceleration.x = imu.linear_acceleration().x();
  descriptor.linear_acceleration.y = imu.linear_acceleration().y();
  descriptor.linear_acceleration.z = imu.linear_acceleration().z();
  CopyCovariance(imu.linear_acceleration_covariance(), descriptor.linear_acceleration_covariance);
  return ConversionResult::Success();
}

template <>
ConversionResult ConvertHcmToContract<ImuHcmContract, ImuHcmDescriptor>(
    const ImuHcmDescriptor & descriptor,
    ImuHcmContract & imu)
{
  imu.Clear();
  ConversionResult time =
      NanosToContractTime(descriptor.timestampNs, *imu.mutable_header()->mutable_stamp());
  if (!time)
    return time;
  imu.mutable_header()->set_frame_id(HcmStringToContract(descriptor.frameId));

  imu.mutable_orientation()->set_x(descriptor.orientation.x);
  imu.mutable_orientation()->set_y(descriptor.orientation.y);
  imu.mutable_orientation()->set_z(descriptor.orientation.z);
  imu.mutable_orientation()->set_w(descriptor.orientation.w);
  AddCovariance(descriptor.orientation_covariance, *imu.mutable_orientation_covariance());
  imu.mutable_angular_velocity()->set_x(descriptor.angular_velocity.x);
  imu.mutable_angular_velocity()->set_y(descriptor.angular_velocity.y);
  imu.mutable_angular_velocity()->set_z(descriptor.angular_velocity.z);
  AddCovariance(descriptor.angular_velocity_covariance, *imu.mutable_angular_velocity_covariance());
  imu.mutable_linear_acceleration()->set_x(descriptor.linear_acceleration.x);
  imu.mutable_linear_acceleration()->set_y(descriptor.linear_acceleration.y);
  imu.mutable_linear_acceleration()->set_z(descriptor.linear_acceleration.z);
  AddCovariance(
      descriptor.linear_acceleration_covariance, *imu.mutable_linear_acceleration_covariance());
  return ValidateImu(imu);
}

}  // namespace qrb::adapter
