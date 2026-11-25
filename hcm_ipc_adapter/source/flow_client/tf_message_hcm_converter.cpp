// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_adapter/hcm_ipc_adapter/flow_client/tf_message_hcm_converter.hpp"

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include "qrb_ros_adapter/hcm_ipc_adapter/flow_client/hcm_dma_buffer.hpp"
#include "qrb_ros_adapter/ipc_runtime/contract_time.hpp"
#include "qrb_ros_adapter/ipc_runtime/contract_validation.hpp"

namespace qrb::adapter
{

namespace
{

/**
 * @brief How many transforms fit the descriptor's buffer from its offset on. The flow client
 * syncs the buffer around the conversion. InspectHcmBuffer() reports an offset past the size
 * as kUnusable, so the subtraction cannot wrap.
 */
bool BufferCapacity(const TFMessageHcmDescriptor & descriptor, std::size_t & capacity)
{
  int fd = -1;
  const HcmBufferAccess access = InspectHcmBuffer(descriptor, fd);
  if (access != HcmBufferAccess::kLocalDmaBuf && access != HcmBufferAccess::kHostMemory)
    return false;
  capacity = (descriptor.size - descriptor.offset) / sizeof(QRB::Memory::TransformStamped_t);
  return true;
}

}  // namespace

template <>
ConversionResult ConvertContractToHcm<TFMessageHcmContract, TFMessageHcmDescriptor>(
    const TFMessageHcmContract & message,
    const DmaBufView & /*payload*/,
    TFMessageHcmDescriptor & descriptor)
{
  // Everything that can fail is checked before the buffer is written, so a
  // rejected message leaves the host's descriptor and buffer untouched.
  ConversionResult validation = ValidateTFMessage(message);
  if (!validation)
    return validation;

  const std::size_t count = static_cast<std::size_t>(message.transforms_size());
  std::size_t capacity = 0;
  if (count > 0 && (!BufferCapacity(descriptor, capacity) || count > capacity)) {
    return ConversionResult::Failure(
        "HCM TFMessage descriptor buffer is missing or holds fewer "
        "than " +
        std::to_string(count) + " transforms");
  }
  for (std::size_t i = 0; i < count; ++i) {
    const auto & transform = message.transforms(static_cast<int>(i));
    const std::string where = "transforms[" + std::to_string(i) + "]: ";
    if (transform.header().frame_id().size() > QRB::Memory::QRB_FRAME_ID_MAX_LEN)
      return ConversionResult::Failure(
          where + "frame_id is longer than the HCM TFMessage descriptor holds");
    if (transform.child_frame_id().size() > QRB::Memory::QRB_FRAME_ID_MAX_LEN)
      return ConversionResult::Failure(
          where + "child_frame_id is longer than the HCM TFMessage descriptor holds");
    std::uint64_t timestampNs = 0;
    ConversionResult time = ContractTimeToNanos(transform.header().stamp(), timestampNs);
    if (!time)
      return ConversionResult::Failure(where + time.Diagnostic());
  }

  // memcpy, since the host's offset need not align the array for direct access.
  auto * bytes = static_cast<std::uint8_t *>(count > 0 ? descriptor.GetDataPtr() : nullptr);
  for (std::size_t i = 0; i < count; ++i) {
    const auto & source = message.transforms(static_cast<int>(i));
    QRB::Memory::TransformStamped_t out{};
    (void)ContractTimeToNanos(source.header().stamp(), out.timestampNs);  // checked above
    (void)AssignHcmString(source.header().frame_id(), out.frameId);       // fits, checked above
    (void)AssignHcmString(source.child_frame_id(), out.childFrameId);     // fits, checked above
    TransformToHcm(source.transform(), out.transform);
    std::memcpy(bytes + i * sizeof(out), &out, sizeof(out));
  }
  descriptor.transformCount = static_cast<std::uint32_t>(count);
  descriptor.validSize = TFMessageHcmDescriptor::CalcBufferSize(descriptor.transformCount);
  return ConversionResult::Success();
}

template <>
ConversionResult ConvertHcmToContract<TFMessageHcmContract, TFMessageHcmDescriptor>(
    const TFMessageHcmDescriptor & descriptor,
    TFMessageHcmContract & message)
{
  message.Clear();
  const std::size_t count = descriptor.transformCount;
  std::size_t capacity = 0;
  if (count > 0 && (!BufferCapacity(descriptor, capacity) || count > capacity)) {
    return ConversionResult::Failure(
        "HCM TFMessage transformCount exceeds the descriptor's buffer");
  }

  std::vector<QRB::Memory::TransformStamped_t> transforms(count);
  if (count > 0)
    std::memcpy(transforms.data(), descriptor.GetDataPtr(),
        count * sizeof(QRB::Memory::TransformStamped_t));

  message.mutable_transforms()->Reserve(static_cast<int>(count));
  for (std::size_t i = 0; i < count; ++i) {
    const QRB::Memory::TransformStamped_t & source = transforms[i];
    auto * out = message.add_transforms();
    ConversionResult time =
        NanosToContractTime(source.timestampNs, *out->mutable_header()->mutable_stamp());
    if (!time)
      return ConversionResult::Failure(
          "transforms[" + std::to_string(i) + "]: " + time.Diagnostic());
    out->mutable_header()->set_frame_id(HcmStringToContract(source.frameId));
    out->set_child_frame_id(HcmStringToContract(source.childFrameId));
    TransformToContract(source.transform, *out->mutable_transform());
  }

  // A non-finite value produced on the HCM side must not be published to ROS.
  return ValidateTFMessage(message);
}

}  // namespace qrb::adapter
