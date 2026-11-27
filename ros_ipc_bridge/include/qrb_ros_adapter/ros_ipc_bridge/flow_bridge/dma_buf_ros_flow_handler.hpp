// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_ADAPTER_ROS_IPC_BRIDGE_FLOW_BRIDGE_DMA_BUF_ROS_FLOW_HANDLER_HPP
#define QRB_ROS_ADAPTER_ROS_IPC_BRIDGE_FLOW_BRIDGE_DMA_BUF_ROS_FLOW_HANDLER_HPP

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <lib_mem_dmabuf/dmabuf.hpp>
#include <limits>
#include <memory>
#include <rclcpp/publisher.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/type_adapter.hpp>
#include <string>
#include <utility>

#include "qrb_ros_adapter/ros_ipc_bridge/flow_bridge/fetch_subscription.hpp"
#include "qrb_ros_adapter/ros_ipc_bridge/flow_bridge/ros_conversion.hpp"
#include "qrb_ros_adapter/ros_ipc_bridge/flow_bridge/ros_dma_buffer_pool.hpp"
#include "qrb_ros_adapter/ros_ipc_bridge/flow_bridge/ros_flow_handler.hpp"
#include "qrb_ros_adapter/ros_ipc_bridge/flow_bridge/ros_qos.hpp"

namespace qrb::adapter
{

/**
 * @brief Complete ROS endpoint and ownership implementation for one DmaBuf Contract.
 *
 * The message pair supplies one RosConversion policy. Its two methods bind the
 * message's DMA-BUF through typed payload contexts while this handler owns all
 * transport validation, IPC transfer and copy lifecycle.
 */
template <typename RosMessageT, typename ContractT>
class DmaBufRosFlowHandler final : public TypedRosFlowHandler<ContractT>
{
  static_assert(rclcpp::TypeAdapter<RosMessageT>::is_specialized::value,
      "a DmaBuf Flow must use a qrb_ros_transport custom type with a registered "
      "rclcpp::TypeAdapter");

public:
  explicit DmaBufRosFlowHandler(std::string heapName = "/dev/dma_heap/system")
    : heapName_(std::move(heapName))
  {
  }

  Status Open(rclcpp::Node & node, const FlowConfig & config, std::string &) override
  {
    config_ = config;
    const rclcpp::QoS qos = MakeRosQos(config);
    if (config.direction == Direction::kRosToHcm) {
      subscription_.Open(node, config.rosTopic, qos);
    } else {
      pool_ =
          std::make_unique<RosDmaBufferPool>(RosDmaBufferPoolSize(config.qosProfile), heapName_);
      publisher_ = node.create_publisher<RosMessageT>(config.rosTopic, qos);
    }
    RCLCPP_INFO(node.get_logger(), "ROS endpoint QoS: %s, %s, depth=%u",
        ToString(config.qosProfile.reliability), ToString(config.qosProfile.durability),
        config.qosProfile.depth);
    return Status::kOk;
  }

  void Close() noexcept override
  {
    subscription_.Close();
    publisher_.reset();
    if (pool_ != nullptr)
      pool_->Shutdown();
    pool_.reset();
  }

protected:
  Status FetchTyped(ContractT & target,
      OutboundPayload & outPayload,
      std::string & diagnostic) override
  {
    outPayload = OutboundPayload{};
    if (!subscription_.IsOpen())
      return Status::kBadState;

    const std::shared_ptr<RosMessageT> message = subscription_.Take();
    if (message == nullptr)
      return Status::kNoData;

    RosToContractPayload conversionPayload;
    ConversionResult converted =
        RosConversion<RosMessageT, ContractT>::RosToContract(*message, target, conversionPayload);
    if (!converted) {
      diagnostic += converted.Diagnostic();
      return Status::kConversionError;
    }
    if (!conversionPayload.IsBound()) {
      diagnostic += "DMA-BUF conversion did not bind a source payload; ";
      return Status::kConversionError;
    }
    // SCM_RIGHTS transfers another reference to the same kernel Buffer.
    // RosFlowBridgeBase retains this owner until HCM confirms its copy.
    outPayload.fd = conversionPayload.Buffer()->fd();
    outPayload.keepAlive = conversionPayload.Buffer();
    outPayload.offset = conversionPayload.Offset();
    outPayload.hasPayload = true;
    return Status::kOk;
  }

  Status PublishTyped(const ContractT & source,
      InboundPayload & payload,
      std::string & diagnostic) override
  {
    if (publisher_ == nullptr)
      return Status::kBadState;
    if (!payload.hasPayload || !payload.fd.IsValid()) {
      diagnostic += "DmaBuf contract received no payload fd; ";
      return Status::kProtocolError;
    }

    RosMessageT message;
    if (payload.offset > payload.capacity) {
      diagnostic += "Contract payload exceeds the received DMA-BUF range; ";
      return Status::kConversionError;
    }

    ContractToRosPayload conversionPayload(payload.capacity - payload.offset);
    ConversionResult converted =
        RosConversion<RosMessageT, ContractT>::ContractToRos(source, message, conversionPayload);
    if (!converted) {
      diagnostic += converted.Diagnostic();
      return Status::kConversionError;
    }
    if (!conversionPayload.IsBound() ||
        conversionPayload.Size() > std::numeric_limits<std::size_t>::max()) {
      diagnostic += "DMA-BUF conversion did not bind a valid destination payload; ";
      return Status::kConversionError;
    }

    std::shared_ptr<lib_mem_dmabuf::DmaBuffer> buffer;
    Status copied = CopyToRosBuffer(payload, conversionPayload, buffer, diagnostic);
    if (copied != Status::kOk)
      return copied;

    if (!conversionPayload.Assign(std::move(buffer))) {
      diagnostic += "failed to assign the ROS DMA-BUF destination; ";
      return Status::kInternalError;
    }
    publisher_->publish(std::move(message));
    return Status::kOk;
  }

private:
  /**
   * @brief Copy the received payload into a pool buffer the ROS message owns.
   *
   * Without a region plan the destination mirrors the received range and is
   * copied flat. With one, the destination size is independent of the received
   * range: the allocation is zeroed and only the planned valid rows are copied, so
   * padding never carries a previous frame's data.
   */
  Status CopyToRosBuffer(InboundPayload & payload,
      const ContractToRosPayload & conversionPayload,
      std::shared_ptr<lib_mem_dmabuf::DmaBuffer> & destination,
      std::string & diagnostic)
  {
    const std::size_t size = static_cast<std::size_t>(conversionPayload.Size());
    const bool repack = conversionPayload.RegionCount() != 0;
    if (pool_ == nullptr || size == 0 || payload.offset > payload.capacity ||
        (!repack && size > payload.capacity - payload.offset)) {
      diagnostic += "invalid inbound DMA-BUF size or unavailable ROS buffer pool; ";
      return Status::kBadState;
    }

    destination = pool_->Acquire(size, std::chrono::milliseconds(config_.timeoutMs));
    if (destination == nullptr) {
      diagnostic += "ROS DMA-BUF pool acquisition timed out or allocation failed; ";
      return Status::kResourceError;
    }

    auto source =
        std::make_shared<lib_mem_dmabuf::DmaBuffer>(payload.fd.Release(), payload.capacity);
    if (!source->map()) {
      diagnostic += "failed to map the HCM DMA-BUF; ";
      return Status::kResourceError;
    }
    if (!source->sync_start()) {
      source->unmap();
      diagnostic += "failed to synchronize the HCM DMA-BUF; ";
      return Status::kResourceError;
    }

    auto * sourceBytes = static_cast<std::uint8_t *>(source->addr());
    // set_data() maps the destination itself; the repack path keeps the mapping
    // open across the per-row copies, so it maps and synchronizes explicitly.
    bool copied = false;
    bool destinationMapped = false;
    if (!repack) {
      copied = destination->set_data(sourceBytes + payload.offset, size);
      destinationMapped = true;
    } else if (destination->map()) {
      destinationMapped = true;
      if (destination->sync_start()) {
        auto * destinationBytes = static_cast<std::uint8_t *>(destination->addr());
        std::memset(destinationBytes, 0, size);
        for (std::size_t index = 0; index < conversionPayload.RegionCount(); ++index) {
          const PayloadCopyRegion & region = conversionPayload.RegionAt(index);
          const std::uint8_t * from = sourceBytes + payload.offset + region.sourceOffset;
          std::uint8_t * to = destinationBytes + region.destinationOffset;
          for (std::uint64_t row = 0; row < region.rows; ++row) {
            std::memcpy(to + row * region.destinationStride, from + row * region.sourceStride,
                static_cast<std::size_t>(region.rowBytes));
          }
        }
        copied = destination->sync_end();
      }
    }
    const bool sourceSyncEnded = source->sync_end();
    const bool sourceUnmapped = source->unmap();
    const bool destinationUnmapped = !destinationMapped || destination->unmap();
    if (!copied || !sourceSyncEnded || !sourceUnmapped || !destinationUnmapped) {
      diagnostic += "failed to copy or synchronize the ROS-side DMA-BUF; ";
      destination.reset();
      return Status::kResourceError;
    }

    payload.hasPayload = false;
    return Status::kOk;
  }

  std::string heapName_;
  FlowConfig config_;
  FetchSubscription<RosMessageT> subscription_;
  typename rclcpp::Publisher<RosMessageT>::SharedPtr publisher_;
  std::unique_ptr<RosDmaBufferPool> pool_;
};

}  // namespace qrb::adapter

#endif  // QRB_ROS_ADAPTER_ROS_IPC_BRIDGE_FLOW_BRIDGE_DMA_BUF_ROS_FLOW_HANDLER_HPP
