// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_ADAPTER_ROS_IPC_BRIDGE_FLOW_BRIDGE_ROS_FLOW_BRIDGE_BASE_HPP
#define QRB_ROS_ADAPTER_ROS_IPC_BRIDGE_FLOW_BRIDGE_ROS_FLOW_BRIDGE_BASE_HPP

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <rclcpp/node.hpp>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include "qrb_ros_adapter/ipc_runtime/flow_config.hpp"
#include "qrb_ros_adapter/ipc_runtime/flow_registration.hpp"
#include "qrb_ros_adapter/ipc_runtime/flow_types.hpp"
#include "qrb_ros_adapter/ipc_runtime/ipc_session.hpp"
#include "qrb_ros_adapter/ipc_runtime/owned_fd.hpp"
#include "qrb_ros_adapter/ipc_runtime/status.hpp"
#include "qrb_ros_adapter/ros_ipc_bridge/flow_bridge/flow_payload.hpp"
#include "qrb_ros_adapter/ros_ipc_bridge/flow_bridge/ros_flow_handler.hpp"

namespace google::protobuf
{
class MessageLite;
}

namespace qrb::adapter
{

/** Terminal data-session event delivered to the Manager for asynchronous cleanup. */
struct FlowTerminalEvent
{
  std::string flowId;
  std::string instanceToken;
  Status reason{Status::kOk};
  std::string diagnostic;
  /** The HCM side closed the data Session: DeInitialize, or its process exited. */
  bool peerClosed{false};
};

using FlowTerminalCallback = std::function<void(FlowTerminalEvent)>;

/**
 * @brief Common node implementation for every generated typed ROS Flow.
 *
 * The base owns:
 *   - the common ROS parameters and their validation
 *   - the Contract Descriptor cross-check
 *   - the IPC data Server lifecycle, handshake and control protocol
 *   - the frame service loop and ROS_TO_HCM payload ownership bookkeeping
 *   - lifecycle state and logging
 *
 * A generated typed RosFlowHandler owned by the base owns the Publisher or
 * Subscription and the ROS/Contract conversion. The Contract mapping registry
 * selects the handler type; the service thread never calls developer-defined
 * virtual functions on the node.
 *
 * The Manager creates each Flow Bridge after validating a REGISTER_FLOW request.
 * The Flow Bridge binds its data socket during construction and then a dedicated
 * service thread accepts the single HCM client, completes the handshake and
 * serves frames. Publisher::publish() is safe to call from that thread.
 *
 * On ros_to_hcm, messages wait in the Subscription's QoS history; each
 * FETCH_REQUEST makes the service thread take the oldest one (see
 * fetch_subscription.hpp), not the shared executor.
 *
 * A configuration or Descriptor failure makes construction throw.
 */
class RosFlowBridgeBase : public rclcpp::Node
{
public:
  RosFlowBridgeBase(const std::string & nodeName,
      const rclcpp::NodeOptions & options,
      std::string_view contractTypeName,
      std::unique_ptr<RosFlowHandler> handler,
      FlowTerminalCallback terminalCallback = {});
  ~RosFlowBridgeBase() override;

private:
  Status Initialize(const ContractDescriptor & descriptor);

  /** Stop the service thread before closing and destroying the owned handler. */
  void Close() noexcept;

  /** @brief Read the Flow configuration and Manager-issued registration parameters. */
  Status DeclareAndReadParameters(FlowConfig & config, FlowRegistration & registration);

  /** @brief Handshake, then serve frames until stopped. */
  void ServiceLoop();

  /** @brief Handle one received frame. */
  Status HandleFrame(const TransportHeader & header,
      const std::vector<std::uint8_t> & payload,
      std::vector<OwnedFd> & fds);

  Status HandleFetchRequest(std::uint64_t requestId);
  Status HandlePublishRequest(const TransportHeader & header,
      const std::vector<std::uint8_t> & payload,
      std::vector<OwnedFd> & fds);

  /** @brief Record a terminal failure, stop serving and notify the Manager.
   * `peerClosed` marks the HCM side ending the Session, logged as INFO. */
  void Fail(Status reason, const std::string & diagnostic, bool peerClosed = false);

  /** @brief Queue one terminal event without performing cleanup on this thread. */
  void NotifyTerminal(Status reason, const std::string & diagnostic, bool peerClosed);

  FlowConfig config_;
  FlowRegistration registration_;
  const ContractDescriptor * descriptor_{nullptr};
  IpcSession session_;
  std::unique_ptr<google::protobuf::MessageLite> contractMessage_;
  std::string diagnostic_;
  std::atomic<Status> terminalReason_{Status::kOk};
  std::atomic<bool> terminalNotified_{false};
  FlowTerminalCallback terminalCallback_;
  std::unique_ptr<RosFlowHandler> handler_;

  std::thread serviceThread_;
  std::atomic<bool> serviceRunning_{false};

  /** Guards Close() against running its teardown twice. */
  bool stopped_{false};

  std::uint64_t framesIn_{0};
  std::uint64_t framesOut_{0};
  std::uint64_t noDataResponses_{0};
  std::uint64_t conversionErrors_{0};
};

}  // namespace qrb::adapter

#endif  // QRB_ROS_ADAPTER_ROS_IPC_BRIDGE_FLOW_BRIDGE_ROS_FLOW_BRIDGE_BASE_HPP
