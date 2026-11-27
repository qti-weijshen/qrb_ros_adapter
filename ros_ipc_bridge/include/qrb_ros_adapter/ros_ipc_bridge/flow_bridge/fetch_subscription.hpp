// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_ADAPTER_ROS_IPC_BRIDGE_FLOW_BRIDGE_FETCH_SUBSCRIPTION_HPP
#define QRB_ROS_ADAPTER_ROS_IPC_BRIDGE_FLOW_BRIDGE_FETCH_SUBSCRIPTION_HPP

#include <chrono>
#include <memory>
#include <rclcpp/callback_group.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/subscription_options.hpp>
#include <string>
#include <utility>

namespace qrb::adapter
{

/**
 * @brief A Subscription whose messages wait in its QoS history until the HCM IPC Adapter
 * fetches one.
 *
 * The Subscription has a callback group of its own that the node's executor does not
 * spin. Take() runs its callback on a Flow-private executor, one message per call, oldest
 * first. The KEEP_LAST depth bounds how many messages wait: in the intra-process buffer
 * (the qrb_ros_transport zero-copy path) or in the middleware queue.
 *
 * Open() and Close() run on the node's thread and Take() on the Flow's service thread,
 * never at the same time.
 */
template <typename RosMessageT>
class FetchSubscription
{
public:
  void Open(rclcpp::Node & node, const std::string & topic, const rclcpp::QoS & qos)
  {
    group_ = node.create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    rclcpp::SubscriptionOptions options;
    options.callback_group = group_;
    subscription_ = node.create_subscription<RosMessageT>(
        topic, qos, [this](std::shared_ptr<RosMessageT> message) { taken_ = std::move(message); },
        options);
    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_callback_group(group_, node.get_node_base_interface());
  }

  bool IsOpen() const { return subscription_ != nullptr; }

  /**
   * @brief Take the oldest waiting message without waiting for one.
   *
   * @return nullptr if no message is waiting.
   */
  std::shared_ptr<RosMessageT> Take()
  {
    taken_.reset();
    // A spin may run the executor's own notification, or drop the middleware copy of an
    // intra-process message, instead of handing out a message.
    for (int spin = 0; spin < kMaxSpins && taken_ == nullptr; ++spin)
      executor_->spin_once(std::chrono::nanoseconds(0));
    return std::move(taken_);
  }

  /** @brief Drop the Subscription and every message waiting in it. */
  void Close() noexcept
  {
    executor_.reset();
    subscription_.reset();
    group_.reset();
    taken_.reset();
  }

private:
  static constexpr int kMaxSpins = 4;

  rclcpp::CallbackGroup::SharedPtr group_;
  typename rclcpp::Subscription<RosMessageT>::SharedPtr subscription_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::shared_ptr<RosMessageT> taken_;
};

}  // namespace qrb::adapter

#endif  // QRB_ROS_ADAPTER_ROS_IPC_BRIDGE_FLOW_BRIDGE_FETCH_SUBSCRIPTION_HPP
