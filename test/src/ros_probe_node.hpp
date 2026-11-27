// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_ADAPTER_TEST_ROS_PROBE_NODE_HPP
#define QRB_ROS_ADAPTER_TEST_ROS_PROBE_NODE_HPP

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <thread>

#include "probe.hpp"

namespace qrb::adapter::stream_probe
{

class ProbeEndpoint;
class ProbeRecorder;

/**
 * @brief The ROS side of a stream probe: publishes probed frames to the ROS
 * IPC Bridge (direction 1, ROS to HCM) or subscribes to the frames it
 * publishes and checks them (direction 0, HCM to ROS).
 *
 * A component: load it into the Bridge's `qrb_ros_adapter_container` to share the
 * Bridge's process and intra-process (zero-copy) path, into any other
 * component container, or run it alone with `ros_stream_probe_node` to go
 * through DDS. Parameters are read once at construction (see probe.hpp) and
 * are then read-only; the Flow comes from the Adapter's static config
 * `adapter_config` names.
 *
 * The probe stops at `duration`; unloading stops it as well. It writes a
 * summary to the ROS log and, with `log_file`, a text log of every frame and a
 * monitor line every 5 seconds.
 */
class RosStreamProbeNode : public rclcpp::Node
{
public:
  explicit RosStreamProbeNode(const rclcpp::NodeOptions & options);
  ~RosStreamProbeNode() override;

  RosStreamProbeNode(const RosStreamProbeNode &) = delete;
  RosStreamProbeNode & operator=(const RosStreamProbeNode &) = delete;

  /** @brief Stop sending or counting and write the summary; later calls do nothing. */
  void Finish(const std::string & reason);

  /** @brief Whether the probe has stopped. */
  bool Done() const;

  /** @brief 0, or 1 if a frame failed verification or belonged to another stream. */
  int ExitCode() const;

private:
  /** @throws std::invalid_argument on bad parameters */
  void LoadParams();
  void DeclareEffectiveParams();
  void SendLoop();
  void Tick();

  ProbeParams params_;
  ProbeFlow flow_;
  bool sender_{false};
  std::unique_ptr<ProbeRecorder> recorder_;
  std::unique_ptr<ProbeEndpoint> endpoint_;
  rclcpp::TimerBase::SharedPtr tickTimer_;

  std::mutex wakeMutex_;
  std::condition_variable wake_;
  std::atomic<bool> stopping_{false};
  std::thread senderThread_;
};

}  // namespace qrb::adapter::stream_probe

#endif  // QRB_ROS_ADAPTER_TEST_ROS_PROBE_NODE_HPP
