// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_ADAPTER_HCM_IPC_ADAPTER_IPC_ADAPTER_NODE_HCM_IPC_ADAPTER_NODE_HPP
#define QRB_ROS_ADAPTER_HCM_IPC_ADAPTER_IPC_ADAPTER_NODE_HCM_IPC_ADAPTER_NODE_HPP

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

#include "QC/Node/Ifs/QCNodeConfigIfs.hpp"
#include "QC/Node/Ifs/QCNodeIfs.hpp"
#include "QC/Node/Ifs/QCNodeMonitoringIfs.hpp"
#include "qrb_ros_adapter/hcm_ipc_adapter/flow_client/hcm_flow_client.hpp"
#include "qrb_ros_adapter/hcm_ipc_adapter/ipc_adapter_node/hcm_ipc_adapter_config.hpp"

/** QCNodeFactory node type of the IPC Adapter Node, which is built into libQCNode.so. */
#define QC_NODE_TYPE_IPC_ADAPTER QC::QC_NODE_TYPE_CUSTOM_3

namespace qrb::adapter
{

struct HcmIpcAdapterNodeConfigData : public QC::QCNodeConfigBase_t
{
  HcmIpcAdapterConfig adapter;
};

class HcmIpcAdapterNodeConfig final : public QC::QCNodeConfigIfs
{
public:
  QC::QCStatus_e VerifyAndSet(const std::string config, std::string & errors) override;
  // QCNode SDKs declare GetOptions() either as a reference getter or with a
  // status return; both forms are provided so the node builds against either.
  const std::string & GetOptions();
  QC::QCStatus_e GetOptions(std::string & options);
  const QC::QCNodeConfigBase_t & Get() override;

  const HcmIpcAdapterConfig & Adapter() const { return config_.adapter; }

  /** @brief Forget the configuration, e.g. one whose Initialize() failed. */
  void Clear();

private:
  HcmIpcAdapterNodeConfigData config_{};
  std::string options_;
};

struct HcmIpcAdapterMonitoringData : public QC::QCNodeMonitoringBase_t
{
  /** 1: the data Session with the ROS Bridge is open; 0 before it is set up and after a fatal
   * error. */
  std::uint32_t sessionConnected{0};
  std::uint64_t frameCount{0};
  /** Calls that failed: dropped frames (frame errors), then every call after a fatal error. */
  std::uint64_t errorCount{0};
  /** Latency values cover only calls that moved a sample; no-data and failed calls are excluded. */
  std::uint64_t lastLatencyNs{0};
  std::uint64_t averageLatencyNs{0};
  std::uint64_t minLatencyNs{0};
  std::uint64_t maxLatencyNs{0};
  /** ROS_TO_HCM calls that found no fresh sample; counted in frameCount, not errorCount. */
  std::uint64_t noDataCount{0};
};

class HcmIpcAdapterNodeMonitoring final : public QC::QCNodeMonitoringIfs
{
public:
  HcmIpcAdapterNodeMonitoring();

  QC::QCStatus_e VerifyAndSet(const std::string config, std::string & errors) override;
  // QCNode SDKs declare GetOptions() either as a reference getter or with a
  // status return; both forms are provided so the node builds against either.
  const std::string & GetOptions();
  QC::QCStatus_e GetOptions(std::string & options);
  const QC::QCNodeMonitoringBase_t & Get() override;
  std::uint32_t GetMaximalSize() override;
  std::uint32_t GetCurrentSize() override;
  QC::QCStatus_e Place(void * data, std::uint32_t & size) override;

  void RecordProcess(Status status, std::uint64_t latencyNs);
  void SetSessionConnected(bool connected);
  void Reset();

private:
  std::mutex mutex_;
  HcmIpcAdapterMonitoringData data_{};
  std::uint64_t latencySamples_{0};
  std::uint64_t totalLatencyNs_{0};
  std::string options_;
};

/**
 * @brief Synchronous QCNode implementation for one HCM IPC Flow.
 *
 * The host owns scheduling, threads, frame descriptors, and buffer lifetime.
 * This node implements the standard QCNodeIfs extension point directly. It
 * borrows local frame buffers only for the duration of ProcessFrameDescriptor().
 *
 * A frame error (IsFrameError(), e.g. a sample larger than the host's buffer)
 * fails only that call and counts in errorCount; the Flow goes on. A fatal
 * Flow status closes the Session without changing the QCNode state; every
 * later ProcessFrameDescriptor() returns that status until the host calls
 * Stop() and Start(). Start() then registers the Flow again; if that fails,
 * the node stays READY and the host may call Start() again.
 */
class HcmIpcAdapterNode final : public QC::QCNodeIfs
{
public:
  /** The bootstrap socket path is fixed in production; another path reaches a private Manager. */
  explicit HcmIpcAdapterNode(std::string bootstrapSocketPath = kDefaultBootstrapSocketPath);
  ~HcmIpcAdapterNode();

  QC::QCStatus_e Initialize(QC::QCNodeInit_t & config) override;
  QC::QCStatus_e DeInitialize() override;
  QC::QCStatus_e Start() override;
  QC::QCStatus_e Stop() override;
  QC::QCStatus_e ProcessFrameDescriptor(QC::QCFrameDescriptorNodeIfs & frameDesc) override;
  QC::QCObjectState_e GetState() override;
  QC::QCNodeConfigIfs & GetConfigurationIfs() override;
  QC::QCNodeMonitoringIfs & GetMonitoringIfs() override;

private:
  Status Process(QC::QCFrameDescriptorNodeIfs & frameDesc);
  void FinishOperation(Status status);

  const std::string bootstrapSocketPath_;
  std::mutex mutex_;
  std::atomic<QC::QCObjectState_e> state_{QC::QC_OBJECT_STATE_INITIAL};
  HcmIpcAdapterNodeConfig config_;
  HcmIpcAdapterNodeMonitoring monitoring_;
  HcmFlowClient client_;
  Status flowFailure_{Status::kOk};  // first fatal status since the Session was set up
};

}  // namespace qrb::adapter

#endif  // QRB_ROS_ADAPTER_HCM_IPC_ADAPTER_IPC_ADAPTER_NODE_HCM_IPC_ADAPTER_NODE_HPP
