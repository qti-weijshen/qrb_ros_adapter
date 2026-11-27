// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_adapter/hcm_ipc_adapter/ipc_adapter_node/hcm_ipc_adapter_node.hpp"

#include <chrono>
#include <cstring>
#include <utility>

#include "QC/Infras/Log/Logger.hpp"
#include "QC/Node/NodeBase.hpp"

namespace qrb::adapter
{

namespace
{

QC::QCStatus_e ToQcStatus(Status status)
{
  switch (status) {
    // No fresh ROS sample is a normal outcome: the call succeeds and
    // leaves the descriptor untouched.
    case Status::kOk:
    case Status::kNoData:
      return QC::QC_STATUS_OK;
    case Status::kTimeout:
      return QC::QC_STATUS_TIMEOUT;
    case Status::kBadState:
      return QC::QC_STATUS_BAD_STATE;
    case Status::kInvalidArgument:
      return QC::QC_STATUS_BAD_ARGUMENTS;
    case Status::kUnsupportedContract:
    case Status::kVersionMismatch:
      return QC::QC_STATUS_UNSUPPORTED;
    case Status::kResourceError:
      return QC::QC_STATUS_NO_RESOURCE;
    case Status::kProtocolError:
    case Status::kConversionError:
    case Status::kInternalError:
      return QC::QC_STATUS_FAIL;
  }
  return QC::QC_STATUS_FAIL;
}

// Logs `what` with the diagnostic, which lists its reasons as "<reason>; <reason>; ".
void LogError(const std::string & what, std::string diagnostic)
{
  while (!diagnostic.empty() && (diagnostic.back() == ' ' || diagnostic.back() == ';'))
    diagnostic.pop_back();
  using namespace QC;  // QC_LOG_ERROR names the QC logger types unqualified.
  QC_LOG_ERROR("%s: %s", what.c_str(), diagnostic.empty() ? "no details" : diagnostic.c_str());
}

std::string FlowFailure(const char * operation, const std::string & flowId, Status status)
{
  return std::string("IPC Adapter flow '") + flowId + "' " + operation + " failed (" +
         ToString(status) + ")";
}

}  // namespace

QC::QCStatus_e HcmIpcAdapterNodeConfig::VerifyAndSet(const std::string config, std::string & errors)
{
  HcmIpcAdapterConfig parsed;
  Status status = ParseHcmIpcAdapterConfigJson(config, parsed, errors);
  if (status != Status::kOk)
    return ToQcStatus(status);

  config_.adapter = std::move(parsed);
  config_.nodeId.name = config_.adapter.nodeName;
  config_.nodeId.id = static_cast<std::uint8_t>(config_.adapter.nodeId);
  config_.nodeId.type = QC_NODE_TYPE_IPC_ADAPTER;
  config_.numOfEntries = 1;
  return QC::QC_STATUS_OK;
}

void HcmIpcAdapterNodeConfig::Clear()
{
  config_ = {};
}

const std::string & HcmIpcAdapterNodeConfig::GetOptions()
{
  if (options_.empty())
    options_ = HcmIpcAdapterConfigOptions();
  return options_;
}

QC::QCStatus_e HcmIpcAdapterNodeConfig::GetOptions(std::string & options)
{
  options = GetOptions();
  return QC::QC_STATUS_OK;
}

const QC::QCNodeConfigBase_t & HcmIpcAdapterNodeConfig::Get()
{
  return config_;
}

namespace
{

constexpr std::uint32_t kMonitoringEntries = 8;

}  // namespace

HcmIpcAdapterNodeMonitoring::HcmIpcAdapterNodeMonitoring()
{
  data_.numOfEntries = kMonitoringEntries;
}

QC::QCStatus_e HcmIpcAdapterNodeMonitoring::VerifyAndSet(const std::string /*config*/,
    std::string & /*errors*/)
{
  return QC::QC_STATUS_OK;
}

const std::string & HcmIpcAdapterNodeMonitoring::GetOptions()
{
  if (options_.empty())
    options_ = R"({"format":"HcmIpcAdapterMonitoringData"})";
  return options_;
}

QC::QCStatus_e HcmIpcAdapterNodeMonitoring::GetOptions(std::string & options)
{
  options = GetOptions();
  return QC::QC_STATUS_OK;
}

const QC::QCNodeMonitoringBase_t & HcmIpcAdapterNodeMonitoring::Get()
{
  return data_;
}

std::uint32_t HcmIpcAdapterNodeMonitoring::GetMaximalSize()
{
  return sizeof(HcmIpcAdapterMonitoringData);
}

std::uint32_t HcmIpcAdapterNodeMonitoring::GetCurrentSize()
{
  return sizeof(HcmIpcAdapterMonitoringData);
}

QC::QCStatus_e HcmIpcAdapterNodeMonitoring::Place(void * data, std::uint32_t & size)
{
  if (data == nullptr)
    return QC::QC_STATUS_BAD_ARGUMENTS;
  if (size < sizeof(HcmIpcAdapterMonitoringData)) {
    size = sizeof(HcmIpcAdapterMonitoringData);
    return QC::QC_STATUS_OUT_OF_BOUND;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  std::memcpy(data, &data_, sizeof(data_));
  size = sizeof(data_);
  return QC::QC_STATUS_OK;
}

void HcmIpcAdapterNodeMonitoring::RecordProcess(Status status, std::uint64_t latencyNs)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (status == Status::kNoData) {
    ++data_.noDataCount;
    return;
  }
  ++data_.frameCount;
  if (status != Status::kOk) {
    ++data_.errorCount;
    return;
  }

  // Only calls that moved a sample contribute to the latency values.
  ++latencySamples_;
  totalLatencyNs_ += latencyNs;
  data_.lastLatencyNs = latencyNs;
  data_.averageLatencyNs = totalLatencyNs_ / latencySamples_;
  if (latencySamples_ == 1 || latencyNs < data_.minLatencyNs)
    data_.minLatencyNs = latencyNs;
  if (latencyNs > data_.maxLatencyNs)
    data_.maxLatencyNs = latencyNs;
}

void HcmIpcAdapterNodeMonitoring::SetSessionConnected(bool connected)
{
  std::lock_guard<std::mutex> lock(mutex_);
  data_.sessionConnected = connected ? 1u : 0u;
}

void HcmIpcAdapterNodeMonitoring::Reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  data_ = {};
  data_.numOfEntries = kMonitoringEntries;
  latencySamples_ = 0;
  totalLatencyNs_ = 0;
}

HcmIpcAdapterNode::HcmIpcAdapterNode(std::string bootstrapSocketPath)
  : bootstrapSocketPath_(std::move(bootstrapSocketPath))
{
}

HcmIpcAdapterNode::~HcmIpcAdapterNode()
{
  std::lock_guard<std::mutex> lock(mutex_);
  client_.Close();
}

QC::QCStatus_e HcmIpcAdapterNode::Initialize(QC::QCNodeInit_t & config)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (state_.load() != QC::QC_OBJECT_STATE_INITIAL)
    return QC::QC_STATUS_BAD_STATE;

  monitoring_.Reset();
  flowFailure_ = Status::kOk;
  state_.store(QC::QC_OBJECT_STATE_INITIALIZING);
  std::string diagnostic;
  QC::QCStatus_e configStatus = config_.VerifyAndSet(config.config, diagnostic);
  if (configStatus != QC::QC_STATUS_OK) {
    LogError("IPC Adapter config rejected", diagnostic);
    config_.Clear();
    state_.store(QC::QC_OBJECT_STATE_INITIAL);
    return configStatus;
  }

  const FlowConfig & flow = config_.Adapter().flowConfig;
  Status initialized = client_.Initialize(flow, bootstrapSocketPath_, diagnostic);
  QC::QCStatus_e result = ToQcStatus(initialized);
  if (initialized != Status::kOk) {
    LogError(FlowFailure("Initialize", flow.flowId, initialized), diagnostic);
    // Any failure clears the configuration and returns the node to INITIAL. NO_RESOURCE may be
    // transient, so the host may retry Initialize.
    client_.Close();
    config_.Clear();
  }
  monitoring_.SetSessionConnected(client_.IsOpen());
  state_.store(
      initialized == Status::kOk ? QC::QC_OBJECT_STATE_READY : QC::QC_OBJECT_STATE_INITIAL);
  return result;
}

QC::QCStatus_e HcmIpcAdapterNode::Start()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (state_.load() != QC::QC_OBJECT_STATE_READY)
    return QC::QC_STATUS_BAD_STATE;

  state_.store(QC::QC_OBJECT_STATE_STARTING);
  if (flowFailure_ != Status::kOk) {
    // The Session was lost: register the Flow again, under a new token.
    std::string diagnostic;
    const FlowConfig & flow = config_.Adapter().flowConfig;
    Status reconnected = client_.Initialize(flow, bootstrapSocketPath_, diagnostic);
    if (reconnected != Status::kOk) {
      LogError(FlowFailure("Start", flow.flowId, reconnected), diagnostic);
      client_.Close();
      state_.store(QC::QC_OBJECT_STATE_READY);
      return ToQcStatus(reconnected);
    }
    flowFailure_ = Status::kOk;
    monitoring_.SetSessionConnected(client_.IsOpen());
  }
  state_.store(QC::QC_OBJECT_STATE_RUNNING);
  return QC::QC_STATUS_OK;
}

QC::QCStatus_e HcmIpcAdapterNode::ProcessFrameDescriptor(QC::QCFrameDescriptorNodeIfs & frameDesc)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (state_.load() != QC::QC_OBJECT_STATE_RUNNING)
    return QC::QC_STATUS_BAD_STATE;

  const auto start = std::chrono::steady_clock::now();
  const Status status = Process(frameDesc);
  FinishOperation(status);
  const auto latency = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now() - start);
  monitoring_.RecordProcess(status, static_cast<std::uint64_t>(latency.count()));
  return ToQcStatus(status);
}

Status HcmIpcAdapterNode::Process(QC::QCFrameDescriptorNodeIfs & frameDesc)
{
  if (flowFailure_ != Status::kOk)
    return flowFailure_;
  QC::Memory::QCBufferDescriptorBase_t & descriptor =
      frameDesc.GetBuffer(config_.Adapter().globalBufferId);
  return config_.Adapter().flowConfig.direction == Direction::kRosToHcm ?
             client_.Fetch(descriptor) :
             client_.Publish(descriptor);
}

void HcmIpcAdapterNode::FinishOperation(Status status)
{
  // A frame error drops only that frame, as long as the client kept the Session.
  if (flowFailure_ != Status::kOk || status == Status::kOk)
    return;
  if (!IsFatalForFlow(status) && client_.IsOpen())
    return;
  LogError(FlowFailure("ProcessFrameDescriptor", config_.Adapter().flowConfig.flowId, status),
      client_.FailureDiagnostic());
  client_.Close();
  flowFailure_ = status;
  monitoring_.SetSessionConnected(false);
}

QC::QCStatus_e HcmIpcAdapterNode::Stop()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (state_.load() != QC::QC_OBJECT_STATE_RUNNING)
    return QC::QC_STATUS_BAD_STATE;

  state_.store(QC::QC_OBJECT_STATE_STOPING);
  state_.store(QC::QC_OBJECT_STATE_READY);
  return QC::QC_STATUS_OK;
}

QC::QCStatus_e HcmIpcAdapterNode::DeInitialize()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (state_.load() != QC::QC_OBJECT_STATE_READY)
    return QC::QC_STATUS_BAD_STATE;

  state_.store(QC::QC_OBJECT_STATE_DEINITIALIZING);
  client_.Close();
  flowFailure_ = Status::kOk;
  monitoring_.SetSessionConnected(false);
  state_.store(QC::QC_OBJECT_STATE_INITIAL);
  return QC::QC_STATUS_OK;
}

QC::QCObjectState_e HcmIpcAdapterNode::GetState()
{
  return state_.load();
}

QC::QCNodeConfigIfs & HcmIpcAdapterNode::GetConfigurationIfs()
{
  return config_;
}

QC::QCNodeMonitoringIfs & HcmIpcAdapterNode::GetMonitoringIfs()
{
  return monitoring_;
}

}  // namespace qrb::adapter

namespace
{

// Register the IPC Adapter Node with QCNodeFactory at static-initialization time.
const QC::Node::NodeRegistrar<qrb::adapter::HcmIpcAdapterNode> g_registerIpcAdapter{
    QC_NODE_TYPE_IPC_ADAPTER};

}  // namespace
