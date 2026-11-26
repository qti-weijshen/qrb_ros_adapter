// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include <gtest/gtest.h>

#include <string>

#include "qrb_ros_adapter/ipc_runtime/contract_descriptor_registry.hpp"
#include "qrb_ros_adapter/ipc_runtime/control_status.hpp"
#include "qrb_ros_adapter/ipc_runtime/flow_config.hpp"

using namespace qrb::adapter;

namespace
{

FlowConfig MakeValidConfig()
{
  FlowConfig config;
  config.flowId = "flow_a";
  config.contractTypeName = std::string(kPoseWithCovarianceStampedContractType);
  config.direction = Direction::kRosToHcm;
  config.rosTopic = "/robot/pose";
  return config;
}

const ContractDescriptor & PoseDescriptor()
{
  return *FindContractDescriptor(kPoseWithCovarianceStampedContractType);
}

}  // namespace

// ValidateFlowConfig accepts the limits, rejects each out-of-range field with a diagnostic, and
// rejects unknown Direction and QoS values that arrive over the wire.
TEST(FlowConfigValidation, RejectsOutOfRangeFields)
{
  std::string diagnostic;
  FlowConfig limits = MakeValidConfig();
  limits.timeoutMs = kMaxTimeoutMs;
  limits.startupTimeoutMs = kMaxStartupTimeoutMs;
  EXPECT_EQ(ValidateFlowConfig(limits, PoseDescriptor(), diagnostic), Status::kOk) << diagnostic;

  const struct
  {
    const char * name;
    void (*mutate)(FlowConfig &);
    Status expected;
  } cases[] = {
      {"EmptyFlowId", [](FlowConfig & c) { c.flowId.clear(); }, Status::kInvalidArgument},
      {"OtherContract",
          [](FlowConfig & c) { c.contractTypeName = std::string(kPointCloudContractType); },
          Status::kInvalidArgument},
      {"TopicWithSpace", [](FlowConfig & c) { c.rosTopic = "/a b"; }, Status::kInvalidArgument},
      {"TimeoutBelowMin", [](FlowConfig & c) { c.timeoutMs = kMinTimeoutMs - 1; },
          Status::kInvalidArgument},
      {"QosDepthZero", [](FlowConfig & c) { c.qosProfile.depth = 0; }, Status::kInvalidArgument},
      {"QosDepthAboveMax", [](FlowConfig & c) { c.qosProfile.depth = kMaxQosDepth + 1; },
          Status::kInvalidArgument},
      {"UnspecifiedDirection", [](FlowConfig & c) { c.direction = Direction::kUnspecified; },
          Status::kUnsupportedContract},
      {"WireDirection",
          [](FlowConfig & c) {
            c.direction = FromProtoDirection(static_cast<ipc_proto::Direction>(99));
          },
          Status::kUnsupportedContract},
      {"WireReliability",
          [](FlowConfig & c) {
            ipc_proto::QosProfile wire;
            wire.set_reliability(static_cast<ipc_proto::QosReliability>(99));
            wire.set_depth(10);
            c.qosProfile = FromProtoQosProfile(wire);
          },
          Status::kInvalidArgument},
      {"WireDurability",
          [](FlowConfig & c) {
            ipc_proto::QosProfile wire;
            wire.set_durability(static_cast<ipc_proto::QosDurability>(99));
            wire.set_depth(10);
            c.qosProfile = FromProtoQosProfile(wire);
          },
          Status::kInvalidArgument},
  };
  for (const auto & c : cases) {
    SCOPED_TRACE(c.name);
    FlowConfig config = MakeValidConfig();
    c.mutate(config);
    diagnostic.clear();
    EXPECT_EQ(ValidateFlowConfig(config, PoseDescriptor(), diagnostic), c.expected);
    EXPECT_FALSE(diagnostic.empty());
  }

  ipc_proto::QosProfile wire;
  const QosProfile qos{QosReliability::kBestEffort, QosDurability::kTransientLocal, 3};
  ToProtoQosProfile(qos, wire);
  const QosProfile back = FromProtoQosProfile(wire);
  EXPECT_EQ(back.reliability, qos.reliability);
  EXPECT_EQ(back.durability, qos.durability);
  EXPECT_EQ(back.depth, qos.depth);
}

// Every Status keeps its class across the proto mapping and is exactly one of: success or no
// data, a frame error, or fatal for the Flow.
TEST(StatusMapping, ClassifiesEveryStatusAndSurvivesTheWire)
{
  const Status all[] = {Status::kOk, Status::kNoData, Status::kInvalidArgument,
      Status::kUnsupportedContract, Status::kProtocolError, Status::kConversionError,
      Status::kResourceError, Status::kInternalError, Status::kVersionMismatch, Status::kTimeout,
      Status::kBadState};
  for (Status status : all) {
    SCOPED_TRACE(ToString(status));
    const ipc_proto::StatusCode code = ToProtoStatusCode(status);
    EXPECT_NE(code, ipc_proto::STATUS_CODE_UNSPECIFIED);
    const Status back = FromProtoStatusCode(code);
    EXPECT_EQ(IsOk(back), IsOk(status));
    EXPECT_EQ(IsFatalForFlow(back), IsFatalForFlow(status));
    EXPECT_EQ(IsFrameError(back), IsFrameError(status));
    const int kinds = (status == Status::kOk || status == Status::kNoData) + IsFrameError(status) +
                      IsFatalForFlow(status);
    EXPECT_EQ(kinds, 1);
  }
  EXPECT_TRUE(IsFrameError(Status::kConversionError));
  EXPECT_TRUE(IsFrameError(Status::kInvalidArgument));
  EXPECT_TRUE(IsFatalForFlow(Status::kProtocolError));
  EXPECT_TRUE(IsFatalForFlow(Status::kResourceError));
  EXPECT_EQ(FromProtoStatusCode(static_cast<ipc_proto::StatusCode>(99)), Status::kProtocolError);
}
