// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <limits>
#include <nlohmann/json.hpp>
#include <string>
#include <utility>

#include "qrb_ros_adapter/hcm_ipc_adapter/ipc_adapter_node/hcm_ipc_adapter_config.hpp"
#include "qrb_ros_adapter/ipc_runtime/contract_descriptor_registry.hpp"

using namespace qrb::adapter;

namespace
{

/** The required fields only. */
nlohmann::json MakeConfig()
{
  nlohmann::json root;
  root["static"] = {
      {"name", "pose_adapter"},
      {"contract_type", std::string(kPoseWithCovarianceStampedContractType)},
      {"direction", kDirectionCodeRosToHcm},
      {"ros_topic", "/localization/pose"},
  };
  return root;
}

}  // namespace

// The QCNode static block is the HCM side's only Flow configuration; a bad
// one fails with a reason and is never half-applied.
TEST(HcmIpcAdapterConfig, ParsesTheStaticBlockAndRejectsInvalidOnesUntouched)
{
  HcmIpcAdapterConfig config;
  std::string diagnostic;
  ASSERT_EQ(ParseHcmIpcAdapterConfigJson(MakeConfig().dump(), config, diagnostic), Status::kOk)
      << diagnostic;
  // The QCNode name is the Flow identity; omitted optional fields take their defaults.
  EXPECT_EQ(config.flowConfig.flowId, "pose_adapter");
  EXPECT_EQ(config.flowConfig.direction, Direction::kRosToHcm);
  EXPECT_EQ(config.flowConfig.rosTopic, "/localization/pose");
  EXPECT_TRUE(config.flowConfig.qosProfile == QosProfile{});
  EXPECT_EQ(config.globalBufferId, kHcmDefaultGlobalBufferId);

  nlohmann::json full = MakeConfig();
  full["static"]["direction"] = kDirectionCodeHcmToRos;
  full["static"]["id"] = 7;
  full["static"]["qos_profile"] = {
      {"reliability", "best_effort"}, {"durability", "transient_local"}, {"depth", 1}};
  full["static"]["globalBufferIdMap"] = {{{"name", "Input"}, {"id", 5}}};
  ASSERT_EQ(ParseHcmIpcAdapterConfigJson(full.dump(), config, diagnostic), Status::kOk)
      << diagnostic;
  EXPECT_EQ(config.nodeId, 7u);
  EXPECT_EQ(config.globalBufferId, 5u);
  EXPECT_EQ(config.flowConfig.direction, Direction::kHcmToRos);
  EXPECT_TRUE((config.flowConfig.qosProfile ==
               QosProfile{QosReliability::kBestEffort, QosDurability::kTransientLocal, 1}));

  // Each QoS member is optional on its own.
  nlohmann::json partial = MakeConfig();
  partial["static"]["qos_profile"] = {{"depth", 4}};
  ASSERT_EQ(ParseHcmIpcAdapterConfigJson(partial.dump(), config, diagnostic), Status::kOk)
      << diagnostic;
  EXPECT_TRUE((config.flowConfig.qosProfile ==
               QosProfile{QosReliability::kReliable, QosDurability::kVolatile, 4}));

  // The advertised option schema lists every required field.
  const nlohmann::json options = nlohmann::json::parse(HcmIpcAdapterConfigOptions());
  const nlohmann::json required = MakeConfig();
  EXPECT_EQ(options["required"].size(), required["static"].size());
  for (const auto & item : required["static"].items()) {
    const std::string key = "static." + item.key();
    EXPECT_NE(std::find(options["required"].begin(), options["required"].end(), key),
        options["required"].end())
        << key;
  }

  const auto with = [](const char * key, nlohmann::json value) {
    nlohmann::json root = MakeConfig();
    root["static"][key] = std::move(value);
    return root.dump();
  };
  const auto withQos = [](const char * key, nlohmann::json value) {
    nlohmann::json root = MakeConfig();
    root["static"]["qos_profile"][key] = std::move(value);
    return root.dump();
  };
  nlohmann::json missing = MakeConfig();
  missing["static"].erase("ros_topic");

  const struct
  {
    const char * name;
    std::string json;
    Status expected;
  } cases[] = {
      {"MalformedJson", "{\"static\":", Status::kInvalidArgument},
      {"MissingStatic", "{}", Status::kInvalidArgument},
      {"MissingRosTopic", missing.dump(), Status::kInvalidArgument},
      {"EmptyName", with("name", ""), Status::kInvalidArgument},
      // direction is the integer 0 or 1, not a name.
      {"UnknownDirection", with("direction", 2), Status::kInvalidArgument},
      {"DirectionString", with("direction", "ros_to_hcm"), Status::kInvalidArgument},
      {"IdOutOfRange", with("id", 256), Status::kInvalidArgument},
      // qos_profile is an object of raw ROS policies, not a preset name.
      {"QosProfileNotObject", with("qos_profile", "sensor_data"), Status::kInvalidArgument},
      {"UnknownQosReliability", withQos("reliability", "system_default"), Status::kInvalidArgument},
      {"UnsupportedQosPolicy", withQos("history", "keep_all"), Status::kInvalidArgument},
      // The node has one buffer, so the QCNode globalBufferIdMap has one full entry.
      {"BufferMapNotArray", with("globalBufferIdMap", {{"name", "Output"}, {"id", 1}}),
          Status::kInvalidArgument},
      {"BufferMapTwoEntries",
          with("globalBufferIdMap",
              {{{"name", "Input"}, {"id", 0}}, {{"name", "Output"}, {"id", 1}}}),
          Status::kInvalidArgument},
      {"BufferMapUnsetId",
          with("globalBufferIdMap",
              {{{"name", "Output"}, {"id", std::numeric_limits<std::uint32_t>::max()}}}),
          Status::kInvalidArgument},
      {"UnknownField", with("flow_id", "pose_to_hcm"), Status::kInvalidArgument},
      {"UnknownContract", with("contract_type", "qrb.contract.unknown.v1.Missing"),
          Status::kUnsupportedContract},
  };
  for (const auto & c : cases) {
    SCOPED_TRACE(c.name);
    HcmIpcAdapterConfig untouched;
    untouched.nodeName = "untouched";
    diagnostic.clear();
    EXPECT_EQ(ParseHcmIpcAdapterConfigJson(c.json, untouched, diagnostic), c.expected);
    EXPECT_FALSE(diagnostic.empty());
    EXPECT_EQ(untouched.nodeName, "untouched");
  }
}
