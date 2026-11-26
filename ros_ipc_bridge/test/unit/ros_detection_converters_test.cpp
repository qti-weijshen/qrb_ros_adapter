// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

// Converters are stateless functions over message structures: no ROS Context, no node.

#include <gtest/gtest.h>

#include <limits>
#include <string>

#include "qrb_ros_adapter/ros_ipc_bridge/flow_bridge/detection_2d_ros_converter.hpp"
#include "qrb_ros_adapter/ros_ipc_bridge/flow_bridge/detection_3d_ros_converter.hpp"

using namespace qrb::adapter;

namespace
{

vision_msgs::msg::ObjectHypothesisWithPose MakeHypothesis(const char * classId,
    double score,
    bool withCovariance)
{
  vision_msgs::msg::ObjectHypothesisWithPose result;
  result.hypothesis.class_id = classId;
  result.hypothesis.score = score;
  result.pose.pose.position.x = 0.5;
  result.pose.pose.position.y = -0.75;
  result.pose.pose.position.z = 1.25;
  result.pose.pose.orientation.x = 0.1;
  result.pose.pose.orientation.y = 0.2;
  result.pose.pose.orientation.z = 0.3;
  result.pose.pose.orientation.w = 0.9;
  if (withCovariance) {
    for (std::size_t i = 0; i < result.pose.covariance.size(); ++i)
      result.pose.covariance[i] = 0.25 * static_cast<double>(i + 1);
  }
  return result;
}

}  // namespace

// A Detection2DArray survives ROS -> Contract -> wire -> ROS, the empty id and an all-zero
// covariance left out included; a detection's own stamp is not carried and it gets the array's
// header back. A detection in another frame and an invalid value are rejected.
TEST(Detection2DRosConverter, RoundTripsEveryFieldAndRejectsWhatItCannotCarry)
{
  Detection2DRosMessage source;
  source.header.stamp.sec = 12;
  source.header.stamp.nanosec = 345;
  source.header.frame_id = "camera";
  source.detections.resize(2);
  auto & first = source.detections[0];
  first.header = source.header;
  first.id = "track-1";
  first.bbox.center.position.x = 320.5;
  first.bbox.center.position.y = 240.25;
  first.bbox.center.theta = 0.125;
  first.bbox.size_x = 64.0;
  first.bbox.size_y = 32.0;
  first.results = {MakeHypothesis("person", 0.9, false), MakeHypothesis("mannequin", 0.1, true)};
  auto & second = source.detections[1];
  second.header = source.header;
  second.bbox.center.position.x = 16.5;
  second.bbox.center.position.y = 24.75;
  second.bbox.center.theta = -0.5;
  second.bbox.size_x = 8.0;
  second.bbox.size_y = 6.0;
  second.results = {MakeHypothesis("cup", 0.5, false)};

  Detection2DRosContract contract;
  ASSERT_TRUE(Detection2DRosConversion::RosToContract(source, contract));
  ASSERT_EQ(contract.detections_size(), 2);
  EXPECT_DOUBLE_EQ(contract.detections(0).bbox().center().position().y(), 240.25);
  EXPECT_DOUBLE_EQ(contract.detections(0).bbox().size_y(), 32.0);
  EXPECT_EQ(contract.detections(0).results(0).pose().covariance_size(), 0);
  EXPECT_DOUBLE_EQ(contract.detections(0).results(1).pose().covariance(35), 9.0);
  Detection2DRosContract received;
  ASSERT_TRUE(received.ParseFromString(contract.SerializeAsString()));
  Detection2DRosMessage back;
  back.detections.resize(5);  // stale
  ASSERT_TRUE(Detection2DRosConversion::ContractToRos(received, back));
  EXPECT_EQ(back, source);

  Detection2DRosMessage unstamped = source;
  unstamped.detections[0].header = std_msgs::msg::Header();
  unstamped.detections[1].header.stamp.sec = 99;
  ASSERT_TRUE(Detection2DRosConversion::RosToContract(unstamped, contract));
  ASSERT_TRUE(Detection2DRosConversion::ContractToRos(contract, back));
  EXPECT_EQ(back, source);

  Detection2DRosMessage otherFrame = source;
  otherFrame.detections[1].header.frame_id = "base_link";
  const ConversionResult frame = Detection2DRosConversion::RosToContract(otherFrame, contract);
  EXPECT_FALSE(frame);
  EXPECT_NE(frame.Diagnostic().find("detection 1"), std::string::npos);
  Detection2DRosMessage invalid = source;
  invalid.detections[0].bbox.size_x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(Detection2DRosConversion::RosToContract(invalid, contract));
}

// A Detection3DArray survives the same trip with the box center and size kept apart, and an unset
// box orientation is passed on as zeros; an invalid value is rejected.
TEST(Detection3DRosConverter, RoundTripsEveryFieldAndRejectsWhatItCannotCarry)
{
  Detection3DRosMessage source;
  source.header.stamp.sec = 7;
  source.header.stamp.nanosec = 8;
  source.header.frame_id = "lidar";
  source.detections.resize(1);
  auto & detection = source.detections[0];
  detection.header = source.header;
  detection.id = "car-7";
  detection.bbox.center.position.x = 10.0;
  detection.bbox.center.position.y = -3.5;
  detection.bbox.center.position.z = 0.75;
  detection.bbox.center.orientation.x = 0.1;
  detection.bbox.center.orientation.y = 0.2;
  detection.bbox.center.orientation.z = 0.3;
  detection.bbox.center.orientation.w = 0.9;
  detection.bbox.size.x = 4.5;
  detection.bbox.size.y = 2.0;
  detection.bbox.size.z = 1.5;
  detection.results = {MakeHypothesis("car", 0.8, true)};

  Detection3DRosContract contract;
  ASSERT_TRUE(Detection3DRosConversion::RosToContract(source, contract));
  ASSERT_EQ(contract.detections_size(), 1);
  EXPECT_DOUBLE_EQ(contract.detections(0).bbox().center().position().z(), 0.75);
  EXPECT_DOUBLE_EQ(contract.detections(0).bbox().center().orientation().x(), 0.1);
  EXPECT_DOUBLE_EQ(contract.detections(0).bbox().size().z(), 1.5);
  Detection3DRosContract received;
  ASSERT_TRUE(received.ParseFromString(contract.SerializeAsString()));
  Detection3DRosMessage back;
  ASSERT_TRUE(Detection3DRosConversion::ContractToRos(received, back));
  EXPECT_EQ(back, source);

  Detection3DRosMessage zeroOrientation = source;
  zeroOrientation.detections[0].bbox.center.orientation = geometry_msgs::msg::Quaternion();
  zeroOrientation.detections[0].bbox.center.orientation.w = 0.0;
  ASSERT_TRUE(Detection3DRosConversion::RosToContract(zeroOrientation, contract));
  ASSERT_TRUE(Detection3DRosConversion::ContractToRos(contract, back));
  EXPECT_EQ(back, zeroOrientation);

  Detection3DRosMessage negative = source;
  negative.detections[0].bbox.size.z = -1.0;
  EXPECT_FALSE(Detection3DRosConversion::RosToContract(negative, contract));
}
