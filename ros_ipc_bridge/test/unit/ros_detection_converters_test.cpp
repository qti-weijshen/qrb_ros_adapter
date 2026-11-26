// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

// Converters are stateless functions over message structures: no ROS Context, no node.

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <string>

#include "qrb_ros_adapter/ros_ipc_bridge/flow_bridge/detection_2d_ros_converter.hpp"
#include "qrb_ros_adapter/ros_ipc_bridge/flow_bridge/detection_3d_ros_converter.hpp"

using namespace qrb::adapter;

namespace
{

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

vision_msgs::msg::ObjectHypothesisWithPose MakeHypothesis(const char * classId,
    double score,
    bool withCovariance)
{
  vision_msgs::msg::ObjectHypothesisWithPose result;
  result.hypothesis.class_id = classId;
  result.hypothesis.score = score;
  result.pose.pose.position.x = 0.5;
  result.pose.pose.orientation.z = 0.6;
  result.pose.pose.orientation.w = 0.8;
  if (withCovariance) {
    for (std::size_t i = 0; i < result.pose.covariance.size(); ++i)
      result.pose.covariance[i] = 0.25 * static_cast<double>(i);
  }
  return result;
}

/** @brief Two detections, the second with no id; detection headers equal the array's. */
Detection2DRosMessage MakeRos2d()
{
  Detection2DRosMessage array;
  array.header.stamp.sec = 12;
  array.header.stamp.nanosec = 345;
  array.header.frame_id = "camera";
  array.detections.resize(2);
  auto & first = array.detections[0];
  first.header = array.header;
  first.id = "track-1";
  first.bbox.center.position.x = 320.5;
  first.bbox.center.position.y = 240.25;
  first.bbox.center.theta = 0.125;
  first.bbox.size_x = 64.0;
  first.bbox.size_y = 32.0;
  first.results = {MakeHypothesis("person", 0.9, false), MakeHypothesis("mannequin", 0.1, true)};
  auto & second = array.detections[1];
  second.header = array.header;
  second.bbox.size_x = 8.0;
  second.results = {MakeHypothesis("cup", 0.5, false)};
  return array;
}

Detection3DRosMessage MakeRos3d()
{
  Detection3DRosMessage array;
  array.header.stamp.sec = 7;
  array.header.frame_id = "lidar";
  array.detections.resize(1);
  auto & detection = array.detections[0];
  detection.header = array.header;
  detection.id = "car-7";
  detection.bbox.center.position.x = 10.0;
  detection.bbox.center.position.z = 0.75;
  detection.bbox.center.orientation.z = 0.6;
  detection.bbox.center.orientation.w = 0.8;
  detection.bbox.size.x = 4.5;
  detection.bbox.size.y = 2.0;
  detection.bbox.size.z = 1.5;
  detection.results = {MakeHypothesis("car", 0.8, true)};
  return array;
}

}  // namespace

// Every field of a Detection2DArray survives ROS -> Contract -> wire -> ROS; a known covariance is
// carried and an all-zero one is left out of the Contract; an empty array is a result; a detection
// in another frame or a non-finite value is rejected, and so is an invalid Contract.
TEST(Detection2DRosConverter, RoundTripsEveryFieldAndRejectsWhatItCannotCarry)
{
  const Detection2DRosMessage source = MakeRos2d();
  Detection2DRosContract contract;
  ASSERT_TRUE(Detection2DRosConversion::RosToContract(source, contract));
  EXPECT_EQ(contract.header().frame_id(), "camera");
  ASSERT_EQ(contract.detections_size(), 2);
  EXPECT_EQ(contract.detections(0).id(), "track-1");
  EXPECT_DOUBLE_EQ(contract.detections(0).bbox().center().position().x(), 320.5);
  EXPECT_DOUBLE_EQ(contract.detections(0).bbox().center().theta(), 0.125);
  EXPECT_DOUBLE_EQ(contract.detections(0).bbox().size_y(), 32.0);
  ASSERT_EQ(contract.detections(0).results_size(), 2);
  EXPECT_EQ(contract.detections(0).results(0).hypothesis().class_id(), "person");
  EXPECT_EQ(contract.detections(0).results(0).pose().covariance_size(), 0);
  EXPECT_EQ(contract.detections(0).results(1).pose().covariance_size(), 36);

  // The Contract crosses the socket as bytes.
  Detection2DRosContract received;
  ASSERT_TRUE(received.ParseFromString(contract.SerializeAsString()));
  Detection2DRosMessage back;
  back.detections.resize(5);  // stale content must not survive
  ASSERT_TRUE(Detection2DRosConversion::ContractToRos(received, back));
  EXPECT_EQ(back, source);

  // A detection's own stamp is not carried and an empty frame_id is allowed; back on ROS every
  // detection gets the header of the array.
  Detection2DRosMessage unstamped = source;
  unstamped.detections[0].header = std_msgs::msg::Header();
  unstamped.detections[1].header.stamp.sec = 99;
  ASSERT_TRUE(Detection2DRosConversion::RosToContract(unstamped, contract));
  ASSERT_TRUE(Detection2DRosConversion::ContractToRos(contract, back));
  EXPECT_EQ(back, source);

  Detection2DRosMessage empty;
  empty.header.frame_id = "camera";
  ASSERT_TRUE(Detection2DRosConversion::RosToContract(empty, contract));
  EXPECT_EQ(contract.detections_size(), 0);
  ASSERT_TRUE(Detection2DRosConversion::ContractToRos(contract, back));
  EXPECT_EQ(back, empty);

  Detection2DRosMessage otherFrame = source;
  otherFrame.detections[1].header.frame_id = "base_link";
  const ConversionResult frame = Detection2DRosConversion::RosToContract(otherFrame, contract);
  EXPECT_FALSE(frame);
  EXPECT_NE(frame.Diagnostic().find("detection 1"), std::string::npos);

  Detection2DRosMessage nan = source;
  nan.detections[0].bbox.size_x = kNaN;
  EXPECT_FALSE(Detection2DRosConversion::RosToContract(nan, contract));
  nan = source;
  nan.detections[1].results[0].pose.covariance[3] = kNaN;
  EXPECT_FALSE(Detection2DRosConversion::RosToContract(nan, contract));
  Detection2DRosMessage slashFrame = source;
  slashFrame.header.frame_id = "/camera";
  EXPECT_FALSE(Detection2DRosConversion::RosToContract(slashFrame, contract));

  // A Contract from a peer is validated before it becomes a ROS message.
  Detection2DRosContract invalid = received;
  invalid.mutable_detections(0)->clear_bbox();
  Detection2DRosMessage untouched = source;
  EXPECT_FALSE(Detection2DRosConversion::ContractToRos(invalid, untouched));
  EXPECT_EQ(untouched, source);
  invalid = received;
  invalid.mutable_detections(0)->mutable_results(1)->mutable_pose()->add_covariance(1.0);
  EXPECT_FALSE(Detection2DRosConversion::ContractToRos(invalid, untouched));
}

// The same for a Detection3DArray, whose boxes have a size and an orientation.
TEST(Detection3DRosConverter, RoundTripsEveryFieldAndRejectsWhatItCannotCarry)
{
  const Detection3DRosMessage source = MakeRos3d();
  Detection3DRosContract contract;
  ASSERT_TRUE(Detection3DRosConversion::RosToContract(source, contract));
  EXPECT_EQ(contract.header().frame_id(), "lidar");
  ASSERT_EQ(contract.detections_size(), 1);
  EXPECT_DOUBLE_EQ(contract.detections(0).bbox().center().position().z(), 0.75);
  EXPECT_DOUBLE_EQ(contract.detections(0).bbox().center().orientation().w(), 0.8);
  EXPECT_DOUBLE_EQ(contract.detections(0).bbox().size().x(), 4.5);
  EXPECT_EQ(contract.detections(0).results(0).pose().covariance_size(), 36);

  Detection3DRosContract received;
  ASSERT_TRUE(received.ParseFromString(contract.SerializeAsString()));
  Detection3DRosMessage back;
  ASSERT_TRUE(Detection3DRosConversion::ContractToRos(received, back));
  EXPECT_EQ(back, source);

  // A publisher that never sets the box orientation sends zeros; they are passed on.
  Detection3DRosMessage zeroOrientation = source;
  zeroOrientation.detections[0].bbox.center.orientation = geometry_msgs::msg::Quaternion();
  zeroOrientation.detections[0].bbox.center.orientation.w = 0.0;
  ASSERT_TRUE(Detection3DRosConversion::RosToContract(zeroOrientation, contract));
  ASSERT_TRUE(Detection3DRosConversion::ContractToRos(contract, back));
  EXPECT_EQ(back, zeroOrientation);

  Detection3DRosMessage otherFrame = source;
  otherFrame.detections[0].header.frame_id = "map";
  EXPECT_FALSE(Detection3DRosConversion::RosToContract(otherFrame, contract));
  Detection3DRosMessage negative = source;
  negative.detections[0].bbox.size.z = -1.0;
  EXPECT_FALSE(Detection3DRosConversion::RosToContract(negative, contract));
  Detection3DRosMessage infinite = source;
  infinite.detections[0].bbox.center.position.x = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(Detection3DRosConversion::RosToContract(infinite, contract));

  Detection3DRosContract invalid = received;
  invalid.mutable_detections(0)->clear_bbox();
  Detection3DRosMessage untouched = source;
  EXPECT_FALSE(Detection3DRosConversion::ContractToRos(invalid, untouched));
  EXPECT_EQ(untouched, source);
}
