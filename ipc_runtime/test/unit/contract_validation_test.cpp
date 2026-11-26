// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_adapter/ipc_runtime/contract_validation.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <string>

#include "qrb_ros_adapter/ipc_runtime/transport_header.hpp"

using namespace qrb::adapter;

namespace geometry_proto = ::qrb::contract::geometry::v1;
namespace sensor_proto = ::qrb::contract::sensor::v1;
namespace vision_proto = ::qrb::contract::vision::v1;

namespace
{

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();

// 2 rows x 4 points of 16 bytes: x/y/z FLOAT32 and 4 UINT8 intensity bytes.
sensor_proto::PointCloud2 MakeCloud()
{
  sensor_proto::PointCloud2 cloud;
  cloud.mutable_header()->set_frame_id("lidar");
  cloud.set_height(2);
  cloud.set_width(4);
  cloud.set_point_step(16);
  cloud.set_row_step(64);
  const char * names[] = {"x", "y", "z", "intensity"};
  for (std::uint32_t i = 0; i < 4; ++i) {
    auto * field = cloud.add_fields();
    field->set_name(names[i]);
    field->set_offset(4 * i);
    field->set_datatype(
        i < 3 ? sensor_proto::POINT_FIELD_TYPE_FLOAT32 : sensor_proto::POINT_FIELD_TYPE_UINT8);
    field->set_count(i < 3 ? 1 : 4);
  }
  return cloud;
}

constexpr std::uint64_t kCloudBytes = 128;

}  // namespace

// A frame_id may be empty and up to kMaxFrameIdLength long, but not absolute or hold a NUL; a
// header also needs nanosec below one second.
TEST(ContractValidation, ValidatesFrameIdsAndHeaders)
{
  EXPECT_TRUE(ValidateFrameId(""));
  EXPECT_TRUE(ValidateFrameId(std::string(contract_limits::kMaxFrameIdLength, 'f')));
  EXPECT_FALSE(ValidateFrameId(std::string(contract_limits::kMaxFrameIdLength + 1, 'f')));
  EXPECT_FALSE(ValidateFrameId("/map"));
  EXPECT_FALSE(ValidateFrameId(std::string("base\0link", 9)));

  qrb::contract::common::v1::Header header;
  header.set_frame_id("map");
  header.mutable_stamp()->set_nanosec(999999999U);
  EXPECT_TRUE(ValidateHeader(header));
  header.mutable_stamp()->set_nanosec(1000000000U);
  EXPECT_FALSE(ValidateHeader(header));
}

// PoseWithCovarianceStamped needs a finite pose with a non-zero quaternion and 0 or 36 finite
// covariances.
TEST(ContractValidation, RejectsInvalidPoseWithCovarianceStamped)
{
  using Pose = geometry_proto::PoseWithCovarianceStamped;
  Pose pose;
  pose.mutable_header()->set_frame_id("map");
  pose.mutable_pose()->mutable_pose()->mutable_orientation()->set_w(1.0);
  EXPECT_TRUE(ValidatePoseWithCovarianceStamped(pose));
  for (int i = 0; i < 36; ++i)
    pose.mutable_pose()->add_covariance(0.1);
  EXPECT_TRUE(ValidatePoseWithCovarianceStamped(pose));

  const auto rejects = [&pose](auto mutate) {
    Pose value = pose;
    mutate(value);
    return !ValidatePoseWithCovarianceStamped(value);
  };
  EXPECT_TRUE(rejects([](Pose & p) { p.clear_pose(); }));
  EXPECT_TRUE(
      rejects([](Pose & p) { p.mutable_pose()->mutable_pose()->mutable_position()->set_y(kNaN); }));
  EXPECT_TRUE(
      rejects([](Pose & p) { p.mutable_pose()->mutable_pose()->mutable_orientation()->set_w(0); }));
  EXPECT_TRUE(rejects([](Pose & p) { p.mutable_pose()->mutable_covariance()->RemoveLast(); }));
  EXPECT_TRUE(rejects([](Pose & p) { p.mutable_pose()->set_covariance(7, kInf); }));
}

// A PointCloud2 layout must fit the external payload exactly as declared; a larger payload is
// fine.
TEST(ContractValidation, RejectsPointCloudLayoutsThePayloadCannotBack)
{
  using Cloud = sensor_proto::PointCloud2;
  EXPECT_TRUE(ValidatePointCloud2(MakeCloud(), kCloudBytes));
  EXPECT_TRUE(ValidatePointCloud2(MakeCloud(), kCloudBytes + 4096));
  EXPECT_FALSE(ValidatePointCloud2(MakeCloud(), kCloudBytes - 1));

  const auto rejects = [](auto mutate) {
    Cloud value = MakeCloud();
    mutate(value);
    return !ValidatePointCloud2(value, std::numeric_limits<std::uint64_t>::max());
  };
  EXPECT_TRUE(rejects([](Cloud & c) { c.set_width(std::numeric_limits<std::uint32_t>::max()); }));
  EXPECT_TRUE(rejects([](Cloud & c) { c.mutable_fields(3)->set_count(5); }));
  EXPECT_TRUE(rejects([](Cloud & c) {
    c.mutable_fields(0)->set_datatype(sensor_proto::POINT_FIELD_TYPE_UNSPECIFIED);
  }));
}

// Odometry adds a valid child_frame_id, which may be empty, and a present twist with 0 or 36
// finite covariances.
TEST(ContractValidation, RejectsInvalidOdometry)
{
  using Odom = qrb::contract::nav::v1::Odometry;
  Odom odom;
  odom.mutable_header()->set_frame_id("odom");
  odom.mutable_pose()->mutable_pose()->mutable_orientation()->set_w(1.0);
  odom.mutable_twist()->mutable_twist()->mutable_linear()->set_x(0.3);
  EXPECT_TRUE(ValidateOdometry(odom));
  odom.set_child_frame_id("base_link");
  EXPECT_TRUE(ValidateOdometry(odom));

  const auto rejects = [&odom](auto mutate) {
    Odom value = odom;
    mutate(value);
    return !ValidateOdometry(value);
  };
  EXPECT_TRUE(rejects([](Odom & o) { o.set_child_frame_id("/base_link"); }));
  EXPECT_TRUE(rejects([](Odom & o) { o.clear_twist(); }));
  EXPECT_TRUE(rejects([](Odom & o) {
    for (int i = 0; i < 36; ++i)
      o.mutable_twist()->add_covariance(i == 7 ? kInf : 0);
  }));
}

// TwistStamped needs a present, finite twist.
TEST(ContractValidation, RejectsInvalidTwistStamped)
{
  geometry_proto::TwistStamped twist;
  twist.mutable_twist()->mutable_linear()->set_x(0.5);
  EXPECT_TRUE(ValidateTwistStamped(twist));
  twist.mutable_twist()->mutable_angular()->set_y(-kInf);
  EXPECT_FALSE(ValidateTwistStamped(twist));
  twist.clear_twist();
  EXPECT_FALSE(ValidateTwistStamped(twist));
}

// TFMessage checks every transform's translation and non-zero rotation, holds at most
// kMaxTransformCount transforms, and is valid when empty.
TEST(ContractValidation, RejectsInvalidTFMessage)
{
  using TF = ::qrb::contract::tf2::v1::TFMessage;
  TF message;
  EXPECT_TRUE(ValidateTFMessage(message));
  for (int i = 0; i < 2; ++i) {
    auto * transform = message.add_transforms();
    transform->set_child_frame_id("odom");
    transform->mutable_transform()->mutable_rotation()->set_w(1.0);
  }
  EXPECT_TRUE(ValidateTFMessage(message));

  const auto rejects = [&message](auto mutate) {
    TF value = message;
    mutate(value);
    return !ValidateTFMessage(value);
  };
  EXPECT_TRUE(rejects([](TF & m) { m.mutable_transforms(1)->clear_transform(); }));
  EXPECT_TRUE(rejects(
      [](TF & m) { m.mutable_transforms(1)->mutable_transform()->mutable_rotation()->set_w(0); }));
  EXPECT_TRUE(rejects([](TF & m) {
    m.mutable_transforms(0)->mutable_transform()->mutable_translation()->set_x(kNaN);
  }));
  EXPECT_TRUE(rejects([](TF & m) {
    while (static_cast<std::size_t>(m.transforms_size()) <= contract_limits::kMaxTransformCount)
      *m.add_transforms() = m.transforms(0);
  }));
}

// JointState holds at most kMaxJointCount joints with non-empty, bounded names; position, velocity
// and effort are each empty (a missing measurement) or one finite value per name.
TEST(ContractValidation, RejectsInvalidJointStateAndPreservesMissingMeasurements)
{
  using Message = sensor_proto::JointState;
  Message message;
  EXPECT_TRUE(ValidateJointState(message));
  for (int i = 0; i < 3; ++i) {
    message.add_name("joint_" + std::to_string(i));
    message.add_position(-0.5 * i);
    message.add_effort(0.0);
  }
  EXPECT_TRUE(ValidateJointState(message));

  const auto rejects = [&message](auto mutate) {
    Message value = message;
    mutate(value);
    return !ValidateJointState(value);
  };
  EXPECT_TRUE(rejects([](Message & m) { m.set_name(1, ""); }));
  EXPECT_TRUE(rejects([](Message & m) {
    m.set_name(1, std::string(contract_limits::kMaxJointNameLength + 1, 'n'));
  }));
  EXPECT_TRUE(rejects([](Message & m) { m.mutable_position()->RemoveLast(); }));
  EXPECT_TRUE(rejects([](Message & m) { m.set_effort(1, kInf); }));
  EXPECT_TRUE(rejects([](Message & m) {
    while (static_cast<std::size_t>(m.name_size()) <= contract_limits::kMaxJointCount)
      m.add_name("joint_" + std::to_string(m.name_size()));
    m.clear_position();
    m.clear_effort();
  }));
}

// Without planes an ImageLayout uses the qrb_ros_transport aligned layout (128-pixel rows, 32-row
// planes, 256-pixel rgb allocations, 4096-byte allocations); explicit planes describe a native
// layout. Both must fit the payload.
TEST(ContractValidation, ResolvesImageLayoutsAndRejectsInvalidOnes)
{
  ImageLayout layout;
  ASSERT_TRUE(CanonicalImageLayout(1920, 1080, "rgb8", layout));
  EXPECT_EQ(layout.planes[0].stride, 5760u);
  EXPECT_EQ(layout.planes[0].actualHeight, 1088u);
  EXPECT_EQ(layout.storageSize, 2048u * 1088 * 3);
  ASSERT_TRUE(CanonicalImageLayout(1928, 1208, "nv12", layout));
  EXPECT_EQ(layout.planes[0].stride, 2048u);
  EXPECT_EQ(layout.planes[1].offset, 2048u * 1216);
  EXPECT_EQ(layout.planes[1].rows, 604u);
  EXPECT_EQ(layout.storageSize, 2048u * 1216 * 3 / 2);

  // A 4x2 rgb8 image in one explicit, padded native plane.
  using Image = sensor_proto::ImageLayout;
  Image image;
  image.set_width(4);
  image.set_height(2);
  image.set_encoding("rgb8");
  image.set_step(12);
  Image canonical = image;
  ASSERT_TRUE(CanonicalImageLayout(4, 2, "rgb8", layout));
  EXPECT_TRUE(ValidateImageLayout(canonical, layout.storageSize));
  EXPECT_FALSE(ValidateImageLayout(canonical, layout.storageSize - 1));
  image.set_batch_size(1);
  auto * plane = image.add_planes();
  plane->set_stride(16);
  plane->set_actual_height(3);
  plane->set_buffer_size(64);
  ASSERT_TRUE(ValidateImageLayout(image, 64));
  EXPECT_FALSE(ValidateImageLayout(image, 63));

  const auto rejects = [&image](auto mutate) {
    Image value = image;
    mutate(value);
    return !ValidateImageLayout(value, std::numeric_limits<std::uint64_t>::max());
  };
  EXPECT_TRUE(rejects([](Image & i) { i.set_encoding("rgba8"); }));
  EXPECT_TRUE(rejects([](Image & i) { i.set_step(16); }));
  EXPECT_TRUE(rejects([](Image & i) { i.set_data("pixel"); }));
  EXPECT_TRUE(rejects([](Image & i) { i.set_is_bigendian(true); }));
  EXPECT_TRUE(rejects([](Image & i) { i.set_batch_size(2); }));
  EXPECT_TRUE(rejects([](Image & i) { i.add_planes(); }));
  EXPECT_TRUE(rejects([](Image & i) { i.mutable_planes(0)->set_stride(11); }));
  EXPECT_TRUE(rejects([](Image & i) { i.mutable_planes(0)->set_buffer_size(47); }));
  EXPECT_TRUE(rejects([](Image & i) {
    i.set_encoding("nv12");
    i.set_width(3);
    i.set_step(3);
  }));
}

// PoseStamped needs a finite pose with a non-zero quaternion; a missing position and an
// unnormalized quaternion are accepted.
TEST(ContractValidation, RejectsInvalidPoseStamped)
{
  geometry_proto::PoseStamped pose;
  pose.mutable_pose()->mutable_orientation()->set_w(2.0);
  EXPECT_TRUE(ValidatePoseStamped(pose));
  pose.mutable_pose()->mutable_orientation()->set_y(-kInf);
  EXPECT_FALSE(ValidatePoseStamped(pose));
  pose.mutable_pose()->clear_orientation();
  EXPECT_FALSE(ValidatePoseStamped(pose));
  pose.clear_pose();
  EXPECT_FALSE(ValidatePoseStamped(pose));
}

// PoseArray checks every pose as PoseStamped does, holds at most kMaxPoseCount poses, names a
// rejected pose by its index, and a full array of the largest poses fits one IPC payload.
TEST(ContractValidation, RejectsInvalidPoseArrayAndAFullOneFitsOnePayload)
{
  geometry_proto::PoseArray poses;
  EXPECT_TRUE(ValidatePoseArray(poses));
  poses.mutable_header()->mutable_stamp()->set_sec(std::numeric_limits<std::int32_t>::max());
  poses.mutable_header()->mutable_stamp()->set_nanosec(999999999U);
  poses.mutable_header()->set_frame_id(std::string(contract_limits::kMaxFrameIdLength, 'f'));
  // Every field is non-zero since Protobuf leaves a zero out.
  for (std::size_t i = 0; i < contract_limits::kMaxPoseCount; ++i) {
    auto * pose = poses.add_poses();
    pose->mutable_position()->set_x(-1.0);
    pose->mutable_position()->set_y(-2.0);
    pose->mutable_position()->set_z(-3.0);
    pose->mutable_orientation()->set_x(-0.1);
    pose->mutable_orientation()->set_y(-0.2);
    pose->mutable_orientation()->set_z(-0.3);
    pose->mutable_orientation()->set_w(-0.4);
  }
  ASSERT_TRUE(ValidatePoseArray(poses));
  EXPECT_LT(poses.ByteSizeLong(), kMaxPayloadSize);

  geometry_proto::PoseArray tooMany = poses;
  *tooMany.add_poses() = poses.poses(0);
  EXPECT_FALSE(ValidatePoseArray(tooMany));
  poses.mutable_poses(2)->clear_orientation();
  const ConversionResult badPose = ValidatePoseArray(poses);
  EXPECT_FALSE(badPose);
  EXPECT_EQ(badPose.Diagnostic().rfind("poses[2]: ", 0), 0u) << badPose.Diagnostic();
}

// Vector3Stamped needs a present, finite vector; the zero vector is a value like any other.
TEST(ContractValidation, RejectsInvalidVector3Stamped)
{
  geometry_proto::Vector3Stamped vector;
  vector.mutable_vector()->set_z(0.0);
  EXPECT_TRUE(ValidateVector3Stamped(vector));
  vector.mutable_vector()->set_y(kInf);
  EXPECT_FALSE(ValidateVector3Stamped(vector));
  vector.clear_vector();
  EXPECT_FALSE(ValidateVector3Stamped(vector));
}

// A detection array is valid when every number is finite, the strings fit and the optional parts
// are consistent; the check names the detection that failed. An empty array, a box without
// orientation and a hypothesis without pose are valid.
TEST(ContractValidation, AcceptsDetectionArraysAndRejectsInvalidDetections)
{
  using Array2D = vision_proto::Detection2DArray;
  Array2D array;
  EXPECT_TRUE(ValidateDetection2DArray(array));
  for (int i = 0; i < 2; ++i) {
    auto * detection = array.add_detections();
    detection->mutable_bbox()->set_size_x(8.0);
    auto * result = detection->add_results();
    result->mutable_hypothesis()->set_class_id("person");
    result->mutable_pose()->mutable_pose()->mutable_orientation()->set_w(1.0);
  }
  array.mutable_detections(1)->mutable_results(0)->clear_pose();
  EXPECT_TRUE(ValidateDetection2DArray(array));

  const auto rejects = [&array](auto mutate) {
    Array2D value = array;
    mutate(value);
    return !ValidateDetection2DArray(value);
  };
  EXPECT_TRUE(rejects([](Array2D & a) {
    a.mutable_detections(0)->mutable_bbox()->mutable_center()->set_theta(kInf);
  }));
  EXPECT_TRUE(
      rejects([](Array2D & a) { a.mutable_detections(0)->mutable_bbox()->set_size_y(-1.0); }));
  EXPECT_TRUE(rejects([](Array2D & a) {
    a.mutable_detections(0)->set_id(
        std::string(contract_limits::kMaxDetectionStringLength + 1, 'i'));
  }));
  EXPECT_TRUE(
      rejects([](Array2D & a) { a.mutable_detections(0)->set_id(std::string("a\0b", 3)); }));
  EXPECT_TRUE(rejects([](Array2D & a) {
    a.mutable_detections(0)->mutable_results(0)->mutable_hypothesis()->set_score(kNaN);
  }));
  EXPECT_TRUE(rejects(
      [](Array2D & a) { a.mutable_detections(0)->mutable_results(0)->clear_hypothesis(); }));
  EXPECT_TRUE(rejects([](Array2D & a) {
    a.mutable_detections(0)->mutable_results(0)->mutable_pose()->add_covariance(1.0);
  }));
  array.mutable_detections(1)->clear_bbox();
  const ConversionResult second = ValidateDetection2DArray(array);
  EXPECT_FALSE(second);
  EXPECT_NE(second.Diagnostic().find("detection 1"), std::string::npos);

  vision_proto::Detection3DArray array3d;
  array3d.add_detections()->mutable_bbox()->mutable_size()->set_x(4.0);
  EXPECT_TRUE(ValidateDetection3DArray(array3d));
  array3d.mutable_detections(0)->mutable_bbox()->mutable_center()->mutable_orientation()->set_x(
      kInf);
  EXPECT_FALSE(ValidateDetection3DArray(array3d));
  array3d.mutable_detections(0)->mutable_bbox()->clear_center();
  array3d.mutable_detections(0)->mutable_bbox()->mutable_size()->set_z(-0.5);
  EXPECT_FALSE(ValidateDetection3DArray(array3d));
}

// Imu needs a present non-zero orientation, finite angular_velocity and linear_acceleration, and
// 0-or-9-value covariances. Per REP-145, orientation_covariance[0] == -1 marks the orientation
// unknown, and then an all-zero quaternion is valid.
TEST(ContractValidation, RejectsInvalidImu)
{
  using Imu = sensor_proto::Imu;
  Imu imu;
  imu.mutable_header()->set_frame_id("imu_link");
  imu.mutable_orientation()->set_w(1.0);
  imu.mutable_angular_velocity()->set_x(0.1);
  imu.mutable_linear_acceleration()->set_z(-9.8);
  EXPECT_TRUE(ValidateImu(imu));
  Imu rep145 = imu;
  rep145.mutable_orientation()->set_w(0.0);
  for (int i = 0; i < 9; ++i)
    rep145.add_orientation_covariance(i == 0 ? -1.0 : 0.0);
  EXPECT_TRUE(ValidateImu(rep145));

  const auto rejects = [&imu](auto mutate) {
    Imu value = imu;
    mutate(value);
    return !ValidateImu(value);
  };
  EXPECT_TRUE(rejects([](Imu & m) { m.clear_orientation(); }));
  EXPECT_TRUE(rejects([](Imu & m) { m.mutable_orientation()->set_w(0.0); }));
  EXPECT_TRUE(rejects([](Imu & m) { m.mutable_angular_velocity()->set_y(-kInf); }));
  EXPECT_TRUE(rejects([](Imu & m) { m.clear_linear_acceleration(); }));
  EXPECT_TRUE(rejects([](Imu & m) { m.add_angular_velocity_covariance(0); }));
}
