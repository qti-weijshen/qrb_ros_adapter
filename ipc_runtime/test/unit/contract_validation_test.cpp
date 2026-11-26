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

geometry_proto::PoseWithCovarianceStamped MakePose()
{
  geometry_proto::PoseWithCovarianceStamped pose;
  pose.mutable_header()->mutable_stamp()->set_sec(1);
  pose.mutable_header()->set_frame_id("map");
  auto * value = pose.mutable_pose();
  value->mutable_pose()->mutable_position()->set_x(1.0);
  value->mutable_pose()->mutable_orientation()->set_w(1.0);
  return pose;
}

// 2 rows x 4 points of 16 bytes: x/y/z FLOAT32 and 4 UINT8 intensity bytes.
sensor_proto::PointCloud2 MakeCloud()
{
  sensor_proto::PointCloud2 cloud;
  cloud.mutable_header()->set_frame_id("lidar");
  cloud.set_height(2);
  cloud.set_width(4);
  cloud.set_point_step(16);
  cloud.set_row_step(64);
  const char * names[] = {"x", "y", "z"};
  for (std::uint32_t i = 0; i < 3; ++i) {
    auto * field = cloud.add_fields();
    field->set_name(names[i]);
    field->set_offset(4 * i);
    field->set_datatype(sensor_proto::POINT_FIELD_TYPE_FLOAT32);
    field->set_count(1);
  }
  auto * intensity = cloud.add_fields();
  intensity->set_name("intensity");
  intensity->set_offset(12);
  intensity->set_datatype(sensor_proto::POINT_FIELD_TYPE_UINT8);
  intensity->set_count(4);
  return cloud;
}

constexpr std::uint64_t kCloudBytes = 128;

sensor_proto::Imu MakeImu()
{
  sensor_proto::Imu imu;
  imu.mutable_header()->mutable_stamp()->set_sec(1);
  imu.mutable_header()->set_frame_id("imu_link");
  imu.mutable_orientation()->set_w(1.0);
  imu.mutable_angular_velocity()->set_x(0.1);
  imu.mutable_linear_acceleration()->set_z(-9.8);
  return imu;
}

}  // namespace

// PoseWithCovarianceStamped needs a valid header, a finite pose with a non-zero quaternion and 0
// or 36 covariances; a PointCloud2 layout must fit the external payload exactly as declared.
TEST(ContractValidation, RejectsInvalidPosesAndLayoutsThePayloadCannotBack)
{
  geometry_proto::PoseWithCovarianceStamped full = MakePose();
  EXPECT_TRUE(ValidatePoseWithCovarianceStamped(full));
  for (int i = 0; i < 36; ++i)
    full.mutable_pose()->add_covariance(0.1);
  EXPECT_TRUE(ValidatePoseWithCovarianceStamped(full));

  using Pose = geometry_proto::PoseWithCovarianceStamped;
  const struct
  {
    const char * name;
    void (*mutate)(Pose &);
  } cases[] = {
      {"NonFinitePosition",
          [](Pose & p) {
            p.mutable_pose()->mutable_pose()->mutable_position()->set_y(
                std::numeric_limits<double>::quiet_NaN());
          }},
      {"ZeroQuaternion",
          [](Pose & p) { p.mutable_pose()->mutable_pose()->mutable_orientation()->set_w(0.0); }},
      {"PartialCovariance",
          [](Pose & p) {
            for (int i = 0; i < 35; ++i) {
              p.mutable_pose()->add_covariance(0);
            }
          }},
      {"MissingPose", [](Pose & p) { p.clear_pose(); }},
      {"AbsoluteFrameId", [](Pose & p) { p.mutable_header()->set_frame_id("/map"); }},
      {"NanosecOverflow",
          [](Pose & p) { p.mutable_header()->mutable_stamp()->set_nanosec(1000000000U); }},
  };
  for (const auto & c : cases) {
    SCOPED_TRACE(c.name);
    Pose pose = MakePose();
    c.mutate(pose);
    const ConversionResult rejected = ValidatePoseWithCovarianceStamped(pose);
    EXPECT_FALSE(rejected);
    EXPECT_FALSE(rejected.Diagnostic().empty());
  }

  EXPECT_TRUE(ValidatePointCloud2(MakeCloud(), kCloudBytes));
  // A larger DMA-BUF is fine; the layout addresses only its prefix.
  EXPECT_TRUE(ValidatePointCloud2(MakeCloud(), kCloudBytes + 4096));

  using Cloud = sensor_proto::PointCloud2;
  const struct
  {
    const char * name;
    void (*mutate)(Cloud &);
    std::uint64_t payloadBytes;
  } clouds[] = {
      {"PayloadTooSmall", [](Cloud &) {}, kCloudBytes - 1},
      {"NanosecOverflow",
          [](Cloud & c) { c.mutable_header()->mutable_stamp()->set_nanosec(1000000000U); },
          kCloudBytes},
      {"WidthOverflow", [](Cloud & c) { c.set_width(std::numeric_limits<std::uint32_t>::max()); },
          std::numeric_limits<std::uint64_t>::max()},
      {"FieldBeyondPointStep", [](Cloud & c) { c.mutable_fields(3)->set_count(5); }, kCloudBytes},
      {"UnknownDatatype",
          [](Cloud & c) {
            c.mutable_fields(0)->set_datatype(sensor_proto::POINT_FIELD_TYPE_UNSPECIFIED);
          },
          kCloudBytes},
  };
  for (const auto & c : clouds) {
    SCOPED_TRACE(c.name);
    Cloud cloud = MakeCloud();
    c.mutate(cloud);
    const ConversionResult result = ValidatePointCloud2(cloud, c.payloadBytes);
    EXPECT_FALSE(result);
    EXPECT_FALSE(result.Diagnostic().empty());
  }
}

namespace
{

qrb::contract::nav::v1::Odometry MakeOdometry()
{
  qrb::contract::nav::v1::Odometry odom;
  odom.mutable_header()->mutable_stamp()->set_sec(1);
  odom.mutable_header()->set_frame_id("odom");
  odom.set_child_frame_id("base_link");
  odom.mutable_pose()->mutable_pose()->mutable_orientation()->set_w(1.0);
  odom.mutable_twist()->mutable_twist()->mutable_linear()->set_x(0.3);
  return odom;
}

}  // namespace

// Odometry shares the Pose rules for its pose and adds the same
// finite / 36-value rules for its twist and a frame_id check for child_frame_id.
TEST(ContractValidation, RejectsInvalidOdometry)
{
  qrb::contract::nav::v1::Odometry full = MakeOdometry();
  EXPECT_TRUE(ValidateOdometry(full));
  for (int i = 0; i < 36; ++i) {
    full.mutable_pose()->add_covariance(0.1);
    full.mutable_twist()->add_covariance(0.2);
  }
  EXPECT_TRUE(ValidateOdometry(full));
  // An empty child_frame_id is allowed, as an empty frame_id is.
  qrb::contract::nav::v1::Odometry noChild = MakeOdometry();
  noChild.clear_child_frame_id();
  EXPECT_TRUE(ValidateOdometry(noChild));

  using Odom = qrb::contract::nav::v1::Odometry;
  const struct
  {
    const char * name;
    void (*mutate)(Odom &);
  } cases[] = {
      {"MissingPose", [](Odom & o) { o.clear_pose(); }},
      {"MissingTwist", [](Odom & o) { o.clear_twist(); }},
      {"ZeroQuaternion",
          [](Odom & o) { o.mutable_pose()->mutable_pose()->mutable_orientation()->set_w(0.0); }},
      {"NonFiniteLinear",
          [](Odom & o) {
            o.mutable_twist()->mutable_twist()->mutable_linear()->set_y(
                std::numeric_limits<double>::quiet_NaN());
          }},
      {"NonFiniteAngular",
          [](Odom & o) {
            o.mutable_twist()->mutable_twist()->mutable_angular()->set_z(
                std::numeric_limits<double>::infinity());
          }},
      {"PartialTwistCovariance",
          [](Odom & o) {
            for (int i = 0; i < 35; ++i)
              o.mutable_twist()->add_covariance(0);
          }},
      {"NonFiniteTwistCovariance",
          [](Odom & o) {
            for (int i = 0; i < 36; ++i)
              o.mutable_twist()->add_covariance(
                  i == 7 ? std::numeric_limits<double>::infinity() : 0);
          }},
      {"AbsoluteChildFrameId", [](Odom & o) { o.set_child_frame_id("/base_link"); }},
      {"AbsoluteFrameId", [](Odom & o) { o.mutable_header()->set_frame_id("/odom"); }},
      {"NanosecOverflow",
          [](Odom & o) { o.mutable_header()->mutable_stamp()->set_nanosec(1000000000U); }},
  };
  for (const auto & c : cases) {
    SCOPED_TRACE(c.name);
    Odom odom = MakeOdometry();
    c.mutate(odom);
    const ConversionResult rejected = ValidateOdometry(odom);
    EXPECT_FALSE(rejected);
    EXPECT_FALSE(rejected.Diagnostic().empty());
  }
}

// TwistStamped needs a valid header and a present, finite twist.
TEST(ContractValidation, RejectsInvalidTwistStamped)
{
  geometry_proto::TwistStamped twist;
  twist.mutable_header()->set_frame_id("base_link");
  twist.mutable_twist()->mutable_linear()->set_x(0.5);
  EXPECT_TRUE(ValidateTwistStamped(twist));

  using Twist = geometry_proto::TwistStamped;
  const struct
  {
    const char * name;
    void (*mutate)(Twist &);
  } cases[] = {
      {"MissingTwist", [](Twist & t) { t.clear_twist(); }},
      {"NonFiniteLinear",
          [](Twist & t) {
            t.mutable_twist()->mutable_linear()->set_z(std::numeric_limits<double>::quiet_NaN());
          }},
      {"NonFiniteAngular",
          [](Twist & t) {
            t.mutable_twist()->mutable_angular()->set_y(-std::numeric_limits<double>::infinity());
          }},
      {"AbsoluteFrameId", [](Twist & t) { t.mutable_header()->set_frame_id("/base_link"); }},
      {"NanosecOverflow",
          [](Twist & t) { t.mutable_header()->mutable_stamp()->set_nanosec(1000000000U); }},
  };
  for (const auto & c : cases) {
    SCOPED_TRACE(c.name);
    Twist value = twist;
    c.mutate(value);
    const ConversionResult rejected = ValidateTwistStamped(value);
    EXPECT_FALSE(rejected);
    EXPECT_FALSE(rejected.Diagnostic().empty());
  }
}

// TFMessage checks every transform: header, child_frame_id, a finite translation and a finite,
// non-zero rotation; it holds at most kMaxTransformCount transforms, and an empty message is valid.
TEST(ContractValidation, RejectsInvalidTFMessage)
{
  using TF = ::qrb::contract::tf2::v1::TFMessage;
  TF message;
  EXPECT_TRUE(ValidateTFMessage(message));
  for (int i = 0; i < 2; ++i) {
    auto * transform = message.add_transforms();
    transform->mutable_header()->set_frame_id("map");
    transform->set_child_frame_id("odom");
    transform->mutable_transform()->mutable_rotation()->set_w(1.0);
  }
  EXPECT_TRUE(ValidateTFMessage(message));

  const struct
  {
    const char * name;
    void (*mutate)(TF &);
  } cases[] = {
      {"MissingTransform", [](TF & m) { m.mutable_transforms(1)->clear_transform(); }},
      {"ZeroRotation",
          [](TF & m) {
            m.mutable_transforms(1)->mutable_transform()->mutable_rotation()->set_w(0);
          }},
      {"NonFiniteTranslation",
          [](TF & m) {
            m.mutable_transforms(0)->mutable_transform()->mutable_translation()->set_x(
                std::numeric_limits<double>::quiet_NaN());
          }},
      {"NonFiniteRotation",
          [](TF & m) {
            m.mutable_transforms(1)->mutable_transform()->mutable_rotation()->set_y(
                std::numeric_limits<double>::infinity());
          }},
      {"AbsoluteChildFrameId",
          [](TF & m) { m.mutable_transforms(1)->set_child_frame_id("/odom"); }},
      {"AbsoluteFrameId",
          [](TF & m) { m.mutable_transforms(0)->mutable_header()->set_frame_id("/map"); }},
      {"NanosecOverflow",
          [](TF & m) {
            m.mutable_transforms(0)->mutable_header()->mutable_stamp()->set_nanosec(1000000000U);
          }},
      {"TooManyTransforms",
          [](TF & m) {
            const auto first = m.transforms(0);
            while (static_cast<std::size_t>(m.transforms_size()) <=
                   contract_limits::kMaxTransformCount)
              *m.add_transforms() = first;
          }},
  };
  for (const auto & c : cases) {
    SCOPED_TRACE(c.name);
    TF value = message;
    c.mutate(value);
    const ConversionResult rejected = ValidateTFMessage(value);
    EXPECT_FALSE(rejected);
    EXPECT_FALSE(rejected.Diagnostic().empty());
  }
}

// JointState holds at most kMaxJointCount joints with non-empty, bounded names; position, velocity
// and effort are each empty (a missing measurement) or one finite value per name.
TEST(ContractValidation, RejectsInvalidJointStateAndPreservesMissingMeasurements)
{
  using Message = sensor_proto::JointState;
  Message message;
  EXPECT_TRUE(ValidateJointState(message));
  message.mutable_header()->set_frame_id("base_link");
  for (int i = 0; i < 3; ++i) {
    message.add_name("joint_" + std::to_string(i));
    message.add_position(-0.5 * i);
    message.add_velocity(0.0);
    message.add_effort(0.0);
  }
  EXPECT_TRUE(ValidateJointState(message));
  for (unsigned mask = 0; mask < 8; ++mask) {
    Message missing = message;
    if (mask & 1u)
      missing.clear_position();
    if (mask & 2u)
      missing.clear_velocity();
    if (mask & 4u)
      missing.clear_effort();
    EXPECT_TRUE(ValidateJointState(missing)) << mask;
  }

  const struct
  {
    const char * name;
    void (*mutate)(Message &);
  } cases[] = {
      {"EmptyName", [](Message & m) { m.set_name(1, ""); }},
      {"LongName",
          [](Message & m) {
            m.set_name(1, std::string(contract_limits::kMaxJointNameLength + 1, 'n'));
          }},
      {"NulName", [](Message & m) { m.set_name(1, std::string("joint\0x", 7)); }},
      {"PartialPosition", [](Message & m) { m.mutable_position()->RemoveLast(); }},
      {"ExtraVelocity", [](Message & m) { m.add_velocity(0.0); }},
      {"NonFiniteEffort",
          [](Message & m) { m.set_effort(1, std::numeric_limits<double>::infinity()); }},
      {"NonFinitePosition",
          [](Message & m) { m.set_position(1, std::numeric_limits<double>::quiet_NaN()); }},
      {"AbsoluteFrameId", [](Message & m) { m.mutable_header()->set_frame_id("/base_link"); }},
      {"NanosecOverflow",
          [](Message & m) { m.mutable_header()->mutable_stamp()->set_nanosec(1000000000U); }},
      {"TooManyJoints",
          [](Message & m) {
            while (static_cast<std::size_t>(m.name_size()) <= contract_limits::kMaxJointCount) {
              m.add_name("joint_" + std::to_string(m.name_size()));
              m.add_position(0.0);
              m.add_velocity(0.0);
              m.add_effort(0.0);
            }
          }},
  };
  for (const auto & c : cases) {
    SCOPED_TRACE(c.name);
    Message value = message;
    c.mutate(value);
    const ConversionResult rejected = ValidateJointState(value);
    EXPECT_FALSE(rejected);
    EXPECT_FALSE(rejected.Diagnostic().empty());
  }
}

namespace
{

/** @brief A 4x2 rgb8 image in one explicit, padded native plane. */
sensor_proto::ImageLayout MakeImage()
{
  sensor_proto::ImageLayout image;
  image.mutable_header()->mutable_stamp()->set_sec(1);
  image.set_width(4);
  image.set_height(2);
  image.set_encoding("rgb8");
  image.set_step(12);
  image.set_batch_size(1);
  auto * plane = image.add_planes();
  plane->set_stride(16);
  plane->set_actual_height(3);
  plane->set_buffer_size(64);
  return image;
}

geometry_proto::PoseStamped MakePoseStamped()
{
  geometry_proto::PoseStamped pose;
  pose.mutable_header()->mutable_stamp()->set_sec(1);
  pose.mutable_header()->set_frame_id("base_link");
  pose.mutable_pose()->mutable_position()->set_x(1.0);
  pose.mutable_pose()->mutable_orientation()->set_w(1.0);
  return pose;
}

}  // namespace

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

  sensor_proto::ImageLayout image = MakeImage();
  image.clear_planes();
  image.set_batch_size(0);
  ASSERT_TRUE(CanonicalImageLayout(4, 2, "rgb8", layout));
  EXPECT_TRUE(ValidateImageLayout(image, layout.storageSize));
  EXPECT_FALSE(ValidateImageLayout(image, layout.storageSize - 1));
  ASSERT_TRUE(ValidateImageLayout(MakeImage(), 64));
  EXPECT_FALSE(ValidateImageLayout(MakeImage(), 63));

  using Image = sensor_proto::ImageLayout;
  const struct
  {
    const char * name;
    void (*mutate)(Image &);
  } cases[] = {
      {"UnknownEncoding", [](Image & i) { i.set_encoding("rgba8"); }},
      {"PhysicalStep", [](Image & i) { i.set_step(16); }},
      {"EmbeddedData", [](Image & i) { i.set_data("pixel"); }},
      {"AbsoluteFrameId", [](Image & i) { i.mutable_header()->set_frame_id("/camera"); }},
      {"NanosecOverflow",
          [](Image & i) { i.mutable_header()->mutable_stamp()->set_nanosec(1000000000); }},
      {"BigEndian", [](Image & i) { i.set_is_bigendian(true); }},
      {"MultiFrame", [](Image & i) { i.set_batch_size(2); }},
      {"ExtraPlane", [](Image & i) { i.add_planes(); }},
      {"ShortStride", [](Image & i) { i.mutable_planes(0)->set_stride(11); }},
      {"ShortPlane", [](Image & i) { i.mutable_planes(0)->set_buffer_size(47); }},
      {"OddNv12",
          [](Image & i) {
            i.set_encoding("nv12");
            i.set_width(3);
            i.set_step(3);
          }},
  };
  for (const auto & c : cases) {
    SCOPED_TRACE(c.name);
    Image value = MakeImage();
    c.mutate(value);
    EXPECT_FALSE(ValidateImageLayout(value, std::numeric_limits<std::uint64_t>::max()));
  }
}

// PoseStamped needs a valid header and a present pose with a finite position and orientation and
// a quaternion that is not zero; one that is not of unit length is passed on as it is.
TEST(ContractValidation, RejectsInvalidPoseStamped)
{
  const geometry_proto::PoseStamped pose = MakePoseStamped();
  EXPECT_TRUE(ValidatePoseStamped(pose));
  // An empty frame_id is allowed, and so is the longest one.
  geometry_proto::PoseStamped noFrame = pose;
  noFrame.mutable_header()->clear_frame_id();
  EXPECT_TRUE(ValidatePoseStamped(noFrame));
  geometry_proto::PoseStamped longestFrame = pose;
  longestFrame.mutable_header()->set_frame_id(std::string(contract_limits::kMaxFrameIdLength, 'f'));
  EXPECT_TRUE(ValidatePoseStamped(longestFrame));
  // A half turn has w == 0 and is a rotation like any other.
  geometry_proto::PoseStamped halfTurn = pose;
  halfTurn.mutable_pose()->mutable_orientation()->set_x(1.0);
  halfTurn.mutable_pose()->mutable_orientation()->set_w(0.0);
  EXPECT_TRUE(ValidatePoseStamped(halfTurn));
  // A position left out reads as the origin; it is the orientation that must be set.
  geometry_proto::PoseStamped origin = pose;
  origin.mutable_pose()->clear_position();
  EXPECT_TRUE(ValidatePoseStamped(origin));
  geometry_proto::PoseStamped unnormalized = pose;
  unnormalized.mutable_pose()->mutable_orientation()->set_w(2.0);
  EXPECT_TRUE(ValidatePoseStamped(unnormalized));

  // One non-finite component of the seven is enough to reject a pose.
  const double kNaN = std::numeric_limits<double>::quiet_NaN();
  for (auto set : {&geometry_proto::Point::set_x, &geometry_proto::Point::set_y,
           &geometry_proto::Point::set_z}) {
    geometry_proto::PoseStamped value = pose;
    (value.mutable_pose()->mutable_position()->*set)(kNaN);
    EXPECT_FALSE(ValidatePoseStamped(value));
  }
  for (auto set : {&geometry_proto::Quaternion::set_x, &geometry_proto::Quaternion::set_y,
           &geometry_proto::Quaternion::set_z, &geometry_proto::Quaternion::set_w}) {
    geometry_proto::PoseStamped value = pose;
    (value.mutable_pose()->mutable_orientation()->*set)(kNaN);
    EXPECT_FALSE(ValidatePoseStamped(value));
  }

  using Stamped = geometry_proto::PoseStamped;
  const struct
  {
    const char * name;
    void (*mutate)(Stamped &);
  } cases[] = {
      {"MissingPose", [](Stamped & p) { p.clear_pose(); }},
      {"MissingOrientation", [](Stamped & p) { p.mutable_pose()->clear_orientation(); }},
      {"NonFinitePosition",
          [](Stamped & p) {
            p.mutable_pose()->mutable_position()->set_z(std::numeric_limits<double>::quiet_NaN());
          }},
      {"NonFiniteOrientation",
          [](Stamped & p) {
            p.mutable_pose()->mutable_orientation()->set_y(
                -std::numeric_limits<double>::infinity());
          }},
      {"ZeroQuaternion", [](Stamped & p) { p.mutable_pose()->mutable_orientation()->set_w(0.0); }},
      {"AbsoluteFrameId", [](Stamped & p) { p.mutable_header()->set_frame_id("/base_link"); }},
      {"FrameIdTooLong",
          [](Stamped & p) {
            p.mutable_header()->set_frame_id(
                std::string(contract_limits::kMaxFrameIdLength + 1, 'f'));
          }},
      {"NulInFrameId",
          [](Stamped & p) { p.mutable_header()->set_frame_id(std::string("base\0link", 9)); }},
      {"NanosecOverflow",
          [](Stamped & p) { p.mutable_header()->mutable_stamp()->set_nanosec(1000000000U); }},
  };
  for (const auto & c : cases) {
    SCOPED_TRACE(c.name);
    Stamped value = pose;
    c.mutate(value);
    const ConversionResult rejected = ValidatePoseStamped(value);
    EXPECT_FALSE(rejected);
    EXPECT_FALSE(rejected.Diagnostic().empty());
  }
}

namespace
{

geometry_proto::PoseArray MakePoseArray(int count)
{
  geometry_proto::PoseArray poses;
  poses.mutable_header()->mutable_stamp()->set_sec(1);
  poses.mutable_header()->set_frame_id("base_link");
  for (int i = 0; i < count; ++i) {
    auto * pose = poses.add_poses();
    pose->mutable_position()->set_x(0.1 * i);
    pose->mutable_orientation()->set_w(1.0);
  }
  return poses;
}

constexpr int kPoseLimit = static_cast<int>(contract_limits::kMaxPoseCount);

}  // namespace

// PoseArray checks every pose as PoseStamped does its pose, and the header, holds at
// most kMaxPoseCount poses, and is valid when empty: "no pose found" is a result.
TEST(ContractValidation, RejectsInvalidPoseArray)
{
  EXPECT_TRUE(ValidatePoseArray(MakePoseArray(0)));
  EXPECT_TRUE(ValidatePoseArray(MakePoseArray(3)));
  // A half turn has w == 0 and is a rotation like any other.
  geometry_proto::PoseArray halfTurn = MakePoseArray(3);
  halfTurn.mutable_poses(1)->mutable_orientation()->set_x(1.0);
  halfTurn.mutable_poses(1)->mutable_orientation()->set_w(0.0);
  EXPECT_TRUE(ValidatePoseArray(halfTurn));
  // The limit itself is fine; one pose more is not.
  EXPECT_TRUE(ValidatePoseArray(MakePoseArray(kPoseLimit)));
  const ConversionResult tooMany = ValidatePoseArray(MakePoseArray(kPoseLimit + 1));
  EXPECT_FALSE(tooMany);
  EXPECT_FALSE(tooMany.Diagnostic().empty());

  using Poses = geometry_proto::PoseArray;
  const Poses poses = MakePoseArray(3);
  const struct
  {
    const char * name;
    void (*mutate)(Poses &);
  } cases[] = {
      {"NonFinitePosition",
          [](Poses & p) {
            p.mutable_poses(1)->mutable_position()->set_z(-std::numeric_limits<double>::infinity());
          }},
      {"NonFiniteOrientation",
          [](Poses & p) {
            p.mutable_poses(2)->mutable_orientation()->set_y(
                std::numeric_limits<double>::quiet_NaN());
          }},
      {"ZeroQuaternion", [](Poses & p) { p.mutable_poses(0)->mutable_orientation()->set_w(0.0); }},
      {"MissingOrientation", [](Poses & p) { p.mutable_poses(1)->clear_orientation(); }},
      {"AbsoluteFrameId", [](Poses & p) { p.mutable_header()->set_frame_id("/base_link"); }},
      {"FrameIdTooLong",
          [](Poses & p) {
            p.mutable_header()->set_frame_id(
                std::string(contract_limits::kMaxFrameIdLength + 1, 'f'));
          }},
      {"NanosecOverflow",
          [](Poses & p) { p.mutable_header()->mutable_stamp()->set_nanosec(1000000000U); }},
  };
  for (const auto & c : cases) {
    SCOPED_TRACE(c.name);
    Poses value = poses;
    c.mutate(value);
    const ConversionResult rejected = ValidatePoseArray(value);
    EXPECT_FALSE(rejected);
    EXPECT_FALSE(rejected.Diagnostic().empty());
  }

  // A rejected pose is named by its index; a rejected header is not a pose.
  Poses badPose = poses;
  badPose.mutable_poses(2)->mutable_orientation()->set_w(0.0);
  const ConversionResult namedPose = ValidatePoseArray(badPose);
  EXPECT_FALSE(namedPose);
  EXPECT_EQ(namedPose.Diagnostic().rfind("poses[2]: ", 0), 0u) << namedPose.Diagnostic();
  Poses badHeader = poses;
  badHeader.mutable_header()->set_frame_id("/base_link");
  const ConversionResult header = ValidatePoseArray(badHeader);
  EXPECT_FALSE(header);
  EXPECT_EQ(header.Diagnostic().find("poses["), std::string::npos) << header.Diagnostic();
}

// kMaxPoseCount holds what its comment claims: a pose takes at most 69
// bytes in the array, so that many poses with the longest frame_id, and every field non-zero
// since Protobuf leaves a zero out, take about 68 KiB, within one IPC payload.
TEST(ContractValidation, AFullPoseArrayFitsOneIpcPayload)
{
  geometry_proto::PoseArray poses = MakePoseArray(0);
  poses.mutable_header()->mutable_stamp()->set_sec(std::numeric_limits<std::int32_t>::max());
  poses.mutable_header()->mutable_stamp()->set_nanosec(999999999U);
  poses.mutable_header()->set_frame_id(std::string(contract_limits::kMaxFrameIdLength, 'f'));
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
  ASSERT_EQ(static_cast<std::size_t>(poses.poses_size()), contract_limits::kMaxPoseCount);

  // The tag and length byte of the array entry plus the pose itself.
  EXPECT_LE(poses.poses(0).ByteSizeLong() + 2, 69u);
  EXPECT_LE(poses.ByteSizeLong(), 68u * 1024u);
  EXPECT_LT(poses.ByteSizeLong(), kMaxPayloadSize);
}

namespace
{

geometry_proto::Vector3Stamped MakeVector3Stamped()
{
  geometry_proto::Vector3Stamped vector;
  vector.mutable_header()->mutable_stamp()->set_sec(1);
  vector.mutable_header()->set_frame_id("base_link");
  vector.mutable_vector()->set_z(-1.0);
  return vector;
}

}  // namespace

// Vector3Stamped needs a valid header and a present, finite vector; the zero vector
// is a value like any other.
TEST(ContractValidation, RejectsInvalidVector3Stamped)
{
  const geometry_proto::Vector3Stamped vector = MakeVector3Stamped();
  EXPECT_TRUE(ValidateVector3Stamped(vector));
  geometry_proto::Vector3Stamped zero = vector;
  zero.mutable_vector()->set_z(0.0);
  EXPECT_TRUE(zero.has_vector());
  EXPECT_TRUE(ValidateVector3Stamped(zero));

  using Vector = geometry_proto::Vector3Stamped;
  const struct
  {
    const char * name;
    void (*mutate)(Vector &);
  } cases[] = {
      {"MissingVector", [](Vector & v) { v.clear_vector(); }},
      {"NonFiniteX",
          [](Vector & v) { v.mutable_vector()->set_x(std::numeric_limits<double>::quiet_NaN()); }},
      {"NonFiniteY",
          [](Vector & v) { v.mutable_vector()->set_y(std::numeric_limits<double>::infinity()); }},
      {"NonFiniteZ",
          [](Vector & v) { v.mutable_vector()->set_z(-std::numeric_limits<double>::infinity()); }},
      {"AbsoluteFrameId", [](Vector & v) { v.mutable_header()->set_frame_id("/base_link"); }},
      {"NanosecOverflow",
          [](Vector & v) { v.mutable_header()->mutable_stamp()->set_nanosec(1000000000U); }},
  };
  for (const auto & c : cases) {
    SCOPED_TRACE(c.name);
    Vector value = vector;
    c.mutate(value);
    const ConversionResult rejected = ValidateVector3Stamped(value);
    EXPECT_FALSE(rejected);
    EXPECT_FALSE(rejected.Diagnostic().empty());
  }
}

namespace
{

void FillHypothesis(vision_proto::ObjectHypothesisWithPose * result,
    const char * classId,
    double score,
    bool withCovariance)
{
  result->mutable_hypothesis()->set_class_id(classId);
  result->mutable_hypothesis()->set_score(score);
  result->mutable_pose()->mutable_pose()->mutable_orientation()->set_w(1.0);
  if (withCovariance) {
    for (int i = 0; i < 36; ++i)
      result->mutable_pose()->add_covariance(0.5 * i);
  }
}

vision_proto::Detection2DArray MakeDetections2D()
{
  vision_proto::Detection2DArray array;
  array.mutable_header()->mutable_stamp()->set_sec(3);
  array.mutable_header()->set_frame_id("camera");
  auto * first = array.add_detections();
  first->set_id("track-1");
  first->mutable_bbox()->mutable_center()->mutable_position()->set_x(320.5);
  first->mutable_bbox()->mutable_center()->mutable_position()->set_y(240.0);
  first->mutable_bbox()->mutable_center()->set_theta(0.25);
  first->mutable_bbox()->set_size_x(64.0);
  first->mutable_bbox()->set_size_y(32.0);
  FillHypothesis(first->add_results(), "person", 0.9, false);
  FillHypothesis(first->add_results(), "mannequin", 0.1, true);
  auto * second = array.add_detections();
  second->mutable_bbox()->set_size_x(8.0);
  FillHypothesis(second->add_results(), "cup", 0.5, false);
  return array;
}

vision_proto::Detection3DArray MakeDetections3D()
{
  vision_proto::Detection3DArray array;
  array.mutable_header()->set_frame_id("lidar");
  auto * detection = array.add_detections();
  detection->set_id("car-7");
  detection->mutable_bbox()->mutable_center()->mutable_position()->set_z(1.0);
  detection->mutable_bbox()->mutable_center()->mutable_orientation()->set_w(1.0);
  detection->mutable_bbox()->mutable_size()->set_x(4.0);
  detection->mutable_bbox()->mutable_size()->set_y(2.0);
  detection->mutable_bbox()->mutable_size()->set_z(1.5);
  FillHypothesis(detection->add_results(), "car", 0.8, true);
  return array;
}

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();

}  // namespace

// A detection array is valid when every number is finite, the strings fit and the
// optional parts are consistent; the check names the detection that failed. An empty array
// ("nothing detected") and a box without orientation are valid.
TEST(ContractValidation, AcceptsDetectionArraysAndRejectsInvalidDetections)
{
  EXPECT_TRUE(ValidateDetection2DArray(MakeDetections2D()));
  EXPECT_TRUE(ValidateDetection3DArray(MakeDetections3D()));

  vision_proto::Detection2DArray empty2d;
  empty2d.mutable_header()->set_frame_id("camera");
  EXPECT_TRUE(ValidateDetection2DArray(empty2d));
  vision_proto::Detection3DArray empty3d;
  EXPECT_TRUE(ValidateDetection3DArray(empty3d));

  // Publishers that never set the quaternion send all zeros; such a box is passed on as it is.
  vision_proto::Detection3DArray noOrientation = MakeDetections3D();
  noOrientation.mutable_detections(0)->mutable_bbox()->mutable_center()->clear_orientation();
  EXPECT_TRUE(ValidateDetection3DArray(noOrientation));
  // A hypothesis without pose reads as the identity.
  vision_proto::Detection2DArray noPose = MakeDetections2D();
  noPose.mutable_detections(1)->mutable_results(0)->clear_pose();
  EXPECT_TRUE(ValidateDetection2DArray(noPose));

  const auto invalid2d = [](const char * what, auto && mutate) {
    vision_proto::Detection2DArray array = MakeDetections2D();
    mutate(array);
    const ConversionResult result = ValidateDetection2DArray(array);
    EXPECT_FALSE(result) << what;
    EXPECT_FALSE(result.Diagnostic().empty()) << what;
    return result.Diagnostic();
  };
  invalid2d("bad header", [](auto & a) { a.mutable_header()->set_frame_id("/camera"); });
  invalid2d(
      "nanosec", [](auto & a) { a.mutable_header()->mutable_stamp()->set_nanosec(1000000000u); });
  invalid2d("no bbox", [](auto & a) { a.mutable_detections(1)->clear_bbox(); });
  invalid2d("nan center", [](auto & a) {
    a.mutable_detections(0)->mutable_bbox()->mutable_center()->mutable_position()->set_x(kNaN);
  });
  invalid2d("inf theta",
      [](auto & a) { a.mutable_detections(0)->mutable_bbox()->mutable_center()->set_theta(kInf); });
  invalid2d(
      "negative size", [](auto & a) { a.mutable_detections(0)->mutable_bbox()->set_size_y(-1.0); });
  invalid2d(
      "nan size", [](auto & a) { a.mutable_detections(0)->mutable_bbox()->set_size_x(kNaN); });
  invalid2d("long id", [](auto & a) {
    a.mutable_detections(0)->set_id(
        std::string(contract_limits::kMaxDetectionStringLength + 1, 'i'));
  });
  invalid2d("nul id", [](auto & a) { a.mutable_detections(0)->set_id(std::string("a\0b", 3)); });
  invalid2d("long class", [](auto & a) {
    auto * hypothesis = a.mutable_detections(0)->mutable_results(0)->mutable_hypothesis();
    hypothesis->set_class_id(std::string(contract_limits::kMaxDetectionStringLength + 1, 'c'));
  });
  invalid2d("nul class", [](auto & a) {
    auto * hypothesis = a.mutable_detections(0)->mutable_results(0)->mutable_hypothesis();
    hypothesis->set_class_id(std::string("a\0b", 3));
  });
  invalid2d("nan score", [](auto & a) {
    a.mutable_detections(0)->mutable_results(0)->mutable_hypothesis()->set_score(kNaN);
  });
  invalid2d("no hypothesis",
      [](auto & a) { a.mutable_detections(0)->mutable_results(0)->clear_hypothesis(); });
  invalid2d("nan pose", [](auto & a) {
    auto * pose = a.mutable_detections(0)->mutable_results(0)->mutable_pose();
    pose->mutable_pose()->mutable_position()->set_y(kNaN);
  });
  invalid2d("short covariance", [](auto & a) {
    a.mutable_detections(0)->mutable_results(0)->mutable_pose()->add_covariance(1.0);
  });
  invalid2d("nan covariance", [](auto & a) {
    a.mutable_detections(0)->mutable_results(1)->mutable_pose()->set_covariance(3, kNaN);
  });
  // The failure names the detection, so the bad one can be found.
  const std::string second =
      invalid2d("second", [](auto & a) { a.mutable_detections(1)->clear_bbox(); });
  EXPECT_NE(second.find("detection 1"), std::string::npos);

  const auto invalid3d = [](const char * what, auto && mutate) {
    vision_proto::Detection3DArray array = MakeDetections3D();
    mutate(array);
    EXPECT_FALSE(ValidateDetection3DArray(array)) << what;
  };
  invalid3d("no bbox", [](auto & a) { a.mutable_detections(0)->clear_bbox(); });
  invalid3d("nan center", [](auto & a) {
    a.mutable_detections(0)->mutable_bbox()->mutable_center()->mutable_position()->set_x(kNaN);
  });
  invalid3d("inf orientation", [](auto & a) {
    a.mutable_detections(0)->mutable_bbox()->mutable_center()->mutable_orientation()->set_x(kInf);
  });
  invalid3d("negative size",
      [](auto & a) { a.mutable_detections(0)->mutable_bbox()->mutable_size()->set_z(-0.5); });
  invalid3d("nan size",
      [](auto & a) { a.mutable_detections(0)->mutable_bbox()->mutable_size()->set_x(kNaN); });
  invalid3d("bad class", [](auto & a) {
    auto * hypothesis = a.mutable_detections(0)->mutable_results(0)->mutable_hypothesis();
    hypothesis->set_class_id(std::string(contract_limits::kMaxDetectionStringLength + 1, 'c'));
  });
}

// Imu needs a valid header, a present non-zero orientation, finite angular_velocity and
// linear_acceleration, and 0-or-9-value covariances.
TEST(ContractValidation, RejectsInvalidImu)
{
  sensor_proto::Imu full = MakeImu();
  EXPECT_TRUE(ValidateImu(full));
  for (int i = 0; i < 9; ++i) {
    full.add_orientation_covariance(0.1 * i);
    full.add_angular_velocity_covariance(0.2 * i);
    full.add_linear_acceleration_covariance(0.3 * i);
  }
  EXPECT_TRUE(ValidateImu(full));

  using Imu = sensor_proto::Imu;
  const struct
  {
    const char * name;
    void (*mutate)(Imu &);
  } cases[] = {
      {"MissingOrientation", [](Imu & m) { m.clear_orientation(); }},
      {"ZeroQuaternion", [](Imu & m) { m.mutable_orientation()->set_w(0.0); }},
      {"NonFiniteOrientation",
          [](Imu & m) {
            m.mutable_orientation()->set_x(std::numeric_limits<double>::quiet_NaN());
          }},
      {"MissingAngularVelocity", [](Imu & m) { m.clear_angular_velocity(); }},
      {"NonFiniteAngularVelocity",
          [](Imu & m) {
            m.mutable_angular_velocity()->set_y(-std::numeric_limits<double>::infinity());
          }},
      {"MissingLinearAcceleration", [](Imu & m) { m.clear_linear_acceleration(); }},
      {"NonFiniteLinearAcceleration",
          [](Imu & m) {
            m.mutable_linear_acceleration()->set_z(std::numeric_limits<double>::quiet_NaN());
          }},
      {"PartialOrientationCovariance",
          [](Imu & m) {
            for (int i = 0; i < 8; ++i)
              m.add_orientation_covariance(0);
          }},
      {"PartialAngularVelocityCovariance",
          [](Imu & m) {
            for (int i = 0; i < 8; ++i)
              m.add_angular_velocity_covariance(0);
          }},
      {"PartialLinearAccelerationCovariance",
          [](Imu & m) {
            for (int i = 0; i < 8; ++i)
              m.add_linear_acceleration_covariance(0);
          }},
      {"AbsoluteFrameId", [](Imu & m) { m.mutable_header()->set_frame_id("/imu_link"); }},
      {"NanosecOverflow",
          [](Imu & m) { m.mutable_header()->mutable_stamp()->set_nanosec(1000000000U); }},
  };
  for (const auto & c : cases) {
    SCOPED_TRACE(c.name);
    Imu imu = MakeImu();
    c.mutate(imu);
    const ConversionResult rejected = ValidateImu(imu);
    EXPECT_FALSE(rejected);
    EXPECT_FALSE(rejected.Diagnostic().empty());
  }

  // Per REP-145: orientation_covariance[0] == -1 signals that orientation is
  // unknown. An all-zero quaternion is valid in this case; IMUs that only
  // report angular velocity and linear acceleration use this convention.
  {
    Imu rep145 = MakeImu();
    rep145.mutable_orientation()->set_w(0.0);
    for (int i = 0; i < 9; ++i)
      rep145.add_orientation_covariance(i == 0 ? -1.0 : 0.0);
    EXPECT_TRUE(ValidateImu(rep145));
  }
}
