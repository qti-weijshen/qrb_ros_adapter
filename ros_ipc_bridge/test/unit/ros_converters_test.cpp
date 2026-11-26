// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

// Converters are stateless functions over message structures: no ROS Context,
// no node, no DMA heap. A memfd stands in for a DMA-BUF where only fd identity
// and size matter.

#include <gtest/gtest.h>
#include <sys/mman.h>
#include <unistd.h>

#include <cstdint>
#include <lib_mem_dmabuf/dmabuf.hpp>
#include <limits>
#include <memory>
#include <sensor_msgs/msg/point_field.hpp>

#include "qrb_ros_adapter/ros_ipc_bridge/flow_bridge/image_ros_converter.hpp"
#include "qrb_ros_adapter/ros_ipc_bridge/flow_bridge/imu_ros_converter.hpp"
#include "qrb_ros_adapter/ros_ipc_bridge/flow_bridge/joint_state_ros_converter.hpp"
#include "qrb_ros_adapter/ros_ipc_bridge/flow_bridge/odometry_ros_converter.hpp"
#include "qrb_ros_adapter/ros_ipc_bridge/flow_bridge/point_cloud_ros_converter.hpp"
#include "qrb_ros_adapter/ros_ipc_bridge/flow_bridge/pose_array_ros_converter.hpp"
#include "qrb_ros_adapter/ros_ipc_bridge/flow_bridge/pose_ros_converter.hpp"
#include "qrb_ros_adapter/ros_ipc_bridge/flow_bridge/pose_stamped_ros_converter.hpp"
#include "qrb_ros_adapter/ros_ipc_bridge/flow_bridge/tf_message_ros_converter.hpp"
#include "qrb_ros_adapter/ros_ipc_bridge/flow_bridge/twist_ros_converter.hpp"
#include "qrb_ros_adapter/ros_ipc_bridge/flow_bridge/vector3_stamped_ros_converter.hpp"

using namespace qrb::adapter;

namespace sensor_proto = ::qrb::contract::sensor::v1;

namespace
{

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

std::shared_ptr<lib_mem_dmabuf::DmaBuffer> MakeMemfdBuffer(std::size_t size)
{
  const int fd = ::memfd_create("qrb_ros_adapter_converter_test", MFD_CLOEXEC);
  if (fd < 0)
    return nullptr;
  if (::ftruncate(fd, static_cast<off_t>(size)) != 0) {
    ::close(fd);
    return nullptr;
  }
  return std::make_shared<lib_mem_dmabuf::DmaBuffer>(fd, size);
}

template <typename Quaternion>
void SetOrientation(Quaternion & orientation)
{
  orientation.x = 0.1;
  orientation.y = 0.2;
  orientation.z = 0.3;
  orientation.w = 0.9;
}

sensor_msgs::msg::PointField Field(const char * name, std::uint32_t offset, std::uint8_t datatype)
{
  sensor_msgs::msg::PointField field;
  field.name = name;
  field.offset = offset;
  field.datatype = datatype;
  field.count = 1;
  return field;
}

constexpr std::uint32_t kPointStep = 32;
constexpr std::uint32_t kPoints = 4;
constexpr std::uint32_t kCloudBytes = kPointStep * kPoints;

/** @brief Four 32-byte points, each with one field of every PointField datatype. */
PointCloudRosMessage MakeRosCloud(std::size_t bufferSize = kCloudBytes)
{
  using PF = sensor_msgs::msg::PointField;
  // The transport type leaves its scalars uninitialized; an indeterminate
  // bool serializes as a malformed varint.
  auto cloud = PointCloudRosMessage();
  cloud.header.stamp.sec = 100;
  cloud.header.stamp.nanosec = 500;
  cloud.header.frame_id = "lidar";
  cloud.height = 1;
  cloud.width = kPoints;
  cloud.point_step = kPointStep;
  cloud.row_step = kCloudBytes;
  cloud.is_dense = true;
  cloud.fields = {Field("i8", 0, PF::INT8), Field("u8", 1, PF::UINT8), Field("i16", 2, PF::INT16),
      Field("u16", 4, PF::UINT16), Field("i32", 8, PF::INT32), Field("u32", 12, PF::UINT32),
      Field("f32", 16, PF::FLOAT32), Field("f64", 24, PF::FLOAT64)};
  cloud.data_fd = MakeMemfdBuffer(bufferSize);
  return cloud;
}

}  // namespace

// PointCloud2 metadata and every PointField datatype round trip, and the payload binds the
// DMA-BUF over the layout size instead of copying bytes; an invalid layout, a buffer that cannot
// back it, or a received range that does not cover it is rejected.
TEST(PointCloudRosConverter, MapsMetadataBindsTheDmaBufAndRejectsWhatItCannotBack)
{
  const PointCloudRosMessage source = MakeRosCloud(kCloudBytes + 64);
  ASSERT_NE(source.data_fd, nullptr);
  PointCloudRosContract contract;
  RosToContractPayload outbound;
  ASSERT_TRUE(PointCloudRosConversion::RosToContract(source, contract, outbound));
  EXPECT_EQ(contract.fields(7).datatype(), sensor_proto::POINT_FIELD_TYPE_FLOAT64);
  EXPECT_EQ(contract.fields(3).offset(), 4u);
  EXPECT_EQ(outbound.Buffer(), source.data_fd);
  EXPECT_EQ(outbound.Size(), kCloudBytes);

  auto back = PointCloudRosMessage();
  ContractToRosPayload inbound(kCloudBytes);
  ASSERT_TRUE(PointCloudRosConversion::ContractToRos(contract, back, inbound));
  EXPECT_EQ(back.data_fd, nullptr);  // the caller supplies the host storage buffer
  const auto destination = MakeMemfdBuffer(kCloudBytes);
  ASSERT_TRUE(inbound.Assign(destination));
  EXPECT_EQ(back.data_fd, destination);
  PointCloudRosContract again;
  RosToContractPayload rebound;
  ASSERT_TRUE(PointCloudRosConversion::RosToContract(back, again, rebound));
  EXPECT_EQ(again.SerializeAsString(), contract.SerializeAsString());

  PointCloudRosMessage invalid = MakeRosCloud();
  invalid.fields[0].datatype = 0;
  PointCloudRosMessage tooShort = MakeRosCloud(kCloudBytes - 1);
  PointCloudRosMessage withoutBuffer = MakeRosCloud();
  withoutBuffer.data_fd.reset();
  for (const PointCloudRosMessage * rejected : {&invalid, &tooShort, &withoutBuffer}) {
    RosToContractPayload payload;
    EXPECT_FALSE(PointCloudRosConversion::RosToContract(*rejected, contract, payload));
    EXPECT_FALSE(payload.IsBound());
  }
  ContractToRosPayload tooSmall(kCloudBytes - 1);
  EXPECT_FALSE(PointCloudRosConversion::ContractToRos(again, back, tooSmall));
  EXPECT_FALSE(tooSmall.IsBound());
}

// A source payload binds once within offset + length <= capacity; a received payload binds once
// within its range, is assigned only after it is bound, and plans non-empty, in-range,
// non-overlapping copy regions.
TEST(RosFlowPayload, BindsOnceWithinRangeAndPlansValidCopyRegions)
{
  const auto buffer = MakeMemfdBuffer(256);
  ASSERT_NE(buffer, nullptr);
  RosToContractPayload bounded;
  EXPECT_FALSE(bounded.Bind(nullptr, 16));
  EXPECT_FALSE(bounded.Bind(buffer, 129, 128));
  EXPECT_FALSE(bounded.IsBound());
  ASSERT_TRUE(bounded.Bind(buffer, 128, 128));
  EXPECT_EQ(bounded.Offset(), 128u);
  EXPECT_FALSE(bounded.Bind(buffer, 16));
  EXPECT_EQ(bounded.Size(), 128u);

  std::shared_ptr<lib_mem_dmabuf::DmaBuffer> unassigned;
  ContractToRosPayload received(64);
  EXPECT_FALSE(received.Assign(MakeMemfdBuffer(64)));
  EXPECT_FALSE(received.Bind(unassigned, 65));
  ASSERT_TRUE(received.Bind(unassigned, 64));
  EXPECT_FALSE(received.Bind(unassigned, 32));
  EXPECT_FALSE(received.Assign(nullptr));
  EXPECT_EQ(unassigned, nullptr);

  std::shared_ptr<lib_mem_dmabuf::DmaBuffer> unused;
  const PayloadCopyRegion ok{0, 16, 0, 32, 16, 4};
  {
    ContractToRosPayload planned(64);
    EXPECT_FALSE(planned.BindRegions(unused, 128, nullptr, 1));
    EXPECT_FALSE(planned.BindRegions(unused, 0, &ok, 1));
    EXPECT_FALSE(planned.BindRegions(unused, 128, &ok, 0));
    const PayloadCopyRegion rejected[] = {
        {0, 16, 0, 32, 16, 0},   // no rows
        {0, 16, 0, 32, 17, 4},   // row wider than the source stride
        {1, 16, 0, 32, 16, 4},   // past the source range
        {0, 16, 32, 32, 16, 4},  // past the destination allocation
    };
    for (const PayloadCopyRegion & region : rejected)
      EXPECT_FALSE(planned.BindRegions(unused, 128, &region, 1));
    const PayloadCopyRegion overlapping[2] = {ok, {0, 16, 64, 32, 16, 2}};
    EXPECT_FALSE(planned.BindRegions(unused, 128, overlapping, 2));
    EXPECT_FALSE(planned.IsBound());
    EXPECT_EQ(planned.RegionCount(), 0u);
  }
  ContractToRosPayload planned(64);
  ASSERT_TRUE(planned.BindRegions(unused, 128, &ok, 1));
  EXPECT_EQ(planned.Size(), 128u);
  EXPECT_EQ(planned.RegionAt(0).rows, 4u);
  EXPECT_FALSE(planned.BindRegions(unused, 128, &ok, 1));
  EXPECT_FALSE(planned.Bind(unused, 64));
}

// nav_msgs/Odometry round trips with both frames, the pose and the twist with their covariances;
// an invalid value is rejected.
TEST(OdometryRosConverter, MapsEveryFieldBothWaysAndRejectsInvalidValues)
{
  OdometryRosMessage ros;
  ros.header.stamp.sec = 5;
  ros.header.stamp.nanosec = 6;
  ros.header.frame_id = "odom";
  ros.child_frame_id = "base_link";
  ros.pose.pose.position.x = 1.0;
  ros.pose.pose.position.y = 2.0;
  ros.pose.pose.position.z = 3.0;
  SetOrientation(ros.pose.pose.orientation);
  ros.twist.twist.linear.x = 0.3;
  ros.twist.twist.linear.y = 0.4;
  ros.twist.twist.linear.z = 0.5;
  ros.twist.twist.angular.x = -0.1;
  ros.twist.twist.angular.y = -0.2;
  ros.twist.twist.angular.z = -0.3;
  for (std::size_t i = 0; i < 36; ++i) {
    ros.pose.covariance[i] = 0.01 * static_cast<double>(i + 1);
    ros.twist.covariance[i] = -0.01 * static_cast<double>(i + 1);
  }

  OdometryRosContract contract;
  ASSERT_TRUE(OdometryRosConversion::RosToContract(ros, contract));
  EXPECT_EQ(contract.child_frame_id(), "base_link");
  EXPECT_DOUBLE_EQ(contract.pose().covariance(0), 0.01);
  EXPECT_DOUBLE_EQ(contract.twist().covariance(0), -0.01);
  EXPECT_DOUBLE_EQ(contract.twist().twist().linear().y(), 0.4);
  OdometryRosMessage back;
  ASSERT_TRUE(OdometryRosConversion::ContractToRos(contract, back));
  EXPECT_EQ(back, ros);

  ros.twist.twist.angular.x = kNaN;
  EXPECT_FALSE(OdometryRosConversion::RosToContract(ros, contract));
}

// geometry_msgs/PoseWithCovarianceStamped round trips, an empty Contract covariance reaches ROS as
// the zero matrix, and an invalid message or Contract is rejected.
TEST(PoseRosConverter, MapsEveryFieldBothWaysAndRejectsInvalidValues)
{
  PoseRosMessage ros;
  ros.header.stamp.sec = 3;
  ros.header.stamp.nanosec = 4;
  ros.header.frame_id = "map";
  ros.pose.pose.position.x = 2.0;
  ros.pose.pose.position.y = -1.0;
  ros.pose.pose.position.z = 0.5;
  SetOrientation(ros.pose.pose.orientation);
  for (std::size_t i = 0; i < 36; ++i)
    ros.pose.covariance[i] = 0.01 * static_cast<double>(i + 1);

  PoseRosContract contract;
  ASSERT_TRUE(PoseRosConversion::RosToContract(ros, contract));
  EXPECT_EQ(contract.header().stamp().nanosec(), 4u);
  EXPECT_DOUBLE_EQ(contract.pose().pose().position().y(), -1.0);
  EXPECT_DOUBLE_EQ(contract.pose().pose().orientation().x(), 0.1);
  EXPECT_DOUBLE_EQ(contract.pose().covariance(1), 0.02);
  PoseRosMessage back;
  ASSERT_TRUE(PoseRosConversion::ContractToRos(contract, back));
  EXPECT_EQ(back, ros);

  contract.mutable_pose()->clear_covariance();
  ASSERT_TRUE(PoseRosConversion::ContractToRos(contract, back));
  EXPECT_EQ(back.pose.covariance, decltype(back.pose.covariance){});

  contract.mutable_header()->set_frame_id("/map");
  EXPECT_FALSE(PoseRosConversion::ContractToRos(contract, back));
  ros.pose.pose.position.x = kNaN;
  EXPECT_FALSE(PoseRosConversion::RosToContract(ros, contract));
}

// geometry_msgs/TwistStamped round trips; an invalid value is rejected.
TEST(TwistRosConverter, MapsEveryFieldBothWaysAndRejectsInvalidValues)
{
  TwistRosMessage ros;
  ros.header.stamp.sec = 7;
  ros.header.stamp.nanosec = 8;
  ros.header.frame_id = "base_link";
  ros.twist.linear.x = 0.5;
  ros.twist.linear.y = -0.2;
  ros.twist.linear.z = 0.1;
  ros.twist.angular.x = 0.3;
  ros.twist.angular.y = -0.4;
  ros.twist.angular.z = 0.25;

  TwistRosContract contract;
  ASSERT_TRUE(TwistRosConversion::RosToContract(ros, contract));
  EXPECT_DOUBLE_EQ(contract.twist().linear().z(), 0.1);
  EXPECT_DOUBLE_EQ(contract.twist().angular().x(), 0.3);
  TwistRosMessage back;
  ASSERT_TRUE(TwistRosConversion::ContractToRos(contract, back));
  EXPECT_EQ(back, ros);

  ros.twist.linear.z = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(TwistRosConversion::RosToContract(ros, contract));
}

// tf2_msgs/TFMessage round trips every transform and replaces stale ones; an invalid transform is
// rejected.
TEST(TFMessageRosConverter, MapsEveryTransformBothWaysAndRejectsInvalidValues)
{
  TFMessageRosMessage ros;
  ros.transforms.resize(2);
  for (std::size_t i = 0; i < ros.transforms.size(); ++i) {
    auto & transform = ros.transforms[i];
    transform.header.stamp.sec = static_cast<std::int32_t>(10 + i);
    transform.header.stamp.nanosec = static_cast<std::uint32_t>(3 * (i + 1));
    transform.header.frame_id = i == 0 ? "map" : "odom";
    transform.child_frame_id = i == 0 ? "odom" : "base_link";
    transform.transform.translation.x = 0.5 * static_cast<double>(i + 1);
    transform.transform.translation.y = 1.5;
    transform.transform.translation.z = -0.25;
    SetOrientation(transform.transform.rotation);
  }

  TFMessageRosContract contract;
  ASSERT_TRUE(TFMessageRosConversion::RosToContract(ros, contract));
  ASSERT_EQ(contract.transforms_size(), 2);
  EXPECT_EQ(contract.transforms(1).header().frame_id(), "odom");
  EXPECT_EQ(contract.transforms(1).child_frame_id(), "base_link");
  EXPECT_DOUBLE_EQ(contract.transforms(1).transform().translation().x(), 1.0);
  EXPECT_DOUBLE_EQ(contract.transforms(1).transform().rotation().x(), 0.1);
  TFMessageRosMessage back;
  back.transforms.resize(5);  // stale
  ASSERT_TRUE(TFMessageRosConversion::ContractToRos(contract, back));
  EXPECT_EQ(back, ros);

  ros.transforms[1].transform.rotation.w = kNaN;
  EXPECT_FALSE(TFMessageRosConversion::RosToContract(ros, contract));
}

namespace
{

// A 1920x1080 rgb8 image: its logical step and its full aligned allocation, which is neither
// step * height nor pitch * aligned height.
constexpr std::uint32_t kImageWidth = 1920;
constexpr std::uint32_t kImageHeight = 1080;
constexpr std::uint64_t kImageStep = 5760;
constexpr std::uint64_t kImageAllocation = 6684672;

/** @brief A memfd-backed image, value-initialized: the transport type leaves its scalars
 * uninitialized. */
ImageRosMessage MakeRosImage(std::uint32_t width = kImageWidth,
    std::uint32_t height = kImageHeight,
    const char * encoding = "rgb8",
    std::uint64_t bufferSize = kImageAllocation)
{
  auto image = ImageRosMessage();
  image.header.stamp.sec = 7;
  image.header.stamp.nanosec = 123456789;
  image.header.frame_id = "camera";
  image.width = static_cast<int>(width);
  image.height = static_cast<int>(height);
  image.encoding = encoding;
  image.dmabuf = MakeMemfdBuffer(bufferSize);
  return image;
}

}  // namespace

// ROS -> Contract binds the producer's buffer over the canonical aligned layout and leaves planes
// empty; Contract -> ROS turns a native plane layout into a valid-pixel copy plan over the
// canonical destination. Odd NV12 dimensions and a missing source time are rejected.
TEST(ImageRosConverter, MapsMetadataBindsTheDmaBufAndPlansTheCanonicalRepack)
{
  const ImageRosMessage source =
      MakeRosImage(kImageWidth, kImageHeight, "rgb8", kImageAllocation + 4096);
  ASSERT_NE(source.dmabuf, nullptr);
  ImageRosContract contract;
  RosToContractPayload outbound;
  ASSERT_TRUE(ImageRosConversion::RosToContract(source, contract, outbound));
  EXPECT_EQ(contract.header().stamp().nanosec(), 123456789u);
  EXPECT_EQ(contract.width(), kImageWidth);
  EXPECT_EQ(contract.height(), kImageHeight);
  EXPECT_EQ(contract.step(), kImageStep);
  EXPECT_EQ(contract.planes_size(), 0);  // the proto reserves planes for HCM -> ROS
  EXPECT_EQ(outbound.Buffer(), source.dmabuf);
  EXPECT_EQ(outbound.Size(), kImageAllocation);

  // An explicit compact native plane still gets the canonical, padded destination.
  constexpr std::uint32_t kTightBuffer = kImageStep * kImageHeight;
  ImageRosContract compact = contract;
  compact.set_batch_size(1);
  auto * plane = compact.add_planes();
  plane->set_stride(kImageStep);
  plane->set_actual_height(kImageHeight);
  plane->set_buffer_size(kTightBuffer);
  auto back = ImageRosMessage();
  ContractToRosPayload inbound(kTightBuffer);
  ASSERT_TRUE(ImageRosConversion::ContractToRos(compact, back, inbound));
  EXPECT_EQ(inbound.Size(), kImageAllocation);
  ASSERT_EQ(inbound.RegionCount(), 1u);
  EXPECT_EQ(inbound.RegionAt(0).sourceStride, kImageStep);
  EXPECT_EQ(inbound.RegionAt(0).rows, kImageHeight);

  // NV12: the UV plane starts after the canonical Y storage (pitch 128 * aligned 32).
  ImageRosContract nv12;
  RosToContractPayload bound;
  ASSERT_TRUE(ImageRosConversion::RosToContract(MakeRosImage(64, 32, "nv12", 8192), nv12, bound));
  EXPECT_EQ(nv12.step(), 64u);
  ContractToRosPayload nv12Inbound(8192);
  ASSERT_TRUE(ImageRosConversion::ContractToRos(nv12, back, nv12Inbound));
  ASSERT_EQ(nv12Inbound.RegionCount(), 2u);
  EXPECT_EQ(nv12Inbound.RegionAt(1).destinationOffset, 4096u);
  EXPECT_EQ(nv12Inbound.RegionAt(1).rows, 16u);

  RosToContractPayload rejected;
  EXPECT_FALSE(ImageRosConversion::RosToContract(MakeRosImage(65, 32, "nv12"), nv12, rejected));
  EXPECT_FALSE(rejected.IsBound());
  ImageRosContract missingTime = contract;
  missingTime.mutable_header()->clear_stamp();
  ContractToRosPayload unbound(kImageAllocation);
  EXPECT_FALSE(ImageRosConversion::ContractToRos(missingTime, back, unbound));
  EXPECT_FALSE(unbound.IsBound());
}

// sensor_msgs/JointState round trips, and a measurement that is not supplied stays empty; an
// invalid value is rejected.
TEST(JointStateRosConverter, MapsEveryFieldBothWaysAndRejectsMalformedMeasurements)
{
  JointStateRosMessage ros;
  ros.header.stamp.sec = 5;
  ros.header.stamp.nanosec = 6;
  ros.header.frame_id = "stream_probe";
  ros.name = {"joint_0", "joint_1", "joint_2"};
  ros.position = {0.1, 0.2, 0.3};
  ros.velocity = {1.0, 2.0, 3.0};
  ros.effort = {-0.5, -1.5, -2.5};

  JointStateRosContract contract;
  ASSERT_TRUE(JointStateRosConversion::RosToContract(ros, contract));
  EXPECT_EQ(contract.name(2), "joint_2");
  EXPECT_DOUBLE_EQ(contract.position(1), 0.2);
  EXPECT_DOUBLE_EQ(contract.velocity(1), 2.0);
  EXPECT_DOUBLE_EQ(contract.effort(1), -1.5);
  JointStateRosMessage back;
  back.name.resize(5);  // stale
  ASSERT_TRUE(JointStateRosConversion::ContractToRos(contract, back));
  EXPECT_EQ(back, ros);

  JointStateRosMessage positionOnly = ros;
  positionOnly.velocity.clear();
  positionOnly.effort.clear();
  ASSERT_TRUE(JointStateRosConversion::RosToContract(positionOnly, contract));
  EXPECT_EQ(contract.velocity_size(), 0);
  ASSERT_TRUE(JointStateRosConversion::ContractToRos(contract, back));
  EXPECT_EQ(back, positionOnly);

  ros.velocity[1] = kNaN;
  EXPECT_FALSE(JointStateRosConversion::RosToContract(ros, contract));
}

// geometry_msgs/PoseStamped round trips; an invalid value is rejected.
TEST(PoseStampedRosConverter, MapsEveryFieldBothWaysAndRejectsInvalidValues)
{
  PoseStampedRosMessage ros;
  ros.header.stamp.sec = 9;
  ros.header.stamp.nanosec = 10;
  ros.header.frame_id = "base_link";
  ros.pose.position.x = 1.5;
  ros.pose.position.y = -2.0;
  ros.pose.position.z = 0.25;
  SetOrientation(ros.pose.orientation);

  PoseStampedRosContract contract;
  ASSERT_TRUE(PoseStampedRosConversion::RosToContract(ros, contract));
  EXPECT_DOUBLE_EQ(contract.pose().position().z(), 0.25);
  EXPECT_DOUBLE_EQ(contract.pose().orientation().x(), 0.1);
  PoseStampedRosMessage back;
  ASSERT_TRUE(PoseStampedRosConversion::ContractToRos(contract, back));
  EXPECT_EQ(back, ros);

  ros.pose.position.z = kNaN;
  EXPECT_FALSE(PoseStampedRosConversion::RosToContract(ros, contract));
}

// geometry_msgs/PoseArray round trips every pose and replaces stale ones; an invalid pose is
// rejected.
TEST(PoseArrayRosConverter, MapsEveryPoseBothWaysAndRejectsInvalidValues)
{
  PoseArrayRosMessage ros;
  ros.header.stamp.sec = 11;
  ros.header.stamp.nanosec = 12;
  ros.header.frame_id = "base_link";
  ros.poses.resize(3);
  for (std::size_t i = 0; i < ros.poses.size(); ++i) {
    auto & pose = ros.poses[i];
    pose.position.x = 0.5 * static_cast<double>(i + 1);
    pose.position.y = -1.5 * static_cast<double>(i + 1);
    pose.position.z = -0.25;
    SetOrientation(pose.orientation);
  }

  PoseArrayRosContract contract;
  ASSERT_TRUE(PoseArrayRosConversion::RosToContract(ros, contract));
  ASSERT_EQ(contract.poses_size(), 3);
  EXPECT_DOUBLE_EQ(contract.poses(1).position().y(), -3.0);
  EXPECT_DOUBLE_EQ(contract.poses(1).orientation().x(), 0.1);
  PoseArrayRosMessage back;
  back.poses.resize(5);  // stale
  ASSERT_TRUE(PoseArrayRosConversion::ContractToRos(contract, back));
  EXPECT_EQ(back, ros);

  ros.poses[1].position.y = kNaN;
  EXPECT_FALSE(PoseArrayRosConversion::RosToContract(ros, contract));
}

// geometry_msgs/Vector3Stamped round trips, the zero vector held as present; an invalid value is
// rejected.
TEST(Vector3StampedRosConverter, MapsEveryFieldBothWaysAndRejectsInvalidValues)
{
  Vector3StampedRosMessage ros;
  ros.header.stamp.sec = 15;
  ros.header.stamp.nanosec = 16;
  ros.header.frame_id = "base_link";
  ros.vector.x = 0.5;
  ros.vector.y = -0.25;
  ros.vector.z = -1.0;

  Vector3StampedRosContract contract;
  ASSERT_TRUE(Vector3StampedRosConversion::RosToContract(ros, contract));
  EXPECT_DOUBLE_EQ(contract.vector().x(), 0.5);
  EXPECT_DOUBLE_EQ(contract.vector().z(), -1.0);
  Vector3StampedRosMessage back;
  ASSERT_TRUE(Vector3StampedRosConversion::ContractToRos(contract, back));
  EXPECT_EQ(back, ros);

  ros.vector = geometry_msgs::msg::Vector3();
  ASSERT_TRUE(Vector3StampedRosConversion::RosToContract(ros, contract));
  EXPECT_TRUE(contract.has_vector());
  ASSERT_TRUE(Vector3StampedRosConversion::ContractToRos(contract, back));
  EXPECT_EQ(back, ros);

  ros.vector.z = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(Vector3StampedRosConversion::RosToContract(ros, contract));
}

// sensor_msgs/Imu round trips with its three 9-value covariances; an invalid value is rejected.
TEST(ImuRosConverter, MapsEveryFieldBothWaysAndRejectsInvalidValues)
{
  ImuRosMessage ros;
  ros.header.stamp.sec = 3;
  ros.header.stamp.nanosec = 250000000;
  ros.header.frame_id = "imu_link";
  SetOrientation(ros.orientation);
  ros.angular_velocity.x = 0.4;
  ros.angular_velocity.y = -0.5;
  ros.angular_velocity.z = 0.6;
  ros.linear_acceleration.x = 1.1;
  ros.linear_acceleration.y = -1.2;
  ros.linear_acceleration.z = -9.8;
  for (std::size_t i = 0; i < 9; ++i) {
    ros.orientation_covariance[i] = 0.01 * static_cast<double>(i + 1);
    ros.angular_velocity_covariance[i] = -0.01 * static_cast<double>(i + 1);
    ros.linear_acceleration_covariance[i] = 0.1 * static_cast<double>(i + 1);
  }

  ImuRosContract contract;
  ASSERT_TRUE(ImuRosConversion::RosToContract(ros, contract));
  EXPECT_DOUBLE_EQ(contract.orientation().x(), 0.1);
  EXPECT_DOUBLE_EQ(contract.angular_velocity().y(), -0.5);
  EXPECT_DOUBLE_EQ(contract.linear_acceleration().x(), 1.1);
  EXPECT_DOUBLE_EQ(contract.angular_velocity_covariance(0), -0.01);
  ImuRosMessage back;
  ASSERT_TRUE(ImuRosConversion::ContractToRos(contract, back));
  EXPECT_EQ(back, ros);

  ros.angular_velocity.y = kNaN;
  EXPECT_FALSE(ImuRosConversion::RosToContract(ros, contract));
}
