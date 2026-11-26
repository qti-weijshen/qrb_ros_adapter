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

#include "qrb_ros_adapter/ipc_runtime/contract_limits.hpp"
#include "qrb_ros_adapter/ipc_runtime/contract_time.hpp"
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

// PointCloud2 metadata and every PointField datatype map 1:1 and the
// payload context binds the DMA-BUF instead of copying bytes; a layout the
// payload cannot back is rejected, and a payload context binds once within
// offset + length <= capacity.
TEST(PointCloudRosConverter, MapsMetadataBindsTheDmaBufAndRejectsWhatItCannotBack)
{
  const PointCloudRosMessage source = MakeRosCloud(kCloudBytes + 64);
  ASSERT_NE(source.data_fd, nullptr);

  PointCloudRosContract contract;
  RosToContractPayload outbound;
  ASSERT_TRUE(PointCloudRosConversion::RosToContract(source, contract, outbound));
  EXPECT_EQ(contract.header().stamp().sec(), 100);
  EXPECT_EQ(contract.header().stamp().nanosec(), 500u);
  EXPECT_EQ(contract.header().frame_id(), "lidar");
  EXPECT_EQ(contract.row_step(), kCloudBytes);
  ASSERT_EQ(contract.fields_size(), 8);
  EXPECT_EQ(contract.fields(7).datatype(), sensor_proto::POINT_FIELD_TYPE_FLOAT64);
  // The valid range is the layout size, not the buffer capacity.
  EXPECT_EQ(outbound.Buffer(), source.data_fd);
  EXPECT_EQ(outbound.Size(), kCloudBytes);
  EXPECT_EQ(outbound.Offset(), 0u);

  auto back = PointCloudRosMessage();
  ContractToRosPayload inbound(kCloudBytes);
  ASSERT_TRUE(PointCloudRosConversion::ContractToRos(contract, back, inbound));
  EXPECT_EQ(inbound.Size(), kCloudBytes);
  EXPECT_EQ(back.data_fd, nullptr);  // the caller supplies the host storage buffer
  const auto destination = MakeMemfdBuffer(kCloudBytes);
  ASSERT_TRUE(inbound.Assign(destination));
  EXPECT_EQ(back.data_fd, destination);

  EXPECT_EQ(back.header, source.header);
  EXPECT_EQ(back.fields, source.fields);
  EXPECT_EQ(back.width, source.width);
  EXPECT_EQ(back.point_step, source.point_step);
  EXPECT_EQ(back.row_step, source.row_step);
  EXPECT_EQ(back.is_dense, source.is_dense);

  {
    PointCloudRosMessage invalid = MakeRosCloud();
    invalid.fields[0].datatype = 0;  // not a PointField datatype
    RosToContractPayload payload;
    EXPECT_FALSE(PointCloudRosConversion::RosToContract(invalid, contract, payload));
    EXPECT_FALSE(payload.IsBound());
  }
  {
    PointCloudRosMessage tooShort = MakeRosCloud(kCloudBytes - 1);
    RosToContractPayload payload;
    EXPECT_FALSE(PointCloudRosConversion::RosToContract(tooShort, contract, payload));
  }
  {
    PointCloudRosMessage withoutBuffer = MakeRosCloud();
    withoutBuffer.data_fd.reset();
    RosToContractPayload payload;
    EXPECT_FALSE(PointCloudRosConversion::RosToContract(withoutBuffer, contract, payload));
  }

  // HCM -> ROS: the received DMA-BUF range must cover the declared layout.
  RosToContractPayload valid;
  ASSERT_TRUE(PointCloudRosConversion::RosToContract(MakeRosCloud(), contract, valid));
  auto target = PointCloudRosMessage();
  ContractToRosPayload tooSmall(kCloudBytes - 1);
  EXPECT_FALSE(PointCloudRosConversion::ContractToRos(contract, target, tooSmall));
  EXPECT_FALSE(tooSmall.IsBound());

  // Source side: offset + length must fit the producer's buffer.
  const auto buffer = MakeMemfdBuffer(256);
  ASSERT_NE(buffer, nullptr);
  RosToContractPayload bounded;
  EXPECT_FALSE(bounded.Bind(nullptr, 16));
  EXPECT_FALSE(bounded.Bind(buffer, 129, 128));
  EXPECT_FALSE(bounded.IsBound());
  ASSERT_TRUE(bounded.Bind(buffer, 128, 128));
  EXPECT_EQ(bounded.Offset(), 128u);
  EXPECT_FALSE(bounded.Bind(buffer, 16));  // already bound
  EXPECT_EQ(bounded.Size(), 128u);

  // Destination side: the length must fit the received range.
  std::shared_ptr<lib_mem_dmabuf::DmaBuffer> unassigned;
  ContractToRosPayload received(64);
  EXPECT_FALSE(received.Assign(MakeMemfdBuffer(64)));  // nothing bound yet
  EXPECT_FALSE(received.Bind(unassigned, 65));
  ASSERT_TRUE(received.Bind(unassigned, 64));
  EXPECT_FALSE(received.Bind(unassigned, 32));  // already bound
  EXPECT_FALSE(received.Assign(nullptr));
  EXPECT_EQ(unassigned, nullptr);
}

// nav_msgs/Odometry maps 1:1 to the Contract, both frames, pose and
// twist with their covariances; an invalid value is rejected in either direction.
TEST(OdometryRosConverter, MapsEveryFieldBothWaysAndRejectsInvalidValues)
{
  OdometryRosMessage ros;
  ros.header.stamp.sec = 5;
  ros.header.stamp.nanosec = 6;
  ros.header.frame_id = "odom";
  ros.child_frame_id = "base_link";
  ros.pose.pose.position.x = 1.0;
  ros.pose.pose.orientation.w = 1.0;
  ros.twist.twist.linear.x = 0.3;
  ros.twist.twist.angular.z = -0.1;
  for (std::size_t i = 0; i < 36; ++i) {
    ros.pose.covariance[i] = 0.01 * static_cast<double>(i);
    ros.twist.covariance[i] = 0.02 * static_cast<double>(i);
  }

  OdometryRosContract contract;
  ASSERT_TRUE(OdometryRosConversion::RosToContract(ros, contract));
  EXPECT_EQ(contract.header().frame_id(), "odom");
  EXPECT_EQ(contract.child_frame_id(), "base_link");
  EXPECT_EQ(contract.header().stamp().nanosec(), 6u);
  EXPECT_DOUBLE_EQ(contract.twist().twist().linear().x(), 0.3);
  ASSERT_EQ(contract.twist().covariance_size(), 36);
  EXPECT_DOUBLE_EQ(contract.twist().covariance(35), 0.70);

  OdometryRosMessage back;
  ASSERT_TRUE(OdometryRosConversion::ContractToRos(contract, back));
  EXPECT_EQ(back, ros);

  // An empty Contract covariance means unknown; ROS gets the zero matrix.
  contract.mutable_twist()->clear_covariance();
  ASSERT_TRUE(OdometryRosConversion::ContractToRos(contract, back));
  EXPECT_DOUBLE_EQ(back.twist.covariance[35], 0.0);

  OdometryRosMessage invalid = ros;
  invalid.twist.twist.angular.x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(OdometryRosConversion::RosToContract(invalid, contract));
  OdometryRosContract badContract;
  ASSERT_TRUE(OdometryRosConversion::RosToContract(ros, badContract));
  badContract.set_child_frame_id("/base_link");
  EXPECT_FALSE(OdometryRosConversion::ContractToRos(badContract, back));
}

// geometry_msgs/PoseWithCovarianceStamped maps 1:1 to the Contract, the covariance included; an
// invalid value is rejected in either direction.
TEST(PoseRosConverter, MapsEveryFieldBothWaysAndRejectsInvalidValues)
{
  PoseRosMessage ros;
  ros.header.stamp.sec = 3;
  ros.header.stamp.nanosec = 4;
  ros.header.frame_id = "map";
  ros.pose.pose.position.x = 2.0;
  ros.pose.pose.position.y = -1.0;
  ros.pose.pose.orientation.z = 0.6;
  ros.pose.pose.orientation.w = 0.8;
  for (std::size_t i = 0; i < 36; ++i)
    ros.pose.covariance[i] = 0.01 * static_cast<double>(i);

  PoseRosContract contract;
  ASSERT_TRUE(PoseRosConversion::RosToContract(ros, contract));
  EXPECT_EQ(contract.header().frame_id(), "map");
  EXPECT_EQ(contract.header().stamp().nanosec(), 4u);
  EXPECT_DOUBLE_EQ(contract.pose().pose().position().y(), -1.0);
  EXPECT_DOUBLE_EQ(contract.pose().pose().orientation().z(), 0.6);
  ASSERT_EQ(contract.pose().covariance_size(), 36);
  EXPECT_DOUBLE_EQ(contract.pose().covariance(35), 0.35);

  PoseRosMessage back;
  back.header.frame_id = "stale";
  back.pose.pose.position.x = 99.0;
  ASSERT_TRUE(PoseRosConversion::ContractToRos(contract, back));
  EXPECT_EQ(back, ros);

  // An empty Contract covariance means unknown; ROS gets the zero matrix.
  contract.mutable_pose()->clear_covariance();
  ASSERT_TRUE(PoseRosConversion::ContractToRos(contract, back));
  EXPECT_DOUBLE_EQ(back.pose.covariance[35], 0.0);

  PoseRosMessage invalid = ros;
  invalid.pose.pose.position.x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(PoseRosConversion::RosToContract(invalid, contract));
  PoseRosMessage zero = ros;
  zero.pose.pose.orientation.z = 0.0;
  zero.pose.pose.orientation.w = 0.0;
  EXPECT_FALSE(PoseRosConversion::RosToContract(zero, contract));
  PoseRosContract badContract;
  ASSERT_TRUE(PoseRosConversion::RosToContract(ros, badContract));
  badContract.mutable_header()->set_frame_id("/map");
  EXPECT_FALSE(PoseRosConversion::ContractToRos(badContract, back));
}

// geometry_msgs/TwistStamped maps 1:1 to the Contract; an invalid
// value is rejected in either direction.
TEST(TwistRosConverter, MapsEveryFieldBothWaysAndRejectsInvalidValues)
{
  TwistRosMessage ros;
  ros.header.stamp.sec = 7;
  ros.header.stamp.nanosec = 8;
  ros.header.frame_id = "base_link";
  ros.twist.linear.x = 0.5;
  ros.twist.linear.y = -0.2;
  ros.twist.angular.z = 0.25;

  TwistRosContract contract;
  ASSERT_TRUE(TwistRosConversion::RosToContract(ros, contract));
  EXPECT_EQ(contract.header().frame_id(), "base_link");
  EXPECT_EQ(contract.header().stamp().nanosec(), 8u);
  EXPECT_DOUBLE_EQ(contract.twist().linear().y(), -0.2);
  EXPECT_DOUBLE_EQ(contract.twist().angular().z(), 0.25);

  TwistRosMessage back;
  ASSERT_TRUE(TwistRosConversion::ContractToRos(contract, back));
  EXPECT_EQ(back, ros);

  TwistRosMessage invalid = ros;
  invalid.twist.linear.z = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(TwistRosConversion::RosToContract(invalid, contract));
  TwistRosContract badContract;
  ASSERT_TRUE(TwistRosConversion::RosToContract(ros, badContract));
  badContract.mutable_header()->set_frame_id("/base_link");
  EXPECT_FALSE(TwistRosConversion::ContractToRos(badContract, back));
}

// tf2_msgs/TFMessage maps every transform 1:1 to the Contract; an
// invalid transform is rejected in either direction.
TEST(TFMessageRosConverter, MapsEveryTransformBothWaysAndRejectsInvalidValues)
{
  TFMessageRosMessage ros;
  ros.transforms.resize(2);
  for (std::size_t i = 0; i < ros.transforms.size(); ++i) {
    auto & transform = ros.transforms[i];
    transform.header.stamp.sec = static_cast<std::int32_t>(10 + i);
    transform.header.stamp.nanosec = static_cast<std::uint32_t>(3 * i);
    transform.header.frame_id = i == 0 ? "map" : "odom";
    transform.child_frame_id = i == 0 ? "odom" : "base_link";
    transform.transform.translation.x = 0.5 * static_cast<double>(i);
    transform.transform.rotation.z = 0.6;
    transform.transform.rotation.w = 0.8;
  }

  TFMessageRosContract contract;
  ASSERT_TRUE(TFMessageRosConversion::RosToContract(ros, contract));
  ASSERT_EQ(contract.transforms_size(), 2);
  EXPECT_EQ(contract.transforms(1).child_frame_id(), "base_link");
  EXPECT_EQ(contract.transforms(1).header().stamp().nanosec(), 3u);
  EXPECT_DOUBLE_EQ(contract.transforms(1).transform().translation().x(), 0.5);

  TFMessageRosMessage back;
  back.transforms.resize(5);  // stale
  ASSERT_TRUE(TFMessageRosConversion::ContractToRos(contract, back));
  EXPECT_EQ(back, ros);

  TFMessageRosMessage invalid = ros;
  invalid.transforms[1].transform.rotation.w = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(TFMessageRosConversion::RosToContract(invalid, contract));
  TFMessageRosContract badContract;
  ASSERT_TRUE(TFMessageRosConversion::RosToContract(ros, badContract));
  badContract.mutable_transforms(0)->mutable_header()->set_frame_id("/map");
  EXPECT_FALSE(TFMessageRosConversion::ContractToRos(badContract, back));
}

namespace
{

// The four sizes of a 1920x1080 rgb8 image; the converter must pick the right one.
constexpr std::uint32_t kImageWidth = 1920;
constexpr std::uint32_t kImageHeight = 1080;
constexpr std::uint64_t kImageStep = 5760;           // logical: width * 3
constexpr std::uint64_t kImagePixels = 6220800;      // step * height
constexpr std::uint64_t kImagePitchRows = 6266880;   // pitch * aligned height
constexpr std::uint64_t kImageAllocation = 6684672;  // full aligned allocation

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

/** @brief One explicit plane, as an HCM peer describes its native layout. */
void AddPlane(ImageRosContract & image,
    std::uint32_t stride,
    std::uint32_t actualHeight,
    std::uint32_t bufferSize)
{
  auto * plane = image.add_planes();
  plane->set_stride(stride);
  plane->set_actual_height(actualHeight);
  plane->set_buffer_size(bufferSize);
}
}  // namespace

// Image metadata maps to the fields the custom transport type actually has, ROS -> Contract binds
// the producer's buffer over the canonical aligned layout and leaves planes empty per the proto,
// and Contract -> ROS turns an arbitrary native plane layout into a valid-pixel copy plan over the
// independently sized canonical destination.
TEST(ImageRosConverter, MapsMetadataBindsTheDmaBufAndPlansTheCanonicalRepack)
{
  // ROS -> Contract: the bound range is the full allocation, not the logical
  // pixels and not the buffer capacity.
  const ImageRosMessage source =
      MakeRosImage(kImageWidth, kImageHeight, "rgb8", kImageAllocation + 4096);
  ASSERT_NE(source.dmabuf, nullptr);

  ImageRosContract contract;
  RosToContractPayload outbound;
  ASSERT_TRUE(ImageRosConversion::RosToContract(source, contract, outbound));
  EXPECT_EQ(contract.header().stamp().sec(), 7);
  EXPECT_EQ(contract.header().stamp().nanosec(), 123456789u);
  EXPECT_EQ(contract.header().frame_id(), "camera");
  EXPECT_FALSE(contract.is_bigendian());
  EXPECT_EQ(contract.width(), kImageWidth);
  EXPECT_EQ(contract.height(), kImageHeight);
  EXPECT_EQ(contract.encoding(), "rgb8");
  EXPECT_EQ(contract.step(), kImageStep);
  EXPECT_TRUE(contract.data().empty());
  EXPECT_EQ(contract.planes_size(), 0);  // the proto reserves planes for HCM -> ROS
  EXPECT_EQ(contract.batch_size(), 0u);
  EXPECT_EQ(outbound.Buffer(), source.dmabuf);
  EXPECT_EQ(outbound.Size(), kImageAllocation);
  EXPECT_NE(outbound.Size(), kImagePixels);
  EXPECT_NE(outbound.Size(), kImagePitchRows);
  EXPECT_EQ(outbound.Offset(), 0u);

  // Contract -> ROS, explicit compact native plane: the destination
  // allocation stays canonical and is deliberately *not* bounded by the
  // smaller source range.
  {
    constexpr std::uint32_t kTightBuffer = kImageStep * kImageHeight;
    ImageRosContract compact = contract;
    compact.set_batch_size(1);
    AddPlane(compact, kImageStep, kImageHeight, kTightBuffer);

    auto back = ImageRosMessage();
    ContractToRosPayload inbound(kTightBuffer);
    ASSERT_TRUE(ImageRosConversion::ContractToRos(compact, back, inbound));
    EXPECT_EQ(inbound.Size(), kImageAllocation);
    EXPECT_GT(inbound.Size(), inbound.AvailableCapacity());  // compact -> padded
    ASSERT_EQ(inbound.RegionCount(), 1u);
    EXPECT_EQ(inbound.RegionAt(0).sourceStride, kImageStep);
    EXPECT_EQ(inbound.RegionAt(0).rows, kImageHeight);
  }

  // NV12 adds a UV plane whose destination starts at the canonical Y storage
  // and whose last plane absorbs the trailing allocation padding.
  {
    constexpr std::uint32_t kNv12Width = 64;
    constexpr std::uint32_t kNv12Height = 32;
    constexpr std::uint64_t kNv12YStorage = 4096;    // pitch 128 * aligned 32
    constexpr std::uint64_t kNv12Allocation = 8192;  // align4096( Y * 3/2 )
    ImageRosContract nv12;
    RosToContractPayload bound;
    ASSERT_TRUE(ImageRosConversion::RosToContract(
        MakeRosImage(kNv12Width, kNv12Height, "nv12", kNv12Allocation), nv12, bound));
    EXPECT_EQ(nv12.step(), kNv12Width);  // NV12 logical step is width, not width * 3
    EXPECT_EQ(bound.Size(), kNv12Allocation);

    auto back = ImageRosMessage();
    ContractToRosPayload inbound(kNv12Allocation);
    ASSERT_TRUE(ImageRosConversion::ContractToRos(nv12, back, inbound));
    EXPECT_EQ(inbound.Size(), kNv12Allocation);
    ASSERT_EQ(inbound.RegionCount(), 2u);
    EXPECT_EQ(inbound.RegionAt(0).destinationOffset, 0u);
    EXPECT_EQ(inbound.RegionAt(0).rows, kNv12Height);
    EXPECT_EQ(inbound.RegionAt(1).destinationOffset, kNv12YStorage);
    EXPECT_EQ(inbound.RegionAt(1).rows, kNv12Height / 2);
    EXPECT_EQ(inbound.RegionAt(1).rowBytes, kNv12Width);

    // Odd NV12 dimensions would make the UV row count disagree with the
    // external TypeAdapter's data cropping.
    ImageRosContract odd;
    RosToContractPayload rejected;
    EXPECT_FALSE(ImageRosConversion::RosToContract(
        MakeRosImage(kNv12Width + 1, kNv12Height, "nv12"), odd, rejected));
    EXPECT_FALSE(rejected.IsBound());
  }

  // A missing source time must not silently become a zero ROS stamp.
  {
    ImageRosContract missingTime = contract;
    missingTime.mutable_header()->clear_stamp();
    auto back = MakeRosImage();
    const auto previous = back.header;
    ContractToRosPayload inbound(kImageAllocation);
    EXPECT_FALSE(ImageRosConversion::ContractToRos(missingTime, back, inbound));
    EXPECT_FALSE(inbound.IsBound());
    EXPECT_EQ(back.header, previous);
  }

  // The region plan itself: bound once, over non-empty in-range rows whose
  // destination regions do not overlap.
  {
    std::shared_ptr<lib_mem_dmabuf::DmaBuffer> unused;
    const PayloadCopyRegion ok{0, 16, 0, 32, 16, 4};
    {
      ContractToRosPayload planned(64);
      EXPECT_FALSE(planned.BindRegions(unused, 128, nullptr, 1));
      EXPECT_FALSE(planned.BindRegions(unused, 0, &ok, 1));
      EXPECT_FALSE(planned.BindRegions(unused, 128, &ok, 0));
      const PayloadCopyRegion empty{0, 16, 0, 32, 16, 0};
      EXPECT_FALSE(planned.BindRegions(unused, 128, &empty, 1));
      const PayloadCopyRegion wideRow{0, 16, 0, 32, 17, 4};
      EXPECT_FALSE(planned.BindRegions(unused, 128, &wideRow, 1));
      const PayloadCopyRegion pastSource{1, 16, 0, 32, 16, 4};
      EXPECT_FALSE(planned.BindRegions(unused, 128, &pastSource, 1));
      // In range at the source, past the end of the destination allocation.
      const PayloadCopyRegion pastDestination{0, 16, 32, 32, 16, 4};
      EXPECT_FALSE(planned.BindRegions(unused, 128, &pastDestination, 1));
      EXPECT_FALSE(planned.IsBound());
      EXPECT_EQ(planned.RegionCount(), 0u);
    }
    {
      // Overlapping destinations would let one plane overwrite another.
      const PayloadCopyRegion overlapping[2] = {ok, {0, 16, 64, 32, 16, 2}};
      ContractToRosPayload planned(64);
      EXPECT_FALSE(planned.BindRegions(unused, 128, overlapping, 2));
    }
    {
      ContractToRosPayload planned(64);
      ASSERT_TRUE(planned.BindRegions(unused, 128, &ok, 1));
      EXPECT_TRUE(planned.IsBound());
      EXPECT_EQ(planned.Size(), 128u);
      EXPECT_EQ(planned.RegionAt(0).rows, 4u);
      EXPECT_FALSE(planned.BindRegions(unused, 128, &ok, 1));  // already bound
      EXPECT_FALSE(planned.Bind(unused, 64));                  // already bound
    }
  }
}

// sensor_msgs/JointState maps 1:1 to the Contract; an empty measurement array stays empty, and
// malformed array lengths or invalid values are rejected in either direction.
TEST(JointStateRosConverter, MapsEveryFieldBothWaysAndRejectsMalformedMeasurements)
{
  JointStateRosMessage ros;
  ros.header.stamp.sec = 5;
  ros.header.stamp.nanosec = 6;
  ros.header.frame_id = "stream_probe";
  ros.name = {"joint_0", "joint_1", "joint_2"};
  ros.position = {0.1, 0.2, 0.3};
  ros.velocity = {1.0, 2.0, 3.0};
  ros.effort = {0.0, 0.0, 0.0};

  JointStateRosContract contract;
  ASSERT_TRUE(JointStateRosConversion::RosToContract(ros, contract));
  EXPECT_EQ(contract.header().frame_id(), "stream_probe");
  EXPECT_EQ(contract.header().stamp().nanosec(), 6u);
  ASSERT_EQ(contract.name_size(), 3);
  EXPECT_EQ(contract.name(2), "joint_2");
  ASSERT_EQ(contract.position_size(), 3);
  EXPECT_DOUBLE_EQ(contract.position(1), 0.2);
  EXPECT_DOUBLE_EQ(contract.velocity(2), 3.0);
  EXPECT_EQ(contract.effort_size(), 3);

  JointStateRosMessage back;
  back.name.resize(5);  // stale
  ASSERT_TRUE(JointStateRosConversion::ContractToRos(contract, back));
  EXPECT_EQ(back, ros);

  // A measurement that is not supplied stays empty and is never zero-filled.
  JointStateRosMessage positionOnly = ros;
  positionOnly.velocity.clear();
  positionOnly.effort.clear();
  JointStateRosContract partial;
  ASSERT_TRUE(JointStateRosConversion::RosToContract(positionOnly, partial));
  EXPECT_EQ(partial.velocity_size(), 0);
  EXPECT_EQ(partial.effort_size(), 0);
  JointStateRosMessage partialBack;
  ASSERT_TRUE(JointStateRosConversion::ContractToRos(partial, partialBack));
  EXPECT_EQ(partialBack, positionOnly);

  // A measurement array that is neither empty nor as long as name.
  JointStateRosMessage shortPosition = ros;
  shortPosition.position.pop_back();
  EXPECT_FALSE(JointStateRosConversion::RosToContract(shortPosition, contract));
  JointStateRosMessage longEffort = ros;
  longEffort.effort.push_back(0.0);
  EXPECT_FALSE(JointStateRosConversion::RosToContract(longEffort, contract));

  JointStateRosMessage invalid = ros;
  invalid.velocity[1] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(JointStateRosConversion::RosToContract(invalid, contract));
  JointStateRosMessage emptyName = ros;
  emptyName.name[0].clear();
  EXPECT_FALSE(JointStateRosConversion::RosToContract(emptyName, contract));

  JointStateRosContract badContract;
  ASSERT_TRUE(JointStateRosConversion::RosToContract(ros, badContract));
  badContract.mutable_header()->set_frame_id("/stream_probe");
  EXPECT_FALSE(JointStateRosConversion::ContractToRos(badContract, back));
  JointStateRosContract mismatched;
  ASSERT_TRUE(JointStateRosConversion::RosToContract(ros, mismatched));
  mismatched.mutable_position()->RemoveLast();
  EXPECT_FALSE(JointStateRosConversion::ContractToRos(mismatched, back));
}

// geometry_msgs/PoseStamped maps 1:1 to the Contract; an invalid value is rejected in either
// direction.
TEST(PoseStampedRosConverter, MapsEveryFieldBothWaysAndRejectsInvalidValues)
{
  PoseStampedRosMessage ros;
  ros.header.stamp.sec = 9;
  ros.header.stamp.nanosec = 10;
  ros.header.frame_id = "base_link";
  ros.pose.position.x = 1.5;
  ros.pose.position.y = -2.0;
  ros.pose.position.z = 0.25;
  ros.pose.orientation.z = 0.6;
  ros.pose.orientation.w = 0.8;

  PoseStampedRosContract contract;
  ASSERT_TRUE(PoseStampedRosConversion::RosToContract(ros, contract));
  EXPECT_EQ(contract.header().frame_id(), "base_link");
  EXPECT_EQ(contract.header().stamp().sec(), 9);
  EXPECT_EQ(contract.header().stamp().nanosec(), 10u);
  EXPECT_DOUBLE_EQ(contract.pose().position().y(), -2.0);
  EXPECT_DOUBLE_EQ(contract.pose().orientation().z(), 0.6);
  EXPECT_DOUBLE_EQ(contract.pose().orientation().w(), 0.8);

  PoseStampedRosMessage back;
  back.header.frame_id = "stale";
  back.pose.position.x = 99.0;
  ASSERT_TRUE(PoseStampedRosConversion::ContractToRos(contract, back));
  EXPECT_EQ(back, ros);

  PoseStampedRosMessage invalid = ros;
  invalid.pose.position.z = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(PoseStampedRosConversion::RosToContract(invalid, contract));
  PoseStampedRosMessage zero = ros;
  zero.pose.orientation.z = 0.0;
  zero.pose.orientation.w = 0.0;
  EXPECT_FALSE(PoseStampedRosConversion::RosToContract(zero, contract));

  // A rejected Contract leaves the ROS message as it was.
  PoseStampedRosContract badContract;
  ASSERT_TRUE(PoseStampedRosConversion::RosToContract(ros, badContract));
  badContract.mutable_header()->set_frame_id("/base_link");
  EXPECT_FALSE(PoseStampedRosConversion::ContractToRos(badContract, back));
  EXPECT_EQ(back, ros);
  badContract.mutable_header()->set_frame_id("base_link");
  badContract.clear_pose();
  EXPECT_FALSE(PoseStampedRosConversion::ContractToRos(badContract, back));
  EXPECT_EQ(back, ros);
  // A pose with no orientation reads as the zero quaternion.
  badContract.mutable_pose()->mutable_position()->set_x(1.0);
  EXPECT_FALSE(PoseStampedRosConversion::ContractToRos(badContract, back));
  EXPECT_EQ(back, ros);
}

// geometry_msgs/PoseArray maps every pose 1:1 to the Contract, an empty array
// included; an invalid pose, or more poses than a Contract holds, is rejected in either direction.
TEST(PoseArrayRosConverter, MapsEveryPoseBothWaysAndRejectsInvalidValues)
{
  PoseArrayRosMessage ros;
  ros.header.stamp.sec = 11;
  ros.header.stamp.nanosec = 12;
  ros.header.frame_id = "base_link";
  ros.poses.resize(3);
  for (std::size_t i = 0; i < ros.poses.size(); ++i) {
    auto & pose = ros.poses[i];
    pose.position.x = 0.5 * static_cast<double>(i);
    pose.position.z = -0.25;
    pose.orientation.z = 0.6;
    pose.orientation.w = 0.8;
  }

  PoseArrayRosContract contract;
  ASSERT_TRUE(PoseArrayRosConversion::RosToContract(ros, contract));
  ASSERT_EQ(contract.poses_size(), 3);
  EXPECT_EQ(contract.header().frame_id(), "base_link");
  EXPECT_EQ(contract.header().stamp().nanosec(), 12u);
  EXPECT_DOUBLE_EQ(contract.poses(2).position().x(), 1.0);
  EXPECT_DOUBLE_EQ(contract.poses(2).position().z(), -0.25);
  EXPECT_DOUBLE_EQ(contract.poses(2).orientation().w(), 0.8);

  PoseArrayRosMessage back;
  back.header.frame_id = "stale";
  back.poses.resize(5);  // stale
  ASSERT_TRUE(PoseArrayRosConversion::ContractToRos(contract, back));
  EXPECT_EQ(back, ros);

  // An empty array is a result: it keeps its header and replaces the poses of a stale message.
  PoseArrayRosMessage empty;
  empty.header.stamp.sec = 3;
  empty.header.frame_id = "base_link";
  PoseArrayRosContract emptyContract;
  ASSERT_TRUE(PoseArrayRosConversion::RosToContract(empty, emptyContract));
  EXPECT_EQ(emptyContract.poses_size(), 0);
  EXPECT_EQ(emptyContract.header().frame_id(), "base_link");
  ASSERT_TRUE(PoseArrayRosConversion::ContractToRos(emptyContract, back));
  EXPECT_EQ(back, empty);

  PoseArrayRosMessage invalid = ros;
  invalid.poses[1].position.y = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(PoseArrayRosConversion::RosToContract(invalid, contract));
  PoseArrayRosMessage zero = ros;
  zero.poses[2].orientation.z = 0.0;
  zero.poses[2].orientation.w = 0.0;
  EXPECT_FALSE(PoseArrayRosConversion::RosToContract(zero, contract));

  // A rejected Contract leaves the ROS message as it was.
  back = ros;
  PoseArrayRosContract badContract;
  ASSERT_TRUE(PoseArrayRosConversion::RosToContract(ros, badContract));
  badContract.mutable_poses(0)->mutable_orientation()->set_z(0.0);
  badContract.mutable_poses(0)->mutable_orientation()->set_w(0.0);
  EXPECT_FALSE(PoseArrayRosConversion::ContractToRos(badContract, back));
  EXPECT_EQ(back, ros);
  ASSERT_TRUE(PoseArrayRosConversion::RosToContract(ros, badContract));
  badContract.mutable_header()->set_frame_id("/base_link");
  EXPECT_FALSE(PoseArrayRosConversion::ContractToRos(badContract, back));
  EXPECT_EQ(back, ros);

  // As many poses as a Contract may hold go through; one more is rejected both ways.
  PoseArrayRosMessage full;
  full.poses.resize(contract_limits::kMaxPoseCount);
  ASSERT_TRUE(PoseArrayRosConversion::RosToContract(full, contract));
  ASSERT_TRUE(PoseArrayRosConversion::ContractToRos(contract, back));
  EXPECT_TRUE(back == full);
  PoseArrayRosMessage tooMany;
  tooMany.poses.resize(contract_limits::kMaxPoseCount + 1);
  EXPECT_FALSE(PoseArrayRosConversion::RosToContract(tooMany, contract));
  PoseArrayRosContract tooManyContract;
  for (std::size_t i = 0; i <= contract_limits::kMaxPoseCount; ++i)
    tooManyContract.add_poses()->mutable_orientation()->set_w(1.0);
  EXPECT_FALSE(PoseArrayRosConversion::ContractToRos(tooManyContract, back));
}

// geometry_msgs/Vector3Stamped maps 1:1 to the Contract, the zero vector included; an
// invalid value is rejected in either direction.
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
  EXPECT_EQ(contract.header().frame_id(), "base_link");
  EXPECT_EQ(contract.header().stamp().nanosec(), 16u);
  EXPECT_DOUBLE_EQ(contract.vector().x(), 0.5);
  EXPECT_DOUBLE_EQ(contract.vector().y(), -0.25);
  EXPECT_DOUBLE_EQ(contract.vector().z(), -1.0);

  Vector3StampedRosMessage back;
  back.header.frame_id = "stale";
  back.vector.x = 99.0;
  ASSERT_TRUE(Vector3StampedRosConversion::ContractToRos(contract, back));
  EXPECT_EQ(back, ros);

  // The zero vector is a value like any other, and a Contract holds it as present.
  Vector3StampedRosMessage zero = ros;
  zero.vector = geometry_msgs::msg::Vector3();
  Vector3StampedRosContract zeroContract;
  ASSERT_TRUE(Vector3StampedRosConversion::RosToContract(zero, zeroContract));
  EXPECT_TRUE(zeroContract.has_vector());
  ASSERT_TRUE(Vector3StampedRosConversion::ContractToRos(zeroContract, back));
  EXPECT_EQ(back, zero);

  Vector3StampedRosMessage invalid = ros;
  invalid.vector.z = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(Vector3StampedRosConversion::RosToContract(invalid, contract));

  // A rejected Contract leaves the ROS message as it was.
  back = ros;
  Vector3StampedRosContract badContract;
  ASSERT_TRUE(Vector3StampedRosConversion::RosToContract(ros, badContract));
  badContract.mutable_header()->set_frame_id("/base_link");
  EXPECT_FALSE(Vector3StampedRosConversion::ContractToRos(badContract, back));
  EXPECT_EQ(back, ros);
  badContract.mutable_header()->set_frame_id("base_link");
  badContract.clear_vector();
  EXPECT_FALSE(Vector3StampedRosConversion::ContractToRos(badContract, back));
  EXPECT_EQ(back, ros);
}

// sensor_msgs/Imu maps 1:1 to the Contract; 9-value covariances round-trip exactly, an empty
// Contract covariance becomes the zero array, and an invalid value is rejected in either direction.
TEST(ImuRosConverter, MapsEveryFieldBothWaysAndRejectsInvalidValues)
{
  ImuRosMessage ros;
  ros.header.stamp.sec = 3;
  ros.header.stamp.nanosec = 250000000;
  ros.header.frame_id = "imu_link";
  ros.orientation.w = 1.0;
  ros.angular_velocity.x = 0.1;
  ros.angular_velocity.y = -0.2;
  ros.angular_velocity.z = 0.3;
  ros.linear_acceleration.z = -9.8;
  for (std::size_t i = 0; i < 9; ++i) {
    ros.orientation_covariance[i] = 0.01 * static_cast<double>(i);
    ros.angular_velocity_covariance[i] = 0.02 * static_cast<double>(i);
    ros.linear_acceleration_covariance[i] = 0.03 * static_cast<double>(i);
  }

  ImuRosContract contract;
  ASSERT_TRUE(ImuRosConversion::RosToContract(ros, contract));
  EXPECT_EQ(contract.header().frame_id(), "imu_link");
  EXPECT_EQ(contract.header().stamp().nanosec(), 250000000u);
  EXPECT_DOUBLE_EQ(contract.orientation().w(), 1.0);
  EXPECT_DOUBLE_EQ(contract.angular_velocity().x(), 0.1);
  EXPECT_DOUBLE_EQ(contract.linear_acceleration().z(), -9.8);
  ASSERT_EQ(contract.orientation_covariance_size(), 9);
  EXPECT_DOUBLE_EQ(contract.orientation_covariance(8), 0.08);

  ImuRosMessage back;
  ASSERT_TRUE(ImuRosConversion::ContractToRos(contract, back));
  EXPECT_EQ(back, ros);

  // An empty Contract covariance means unknown; ROS gets the zero array.
  contract.clear_orientation_covariance();
  ASSERT_TRUE(ImuRosConversion::ContractToRos(contract, back));
  EXPECT_DOUBLE_EQ(back.orientation_covariance[8], 0.0);

  ImuRosMessage invalid = ros;
  invalid.angular_velocity.y = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(ImuRosConversion::RosToContract(invalid, contract));
  ImuRosContract badContract;
  ASSERT_TRUE(ImuRosConversion::RosToContract(ros, badContract));
  badContract.mutable_header()->set_frame_id("/imu_link");
  EXPECT_FALSE(ImuRosConversion::ContractToRos(badContract, back));
}
