// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "qrb_ros_adapter/hcm_ipc_adapter/flow_client/image_hcm_converter.hpp"
#include "qrb_ros_adapter/hcm_ipc_adapter/flow_client/imu_hcm_converter.hpp"
#include "qrb_ros_adapter/hcm_ipc_adapter/flow_client/joint_state_hcm_converter.hpp"
#include "qrb_ros_adapter/hcm_ipc_adapter/flow_client/odometry_hcm_converter.hpp"
#include "qrb_ros_adapter/hcm_ipc_adapter/flow_client/point_cloud_hcm_converter.hpp"
#include "qrb_ros_adapter/hcm_ipc_adapter/flow_client/pose_array_hcm_converter.hpp"
#include "qrb_ros_adapter/hcm_ipc_adapter/flow_client/pose_hcm_converter.hpp"
#include "qrb_ros_adapter/hcm_ipc_adapter/flow_client/pose_stamped_hcm_converter.hpp"
#include "qrb_ros_adapter/hcm_ipc_adapter/flow_client/tf_message_hcm_converter.hpp"
#include "qrb_ros_adapter/hcm_ipc_adapter/flow_client/twist_hcm_converter.hpp"
#include "qrb_ros_adapter/hcm_ipc_adapter/flow_client/vector3_stamped_hcm_converter.hpp"
#include "qrb_ros_adapter/ipc_runtime/contract_limits.hpp"
#include "qrb_ros_adapter/ipc_runtime/contract_validation.hpp"

using namespace qrb::adapter;

namespace sensor_proto = ::qrb::contract::sensor::v1;

namespace
{

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();

template <typename Quaternion>
void SetOrientation(Quaternion * orientation)
{
  orientation->set_x(0.1);
  orientation->set_y(0.2);
  orientation->set_z(0.3);
  orientation->set_w(0.9);
}

PoseHcmContract MakePose()
{
  PoseHcmContract pose;
  pose.mutable_header()->mutable_stamp()->set_sec(1);
  pose.mutable_header()->mutable_stamp()->set_nanosec(23456789);
  pose.mutable_header()->set_frame_id("map");
  auto * value = pose.mutable_pose();
  value->mutable_pose()->mutable_position()->set_x(1.5);
  value->mutable_pose()->mutable_position()->set_y(-2.0);
  value->mutable_pose()->mutable_position()->set_z(0.25);
  SetOrientation(value->mutable_pose()->mutable_orientation());
  for (int i = 0; i < 36; ++i)
    value->add_covariance(0.01 * (i + 1));
  return pose;
}

}  // namespace

// PoseWithCovarianceStamped round trips; an invalid value, a frame_id longer than
// QRB_FRAME_ID_MAX_LEN or a negative stamp is rejected, and so is a non-finite value or a
// timestampNs ROS cannot hold on the way back.
TEST(HcmConverters, PoseRoundTripsAndRejectsWhatEitherSideCannotHold)
{
  const PoseHcmContract source = MakePose();
  PoseHcmDescriptor descriptor;
  ASSERT_TRUE(ConvertContractToHcm(source, DmaBufView{}, descriptor));
  EXPECT_EQ(descriptor.timestampNs, 1023456789u);
  EXPECT_DOUBLE_EQ(descriptor.poseWithCovariance.pose.position.y, -2.0);
  EXPECT_DOUBLE_EQ(descriptor.poseWithCovariance.pose.orientation.x, 0.1);
  EXPECT_DOUBLE_EQ(descriptor.poseWithCovariance.covariance[1], 0.02);
  PoseHcmContract back;
  ASSERT_TRUE(ConvertHcmToContract(descriptor, back));
  EXPECT_EQ(back.SerializeAsString(), source.SerializeAsString());

  PoseHcmContract invalid = MakePose();
  invalid.mutable_pose()->mutable_pose()->mutable_position()->set_x(kNaN);
  PoseHcmContract longFrame = MakePose();
  longFrame.mutable_header()->set_frame_id(std::string(QRB::Memory::QRB_FRAME_ID_MAX_LEN + 1, 'f'));
  PoseHcmContract negativeStamp = MakePose();
  negativeStamp.mutable_header()->mutable_stamp()->set_sec(-1);
  for (const PoseHcmContract * rejected : {&invalid, &longFrame, &negativeStamp}) {
    PoseHcmDescriptor out;
    EXPECT_FALSE(ConvertContractToHcm(*rejected, DmaBufView{}, out));
  }
  PoseHcmContract longest = MakePose();
  longest.mutable_header()->set_frame_id(std::string(QRB::Memory::QRB_FRAME_ID_MAX_LEN, 'f'));
  PoseHcmDescriptor kept;
  ASSERT_TRUE(ConvertContractToHcm(longest, DmaBufView{}, kept));
  EXPECT_EQ(HcmStringToContract(kept.frameId), longest.header().frame_id());

  PoseHcmDescriptor nonFinite = descriptor;
  nonFinite.poseWithCovariance.covariance[7] = kInf;
  PoseHcmDescriptor tooLate = descriptor;
  tooLate.timestampNs = std::numeric_limits<std::uint64_t>::max();
  for (const PoseHcmDescriptor * rejected : {&nonFinite, &tooLate}) {
    PoseHcmContract out;
    EXPECT_FALSE(ConvertHcmToContract(*rejected, out));
  }
}

namespace
{

// 1 row x 4 points of 16 bytes, x/y/z FLOAT32.
constexpr std::size_t kCloudBytes = 64;

PointCloudHcmContract MakeCloud()
{
  PointCloudHcmContract cloud;
  cloud.mutable_header()->mutable_stamp()->set_sec(2);
  cloud.mutable_header()->mutable_stamp()->set_nanosec(42);
  cloud.mutable_header()->set_frame_id("lidar");
  cloud.set_height(1);
  cloud.set_width(4);
  cloud.set_point_step(16);
  cloud.set_row_step(64);
  cloud.set_is_dense(true);
  const char * names[] = {"x", "y", "z"};
  for (std::uint32_t i = 0; i < 3; ++i) {
    auto * field = cloud.add_fields();
    field->set_name(names[i]);
    field->set_offset(4 * i);
    field->set_datatype(sensor_proto::POINT_FIELD_TYPE_FLOAT32);
    field->set_count(1);
  }
  return cloud;
}

/** @brief A PointCloud descriptor over heap storage; only pBuf, size and offset are read. */
struct HeapCloudDescriptor
{
  explicit HeapCloudDescriptor(std::size_t capacity, std::size_t offset = 0) : storage(capacity, 0)
  {
    descriptor.pBuf = storage.data();
    descriptor.size = capacity;
    descriptor.offset = offset;
  }

  std::vector<std::uint8_t> storage;
  PointCloudHcmDescriptor descriptor;
};

}  // namespace

// PointCloud2 copies exactly the layout size to the descriptor offset and round trips; the host
// buffer, the source payload and the descriptor's fields must hold the cloud, and validSize must
// cover the layout and fit the buffer.
TEST(HcmConverters, PointCloudCopiesThePayloadAndBoundsItOnBothEnds)
{
  std::vector<std::uint8_t> source(kCloudBytes + 32, 0xA5);
  const DmaBufView payload{source.data(), source.size()};
  HeapCloudDescriptor target(256, 16);
  ASSERT_TRUE(ConvertContractToHcm(MakeCloud(), payload, target.descriptor));
  EXPECT_EQ(target.descriptor.validSize, kCloudBytes);
  EXPECT_EQ(target.descriptor.width, 4u);
  EXPECT_EQ(target.descriptor.fields[2].offset, 8u);
  EXPECT_EQ(target.storage[15], 0u);
  EXPECT_EQ(target.storage[16], 0xA5);
  EXPECT_EQ(target.storage[16 + kCloudBytes - 1], 0xA5);
  EXPECT_EQ(target.storage[16 + kCloudBytes], 0u);
  PointCloudHcmContract back;
  ASSERT_TRUE(ConvertHcmToContract(target.descriptor, back));
  EXPECT_EQ(back.SerializeAsString(), MakeCloud().SerializeAsString());

  HeapCloudDescriptor small(kCloudBytes - 1);
  EXPECT_FALSE(ConvertContractToHcm(MakeCloud(), payload, small.descriptor));
  EXPECT_EQ(small.descriptor.validSize, 0u);
  HeapCloudDescriptor other(256);
  EXPECT_FALSE(ConvertContractToHcm(
      MakeCloud(), DmaBufView{source.data(), kCloudBytes - 1}, other.descriptor));
  EXPECT_FALSE(
      ConvertContractToHcm(MakeCloud(), DmaBufView{nullptr, kCloudBytes}, other.descriptor));

  PointCloudHcmContract tooManyFields = MakeCloud();
  tooManyFields.set_point_step(QRB::Memory::QRB_POINT_CLOUD_MAX_FIELDS + 1);
  tooManyFields.set_row_step(tooManyFields.point_step() * tooManyFields.width());
  tooManyFields.clear_fields();
  for (std::uint32_t i = 0; i <= QRB::Memory::QRB_POINT_CLOUD_MAX_FIELDS; ++i) {
    auto * field = tooManyFields.add_fields();
    field->set_name("f" + std::to_string(i));
    field->set_offset(i);
    field->set_datatype(sensor_proto::POINT_FIELD_TYPE_UINT8);
    field->set_count(1);
  }
  PointCloudHcmContract longName = MakeCloud();
  longName.mutable_fields(0)->set_name(
      std::string(QRB::Memory::QRB_POINT_FIELD_NAME_MAX_LEN + 1, 'n'));
  for (const PointCloudHcmContract * rejected : {&tooManyFields, &longName}) {
    HeapCloudDescriptor host(256);
    EXPECT_FALSE(ConvertContractToHcm(*rejected, payload, host.descriptor));
    EXPECT_EQ(host.descriptor.validSize, 0u);
    EXPECT_EQ(host.descriptor.numFields, 0u);
  }

  for (const std::size_t validSize : {kCloudBytes - 1, target.descriptor.size + 1}) {
    target.descriptor.validSize = validSize;
    EXPECT_FALSE(ConvertHcmToContract(target.descriptor, back));
  }
}

// Odometry keeps both frames, the pose and the twist with their covariances, and rejects an
// invalid value.
TEST(HcmConverters, OdometryRoundTripsAndRejectsInvalidValues)
{
  OdometryHcmContract source;
  source.mutable_header()->mutable_stamp()->set_sec(3);
  source.mutable_header()->set_frame_id("odom");
  source.set_child_frame_id("base_link");
  auto * pose = source.mutable_pose();
  pose->mutable_pose()->mutable_position()->set_x(4.0);
  pose->mutable_pose()->mutable_position()->set_y(5.0);
  pose->mutable_pose()->mutable_position()->set_z(6.0);
  SetOrientation(pose->mutable_pose()->mutable_orientation());
  auto * twist = source.mutable_twist()->mutable_twist();
  twist->mutable_linear()->set_x(0.3);
  twist->mutable_linear()->set_y(0.4);
  twist->mutable_linear()->set_z(0.5);
  twist->mutable_angular()->set_x(-0.1);
  twist->mutable_angular()->set_y(-0.2);
  twist->mutable_angular()->set_z(-0.3);
  for (int i = 0; i < 36; ++i) {
    pose->add_covariance(0.01 * (i + 1));
    source.mutable_twist()->add_covariance(-0.01 * (i + 1));
  }
  OdometryHcmDescriptor descriptor;
  ASSERT_TRUE(ConvertContractToHcm(source, DmaBufView{}, descriptor));
  EXPECT_TRUE(descriptor.frameId == "odom");
  EXPECT_TRUE(descriptor.childFrameId == "base_link");
  EXPECT_DOUBLE_EQ(descriptor.poseWithCovariance.covariance[0], 0.01);
  EXPECT_DOUBLE_EQ(descriptor.twistWithCovariance.covariance[0], -0.01);
  OdometryHcmContract back;
  ASSERT_TRUE(ConvertHcmToContract(descriptor, back));
  EXPECT_EQ(back.SerializeAsString(), source.SerializeAsString());

  twist->mutable_linear()->set_x(kNaN);
  EXPECT_FALSE(ConvertContractToHcm(source, DmaBufView{}, descriptor));
}

// TwistStamped keeps its frame, stamp and twist, and rejects an invalid value.
TEST(HcmConverters, TwistRoundTripsAndRejectsInvalidValues)
{
  TwistHcmContract source;
  source.mutable_header()->mutable_stamp()->set_sec(2);
  source.mutable_header()->set_frame_id("base_link");
  source.mutable_twist()->mutable_linear()->set_x(0.5);
  source.mutable_twist()->mutable_linear()->set_y(-0.2);
  source.mutable_twist()->mutable_linear()->set_z(0.1);
  source.mutable_twist()->mutable_angular()->set_x(0.3);
  source.mutable_twist()->mutable_angular()->set_y(-0.4);
  source.mutable_twist()->mutable_angular()->set_z(0.25);
  TwistHcmDescriptor descriptor;
  ASSERT_TRUE(ConvertContractToHcm(source, DmaBufView{}, descriptor));
  EXPECT_DOUBLE_EQ(descriptor.twist.linear.z, 0.1);
  EXPECT_DOUBLE_EQ(descriptor.twist.angular.x, 0.3);
  TwistHcmContract back;
  ASSERT_TRUE(ConvertHcmToContract(descriptor, back));
  EXPECT_EQ(back.SerializeAsString(), source.SerializeAsString());

  source.mutable_twist()->mutable_angular()->set_x(kNaN);
  EXPECT_FALSE(ConvertContractToHcm(source, DmaBufView{}, descriptor));
}

namespace
{

TFMessageHcmContract MakeTFMessage(int count)
{
  TFMessageHcmContract message;
  for (int i = 0; i < count; ++i) {
    auto * transform = message.add_transforms();
    transform->mutable_header()->mutable_stamp()->set_sec(4 + i);
    transform->mutable_header()->mutable_stamp()->set_nanosec(11u * static_cast<unsigned>(i + 1));
    transform->mutable_header()->set_frame_id(i == 0 ? "map" : "odom");
    transform->set_child_frame_id(i == 0 ? "odom" : "base_link");
    transform->mutable_transform()->mutable_translation()->set_x(1.0 + i);
    transform->mutable_transform()->mutable_translation()->set_y(2.0 + i);
    transform->mutable_transform()->mutable_translation()->set_z(-0.25);
    SetOrientation(transform->mutable_transform()->mutable_rotation());
  }
  return message;
}

/** @brief A buffer-array descriptor over heap storage for `capacity` records. */
template <typename Descriptor>
struct HeapArrayDescriptor
{
  explicit HeapArrayDescriptor(std::uint32_t capacity, std::size_t offset = 0)
    : storage(offset + Descriptor::CalcBufferSize(capacity), 0)
  {
    descriptor.pBuf = storage.data();
    descriptor.size = storage.size();
    descriptor.offset = offset;
    descriptor.allocatorType = QC::Memory::QC_MEMORY_ALLOCATOR_HEAP;
  }

  std::vector<std::uint8_t> storage;
  Descriptor descriptor;
};

using HeapTFDescriptor = HeapArrayDescriptor<TFMessageHcmDescriptor>;

}  // namespace

// TFMessage writes every transform into the descriptor's buffer at any offset and reads them back
// unchanged; an empty message needs no buffer. What the buffer cannot hold is rejected and leaves
// it untouched, and so are an unreachable buffer and a transformCount beyond the buffer.
TEST(HcmConverters, TFMessageRoundTripsThroughTheBufferAndRejectsWhatItCannotHold)
{
  const TFMessageHcmContract source = MakeTFMessage(3);
  HeapTFDescriptor heap(3, 5);
  ASSERT_TRUE(ConvertContractToHcm(source, DmaBufView{}, heap.descriptor));
  EXPECT_EQ(heap.descriptor.transformCount, 3u);
  QRB::Memory::TransformStamped_t second{};
  std::memcpy(&second, static_cast<std::uint8_t *>(heap.descriptor.GetDataPtr()) + sizeof(second),
      sizeof(second));
  EXPECT_EQ(second.timestampNs, 5000000022u);
  EXPECT_TRUE(second.frameId == "odom");
  EXPECT_TRUE(second.childFrameId == "base_link");
  EXPECT_DOUBLE_EQ(second.transform.translation.y, 3.0);
  TFMessageHcmContract back;
  ASSERT_TRUE(ConvertHcmToContract(heap.descriptor, back));
  EXPECT_EQ(back.SerializeAsString(), source.SerializeAsString());

  TFMessageHcmDescriptor bufferless;
  ASSERT_TRUE(ConvertContractToHcm(TFMessageHcmContract{}, DmaBufView{}, bufferless));
  ASSERT_TRUE(ConvertHcmToContract(bufferless, back));
  EXPECT_EQ(back.transforms_size(), 0);

  HeapTFDescriptor untouched(2);
  untouched.descriptor.transformCount = 1;
  const std::vector<std::uint8_t> before = untouched.storage;
  TFMessageHcmContract invalid = MakeTFMessage(2);
  invalid.mutable_transforms(1)->mutable_transform()->mutable_rotation()->set_x(kNaN);
  for (const TFMessageHcmContract * rejected : {&source, &std::as_const(invalid)}) {
    EXPECT_FALSE(ConvertContractToHcm(*rejected, DmaBufView{}, untouched.descriptor));
    EXPECT_EQ(untouched.descriptor.transformCount, 1u);
    EXPECT_EQ(untouched.storage, before);
  }

  HeapTFDescriptor foreign(3);
  foreign.descriptor.allocatorType = QC::Memory::QC_MEMORY_ALLOCATOR_LAST;
  EXPECT_FALSE(ConvertContractToHcm(source, DmaBufView{}, foreign.descriptor));
  untouched.descriptor.transformCount = 3;
  EXPECT_FALSE(ConvertHcmToContract(untouched.descriptor, back));
}

// Imu keeps its orientation, angular_velocity and linear_acceleration with their covariances,
// carries an absent covariance as 9 values of -1 both ways, and rejects an invalid value.
TEST(HcmConverters, ImuRoundTripsAndMarksAnAbsentCovarianceUnknown)
{
  ImuHcmContract source;
  source.mutable_header()->mutable_stamp()->set_sec(5);
  source.mutable_header()->set_frame_id("imu_link");
  SetOrientation(source.mutable_orientation());
  source.mutable_angular_velocity()->set_x(0.4);
  source.mutable_angular_velocity()->set_y(-0.5);
  source.mutable_angular_velocity()->set_z(0.6);
  source.mutable_linear_acceleration()->set_x(1.1);
  source.mutable_linear_acceleration()->set_y(-1.2);
  source.mutable_linear_acceleration()->set_z(-9.8);
  for (int i = 0; i < 9; ++i) {
    source.add_orientation_covariance(0.01 * (i + 1));
    source.add_angular_velocity_covariance(-0.01 * (i + 1));
    source.add_linear_acceleration_covariance(0.1 * (i + 1));
  }
  ImuHcmDescriptor descriptor;
  ASSERT_TRUE(ConvertContractToHcm(source, DmaBufView{}, descriptor));
  EXPECT_DOUBLE_EQ(descriptor.orientation.x, 0.1);
  EXPECT_DOUBLE_EQ(descriptor.angular_velocity.y, -0.5);
  EXPECT_DOUBLE_EQ(descriptor.linear_acceleration.x, 1.1);
  EXPECT_DOUBLE_EQ(descriptor.angular_velocity_covariance[0], -0.01);
  ImuHcmContract back;
  ASSERT_TRUE(ConvertHcmToContract(descriptor, back));
  EXPECT_EQ(back.SerializeAsString(), source.SerializeAsString());

  ImuHcmContract noCovariance = source;
  noCovariance.clear_orientation_covariance();
  noCovariance.clear_angular_velocity_covariance();
  noCovariance.clear_linear_acceleration_covariance();
  ImuHcmDescriptor unknown;
  ASSERT_TRUE(ConvertContractToHcm(noCovariance, DmaBufView{}, unknown));
  EXPECT_DOUBLE_EQ(unknown.orientation_covariance[0], -1.0);
  EXPECT_DOUBLE_EQ(unknown.angular_velocity_covariance[4], -1.0);
  EXPECT_DOUBLE_EQ(unknown.linear_acceleration_covariance[8], -1.0);
  ASSERT_TRUE(ConvertHcmToContract(unknown, back));
  ASSERT_EQ(back.linear_acceleration_covariance_size(), 9);
  EXPECT_DOUBLE_EQ(back.linear_acceleration_covariance(8), -1.0);

  source.mutable_orientation()->Clear();
  EXPECT_FALSE(ConvertContractToHcm(source, DmaBufView{}, descriptor));
}

namespace
{

// Fixed clocks: qtimer 1000 s after boot pairs with ROS time 1700000000 s.
constexpr std::uint64_t kSecond = 1000000000ULL;
constexpr std::uint64_t kQtimerNowNs = 1000 * kSecond;
constexpr std::uint64_t kRealtimeNowNs = 1700000000 * kSecond;

HcmClockSample FixedClocks()
{
  return HcmClockSample{kQtimerNowNs, kRealtimeNowNs};
}

/** @brief Restores the clock sampler that was set before it. */
struct ScopedClockSampler
{
  ~ScopedClockSampler() { SetHcmClockSampler(previous); }
  HcmClockSampler previous;
};

/** @brief A Camera frame over heap storage, laid out as the host allocated it. */
struct HeapCameraFrame
{
  HeapCameraFrame(std::uint32_t stride, std::size_t capacity) : storage(capacity, 0x11)
  {
    descriptor.pBuf = storage.data();
    descriptor.size = capacity;
    descriptor.offset = 0;
    descriptor.validSize = capacity;
    descriptor.format = QC::QC_IMAGE_FORMAT_NV12;
    descriptor.width = 64;
    descriptor.height = 32;
    descriptor.batchSize = 1;
    descriptor.numPlanes = 2;
    descriptor.stride[0] = descriptor.stride[1] = stride;
    descriptor.actualHeight[0] = 32;
    descriptor.actualHeight[1] = 16;
    descriptor.planeBufSize[0] = stride * 32;
    descriptor.planeBufSize[1] = stride * 16;
    descriptor.timestamp = 0;
  }

  std::vector<std::uint8_t> storage;
  ImageHcmDescriptor descriptor;
};

}  // namespace

// A 64x32 nv12 ImageLayout from ROS lands in the rows of the host's own Camera frame layout, or
// adopts the aligned ROS layout when the descriptor has none; the planes and the stamp travel
// back, and the camera qtimer converts to and from ROS time.
TEST(HcmConverters, ImageKeepsTheHostLayoutAndConvertsTheQtimer)
{
  const ScopedClockSampler clocks{SetHcmClockSampler(&FixedClocks)};
  ImageHcmContract source;
  source.mutable_header()->mutable_stamp()->set_sec(1700000001);
  source.mutable_header()->mutable_stamp()->set_nanosec(5);
  source.mutable_header()->set_frame_id("camera");
  source.set_width(64);
  source.set_height(32);
  source.set_encoding("nv12");
  source.set_step(64);
  ImageLayout ros;
  ASSERT_TRUE(ResolveImageLayout(source, std::numeric_limits<std::uint64_t>::max(), ros));
  std::vector<std::uint8_t> payload(ros.storageSize, 0);
  for (std::size_t p = 0; p < ros.planeCount; ++p)
    for (std::uint64_t row = 0; row < ros.planes[p].rows; ++row)
      std::memset(payload.data() + ros.planes[p].offset + row * ros.planes[p].stride,
          static_cast<int>(p * 64 + row + 1), ros.planes[p].rowBytes);
  const DmaBufView view{payload.data(), payload.size()};

  HeapCameraFrame host(64, 64 * 48);
  ASSERT_TRUE(ConvertContractToHcm(source, view, host.descriptor));
  EXPECT_EQ(host.descriptor.validSize, 64u * 48);
  EXPECT_EQ(host.descriptor.timestamp, kQtimerNowNs + kSecond + 5);
  EXPECT_EQ(host.storage[0], 1);              // Y row 0
  EXPECT_EQ(host.storage[64 * 31], 32);       // Y row 31
  EXPECT_EQ(host.storage[64 * 32], 65);       // UV row 0, right after the Y plane
  EXPECT_EQ(host.storage[64 * 47 + 63], 80);  // UV row 15

  ImageHcmContract back;
  ASSERT_TRUE(ConvertHcmToContract(host.descriptor, back));
  EXPECT_EQ(back.header().stamp().SerializeAsString(), source.header().stamp().SerializeAsString());
  EXPECT_TRUE(back.header().frame_id().empty());  // Camera frames carry no frame ID
  ASSERT_EQ(back.planes_size(), 2);
  EXPECT_EQ(back.planes(1).stride(), 64u);
  EXPECT_EQ(back.planes(1).buffer_size(), 64u * 16);

  std::vector<std::uint8_t> storage(ros.storageSize, 0);
  ImageHcmDescriptor bare;
  bare.pBuf = storage.data();
  bare.size = storage.size();
  ASSERT_TRUE(ConvertContractToHcm(source, view, bare));
  EXPECT_EQ(bare.stride[0], ros.planes[0].stride);
  EXPECT_EQ(storage, payload);

  // A ROS time before boot has no qtimer value, and a host frame too small is left untouched.
  ImageHcmContract atBoot = source;
  atBoot.mutable_header()->mutable_stamp()->set_sec(1700000000 - 1000);
  atBoot.mutable_header()->mutable_stamp()->set_nanosec(0);
  EXPECT_FALSE(ConvertContractToHcm(atBoot, view, host.descriptor));
  HeapCameraFrame small(64, 64 * 48 - 1);
  EXPECT_FALSE(ConvertContractToHcm(source, view, small.descriptor));
  EXPECT_EQ(small.storage, std::vector<std::uint8_t>(64 * 48 - 1, 0x11));
  // A format ROS does not carry, or a qtimer of 0, does not reach ROS.
  host.descriptor.format = QC::QC_IMAGE_FORMAT_UYVY;
  EXPECT_FALSE(ConvertHcmToContract(host.descriptor, back));
  host.descriptor.format = QC::QC_IMAGE_FORMAT_NV12;
  host.descriptor.timestamp = 0;
  EXPECT_FALSE(ConvertHcmToContract(host.descriptor, back));
}

namespace
{

/** @brief Constructed and aligned host storage for a JointState descriptor. */
struct HeapJointDescriptor
{
  explicit HeapJointDescriptor(std::uint32_t count) : storage(count)
  {
    descriptor.pBuf = storage.data();
    descriptor.size = JointStateHcmDescriptor::CalcBufferSize(count);
    descriptor.allocatorType = QC::Memory::QC_MEMORY_ALLOCATOR_HEAP;
  }

  std::vector<QRB::Memory::JointState_t> storage;
  JointStateHcmDescriptor descriptor;
};

}  // namespace

// JointState keeps the parallel arrays in aligned host records; a missing measurement, which the
// records cannot hold, is rejected and leaves them untouched, and so is a buffer that is too small
// or misaligned.
TEST(HcmConverters, JointStateRoundTripsThroughHostRecordsAndRejectsWhatTheyCannotHold)
{
  JointStateHcmContract source;
  source.mutable_header()->mutable_stamp()->set_sec(9);
  source.mutable_header()->mutable_stamp()->set_nanosec(10);
  source.mutable_header()->set_frame_id("base_link");
  for (int i = 0; i < 3; ++i) {
    source.add_name("joint_" + std::to_string(i));
    source.add_position(-0.5 * (i + 1));
    source.add_velocity(0.25 * (i + 1));
    source.add_effort(0.1 * (i + 1));
  }
  HeapJointDescriptor heap(3);
  ASSERT_TRUE(ConvertContractToHcm(source, DmaBufView{}, heap.descriptor));
  EXPECT_EQ(heap.descriptor.jointCount, 3u);
  EXPECT_DOUBLE_EQ(heap.storage[1].position, -1.0);
  EXPECT_DOUBLE_EQ(heap.storage[1].velocity, 0.5);
  EXPECT_DOUBLE_EQ(heap.storage[1].effort, 0.2);
  JointStateHcmContract back;
  ASSERT_TRUE(ConvertHcmToContract(heap.descriptor, back));
  EXPECT_EQ(back.SerializeAsString(), source.SerializeAsString());

  const auto records = [&heap] {
    return std::string(
        static_cast<const char *>(heap.descriptor.GetDataPtr()), heap.descriptor.validSize);
  };
  const std::string before = records();
  JointStateHcmContract missing = source;
  missing.clear_velocity();
  EXPECT_FALSE(ConvertContractToHcm(missing, DmaBufView{}, heap.descriptor));
  EXPECT_EQ(heap.descriptor.jointCount, 3u);
  EXPECT_EQ(records(), before);

  HeapJointDescriptor small(2);
  EXPECT_FALSE(ConvertContractToHcm(source, DmaBufView{}, small.descriptor));
  EXPECT_EQ(small.descriptor.jointCount, 0u);
  heap.descriptor.offset = 1;
  heap.descriptor.size += 1;
  EXPECT_FALSE(ConvertContractToHcm(source, DmaBufView{}, heap.descriptor));
}

// PoseStamped keeps its stamp and pose and leaves the inherited buffer part to the host; an
// invalid value is rejected.
TEST(HcmConverters, PoseStampedRoundTripsAndRejectsInvalidValues)
{
  PoseStampedHcmContract source;
  source.mutable_header()->mutable_stamp()->set_sec(5);
  source.mutable_header()->set_frame_id("base_link");
  source.mutable_pose()->mutable_position()->set_x(1.5);
  source.mutable_pose()->mutable_position()->set_y(-2.0);
  source.mutable_pose()->mutable_position()->set_z(0.25);
  SetOrientation(source.mutable_pose()->mutable_orientation());
  PoseStampedHcmDescriptor descriptor;
  descriptor.id = 7;
  descriptor.validSize = 11;
  ASSERT_TRUE(ConvertContractToHcm(source, DmaBufView{}, descriptor));
  EXPECT_EQ(descriptor.id, 7u);
  EXPECT_EQ(descriptor.validSize, 11u);
  EXPECT_DOUBLE_EQ(descriptor.pose.position.z, 0.25);
  EXPECT_DOUBLE_EQ(descriptor.pose.orientation.x, 0.1);
  PoseStampedHcmContract back;
  ASSERT_TRUE(ConvertHcmToContract(descriptor, back));
  EXPECT_EQ(back.SerializeAsString(), source.SerializeAsString());

  source.mutable_pose()->mutable_position()->set_y(kNaN);
  EXPECT_FALSE(ConvertContractToHcm(source, DmaBufView{}, descriptor));
}

using HeapPoseArrayDescriptor = HeapArrayDescriptor<PoseArrayHcmDescriptor>;

// PoseArray writes every pose into the descriptor's buffer and reads them back unchanged; more
// poses than the buffer holds are rejected and leave it untouched, and more than a Contract may
// hold are rejected on the way back.
TEST(HcmConverters, PoseArrayRoundTripsThroughTheBufferAndRejectsWhatItCannotHold)
{
  PoseArrayHcmContract source;
  source.mutable_header()->mutable_stamp()->set_sec(6);
  source.mutable_header()->set_frame_id("base_link");
  for (int i = 0; i < 3; ++i) {
    auto * pose = source.add_poses();
    pose->mutable_position()->set_x(1.5 * (i + 1));
    pose->mutable_position()->set_y(-0.5 * (i + 1));
    pose->mutable_position()->set_z(-0.25);
    SetOrientation(pose->mutable_orientation());
  }
  HeapPoseArrayDescriptor heap(3);
  ASSERT_TRUE(ConvertContractToHcm(source, DmaBufView{}, heap.descriptor));
  EXPECT_EQ(heap.descriptor.poseCount, 3u);
  QRB::Memory::Pose_t second{};
  std::memcpy(&second, static_cast<std::uint8_t *>(heap.descriptor.GetDataPtr()) + sizeof(second),
      sizeof(second));
  EXPECT_DOUBLE_EQ(second.position.y, -1.0);
  EXPECT_DOUBLE_EQ(second.orientation.x, 0.1);
  PoseArrayHcmContract back;
  ASSERT_TRUE(ConvertHcmToContract(heap.descriptor, back));
  EXPECT_EQ(back.SerializeAsString(), source.SerializeAsString());

  HeapPoseArrayDescriptor untouched(2);
  untouched.descriptor.poseCount = 1;
  untouched.descriptor.validSize = 9;
  const std::vector<std::uint8_t> before = untouched.storage;
  EXPECT_FALSE(ConvertContractToHcm(source, DmaBufView{}, untouched.descriptor));
  EXPECT_EQ(untouched.descriptor.poseCount, 1u);
  EXPECT_EQ(untouched.descriptor.validSize, 9u);
  EXPECT_EQ(untouched.storage, before);

  const auto limit = static_cast<std::uint32_t>(contract_limits::kMaxPoseCount) + 1;
  HeapPoseArrayDescriptor roomy(limit);
  roomy.descriptor.poseCount = limit;
  EXPECT_FALSE(ConvertHcmToContract(roomy.descriptor, back));
}

// Vector3Stamped keeps its stamp and vector, the zero vector among them, and rejects an invalid
// value.
TEST(HcmConverters, Vector3StampedRoundTripsAndRejectsInvalidValues)
{
  Vector3StampedHcmContract source;
  source.mutable_header()->mutable_stamp()->set_sec(8);
  source.mutable_header()->set_frame_id("base_link");
  source.mutable_vector()->set_x(0.5);
  source.mutable_vector()->set_y(-0.25);
  source.mutable_vector()->set_z(-1.0);
  Vector3StampedHcmDescriptor descriptor;
  ASSERT_TRUE(ConvertContractToHcm(source, DmaBufView{}, descriptor));
  EXPECT_DOUBLE_EQ(descriptor.vector.x, 0.5);
  EXPECT_DOUBLE_EQ(descriptor.vector.z, -1.0);
  Vector3StampedHcmContract back;
  ASSERT_TRUE(ConvertHcmToContract(descriptor, back));
  EXPECT_EQ(back.SerializeAsString(), source.SerializeAsString());

  source.mutable_vector()->Clear();
  ASSERT_TRUE(ConvertContractToHcm(source, DmaBufView{}, descriptor));
  ASSERT_TRUE(ConvertHcmToContract(descriptor, back));
  EXPECT_EQ(back.SerializeAsString(), source.SerializeAsString());

  source.mutable_vector()->set_y(kNaN);
  EXPECT_FALSE(ConvertContractToHcm(source, DmaBufView{}, descriptor));
}
