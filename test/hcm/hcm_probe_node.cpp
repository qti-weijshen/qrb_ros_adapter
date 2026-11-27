// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "hcm_probe_node.hpp"

#define QRB_CONTRACT_MAPPING_INCLUDE_HCM_HEADERS
#include "qrb_ros_adapter/ipc_runtime/contract_mappings.def"
#undef QRB_CONTRACT_MAPPING_INCLUDE_HCM_HEADERS

#include <algorithm>
#include <array>
#include <atomic>
#include <cstring>
#include <limits>
#include <mutex>
#include <nlohmann/json.hpp>
#include <string>
#include <utility>

#include "QC/Infras/Memory/BufferDescriptor.hpp"
#include "QC/Node/Ifs/QCNodeConfigIfs.hpp"
#include "QC/Node/NodeBase.hpp"
#include "qrb_ros_adapter/hcm_ipc_adapter/flow_client/hcm_dma_buffer.hpp"
#include "qrb_ros_adapter/ipc_runtime/contract_validation.hpp"

namespace qrb::adapter::stream_probe
{

// --- The probe in each Contract's descriptor --------------------------------------
//
// A new Contract needs an HcmProbeSample<> for its descriptor type; without it
// this package does not compile. The layout is the one RosProbeSample<> uses,
// so either side checks what the other sends.

/** @brief What a sender keeps between frames. */
struct HcmSenderState
{
  std::uint32_t streamTag{0};
  /** The payload bytes Stamp() wrote into the latest frame, for the Contracts that vary in size. */
  std::uint64_t sentBytes{0};
  /**
   * The buffers that already hold the pattern, so it is written once per
   * buffer of a host that cycles through a few.
   */
  std::array<const void *, 8> patterned{};
  std::size_t nextPatterned{0};
  std::size_t patternedBytes{0};
  /** The send time Stamp() put in the latest frame. */
  std::uint64_t lastSendNs{0};
};

/** @brief What one sent frame became. */
enum class HcmStampResult
{
  kOk,
  kBadBuffer,  // the frame's buffer cannot hold the payload
  kSyncFailed,
};

/**
 * @brief The fd to sync around CPU access to the buffer of `descriptor`; the same rule as
 * the Adapter: false unless the buffer is a DMA-BUF of this process.
 */
inline bool LocalBufferFd(const QC::Memory::BufferDescriptor_t & descriptor, int & fd)
{
  return adapter::InspectHcmBuffer(descriptor, fd) == adapter::HcmBufferAccess::kLocalDmaBuf;
}

template <typename>
inline constexpr bool kAlwaysFalseHcm = false;

/**
 * @brief The probe in descriptor `DescriptorT`. Specialize it for the
 * descriptor of every QRB_CONTRACT_MAPPING entry, with:
 *
 *   // The payload bytes of the frame Stamp() just sent.
 *   static std::uint64_t SentBytes( const HcmSenderState & );
 *   static HcmStampResult Stamp( DescriptorT &, HcmSenderState &, std::uint64_t sequence );
 *   static FrameReading Check( const DescriptorT &, std::uint32_t streamTag,
 *       std::uint64_t &payloadBytes );
 *
 * and, for a descriptor whose buffer the host sizes or lays out per frame:
 *
 *   // Bytes of the buffer a frame of `size` needs, to send or to receive;
 *   // `why` tells when `size` does not suit. Without it: 0, no buffer.
 *   static std::uint64_t BufferBytes( const ProbeFrameSize &size, std::string &why );
 *   // Ready the descriptor of a frame before the host offers it. Without it: nothing.
 *   static void Prepare( DescriptorT &, const ProbeFrameSize &size );
 *
 * Stamp() takes the send time itself, as late as it can, and keeps it in
 * `HcmSenderState::lastSendNs`.
 */
template <typename DescriptorT>
struct HcmProbeSample
{
  static_assert(kAlwaysFalseHcm<DescriptorT>,
      "add an HcmProbeSample<> for the new Contract in test/hcm/hcm_probe_node.cpp");
};

// --- geometry.v1.PoseWithCovarianceStamped (Inline) ---------------------------

/** The probe fills the covariance; the pose is the identity. */
template <>
struct HcmProbeSample<adapter::PoseHcmDescriptor>
{
  using Descriptor = adapter::PoseHcmDescriptor;
  static constexpr const char * kFrameId = "stream_probe";

  static std::uint64_t SentBytes(const HcmSenderState &)
  {
    return sizeof(std::declval<const Descriptor &>().poseWithCovariance.covariance);
  }

  static HcmStampResult Stamp(Descriptor & pose, HcmSenderState & state, std::uint64_t sequence)
  {
    if (pose.frameId != kFrameId)
      (void)pose.frameId.Assign(kFrameId);  // fits FrameId_t
    pose.poseWithCovariance.pose.position.x = 0.0;
    pose.poseWithCovariance.pose.position.y = 0.0;
    pose.poseWithCovariance.pose.position.z = 0.0;
    pose.poseWithCovariance.pose.orientation.x = 0.0;
    pose.poseWithCovariance.pose.orientation.y = 0.0;
    pose.poseWithCovariance.pose.orientation.z = 0.0;
    pose.poseWithCovariance.pose.orientation.w = 1.0;
    const std::uint64_t now = MonotonicNowNs();
    state.lastSendNs = now;
    pose.timestampNs = now;
    StampDoubles(pose.poseWithCovariance.covariance.data(),
        pose.poseWithCovariance.covariance.size(), state.streamTag, sequence, now);
    return HcmStampResult::kOk;
  }

  static FrameReading Check(const Descriptor & pose,
      std::uint32_t streamTag,
      std::uint64_t & payloadBytes)
  {
    payloadBytes = sizeof(pose.poseWithCovariance.covariance);
    FrameReading reading = CheckDoubles(pose.poseWithCovariance.covariance.data(),
        pose.poseWithCovariance.covariance.size(), streamTag);
    if (reading.check == FrameCheck::kOk &&
        (pose.frameId != kFrameId || pose.poseWithCovariance.pose.orientation.w != 1.0))
      reading.check = FrameCheck::kBadLayout;
    return reading;
  }
};

// --- nav.v1.Odometry (Inline) -------------------------------------------------

/**
 * The probe fills the pose covariance; the pose is the identity,
 * the twist is zero and its covariance carries a fixed marker, so a sample that
 * mixes up pose and twist is caught as a bad layout.
 */
template <>
struct HcmProbeSample<adapter::OdometryHcmDescriptor>
{
  using Descriptor = adapter::OdometryHcmDescriptor;
  static constexpr const char * kFrameId = "stream_probe";
  static constexpr const char * kChildFrameId = "stream_probe_base";
  static constexpr double kTwistCovarianceMarker = 0.5;

  static std::uint64_t SentBytes(const HcmSenderState &)
  {
    return sizeof(std::declval<const Descriptor &>().poseWithCovariance.covariance);
  }

  static HcmStampResult Stamp(Descriptor & odom, HcmSenderState & state, std::uint64_t sequence)
  {
    if (odom.frameId != kFrameId)
      (void)odom.frameId.Assign(kFrameId);  // fits FrameId_t
    if (odom.childFrameId != kChildFrameId)
      (void)odom.childFrameId.Assign(kChildFrameId);       // fits FrameId_t
    odom.poseWithCovariance.pose = QRB::Memory::Pose_t{};  // identity
    odom.twistWithCovariance.twist = QRB::Memory::Twist_t{};
    odom.twistWithCovariance.covariance.fill(kTwistCovarianceMarker);
    const std::uint64_t now = MonotonicNowNs();
    state.lastSendNs = now;
    odom.timestampNs = now;
    StampDoubles(odom.poseWithCovariance.covariance.data(),
        odom.poseWithCovariance.covariance.size(), state.streamTag, sequence, now);
    return HcmStampResult::kOk;
  }

  static FrameReading Check(const Descriptor & odom,
      std::uint32_t streamTag,
      std::uint64_t & payloadBytes)
  {
    payloadBytes = sizeof(odom.poseWithCovariance.covariance);
    FrameReading reading = CheckDoubles(odom.poseWithCovariance.covariance.data(),
        odom.poseWithCovariance.covariance.size(), streamTag);
    if (reading.check == FrameCheck::kOk &&
        (odom.frameId != kFrameId || odom.childFrameId != kChildFrameId ||
            odom.poseWithCovariance.pose.orientation.w != 1.0 ||
            odom.twistWithCovariance.covariance[0] != kTwistCovarianceMarker ||
            odom.twistWithCovariance.covariance[35] != kTwistCovarianceMarker))
      reading.check = FrameCheck::kBadLayout;
    return reading;
  }
};

// --- geometry.v1.TwistStamped (Inline) ----------------------------------------

/**
 * The twist has no covariance, so the probe stamps its six values themselves,
 * linear x, y, z then angular x, y, z: the smallest stamp StampDoubles() takes.
 */
template <>
struct HcmProbeSample<adapter::TwistHcmDescriptor>
{
  using Descriptor = adapter::TwistHcmDescriptor;
  static constexpr const char * kFrameId = "stream_probe";
  using Values = std::array<double, kMinProbeDoubles>;

  static std::uint64_t SentBytes(const HcmSenderState &) { return sizeof(Values); }

  static HcmStampResult Stamp(Descriptor & twist, HcmSenderState & state, std::uint64_t sequence)
  {
    if (twist.frameId != kFrameId)
      (void)twist.frameId.Assign(kFrameId);  // fits FrameId_t
    const std::uint64_t now = MonotonicNowNs();
    state.lastSendNs = now;
    twist.timestampNs = now;
    Values values{};
    StampDoubles(values.data(), values.size(), state.streamTag, sequence, now);
    twist.twist.linear.x = values[0];
    twist.twist.linear.y = values[1];
    twist.twist.linear.z = values[2];
    twist.twist.angular.x = values[3];
    twist.twist.angular.y = values[4];
    twist.twist.angular.z = values[5];
    return HcmStampResult::kOk;
  }

  static FrameReading Check(const Descriptor & twist,
      std::uint32_t streamTag,
      std::uint64_t & payloadBytes)
  {
    payloadBytes = sizeof(Values);
    const Values values{twist.twist.linear.x, twist.twist.linear.y, twist.twist.linear.z,
        twist.twist.angular.x, twist.twist.angular.y, twist.twist.angular.z};
    FrameReading reading = CheckDoubles(values.data(), values.size(), streamTag);
    if (reading.check == FrameCheck::kOk && twist.frameId != kFrameId)
      reading.check = FrameCheck::kBadLayout;
    return reading;
  }
};

// --- tf2.v1.TFMessage (Inline, over the descriptor's buffer) ------------------

/**
 * kTransforms transforms in the descriptor's buffer. The six translation and
 * rotation x, y, z of the first carry the probe stamp, the bytes either side
 * reports; every transform has the identity rotation's w and its own child
 * frame, so a sample that drops or reorders transforms is caught as a bad layout.
 *
 * The buffer is the host's: the shared frames give a TFMessage descriptor heap
 * storage for the most transforms a message may carry.
 */
template <>
struct HcmProbeSample<adapter::TFMessageHcmDescriptor>
{
  using Descriptor = adapter::TFMessageHcmDescriptor;
  using Transform = QRB::Memory::TransformStamped_t;
  static constexpr const char * kFrameId = "stream_probe";
  static constexpr std::uint32_t kTransforms = 4;
  using Values = std::array<double, kMinProbeDoubles>;

  static std::uint64_t SentBytes(const HcmSenderState &) { return sizeof(Values); }

  static HcmStampResult Stamp(Descriptor & tf, HcmSenderState & state, std::uint64_t sequence)
  {
    // Any buffer the CPU reaches: a local DMA-BUF is synced, heap memory needs no sync.
    int fd = -1;
    if (!HoldsTransforms(tf, kTransforms) || !UsableBuffer(tf, fd))
      return HcmStampResult::kBadBuffer;
    if (adapter::BeginDmaBufCpuWrite(fd) != adapter::Status::kOk)
      return HcmStampResult::kSyncFailed;
    const std::uint64_t now = MonotonicNowNs();
    state.lastSendNs = now;
    Values values{};
    StampDoubles(values.data(), values.size(), state.streamTag, sequence, now);
    auto * bytes = static_cast<std::uint8_t *>(tf.GetDataPtr());
    for (std::uint32_t i = 0; i < kTransforms; ++i) {
      Transform transform{};
      transform.timestampNs = now;
      (void)transform.frameId.Assign(kFrameId);                      // fits FrameId_t
      (void)transform.childFrameId.Assign(ChildFrameId(i).c_str());  // fits FrameId_t
      if (i == 0) {
        transform.transform.translation.x = values[0];
        transform.transform.translation.y = values[1];
        transform.transform.translation.z = values[2];
        transform.transform.rotation.x = values[3];
        transform.transform.rotation.y = values[4];
        transform.transform.rotation.z = values[5];
      }
      std::memcpy(bytes + i * sizeof(Transform), &transform, sizeof(Transform));
    }
    tf.transformCount = kTransforms;
    tf.validSize = Descriptor::CalcBufferSize(kTransforms);
    if (adapter::EndDmaBufCpuWrite(fd) != adapter::Status::kOk)
      return HcmStampResult::kSyncFailed;
    return HcmStampResult::kOk;
  }

  static FrameReading Check(const Descriptor & tf,
      std::uint32_t streamTag,
      std::uint64_t & payloadBytes)
  {
    payloadBytes = sizeof(Values);
    FrameReading reading;
    int fd = -1;
    if (tf.transformCount == 0 || !HoldsTransforms(tf, tf.transformCount) ||
        !UsableBuffer(tf, fd) || adapter::BeginDmaBufCpuRead(fd) != adapter::Status::kOk)
      return reading;
    std::array<Transform, kTransforms> transforms{};
    const std::uint32_t count = std::min(tf.transformCount, kTransforms);
    std::memcpy(transforms.data(), tf.GetDataPtr(), count * sizeof(Transform));
    const bool ended = adapter::EndDmaBufCpuRead(fd) == adapter::Status::kOk;

    const auto & first = transforms[0].transform;
    const Values values{first.translation.x, first.translation.y, first.translation.z,
        first.rotation.x, first.rotation.y, first.rotation.z};
    reading = CheckDoubles(values.data(), values.size(), streamTag);
    if (reading.check != FrameCheck::kOk)
      return reading;
    if (!ended) {
      reading.check = FrameCheck::kBadPayload;
      return reading;
    }
    bool layout = tf.transformCount == kTransforms;
    for (std::uint32_t i = 0; layout && i < kTransforms; ++i) {
      layout = transforms[i].frameId == kFrameId &&
               transforms[i].childFrameId == ChildFrameId(i).c_str() &&
               transforms[i].transform.rotation.w == 1.0;
    }
    if (!layout)
      reading.check = FrameCheck::kBadLayout;
    return reading;
  }

private:
  static std::string ChildFrameId(std::uint32_t index)
  {
    return "stream_probe_" + std::to_string(index);
  }

  /** The fd if the buffer is a local DMA-BUF, else -1; false for a buffer the CPU cannot reach. */
  static bool UsableBuffer(const Descriptor & tf, int & fd)
  {
    const adapter::HcmBufferAccess access = adapter::InspectHcmBuffer(tf, fd);
    return access == adapter::HcmBufferAccess::kLocalDmaBuf ||
           access == adapter::HcmBufferAccess::kHostMemory;
  }

  static bool HoldsTransforms(const Descriptor & tf, std::uint32_t count)
  {
    return tf.pBuf != nullptr && tf.offset <= tf.size &&
           tf.size - tf.offset >= Descriptor::CalcBufferSize(count);
  }
};

// --- sensor.v1.JointState (inline, over the descriptor's buffer) ---------------

/** Three joints carry the six probe values in position then velocity; effort is zero. */
template <>
struct HcmProbeSample<adapter::JointStateHcmDescriptor>
{
  using Descriptor = adapter::JointStateHcmDescriptor;
  using Joint = QRB::Memory::JointState_t;
  static constexpr const char * kFrameId = "stream_probe";
  static constexpr std::uint32_t kJoints = 3;
  using Values = std::array<double, kMinProbeDoubles>;

  static std::uint64_t SentBytes(const HcmSenderState &) { return sizeof(Values); }

  static HcmStampResult Stamp(Descriptor & descriptor,
      HcmSenderState & state,
      std::uint64_t sequence)
  {
    const std::size_t bytes = Descriptor::CalcBufferSize(kJoints);
    int fd = -1;
    if (descriptor.offset > descriptor.size || descriptor.size - descriptor.offset < bytes ||
        !UsableBuffer(descriptor, fd))
      return HcmStampResult::kBadBuffer;
    void * data = descriptor.GetDataPtr();
    if (data == nullptr || reinterpret_cast<std::uintptr_t>(data) % alignof(Joint) != 0)
      return HcmStampResult::kBadBuffer;

    std::array<Joint, kJoints> joints{};
    for (std::uint32_t i = 0; i < kJoints; ++i)
      (void)joints[i].name.Assign(JointName(i).c_str());  // fits the fixed joint name
    if (adapter::BeginDmaBufCpuWrite(fd) != adapter::Status::kOk)
      return HcmStampResult::kSyncFailed;
    const std::uint64_t now = MonotonicNowNs();
    state.lastSendNs = now;
    Values values{};
    StampDoubles(values.data(), values.size(), state.streamTag, sequence, now);
    for (std::uint32_t i = 0; i < kJoints; ++i) {
      joints[i].position = values[i];
      joints[i].velocity = values[kJoints + i];
    }
    std::memcpy(data, joints.data(), bytes);
    if (adapter::EndDmaBufCpuWrite(fd) != adapter::Status::kOk)
      return HcmStampResult::kSyncFailed;
    (void)descriptor.frameId.Assign(kFrameId);  // fits FrameId_t
    descriptor.timestampNs = now;
    descriptor.jointCount = kJoints;
    descriptor.validSize = bytes;
    return HcmStampResult::kOk;
  }

  static FrameReading Check(const Descriptor & descriptor,
      std::uint32_t streamTag,
      std::uint64_t & payloadBytes)
  {
    payloadBytes = sizeof(Values);
    FrameReading reading;
    if (descriptor.jointCount != kJoints) {
      reading.check = FrameCheck::kBadLayout;
      return reading;
    }
    int fd = -1;
    const Joint * data = descriptor.GetJointPtr(0);
    if (data == nullptr || !UsableBuffer(descriptor, fd) ||
        adapter::BeginDmaBufCpuRead(fd) != adapter::Status::kOk)
      return reading;
    std::array<Joint, kJoints> joints{};
    std::memcpy(joints.data(), data, sizeof(joints));
    if (adapter::EndDmaBufCpuRead(fd) != adapter::Status::kOk)
      return reading;

    Values values{};
    bool layout = descriptor.frameId == kFrameId;
    for (std::uint32_t i = 0; i < kJoints; ++i) {
      values[i] = joints[i].position;
      values[kJoints + i] = joints[i].velocity;
      layout = layout && joints[i].name.Size() <= QRB::Memory::QRB_JOINT_NAME_MAX_LEN &&
               joints[i].name == JointName(i).c_str() && joints[i].effort == 0.0;
    }
    reading = CheckDoubles(values.data(), values.size(), streamTag);
    if (reading.check == FrameCheck::kOk && !layout)
      reading.check = FrameCheck::kBadLayout;
    return reading;
  }

private:
  static std::string JointName(std::uint32_t index) { return "joint_" + std::to_string(index); }

  static bool UsableBuffer(const Descriptor & descriptor, int & fd)
  {
    const adapter::HcmBufferAccess access = adapter::InspectHcmBuffer(descriptor, fd);
    return access == adapter::HcmBufferAccess::kLocalDmaBuf ||
           access == adapter::HcmBufferAccess::kHostMemory;
  }
};

// --- geometry.v1.PoseStamped and PoseArray (inline) ---------------------------

/**
 * The six values StampDoubles() takes sit in the x, y, z of a position and the x, y, z of an
 * orientation, in this order. The w of the quaternion stays 1, which keeps it non-zero; it is not
 * a unit quaternion, which the Contract does not ask for.
 */
namespace hcm_pose_probe
{

using Values = std::array<double, kMinProbeDoubles>;

inline void Fill(QRB::Memory::Pose_t & pose, const Values & values)
{
  pose.position.x = values[0];
  pose.position.y = values[1];
  pose.position.z = values[2];
  pose.orientation.x = values[3];
  pose.orientation.y = values[4];
  pose.orientation.z = values[5];
  pose.orientation.w = 1.0;
}

inline Values Read(const QRB::Memory::Pose_t & pose)
{
  return Values{pose.position.x, pose.position.y, pose.position.z, pose.orientation.x,
      pose.orientation.y, pose.orientation.z};
}

}  // namespace hcm_pose_probe

/**
 * The probe stamps the six values of the pose, and the frame is the probe's. The layout is the
 * one RosProbeSample<> uses, so either side checks what the other sends.
 */
template <>
struct HcmProbeSample<adapter::PoseStampedHcmDescriptor>
{
  using Descriptor = adapter::PoseStampedHcmDescriptor;
  static constexpr const char * kFrameId = "stream_probe";
  using Values = hcm_pose_probe::Values;

  static std::uint64_t SentBytes(const HcmSenderState &) { return sizeof(Values); }

  static HcmStampResult Stamp(Descriptor & pose, HcmSenderState & state, std::uint64_t sequence)
  {
    if (pose.frameId != kFrameId)
      (void)pose.frameId.Assign(kFrameId);  // fits FrameId_t
    const std::uint64_t now = MonotonicNowNs();
    state.lastSendNs = now;
    pose.timestampNs = now;
    Values values{};
    StampDoubles(values.data(), values.size(), state.streamTag, sequence, now);
    hcm_pose_probe::Fill(pose.pose, values);
    return HcmStampResult::kOk;
  }

  static FrameReading Check(const Descriptor & pose,
      std::uint32_t streamTag,
      std::uint64_t & payloadBytes)
  {
    payloadBytes = sizeof(Values);
    const Values values = hcm_pose_probe::Read(pose.pose);
    FrameReading reading = CheckDoubles(values.data(), values.size(), streamTag);
    if (reading.check == FrameCheck::kOk &&
        (pose.frameId != kFrameId || pose.pose.orientation.w != 1.0))
      reading.check = FrameCheck::kBadLayout;
    return reading;
  }
};

/**
 * kPoses poses in the descriptor's buffer. The six values of the first carry the probe stamp,
 * the bytes either side reports; every pose has the identity orientation's w, and each of the
 * others a marker in its x, so a sample that drops or reorders poses is caught as a bad layout.
 *
 * The buffer is the host's: the shared frames give a PoseArray descriptor heap storage for the
 * most poses a message may carry.
 */
template <>
struct HcmProbeSample<adapter::PoseArrayHcmDescriptor>
{
  using Descriptor = adapter::PoseArrayHcmDescriptor;
  using Pose = QRB::Memory::Pose_t;
  static constexpr const char * kFrameId = "stream_probe";
  static constexpr std::uint32_t kPoses = 4;
  using Values = hcm_pose_probe::Values;

  static std::uint64_t SentBytes(const HcmSenderState &) { return sizeof(Values); }

  static HcmStampResult Stamp(Descriptor & poses, HcmSenderState & state, std::uint64_t sequence)
  {
    // Any buffer the CPU reaches: a local DMA-BUF is synced, heap memory needs no sync.
    int fd = -1;
    if (!HoldsPoses(poses, kPoses) || !UsableBuffer(poses, fd))
      return HcmStampResult::kBadBuffer;
    if (adapter::BeginDmaBufCpuWrite(fd) != adapter::Status::kOk)
      return HcmStampResult::kSyncFailed;
    const std::uint64_t now = MonotonicNowNs();
    state.lastSendNs = now;
    Values values{};
    StampDoubles(values.data(), values.size(), state.streamTag, sequence, now);
    auto * bytes = static_cast<std::uint8_t *>(poses.GetDataPtr());
    for (std::uint32_t i = 0; i < kPoses; ++i) {
      Pose pose{};
      if (i == 0)
        hcm_pose_probe::Fill(pose, values);
      else
        pose.position.x = Marker(i);
      std::memcpy(bytes + i * sizeof(Pose), &pose, sizeof(Pose));
    }
    if (poses.frameId != kFrameId)
      (void)poses.frameId.Assign(kFrameId);  // fits FrameId_t
    poses.timestampNs = now;
    poses.poseCount = kPoses;
    poses.validSize = Descriptor::CalcBufferSize(kPoses);
    if (adapter::EndDmaBufCpuWrite(fd) != adapter::Status::kOk)
      return HcmStampResult::kSyncFailed;
    return HcmStampResult::kOk;
  }

  static FrameReading Check(const Descriptor & poses,
      std::uint32_t streamTag,
      std::uint64_t & payloadBytes)
  {
    payloadBytes = sizeof(Values);
    FrameReading reading;
    int fd = -1;
    if (poses.poseCount == 0 || !HoldsPoses(poses, poses.poseCount) || !UsableBuffer(poses, fd) ||
        adapter::BeginDmaBufCpuRead(fd) != adapter::Status::kOk)
      return reading;
    std::array<Pose, kPoses> copy{};
    const std::uint32_t count = std::min(poses.poseCount, kPoses);
    std::memcpy(copy.data(), poses.GetDataPtr(), count * sizeof(Pose));
    const bool ended = adapter::EndDmaBufCpuRead(fd) == adapter::Status::kOk;

    const Values values = hcm_pose_probe::Read(copy[0]);
    reading = CheckDoubles(values.data(), values.size(), streamTag);
    if (reading.check != FrameCheck::kOk)
      return reading;
    if (!ended) {
      reading.check = FrameCheck::kBadPayload;
      return reading;
    }
    bool layout =
        poses.poseCount == kPoses && poses.frameId == kFrameId && copy[0].orientation.w == 1.0;
    for (std::uint32_t i = 1; layout && i < kPoses; ++i)
      layout = IsMarkerPose(copy[i], i);
    if (!layout)
      reading.check = FrameCheck::kBadLayout;
    return reading;
  }

private:
  /** @brief Exact in a double, and different for every pose after the first. */
  static double Marker(std::uint32_t index) { return 1000.0 + static_cast<double>(index); }

  static bool IsMarkerPose(const Pose & pose, std::uint32_t index)
  {
    return pose.position.x == Marker(index) && pose.position.y == 0.0 && pose.position.z == 0.0 &&
           pose.orientation.x == 0.0 && pose.orientation.y == 0.0 && pose.orientation.z == 0.0 &&
           pose.orientation.w == 1.0;
  }

  /** The fd if the buffer is a local DMA-BUF, else -1; false for a buffer the CPU cannot reach. */
  static bool UsableBuffer(const Descriptor & poses, int & fd)
  {
    const adapter::HcmBufferAccess access = adapter::InspectHcmBuffer(poses, fd);
    return access == adapter::HcmBufferAccess::kLocalDmaBuf ||
           access == adapter::HcmBufferAccess::kHostMemory;
  }

  static bool HoldsPoses(const Descriptor & poses, std::uint32_t count)
  {
    return poses.pBuf != nullptr && poses.offset <= poses.size &&
           poses.size - poses.offset >= Descriptor::CalcBufferSize(count);
  }
};

// --- geometry.v1.Vector3Stamped (inline) --------------------------------------

/**
 * A vector has only three values, too few for StampDoubles(), so the probe stamps the three of
 * StampTriple(): stream tag, sequence and sentinel. The send time is the stamp of the
 * descriptor. The layout is the one RosProbeSample<> uses, so either side checks what the other
 * sends.
 */
template <>
struct HcmProbeSample<adapter::Vector3StampedHcmDescriptor>
{
  using Descriptor = adapter::Vector3StampedHcmDescriptor;
  static constexpr const char * kFrameId = "stream_probe";
  using Values = std::array<double, kTripleDoubles>;

  static std::uint64_t SentBytes(const HcmSenderState &) { return sizeof(Values); }

  static HcmStampResult Stamp(Descriptor & vector, HcmSenderState & state, std::uint64_t sequence)
  {
    if (vector.frameId != kFrameId)
      (void)vector.frameId.Assign(kFrameId);  // fits FrameId_t
    const std::uint64_t now = MonotonicNowNs();
    state.lastSendNs = now;
    vector.timestampNs = now;
    Values values{};
    StampTriple(values.data(), state.streamTag, sequence);
    vector.vector.x = values[0];
    vector.vector.y = values[1];
    vector.vector.z = values[2];
    return HcmStampResult::kOk;
  }

  static FrameReading Check(const Descriptor & vector,
      std::uint32_t streamTag,
      std::uint64_t & payloadBytes)
  {
    payloadBytes = sizeof(Values);
    const Values values{vector.vector.x, vector.vector.y, vector.vector.z};
    FrameReading reading = CheckTriple(values.data(), vector.timestampNs, streamTag);
    if (reading.check == FrameCheck::kOk && vector.frameId != kFrameId)
      reading.check = FrameCheck::kBadLayout;
    return reading;
  }
};

// --- sensor.v1.Imu (inline) ---------------------------------------------------

/**
 * The probe stamps linear_acceleration_covariance (nine values); the orientation and angular
 * velocity covariances are filled with -1 as layout markers. The layout is the one
 * RosProbeSample<> uses, so either side checks what the other sends.
 */
template <>
struct HcmProbeSample<adapter::ImuHcmDescriptor>
{
  using Descriptor = adapter::ImuHcmDescriptor;
  static constexpr const char * kFrameId = "stream_probe";

  static std::uint64_t SentBytes(const HcmSenderState &)
  {
    return sizeof(std::declval<const Descriptor &>().linear_acceleration_covariance);
  }

  static HcmStampResult Stamp(Descriptor & imu, HcmSenderState & state, std::uint64_t sequence)
  {
    if (imu.frameId != kFrameId)
      (void)imu.frameId.Assign(kFrameId);  // fits FrameId_t
    imu.orientation.x = 0.0;
    imu.orientation.y = 0.0;
    imu.orientation.z = 0.0;
    imu.orientation.w = 1.0;
    imu.angular_velocity.x = 0.0;
    imu.angular_velocity.y = 0.0;
    imu.angular_velocity.z = 0.0;
    imu.linear_acceleration.x = 0.0;
    imu.linear_acceleration.y = 0.0;
    imu.linear_acceleration.z = 0.0;
    std::fill(std::begin(imu.orientation_covariance), std::end(imu.orientation_covariance), -1.0);
    std::fill(std::begin(imu.angular_velocity_covariance),
        std::end(imu.angular_velocity_covariance), -1.0);
    const std::uint64_t now = MonotonicNowNs();
    state.lastSendNs = now;
    imu.timestampNs = now;
    StampDoubles(imu.linear_acceleration_covariance, std::size(imu.linear_acceleration_covariance),
        state.streamTag, sequence, now);
    return HcmStampResult::kOk;
  }

  static FrameReading Check(const Descriptor & imu,
      std::uint32_t streamTag,
      std::uint64_t & payloadBytes)
  {
    payloadBytes = sizeof(imu.linear_acceleration_covariance);
    FrameReading reading = CheckDoubles(imu.linear_acceleration_covariance,
        std::size(imu.linear_acceleration_covariance), streamTag);
    if (reading.check == FrameCheck::kOk && (imu.frameId != kFrameId || imu.orientation.w != 1.0 ||
                                                imu.orientation_covariance[0] != -1.0))
      reading.check = FrameCheck::kBadLayout;
    return reading;
  }
};

// --- sensor.v1.PointCloud2 (DMA-BUF) ------------------------------------------

/**
 * One row of points x, y, z, intensity (FLOAT32) over the whole buffer of
 * `point_cloud_bytes`; a receiver takes any PointCloud2 that fits the buffer.
 */
template <>
struct HcmProbeSample<adapter::PointCloudHcmDescriptor>
{
  using Descriptor = adapter::PointCloudHcmDescriptor;
  static constexpr const char * kFrameId = "stream_probe";
  static constexpr std::uint32_t kPointStep = 16;
  static constexpr const char * kFieldNames[] = {"x", "y", "z", "intensity"};
  static constexpr std::size_t kFieldCount = sizeof(kFieldNames) / sizeof(kFieldNames[0]);

  static std::uint64_t BufferBytes(const ProbeFrameSize & size, std::string & why)
  {
    if (size.pointCloudBytes < kMinPayloadBytes || size.pointCloudBytes % kPointStep != 0)
      why = "'point_cloud_bytes' of PointCloud2 must be a multiple of " +
            std::to_string(kPointStep) + " and at least " + std::to_string(kMinPayloadBytes);
    return size.pointCloudBytes;
  }

  static std::uint64_t SentBytes(const HcmSenderState & state) { return state.sentBytes; }

  static HcmStampResult Stamp(Descriptor & cloud, HcmSenderState & state, std::uint64_t sequence)
  {
    if (cloud.pBuf == nullptr || cloud.offset > cloud.size)
      return HcmStampResult::kBadBuffer;
    const std::size_t bytes =
        static_cast<std::size_t>((cloud.size - cloud.offset) / kPointStep * kPointStep);
    if (bytes < kMinPayloadBytes || bytes > std::numeric_limits<std::uint32_t>::max())
      return HcmStampResult::kBadBuffer;
    auto * data = static_cast<std::uint8_t *>(cloud.GetDataPtr());
    int fd = -1;
    if (!LocalBufferFd(cloud, fd))
      return HcmStampResult::kBadBuffer;

    if (adapter::BeginDmaBufCpuWrite(fd) != adapter::Status::kOk)
      return HcmStampResult::kSyncFailed;
    if (state.patternedBytes != bytes) {
      state.patterned = {};
      state.patternedBytes = bytes;
    }
    if (std::find(state.patterned.begin(), state.patterned.end(), data) == state.patterned.end()) {
      FillPattern(data, bytes, state.streamTag);
      state.patterned[state.nextPatterned] = data;
      state.nextPatterned = (state.nextPatterned + 1) % state.patterned.size();
    }
    const std::uint64_t now = MonotonicNowNs();
    state.lastSendNs = now;
    StampFrame(data, bytes, state.streamTag, sequence, now);
    if (adapter::EndDmaBufCpuWrite(fd) != adapter::Status::kOk)
      return HcmStampResult::kSyncFailed;

    state.sentBytes = bytes;
    cloud.validSize = bytes;
    cloud.height = 1;
    cloud.width = static_cast<std::uint32_t>(bytes / kPointStep);
    if (!HasSenderFields(cloud)) {
      cloud.numFields = static_cast<std::uint32_t>(kFieldCount);
      for (std::size_t i = 0; i < kFieldCount; ++i)
        cloud.fields[i] = QRB::Memory::PointField_t(kFieldNames[i],
            static_cast<std::uint32_t>(i * 4), QRB::Memory::POINT_FIELD_TYPE_FLOAT32, 1);
    }
    cloud.isBigEndian = false;
    cloud.pointStep = kPointStep;
    cloud.rowStep = static_cast<std::uint32_t>(bytes);
    cloud.isDense = true;
    cloud.timestampNs = now;
    if (cloud.frameId != kFrameId)
      (void)cloud.frameId.Assign(kFrameId);  // fits FrameId_t
    return HcmStampResult::kOk;
  }

  static FrameReading Check(const Descriptor & cloud,
      std::uint32_t streamTag,
      std::uint64_t & payloadBytes)
  {
    payloadBytes = cloud.validSize;
    FrameReading reading;
    if (cloud.pBuf == nullptr || cloud.offset > cloud.size ||
        cloud.validSize > cloud.size - cloud.offset)
      return reading;

    int fd = -1;
    if (!LocalBufferFd(cloud, fd) || adapter::BeginDmaBufCpuRead(fd) != adapter::Status::kOk)
      return reading;
    reading = CheckFrame(static_cast<const std::uint8_t *>(cloud.GetDataPtr()),
        static_cast<std::size_t>(cloud.validSize), streamTag);
    const bool ended = adapter::EndDmaBufCpuRead(fd) == adapter::Status::kOk;
    if (reading.check != FrameCheck::kOk)
      return reading;
    if (!ended) {
      reading.check = FrameCheck::kBadPayload;
      return reading;
    }
    if (!HasSenderLayout(cloud))
      reading.check = FrameCheck::kBadLayout;
    return reading;
  }

private:
  static bool HasSenderFields(const Descriptor & cloud)
  {
    if (cloud.numFields != kFieldCount)
      return false;
    for (std::size_t i = 0; i < kFieldCount; ++i) {
      const QRB::Memory::PointField_t & field = cloud.fields[i];
      if (field.name != kFieldNames[i] || field.offset != i * 4 ||
          field.datatype != QRB::Memory::POINT_FIELD_TYPE_FLOAT32 || field.count != 1)
        return false;
    }
    return true;
  }

  static bool HasSenderLayout(const Descriptor & cloud)
  {
    return cloud.frameId == kFrameId && cloud.height == 1 && cloud.pointStep == kPointStep &&
           cloud.rowStep == std::uint64_t{cloud.width} * kPointStep &&
           cloud.rowStep == cloud.validSize && !cloud.isBigEndian && cloud.isDense &&
           HasSenderFields(cloud);
  }
};

// --- sensor.v1.ImageLayout (DMA-BUF) ------------------------------------------

/**
 * An rgb8 Camera frame of the width and height Prepare() gave the descriptor,
 * whose rows are ProbeImageHcmStride( width ) apart as the QCNode Camera
 * sample allocates them. Its timestamp is the camera qtimer, which the Adapter
 * converts to and from ROS time. A receiver's descriptor has no layout, so it
 * takes the size and the layout of any ROS image that fits the buffer.
 */
template <>
struct HcmProbeSample<adapter::ImageHcmDescriptor>
{
  using Descriptor = adapter::ImageHcmDescriptor;
  /** @brief How far a received timestamp may be from the qtimer now. */
  static constexpr std::uint64_t kStampToleranceNs = 5000000000ull;

  static std::uint64_t BufferBytes(const ProbeFrameSize & size, std::string & why)
  {
    why = CheckImageSize(size.imageWidth, size.imageHeight);
    if (!why.empty())
      return 0;
    adapter::ImageLayout ros;
    if (!adapter::CanonicalImageLayout(size.imageWidth, size.imageHeight, "rgb8", ros)) {
      why = "'image_width' and 'image_height' are too large for a ROS image";
      return 0;
    }
    return std::max(ProbeImageHcmStride(size.imageWidth) * size.imageHeight, ros.storageSize);
  }

  static void Prepare(Descriptor & image, const ProbeFrameSize & size)
  {
    image.format = QC::QC_IMAGE_FORMAT_RGB888;
    image.width = size.imageWidth;
    image.height = size.imageHeight;
    image.batchSize = 1;
    image.numPlanes = 0;
    for (std::uint32_t i = 0; i < QC_NUM_IMAGE_PLANES; ++i) {
      image.stride[i] = 0;
      image.actualHeight[i] = 0;
      image.planeBufSize[i] = 0;
    }
    image.validSize = 0;
  }

  static std::uint64_t SentBytes(const HcmSenderState & state) { return state.sentBytes; }

  static HcmStampResult Stamp(Descriptor & image, HcmSenderState & state, std::uint64_t sequence)
  {
    const std::uint32_t width = image.width;
    const std::uint32_t height = image.height;
    const std::uint64_t stride = ProbeImageHcmStride(width);
    const std::uint64_t storage = stride * height;
    if (!CheckImageSize(width, height).empty() ||
        storage > std::numeric_limits<std::uint32_t>::max() || image.pBuf == nullptr ||
        image.offset > image.size || image.size - image.offset < storage)
      return HcmStampResult::kBadBuffer;
    auto * data = static_cast<std::uint8_t *>(image.GetDataPtr());
    int fd = -1;
    if (!LocalBufferFd(image, fd))
      return HcmStampResult::kBadBuffer;

    image.format = QC::QC_IMAGE_FORMAT_RGB888;
    image.batchSize = 1;
    image.numPlanes = 1;
    for (std::uint32_t i = 0; i < QC_NUM_IMAGE_PLANES; ++i) {
      image.stride[i] = 0;
      image.actualHeight[i] = 0;
      image.planeBufSize[i] = 0;
    }
    image.stride[0] = static_cast<std::uint32_t>(stride);
    image.actualHeight[0] = height;
    image.planeBufSize[0] = static_cast<std::uint32_t>(storage);
    const ProbeImageLayout layout = LayoutOf(image);

    if (adapter::BeginDmaBufCpuWrite(fd) != adapter::Status::kOk)
      return HcmStampResult::kSyncFailed;
    if (state.patternedBytes != storage) {
      state.patterned = {};
      state.patternedBytes = static_cast<std::size_t>(storage);
    }
    if (std::find(state.patterned.begin(), state.patterned.end(), data) == state.patterned.end()) {
      FillImagePattern(data, layout, state.streamTag);
      state.patterned[state.nextPatterned] = data;
      state.nextPatterned = (state.nextPatterned + 1) % state.patterned.size();
    }
    const std::uint64_t now = MonotonicNowNs();
    state.lastSendNs = now;
    StampImage(data, layout, state.streamTag, sequence, now);
    if (adapter::EndDmaBufCpuWrite(fd) != adapter::Status::kOk)
      return HcmStampResult::kSyncFailed;

    state.sentBytes = layout.LogicalBytes();
    image.validSize = static_cast<std::size_t>(storage);
    image.timestamp = adapter::SampleHcmClocks().qtimerNs;
    image.timestampQGPTP = 0;
    image.flags = 0;
    return HcmStampResult::kOk;
  }

  static FrameReading Check(const Descriptor & image,
      std::uint32_t streamTag,
      std::uint64_t & payloadBytes)
  {
    const ProbeImageLayout layout = LayoutOf(image);
    payloadBytes = layout.LogicalBytes();
    FrameReading reading;
    if (image.pBuf == nullptr || image.offset > image.size ||
        image.validSize > image.size - image.offset || layout.planeCount == 0 ||
        StorageEnd(layout) > image.validSize)
      return reading;

    int fd = -1;
    if (!LocalBufferFd(image, fd) || adapter::BeginDmaBufCpuRead(fd) != adapter::Status::kOk)
      return reading;
    reading = CheckImage(static_cast<const std::uint8_t *>(image.GetDataPtr()), layout, streamTag);
    const bool ended = adapter::EndDmaBufCpuRead(fd) == adapter::Status::kOk;
    if (reading.check != FrameCheck::kOk)
      return reading;
    if (!ended) {
      reading.check = FrameCheck::kBadPayload;
      return reading;
    }
    // The sender is the ROS probe: its rgb8 image, stamped with ROS time, which
    // the Adapter converted to the qtimer.
    if (image.format != QC::QC_IMAGE_FORMAT_RGB888 ||
        !WithinNs(image.timestamp, adapter::SampleHcmClocks().qtimerNs, kStampToleranceNs))
      reading.check = FrameCheck::kBadLayout;
    return reading;
  }

private:
  /** @brief The valid rows of `image` in the layout its descriptor describes. */
  static ProbeImageLayout LayoutOf(const Descriptor & image)
  {
    ProbeImageLayout layout;
    const bool nv12 = image.format == QC::QC_IMAGE_FORMAT_NV12;
    const bool rgb =
        image.format == QC::QC_IMAGE_FORMAT_RGB888 || image.format == QC::QC_IMAGE_FORMAT_BGR888;
    if ((!nv12 && !rgb) || image.numPlanes != (nv12 ? 2u : 1u))
      return layout;
    layout.planeCount = image.numPlanes;
    std::uint64_t offset = 0;
    for (std::size_t i = 0; i < layout.planeCount; ++i) {
      layout.planes[i] = {offset, image.stride[i], rgb ? image.width * 3ull : image.width,
          i == 0 ? image.height : image.height / 2ull};
      offset += image.planeBufSize[i] != 0 ? image.planeBufSize[i] :
                                             std::uint64_t{image.stride[i]} * image.actualHeight[i];
    }
    return layout;
  }

  /** @brief One past the last valid byte of `layout`. */
  static std::uint64_t StorageEnd(const ProbeImageLayout & layout)
  {
    const ProbeImagePlane & last = layout.planes[layout.planeCount - 1];
    return last.rows == 0 ? last.offset :
                            last.offset + last.stride * (last.rows - 1) + last.rowBytes;
  }
};

// --- vision.v1.Detection2DArray and Detection3DArray (inline) -------------------

/** @brief The bounding box, the one thing the 2-D and 3-D detection probes differ in. */
struct HcmBox2d
{
  using Box = QRB::Memory::BoundingBox2d_t;

  static void Fill(Box & box, std::size_t detection)
  {
    box.center.position.x = detection_probe::BoxValue(detection, 0);
    box.center.position.y = detection_probe::BoxValue(detection, 1);
    box.center.theta = detection_probe::BoxValue(detection, 2);
    box.sizeX = detection_probe::BoxValue(detection, 3);
    box.sizeY = detection_probe::BoxValue(detection, 4);
  }

  static bool Matches(const Box & box, std::size_t detection)
  {
    return box.center.position.x == detection_probe::BoxValue(detection, 0) &&
           box.center.position.y == detection_probe::BoxValue(detection, 1) &&
           box.center.theta == detection_probe::BoxValue(detection, 2) &&
           box.sizeX == detection_probe::BoxValue(detection, 3) &&
           box.sizeY == detection_probe::BoxValue(detection, 4);
  }
};

struct HcmBox3d
{
  using Box = QRB::Memory::BoundingBox3d_t;

  static void Fill(Box & box, std::size_t detection)
  {
    box.center.position.x = detection_probe::BoxValue(detection, 0);
    box.center.position.y = detection_probe::BoxValue(detection, 1);
    box.center.position.z = detection_probe::BoxValue(detection, 2);
    box.center.orientation.w = 1.0;
    box.size.x = detection_probe::BoxValue(detection, 3);
    box.size.y = detection_probe::BoxValue(detection, 4);
    box.size.z = detection_probe::BoxValue(detection, 5);
  }

  static bool Matches(const Box & box, std::size_t detection)
  {
    return box.center.position.x == detection_probe::BoxValue(detection, 0) &&
           box.center.position.y == detection_probe::BoxValue(detection, 1) &&
           box.center.position.z == detection_probe::BoxValue(detection, 2) &&
           box.center.orientation.x == 0.0 && box.center.orientation.y == 0.0 &&
           box.center.orientation.z == 0.0 && box.center.orientation.w == 1.0 &&
           box.size.x == detection_probe::BoxValue(detection, 3) &&
           box.size.y == detection_probe::BoxValue(detection, 4) &&
           box.size.z == detection_probe::BoxValue(detection, 5);
  }
};

/**
 * The frame of detection_probe as the records in the buffer of a detection descriptor, which
 * holds just the records of its kDetections detections.
 */
template <typename Descriptor, typename Record, typename BoxTraits>
struct HcmDetectionProbe
{
  static constexpr std::size_t kRecordBytes = sizeof(Record) * detection_probe::kDetections;
  static constexpr std::uint64_t kPayloadBytes =
      detection_probe::kCovarianceDoubles * sizeof(double);

  static std::uint64_t BufferBytes(const ProbeFrameSize &, std::string &)
  {
    return Descriptor::CalcBufferSize(static_cast<std::uint32_t>(detection_probe::kDetections));
  }

  static std::uint64_t SentBytes(const HcmSenderState &) { return kPayloadBytes; }

  static HcmStampResult Stamp(Descriptor & descriptor,
      HcmSenderState & state,
      std::uint64_t sequence)
  {
    int fd = -1;
    if (!HasBuffer(descriptor, fd))
      return HcmStampResult::kBadBuffer;

    Record records[detection_probe::kDetections] = {};
    for (std::size_t i = 0; i < detection_probe::kDetections; ++i) {
      Record & record = records[i];
      (void)std::snprintf(record.id, sizeof(record.id), "%s", detection_probe::Id(i));
      BoxTraits::Fill(record.bbox, i);
      record.resultCount = static_cast<std::uint32_t>(detection_probe::HypothesisCount(i));
      for (std::size_t j = 0; j < record.resultCount; ++j) {
        auto & result = record.results[j];
        (void)std::snprintf(
            result.classId, sizeof(result.classId), "%s", detection_probe::ClassId(i, j));
        result.score = detection_probe::Score(i, j);
        result.pose.pose.orientation.w = 1.0;
      }
    }
    const std::uint64_t now = MonotonicNowNs();
    state.lastSendNs = now;
    auto & covariance = records[0].results[0].pose.covariance;
    StampDoubles(covariance.data(), covariance.size(), state.streamTag, sequence, now);

    if (adapter::BeginDmaBufCpuWrite(fd) != adapter::Status::kOk)
      return HcmStampResult::kSyncFailed;
    std::memcpy(descriptor.GetDataPtr(), records, kRecordBytes);
    if (adapter::EndDmaBufCpuWrite(fd) != adapter::Status::kOk)
      return HcmStampResult::kSyncFailed;

    if (descriptor.frameId != detection_probe::kFrameId)
      (void)descriptor.frameId.Assign(detection_probe::kFrameId);  // fits FrameId_t
    descriptor.timestampNs = now;
    descriptor.objCount = static_cast<std::uint32_t>(detection_probe::kDetections);
    descriptor.validSize = kRecordBytes;
    return HcmStampResult::kOk;
  }

  static FrameReading Check(const Descriptor & descriptor,
      std::uint32_t streamTag,
      std::uint64_t & payloadBytes)
  {
    payloadBytes = kPayloadBytes;
    FrameReading reading;
    int fd = -1;
    if (descriptor.objCount != detection_probe::kDetections || !HasBuffer(descriptor, fd))
      return reading;

    Record records[detection_probe::kDetections];
    if (adapter::BeginDmaBufCpuRead(fd) != adapter::Status::kOk)
      return reading;
    std::memcpy(records, descriptor.GetDataPtr(), kRecordBytes);
    const bool ended = adapter::EndDmaBufCpuRead(fd) == adapter::Status::kOk;

    const auto & covariance = records[0].results[0].pose.covariance;
    reading = CheckDoubles(covariance.data(), covariance.size(), streamTag);
    if (reading.check != FrameCheck::kOk)
      return reading;
    if (!ended) {
      reading.check = FrameCheck::kBadPayload;
      return reading;
    }
    if (!HasLayout(descriptor, records))
      reading.check = FrameCheck::kBadLayout;
    return reading;
  }

private:
  /** @brief The two records fit the buffer, which this process can read and write. */
  static bool HasBuffer(const Descriptor & descriptor, int & fd)
  {
    if (descriptor.pBuf == nullptr || descriptor.offset > descriptor.size ||
        descriptor.size - descriptor.offset < kRecordBytes)
      return false;
    const adapter::HcmBufferAccess access = adapter::InspectHcmBuffer(descriptor, fd);
    return access == adapter::HcmBufferAccess::kLocalDmaBuf ||
           access == adapter::HcmBufferAccess::kHostMemory;
  }

  static bool HasLayout(const Descriptor & descriptor,
      const Record (&records)[detection_probe::kDetections])
  {
    if (descriptor.frameId != detection_probe::kFrameId)
      return false;
    for (std::size_t i = 0; i < detection_probe::kDetections; ++i) {
      const Record & record = records[i];
      if (std::strncmp(record.id, detection_probe::Id(i), sizeof(record.id)) != 0 ||
          !BoxTraits::Matches(record.bbox, i) ||
          record.resultCount != detection_probe::HypothesisCount(i))
        return false;
      for (std::size_t j = 0; j < record.resultCount; ++j) {
        const auto & result = record.results[j];
        if (std::strncmp(result.classId, detection_probe::ClassId(i, j), sizeof(result.classId)) !=
                0 ||
            result.score != detection_probe::Score(i, j) || result.pose.pose.orientation.w != 1.0)
          return false;
      }
    }
    return true;
  }
};

template <>
struct HcmProbeSample<adapter::Detection2DHcmDescriptor>
  : HcmDetectionProbe<adapter::Detection2DHcmDescriptor, QRB::Memory::Detection2dRaw_t, HcmBox2d>
{
};

template <>
struct HcmProbeSample<adapter::Detection3DHcmDescriptor>
  : HcmDetectionProbe<adapter::Detection3DHcmDescriptor, QRB::Memory::Detection3dRaw_t, HcmBox3d>
{
};

namespace
{

constexpr std::uint64_t kNsPerSecond = 1000000000ull;
constexpr std::uint32_t kMonitoringEntries = 7;

// --- Contracts -------------------------------------------------------------------

/** @brief The probe of one Contract, whatever its descriptor type. */
struct ContractOps
{
  const char * typeName;
  std::uint64_t (*bufferBytes)(const ProbeFrameSize & size, std::string & why);
  /** False: `buffer` is not the Contract's descriptor. */
  bool (*prepare)(QC::Memory::QCBufferDescriptorBase_t & buffer, const ProbeFrameSize & size);
  std::uint64_t (*sentBytes)(const HcmSenderState &);
  /** False: `buffer` is not the Contract's descriptor. */
  bool (*stamp)(QC::Memory::QCBufferDescriptorBase_t & buffer,
      HcmSenderState & state,
      std::uint64_t sequence,
      HcmStampResult & result);
  bool (*check)(const QC::Memory::QCBufferDescriptorBase_t & buffer,
      std::uint32_t streamTag,
      FrameReading & reading,
      std::uint64_t & payloadBytes);
};

template <typename Sample, typename = void>
struct HasBufferBytes : std::false_type
{
};
template <typename Sample>
struct HasBufferBytes<Sample, std::void_t<decltype(&Sample::BufferBytes)>> : std::true_type
{
};

template <typename Sample, typename = void>
struct HasPrepare : std::false_type
{
};
template <typename Sample>
struct HasPrepare<Sample, std::void_t<decltype(&Sample::Prepare)>> : std::true_type
{
};

template <typename DescriptorT>
std::uint64_t BufferBytesOf(const ProbeFrameSize & size, std::string & why)
{
  if constexpr (HasBufferBytes<HcmProbeSample<DescriptorT>>::value)
    return HcmProbeSample<DescriptorT>::BufferBytes(size, why);
  else
    return 0;
}

template <typename DescriptorT>
bool PrepareAs(QC::Memory::QCBufferDescriptorBase_t & buffer, const ProbeFrameSize & size)
{
  auto * descriptor = dynamic_cast<DescriptorT *>(&buffer);
  if (descriptor == nullptr)
    return false;
  if constexpr (HasPrepare<HcmProbeSample<DescriptorT>>::value)
    HcmProbeSample<DescriptorT>::Prepare(*descriptor, size);
  return true;
}

template <typename DescriptorT>
bool StampAs(QC::Memory::QCBufferDescriptorBase_t & buffer,
    HcmSenderState & state,
    std::uint64_t sequence,
    HcmStampResult & result)
{
  auto * descriptor = dynamic_cast<DescriptorT *>(&buffer);
  if (descriptor == nullptr)
    return false;
  result = HcmProbeSample<DescriptorT>::Stamp(*descriptor, state, sequence);
  return true;
}

template <typename DescriptorT>
bool CheckAs(const QC::Memory::QCBufferDescriptorBase_t & buffer,
    std::uint32_t streamTag,
    FrameReading & reading,
    std::uint64_t & payloadBytes)
{
  const auto * descriptor = dynamic_cast<const DescriptorT *>(&buffer);
  if (descriptor == nullptr)
    return false;
  reading = HcmProbeSample<DescriptorT>::Check(*descriptor, streamTag, payloadBytes);
  return true;
}

const ContractOps kContracts[] = {
#define QRB_CONTRACT_MAPPING(Id, TypeName, Payload, ContractT, DescriptorT, ...)                   \
  {TypeName, &BufferBytesOf<DescriptorT>, &PrepareAs<DescriptorT>,                                 \
      &HcmProbeSample<DescriptorT>::SentBytes, &StampAs<DescriptorT>, &CheckAs<DescriptorT>},
#include "qrb_ros_adapter/ipc_runtime/contract_mappings.def"
#undef QRB_CONTRACT_MAPPING
};

const ContractOps * FindContract(const std::string & contract)
{
  for (const ContractOps & ops : kContracts) {
    if (MatchesContract(contract, ops.typeName))
      return &ops;
  }
  return nullptr;
}

std::string SupportedContracts()
{
  std::string supported;
  for (const ContractOps & ops : kContracts)
    supported += (supported.empty() ? "" : ", ") + ShortContractName(ops.typeName);
  return supported;
}

// --- Static configuration ----------------------------------------------------------

struct ProbeNodeSettings
{
  std::string name;
  std::uint32_t id{0};
  bool sender{false};
  const ContractOps * contract{nullptr};
  std::string topic{ProbeFlow{}.topic};
  double warmup{0.0};
  std::uint32_t globalBufferId{0};
};

// The host writes this config, so only the required fields and the contract
// are checked; a value of the wrong type fails the whole config.
QC::QCStatus_e ParseSettings(const std::string & json,
    ProbeNodeSettings & out,
    std::string & errors)
{
  ProbeNodeSettings settings;
  std::string contract;
  try {
    const nlohmann::json node = nlohmann::json::parse(json).at("static");
    settings.name = node.at("name").get<std::string>();
    const auto direction = node.at("direction").get<std::uint32_t>();
    contract = node.at("contract").get<std::string>();
    if (settings.name.empty() || direction > 1) {
      errors += "\"name\" must be non-empty and \"direction\" 0 or 1; ";
      return QC::QC_STATUS_BAD_ARGUMENTS;
    }
    settings.sender = static_cast<ProbeDirection>(direction) == ProbeDirection::kHcmToRos;
    settings.id = node.value("id", settings.id);
    settings.topic = node.value("topic", settings.topic);
    settings.warmup = node.value("warmup", settings.warmup);
    if (node.contains("globalBufferIdMap"))
      settings.globalBufferId = node["globalBufferIdMap"].at(0).at("id").get<std::uint32_t>();
  } catch (const nlohmann::json::exception & e) {
    errors += std::string(e.what()) + "; ";
    return QC::QC_STATUS_BAD_ARGUMENTS;
  }

  settings.contract = FindContract(contract);
  if (settings.contract == nullptr) {
    errors +=
        "unsupported \"contract\" " + contract + "; supported: " + SupportedContracts() + "; ";
    return QC::QC_STATUS_UNSUPPORTED;
  }
  out = std::move(settings);
  return QC::QC_STATUS_OK;
}

struct HcmStreamProbeConfigData : public QC::QCNodeConfigBase_t
{
  ProbeNodeSettings settings;
};

class HcmStreamProbeNodeConfig final : public QC::QCNodeConfigIfs
{
public:
  QC::QCStatus_e VerifyAndSet(const std::string config, std::string & errors) override
  {
    ProbeNodeSettings parsed;
    const QC::QCStatus_e status = ParseSettings(config, parsed, errors);
    if (status != QC::QC_STATUS_OK)
      return status;
    config_.settings = std::move(parsed);
    config_.nodeId.name = config_.settings.name;
    config_.nodeId.id = static_cast<std::uint8_t>(config_.settings.id);
    config_.nodeId.type = kHcmStreamProbeNodeType;
    config_.numOfEntries = 1;
    return QC::QC_STATUS_OK;
  }

  // Both GetOptions() forms, as in the Adapter, for either QCNode SDK.
  const std::string & GetOptions()
  {
    static const std::string options = R"({"required":["name","direction","contract"],)"
                                       R"("optional":["id","topic","warmup","globalBufferIdMap"]})";
    return options;
  }
  QC::QCStatus_e GetOptions(std::string & options)
  {
    options = GetOptions();
    return QC::QC_STATUS_OK;
  }

  const QC::QCNodeConfigBase_t & Get() override { return config_; }

  const ProbeNodeSettings & Settings() const { return config_.settings; }
  void Clear() { config_ = {}; }

private:
  HcmStreamProbeConfigData config_{};
};

// --- Monitoring --------------------------------------------------------------------

class HcmStreamProbeNodeMonitoring final : public QC::QCNodeMonitoringIfs
{
public:
  HcmStreamProbeNodeMonitoring() { Reset(false, 0.0); }

  QC::QCStatus_e VerifyAndSet(const std::string /*config*/, std::string & /*errors*/) override
  {
    return QC::QC_STATUS_OK;
  }
  const std::string & GetOptions()
  {
    static const std::string options = R"({"format":"HcmStreamProbeMonitoringData"})";
    return options;
  }
  QC::QCStatus_e GetOptions(std::string & options)
  {
    options = GetOptions();
    return QC::QC_STATUS_OK;
  }

  const QC::QCNodeMonitoringBase_t & Get() override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    SnapshotLocked(snapshot_);
    return snapshot_;
  }
  std::uint32_t GetMaximalSize() override { return sizeof(HcmStreamProbeMonitoringData); }
  std::uint32_t GetCurrentSize() override { return sizeof(HcmStreamProbeMonitoringData); }

  QC::QCStatus_e Place(void * data, std::uint32_t & size) override
  {
    if (data == nullptr)
      return QC::QC_STATUS_BAD_ARGUMENTS;
    if (size < sizeof(HcmStreamProbeMonitoringData)) {
      size = sizeof(HcmStreamProbeMonitoringData);
      return QC::QC_STATUS_OUT_OF_BOUND;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    SnapshotLocked(snapshot_);
    std::memcpy(data, &snapshot_, sizeof(snapshot_));
    size = sizeof(snapshot_);
    return QC::QC_STATUS_OK;
  }

  void Reset(bool sender, double warmupSeconds)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    sender_ = sender;
    warmupNs_ = static_cast<std::uint64_t>(warmupSeconds * static_cast<double>(kNsPerSecond));
    firstFrameNs_ = 0;
    processErrors_ = 0;
    sent_ = {};
    received_ = ReceiverStats();
    lastProblem_ = {};
    lastFrame_ = FrameRecord{};
  }

  void RecordSent(std::uint64_t sequence, std::uint64_t sendNs, std::uint64_t bytes)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (firstFrameNs_ == 0)
      firstFrameNs_ = sendNs;
    ++sent_.sent;
    sent_.bytes += bytes;
    lastFrame_ = FrameRecord{};
    lastFrame_.count = ++frames_;
    lastFrame_.sequence = sequence;
    lastFrame_.sendNs = sendNs;
    lastFrame_.bytes = bytes;
  }

  void RecordSendError()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ++sent_.sendErrors;
    ++processErrors_;
  }

  void RecordProcessError()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ++processErrors_;
  }

  void RecordReceived(const FrameReading & reading, std::uint64_t receiveNs, std::uint64_t bytes)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (firstFrameNs_ == 0)
      firstFrameNs_ = receiveNs;
    lastFrame_ =
        received_.Record(reading, receiveNs, bytes, receiveNs - firstFrameNs_ >= warmupNs_);
    lastFrame_.count = ++frames_;
    if (reading.check != FrameCheck::kOk)
      lastProblem_ = reading;
  }

private:
  void SnapshotLocked(HcmStreamProbeMonitoringData & out) const
  {
    out.numOfEntries = kMonitoringEntries;
    out.sender = sender_ ? 1u : 0u;
    out.processErrors = processErrors_;
    out.sent = sent_;
    out.received = received_.Counters();
    out.lastProblem = lastProblem_;
    out.latency = received_.Latency();
    out.lastFrame = lastFrame_;
  }

  std::mutex mutex_;
  bool sender_{false};
  std::uint64_t warmupNs_{0};
  std::uint64_t firstFrameNs_{0};
  std::uint64_t processErrors_{0};
  SenderCounters sent_;
  ReceiverStats received_;
  FrameReading lastProblem_;
  /** Not reset by Initialize(), so lastFrame.count never repeats. */
  std::uint64_t frames_{0};
  FrameRecord lastFrame_;
  HcmStreamProbeMonitoringData snapshot_{};
};

// --- Node ----------------------------------------------------------------------------

class HcmStreamProbeNode final : public QC::QCNodeIfs
{
public:
  QC::QCStatus_e Initialize(QC::QCNodeInit_t & config) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_.load() != QC::QC_OBJECT_STATE_INITIAL)
      return QC::QC_STATUS_BAD_STATE;

    state_.store(QC::QC_OBJECT_STATE_INITIALIZING);
    std::string errors;
    const QC::QCStatus_e status = config_.VerifyAndSet(config.config, errors);
    if (status != QC::QC_STATUS_OK) {
      config_.Clear();
      state_.store(QC::QC_OBJECT_STATE_INITIAL);
      return status;
    }
    const ProbeNodeSettings & settings = config_.Settings();
    monitoring_.Reset(settings.sender, settings.warmup);
    // The sequence goes on across Initialize(): a receiver that keeps
    // running sees one stream, not a restart it would count as reordered.
    senderState_ = HcmSenderState{};
    senderState_.streamTag = StreamTagOf(settings.topic);
    state_.store(QC::QC_OBJECT_STATE_READY);
    return QC::QC_STATUS_OK;
  }

  QC::QCStatus_e Start() override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_.load() != QC::QC_OBJECT_STATE_READY)
      return QC::QC_STATUS_BAD_STATE;
    state_.store(QC::QC_OBJECT_STATE_RUNNING);
    return QC::QC_STATUS_OK;
  }

  QC::QCStatus_e ProcessFrameDescriptor(QC::QCFrameDescriptorNodeIfs & frameDesc) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_.load() != QC::QC_OBJECT_STATE_RUNNING)
      return QC::QC_STATUS_BAD_STATE;

    const ProbeNodeSettings & settings = config_.Settings();
    QC::Memory::QCBufferDescriptorBase_t & buffer = frameDesc.GetBuffer(settings.globalBufferId);
    if (settings.sender) {
      HcmStampResult result = HcmStampResult::kOk;
      if (!settings.contract->stamp(buffer, senderState_, sequence_, result)) {
        monitoring_.RecordProcessError();
        return QC::QC_STATUS_BAD_ARGUMENTS;
      }
      if (result != HcmStampResult::kOk) {
        monitoring_.RecordSendError();
        return result == HcmStampResult::kBadBuffer ? QC::QC_STATUS_INVALID_BUF :
                                                      QC::QC_STATUS_FAIL;
      }
      monitoring_.RecordSent(
          sequence_, senderState_.lastSendNs, settings.contract->sentBytes(senderState_));
      ++sequence_;
      return QC::QC_STATUS_OK;
    }

    const std::uint64_t receiveNs = MonotonicNowNs();
    FrameReading reading;
    std::uint64_t bytes = 0;
    if (!settings.contract->check(buffer, senderState_.streamTag, reading, bytes)) {
      monitoring_.RecordProcessError();
      return QC::QC_STATUS_BAD_ARGUMENTS;
    }
    // A wrong frame is a finding of the probe, not a failure of the call.
    monitoring_.RecordReceived(reading, receiveNs, bytes);
    return QC::QC_STATUS_OK;
  }

  QC::QCStatus_e Stop() override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_.load() != QC::QC_OBJECT_STATE_RUNNING)
      return QC::QC_STATUS_BAD_STATE;
    state_.store(QC::QC_OBJECT_STATE_READY);
    return QC::QC_STATUS_OK;
  }

  QC::QCStatus_e DeInitialize() override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_.load() != QC::QC_OBJECT_STATE_READY)
      return QC::QC_STATUS_BAD_STATE;
    state_.store(QC::QC_OBJECT_STATE_INITIAL);
    return QC::QC_STATUS_OK;
  }

  QC::QCObjectState_e GetState() override { return state_.load(); }
  QC::QCNodeConfigIfs & GetConfigurationIfs() override { return config_; }
  QC::QCNodeMonitoringIfs & GetMonitoringIfs() override { return monitoring_; }

private:
  std::mutex mutex_;
  std::atomic<QC::QCObjectState_e> state_{QC::QC_OBJECT_STATE_INITIAL};
  HcmStreamProbeNodeConfig config_;
  HcmStreamProbeNodeMonitoring monitoring_;
  HcmSenderState senderState_;
  std::uint64_t sequence_{0};
};

// Register the probe with QCNodeFactory at static-initialization time.
const QC::Node::NodeRegistrar<HcmStreamProbeNode> g_registerStreamProbe{kHcmStreamProbeNodeType};

}  // namespace

std::uint64_t HcmProbeBufferBytes(const std::string & contract,
    const ProbeFrameSize & size,
    std::string & why)
{
  why.clear();
  const ContractOps * ops = FindContract(contract);
  if (ops == nullptr) {
    why = "unsupported contract " + contract + "; supported: " + SupportedContracts();
    return 0;
  }
  const std::uint64_t bytes = ops->bufferBytes(size, why);
  if (why.empty() && bytes > std::numeric_limits<std::uint32_t>::max())
    why = "a frame of " + std::to_string(bytes) + " bytes is too large";
  return why.empty() ? bytes : 0;
}

bool HcmProbePrepareFrame(const std::string & contract,
    QC::Memory::QCBufferDescriptorBase_t & descriptor,
    const ProbeFrameSize & size)
{
  const ContractOps * ops = FindContract(contract);
  return ops != nullptr && ops->prepare(descriptor, size);
}

}  // namespace qrb::adapter::stream_probe
