// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_ADAPTER_TEST_HCM_PROBE_NODE_HPP
#define QRB_ROS_ADAPTER_TEST_HCM_PROBE_NODE_HPP

//
// HcmStreamProbeNode, the HCM side probe: a standard QCNode that produces
// (HCM to ROS) or consumes and checks (ROS to HCM) the Contract descriptor of
// a frame it passes to the HCM IPC Adapter. It is built into libQCNodeSample.so
// by the qcnode build and registered with QCNodeFactory as
// kHcmStreamProbeNodeType; SampleStreamProbe hosts it. This header gives hosts
// the layout of its monitoring data and the size of the frames it takes.
//
// A sender fills the buffer of each frame: a point cloud over the whole buffer,
// an image of the width and height HcmProbePrepareFrame() gave the descriptor.
// A receiver takes a point cloud or an image of any size that fits the buffer.
//
// Static configuration, the QCNode `"static"` block:
//
//   {
//     "static": {
//       "name": "stream_probe",
//       "id": 0,
//       "direction": 0,                // 0: sends (HCM to ROS), 1: receives
//       "contract": "PointCloud2",     // short or full Contract name
//       "topic": "/qrb_ros_adapter/stream_probe/points_h2r",   // only for the stream tag
//       "warmup": 0.0,                 // seconds a receiver does not measure
//       "globalBufferIdMap": [ { "name": "Probe", "id": 0 } ]
//     }
//   }
//
// Only `name`, `direction` and `contract` are required.
//

#include <cstdint>
#include <string>
#include <type_traits>

#include "QC/Node/Ifs/QCNodeIfs.hpp"
#include "QC/Node/Ifs/QCNodeMonitoringIfs.hpp"
#include "probe.hpp"

namespace qrb::adapter::stream_probe
{

/**
 * @brief The probe's type in QCNodeFactory. In libQCNode.so the Demo node is
 * CUSTOM_0 and the IPC Adapter Node is CUSTOM_3.
 */
constexpr QC::QCNodeType_e kHcmStreamProbeNodeType = QC::QC_NODE_TYPE_CUSTOM_2;

/** @brief What GetMonitoringIfs().Place() copies out. Reset by Initialize(). */
struct HcmStreamProbeMonitoringData : public QC::QCNodeMonitoringBase_t
{
  /** 1: the node sends, 0: it receives. */
  std::uint32_t sender{0};
  /** ProcessFrameDescriptor() calls that failed, for either role. */
  std::uint64_t processErrors{0};
  SenderCounters sent;
  ReceiverCounters received;
  /** The latest frame a receiver found wrong (verify error or foreign stream). */
  FrameReading lastProblem;
  /** The latest frame sent or received; its `count` tells a new frame from the last one seen. */
  FrameRecord lastFrame;
  /** End-to-end latency of the frames a receiver measured. */
  LatencyHistogram latency;
};
static_assert(std::is_trivially_copyable_v<HcmStreamProbeMonitoringData>,
    "Place() copies the monitoring data with memcpy");

/**
 * @brief Bytes of the buffer of a frame of `contract` of `size`, which a host
 * sends or receives into. 0 for a Contract whose frames need no buffer, or
 * with `why` when `size` does not suit.
 */
std::uint64_t HcmProbeBufferBytes(const std::string & contract,
    const ProbeFrameSize & size,
    std::string & why);

/**
 * @brief Ready the descriptor of a frame of `contract` before the host offers
 * it to the probe or to the Adapter: an image takes the width and height of
 * `size` and no layout. @return false if `descriptor` is not the Contract's.
 */
bool HcmProbePrepareFrame(const std::string & contract,
    QC::Memory::QCBufferDescriptorBase_t & descriptor,
    const ProbeFrameSize & size);

}  // namespace qrb::adapter::stream_probe

#endif  // QRB_ROS_ADAPTER_TEST_HCM_PROBE_NODE_HPP
