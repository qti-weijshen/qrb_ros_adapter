// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include <gtest/gtest.h>
#include <unistd.h>

#include <cmath>
#include <cstring>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "qrb_ros_adapter/hcm_ipc_adapter/flow_client/detection_2d_hcm_converter.hpp"
#include "qrb_ros_adapter/hcm_ipc_adapter/flow_client/detection_3d_hcm_converter.hpp"

using namespace qrb::adapter;

namespace vision_proto = ::qrb::contract::vision::v1;

namespace
{

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

/**
 * @brief A detection descriptor over heap storage, which the converters read and fill like host
 * memory.
 *
 * @p offset is not a multiple of the alignment of a record, to show that the converters do not
 * dereference records in place.
 */
template <typename DescriptorT, typename RecordT>
struct HeapDetectionDescriptor
{
  explicit HeapDetectionDescriptor(std::size_t records, std::size_t offset = 0)
    : storage(offset + records * sizeof(RecordT), 0xCD)
  {
    descriptor.pBuf = storage.data();
    descriptor.size = storage.size();
    descriptor.offset = offset;
    descriptor.allocatorType = QC::Memory::QC_MEMORY_ALLOCATOR_HEAP;
  }

  RecordT Record(std::size_t index) const
  {
    RecordT record;
    std::memcpy(
        &record, storage.data() + descriptor.offset + index * sizeof(RecordT), sizeof(RecordT));
    return record;
  }

  std::vector<std::uint8_t> storage;
  DescriptorT descriptor;
};

void FillHypothesis(vision_proto::ObjectHypothesisWithPose * result,
    const char * classId,
    double score,
    bool withCovariance)
{
  result->mutable_hypothesis()->set_class_id(classId);
  result->mutable_hypothesis()->set_score(score);
  result->mutable_pose()->mutable_pose()->mutable_position()->set_x(0.5);
  result->mutable_pose()->mutable_pose()->mutable_orientation()->set_z(0.6);
  result->mutable_pose()->mutable_pose()->mutable_orientation()->set_w(0.8);
  if (withCovariance) {
    for (int i = 0; i < 36; ++i)
      result->mutable_pose()->add_covariance(0.25 * i);
  }
}

Detection2DHcmContract MakeContract2d()
{
  Detection2DHcmContract array;
  array.mutable_header()->mutable_stamp()->set_sec(12);
  array.mutable_header()->mutable_stamp()->set_nanosec(345);
  array.mutable_header()->set_frame_id("camera");
  auto * first = array.add_detections();
  first->set_id("track-1");
  first->mutable_bbox()->mutable_center()->mutable_position()->set_x(320.5);
  first->mutable_bbox()->mutable_center()->mutable_position()->set_y(240.25);
  first->mutable_bbox()->mutable_center()->set_theta(0.125);
  first->mutable_bbox()->set_size_x(64.0);
  first->mutable_bbox()->set_size_y(32.0);
  FillHypothesis(first->add_results(), "person", 0.9, false);
  FillHypothesis(first->add_results(), "mannequin", 0.1, true);
  auto * second = array.add_detections();
  // The descriptor always holds a center, so the Contract made from it always has one.
  second->mutable_bbox()->mutable_center()->mutable_position();
  second->mutable_bbox()->set_size_x(8.0);
  FillHypothesis(second->add_results(), "cup", 0.5, false);
  return array;
}

Detection3DHcmContract MakeContract3d()
{
  Detection3DHcmContract array;
  array.mutable_header()->mutable_stamp()->set_sec(7);
  array.mutable_header()->set_frame_id("lidar");
  auto * detection = array.add_detections();
  detection->set_id("car-7");
  detection->mutable_bbox()->mutable_center()->mutable_position()->set_x(10.0);
  detection->mutable_bbox()->mutable_center()->mutable_position()->set_z(0.75);
  detection->mutable_bbox()->mutable_center()->mutable_orientation()->set_z(0.6);
  detection->mutable_bbox()->mutable_center()->mutable_orientation()->set_w(0.8);
  detection->mutable_bbox()->mutable_size()->set_x(4.5);
  detection->mutable_bbox()->mutable_size()->set_y(2.0);
  detection->mutable_bbox()->mutable_size()->set_z(1.5);
  FillHypothesis(detection->add_results(), "car", 0.8, true);
  return array;
}

using Heap2d = HeapDetectionDescriptor<Detection2DHcmDescriptor, QRB::Memory::Detection2dRaw_t>;
using Heap3d = HeapDetectionDescriptor<Detection3DHcmDescriptor, QRB::Memory::Detection3dRaw_t>;

/** @brief Gives the descriptor a stale frame, stamp and count to compare after a rejection. */
void MarkUntouched(Detection2DHcmDescriptor & descriptor)
{
  ASSERT_EQ(descriptor.frameId.Assign("untouched"), QC::QC_STATUS_OK);
  descriptor.timestampNs = 4242;
  descriptor.objCount = 17;
  descriptor.validSize = 99;
}

void ExpectUntouched(const Heap2d & heap)
{
  EXPECT_TRUE(heap.descriptor.frameId == "untouched");
  EXPECT_EQ(heap.descriptor.timestampNs, 4242u);
  EXPECT_EQ(heap.descriptor.objCount, 17u);
  EXPECT_EQ(heap.descriptor.validSize, 99u);
  for (std::uint8_t byte : heap.storage)
    ASSERT_EQ(byte, 0xCD);
}

}  // namespace

// Detection2DArray <-> the records in the buffer of the descriptor keep every field,
// at an unaligned buffer offset too; a known covariance is carried and an all-zero one is left
// out of the Contract.
TEST(HcmDetectionConverters, RoundTripEveryFieldOf2dDetections)
{
  const Detection2DHcmContract source = MakeContract2d();
  Heap2d heap(4, 4);
  ASSERT_TRUE((ConvertContractToHcm<Detection2DHcmContract, Detection2DHcmDescriptor>(
      source, DmaBufView{}, heap.descriptor)));

  EXPECT_EQ(heap.descriptor.timestampNs, 12000000345u);
  EXPECT_TRUE(heap.descriptor.frameId == "camera");
  EXPECT_EQ(heap.descriptor.objCount, 2u);
  EXPECT_EQ(heap.descriptor.validSize, 2 * sizeof(QRB::Memory::Detection2dRaw_t));
  EXPECT_EQ(heap.descriptor.type, QC::QC_BUFFER_TYPE_CUSTOM_2);

  const QRB::Memory::Detection2dRaw_t first = heap.Record(0);
  EXPECT_STREQ(first.id, "track-1");
  EXPECT_DOUBLE_EQ(first.bbox.center.position.x, 320.5);
  EXPECT_DOUBLE_EQ(first.bbox.center.position.y, 240.25);
  EXPECT_DOUBLE_EQ(first.bbox.center.theta, 0.125);
  EXPECT_DOUBLE_EQ(first.bbox.sizeX, 64.0);
  EXPECT_DOUBLE_EQ(first.bbox.sizeY, 32.0);
  ASSERT_EQ(first.resultCount, 2u);
  EXPECT_STREQ(first.results[0].classId, "person");
  EXPECT_DOUBLE_EQ(first.results[0].score, 0.9);
  EXPECT_DOUBLE_EQ(first.results[0].pose.pose.orientation.w, 0.8);
  EXPECT_DOUBLE_EQ(first.results[0].pose.covariance[35], 0.0);  // unknown reads as zeros
  EXPECT_STREQ(first.results[1].classId, "mannequin");
  EXPECT_DOUBLE_EQ(first.results[1].pose.covariance[35], 8.75);
  const QRB::Memory::Detection2dRaw_t second = heap.Record(1);
  EXPECT_STREQ(second.id, "");
  EXPECT_EQ(second.resultCount, 1u);
  // The record past objCount is not written.
  EXPECT_EQ(heap.storage[heap.descriptor.offset + 2 * sizeof(QRB::Memory::Detection2dRaw_t)], 0xCD);

  Detection2DHcmContract back;
  back.mutable_header()->set_frame_id("stale");
  back.add_detections()->set_id("stale");
  ASSERT_TRUE((ConvertHcmToContract<Detection2DHcmContract, Detection2DHcmDescriptor>(
      heap.descriptor, back)));
  EXPECT_EQ(back.SerializeAsString(), source.SerializeAsString());
  EXPECT_EQ(back.detections(0).results(0).pose().covariance_size(), 0);
  EXPECT_EQ(back.detections(0).results(1).pose().covariance_size(), 36);

  // An empty array is a result and needs no buffer at all.
  Detection2DHcmContract empty;
  empty.mutable_header()->set_frame_id("camera");
  Detection2DHcmDescriptor bufferless;
  bufferless.objCount = 9;
  ASSERT_TRUE((ConvertContractToHcm<Detection2DHcmContract, Detection2DHcmDescriptor>(
      empty, DmaBufView{}, bufferless)));
  EXPECT_EQ(bufferless.objCount, 0u);
  EXPECT_EQ(bufferless.validSize, 0u);
  Detection2DHcmContract emptyBack;
  ASSERT_TRUE((ConvertHcmToContract<Detection2DHcmContract, Detection2DHcmDescriptor>(
      bufferless, emptyBack)));
  EXPECT_EQ(emptyBack.detections_size(), 0);
  EXPECT_EQ(emptyBack.header().frame_id(), "camera");
}

// A Contract the descriptor cannot hold is rejected whole: the descriptor and its buffer are not
// touched, and nothing is truncated.
TEST(HcmDetectionConverters, RejectContractsThatDoNotFitTheDescriptorWithoutTouchingIt)
{
  const auto rejected = [](const char * what, auto && mutate, std::size_t capacity = 4) {
    Detection2DHcmContract contract = MakeContract2d();
    mutate(contract);
    Heap2d heap(capacity, 4);
    MarkUntouched(heap.descriptor);
    const ConversionResult result =
        ConvertContractToHcm<Detection2DHcmContract, Detection2DHcmDescriptor>(
            contract, DmaBufView{}, heap.descriptor);
    EXPECT_FALSE(result) << what;
    EXPECT_FALSE(result.Diagnostic().empty()) << what;
    ExpectUntouched(heap);
  };
  rejected("long id", [](auto & c) {
    c.mutable_detections(1)->set_id(std::string(QRB_DETECTION_MAX_ID_LEN, 'i'));
  });
  rejected("long class", [](auto & c) {
    c.mutable_detections(1)->mutable_results(0)->mutable_hypothesis()->set_class_id(
        std::string(QRB_DETECTION_MAX_CLASS_ID_LEN, 'c'));
  });
  rejected("too many hypotheses", [](auto & c) {
    for (int i = 0; i < QRB_DETECTION_MAX_RESULTS; ++i)
      FillHypothesis(c.mutable_detections(1)->add_results(), "x", 0.1, false);
  });
  rejected("long frame", [](auto & c) {
    c.mutable_header()->set_frame_id(std::string(QRB::Memory::QRB_FRAME_ID_MAX_LEN + 1, 'f'));
  });
  rejected("negative stamp", [](auto & c) { c.mutable_header()->mutable_stamp()->set_sec(-1); });
  rejected(
      "not valid", [](auto & c) { c.mutable_detections(0)->mutable_bbox()->set_size_x(kNaN); });
  rejected("buffer holds one detection", [](auto &) {}, 1);

  // The longest strings that fit are kept whole.
  Detection2DHcmContract longest = MakeContract2d();
  longest.mutable_detections(0)->set_id(std::string(QRB_DETECTION_MAX_ID_LEN - 1, 'i'));
  longest.mutable_detections(0)->mutable_results(0)->mutable_hypothesis()->set_class_id(
      std::string(QRB_DETECTION_MAX_CLASS_ID_LEN - 1, 'c'));
  Heap2d heap(2);
  ASSERT_TRUE((ConvertContractToHcm<Detection2DHcmContract, Detection2DHcmDescriptor>(
      longest, DmaBufView{}, heap.descriptor)));
  EXPECT_EQ(std::strlen(heap.Record(0).id), QRB_DETECTION_MAX_ID_LEN - 1u);
  EXPECT_EQ(std::strlen(heap.Record(0).results[0].classId), QRB_DETECTION_MAX_CLASS_ID_LEN - 1u);

  // Detections need a buffer this process can write: none, or a DMA-BUF of another process, is
  // refused.
  Detection2DHcmDescriptor bufferless;
  EXPECT_FALSE((ConvertContractToHcm<Detection2DHcmContract, Detection2DHcmDescriptor>(
      MakeContract2d(), DmaBufView{}, bufferless)));
  Heap2d foreign(2);
  foreign.descriptor.allocatorType = QC::Memory::QC_MEMORY_ALLOCATOR_DMA;
  foreign.descriptor.dmaHandle = 5;
  foreign.descriptor.pid = static_cast<decltype(foreign.descriptor.pid)>(::getpid()) + 1;
  EXPECT_FALSE((ConvertContractToHcm<Detection2DHcmContract, Detection2DHcmDescriptor>(
      MakeContract2d(), DmaBufView{}, foreign.descriptor)));
}

// A descriptor whose records cannot be read safely, or hold values ROS must not get, is not sent.
TEST(HcmDetectionConverters, RejectDescriptorsWhoseRecordsCannotBeReadOrSent)
{
  const auto filled = [](std::size_t records = 4) {
    auto heap = std::make_unique<Heap2d>(records, 4);
    EXPECT_TRUE((ConvertContractToHcm<Detection2DHcmContract, Detection2DHcmDescriptor>(
        MakeContract2d(), DmaBufView{}, heap->descriptor)));
    return heap;
  };
  const auto convert = [](Heap2d & heap) {
    Detection2DHcmContract out;
    return ConvertHcmToContract<Detection2DHcmContract, Detection2DHcmDescriptor>(
        heap.descriptor, out);
  };

  auto heap = filled();
  EXPECT_TRUE(convert(*heap));

  heap = filled();
  heap->descriptor.objCount = 5;  // more records than the buffer holds
  EXPECT_FALSE(convert(*heap));
  heap = filled();
  heap->descriptor.objCount = std::numeric_limits<std::uint32_t>::max();
  EXPECT_FALSE(convert(*heap));
  heap = filled();
  heap->descriptor.pBuf = nullptr;
  EXPECT_FALSE(convert(*heap));
  heap = filled();
  heap->descriptor.offset = heap->descriptor.size + 1;
  EXPECT_FALSE(convert(*heap));

  heap = filled();  // resultCount beyond the records' capacity
  QRB::Memory::Detection2dRaw_t record = heap->Record(0);
  record.resultCount = QRB_DETECTION_MAX_RESULTS + 1;
  std::memcpy(heap->storage.data() + heap->descriptor.offset, &record, sizeof(record));
  EXPECT_FALSE(convert(*heap));

  heap = filled();  // a string without terminating NUL
  record = heap->Record(0);
  std::memset(record.id, 'x', sizeof(record.id));
  std::memcpy(heap->storage.data() + heap->descriptor.offset, &record, sizeof(record));
  EXPECT_FALSE(convert(*heap));
  heap = filled();
  record = heap->Record(0);
  std::memset(record.results[0].classId, 'x', sizeof(record.results[0].classId));
  std::memcpy(heap->storage.data() + heap->descriptor.offset, &record, sizeof(record));
  EXPECT_FALSE(convert(*heap));

  heap = filled();  // a non-finite value from the HCM side must not be published to ROS
  record = heap->Record(0);
  record.bbox.sizeX = kNaN;
  std::memcpy(heap->storage.data() + heap->descriptor.offset, &record, sizeof(record));
  EXPECT_FALSE(convert(*heap));
  heap = filled();
  record = heap->Record(0);
  record.results[1].pose.covariance[3] = std::numeric_limits<double>::infinity();
  std::memcpy(heap->storage.data() + heap->descriptor.offset, &record, sizeof(record));
  EXPECT_FALSE(convert(*heap));

  // A timestamp whose seconds do not fit the Contract's int32.
  heap = filled();
  heap->descriptor.timestampNs = 4000000000ULL * 1000000000ULL;
  EXPECT_FALSE(convert(*heap));
}

// Detection3DArray <-> the records in the buffer of the descriptor keep every field, and a
// negative box size or a buffer too small for the detections is rejected.
TEST(HcmDetectionConverters, RoundTripEveryFieldOf3dDetections)
{
  const Detection3DHcmContract source = MakeContract3d();
  Heap3d heap(2, 4);
  ASSERT_TRUE((ConvertContractToHcm<Detection3DHcmContract, Detection3DHcmDescriptor>(
      source, DmaBufView{}, heap.descriptor)));
  EXPECT_EQ(heap.descriptor.timestampNs, 7000000000u);
  EXPECT_TRUE(heap.descriptor.frameId == "lidar");
  EXPECT_EQ(heap.descriptor.objCount, 1u);
  EXPECT_EQ(heap.descriptor.validSize, sizeof(QRB::Memory::Detection3dRaw_t));

  const QRB::Memory::Detection3dRaw_t record = heap.Record(0);
  EXPECT_STREQ(record.id, "car-7");
  EXPECT_DOUBLE_EQ(record.bbox.center.position.z, 0.75);
  EXPECT_DOUBLE_EQ(record.bbox.center.orientation.w, 0.8);
  EXPECT_DOUBLE_EQ(record.bbox.size.x, 4.5);
  EXPECT_DOUBLE_EQ(record.bbox.size.z, 1.5);
  ASSERT_EQ(record.resultCount, 1u);
  EXPECT_STREQ(record.results[0].classId, "car");
  EXPECT_DOUBLE_EQ(record.results[0].pose.covariance[35], 8.75);

  Detection3DHcmContract back;
  ASSERT_TRUE((ConvertHcmToContract<Detection3DHcmContract, Detection3DHcmDescriptor>(
      heap.descriptor, back)));
  EXPECT_EQ(back.SerializeAsString(), source.SerializeAsString());

  Detection3DHcmContract negative = source;
  negative.mutable_detections(0)->mutable_bbox()->mutable_size()->set_x(-1.0);
  Heap3d untouched(2);
  EXPECT_FALSE((ConvertContractToHcm<Detection3DHcmContract, Detection3DHcmDescriptor>(
      negative, DmaBufView{}, untouched.descriptor)));
  EXPECT_EQ(untouched.descriptor.objCount, 0u);
  Heap3d tooSmall(0);
  EXPECT_FALSE((ConvertContractToHcm<Detection3DHcmContract, Detection3DHcmDescriptor>(
      source, DmaBufView{}, tooSmall.descriptor)));
}
