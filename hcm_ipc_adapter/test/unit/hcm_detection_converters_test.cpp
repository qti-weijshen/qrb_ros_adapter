// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include <gtest/gtest.h>
#include <unistd.h>

#include <cstring>
#include <limits>
#include <string>
#include <vector>

#include "qrb_ros_adapter/hcm_ipc_adapter/flow_client/detection_2d_hcm_converter.hpp"
#include "qrb_ros_adapter/hcm_ipc_adapter/flow_client/detection_3d_hcm_converter.hpp"

using namespace qrb::adapter;

namespace vision_proto = ::qrb::contract::vision::v1;

namespace
{

/**
 * @brief A detection descriptor over heap storage, which the converters read and fill like host
 * memory; an @p offset that is not a multiple of the record alignment is allowed.
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

  void Store(std::size_t index, const RecordT & record)
  {
    std::memcpy(
        storage.data() + descriptor.offset + index * sizeof(RecordT), &record, sizeof(RecordT));
  }

  std::vector<std::uint8_t> storage;
  DescriptorT descriptor;
};

using Heap2d = HeapDetectionDescriptor<Detection2DHcmDescriptor, QRB::Memory::Detection2dRaw_t>;
using Heap3d = HeapDetectionDescriptor<Detection3DHcmDescriptor, QRB::Memory::Detection3dRaw_t>;

void FillHypothesis(vision_proto::ObjectHypothesisWithPose * result,
    const char * classId,
    double score,
    bool withCovariance)
{
  result->mutable_hypothesis()->set_class_id(classId);
  result->mutable_hypothesis()->set_score(score);
  auto * pose = result->mutable_pose()->mutable_pose();
  pose->mutable_position()->set_x(0.5);
  pose->mutable_position()->set_y(-0.75);
  pose->mutable_position()->set_z(1.25);
  pose->mutable_orientation()->set_x(0.1);
  pose->mutable_orientation()->set_y(0.2);
  pose->mutable_orientation()->set_z(0.3);
  pose->mutable_orientation()->set_w(0.9);
  if (withCovariance) {
    for (int i = 0; i < 36; ++i)
      result->mutable_pose()->add_covariance(0.25 * (i + 1));
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
  second->mutable_bbox()->mutable_center()->mutable_position()->set_x(16.5);
  second->mutable_bbox()->mutable_center()->mutable_position()->set_y(24.75);
  second->mutable_bbox()->mutable_center()->set_theta(-0.5);
  second->mutable_bbox()->set_size_x(8.0);
  second->mutable_bbox()->set_size_y(6.0);
  FillHypothesis(second->add_results(), "cup", 0.5, false);
  return array;
}

}  // namespace

// Detection2DArray round trips through the records at an unaligned buffer offset, the empty id
// and an all-zero covariance left out included; an empty array needs no buffer.
TEST(HcmDetectionConverters, Detection2dRoundTripsThroughTheRecords)
{
  const Detection2DHcmContract source = MakeContract2d();
  Heap2d heap(4, 4);
  ASSERT_TRUE((ConvertContractToHcm<Detection2DHcmContract, Detection2DHcmDescriptor>(
      source, DmaBufView{}, heap.descriptor)));
  EXPECT_EQ(heap.descriptor.objCount, 2u);
  const QRB::Memory::Detection2dRaw_t first = heap.Record(0);
  EXPECT_DOUBLE_EQ(first.bbox.center.position.y, 240.25);
  EXPECT_DOUBLE_EQ(first.bbox.sizeY, 32.0);
  EXPECT_STREQ(first.results[1].classId, "mannequin");
  EXPECT_DOUBLE_EQ(first.results[1].pose.covariance[35], 9.0);
  Detection2DHcmContract back;
  ASSERT_TRUE((ConvertHcmToContract<Detection2DHcmContract, Detection2DHcmDescriptor>(
      heap.descriptor, back)));
  EXPECT_EQ(back.SerializeAsString(), source.SerializeAsString());

  Detection2DHcmContract empty;
  empty.mutable_header()->mutable_stamp()->set_sec(3);
  empty.mutable_header()->set_frame_id("camera");
  Detection2DHcmDescriptor bufferless;
  bufferless.objCount = 9;
  ASSERT_TRUE((ConvertContractToHcm<Detection2DHcmContract, Detection2DHcmDescriptor>(
      empty, DmaBufView{}, bufferless)));
  EXPECT_EQ(bufferless.objCount, 0u);
  ASSERT_TRUE(
      (ConvertHcmToContract<Detection2DHcmContract, Detection2DHcmDescriptor>(bufferless, back)));
  EXPECT_EQ(back.SerializeAsString(), empty.SerializeAsString());
}

// A Contract the records cannot hold, or a buffer this process cannot write, is rejected and the
// descriptor is left untouched; the longest strings that fit are kept whole. Records that cannot be
// read safely are not sent to ROS.
TEST(HcmDetectionConverters, Detection2dRejectsWhatTheRecordsCannotHold)
{
  Detection2DHcmContract longId = MakeContract2d();
  longId.mutable_detections(1)->set_id(std::string(QRB_DETECTION_MAX_ID_LEN, 'i'));
  Detection2DHcmContract longClass = MakeContract2d();
  longClass.mutable_detections(1)->mutable_results(0)->mutable_hypothesis()->set_class_id(
      std::string(QRB_DETECTION_MAX_CLASS_ID_LEN, 'c'));
  Detection2DHcmContract tooManyResults = MakeContract2d();
  for (int i = 0; i < QRB_DETECTION_MAX_RESULTS; ++i)
    FillHypothesis(tooManyResults.mutable_detections(1)->add_results(), "x", 0.1, false);
  Detection2DHcmContract invalid = MakeContract2d();
  invalid.mutable_detections(0)->mutable_bbox()->set_size_x(-1.0);
  for (const Detection2DHcmContract * rejected : {&longId, &longClass, &tooManyResults, &invalid}) {
    Heap2d heap(4, 4);
    heap.descriptor.objCount = 17;
    EXPECT_FALSE((ConvertContractToHcm<Detection2DHcmContract, Detection2DHcmDescriptor>(
        *rejected, DmaBufView{}, heap.descriptor)));
    EXPECT_EQ(heap.descriptor.objCount, 17u);
    EXPECT_EQ(heap.storage, std::vector<std::uint8_t>(heap.storage.size(), 0xCD));
  }
  Heap2d oneRecord(1);
  EXPECT_FALSE((ConvertContractToHcm<Detection2DHcmContract, Detection2DHcmDescriptor>(
      MakeContract2d(), DmaBufView{}, oneRecord.descriptor)));
  EXPECT_EQ(oneRecord.storage, std::vector<std::uint8_t>(oneRecord.storage.size(), 0xCD));
  Detection2DHcmDescriptor bufferless;
  EXPECT_FALSE((ConvertContractToHcm<Detection2DHcmContract, Detection2DHcmDescriptor>(
      MakeContract2d(), DmaBufView{}, bufferless)));
  Heap2d foreign(2);
  foreign.descriptor.allocatorType = QC::Memory::QC_MEMORY_ALLOCATOR_DMA;
  foreign.descriptor.dmaHandle = 5;
  foreign.descriptor.pid = static_cast<decltype(foreign.descriptor.pid)>(::getpid()) + 1;
  EXPECT_FALSE((ConvertContractToHcm<Detection2DHcmContract, Detection2DHcmDescriptor>(
      MakeContract2d(), DmaBufView{}, foreign.descriptor)));

  Detection2DHcmContract longest = MakeContract2d();
  longest.mutable_detections(0)->set_id(std::string(QRB_DETECTION_MAX_ID_LEN - 1, 'i'));
  longest.mutable_detections(0)->mutable_results(0)->mutable_hypothesis()->set_class_id(
      std::string(QRB_DETECTION_MAX_CLASS_ID_LEN - 1, 'c'));
  Heap2d heap(2);
  ASSERT_TRUE((ConvertContractToHcm<Detection2DHcmContract, Detection2DHcmDescriptor>(
      longest, DmaBufView{}, heap.descriptor)));
  EXPECT_EQ(std::strlen(heap.Record(0).id), QRB_DETECTION_MAX_ID_LEN - 1u);
  EXPECT_EQ(std::strlen(heap.Record(0).results[0].classId), QRB_DETECTION_MAX_CLASS_ID_LEN - 1u);

  const QRB::Memory::Detection2dRaw_t good = heap.Record(0);
  QRB::Memory::Detection2dRaw_t tooManyStored = good;
  tooManyStored.resultCount = QRB_DETECTION_MAX_RESULTS + 1;
  QRB::Memory::Detection2dRaw_t unterminated = good;
  std::memset(unterminated.id, 'x', sizeof(unterminated.id));
  Detection2DHcmContract out;
  for (const QRB::Memory::Detection2dRaw_t * record : {&tooManyStored, &unterminated}) {
    heap.Store(0, *record);
    EXPECT_FALSE((ConvertHcmToContract<Detection2DHcmContract, Detection2DHcmDescriptor>(
        heap.descriptor, out)));
  }
  heap.Store(0, good);
  heap.descriptor.objCount = 3;
  EXPECT_FALSE((ConvertHcmToContract<Detection2DHcmContract, Detection2DHcmDescriptor>(
      heap.descriptor, out)));
  heap.descriptor.objCount = 2;
  heap.descriptor.offset = heap.descriptor.size + 1;
  EXPECT_FALSE((ConvertHcmToContract<Detection2DHcmContract, Detection2DHcmDescriptor>(
      heap.descriptor, out)));
}

// Detection3DArray round trips through the records, the box center and size kept apart.
TEST(HcmDetectionConverters, Detection3dRoundTripsThroughTheRecords)
{
  Detection3DHcmContract source;
  source.mutable_header()->mutable_stamp()->set_sec(7);
  source.mutable_header()->mutable_stamp()->set_nanosec(8);
  source.mutable_header()->set_frame_id("lidar");
  auto * detection = source.add_detections();
  detection->set_id("car-7");
  auto * center = detection->mutable_bbox()->mutable_center();
  center->mutable_position()->set_x(10.0);
  center->mutable_position()->set_y(-3.5);
  center->mutable_position()->set_z(0.75);
  center->mutable_orientation()->set_x(0.1);
  center->mutable_orientation()->set_y(0.2);
  center->mutable_orientation()->set_z(0.3);
  center->mutable_orientation()->set_w(0.9);
  detection->mutable_bbox()->mutable_size()->set_x(4.5);
  detection->mutable_bbox()->mutable_size()->set_y(2.0);
  detection->mutable_bbox()->mutable_size()->set_z(1.5);
  FillHypothesis(detection->add_results(), "car", 0.8, true);

  Heap3d heap(2, 4);
  ASSERT_TRUE((ConvertContractToHcm<Detection3DHcmContract, Detection3DHcmDescriptor>(
      source, DmaBufView{}, heap.descriptor)));
  EXPECT_EQ(heap.descriptor.timestampNs, 7000000008u);
  const QRB::Memory::Detection3dRaw_t record = heap.Record(0);
  EXPECT_DOUBLE_EQ(record.bbox.center.position.z, 0.75);
  EXPECT_DOUBLE_EQ(record.bbox.center.orientation.x, 0.1);
  EXPECT_DOUBLE_EQ(record.bbox.size.z, 1.5);
  Detection3DHcmContract back;
  ASSERT_TRUE((ConvertHcmToContract<Detection3DHcmContract, Detection3DHcmDescriptor>(
      heap.descriptor, back)));
  EXPECT_EQ(back.SerializeAsString(), source.SerializeAsString());
}
