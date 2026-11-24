// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_adapter/hcm_ipc_adapter/flow_client/hcm_dma_buffer.hpp"

#include <gtest/gtest.h>
#include <unistd.h>

#include <climits>
#include <cstdint>

#include "QC/Infras/Memory/BufferDescriptor.hpp"

using namespace qrb::adapter;

// UT-HCM-04: the inherited buffer is classified before the CPU touches it. Only a DMA-BUF of
// this process is synced; heap memory and an fd of -1 are not.
TEST(HcmDmaBuffer, ClassifiesTheInheritedBufferAndSyncsOnlyDmaBufs)
{
  char storage[64] = {};
  int fd = 0;

  QC::Memory::QCBufferDescriptorBase_t base;
  EXPECT_EQ(InspectHcmBuffer(base, fd), HcmBufferAccess::kNone);
  EXPECT_EQ(fd, -1);

  QC::Memory::BufferDescriptor_t buffer;
  buffer.pBuf = nullptr;
  EXPECT_EQ(InspectHcmBuffer(buffer, fd), HcmBufferAccess::kNone);

  buffer.pBuf = storage;
  buffer.size = sizeof(storage);
  buffer.offset = 0;
  buffer.allocatorType = QC::Memory::QC_MEMORY_ALLOCATOR_HEAP;
  fd = 0;
  EXPECT_EQ(InspectHcmBuffer(buffer, fd), HcmBufferAccess::kHostMemory);
  EXPECT_EQ(fd, -1);

  buffer.offset = sizeof(storage) + 1;
  EXPECT_EQ(InspectHcmBuffer(buffer, fd), HcmBufferAccess::kUnusable);
  buffer.offset = 0;

  buffer.allocatorType = QC::Memory::QC_MEMORY_ALLOCATOR_DMA;
  buffer.dmaHandle = 5;
  buffer.pid = 0;
  EXPECT_EQ(InspectHcmBuffer(buffer, fd), HcmBufferAccess::kLocalDmaBuf);
  EXPECT_EQ(fd, 5);
  buffer.pid = ::getpid();
  EXPECT_EQ(InspectHcmBuffer(buffer, fd), HcmBufferAccess::kLocalDmaBuf);

  buffer.pid = ::getpid() + 1;
  EXPECT_EQ(InspectHcmBuffer(buffer, fd), HcmBufferAccess::kUnusable);
  EXPECT_EQ(fd, -1);
  buffer.pid = 0;

  // Any DMA allocator, not only the generic one.
  buffer.allocatorType = QC::Memory::QC_MEMORY_ALLOCATOR_DMA_GPU;
  EXPECT_EQ(InspectHcmBuffer(buffer, fd), HcmBufferAccess::kLocalDmaBuf);
  EXPECT_EQ(fd, 5);
  buffer.allocatorType = QC::Memory::QC_MEMORY_ALLOCATOR_DMA;

  buffer.dmaHandle = static_cast<std::uint64_t>(INT_MAX) + 1;
  EXPECT_EQ(InspectHcmBuffer(buffer, fd), HcmBufferAccess::kUnusable);
  buffer.dmaHandle = 5;

  buffer.allocatorType = QC::Memory::QC_MEMORY_ALLOCATOR_LAST;
  EXPECT_EQ(InspectHcmBuffer(buffer, fd), HcmBufferAccess::kUnusable);

  EXPECT_EQ(BeginDmaBufCpuRead(-1), Status::kOk);
  EXPECT_EQ(EndDmaBufCpuRead(-1), Status::kOk);
  EXPECT_EQ(BeginDmaBufCpuWrite(-1), Status::kOk);
  EXPECT_EQ(EndDmaBufCpuWrite(-1), Status::kOk);
}
