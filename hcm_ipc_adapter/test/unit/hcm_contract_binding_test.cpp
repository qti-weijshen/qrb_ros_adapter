// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include <gtest/gtest.h>
#include <unistd.h>

#include <climits>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "QC/Infras/Memory/BufferDescriptor.hpp"
#include "qrb_ros_adapter/hcm_ipc_adapter/flow_client/hcm_contract_registry.hpp"
#include "qrb_ros_adapter/hcm_ipc_adapter/flow_client/hcm_converters.hpp"
#include "qrb_ros_adapter/hcm_ipc_adapter/flow_client/hcm_dma_buffer.hpp"
#include "qrb_ros_adapter/ipc_runtime/contract_descriptor_registry.hpp"

using namespace qrb::adapter;

namespace
{

/** @brief One Contract, with a Contract message and a descriptor of its own types. */
struct BindingEntry
{
  std::string contractType;
  std::unique_ptr<google::protobuf::MessageLite> contract;
  std::unique_ptr<QC::Memory::QCBufferDescriptorBase_t> descriptor;
};

template <typename ContractT, typename DescriptorT>
BindingEntry MakeBindingEntry(const char * contractType)
{
  return BindingEntry{contractType, std::make_unique<ContractT>(), std::make_unique<DescriptorT>()};
}

std::vector<BindingEntry> MakeEveryBindingEntry()
{
  std::vector<BindingEntry> entries;
#define QRB_CONTRACT_MAPPING(Id, TypeName, Payload, ContractT, DescriptorT, ...)                   \
  entries.push_back(MakeBindingEntry<ContractT, DescriptorT>(TypeName));
#include "qrb_ros_adapter/ipc_runtime/contract_mappings.def"
#undef QRB_CONTRACT_MAPPING
  return entries;
}

}  // namespace

// Every Contract has one complete binding that takes only the descriptor and the Contract message
// of its own Contract, whichever way it converts; the factory creates by name a descriptor its
// binding accepts and refuses an unknown name.
TEST(HcmContractBinding, EveryContractHasOneBindingThatTakesOnlyItsOwnTypes)
{
  const std::vector<BindingEntry> entries = MakeEveryBindingEntry();
  ASSERT_EQ(HcmContractBindingCount(), ContractDescriptorCount());
  ASSERT_EQ(entries.size(), HcmContractBindingCount());
  EXPECT_EQ(HcmContractBindingAt(HcmContractBindingCount()), nullptr);
  EXPECT_EQ(FindHcmContractBinding("qrb.contract.unknown.v1.Missing"), nullptr);

  for (std::size_t i = 0; i < entries.size(); ++i) {
    const std::string & name = entries[i].contractType;
    SCOPED_TRACE(name);
    const HcmContractBinding * binding = FindHcmContractBinding(name);
    ASSERT_NE(binding, nullptr);
    EXPECT_EQ(binding, HcmContractBindingAt(i));
    ASSERT_NE(binding->descriptor, nullptr);
    ASSERT_NE(binding->acceptsDescriptor, nullptr);
    ASSERT_NE(binding->contractToHcm, nullptr);
    ASSERT_NE(binding->hcmToContract, nullptr);
    ASSERT_NE(binding->createDescriptor, nullptr);

    std::unique_ptr<QC::Memory::QCBufferDescriptorBase_t> created;
    ASSERT_EQ(CreateQrbBufferDescriptor(name, created), QC::QC_STATUS_OK);
    ASSERT_NE(created, nullptr);
    EXPECT_TRUE(binding->acceptsDescriptor(*created));

    for (std::size_t j = 0; j < entries.size(); ++j) {
      SCOPED_TRACE(entries[j].contractType);
      EXPECT_EQ(binding->acceptsDescriptor(*entries[j].descriptor), i == j);
      if (i == j)
        continue;
      EXPECT_FALSE(
          binding->contractToHcm(*entries[i].contract, DmaBufView{}, *entries[j].descriptor));
      EXPECT_FALSE(binding->hcmToContract(*entries[j].descriptor, *entries[i].contract));
      EXPECT_FALSE(
          binding->contractToHcm(*entries[j].contract, DmaBufView{}, *entries[i].descriptor));
      EXPECT_FALSE(binding->hcmToContract(*entries[i].descriptor, *entries[j].contract));
    }
  }

  std::unique_ptr<QC::Memory::QCBufferDescriptorBase_t> descriptor =
      std::make_unique<QC::Memory::QCBufferDescriptorBase_t>();
  EXPECT_EQ(CreateQrbBufferDescriptor("qrb.contract.unknown.v1.Missing", descriptor),
      QC::QC_STATUS_UNSUPPORTED);
  EXPECT_EQ(descriptor, nullptr);
}

// The inherited buffer is classified before the CPU touches it; sync calls with an fd of -1
// succeed without work.
TEST(HcmDmaBuffer, ClassifiesTheInheritedBufferAndNoOpsSyncForFdMinusOne)
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
