// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

#include "qrb_ros_adapter/hcm_ipc_adapter/flow_client/hcm_contract_registry.hpp"
#include "qrb_ros_adapter/hcm_ipc_adapter/flow_client/hcm_converters.hpp"
#include "qrb_ros_adapter/ipc_runtime/contract_descriptor_registry.hpp"

using namespace qrb::adapter;

// Every shared Contract has exactly one complete HCM binding, and the
// descriptor factory creates, by Contract name, descriptors only that binding
// accepts.
TEST(HcmContractBinding, EveryContractHasOneCompleteBindingAndAMatchingFactory)
{
  ASSERT_EQ(HcmContractBindingCount(), ContractDescriptorCount());
  for (std::size_t i = 0; i < HcmContractBindingCount(); ++i) {
    const HcmContractBinding * binding = HcmContractBindingAt(i);
    ASSERT_NE(binding, nullptr);
    ASSERT_NE(binding->descriptor, nullptr);
    const std::string name(binding->descriptor->typeName);
    EXPECT_EQ(FindHcmContractBinding(name), binding) << name;
    EXPECT_NE(binding->acceptsDescriptor, nullptr) << name;
    EXPECT_NE(binding->contractToHcm, nullptr) << name;
    EXPECT_NE(binding->hcmToContract, nullptr) << name;
    EXPECT_NE(binding->createDescriptor, nullptr) << name;
  }
  EXPECT_EQ(HcmContractBindingAt(HcmContractBindingCount()), nullptr);
  EXPECT_EQ(FindHcmContractBinding("qrb.contract.unknown.v1.Missing"), nullptr);

  // Descriptors are told apart by C++ type, so a created descriptor is
  // accepted by its own binding and by no other, whatever its buffer type.
  for (std::size_t i = 0; i < HcmContractBindingCount(); ++i) {
    const HcmContractBinding & binding = *HcmContractBindingAt(i);
    const std::string name(binding.descriptor->typeName);
    std::unique_ptr<QC::Memory::QCBufferDescriptorBase_t> created;
    ASSERT_EQ(CreateQrbBufferDescriptor(name, created), QC::QC_STATUS_OK) << name;
    ASSERT_NE(created, nullptr) << name;
    for (std::size_t j = 0; j < HcmContractBindingCount(); ++j) {
      EXPECT_EQ(HcmContractBindingAt(j)->acceptsDescriptor(*created), i == j) << name;
    }
  }

  std::unique_ptr<QC::Memory::QCBufferDescriptorBase_t> descriptor =
      std::make_unique<QC::Memory::QCBufferDescriptorBase_t>();
  EXPECT_EQ(CreateQrbBufferDescriptor("qrb.contract.unknown.v1.Missing", descriptor),
      QC::QC_STATUS_UNSUPPORTED);
  EXPECT_EQ(descriptor, nullptr);
}

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

// Contracts share the value types of qrb_contracts, several descriptors share a buffer type
// (CUSTOM_4) and the detection descriptors share a base class, but a binding takes only the
// descriptor and the Contract message of its own Contract, whichever way it converts.
TEST(HcmContractBinding, EveryBindingTakesOnlyItsOwnDescriptorAndContract)
{
  const std::vector<BindingEntry> entries = MakeEveryBindingEntry();
  ASSERT_EQ(entries.size(), HcmContractBindingCount());

  for (std::size_t i = 0; i < entries.size(); ++i) {
    const std::string & name = entries[i].contractType;
    const HcmContractBinding * binding = FindHcmContractBinding(name);
    ASSERT_NE(binding, nullptr) << name;
    for (std::size_t j = 0; j < entries.size(); ++j) {
      const std::string & other = entries[j].contractType;
      EXPECT_EQ(binding->acceptsDescriptor(*entries[j].descriptor), i == j) << name << " " << other;
      if (i == j)
        continue;
      // The descriptor of another Contract, in either direction.
      EXPECT_FALSE(
          binding->contractToHcm(*entries[i].contract, DmaBufView{}, *entries[j].descriptor))
          << name << " " << other;
      EXPECT_FALSE(binding->hcmToContract(*entries[j].descriptor, *entries[i].contract))
          << name << " " << other;
      // The message of another Contract, in either direction.
      EXPECT_FALSE(
          binding->contractToHcm(*entries[j].contract, DmaBufView{}, *entries[i].descriptor))
          << name << " " << other;
      EXPECT_FALSE(binding->hcmToContract(*entries[i].descriptor, *entries[j].contract))
          << name << " " << other;
    }
  }
}
