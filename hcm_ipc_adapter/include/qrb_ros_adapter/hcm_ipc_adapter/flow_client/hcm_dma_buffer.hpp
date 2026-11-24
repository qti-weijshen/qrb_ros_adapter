// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_ADAPTER_HCM_IPC_ADAPTER_FLOW_CLIENT_HCM_DMA_BUFFER_HPP
#define QRB_ROS_ADAPTER_HCM_IPC_ADAPTER_FLOW_CLIENT_HCM_DMA_BUFFER_HPP

#include "QC/Infras/Memory/Ifs/QCBufferDescriptorBase.hpp"
#include "qrb_ros_adapter/ipc_runtime/status.hpp"

namespace qrb::adapter
{

/** @brief How the CPU can reach the buffer an HCM descriptor inherits. */
enum class HcmBufferAccess
{
  kNone,         ///< No inherited buffer, or no storage attached.
  kHostMemory,   ///< Heap memory the CPU reaches directly.
  kLocalDmaBuf,  ///< A DMA-BUF of this process, mapped at pBuf.
  kUnusable,     ///< Invalid offset, foreign process or unknown allocator.
};

/**
 * @brief Classify the inherited buffer of @p descriptor.
 *
 * @param fd The DMA-BUF fd for kLocalDmaBuf, otherwise -1.
 */
HcmBufferAccess InspectHcmBuffer(const QC::Memory::QCBufferDescriptorBase_t & descriptor, int & fd);

/**
 * @brief Bracket CPU access to a DMA-BUF.
 *
 * A negative fd is a no-op, so callers can pass the fd from InspectHcmBuffer() for any buffer.
 * A buffer without sync support (ENOTTY) also succeeds.
 */
Status BeginDmaBufCpuRead(int fd);
Status EndDmaBufCpuRead(int fd);
Status BeginDmaBufCpuWrite(int fd);
Status EndDmaBufCpuWrite(int fd);

}  // namespace qrb::adapter

#endif  // QRB_ROS_ADAPTER_HCM_IPC_ADAPTER_FLOW_CLIENT_HCM_DMA_BUFFER_HPP
