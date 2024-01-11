/**
 * Copyright (c) 2009, 2020 Robert Bosch GmbH and its subsidiaries.
 * This program and the accompanying materials are made available under
 * the terms of the Bosch Internal Open Source License v4
 * which accompanies this distribution, and is available at
 * http://bios.intranet.bosch.com/bioslv4.txt
 */

#include "radar_manager_ros/internal/communication/odt.hpp"

#include "rclcpp/rclcpp.hpp"

CODTEntry::CODTEntry(size_t ofs, size_t sz_bytes) : ofs_(ofs), size_(sz_bytes)
{
}

size_t CODTEntry::getOfs() const
{
  return ofs_;
}

size_t CODTEntry::getSize() const
{
  return size_;
}

CODT::CODT(uint32_t target_address, size_t sz_bytes, size_t max_odt_entry_size)
  : target_base_addr_(target_address), host_base_ptr_(NULL), size_(sz_bytes)
{
  size_t ofs = 0;  // start offset of odt entry inside of/relative to odt
  do {
    size_t bytes = sz_bytes - ofs;  // remaining size to allocate
    if (bytes > max_odt_entry_size) {
      bytes = max_odt_entry_size;
    }
    entries_.push_back(CODTEntry(ofs, bytes));
    ofs += bytes;

  } while (ofs < sz_bytes);
}

void CODT::reset(uint8_t* base_addr)
{
  host_base_ptr_ = base_addr;
}

void CODT::appendData(const uint8_t* src_ptr)
{
  uint32_t offset;
  for (size_t write_idx = 0; write_idx < entries_.size(); write_idx++) {
    offset = entries_[write_idx].getOfs();
    memcpy((host_base_ptr_ + offset), (src_ptr + offset), entries_[write_idx].getSize());
  }
}

uint8_t* CODT::getHostMemoryBasePointer() const
{
  return host_base_ptr_;
}

size_t CODT::getSize() const
{
  return size_;
}

uint32_t CODT::getTargetBaseAddress() const
{
  return target_base_addr_;
}

size_t CODT::getNumEntries() const
{
  return entries_.size();
}

size_t CODT::getSizeOfEntry(size_t entry) const
{
  return entries_[entry].getSize();
}

size_t CODT::getOffsetOfEntry(size_t entry) const
{
  return entries_[entry].getOfs();
}
