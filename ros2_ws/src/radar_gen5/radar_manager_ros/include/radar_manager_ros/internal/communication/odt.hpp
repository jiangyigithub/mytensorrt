/**
 * Copyright (c) 2009, 2020 Robert Bosch GmbH and its subsidiaries.
 * This program and the accompanying materials are made available under
 * the terms of the Bosch Internal Open Source License v4
 * which accompanies this distribution, and is available at
 * http://bios.intranet.bosch.com/bioslv4.txt
 */

#ifndef ODT_HPP_
#define ODT_HPP_

#include <cstring>
#include <map>
#include <string>
#include <vector>

#include <assert.h>

class CODTEntry
{
public:
  CODTEntry(size_t ofs, size_t sz_bytes);

  //	//! \return true if last entry of this ODT was copied
  //	bool copyToHost(const uint8_t* src_ptr);

  size_t getOfs() const;
  size_t getSize() const;

private:
  size_t ofs_;
  size_t size_;
};

class CODT
{
public:
  CODT(uint32_t target_address, size_t sz_bytes, size_t max_odt_entry_size);

  void reset(uint8_t* base_addr);
  void appendData(const uint8_t* src_ptr);

  uint8_t* getHostMemoryBasePointer() const;
  size_t getSize() const;
  uint32_t getTargetBaseAddress() const;

  size_t getNumEntries() const;

  size_t getSizeOfEntry(size_t entry) const;
  size_t getOffsetOfEntry(size_t entry) const;

private:
  uint32_t target_base_addr_;
  uint8_t* host_base_ptr_;
  size_t size_;
  std::vector<CODTEntry> entries_;
};

#endif
