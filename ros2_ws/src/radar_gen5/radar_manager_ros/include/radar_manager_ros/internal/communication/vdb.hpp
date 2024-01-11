/**
 * Copyright (c) 2009, 2020 Robert Bosch GmbH and its subsidiaries.
 * This program and the accompanying materials are made available under
 * the terms of the Bosch Internal Open Source License v4
 * which accompanies this distribution, and is available at
 * http://bios.intranet.bosch.com/bioslv4.txt
 */

#ifndef VDB_HPP_
#define VDB_HPP_

#include <cstring>
#include <map>
#include <string>
#include <vector>

#include <assert.h>

#include "odt.hpp"

struct PreviousMsg
{
  std::string name;
  std::vector<uint8_t> data;
  bool duplication_happened;
};

class CVDBPacket
{
public:
  struct vdb_packet_item
  {
    vdb_packet_item(uint32_t target_addr, size_t size, size_t max_odt_entry_size, bool received)
      : odt_(target_addr, size, max_odt_entry_size), received_(received)
    {
    }
    CODT odt_;
    bool received_;
  };

  CVDBPacket(
      const std::string& struct_name,
      uint32_t target_addr,
      size_t size,
      uint16_t odt_ctr,
      size_t max_dto_size,
      size_t max_odt_entry_size,
      uint16_t daq_list_id);

  bool checkVdbPacket();
  void reset_received();  // reset received fields in vdb_packet_item
  void reset(uint8_t* base_addr);
  void appendData(uint16_t pid, uint8_t* src_ptr);

  std::string getName() const;
  size_t getPacketSize() const;
  size_t getNumOdts();
  size_t getTotalNumOdts();
  size_t getFirstOdtPid();
  CODT* getOdtHostPtr(uint16_t pid);
  uint32_t getOdtTargetAdr() const;
  uint16_t getDaqListId() const;
  std::vector<uint8_t> getData() const;

  bool isValid() const;
  bool isPidInVdB(uint16_t pid) const;

private:
  std::map<uint8_t, vdb_packet_item> vdb_packet_map_;
  std::map<uint8_t, vdb_packet_item>::iterator it_;

  uint16_t daq_list_id_;
  std::string name_;
  size_t size_;
  bool valid_;
  uint16_t odt_ctr_;
};

#endif
