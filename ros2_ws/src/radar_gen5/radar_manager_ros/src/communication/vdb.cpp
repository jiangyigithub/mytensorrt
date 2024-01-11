/**
 * Copyright (c) 2009, 2020 Robert Bosch GmbH and its subsidiaries.
 * This program and the accompanying materials are made available under
 * the terms of the Bosch Internal Open Source License v4
 * which accompanies this distribution, and is available at
 * http://bios.intranet.bosch.com/bioslv4.txt
 */

#include "rclcpp/rclcpp.hpp"

#include "radar_manager_ros/internal/communication/vdb.hpp"
#include "radar_manager_ros/radar_manager.hpp"


CVDBPacket::CVDBPacket(
    const std::string& struct_name,
    uint32_t target_addr,
    size_t size,
    uint16_t odt_ctr,
    size_t max_dto_size,
    size_t max_odt_entry_size,
    uint16_t daq_list_id)
  : name_(struct_name), size_(size), valid_(false), odt_ctr_(odt_ctr), daq_list_id_(daq_list_id)
{
  size_t max_odt_size;
  uint32_t offset = 0;
  size_t size_left = size_;
  size_t header_size = 8;  // 8 = 1 PID + 3 Unused + 4 Timestamp

  // for RadarGen5Plus (=LRR5) the following values worked:
  // max_dto_size = 246;
  // max_odt_entry_size = 246;

  max_odt_size = max_dto_size - header_size;  // first ODT with timestamp
  vdb_packet_map_.clear();

  do {
    RCLCPP_DEBUG_STREAM(radar_manager::CRadarManager::getInstance()->get_logger(), 
         "MAKING PACKET: remaining data: " << size_left << ", max_odt_size: " << max_odt_size);

    // VDB-Packet with single ODT or last ODT of VDB-Packet with multiple ODTs
    if (size_left <= max_odt_size) {
      RCLCPP_DEBUG_STREAM(radar_manager::CRadarManager::getInstance()->get_logger(), 
        "current odt size: " << size_left << " max_odt_size: " << max_odt_size);

      vdb_packet_map_.insert(std::pair<uint8_t, vdb_packet_item>(
          odt_ctr_, vdb_packet_item(target_addr, size_left, max_odt_entry_size, false)));
      size_left = 0;
    } else {
      if (vdb_packet_map_.empty())  // first VDB-Packet with multiple ODTs
      {
        header_size = 8;  // first ODT with timestamp
      } else              // further VDB-Packet with multiple ODTs
      {
        header_size = 4;  // following ODTs w/o timestamp
      }
      max_odt_size = max_dto_size - header_size;
      uint32_t current_odt_size = max_odt_size;
      // offset = current_odt_size;

      // old offset calculation sometimes gave zero for 1st package !
      // example: offset = int(242/246)*246 = 0
      // offset = (int)(max_odt_size / max_odt_entry_size) * max_odt_entry_size;

      RCLCPP_DEBUG_STREAM(radar_manager::CRadarManager::getInstance()->get_logger(), 
          "current odt size: " << current_odt_size << " max_odt_size: " << max_odt_size);

      vdb_packet_map_.insert(std::pair<uint8_t, vdb_packet_item>(
          odt_ctr_, vdb_packet_item(target_addr, current_odt_size, max_odt_entry_size, false)));
      target_addr += current_odt_size;
      size_left -= current_odt_size;
    }
    odt_ctr_++;
  } while (size_left > 0);
}

void CVDBPacket::reset(uint8_t* base_addr)
{
  uint32_t offset = 0;
  for (it_ = vdb_packet_map_.begin(); it_ != vdb_packet_map_.end(); it_++) {
    it_->second.odt_.reset(base_addr + offset);
    offset += it_->second.odt_.getSize();
  }
  valid_ = false;
}

bool CVDBPacket::checkVdbPacket()
{
  for (it_ = vdb_packet_map_.begin(); it_ != vdb_packet_map_.end(); it_++) {
    if (it_->second.received_ != true)  // Not all VDB Packets received?
    {
      valid_ = false;
      return valid_;
    }
  }
  valid_ = true;
  return valid_;
}

void CVDBPacket::reset_received()  // reset received fields in vdb_packet_item
{
  for (it_ = vdb_packet_map_.begin(); it_ != vdb_packet_map_.end(); it_++) {
    it_->second.received_ = false;
  }
  valid_ = false;
}

bool CVDBPacket::isPidInVdB(uint16_t pid) const
{
  return vdb_packet_map_.find(pid) != vdb_packet_map_.end();
}

void CVDBPacket::appendData(uint16_t pid, uint8_t* src_ptr)
{
  if (src_ptr == NULL)
    RCLCPP_ERROR_STREAM(radar_manager::CRadarManager::getInstance()->get_logger(), "appendData: src_ptr == NULL");

  it_ = vdb_packet_map_.find(pid);

  if (it_ != vdb_packet_map_.end())  // Belongs PID to this VDB Packet?
  {
    it_->second.odt_.appendData(src_ptr);
    it_->second.received_ = true;
  } else {
    valid_ = false;
  }
}

CODT* CVDBPacket::getOdtHostPtr(uint16_t pid)
{
  it_ = vdb_packet_map_.find(pid);

  if (it_ != vdb_packet_map_.end())  // Belongs PID to this VDB Packet?
  {
    return &it_->second.odt_;
  } else {
    return NULL;
  }
}

uint32_t CVDBPacket::getOdtTargetAdr() const
{
  const auto it = vdb_packet_map_.cbegin();

  if (it != vdb_packet_map_.cend()) {
    return it->second.odt_.getTargetBaseAddress();
  } else {
    return 0;
  }
}

std::vector<uint8_t> CVDBPacket::getData() const
{
  std::vector<uint8_t> tmp_data;

  for (const auto& vdb_packet : vdb_packet_map_) {
    uint8_t* base = vdb_packet.second.odt_.getHostMemoryBasePointer();
    size_t sz = vdb_packet.second.odt_.getSize();
    std::copy(base, base + sz, std::back_inserter(tmp_data));
  }

  return tmp_data;
}

std::string CVDBPacket::getName() const
{
  return name_;
}

size_t CVDBPacket::getPacketSize() const
{
  return size_;
}

size_t CVDBPacket::getNumOdts()
{
  return vdb_packet_map_.size();
}

size_t CVDBPacket::getTotalNumOdts()
{
  return odt_ctr_;
}

size_t CVDBPacket::getFirstOdtPid()
{
  if (size_ > 0) {
    return vdb_packet_map_.begin()->first;
  } else {
    return -1;
  }
}
bool CVDBPacket::isValid() const
{
  return valid_;
}
uint16_t CVDBPacket::getDaqListId() const
{
  return daq_list_id_;
}
