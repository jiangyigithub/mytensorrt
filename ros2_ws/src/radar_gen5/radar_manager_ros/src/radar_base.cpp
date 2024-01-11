/**
 * Copyright (c) 2009, 2020 Robert Bosch GmbH and its subsidiaries.
 * This program and the accompanying materials are made available under
 * the terms of the Bosch Internal Open Source License v4
 * which accompanies this distribution, and is available at
 * http://bios.intranet.bosch.com/bioslv4.txt
 */

#include <fstream>
#include <sstream>

#include <algorithm>
#include <errno.h>
#include <stdint.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

#include <radar_manager_ros/radar_base.hpp>

using namespace radar_manager;

CRadarBase::CRadarBase(CRadarBase::radar_basic_info basic_info, std::shared_ptr<rclcpp::Node> node)
    : basic_info_(basic_info),
      node_(node),
      tcp_timeout_(1),
      wait_time_(1),
      SocketFD(-1),
      dto_message_received_(false),
      run_thread_(),
      is_running_(),
      swVersionFromConfig_(SW_VERSION_DEFAULT),
      buildInfoFromConfig_(BUILD_INFO_DEFAULT),
      diag_updater_(node)
{
  // In ROS2 parameters always have to be declared first (this does not mean they are set)
  // Note: These parameters are global, i.e. same for every device. They should not be read here but rather
  // at the one and only instance of radar manager. This would make double sense, also since the
  // parameter storing class is named radar_manager_parameters, and not radar_base_parameters
  node_->declare_parameter("publisher_topic_robs");
  node_->declare_parameter("msg_queue_size");
  node_->declare_parameter("desired_freq_min");
  node_->declare_parameter("desired_freq_max");
  node_->declare_parameter("freq_tol");

  is_running_.store(false);
  Init();

  diag_updater_.setHardwareID("radar_manager");
  diag_updater_.add("VDB Packets updater", this, &CRadarBase::diagnoseVDBPackets);
  diag_radar_rob_ = std::make_unique<diagnostic_updater::TopicDiagnostic>(
      params_.publisher_topic_robs + "/" + basic_info_.device_position,
      diag_updater_,
      diagnostic_updater::FrequencyStatusParam(
          &params_.desired_freq_min, &params_.desired_freq_max, params_.freq_tol),
      diagnostic_updater::TimeStampStatusParam(-1, 1));

  diag_updater_timer_ = node_->create_wall_timer(std::chrono::milliseconds(1000), [this] { diag_updater_.force_update(); });
}

CRadarBase::~CRadarBase()
{
  Stop();

  if (SocketFD != -1)
  {
    // close xcp connection
    RCLCPP_DEBUG_STREAM(node_->get_logger(), basic_info_.device_position << ": Communication: Sending frame CC_DISCONNECT");
    sendCtoMessage(CC_DISCONNECT, NULL, 0, SocketFD);

    // close tcp socket
    closeSocket(SocketFD);
    SocketFD = -1;
  }
}

void CRadarBase::Init()
{
  RCLCPP_DEBUG_STREAM(node_->get_logger(), "Radar Base Initialization...");
  std::cout << "[CRadarBase::Init()]" << std::endl;

  // load node parameters
  params_.getParamsFromNodeHandle(node_);

  radar_data_rob_pub_ =
      node_->create_publisher<radar_msgs::msg::RadarROB2>(params_.publisher_topic_robs, params_.msg_queue_size);
}

void CRadarBase::Start()
{
  // loop while data received
  is_running_.store(true);
  run_thread_ = std::make_shared<std::thread>(std::bind(&CRadarBase::run, this));
}

void CRadarBase::Stop()
{
  is_running_.store(false);
  run_thread_->join();
  // TODO: ensure that receive function returns after no data could be received. no blocking
  // wait for run function to terminate. if not cancel thread
  //  int wait_counter=0;
  //  while (pthread_kill(run_thread_, 0) == 0 && wait_counter < 10)
  //  {
  //      usleep(10 * 1000);
  //      wait_counter++;
  //  }
  //  if (pthread_kill(run_thread_, 0))
  //  {
  //      //thread still running
  //      pthread_cancel(run_thread_);
  //  }
}

bool CRadarBase::getFactoryData(radar_msgs::msg::RadarFactoryData &temp_radarFactoryData)
{
  bool returnValue = false;
  // only perform the following if lock can be obtained
  if (staticDataSt_.staticPacketsMutex.try_lock() == true)
  {
    // only update the following information, if initialisation occured
    if (staticDataSt_.staticDataInitialised == true)
    {
      temp_radarFactoryData.sw_version_from_ecu = staticDataSt_.swVersionFromECU;
      temp_radarFactoryData.build_info_from_ecu = staticDataSt_.buildInfoFromECU;
      temp_radarFactoryData.sw_version_from_config = swVersionFromConfig_;
      temp_radarFactoryData.build_info_from_config = buildInfoFromConfig_;
      temp_radarFactoryData.device_type = basic_info_.device_type;         // what sensor type
      temp_radarFactoryData.device_position = basic_info_.device_position; // where mounted
      if (staticDataSt_.factoryDataRead == true)                           // only if factory data was read
      {
        // make factory data long enough for data
        temp_radarFactoryData.factory_data.resize(FACTORY_DATA_NUM_BYTES);
        for (uint32_t i = 0; i < FACTORY_DATA_NUM_BYTES; i++)
        {
          temp_radarFactoryData.factory_data[i] = staticDataSt_.factoryData[i];
        }
      }
    }
    // true or false on whether new data was received
    returnValue = staticDataSt_.staticDataInitialised;
  }
  staticDataSt_.staticPacketsMutex.unlock();
  return returnValue;
}

void CRadarBase::run()
{
  // Connect sensor device
  CRadarBase::connect();
  if (stage == DATA_AQUISITION_STATE)
  {
    RCLCPP_DEBUG_STREAM(node_->get_logger(), basic_info_.device_position << ": Data Acquisition: Start measurement...");
  }

  while (isRunning() && rclcpp::ok())
  {
    if (stage == DATA_AQUISITION_STATE)
    {
      receiveAndPublish();
    }
    else if (stage == FATAL_STATE)
    {
      break;
    }
    else
    {
      // Try to Reconnect
      CRadarBase::connect();
    }
  }
}

void CRadarBase::receiveAndPublish()
{
  memset(&dto_message, 0, sizeof(xcp_message_dto)); // todo: zeroing before overdub necessary?

  while (isRunning() && rclcpp::ok())
  {
    if (stage == DATA_AQUISITION_STATE)
    {
      receiveXcpRadarData();
      if (dto_message_received_)
      {
        // find matching vdb_packet_ entry for the received packet
        vdb_packet_number =
            CRadarBase::daq_list2vdb_index(dto_message.dto_list_id, dto_message.pid);

        // insert message in the vdb_packet_ entry
        vdb_packets_[vdb_packet_number].appendData(
            (uint16_t)dto_message.pid, dto_message.data); // &dto_message.data[0]);

        if (allVdbPacketsFromDaqListReceived(dto_message.dto_list_id))
        {
          checkDuplicateRobPackets(dto_message.dto_list_id);
          publishRobPackets(dto_message.dto_list_id);
          resetPacketsFromDaqList(dto_message.dto_list_id);
          resetDuplicationIndicator();
        }
        dto_message_received_ = false;
      }
    }
    else if (stage == FATAL_STATE)
    {
      RCLCPP_INFO_STREAM(node_->get_logger(), "receiveAndPublish >> stage == FATAL_STATE");
      return;
    }
    else if (stage == ERROR_STATE)
    {
      // Reconnect
      RCLCPP_INFO_STREAM(node_->get_logger(), "receiveAndPublish >> ERROR_STATE >> connect()");
      connect();
    }
    else // any other stage during sensor configuration
    {
      RCLCPP_INFO_STREAM(node_->get_logger(), "receiveAndPublish >> fatal_error >> FATAL_STATE");
      std::stringstream fatal_error;
      fatal_error << basic_info_.device_position << ": Initialization: "
                  << "Sensor configuration error: insufficient parameter file or software revision";
      RCLCPP_FATAL_STREAM(node_->get_logger(), fatal_error.str());
      stage = FATAL_STATE;
    }
  }
}

/// addVdbPackets creates the list of vdb packets to store the name-address-size config
// In order to handle multiple DAQ lists, entries are added (not replaced)
int CRadarBase::addVdbPackets(int current_daq_list_index)
{
  std::string path = ament_index_cpp::get_package_share_directory("radar_manager_ros") + "/params/";

  std::ifstream file;
  std::string lineString;

  std::string name;
  uint32_t addr;
  size_t size;
  bool diag_active;
  size_t req_num_nonzero;
  bool filter_duplication = false;
  uint16_t next_odt;
  int number_of_added_vdbs = 0;
  int number_of_initial_odts = 0;

  //  if (vdb_packets_.size() == 0){
  next_odt = 0; // first entry in odt list
                //  }
                //  else{
                //    next_odt = vdb_packets_.back().getTotalNumOdts();
                //  }
  number_of_initial_odts = next_odt;

  RCLCPP_DEBUG_STREAM(node_->get_logger(),
                      basic_info_.device_position << ", list entry " << current_daq_list_index
                                                  << ": Initialization: Read Parameter from: "
                                                  << path +
                                                         basic_info_.daq_list_[current_daq_list_index].params_file);

  if (sensor_settings_.getMaxOdtEntrySizeDaq() <= 0 || sensor_settings_.getMaxDtoSize() <= 0)
  {
    RCLCPP_ERROR_STREAM(node_->get_logger(),
                        basic_info_.device_position << ": Max DTO (" << sensor_settings_.getMaxDtoSize()
                                                    << ") or max ODT (" << sensor_settings_.getMaxOdtEntrySizeDaq()
                                                    << ") sizes not initialized.");
    stage = FATAL_STATE;
    return number_of_added_vdbs;
  }

  file.open(path + basic_info_.daq_list_[current_daq_list_index].params_file);

  if (file.is_open())
  {
    while (std::getline(file, lineString, '\n'))
    {
      std::stringstream ss;
      ss << lineString;

      if (lineString[0] != '#') // ignore comments
      {
        if (ss >> name >> std::hex >> addr >> size >> std::boolalpha >> diag_active >> std::dec >>
            req_num_nonzero >> std::boolalpha >> filter_duplication)
        {
          RCLCPP_DEBUG_STREAM(node_->get_logger(),
                              basic_info_.device_position << ": Parameters:  " << std::hex << "0x" << addr
                                                          << " + size 0x" << size << ": \t" << name);

          if (addr >= 0x40000000) // RAM only if address is greater than this. Otherwise it is
                                  // static memory
          {
            RCLCPP_DEBUG_STREAM(node_->get_logger(),
                                "max odt entry size: " << sensor_settings_.getMaxOdtEntrySizeDaq());

            if (filter_duplication == true)
            {
              PreviousMsg msg;
              msg.name = name;
              msg.duplication_happened = false;
              vdb_packets_duplicates_.push_back(msg);
            }
            vdb_packets_.push_back(CVDBPacket(
                name,
                addr,
                size,
                next_odt,
                sensor_settings_.getMaxDtoSize(),
                sensor_settings_.getMaxOdtEntrySizeDaq(),
                (uint16_t)current_daq_list_index));
            next_odt = vdb_packets_.back().getTotalNumOdts();

            diag_info data{0, req_num_nonzero};
            if (diag_active)
            {
              data_cache_.emplace(name, data);
            }

            RCLCPP_INFO_STREAM(node_->get_logger(),
                               "RADAR_MANAGER: " << basic_info_.device_position << ":"
                                                 << " Create data structure: VDB: "
                                                 << vdb_packets_.back().getName() << " with "
                                                 << vdb_packets_.back().getTotalNumOdts() << " ODTs created");
          }
          else
          {
            staticPackets_st staticPacket = {name, addr, size}; // set data into structure
            staticPackets_.push_back(staticPacket);
          }
        }
        else
        {
          // look for revision numbers for words.
          std::string keyword;
          keyword = "SW_Version_string=";
          if (lineString.find(keyword) !=
              std::string::npos) // compare against string::npos for failure to find
          {
            size_t pos;
            if ((pos = lineString.find("=")) != std::string::npos)
            {
              // remove keyword by removing everything before and equal to the "=" sign.
              // IMPORTANT: white spaces remain!!!
              lineString.erase(0, pos + 1); //+1 is to include equal sign
            }
            swVersionFromConfig_ = lineString;
          }

          keyword = "Build_Info_string=";
          if (lineString.find(keyword) != std::string::npos)
          {
            size_t pos;
            if ((pos = lineString.find("=")) != std::string::npos)
            {
              // remove keyword by removing everything before and equal to the "=" sign.
              // IMPORTANT: white spaces remain!!!
              lineString.erase(0, pos + 1); //+1 is to include equal sign
            }
            buildInfoFromConfig_ = lineString;
          }
        }
      }
    }
  }
  else
  {
    std::stringstream fatal_error;
    fatal_error << basic_info_.device_position << ":"
                << " Initialization: Param file not found";
    RCLCPP_DEBUG_STREAM(node_->get_logger(), fatal_error.str());
    stage = FATAL_STATE;
  }
  file.close();

  number_of_added_vdbs = vdb_packets_.back().getTotalNumOdts() - number_of_initial_odts;
  return number_of_added_vdbs;
}

int CRadarBase::daq_list2vdb_index(int daq_list_ind, int pid) const
{
  for (int vdb_ind = 0; vdb_ind < vdb_packets_.size(); vdb_ind++)
  {
    if ((vdb_packets_[vdb_ind].getDaqListId() == daq_list_ind) &&
        (vdb_packets_[vdb_ind].isPidInVdB((uint16_t)pid)))
    {
      return vdb_ind;
    }
  }
  return -1;
}

bool CRadarBase::allVdbPacketsFromDaqListReceived(uint16_t daq_list_index)
{
  for (auto &vdb_packet : vdb_packets_)
  {
    if (vdb_packet.getDaqListId() == daq_list_index && !vdb_packet.checkVdbPacket())
    {
      return false;
    }
  }

  return true;
}

void CRadarBase::resetDuplicationIndicator()
{
  for (auto &element : vdb_packets_duplicates_)
  {
    element.duplication_happened = false;
  }
}

void CRadarBase::checkDuplicateRobPackets(uint16_t daq_list_id)
{
  for (const auto &vdb_packet : vdb_packets_)
  {
    if (vdb_packet.getDaqListId() == daq_list_id)
    {
      for (auto &previous_msg : vdb_packets_duplicates_)
      {
        if (vdb_packet.getName() == previous_msg.name &&
            vdb_packet.getData() == previous_msg.data)
        {
          RCLCPP_WARN_STREAM(node_->get_logger(),
                             "The message: " << vdb_packet.getName()
                                             << " won't be published in the RadarROB2 msg because it was the same "
                                             << "as previously arrived.");
          previous_msg.duplication_happened = true;
        }
        // This will be never true if the if statement above executed
        if (vdb_packet.getName() == previous_msg.name &&
            vdb_packet.getData() != previous_msg.data)
        {
          previous_msg.data = vdb_packet.getData();
        }
      }
    }
  }
}

void CRadarBase::createRobPackets()
{
  radar_data_rob_ = std::make_shared<radar_msgs::msg::RadarROB2>();
  radar_data_rob_->robs.resize(vdb_packets_.size());

  auto rob_it = radar_data_rob_->robs.begin();
  for (int vdb_packet_ind = 0; vdb_packet_ind < vdb_packets_.size(); vdb_packet_ind++)
  {
    rob_it->data.resize(vdb_packets_[vdb_packet_ind].getPacketSize());

    // Let ODT remember where to put received data when DTOs shuffle in...
    // !! TODO: publishRobPackets creates its own RadarROB2 message
    //          the radar_data_rob_ is actually not used except of this line
    //          more investigation is needed
    vdb_packets_[vdb_packet_ind].reset(&rob_it->data[0]);
    rob_it++;
  }
}

void CRadarBase::publishRobPackets(uint16_t daq_list_id) const
{
  radar_msgs::msg::RadarROB2 radar_rob2_msg;

  radar_rob2_msg.header.stamp = time_stamp;
  radar_rob2_msg.header.frame_id = "/" + basic_info_.device_position;
  radar_rob2_msg.device_type = basic_info_.device_type;
  radar_rob2_msg.device_position = basic_info_.device_position;
  radar_rob2_msg.sw_revision = basic_info_.sw_revision;
  radar_rob2_msg.valid = false;

  for (const auto &vdb_packet : vdb_packets_)
  {
    if (vdb_packet.getDaqListId() == daq_list_id)
    {
      auto element_searcher = [&vdb_packet](const PreviousMsg &item) {
        return item.name == vdb_packet.getName();
      };
      const auto searched_element = std::find_if(
          vdb_packets_duplicates_.begin(), vdb_packets_duplicates_.end(), element_searcher);

      // If it's not in the vdb_packets_duplicates_ vector (don't want to filter that data) or if
      // it's in, it is not a duplication If the first statement is true the second won't be
      // evaluated so we don't dereference the end iterator
      if (searched_element == vdb_packets_duplicates_.end() ||
          searched_element->duplication_happened == false)
      {
        radar_msgs::msg::ROBData rob_data;
        rob_data.name = vdb_packet.getName();
        rob_data.base_address = vdb_packet.getOdtTargetAdr();
        rob_data.data = vdb_packet.getData();
        rob_data.valid = true;

        radar_rob2_msg.robs.push_back(rob_data);
      }
    }
  }

  /****************lnl, timer************/
  // radar_msgs::msg::Attribute tcp_t;
  // radar_msgs::msg::Attribute in_t;
  // radar_msgs::msg::Attribute out_t;
  // radar_msgs::msg::Attribute d_t;

  // const rclcpp::Time radar_manager_out_t = node_->now(); //lnl, timer
  // rclcpp::Duration radar_manager_d_t = radar_manager_out_t - radar_manager_in_t;

  // RCLCPP_INFO_STREAM(node_->get_logger(), "101_sent_timer:" << radar_manager_out_t.nanoseconds() << "::");
  // RCLCPP_INFO_STREAM(node_->get_logger(), "202_Tcp_timer:" << radar_tcp_sent << "::");
  // RCLCPP_INFO_STREAM(node_->get_logger(), "303_Duration_timer:" << radar_manager_d_t.seconds() << "::");

  // tcp_t.name = "radar_tcp_sent";
  // tcp_t.nanoseconds = radar_tcp_sent*1.0e+03f;  //radar_tcp_sent_us

  // in_t.name = "radar_manager_in_t";
  // in_t.nanoseconds = radar_manager_in_t.nanoseconds();
  // // in_t.seconds = radar_manager_in_t.seconds();
  // // in_t.nanoseconds = (radar_manager_in_t.seconds()-int(radar_manager_in_t.seconds()))*1.0e+09f;

  // out_t.name = "radar_manager_out_t";
  // out_t.nanoseconds = radar_manager_out_t.nanoseconds();

  // d_t.name = "radar_manager_d_t";
  // d_t.nanoseconds = radar_manager_d_t.nanoseconds();

  // radar_rob2_msg.attributes.push_back(tcp_t);
  // radar_rob2_msg.attributes.push_back(in_t);
  // radar_rob2_msg.attributes.push_back(out_t);
  // radar_rob2_msg.attributes.push_back(d_t);
  /************************************/

  radar_data_rob_pub_->publish(radar_rob2_msg);
  diag_radar_rob_->tick(time_stamp);
  // diag_radar_rob_->tick(radar_manager_in_t);
  // diag_radar_rob_->tick(radar_manager_out_t); 
}

void CRadarBase::resetPacketsFromDaqList(uint16_t daq_list_id)
{
  for (auto &vdb_packet : vdb_packets_)
  {
    if (vdb_packet.getDaqListId() == daq_list_id)
    {
      // cache number of non-zero elements
      auto it = data_cache_.find(vdb_packet.getName());
      if (it != data_cache_.end())
      {
        auto data = vdb_packet.getData();
        it->second.num_nonzero = std::count_if(data.begin(), data.end(), [](uint8_t i) {
          return i != 0;
        });
      }
      // reset vdb packet
      vdb_packet.reset_received();
    }
  }
}

void CRadarBase::diagnoseVDBPackets(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  auto level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string diag_msgs;
  for (auto &it : data_cache_)
  {
    if (it.second.num_nonzero < it.second.req_num_nonzero)
    {
      level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      diag_msgs += ("last " + it.first + " data is invalid!\n");
    }
    stat.add(it.first + " num_nonzero", it.second.num_nonzero);
    stat.add(it.first + " req_num_nonzero", it.second.req_num_nonzero);
  }
  if (!diag_msgs.empty())
    diag_msgs.resize(diag_msgs.size() - 1);

  stat.summary(level, diag_msgs);
}

void CRadarBase::receiveXcpRadarData()
{
  tcp_header header_temp;

  // identification field size in bytes (radar_gen5_plus (=LRR5): 2 bytes)
  size_t identification_field_size = sensor_settings_.getIdFieldSize();
  RCLCPP_DEBUG_STREAM(node_->get_logger(), "ID field size: " << sensor_settings_.getIdFieldSize());

  // timestamp size in bytes (radar_gen5_plus (=LRR5): 4 bytes, first package only)
  size_t timestamp_size = sensor_settings_.getTimestampSize();
  RCLCPP_DEBUG_STREAM(node_->get_logger(), "Timestamp size: " << sensor_settings_.getTimestampSize());

  RCLCPP_DEBUG_STREAM(node_->get_logger(), "max odt entry size: " << sensor_settings_.getMaxOdtEntrySizeDaq());

  std::stringstream error_stream;
  error_stream << basic_info_.device_position
               << ": Data Acquisition: Socket was closed by XCP slave";

  ssize_t rd = recv(SocketFD, &header_temp, sizeof(tcp_header), MSG_WAITALL);

  uint8_t pid_temp;
  if (rd > 0)
  {
    // read the PID Byte
    rd = recv(SocketFD, &pid_temp, 1, MSG_WAITALL);
    if (rd > 0) // has something been read
    {
      if (pid_temp >= 0xFC) // is message CTO and not DTO?
      {
        size_t header_size = 4;
        uint8_t cto_buf[header_temp.len + header_size];
        memcpy(cto_buf, &header_temp, sizeof(tcp_header));
        cto_buf[sizeof(tcp_header)] = pid_temp;
        // read the rest of the CTO frame
        rd = recv(
            SocketFD,
            &cto_buf[sizeof(tcp_header) + 1],
            (header_temp.len - sizeof(tcp_header) - 1),
            MSG_WAITALL);
        if (rd > 0) // has something been read
        {
          xcp_message_cto *cto_message = mapToCtoMessage(cto_buf);
          RCLCPP_WARN_STREAM(node_->get_logger(),
                             basic_info_.device_position << ": Communication: CTO message received, "
                                                         << "cto_message->pid: " << cto_message->pid
                                                         << ", cto_message->data[0]: " << cto_message->data[0]);
          printCtoInfo(cto_message);
          freeCtoMessage(&cto_message);
        }
        else if (rd < 0)
        {
          RCLCPP_ERROR_STREAM(node_->get_logger(),
                              "could not read cto_buf[sizeof(tcp_header)+1]; errno = "
                                  << std::strerror(errno) << " -- " << error_stream.str());
          rd = 0;
          stage = ERROR_STATE;
        }
        return;
      }
      else
      {
        RCLCPP_DEBUG_STREAM(node_->get_logger(),
                            basic_info_.device_position << ": Data Acquisition: DTO message with PID "
                                                        << (int)pid_temp << " received (rd = " << rd
                                                        << ", xcp package len = " << header_temp.len
                                                        << ", ctr = " << header_temp.ctr << ")");

        memcpy(&dto_message.header, &header_temp, sizeof(tcp_header));
        dto_message.pid = pid_temp;
        dto_message.dto_list_id = 0;
        dto_message.unused3 = 0;
        dto_message.unused4 = 0;

        // read the rest of the identification field
        rd = recv(SocketFD, &dto_message.dto_list_id, identification_field_size - 1, MSG_WAITALL);
        if (rd < 0)
        {
          RCLCPP_ERROR_STREAM(node_->get_logger(),
                              "could not read dto_message.dto_list_id; errno = " << std::strerror(errno) << " -- "
                                                                                 << error_stream.str());
          rd = 0;
          stage = ERROR_STATE;
        }

        size_t header_size;
        if (pid_temp == 0) // First DTO Packet received
        {
          header_size = identification_field_size + timestamp_size;

          // set receive time stamp  //lnl, timer
          // time_stamp = node_->now();
          const auto t_temp = node_->now();
          time_stamp = t_temp;
          radar_manager_in_t = t_temp; //lnl, timer

          // get radar time stamp
          rd = recv(SocketFD, &dto_message.timestamp_byte1, 4, MSG_WAITALL);
        }
        else // Following DTO Packet received
        {
          header_size = identification_field_size;
        }

        rd =
            recv(SocketFD, &dto_message.data[0], dto_message.header.len - header_size, MSG_WAITALL);
        if (rd > 0)
        {
          dto_message_received_ = true;
          RCLCPP_DEBUG_STREAM(node_->get_logger(),
                              basic_info_.device_position << ": Data Acquisition: " << header_size << " (header) + "
                                                          << dto_message.header.len - header_size
                                                          << " (data) bytes received, (rd_data = " << rd
                                                          << ", ctr = " << dto_message.header.ctr << ")");
        } // content was read
        else if (rd < 0)
        {
          RCLCPP_ERROR_STREAM(node_->get_logger(),
                              "could not receive dto_message.data[0]; errno = " << std::strerror(errno) << " -- "
                                                                                << error_stream.str());
          rd = 0;
          stage = ERROR_STATE;
        }

        //lnl, timer
        RCLCPP_INFO_STREAM(node_->get_logger(), "000_dto_messsgae_on_timer:" << time_stamp.seconds() << "::");
        radar_tcp_sent = printDtoInfo(&dto_message);
      }
    }
    else if (rd < 0)
    {
      RCLCPP_ERROR_STREAM(node_->get_logger(),
                          "could not receive pid_temp; errno = " << std::strerror(errno) << " -- "
                                                                 << error_stream.str());
      rd = 0;
      stage = ERROR_STATE;
      return;
    }
  } // size byte was read
  else if (rd < 0)
  {
    RCLCPP_ERROR_STREAM(node_->get_logger(),
                        "could not receive header_temp; errno = " << std::strerror(errno) << " -- "
                                                                  << error_stream.str());
    rd = 0;
    // stage = ERROR_STATE;
  }
}

/*Read static data with startAddress and length. The final result is written into the array, where
 * f_retrunData_pui8 is the starting point. The array must be of size= f_length.*/
int CRadarBase::readStaticData(
    uint32_t f_startAddress, uint32_t f_length, uint8_t f_returnData_pui8[])
{
  // size of maximum chunk that can be read a the same time. 254 is number that returns without
  // error..
  static const uint8_t maxChunk = 254;

  uint32_t nrOfChunks = f_length / (static_cast<uint32_t>(maxChunk));
  uint32_t leftovers = f_length % (static_cast<uint32_t>(maxChunk));

  // read out all complete chunks and lastly the leftover
  for (uint32_t count = 0; count < nrOfChunks + 1; count++)
  {
    uint8_t datalengthRequested;
    if (count != nrOfChunks)
    {
      // not last entry: use max chunk
      datalengthRequested = maxChunk;
    }
    else
    {
      // last entry: process only leftovers
      if (leftovers == 0)
      {
        break; // exit for loop, as no further action is needed
      }
      datalengthRequested = static_cast<uint8_t>(leftovers);
    }

    uint32_t chunkOffset = static_cast<uint32_t>(maxChunk) * count; // chunk offset starting at
                                                                    // zero
    uint32_t startAddressOfChunk =
        f_startAddress + chunkOffset; // this calculates start address of each chunk

    uint8_t data_tmp[7] = {
        datalengthRequested,
        0x00 /*unused*/,
        0x00 /*address extension*/,
        static_cast<uint8_t>(
            (startAddressOfChunk & 0xFF000000) >> 24), /*address devided into 4 bytes for sending*/
        static_cast<uint8_t>(
            (startAddressOfChunk & 0x00FF0000) >> 16), /*address devided into 4 bytes for sending*/
        static_cast<uint8_t>(
            (startAddressOfChunk & 0x0000FF00) >> 8), /*address devided into 4 bytes for sending*/
        static_cast<uint8_t>(
            startAddressOfChunk & 0x000000FF)}; /*address devided into 4 bytes for sending*/

    // receive static data command => use SHORT_UPLOAD. data to be returned is configured in
    // data_tmp[7] by: { dataLengthRequested to be returned, unused byte, data extension byte
    // (always use 0x00) and the start address requested} The command is first argument, then the
    // data_tmp structure, then the size of data_tmp, and lastly the socket.
    sendCtoMessage(CC_SHORT_UPLOAD, (uint8_t *)&data_tmp, 7, SocketFD);

    bool t_waitForResponse_b = true; // ensure while loop is entered

    // SERV comes in addition to RES/ERR, read next message
    xcp_message_cto *cto_message = NULL;
    while (t_waitForResponse_b == true)
    {
      std::this_thread::sleep_for(std::chrono::microseconds(200)); // let process sleep for very short time while waiting for an answer

      freeCtoMessage(&cto_message);
      cto_message = receiveCtoMessage(SocketFD, sensor_settings_.getMaxCtoSize());
      printCtoInfo(cto_message);

      t_waitForResponse_b = false;
      if (cto_message == NULL)
      {
        t_waitForResponse_b = true;
      }
      else // message is not null, so data can be accessed
      {
        if ((cto_message->pid == PID_SERV) || (cto_message->header.len == 1))
        {
          t_waitForResponse_b = true;
        }
        if (cto_message->pid == PID_ERR)
        {
          RCLCPP_ERROR_STREAM(node_->get_logger(), "Error receiving response of static data request");
          return -1; // error
        }
      }
    }

    for (uint32_t j = 0; j < datalengthRequested; j++)
    {
      f_returnData_pui8[chunkOffset + j] = cto_message->data[j];
    }
    freeCtoMessage(&cto_message);
  }
  return 0; // ok.
}

void CRadarBase::GetStaticPackets(StaticData &staticDataSt)
{
  std::unique_lock<std::mutex> lockData(
      staticDataSt
          .staticPacketsMutex); // note: compiler warning for unused data lockData is normal!

  bool t_anyStaticDataInitialisedSuccess = false;
  for (std::vector<staticPackets_st>::iterator staticPacket_it = staticPackets_.begin();
       staticPacket_it != staticPackets_.end();
       ++staticPacket_it)
  {
    uint8_t *dataReturnArrayPtr = NULL;
    std::string tempString;
    if (staticPacket_it->name == KEYWORD_FACTORY_DATA)
    {
      dataReturnArrayPtr = &staticDataSt.factoryData[0];
    }
    else if (staticPacket_it->name == KEYWORD_SW_VERSION)
    {
      staticPacket_it->datasize = swVersionFromConfig_.size();
      tempString.resize(staticPacket_it->datasize, '\0');
      dataReturnArrayPtr = reinterpret_cast<uint8_t *>(&tempString[0]);
    }
    else if (staticPacket_it->name == KEYWORD_BUILD_INFO)
    {
      staticPacket_it->datasize = buildInfoFromConfig_.size();
      tempString.resize(staticPacket_it->datasize, '\0');
      dataReturnArrayPtr = reinterpret_cast<uint8_t *>(&tempString[0]);
    }

    if (dataReturnArrayPtr == nullptr)
    {
      RCLCPP_ERROR_STREAM(node_->get_logger(),
                          basic_info_.device_position << ": Reading static data failed. Radar driver stopped.");

      return; // stop driver
    }

    // Read static data via XCP
    int failDataReadFromXCP =
        readStaticData(staticPacket_it->addr, staticPacket_it->datasize, dataReturnArrayPtr);

    if (failDataReadFromXCP != 0)
    {
      RCLCPP_ERROR_STREAM(node_->get_logger(),
                          basic_info_.device_position << ": Reading static data failed. Radar driver stopped.");

      return; // stop driver
    }

    if (staticPacket_it->name == KEYWORD_FACTORY_DATA)
    {
      staticDataSt_.factoryDataRead = true;
    }
    else if (staticPacket_it->name == KEYWORD_SW_VERSION)
    {
      staticDataSt.swVersionFromECU = tempString;

      if (swVersionFromConfig_.compare(SW_VERSION_DEFAULT) == 0)
      {
        RCLCPP_ERROR_STREAM(node_->get_logger(),
                            basic_info_.device_position
                                << ": ECU " << KEYWORD_SW_VERSION
                                << " not configured in radar param file. (old config assumed... continuing radar "
                                   "driver, but please adapt config)");
      }
      else if (swVersionFromConfig_.compare(staticDataSt.swVersionFromECU) != 0) // if strings
                                                                                 // are not equal
      {
        RCLCPP_ERROR_STREAM(node_->get_logger(),
                            basic_info_.device_position << ": ECU " << KEYWORD_SW_VERSION
                                                        << " mismatch. Config expects '" << swVersionFromConfig_
                                                        << "'. ECU has '" << staticDataSt.swVersionFromECU
                                                        << "'. Radar driver stopped.");
        return; // stop driver
      }
      else
      {
        RCLCPP_INFO_STREAM(node_->get_logger(),
                           basic_info_.device_position << ": ECU " << KEYWORD_SW_VERSION
                                                       << " matches config. It is: '"
                                                       << staticDataSt.swVersionFromECU << "'.");
      }
    }
    else if (staticPacket_it->name == KEYWORD_BUILD_INFO)
    {
      staticDataSt.buildInfoFromECU = tempString;

      if (buildInfoFromConfig_.compare(BUILD_INFO_DEFAULT) == 0)
      {
        RCLCPP_ERROR_STREAM(node_->get_logger(),
                            basic_info_.device_position
                                << ": ECU " << KEYWORD_BUILD_INFO
                                << " not configured in radar param file. (old config assumed... continuing radar "
                                   "driver, but please adapt config)");
      }
      else if (buildInfoFromConfig_.compare(staticDataSt.buildInfoFromECU) != 0) // if strings
                                                                                 // are not equal
      {
        RCLCPP_ERROR_STREAM(node_->get_logger(),
                            basic_info_.device_position << ": ECU " << KEYWORD_BUILD_INFO
                                                        << " mismatch. Config expects: '" << buildInfoFromConfig_
                                                        << "'. ECU has: '" << staticDataSt.buildInfoFromECU
                                                        << "'. Radar driver stopped.");
        return; // stop driver
      }
      else
      {
        RCLCPP_INFO_STREAM(node_->get_logger(),
                           basic_info_.device_position << ": ECU " << KEYWORD_BUILD_INFO
                                                       << " matches config. It is: '"
                                                       << staticDataSt.buildInfoFromECU << "'.");
      }
    }
    t_anyStaticDataInitialisedSuccess =
        true; // set to true, if it ever got here... I.e at least one static data entry is defined
              // and no error exit occured.
  }           // end for loop over all static data

  // return factroy data as initialised if any factory data was read successfully.
  staticDataSt.staticDataInitialised = t_anyStaticDataInitialisedSuccess;
}

void CRadarBase::connect()
{
  // (re)initialice variables
  stage = INIT_STATE;
  vdb_packet_number = 0;
  odt_div_number = 0;
  odt_pid = 0;
  odt_entry_number = 0;
  current_daq_list_index = 0;

  while (stage != DATA_AQUISITION_STATE)
  {
    // let process sleep for very short time. This ensures that in an error state,
    // this process will not run at 100%. Here, because it can stay in this while loop also...
    std::this_thread::sleep_for(std::chrono::microseconds(200));

    if (stage == INIT_STATE)
    {
      RCLCPP_DEBUG_STREAM(node_->get_logger(),
                          basic_info_.device_position << ": Initialization: Start to connect to sensor...");

      if (-1 != SocketFD)
      {
        closeSocket(SocketFD);
        SocketFD = -1;
      }
      // make socket blocking with timeout
      struct timeval tv = {tcp_timeout_, 0};
      SocketFD = createSocket(basic_info_.ip.c_str(), basic_info_.port);
      // ==> Write factory data to class member
      // ==> compaport);
      setsockopt(SocketFD, SOL_SOCKET, SO_RCVTIMEO, (struct timeval *)&tv, sizeof(struct timeval));

      // Connect to radar
      connectXcpRadar();

      if (stage == CONNECTED_STATE) // connection established?
      {
        if (staticPackets_.size() > 0)
        {
          RCLCPP_DEBUG_STREAM(node_->get_logger(),
                              basic_info_.device_position
                                  << ": Communication: Send request for shortUpload to get factory data");
          // read static data.
          // ==> Write factory data to class member
          // ==> compare config revision and compile string to those read from the ECU

          // Read static data, where structure is mutex-locked in the process.
          // Note: static packets are for static memory (Addresses < 0x40000000), only used if such
          // data is requested in the params file.
          GetStaticPackets(staticDataSt_);
        }
      }
    }

    else if ((stage != FATAL_STATE) && (stage != ERROR_STATE))
    {
      // Config Radar for data aquisition
      RCLCPP_DEBUG_STREAM(node_->get_logger(),
                          basic_info_.device_position << ": Initialization: Start sensor configuration...");

      configXcpRadar();
    }
    else // (stage == (FATAL_STATE || ERROR_STATE))
    {
      // return for error handling
      return;
    }
  }
}

void CRadarBase::connectXcpRadar()
{
  std::stringstream error_stream;
  RCLCPP_DEBUG_STREAM(node_->get_logger(),
                      "IP Address: " << basic_info_.ip << ", Port: " << basic_info_.port
                                     << ", SocketFD: " << SocketFD);
  if (basic_info_.ip != "" && basic_info_.port != 0)
  {
    if (SocketFD != -1)
    {
      // send CONNECT CTO
      uint8_t data = 0; // 0: normal mode, 1: user defined mode
      RCLCPP_DEBUG_STREAM(node_->get_logger(), basic_info_.device_position << ": Communication: Sending frame CC_CONNECT");
      sendCtoMessage(CC_CONNECT, &data, 1, SocketFD);

      xcp_message_cto *cto_message = receiveCtoMessage(SocketFD, /* buffer size = */ 1024);
      if (cto_message != NULL)
      {
        RCLCPP_DEBUG_STREAM(node_->get_logger(),
                            basic_info_.device_position << ": Communication: CTO message received - CONNECT");
        RCLCPP_DEBUG_STREAM(node_->get_logger(),
                            "PID: " << int(cto_message->pid) << " Data: " << int(cto_message->data[0]));

        if (cto_message->pid == PID_RES)
        {
          // copy ressource info
          sensor_settings_.setConnectInfo(cto_message->pid, cto_message->data);

          // print ressource info
          RCLCPP_INFO_STREAM(node_->get_logger(), "MAX CTO SIZE = " << sensor_settings_.getMaxCtoSize());
          RCLCPP_INFO_STREAM(node_->get_logger(), "MAX DTO SIZE = " << sensor_settings_.getMaxDtoSize());
          RCLCPP_INFO_STREAM(node_->get_logger(),
                             "XCP protocol layer version  = "
                                 << (uint)sensor_settings_.getXcpProtocolLayerVersion());
          RCLCPP_INFO_STREAM(node_->get_logger(),
                             "XCP transport layer version = "
                                 << (uint)sensor_settings_.getXcpTransportLayerVersion());
          RCLCPP_INFO_STREAM(node_->get_logger(),
                             basic_info_.device_position << ": Communication: Sensor connected successfully");

          stage = CONNECTED_STATE;
        }
        else
        {
          error_stream << basic_info_.device_position << ": Communication: Connection rejected!";
          auto &clk = *node_->get_clock();
          RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), clk, 1000, error_stream.str());
          stage = ERROR_STATE;
        }
        //        printCtoInfo(cto_message);
        freeCtoMessage(&cto_message);
        return;
      }
      else
      {
        error_stream << basic_info_.device_position
                     << ": Communication connectXcpRadar(): Timeout! Sensor does not answer!";
        RCLCPP_ERROR_STREAM(node_->get_logger(), error_stream.str());
        closeSocket(SocketFD);
        SocketFD = -1;
        stage = ERROR_STATE;
        return;
      }
    }
    else
    {
      stage = ERROR_STATE;
      error_stream << basic_info_.device_position
                   << ": Communication: Unable to create TCP-Socket!";
      RCLCPP_ERROR_STREAM(node_->get_logger(), error_stream.str());
      return;
    }
  }
  else
  {
    stage = FATAL_STATE;
    error_stream << basic_info_.device_position
                 << ": Initialization: Invalid config file or no config file found!";
    RCLCPP_FATAL_STREAM(node_->get_logger(), error_stream.str());
    return;
  }
}

void CRadarBase::configXcpRadar()
{
  /*------------- process input and decide what to send ----------------------*/
  /*------------- Initial commands -------------------------------------------*/
  // send the GET_STATUS after we got the SERV
  if ((stage == CONNECTED_STATE))
  {
    RCLCPP_DEBUG_STREAM(node_->get_logger(), basic_info_.device_position << ": Communication: Sending frame CC_GET_STATUS");
    sendCtoMessage(CC_GET_STATUS, NULL, 0, SocketFD);
    stage = STATUS_WAIT_RES; // GOT_STATUS_STATE;
  }

  // send a GET_DAQ_PROCESSOR_INFO
  else if (stage == GOT_STATUS_STATE)
  {
    RCLCPP_DEBUG_STREAM(node_->get_logger(),
                        basic_info_.device_position << ": Communication: Sending frame CC_GET_DAQ_PROCESSOR_INFO");
    sendCtoMessage(CC_GET_DAQ_PROCESSOR_INFO, NULL, 0, SocketFD);
    stage = GOT_DAQ_PROCESSOR_INFO_STATE;
  }

  // we send a GET_DAQ_RESOLUTION_INFO
  else if (stage == GOT_DAQ_PROCESSOR_INFO_STATE)
  {
    RCLCPP_DEBUG_STREAM(node_->get_logger(),
                        basic_info_.device_position << ": Communication: Sending frame CC_GET_DAQ_RESOLUTION_INFO");
    sendCtoMessage(CC_GET_DAQ_RESOLUTION_INFO, NULL, 0, SocketFD);
    stage = GOT_DAQ_RESOLUTION_INFO_STATE;
  }

  //-------------------DAQ allocation commands  -----------------------------------------------

  // free existing daqs
  else if (stage == GOT_DAQ_RESOLUTION_INFO_STATE)
  {
    RCLCPP_DEBUG_STREAM(node_->get_logger(), basic_info_.device_position << ": Communication: Sending frame CC_FREE_DAQ");
    sendCtoMessage(CC_FREE_DAQ, NULL, 0, SocketFD);
    current_daq_list_index = 0;
    stage = SENT_FREE_DAQ_STATE;
  }

  // allocate daq lists
  else if (stage == SENT_FREE_DAQ_STATE)
  {
    RCLCPP_DEBUG_STREAM(node_->get_logger(), basic_info_.device_position << ": Communication: Sending frame CC_ALLOC_DAQ");

    uint8_t data_tmp[3];
    memset((void *)data_tmp, 0, 3);
    int num_daqs;
    num_daqs = basic_info_.daq_list_.size();
    if (sensor_settings_.isIntelByteOrder())
    {
      data_tmp[1] = num_daqs;
    }
    else
    {
      data_tmp[2] = num_daqs;
    }

    sendCtoMessage(CC_ALLOC_DAQ, data_tmp, 3, SocketFD);
    vdb_packets_.clear();
    stage = ALLOC_DAQ_STATE;
  }

  // allocate ODTs to daq list
  else if (stage == ALLOC_DAQ_STATE)
  {
    // set up vdb packets
    int number_of_added_vdbs = addVdbPackets(current_daq_list_index);
    createRobPackets();

    RCLCPP_DEBUG_STREAM(node_->get_logger(),
                        basic_info_.device_position
                            << ", list entry " << current_daq_list_index
                            << ": Communication: Sending frame CC_ALLOC_ODT with total ODT_COUNT "
                            << number_of_added_vdbs);

    uint8_t data_tmp[4];
    memset((void *)data_tmp, 0, 4);
    if (sensor_settings_.isIntelByteOrder())
    {
      data_tmp[1] = current_daq_list_index;
    }
    else
    {
      data_tmp[2] = current_daq_list_index;
    }
    // previously single odt list: vdb_packets_.back().getTotalNumOdts();
    data_tmp[3] = number_of_added_vdbs;
    sendCtoMessage(CC_ALLOC_ODT, data_tmp, 4, SocketFD);

    current_daq_list_index++;                                   // add more daqs
    if (current_daq_list_index >= basic_info_.daq_list_.size()) // all daq lists requested?
    {
      current_daq_list_index = 0;
      odt_pid = 0;
      stage = ALLOC_ODT_STATE; // --> Next Stage
    }
    else
    {
      /* stay in same stage */
    }
  }

  //-------------------ODT entries allocation commands  -------------------------------------------
  else if (stage == ALLOC_ODT_STATE)
  {
    uint16_t odt_offset_number = vdb_packets_[vdb_packet_number].getFirstOdtPid();
    if (odt_offset_number < 0)
    {
      // something went wrong: vdb_packets_[vdb_packet_number] does not contain packets!
      std::stringstream error_stream;
      error_stream << basic_info_.device_position << ": Uninitialized vdb packet";
      RCLCPP_ERROR_STREAM(node_->get_logger(), error_stream.str());
      stage = FATAL_STATE;
      return; // go back for error handling
    }
    cto_alloc_odt_entry_data data_tmp;
    memset(&data_tmp, 0, sizeof(cto_alloc_odt_entry_data));
    if (sensor_settings_.isIntelByteOrder())
    {
      data_tmp.daq_list_numberbyte1 =
          (uint8_t)vdb_packets_[vdb_packet_number].getDaqListId(); // current_daq_list_index;
      data_tmp.daq_list_numberbyte2 = 0;
    }
    else
    {
      data_tmp.daq_list_numberbyte1 = 0;
      data_tmp.daq_list_numberbyte2 =
          (uint8_t)vdb_packets_[vdb_packet_number].getDaqListId(); // current_daq_list_index;
    }
    data_tmp.odt_number = odt_div_number + odt_offset_number;
    CODT *odt_tmp =
        vdb_packets_[vdb_packet_number].getOdtHostPtr(odt_div_number + odt_offset_number);

    data_tmp.odt_entries_count = odt_tmp->getNumEntries();

    sendCtoMessage(
        CC_ALLOC_ODT_ENTRY, (uint8_t *)&data_tmp, sizeof(cto_alloc_odt_entry_data), SocketFD);

    RCLCPP_DEBUG_STREAM(node_->get_logger(),
                        basic_info_.device_position
                            << ":"
                            << " Communication: Sending frame CC_ALLOC_ODT_ENTRY , ODT #"
                            << int(odt_div_number + odt_offset_number) << " with " << odt_tmp->getNumEntries()
                            << " ODT-Entries "
                            << " of " << vdb_packets_[vdb_packet_number].getNumOdts() << " ODTs"
                            << " of VDB #" << vdb_packet_number << " of " << vdb_packets_.size() << " VDBs ");

    odt_pid++;
    odt_div_number++;

    if (odt_div_number >= vdb_packets_[vdb_packet_number].getNumOdts()) // all ODTs configured
    {
      vdb_packet_number++;
      odt_div_number = 0;

      if (vdb_packet_number >= vdb_packets_.size()) // all VDB-Packets configured
      {
        vdb_packet_number = 0;
        current_daq_list_index = 0;
        odt_pid = 0;
        odt_div_number = 0;
        odt_entry_number = 0;
        stage = DAQ_WRITE_STATE; // --> Next Stage
      }
    }
  }

  //------------ Setting pointer ----------
  else if (stage == DAQ_WRITE_STATE)
  {
    uint16_t odt_offset_number = vdb_packets_[vdb_packet_number].getFirstOdtPid();
    if (odt_offset_number < 0)
    {
      // something went wrong: vdb_packets_[vdb_packet_number] does not contain packets!
      std::stringstream error_stream;
      error_stream << basic_info_.device_position << ": Uninitialized vdb packet";
      RCLCPP_ERROR_STREAM(node_->get_logger(), error_stream.str());
      stage = FATAL_STATE;
      return; // go back for error handling
    }
    RCLCPP_DEBUG_STREAM(node_->get_logger(),
                        basic_info_.device_position << ": Communication: Sending frame CC_SET_DAQ_PTR to "
                                                    << int(odt_div_number));
    cto_set_daq_pointer_data data_tmp;
    memset(&data_tmp, 0, sizeof(cto_set_daq_pointer_data));
    if (sensor_settings_.isIntelByteOrder())
    {
      data_tmp.daq_list_numberbyte1 = (uint8_t)vdb_packets_[vdb_packet_number].getDaqListId();
      data_tmp.daq_list_numberbyte2 = 0;
    }
    else
    {
      data_tmp.daq_list_numberbyte1 = 0;
      data_tmp.daq_list_numberbyte2 = (uint8_t)vdb_packets_[vdb_packet_number].getDaqListId();
    }
    data_tmp.odt_number = odt_div_number + odt_offset_number;
    data_tmp.odt_entry_number = odt_entry_number;
    sendCtoMessage(CC_SET_DAQ_PTR, (uint8_t *)&data_tmp, sizeof(cto_set_daq_pointer_data), SocketFD);
    stage = DAQ_WRITE_STATE2; // continue with writing all odt entries for this odt
  }

  else if (stage == DAQ_WRITE_STATE2)
  {
    uint16_t odt_offset_number = vdb_packets_[vdb_packet_number].getFirstOdtPid();
    if (odt_offset_number < 0)
    {
      // something went wrong: vdb_packets_[vdb_packet_number] does not contain packets!
      std::stringstream error_stream;
      error_stream << basic_info_.device_position << ": Uninitialized vdb packet";
      RCLCPP_ERROR_STREAM(node_->get_logger(), error_stream.str());
      stage = FATAL_STATE;
      return; // go back for error handling
    }
    cto_write_daq_data data_tmp;
    memset(&data_tmp, 0, sizeof(cto_write_daq_data));
    data_tmp.bit_offset = 0xFF;
    CODT *odt_tmp =
        vdb_packets_[vdb_packet_number].getOdtHostPtr(odt_div_number + odt_offset_number);

    data_tmp.daq_elem_size = odt_tmp->getSizeOfEntry(
        odt_entry_number); // configuration->odt_entry_size[config_counter];
    uint32_t targetAddress =
        odt_tmp->getTargetBaseAddress() + odt_tmp->getOffsetOfEntry(odt_entry_number);
    if (sensor_settings_.isIntelByteOrder())
    {
      data_tmp.daq_elem_addr_byte4 = ((targetAddress) >> (3 * 8));
      data_tmp.daq_elem_addr_byte3 = ((targetAddress << (1 * 8)) >> (3 * 8));
      data_tmp.daq_elem_addr_byte2 = ((targetAddress << (2 * 8)) >> (3 * 8));
      data_tmp.daq_elem_addr_byte1 = ((targetAddress << (3 * 8)) >> (3 * 8));
    }
    else
    {
      data_tmp.daq_elem_addr_byte1 = ((targetAddress) >> (3 * 8));
      data_tmp.daq_elem_addr_byte2 = ((targetAddress << (1 * 8)) >> (3 * 8));
      data_tmp.daq_elem_addr_byte3 = ((targetAddress << (2 * 8)) >> (3 * 8));
      data_tmp.daq_elem_addr_byte4 = ((targetAddress << (3 * 8)) >> (3 * 8));
    }
    RCLCPP_DEBUG_STREAM(node_->get_logger(),
                        basic_info_.device_position
                            << ": Communication: Sending frame CC_WRITE_DAQ for ODT-Entry #" << (int)odt_entry_number
                            << " of " << odt_tmp->getNumEntries() << " Entries"
                            << " of ODT #" << (int)odt_pid << "-" << (int)odt_div_number + odt_offset_number << " of "
                            << vdb_packets_[vdb_packet_number].getNumOdts() << " ODTs"
                            << " of VDB #" << (int)vdb_packet_number << " of " << vdb_packets_.size() << " VDBs, "
                            << (int)data_tmp.daq_elem_size << " bytes");
    sendCtoMessage(CC_WRITE_DAQ, (uint8_t *)&data_tmp, sizeof(cto_write_daq_data), SocketFD);
    odt_entry_number++;
    if (odt_entry_number >= odt_tmp->getNumEntries()) // all ODT-Entries of current ODT configured
    {
      odt_entry_number = 0;
      odt_pid++;
      odt_div_number++;

      if (odt_div_number >= vdb_packets_[vdb_packet_number]
                                .getNumOdts()) // all ODTs of current VDB-Packet configured
      {
        vdb_packet_number++;
        odt_div_number = 0;

        if (vdb_packet_number >= vdb_packets_.size()) // all VDB-Packets configured
        {
          vdb_packet_number = 0;
          current_daq_list_index = 0;
          odt_pid = 0;
          odt_div_number = 0;
          odt_entry_number = 0;
          vdb_packet_number = 0;
          stage = DAQ_WRITE_STATE4; // --> Next Stage
        }
        else // odt of current vdb done, but other vdbs left
        {
          stage = DAQ_WRITE_STATE;
        }
      }
      else // odt entries of current odt done, but other odts left
      {
        stage = DAQ_WRITE_STATE;
      }
    } // else continue with this odt and stay in DAQ_WRITE_STATE2
  }

  //-------------------------------- set daq list mode ..-------------------------------
  else if (stage == DAQ_WRITE_STATE4)
  {
    RCLCPP_DEBUG_STREAM(node_->get_logger(),
                        basic_info_.device_position << ": Communication: Sending frame CC_SET_DAQ_LIST_MODE");
    cto_set_daq_list_mode_data data_tmp;
    memset(&data_tmp, 0, sizeof(cto_set_daq_list_mode_data));
    if (sensor_settings_.isIntelByteOrder())
    {
      data_tmp.daq_list_numberbyte1 = current_daq_list_index;
      data_tmp.daq_list_numberbyte2 = 0;
    }
    else
    {
      data_tmp.daq_list_numberbyte1 = 0;
      data_tmp.daq_list_numberbyte2 = current_daq_list_index;
    }
    data_tmp.mode = 16; // 0x10, timestamp mode

    RCLCPP_DEBUG_STREAM(node_->get_logger(),
                        basic_info_.device_position << ": data_tmp.mode = " << (int)data_tmp.mode
                                                    << "; event_channel_number_ = "
                                                    << basic_info_.daq_list_[current_daq_list_index].event_channel);

    int ev_ch_number = basic_info_.daq_list_[current_daq_list_index].event_channel;
    if (sensor_settings_.isIntelByteOrder())
    {
      data_tmp.event_chan_nb_numberbyte1 = (ev_ch_number >> (8 * 0)) & 0xff;
      data_tmp.event_chan_nb_numberbyte2 = (ev_ch_number >> (8 * 1)) & 0xff;
    }
    else
    {
      data_tmp.event_chan_nb_numberbyte1 = (ev_ch_number >> (8 * 1)) & 0xff;
      data_tmp.event_chan_nb_numberbyte2 = (ev_ch_number >> (8 * 0)) & 0xff;
    }
    data_tmp.trans_rate_prescaler = 1;

    sendCtoMessage(
        CC_SET_DAQ_LIST_MODE, (uint8_t *)&data_tmp, sizeof(cto_set_daq_list_mode_data), SocketFD);

    if (current_daq_list_index >= (basic_info_.daq_list_.size() - 1)) // all daq lists requested?
    {
      current_daq_list_index = 0;
      stage = SET_DAQ_LIST_MODE_STATE; // --> Next Stage // trigger measurement
    }
    else
    {
      current_daq_list_index++; // add more daqs
      /* stay in same stage */
    }
  }

  //-------------------------------- start stop daq list -------------------------------
  else if (stage == SET_DAQ_LIST_MODE_STATE)
  {
    RCLCPP_DEBUG_STREAM(node_->get_logger(),
                        basic_info_.device_position << ": Communication: Sending frame CC_START_STOP_DAQ_LIST");
    cto_start_stop_daq_list_data data_tmp;
    memset(&data_tmp, 0, sizeof(cto_start_stop_daq_list_data));
    if (sensor_settings_.isIntelByteOrder())
    {
      data_tmp.daq_list_numberbyte1 = current_daq_list_index;
      data_tmp.daq_list_numberbyte2 = 0;
    }
    else
    {
      data_tmp.daq_list_numberbyte1 = 0;
      data_tmp.daq_list_numberbyte2 = current_daq_list_index;
    }
    data_tmp.mode = 2;
    sendCtoMessage(
        CC_START_STOP_DAQ_LIST,
        (uint8_t *)&data_tmp,
        sizeof(cto_start_stop_daq_list_data),
        SocketFD);

    if (current_daq_list_index >= (basic_info_.daq_list_.size() - 1)) // all daq lists requested?
    {
      current_daq_list_index = 0;
      stage = START_STOP_DAQ_LIST_STATE2; // --> Next Stage // trigger measurement
    }
    else
    {
      current_daq_list_index++; // add more daqs
      /* stay in same stage */
    }
  }

  //-------------------------------- get the clock   -------------------------------
  else if (stage == START_STOP_DAQ_LIST_STATE2)
  {
    RCLCPP_DEBUG_STREAM(node_->get_logger(),
                        basic_info_.device_position << ": Communication: Sending frame CC_GET_DAQ_CLOCK");
    sendCtoMessage(CC_GET_DAQ_CLOCK, NULL, 0, SocketFD);
    stage = GET_DAQ_CLOCK_STATE;
  }

  //-------------------------------- start stop synch  -------------------------------
  else if (stage == GET_DAQ_CLOCK_STATE)
  {
    RCLCPP_DEBUG_STREAM(node_->get_logger(),
                        basic_info_.device_position << ": Communication: Sending frame CC_START_STOP_SYNCH");
    uint8_t data_tmp = 1;
    sendCtoMessage(CC_START_STOP_SYNCH, &data_tmp, 1, SocketFD);
    stage = DATA_AQUISITION_STATE_WAITRES;
  }

  // final RES frame to confirm start of data acquisition
  else if (stage == DATA_AQUISITION_STATE_WAITRES)
  {
    //    printf("Entering into data acquisition stage\n");
    stage = DATA_AQUISITION_STATE;
    RCLCPP_DEBUG_STREAM(node_->get_logger(),
                        basic_info_.device_position << ": Communication: Sensor configuration finished");
    return; // from now on, we receive DTOs, not CTOs => bail out and do not interpret first data
            // packet as CTO answer
  }

  else
  {
    std::stringstream error_stream;
    error_stream << basic_info_.device_position << ": Undefined config stage.";
    RCLCPP_ERROR_STREAM(node_->get_logger(), error_stream.str());
    stage = FATAL_STATE;
    return; // go back for error handling
  }

  // wait for RES/ERR response message
  int wait_counter = 0;
  int max_wait_counter = 6;
  xcp_message_cto *cto_message = NULL;
  while (wait_counter < max_wait_counter)
  {
    // let process sleep for very short time while waiting for an answer
    std::this_thread::sleep_for(std::chrono::microseconds(200));

    freeCtoMessage(&cto_message);
    cto_message = NULL;
    cto_message = receiveCtoMessage(SocketFD, sensor_settings_.getMaxCtoSize());

    if (cto_message == NULL)
    {
      wait_counter++;
      continue;
    }
    else // message is not null, so data can be accessed
    {
      printCtoInfo(cto_message);

      if ((cto_message->pid == PID_SERV))
      {
        // ignore SERV messages
        RCLCPP_DEBUG_STREAM(node_->get_logger(),
                            basic_info_.device_position << ": Communication: CTO SERV message received: "
                                                        << (int)cto_message->data[0]);
        if (cto_message->data[0] == 0)
        {
          RCLCPP_DEBUG_STREAM(node_->get_logger(), "Reset requested.");
        }
        else if (cto_message->data[0] == 1)
        {
          // text transferred
          RCLCPP_INFO_STREAM(node_->get_logger(), cto_message->data + 1);
        }

        continue;
      }
      else if (cto_message->pid == PID_ERR)
      {
        RCLCPP_ERROR_STREAM(node_->get_logger(),
                            basic_info_.device_position << ": Communication: CTO message NOT acknowledged.");
        stage = ERROR_STATE;
        break;
      }
      else if (cto_message->pid == PID_RES)
      {
        if (stage == STATUS_WAIT_RES)
        {
          RCLCPP_DEBUG_STREAM(node_->get_logger(),
                              basic_info_.device_position << ": Communication: CTO message acknowledged - STATUS.");
          RCLCPP_DEBUG_STREAM(node_->get_logger(),
                              "STATUS current session status: " << int(cto_message->data[0]));
          RCLCPP_DEBUG_STREAM(node_->get_logger(),
                              "STATUS current ressource protection status: " << int(cto_message->data[1]));
          freeCtoMessage(&cto_message);
          cto_message = NULL;
          stage = GOT_STATUS_STATE;
        }
        else if (stage == GOT_DAQ_PROCESSOR_INFO_STATE)
        {
          sensor_settings_.setDaqProcessorInfo(cto_message->pid, cto_message->data);

          RCLCPP_DEBUG_STREAM(node_->get_logger(),
                              basic_info_.device_position
                                  << ": Communication: CTO message acknowledged - DAQ PROCESSOR INFO.");
          freeCtoMessage(&cto_message);
          cto_message = NULL;
        }
        else if (stage == GOT_DAQ_RESOLUTION_INFO_STATE)
        {
          sensor_settings_.setDaqResolutionInfo(cto_message->pid, cto_message->data);

          RCLCPP_DEBUG_STREAM(node_->get_logger(),
                              basic_info_.device_position
                                  << ": Communication: CTO message acknowledged - DAQ RESOLUTION INFO.");
          freeCtoMessage(&cto_message);
          cto_message = NULL;
        }
        else
        {
          RCLCPP_DEBUG_STREAM(node_->get_logger(),
                              basic_info_.device_position
                                  << ": Communication: CTO message acknowledged - other request.");
          freeCtoMessage(&cto_message);
          cto_message = NULL;
          //            stage = FATAL_STATE;
        }
        break;
      }
      else
      {
        RCLCPP_ERROR_STREAM(node_->get_logger(),
                            basic_info_.device_position << ": Communication: CTO message - unknown response."
                                                        << wait_counter);
        stage = ERROR_STATE;
        break;
      }
    }
  }
  if (wait_counter == max_wait_counter)
  {
    RCLCPP_ERROR_STREAM(node_->get_logger(),
                        basic_info_.device_position << ": Communication: Timeout - no message received.");
    stage = ERROR_STATE;
  }
}
