/**
 * Copyright (c) 2009, 2020 Robert Bosch GmbH and its subsidiaries.
 * This program and the accompanying materials are made available under
 * the terms of the Bosch Internal Open Source License v4
 * which accompanies this distribution, and is available at
 * http://bios.intranet.bosch.com/bioslv4.txt
 */

#ifndef RADAR_BASE_HPP_
#define RADAR_BASE_HPP_

#include <atomic>
#include <mutex>
#include <string>
#include <vector>

#include <stdint.h>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_updater/publisher.hpp>
// #include <ros/time.h>
#include "rclcpp/rclcpp.hpp"

#include <radar_msgs/msg/radar_rob2.hpp>
#include <radar_msgs/msg/radar_factory_data_array.hpp>
#include <radar_msgs/msg/radar_factory_data.hpp>

#include "radar_manager_ros/internal/communication/vdb.hpp"
#include "radar_manager_ros/sensor_base.hpp"

#include "radar_manager_ros/radar_manager_parameters.hpp"

extern "C"
{
#include "radar_manager_ros/internal/communication/xcp.h"
}
#define ROSCONSOLE_MIN_SEVERITY ROSCONSOLE_SEVERITY_DEBUG

#define DEVICE_DEFAULT "NO_RADAR"
#define DEVICE_POS_DEFAULT "NO_RADAR"
#define DEVICE_SW_REV_DEFAULT "NO_RADAR"

namespace radar_manager
{
  static const std::string SW_VERSION_DEFAULT = "No revision defined";
  static const std::string BUILD_INFO_DEFAULT = "No compile time string defined";

  // keywords expected in config file
  static const std::string KEYWORD_SW_VERSION = "SW_Version";
  static const std::string KEYWORD_BUILD_INFO = "Build_Info";
  static const std::string KEYWORD_FACTORY_DATA = "FACTORY_DATA";

  static const uint32_t FACTORY_DATA_NUM_BYTES = 65280;

  class CRadarBase : public CSensorBase
  {
  public:
    typedef struct radar_daq_list
    {
      // std::string daq_list_name;
      std::string params_file; // file of daq entries, with ECU addresses and lengths to be read
      int event_channel;       // event, on which the daq list is triggered
    } radar_daq_list;

    typedef struct radar_basic_info
    {
      std::string device_type;
      std::string device_position; // = frame_id
      std::string sw_revision;
      std::string ip;

      int port;

      std::vector<CRadarBase::radar_daq_list> daq_list_; // can be array of daq lists

    } radar_basic_info;

    typedef struct diag_info
    {
      size_t num_nonzero;     // the number of non zero bytes in vdb packet
      size_t req_num_nonzero; // the minimum required number of non zero bytes
    } diag_info;

    struct CRadarXcpSensorSettings
    {
      CRadarXcpSensorSettings()
      {
        memset(connect_info_, 0, 8);
        memset(daq_processor_info_, 0, 8);
        memset(daq_resolution_info_, 0, 8);
      }

      void setConnectInfo(uint8_t pid, uint8_t *data_in)
      {
        connect_info_[0] = pid;
        memcpy(connect_info_ + 1, data_in, 7);
      }

      void setDaqProcessorInfo(uint8_t pid, uint8_t *data_in)
      {
        daq_processor_info_[0] = pid;
        memcpy(daq_processor_info_ + 1, data_in, 7);
      }

      void setDaqResolutionInfo(uint8_t pid, uint8_t *data_in)
      {
        daq_resolution_info_[0] = pid;
        memcpy(daq_resolution_info_ + 1, data_in, 7);
      }

      // BYTE_ORDER = 0 means Intel format, BYTE_ORDER = 1 means Motorola format.
      // Motorola format means MSB on lower address/position.
      // LRR4: Motorola; RadarGen5Plus, LRR5: Intel
      bool isIntelByteOrder()
      {
        return (connect_info_[2] & 1) == 0;
      }

      uint8_t getXcpProtocolLayerVersion()
      {
        return connect_info_[6];
      }

      uint8_t getXcpTransportLayerVersion()
      {
        return connect_info_[7];
      }
      uint32_t getMaxCtoSize()
      {
        return (uint32_t)connect_info_[3];
      }

      uint32_t getMaxDtoSize()
      {
        if (isIntelByteOrder())
        {
          return ((uint32_t(connect_info_[5]) << 8) | (uint32_t(connect_info_[4])));
        }
        else
        {
          return ((uint32_t(connect_info_[4]) << 8) | (uint32_t(connect_info_[5])));
        }
      }

      int32_t getIdFieldSize()
      {
        uint8_t daqKeyByte = daq_processor_info_[7];
        uint8_t idFieldType = daqKeyByte >> 6;
        switch (idFieldType)
        {
        case 0:
          return 1; // absolute ODT number (1 byte PID, todo: in this case, check that PID is not
                    // completely turned off)
          break;
        case 1:
          return 2; // relative ODT number and absolute DAQ list number (BYTE) (1 byte PID, 1 byte
                    // DAQ)
          break;
        case 2:
          return 3; // relative ODT number and absolute DAQ list number (WORD) (1 byte PID, 2 bytes
                    // DAQ)
          break;
        case 3:
          return 4; // relative ODT number and absolute DAQ list number (WORD, aligned) (1 byte
                    // PID, 1 byte FILL, 2 bytes DAQ)
          break;
        default:
          return -1; // cannot occur
        }
      }

      int32_t getTimestampSize()
      {
        uint8_t timestampMode = daq_resolution_info_[5];
        uint8_t timestampSize = timestampMode & 0x07;
        return timestampSize;
      }

      int32_t getMaxOdtEntrySizeDaq()
      {
        return daq_resolution_info_[2];
      }

    private:
      uint8_t connect_info_[8];        // response to CONNECT
      uint8_t daq_processor_info_[8];  // response to GET_DAQ_PROCESSOR_INFO
      uint8_t daq_resolution_info_[8]; // response to GET_DAQ_RESOLUTION_INFO
    };

    CRadarBase(CRadarBase::radar_basic_info basic_info, std::shared_ptr<rclcpp::Node> node);

    virtual ~CRadarBase();
    virtual void Init();

    virtual void Start();
    virtual void Stop();

    // function to return factory data.
    virtual bool getFactoryData(radar_msgs::msg::RadarFactoryData &temp_radarFactoryData);

    /**
   * @brief
   * @return returns the daq list id all messages of which are received and ready to be published
   */
    void receiveAndPublish();

    bool isRunning()
    {
      return is_running_;
    }

    radar_basic_info basic_info_;
    CRadarXcpSensorSettings sensor_settings_;

    struct StaticData
    {
      uint8_t factoryData[FACTORY_DATA_NUM_BYTES];
      std::string swVersionFromECU;
      std::string buildInfoFromECU;
      bool factoryDataRead;
      bool staticDataInitialised;

      std::mutex staticPacketsMutex; // same lock for all sensors

      StaticData()
          : swVersionFromECU(SW_VERSION_DEFAULT),
            buildInfoFromECU(BUILD_INFO_DEFAULT),
            factoryDataRead(false),
            staticDataInitialised(false)
      {
        for (uint32_t i = 0; i < FACTORY_DATA_NUM_BYTES; i++)
        {
          // initialise entire data structure to zero, to easily mark factory data as not read, if not
          // available.
          factoryData[i] = 0;
        }
      }
    };
    StaticData staticDataSt_;

  protected:
    void connect();
    int addVdbPackets(int current_daq_list_index);
    int daq_list2vdb_index(int daq_list_ind, int pid) const;
    void createRobPackets(); //, bool valid, ROBData[] ROBs // uli8fe: re-build all ROB packets (from
                             // vdbPackets)

    void run();
    void connectXcpRadar();
    virtual void configXcpRadar();
    virtual void receiveXcpRadarData();

    int readStaticData(uint32_t f_startAddress, uint32_t f_length, uint8_t f_returnData_pui8[]);

    void GetStaticPackets(StaticData &staticDataSt);

    bool allVdbPacketsFromDaqListReceived(uint16_t daq_list_index);
    void publishRobPackets(uint16_t daq_list_id) const;
    void checkDuplicateRobPackets(uint16_t daq_list_id);
    void resetDuplicationIndicator();
    void resetPacketsFromDaqList(uint16_t daq_list_id);

    void diagnoseVDBPackets(diagnostic_updater::DiagnosticStatusWrapper &stat);

    std::shared_ptr<rclcpp::Node> node_;

    radar_msgs::msg::RadarROB2::SharedPtr radar_data_rob_;

    // static const ros::Duration RADAR_DATA_DELAY;

    // ros::Publisher radar_data_rob_pub_;
    rclcpp::Publisher<radar_msgs::msg::RadarROB2>::SharedPtr radar_data_rob_pub_;

    int tcp_timeout_, wait_time_;

    diagnostic_updater::Updater diag_updater_;
    rclcpp::TimerBase::SharedPtr diag_updater_timer_;
    std::unique_ptr<diagnostic_updater::TopicDiagnostic> diag_radar_rob_;

    std::vector<CVDBPacket> vdb_packets_;
    std::vector<PreviousMsg> vdb_packets_duplicates_;
    std::map<std::string, diag_info> data_cache_; // cached the number of non-zero elements

    struct staticPackets_st
    {
      std::string name;
      uint32_t addr;
      size_t datasize;
    };

    std::vector<staticPackets_st> staticPackets_;

    rclcpp::Time time_stamp;

    //lnl, timer
    // const std::chrono::seconds duration_1sec(1); //std chrono time
    rclcpp::Time radar_manager_in_t{0};
    // rclcpp::Time radar_manager_out_t{0};
    // rclcpp::Duration radar_manager_d_t{0}; //duration_1sec};
    uint32_t radar_tcp_sent{0};

    stage_enum stage;
    int SocketFD;
    // xcp_message_cto * cto_message;
    xcp_message_dto dto_message;
    uint16_t vdb_packet_number;
    uint16_t odt_div_number;  // index of odt in current vdb
    uint16_t odt_pid;         // index of odt (for all odts, globally) --> process ID (pid)
    uint8_t odt_entry_number; // index of entries in odt
    int reconnect_count;
    bool dto_message_received_;
    int current_daq_list_index;

    // was: boost
    std::shared_ptr<std::thread> run_thread_;
    std::atomic<bool> is_running_;

    std::string swVersionFromConfig_;
    std::string buildInfoFromConfig_;

    RadarManagerParameters params_;
  };
} // namespace radar_manager

#endif
