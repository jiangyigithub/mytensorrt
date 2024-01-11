#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include "std_msgs/msg/string.hpp"

#include "rclcpp/rclcpp.hpp"
#include <perception_kit_msgs/msg/objects.hpp>
#include <adma_msgs/msg/adma_data.hpp>
#include <adma_msgs/msg/adma_delta_data.hpp>
#include "geodetic_conv.hpp"

using namespace std::chrono_literals;

#define uint8 unsigned char
#define uint16 unsigned short
#define uint32 unsigned int
#define uint64 unsigned long
#define int32 int
#define int16 short

#define SOCKET_RECV_BUFFER_MAX 5120
#define ADMA_PACKAGE_UDP_DATA_LEN 856
#define ADMA_DELTA_UDP_DATA_LEN 88

typedef float float32;
typedef double float64;

struct GnssTimeDate
{
    uint64 gnss_timestamp_ms_epoch;
    uint32 time_msec;
    uint16 time_week;
    int16 leap_seconds;
};

class AdmaUdpDataPackage : public rclcpp::Node
{
public:
    AdmaUdpDataPackage();
    ~AdmaUdpDataPackage();

private:
    void udp_socket_data();

    float angle_NED_to_ENU(float ang_deg_NED);

    uint16 construct_unsigned_2byte_value(uint8 byte1, uint8 byte2);

    uint32 construct_unsigned_4byte_value(uint8 byte1, uint8 byte2, uint8 byte3, uint8 byte4);

    int16 construct_2byte_value(uint8 byte1, uint8 byte2);

    int32 construct_4byte_value(uint8 byte1, uint8 byte2, uint8 byte3, uint8 byte4);

    uint64 calc_timestamp(uint16 week, uint32 msec);

    void decode_adma_base(uint8 data[],
                          uint16 size,
                          adma_msgs::msg::AdmaData &msg_out);

    void decode_adma_target(uint8 data[],
                            uint16 size,
                            adma_msgs::msg::AdmaDeltaData &msg_out);

    void AdmaToPkit(adma_msgs::msg::AdmaData &adma);
    void AdmaDeltaToPkit(adma_msgs::msg::AdmaDeltaData &adma);

    GnssTimeDate g_GnssTimeDate;
    rclcpp::TimerBase::SharedPtr timer_;
    geodetic_converter::GeodeticConverter geo_converter;

    rclcpp::Publisher<adma_msgs::msg::AdmaData>::SharedPtr admaDataPublisher_;
    rclcpp::Publisher<adma_msgs::msg::AdmaDeltaData>::SharedPtr admaDeltaDataPublisher_;
    rclcpp::Publisher<perception_kit_msgs::msg::Objects>::SharedPtr pub_admaData_base_, pub_admaData_target_;
};