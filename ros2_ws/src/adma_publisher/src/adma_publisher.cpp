#include "adma_publisher.h"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <cmath>
#include <iomanip>

#define SOCKET_RECV_BUFFER_MAX 5120
#define ADMA_PACKAGE_UDP_DATA_LEN 856
#define ADMA_DELTA_UDP_DATA_LEN 88

int client_sockfd = 0;
uint16 udp_port = 2000;
struct sockaddr_in recvAddr;
int addr_len = sizeof(recvAddr);
uint8 data_buffer[SOCKET_RECV_BUFFER_MAX];
uint32 u32MessageUniqueID = 0;
uint32 u32DeltaMessageUniqueID = 0;
double width_car = 1.8;
double length_car = 4.562;
double height_car = 1.5;
float32 g_altitude = 0.0;
float g_yaw = 0.0;

//double ref_point[3] = {31.33761509, 120.7855919, 4.00000000};
double ref_point[3] = {31.5966378, 120.4443893, 4.00000000};
double test_point[3] = {31.337552637294586, 120.78553795069985, 4.00000000};
double test_point_gt[3] = {-5.13415, -6.92443, 0.0206227};

AdmaUdpDataPackage::AdmaUdpDataPackage()
    : Node("adma_udp_handler")
{
  admaDataPublisher_ = this->create_publisher<adma_msgs::msg::AdmaData>("AdmaData", 10);
  admaDeltaDataPublisher_ = this->create_publisher<adma_msgs::msg::AdmaDeltaData>("AdmaDeltaData", 10);

  pub_admaData_target_ = this->create_publisher<perception_kit_msgs::msg::Objects>("adma_gt_target", 10);
  pub_admaData_base_ = this->create_publisher<perception_kit_msgs::msg::Objects>("adma_gt_base", 10);

  timer_ = this->create_wall_timer(10ms, std::bind(&AdmaUdpDataPackage::udp_socket_data, this));

  memset(&recvAddr, 0, sizeof(struct sockaddr_in));
  recvAddr.sin_family = AF_INET;
  recvAddr.sin_port = htons(udp_port);
  recvAddr.sin_addr.s_addr = INADDR_ANY;

  client_sockfd = socket(AF_INET, SOCK_DGRAM, 0);
  if (client_sockfd < 0)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "socket error");
    return;
  }

  bool bBroadcast = true;
  setsockopt(client_sockfd, SOL_SOCKET, SO_REUSEADDR, (const char *)&bBroadcast, sizeof(bool));

  if (bind(client_sockfd, (struct sockaddr *)&recvAddr, sizeof(struct sockaddr)) == -1)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Bind fail");
  }

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "AdmaUdpDataPackage Started");

  // Set reference point
  geo_converter.initialiseReference(ref_point[0], ref_point[1], ref_point[2]);
  RCLCPP_INFO_STREAM(this->get_logger(), "ref_point(lat, lon, h) = ["
                                             << std::fixed << std::setprecision(14)
                                             << ref_point[0] << ", "
                                             << ref_point[1] << ", "
                                             << ref_point[2] << "]");
}

AdmaUdpDataPackage::~AdmaUdpDataPackage()
{
  if (client_sockfd > 0)
  {
    // close(client_sockfd);
    shutdown(client_sockfd, SHUT_RDWR);
  }
}

void AdmaUdpDataPackage::udp_socket_data()
{
  if (client_sockfd < 0)
  {
    return;
  }

  int recv_len = recvfrom(client_sockfd, data_buffer, SOCKET_RECV_BUFFER_MAX, 0, (struct sockaddr *)&recvAddr, (socklen_t *)&addr_len);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "recv_len=%d", recv_len);

  if (ADMA_PACKAGE_UDP_DATA_LEN == recv_len)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "receive adma udp data");
    auto adma_message = adma_msgs::msg::AdmaData();
    // adma_message.message_id = u32MessageUniqueID++;
    decode_adma_base(data_buffer, recv_len, adma_message);
    admaDataPublisher_->publish(adma_message);

    AdmaToPkit(adma_message);
  }
  else if (ADMA_DELTA_UDP_DATA_LEN == recv_len)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "receive adma-delta udp data");
    auto adma_delta_message = adma_msgs::msg::AdmaDeltaData();
    // adma_delta_message.message_id = u32DeltaMessageUniqueID++;
    decode_adma_target(data_buffer, recv_len, adma_delta_message);
    admaDeltaDataPublisher_->publish(adma_delta_message);

    // pub_admaData_target_->publish(message);
  }
}

void AdmaUdpDataPackage::AdmaToPkit(adma_msgs::msg::AdmaData &adma)
{
  // perception_kit_msgs::msg::Objects::SharedPtr message = std::make_shared<perception_kit_msgs::msg::Objects>();
  perception_kit_msgs::msg::Objects msg_out;

  /// ============================================ 赋值 =================================================
  msg_out.header.frame_id = "layered_map_enu";
  msg_out.header.stamp.sec = floor(adma.ins_gnss_ts_msec_epoch / 1000);
  msg_out.header.stamp.nanosec = (adma.ins_gnss_ts_msec_epoch % 1000) * 1000000U;

  perception_kit_msgs::msg::Object obj_tmp;
  obj_tmp.header = msg_out.header;
  obj_tmp.id = 1;
  obj_tmp.existence_probability = 1;
  obj_tmp.width = width_car;
  obj_tmp.width_variance = 0.0;
  obj_tmp.length = length_car;
  obj_tmp.length_variance = 0.0;
  obj_tmp.height = height_car;
  obj_tmp.height_variance = 0.0;
  obj_tmp.x_offset = 0.0;

  perception_kit_msgs::msg::Classification class_tmp;
  class_tmp.obj_class = "car";
  class_tmp.confidence = 1;
  obj_tmp.classification.push_back(class_tmp);

  // test
  double lat, lon, h, e, n, u;
  lat = test_point[0];
  lon = test_point[1];
  h = test_point[2];
  geo_converter.geodetic2Enu(lat, lon, h, &e, &n, &u);
  // RCLCPP_INFO_STREAM(this->get_logger(), "test_point_gt (e, n, u) = ["
  //                                            << std::fixed << std::setprecision(14)
  //                                            << test_point_gt[0] << ", "
  //                                            << test_point_gt[1] << ", "
  //                                            << test_point_gt[2] << "]");

  // RCLCPP_INFO_STREAM(this->get_logger(), "test_point_out(e, n, u) = ["
  //                                            << std::fixed << std::setprecision(14)
  //                                            << e << ", "
  //                                            << n << ", "
  //                                            << u << "]");

  // convert
  double latitude, longitude, altitude, east, north, up;
  latitude = adma.ins_lat_abs;
  longitude = adma.ins_long_abs;
  altitude = adma.ins_height;
  g_altitude = altitude;

  geo_converter.geodetic2Enu(latitude, longitude, altitude, &east, &north, &up);
  RCLCPP_INFO_STREAM(this->get_logger(), "pose(e, n, u) = ["
                                             << std::fixed << std::setprecision(14)
                                             << east << ", "
                                             << north << ", "
                                             << up << "]");

  obj_tmp.position.x = east;
  obj_tmp.position.y = north;
  obj_tmp.position.z = up;

  // ins_vel_frame_x: [NED] / ins_vel_hor_x: [bodyframe] ?
  obj_tmp.velocity.x = adma.ins_vel_frame_y;  // v_east
  obj_tmp.velocity.y = adma.ins_vel_frame_x;  // v_north
  obj_tmp.velocity.z = -adma.ins_vel_frame_z; // v_up

  // [NED] ?
  obj_tmp.acceleration.x = adma.acc_hor_y;
  obj_tmp.acceleration.y = adma.acc_hor_x;
  obj_tmp.acceleration.z = -adma.acc_hor_z;

  // ins_yaw / gps_cog / gps_cog_in_enu ?
  float yaw_deg = adma.gps_cog_in_enu;
  obj_tmp.yaw = yaw_deg * M_PI / 180; // 角度转为弧度
  g_yaw = obj_tmp.yaw;                // 传递
  obj_tmp.yaw_rate = 0.0;
  RCLCPP_INFO_STREAM(this->get_logger(), "yaw[deg/rad] = [" << yaw_deg << "/" << obj_tmp.yaw << "]");

  std::array<float, 121> cov;
  obj_tmp.covariance = cov;

  perception_kit_msgs::msg::Attribute attr_tmp;

  rclcpp::Time t_now = this->now();
  double dt = (t_now - msg_out.header.stamp).nanoseconds() / 1.0E6F; // ms
  RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "adma_latency = " << dt);

  attr_tmp.name = "t_diff_ADMA_to_ICU_in_ms";
  attr_tmp.value.push_back(dt);

  obj_tmp.attributes.push_back(attr_tmp);

  msg_out.objects.push_back(obj_tmp);

  pub_admaData_base_->publish(msg_out);
}

void AdmaUdpDataPackage::AdmaDeltaToPkit(adma_msgs::msg::AdmaDeltaData &adma)
{
  // perception_kit_msgs::msg::Objects::SharedPtr message = std::make_shared<perception_kit_msgs::msg::Objects>();
  perception_kit_msgs::msg::Objects msg_out;

  /// ============================================ 赋值 =================================================
  msg_out.header.frame_id = "layered_map_enu";
  msg_out.header.stamp.sec = floor(adma.ins_gnss_ts_msec_epoch / 1000);
  msg_out.header.stamp.nanosec = (adma.ins_gnss_ts_msec_epoch % 1000) * 1000000U;
  RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "t_adma = ["
                                                       << msg_out.header.stamp.sec
                                                       << " : "
                                                       << msg_out.header.stamp.nanosec
                                                       << "]");

  perception_kit_msgs::msg::Object obj_tmp;
  obj_tmp.header = msg_out.header;
  obj_tmp.id = 1;
  obj_tmp.existence_probability = 1;
  obj_tmp.width = width_car;
  obj_tmp.width_variance = 0.0;
  obj_tmp.length = length_car;
  obj_tmp.length_variance = 0.0;
  obj_tmp.height = height_car;
  obj_tmp.height_variance = 0.0;
  obj_tmp.x_offset = 0.0;

  perception_kit_msgs::msg::Classification class_tmp;
  class_tmp.obj_class = "car";
  class_tmp.confidence = 1;
  obj_tmp.classification.push_back(class_tmp);

  // convert
  double latitude, longitude, altitude, east, north, up;
  latitude = adma.target_latitude;
  longitude = adma.target_longitude;
  altitude = g_altitude;

  geo_converter.geodetic2Enu(latitude, longitude, altitude, &east, &north, &up);
  RCLCPP_INFO_STREAM(this->get_logger(), "admaDelta_point(e, n, u) = ["
                                             << std::fixed << std::setprecision(14)
                                             << east << ", "
                                             << north << ", "
                                             << up << "]");

  obj_tmp.position.x = east;
  obj_tmp.position.y = north;
  obj_tmp.position.z = up;

  // ins_vel_frame_x: [NED] / ins_vel_hor_x: [bodyframe] ?
  obj_tmp.velocity.x = 0.0; // v_east
  obj_tmp.velocity.y = 0.0; // v_north
  obj_tmp.velocity.z = 0.0; // v_up

  // [NED] ?
  obj_tmp.acceleration.x = 0.0;
  obj_tmp.acceleration.y = 0.0;
  obj_tmp.acceleration.z = 0.0;

  // ins_yaw / gps_cog / gps_cog_in_enu ?
  obj_tmp.yaw = g_yaw;
  obj_tmp.yaw_rate = 0.0;

  std::array<float, 121> cov;
  obj_tmp.covariance = cov;

  perception_kit_msgs::msg::Attribute attr_tmp;
  rclcpp::Time t_now = this->now();
  RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "t_now = ["
                                                       << t_now.nanoseconds() / static_cast<uint32_t>(1E9)
                                                       << " : "
                                                       << t_now.nanoseconds() % static_cast<uint32_t>(1E9)
                                                       << "]");

  double dt = (t_now - msg_out.header.stamp).nanoseconds() / 1.0E6F; // ms
  RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "dt[ms] = " << dt);

  attr_tmp.name = "t_diff_ADMA_to_ICU_in_ms";
  attr_tmp.value.push_back(dt);

  obj_tmp.attributes.push_back(attr_tmp);
  msg_out.objects.push_back(obj_tmp);

  pub_admaData_target_->publish(msg_out);
}

float AdmaUdpDataPackage::angle_NED_to_ENU(float ang_deg_NED)
{
  auto a_enu = 90 - ang_deg_NED;
  return a_enu < 0 ? a_enu + 360 : a_enu;
}

uint16 AdmaUdpDataPackage::construct_unsigned_2byte_value(uint8 byte1, uint8 byte2)
{
  uint16 value = 0;
  value = byte1;
  value |= (byte2 << 8);

  return value;
}

uint32 AdmaUdpDataPackage::construct_unsigned_4byte_value(uint8 byte1, uint8 byte2, uint8 byte3, uint8 byte4)
{
  uint32 value = 0;
  value = byte1;
  value |= (byte2 << 8);
  value |= (byte3 << 16);
  value |= (byte4 << 24);

  return value;
}

int16 AdmaUdpDataPackage::construct_2byte_value(uint8 byte1, uint8 byte2)
{
  int16 value = 0;
  value = byte1;
  value |= (byte2 << 8);

  return value;
}

int32 AdmaUdpDataPackage::construct_4byte_value(uint8 byte1, uint8 byte2, uint8 byte3, uint8 byte4)
{
  int32 value = 0;
  value = byte1;
  value |= (byte2 << 8);
  value |= (byte3 << 16);
  value |= (byte4 << 24);

  return value;
}

uint64 AdmaUdpDataPackage::calc_timestamp(uint16 week, uint32 msec)
{
  uint64 base = 315964800000;
  uint64 week_ratio = 604800000;
  uint64 week_msec = week_ratio * week;
  return (uint64)(week_msec + base + msec);
}

void AdmaUdpDataPackage::decode_adma_base(uint8 data[],
                                          uint16 size,
                                          adma_msgs::msg::AdmaData &msg_out)
{
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "decode for base car ...");
  uint16 Status_GPS_Mode = (data[96] & 0x0F);

  //Note: get the POI 1 data as we set the POI1 for vehicle Rear axle center
  int32 GNSS_Lat_Abs = construct_4byte_value(data[440], data[441], data[442], data[443]);
  int32 GNSS_Long_Abs = construct_4byte_value(data[444], data[445], data[446], data[447]);

  uint32 GPS_Time_msec = construct_4byte_value(data[480], data[481], data[482], data[483]);
  uint16 GPS_Time_Week = construct_unsigned_2byte_value(data[484], data[485]);
  uint64 GPS_Timestamp = calc_timestamp(GPS_Time_Week, GPS_Time_msec);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "GPS_Mode: %d", Status_GPS_Mode);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "GPS_Timestamp: %ld, Week: %d, MSec: %d", GPS_Timestamp, GPS_Time_Week, GPS_Time_msec);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "GNSS_Lat_Abs: %d, GNSS_Long_Abs: %d", GNSS_Lat_Abs, GNSS_Long_Abs);

  uint32 INS_Time_msec = construct_4byte_value(data[584], data[585], data[586], data[587]);
  uint16 INS_Time_Week = construct_unsigned_2byte_value(data[588], data[589]);
  int16 Leap_Seconds = construct_unsigned_2byte_value(data[590], data[591]);
  uint64 INS_Timestamp = calc_timestamp(INS_Time_Week, INS_Time_msec);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "INS_Timestamp: %ld, Week: %d, MSec: %d, leapSec: %d", INS_Timestamp, INS_Time_Week, INS_Time_msec, Leap_Seconds);

  int16 INS_Roll = construct_2byte_value(data[504], data[505]);
  int16 INS_Pitch = construct_2byte_value(data[506], data[507]);
  uint16 INS_Yaw = construct_unsigned_2byte_value(data[508], data[509]);
  uint16 GPS_COG = construct_unsigned_2byte_value(data[510], data[511]);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "INS_Yaw: %d, Pitch: %d, Yaw: %d, GPS_COG: %d", INS_Roll, INS_Pitch, INS_Yaw, GPS_COG);

  msg_out.status_gps_mode = Status_GPS_Mode;
  msg_out.ins_gnss_ts_msec_epoch = INS_Timestamp;
  msg_out.ins_time_msec = INS_Time_msec;
  msg_out.ins_time_week = INS_Time_Week;
  msg_out.leap_seconds = Leap_Seconds;

  //POI1
  msg_out.ins_lat_abs = construct_4byte_value(data[608], data[609], data[610], data[611]) * 0.0000001;
  msg_out.ins_long_abs = construct_4byte_value(data[612], data[613], data[614], data[615]) * 0.0000001;
  msg_out.ins_height = construct_4byte_value(data[552], data[553], data[554], data[555]) * 0.01;

  msg_out.ins_roll = INS_Roll * 0.01;
  msg_out.ins_pitch = INS_Pitch * 0.01;
  msg_out.ins_yaw = INS_Yaw * 0.01;
  msg_out.gps_cog = GPS_COG * 0.01;
  msg_out.gps_cog_in_enu = angle_NED_to_ENU(msg_out.gps_cog);

  msg_out.ins_vel_frame_x = construct_2byte_value(data[744], data[745]) * 0.005;
  msg_out.ins_vel_frame_y = construct_2byte_value(data[746], data[747]) * 0.005;
  msg_out.ins_vel_frame_z = construct_2byte_value(data[748], data[749]) * 0.005;

  //POI1
  msg_out.ins_vel_hor_x = construct_2byte_value(data[752], data[753]) * 0.005;
  msg_out.ins_vel_hor_y = construct_2byte_value(data[754], data[755]) * 0.005;
  msg_out.ins_vel_hor_z = construct_2byte_value(data[756], data[757]) * 0.005;

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "INS_Vel_Hor: X=%d, Y=%d, Z=%d", msg_out.ins_vel_hor_x, msg_out.ins_vel_hor_y, msg_out.ins_vel_hor_z);

  //POI1
  msg_out.acc_hor_x = construct_2byte_value(data[232], data[233]) * 0.0004;
  msg_out.acc_hor_y = construct_2byte_value(data[234], data[235]) * 0.0004;
  msg_out.acc_hor_z = construct_2byte_value(data[236], data[237]) * 0.0004;

  //update the current GNSS time date
  g_GnssTimeDate.gnss_timestamp_ms_epoch = INS_Timestamp;
  g_GnssTimeDate.time_week = INS_Time_Week;
  g_GnssTimeDate.time_msec = INS_Time_msec;
  g_GnssTimeDate.leap_seconds = Leap_Seconds;
}

void AdmaUdpDataPackage::decode_adma_target(uint8 data[],
                                            uint16 size,
                                            adma_msgs::msg::AdmaDeltaData &msg_out)
{
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "decode for target car ...");

  msg_out.target_gps_mode = construct_unsigned_2byte_value(data[84], data[85]);
  msg_out.hunter_gps_mode = construct_unsigned_2byte_value(data[86], data[87]);

  float64 target_longitude_integer = construct_4byte_value(data[8], data[9], data[10], data[11]) * 0.001;
  float64 target_latitude_integer = construct_4byte_value(data[12], data[13], data[14], data[15]) * 0.001;

  float32 target_long_abs_decimal;
  float32 target_lat_abs_decimal;

  memcpy(&target_long_abs_decimal, &data[12], 4);
  memcpy(&target_lat_abs_decimal, &data[20], 4);

  msg_out.target_longitude_decimal_places = target_long_abs_decimal * 0.001;
  msg_out.target_latitude_decimal_places = target_lat_abs_decimal * 0.001;

  msg_out.target_longitude = target_longitude_integer + msg_out.target_longitude_decimal_places;
  msg_out.target_latitude = target_latitude_integer + msg_out.target_latitude_decimal_places;

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Gnss_Mode: %d, Long=%f, Long_D=%f, Lat=%f, Lat_D=%f", msg_out.target_gps_mode,
              msg_out.target_longitude,
              msg_out.target_longitude_decimal_places,
              msg_out.target_latitude,
              msg_out.target_latitude_decimal_places);

  float32 long_delta_distance;
  float32 long_delta_velocity;
  memcpy(&long_delta_distance, &data[28], 4);
  memcpy(&long_delta_velocity, &data[32], 4);
  msg_out.long_delta_distance = long_delta_distance;
  msg_out.long_delta_velocity = long_delta_velocity;

  float32 lat_delta_distance;
  float32 lat_delta_velocity;
  memcpy(&lat_delta_distance, &data[36], 4);
  memcpy(&lat_delta_velocity, &data[40], 4);
  msg_out.lat_delta_distance = lat_delta_distance;
  msg_out.lat_delta_velocity = lat_delta_velocity;

  float32 resultant_distance;
  float32 resultant_velocity;
  memcpy(&resultant_distance, &data[44], 4);
  memcpy(&resultant_velocity, &data[48], 4);
  msg_out.resultant_distance = resultant_distance;
  msg_out.resultant_velocity = resultant_velocity;

  float32 angle_of_orientation;
  memcpy(&angle_of_orientation, &data[52], 4);
  msg_out.angle_of_orientation = angle_of_orientation;

  msg_out.delta_time = construct_unsigned_4byte_value(data[60], data[61], data[62], data[63]);

  msg_out.target_forward_velocity = construct_2byte_value(data[64], data[65]) * 0.005;
  msg_out.hunter_forward_velocity = construct_2byte_value(data[66], data[67]) * 0.005;

  msg_out.target_forward_acceleration = construct_2byte_value(data[68], data[69]) * 0.005;
  msg_out.hunter_forward_acceleration = construct_2byte_value(data[70], data[71]) * 0.005;

  msg_out.target_lateral_velocity = construct_2byte_value(data[72], data[73]) * 0.005;
  msg_out.hunter_lateral_velocity = construct_2byte_value(data[74], data[75]) * 0.005;

  msg_out.target_lateral_acceleration = construct_2byte_value(data[76], data[77]) * 0.005;
  msg_out.hunter_lateral_acceleration = construct_2byte_value(data[78], data[79]) * 0.005;

  msg_out.target_pitch_angle = construct_2byte_value(data[80], data[81]) * 0.02;
  msg_out.hunter_pitch_angle = construct_2byte_value(data[82], data[83]) * 0.02;

  msg_out.ins_gnss_ts_msec_epoch = calc_timestamp(g_GnssTimeDate.time_week, msg_out.delta_time);
  msg_out.ins_time_msec = g_GnssTimeDate.time_msec;
  msg_out.ins_time_week = g_GnssTimeDate.time_week;
  msg_out.leap_seconds = g_GnssTimeDate.leap_seconds;
}
