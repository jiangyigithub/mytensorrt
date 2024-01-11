#include <radar_locations_to_pointcloud_converter_ros/radar_locations_to_pointcloud_converter_ros.hpp>

#include <string>
#include <vector>

#include <pcl_conversions/pcl_conversions.h>

using std::placeholders::_1;


namespace radar_locations_to_pointcloud_converter_ros
{

CRadarLocationsToPointcloudConverterRos::CRadarLocationsToPointcloudConverterRos() : Node("radar_locations_to_pointcloud_converter_ros")
{
  params_ = CRadarLocationsToPointcloudConverterRos::getParamsFromNodeHandle();
  radar_loc_subscr_.reserve(params_.subscriber_topic_locations.size());
  for (int subscribe_topic_ind = 0; subscribe_topic_ind < (int)params_.subscriber_topic_locations.size(); subscribe_topic_ind++)
  {
    std::string subscribe_topic = params_.subscriber_topic_locations[subscribe_topic_ind];

    // Create subscriber for radar locations
    int subscribe_queue = params_.subscriber_msg_queue_size;
    auto subscriber = create_subscription<radar_gen5_msgs::msg::LocationInterface>(
      subscribe_topic, 
      subscribe_queue,
      std::bind(&radar_locations_to_pointcloud_converter_ros::CRadarLocationsToPointcloudConverterRos::LocationReceiverCallback, this, std::placeholders::_1));
    radar_loc_subscr_.push_back(subscriber);

    // Publisher for the omputed PCL
    std::string frame_id = params_.subscriber_frame_id[subscribe_topic_ind];
    std::string sTopic = params_.publisher_topic_location_cloud_prefix + "/" + frame_id + "/location_cloud";
    auto publisher = create_publisher<sensor_msgs::msg::PointCloud2>(sTopic, params_.publisher_msg_queue_size);
    m_mapPublisherRadarLocationCloud[sTopic] = publisher;
  }
}

CRadarLocationsToPointcloudConverterRos::Parameters CRadarLocationsToPointcloudConverterRos::getParamsFromNodeHandle()
{
  std::cout << "[CRadarLocationsToPointcloudConverterRos] Declaring all params..." << std::endl;
  declare_parameter("subscriber_topics");
  declare_parameter("subscriber_frame_id");
  declare_parameter("subscriber_msg_queue_size");
  declare_parameter("publisher_topic_prefix");
  declare_parameter("publisher_msg_queue_size");

  std::cout << "[CRadarLocationsToPointcloudConverterRos] Reading params" << std::endl;
  Parameters params;
  params.subscriber_topic_locations = get_parameter("subscriber_topics").as_string_array();
  params.subscriber_frame_id = get_parameter("subscriber_frame_id").as_string_array();
  params.subscriber_msg_queue_size = get_parameter("subscriber_msg_queue_size").as_int();
  params.publisher_topic_location_cloud_prefix = get_parameter("publisher_topic_prefix").as_string();
  params.publisher_msg_queue_size = get_parameter("publisher_msg_queue_size").as_int();
  
  return params;
}

void CRadarLocationsToPointcloudConverterRos::LocationReceiverCallback(const radar_gen5_msgs::msg::LocationInterface::SharedPtr locations_msg)
{
  // locations as PCL point cloud message
  // std::shared_ptr<pcl::PointCloud<pcl::PointXYZI> > pPointCloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI> >();
  // pPointCloud->header.frame_id = locations_msg->header.frame_id;
  // pPointCloud->height = 1;
  
  pcl::PointCloud<pcl::PointXYZI>::Ptr pPointCloud(new pcl::PointCloud<pcl::PointXYZI>);
  pPointCloud->header.frame_id = locations_msg->header.frame_id;
  pPointCloud->height = 1;
  pcl_conversions::toPCL(locations_msg->header.stamp, pPointCloud->header.stamp);

  // fill locations
  for(size_t locIdx = 0; locIdx < locations_msg->raw_locations.size(); locIdx++)
  {
    radar_gen5_msgs::msg::Location loc = locations_msg->raw_locations[locIdx];

    // get current location as PCL point
    pcl::PointXYZI point;
    const float distance = loc.radial_distance;
    const float azimuth  = loc.azimuth_angle;

    // check whether we have a valid elevation angle
    const float elevation = loc.elevation_angle;
    const bool bElevationValid = !(std::isnan(elevation) || std::isinf(elevation)) && (std::abs(elevation) < 10.0e20);

    // set angle and z-value, use elevation angle if it exists
    point.x = distance * std::cos( azimuth ) * (bElevationValid ? cos( elevation ) : 1.f);
    point.y = distance * std::sin( azimuth ) * (bElevationValid ? cos( elevation ) : 1.f);
    point.z = bElevationValid ? distance * std::sin( elevation ) : 0.f;

    // set intensity as relative radial velocity
    point.intensity = loc.radial_velocity;

    // store current point in the PCL cloud
    pPointCloud->push_back(point);
  }

  std::string sTopic = params_.publisher_topic_location_cloud_prefix + "/" + locations_msg->header.frame_id + "/location_cloud";
  if (m_mapPublisherRadarLocationCloud.find(sTopic) == m_mapPublisherRadarLocationCloud.end())
  {
    RCLCPP_ERROR_STREAM(get_logger(), "Radar location to point cloud converter: no advertized message exists for this publisher: " + sTopic);
    return;
  }

  // auto PublisherRadarPointCloud = m_mapPublisherRadarLocationCloud[sTopic];
  // PublisherRadarPointCloud->publish( *pPointCloud );

  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(*pPointCloud, cloud_msg);
  m_mapPublisherRadarLocationCloud[sTopic]->publish(cloud_msg);
}


} // namespace
