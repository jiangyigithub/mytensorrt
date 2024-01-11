#pragma once
#include <map>
#include <memory>
#include <string>
#include <eigen3/Eigen/Geometry>
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer_interface.h>
#include <tf2_ros/visibility_control.h>
#include <tf2/buffer_core.h>
#include <tf2/time.h>
#include <tf2/utils.h>
#include <geometry_msgs/msg/transform_stamped.h>

#include <boost/make_shared.hpp>
#include <boost/format.hpp>
#include <boost/bind.hpp>

#include <radar_gen5_msgs/msg/location_interface.hpp>
#include <perception_kit_msgs/msg/objects.hpp>
#include <perception_kit_msgs/msg/object.hpp>
#include <perception_kit_msgs/msg/motion.hpp>

#include "rclcpp/rclcpp.hpp"
#include "radar_tracker_matlab_wrapper.hpp"

//tf rm, lnl
// #ifndef TF_SWITCH_ON
// #define TF_SWITCH_ON 0

//timer switch, lnl
#ifndef TIMER_SWITCH_ON
#define TIMER_SWITCH_ON 0
#endif

//elevation angle bug fix
#ifndef PHI_SWITCH_ON
#define PHI_SWITCH_ON 1
#endif

class CShuttleToMatlabTrackingBridge : public rclcpp::Node
{
public:
  CShuttleToMatlabTrackingBridge();

  void spin();

private:
  //! ros::Publisher m_publisherObjects;
  //! ros::Publisher m_publisherObjectsPointCloud;
  rclcpp::Publisher<perception_kit_msgs::msg::Objects>::SharedPtr m_publisherObjects;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_publisherObjectsPointCloud;

  // rclcpp::TimerBase::SharedPtr timer_;
  // void timer_callback();

  //todo liang
  perception_kit_msgs::msg::Attribute t_readin, msg_timer_tmp, tracker_in, tracker_out, tracker_dt;
  std::vector<perception_kit_msgs::msg::Attribute> msg_timer_inherit;
  rclcpp::Time radar_object_tracking_t_in, radar_object_tracking_t_out;
  // rclcpp::Duration radar_object_tracking_t_delta;

  //! ros::NodeHandle node_handle_;
  //! ros::NodeHandle priv_node_handle_{ros::NodeHandle("~")};
  // rclcpp::node_interfaces::NodeParametersInterface::SharedPtr npi;
  // rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr nli;

  // This is the frame of the layered map, used for filtering the objects in a post-processing step
  std::string const map_frame_{"radar_02_01"};
  // This is the frame, the tracker operates in. It is static to the vehicle.
  std::string const tracking_frame_{"radar_02_01"};
  // This is the output (target) frame of the radar tracker. Can be configured via parameter 'target_frame' on startup.
  // This frame needs to be fixed to the worlds coordinate system or at least virtually static like the odom frame.
  std::string target_frame_{"radar_02_01"};
  float position_z{0}; //lnl

  CRadarTrackerMatlabWrapper m_radarTrackerMatlabWrapper;

  signed long int i_next_object_id; // need at least int32, see definition of Object.msg
  signed long int i_handle_index2id[NUM_RADAR_OBJECTS];
  bool b_handle_existed_before[NUM_RADAR_OBJECTS];
  bool b_handle_has_obj_offset[NUM_RADAR_OBJECTS];

  //! tf::TransformListener m_TransformListener;
  // tf2::BufferCore buffer{tf2::BUFFER_CORE_DEFAULT_CACHE_TIME};
  tf2_ros::Buffer buffer;
  tf2_ros::TransformListener listener{buffer};

  // Map to store one subscriber for each radar mounted in the vehicle, identified via FrameID
  //! std::map<std::string, boost::shared_ptr<message_filters::Subscriber<radar_gen5_msgs::msg::LocationInterface>>> m_mapSubscriberRadarLocations;
  std::map<std::string, std::unique_ptr<message_filters::Subscriber<radar_gen5_msgs::msg::LocationInterface>>> m_mapSubscriberRadarLocations;

  // ros::Subscriber to ego state messages;
  using egoMotion2LocationTimePolicySingle =
      message_filters::sync_policies::ApproximateTime<radar_gen5_msgs::msg::LocationInterface, perception_kit_msgs::msg::Motion>;

  //! std::map<std::string, boost::shared_ptr<message_filters::Synchronizer<egoMotion2LocationTimePolicySingle>>> m_synchronizerSingle;
  std::map<std::string, std::unique_ptr<message_filters::Synchronizer<egoMotion2LocationTimePolicySingle>>> m_synchronizerSingle;

  //! message_filters::Subscriber<perception_kit_msgs::Motion> m_SubscriberEgoMotion;
  message_filters::Subscriber<perception_kit_msgs::msg::Motion> m_SubscriberEgoMotion;
  // std::unique_ptr<message_filters::Subscriber<perception_kit_msgs::msg::Motion>> m_SubscriberEgoMotion;

  bool isMountedUpsideDown() const;
  signed long int getNextHandleID();

  void transportLocationsToMatlabTracker(radar_gen5_msgs::msg::LocationInterface::ConstPtr receivedLocationsPtr);
  void transportFOVToMatlabTracker(radar_gen5_msgs::msg::LocationInterface::ConstPtr receivedLocationsPtr);

  // Copy matlab object list to output object list
  void publishObjects(/*pcl::IndicesPtr relevant_object_indices,*/ const std_msgs::msg::Header &header);

#if PHI_SWITCH_ON
  // bool project_to_ignore_phi(
  // radar_gen5_msgs::msg::LocationInterface::ConstPtr receivedLocationsPtr,
  // double dMeas,
  // double dVarMeas,
  // double vMeas,
  // double vVarMeas,
  // double elevation_angle,
  // double elevation_angle_variance,
  // double pos_vel[4]
  // );

  //lnl, update 2021.12.06
  bool proj_to_use_phi(
  radar_gen5_msgs::msg::LocationInterface::ConstPtr receivedLocationsPtr,
  double dMeas,
  double dVarMeas,
  double vMeas,
  double vVarMeas,
  double elevation_angle,   // phi, elevation_angle_quality
  double elevation_angle_variance,  //rad^2
  double pos_vel[4]
  );

#endif

//rm tf, lnl
#if TF_SWITCH_ON
  bool rotateObjectToFrameID(const perception_kit_msgs::msg::Object &object_in,
                             const std::string &old_frame_id,
                             const std::string &new_frame_id,
                             perception_kit_msgs::msg::Object &object_out);
#endif

  void publishObjectsPointCloud(const std_msgs::msg::Header &header);

  void onLocationsReceivedCallback(radar_gen5_msgs::msg::LocationInterface::ConstPtr receivedLocationsPtr,
                                   perception_kit_msgs::msg::Motion::ConstPtr egoMotionPtr);

  // pcl::IndicesPtr indicesOfValidObjects() const;
  // //! pcl::IndicesPtr filterObjectsByMap(ros::Time const &timestamp_of_objects) const;

  // @todo: Idea: at the moment the radar tracker is untouched and the objects are filtered by the layered map
  // classifier afterwards. Another way would be to filter the locations prior to the tracking (somewhere within
  // transportLocationsToMatlabTracker). Advantages/Disadvantages have to be evaluated.
  // //! lidar_object_tracking::MapClient map_client_;

#if TF_SWITCH_ON
  // keep a cache of the static sensor to vehicle transforms to avoid repeated lookups
  std::map<std::string, tf2::Transform> sensor_tf_cache_;
#endif

  std::string getEgoMotionTopicName(std::string const &sEgomotionKey) const;

  void subscribeToEgoMotionIfUsed();

  void subscribeToLocations();

protected:
};
