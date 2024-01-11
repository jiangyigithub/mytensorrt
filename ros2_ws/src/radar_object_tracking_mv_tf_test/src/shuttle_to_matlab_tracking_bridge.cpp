#include "shuttle_to_matlab_tracking_bridge.hpp"

#include <math.h>
#include <limits>

using namespace message_filters;
using namespace std;

signed long int const obj_id_max = 2147483647; // 0x7FFFFFFF

namespace matlab_tracking_bridge
{
  namespace internal
  {
    template <class InputType>
    void normalizeAngle(InputType &input)
    {
      while (input > M_PI)
        input -= (2 * M_PI);
      while (input <= -M_PI)
        input += (2 * M_PI);
    }

#if TF_SWITCH_ON
    template <class InputOutputType, class TransformType>
    void transformVector(InputOutputType const &input, TransformType const &transform, InputOutputType &ret)
    {
      tf2::Vector3 const t = transform * tf2::Vector3(input.x, input.y, input.z);
      ret.x = t.x();
      ret.y = t.y();
      ret.z = t.z();
    }
#endif

    void transformObjectCovariance(perception_kit_msgs::msg::Object const &input_object, tf2::Matrix3x3 const &rotation_mtx,
                                   const size_t &x_index, const size_t &y_index, const size_t &num_cov_columns,
                                   perception_kit_msgs::msg::Object &output_object)
    {
      tf2::Matrix3x3 covariance3x3(input_object.covariance[x_index * num_cov_columns + x_index], // cov(x,x)
                                   input_object.covariance[x_index * num_cov_columns + y_index], // cov(x,y)
                                   0,                                                            // cov(px,pz)
                                   input_object.covariance[y_index * num_cov_columns + x_index], // cov(y,x)
                                   input_object.covariance[y_index * num_cov_columns + y_index], // cov(y,y)
                                   0, 0, 0, 0);                                                  // convert to tf-compatible datatype

      tf2::Matrix3x3 acceleration_cov_rotated = rotation_mtx * covariance3x3 * rotation_mtx.transpose();

      output_object.covariance[x_index * num_cov_columns + x_index] =
          acceleration_cov_rotated[0][0];                                                             // cov(x,x)  // convert back to object-msg datatype
      output_object.covariance[x_index * num_cov_columns + y_index] = acceleration_cov_rotated[0][1]; // cov(x,y)
      output_object.covariance[y_index * num_cov_columns + x_index] = acceleration_cov_rotated[1][0]; // cov(y,x)
      output_object.covariance[y_index * num_cov_columns + y_index] = acceleration_cov_rotated[1][1]; // cov(y,y)
    }
  } // namespace internal
} // namespace matlab_tracking_bridge

#if 0
void CShuttleToMatlabTrackingBridge::subscribeToEgoMotionIfUsed()
{
  auto const sEgomotionTopic = getEgoMotionTopicName("ego_motion_topic");

  if (sEgomotionTopic.empty())
  {
    RCLCPP_INFO_STREAM(get_logger(), "Not subscribing to ego motion");
  }

  m_SubscriberEgoMotion.subscribe(node_handle_, sEgomotionTopic, 10);
}


std::string CShuttleToMatlabTrackingBridge::getEgoMotionTopicName(std::string const &key) const
{
  // subscribe ego motion coming from ego state nodes
  std::string topic;
  if (!priv_node_handle_.getParam(key, topic))
  {
    ROS_INFO_STREAM("Could not determine ROS parameter value of '" << key << "'. Starting tracker without vehicle "
                                                                             "motion input");
    return "";
  }

  if (topic == "")
  {
    ROS_INFO_STREAM("ROS parameter '" << key << "' was read as \"\". Starting tracker without vehicle motion input");
    return "";
  }

  return topic;
}
#endif

void CShuttleToMatlabTrackingBridge::subscribeToLocations()
{
  declare_parameter("target_frame");
  get_parameter("target_frame", target_frame_);

  // for (int mSensorIndex = 0;; ++mSensorIndex)
  int mSensorIndex = 0;
  // {
  std::string sKey = (boost::format("radar_tracking_sensor_%d") % mSensorIndex).str();
  // std::string sKey_target_frame = (boost::format("radar_tracking_sensor_frame_%d") % mSensorIndex).str();
  // std::string sTopic{"/radar_decoder_gen5/location_interface"};
  std::string sTopic;
  declare_parameter(sKey);
  // declare_parameter(sKey_target_frame);

  // // rclcpp::node_interfaces::NodeParametersInterface::SharedPtr npi{this->get_node_parameters_interface()};
  // //! if (!priv_node_handle_.getParam(sKey, sTopic))
  bool /*GOTPARAM*/ got_param = get_parameter(sKey, sTopic);
  // get_parameter(sKey_target_frame, target_frame_);

  RCLCPP_INFO_STREAM(this->get_logger(), "got_param: " << /*GOTPARAM*/ got_param << ", sKey: " << sKey << ", sTopic: " << sTopic << "target_frame_:" << target_frame_);
  if (!got_param /*GOTPARAM*/)
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "cannot get sKEY");
    // break;
  }
  else
  {
    //! ROS_INFO_STREAM("Creating location subscriber. Param-Key: " << sKey << ", topic subscribed: " << sTopic);
    RCLCPP_INFO_STREAM(this->get_logger(), "Creating location subscriber. Param-Key: " << sKey << ", topic subscribed: " << sTopic);

    m_mapSubscriberRadarLocations[sKey] =
        std::make_unique<message_filters::Subscriber<radar_gen5_msgs::msg::LocationInterface>>(this, sTopic);

    if (!m_SubscriberEgoMotion.getTopic().empty())
    {
      m_synchronizerSingle[sKey] = std::make_unique<Synchronizer<egoMotion2LocationTimePolicySingle>>(
          egoMotion2LocationTimePolicySingle(10), *m_mapSubscriberRadarLocations[sKey], m_SubscriberEgoMotion);

      m_synchronizerSingle.at(sKey)->registerCallback(
          boost::bind(&CShuttleToMatlabTrackingBridge::onLocationsReceivedCallback, this, _1, _2));
    }
    else
    {
      m_mapSubscriberRadarLocations.at(sKey)->registerCallback(
          boost::bind(&CShuttleToMatlabTrackingBridge::onLocationsReceivedCallback, this, _1, nullptr));
    }
  }
  // }
}

CShuttleToMatlabTrackingBridge::CShuttleToMatlabTrackingBridge() : Node("ShuttleToMatlabTrackingBridge"),
                                                                   buffer(this->get_clock())
{
  //! priv_node_handle_.getParam("target_frame", target_frame_);
  // declare_parameter("map_mode");
  // declare_parameter("grid_resolution");
  // declare_parameter("ego_motion_topic");
  subscribeToLocations();
  // synchronization to the radar locations is necessary
  //! m_publisherObjects = node_handle_.advertise<perception_kit_msgs::Objects>("tracks", 10);
  //! m_publisherObjectsPointCloud = node_handle_.advertise<pcl::PointCloud<pcl::PointXYZI>>("objects_point_cloud", 10);
  m_publisherObjects = this->create_publisher<perception_kit_msgs::msg::Objects>("tracks_ros2_test", 10);
  m_publisherObjectsPointCloud = this->create_publisher<sensor_msgs::msg::PointCloud2>("objects_point_cloud", 10);

  // ID handling
  i_next_object_id = 1;
}

// pcl::IndicesPtr CShuttleToMatlabTrackingBridge::indicesOfValidObjects() const
// {
//   pcl::IndicesPtr valid_indexes = std::make_shared<std::vector<int>>();
//   for (std::size_t obj_idx = 0; obj_idx < NUM_RADAR_OBJECTS; ++obj_idx)
//   {
//     if (m_radarTrackerMatlabWrapper.getObjValid(obj_idx))
//     {
//       valid_indexes->push_back(obj_idx);
//     }
//   }
//   return valid_indexes;
// }

void CShuttleToMatlabTrackingBridge::onLocationsReceivedCallback(
    radar_gen5_msgs::msg::LocationInterface::ConstPtr receivedLocationsPtr,
    perception_kit_msgs::msg::Motion::ConstPtr egoMotionPtr)
{
  // std::string map_mode_;
  // bool GOTMAP = get_parameter("map_mode", map_mode_);
  // RCLCPP_INFO_STREAM(this->get_logger(), "GOTMAP: " << GOTMAP << ", map_mode: " << map_mode_);

  // double grid_resolution_;
  // bool GOTGRID = get_parameter("grid_resolution", grid_resolution_);
  // RCLCPP_INFO_STREAM(this->get_logger(), "GOTGRID: " << GOTGRID << ", grid_resolution_: " << grid_resolution_);

  // RCLCPP_INFO_STREAM(this->get_logger(), "000_ReceiveredCallback00");

#if TF_SWITCH_ON
  RCLCPP_INFO_STREAM(get_logger(), "ttttttttttttttttttttttttttttttttttttttttttttttttttttttttt");
  // Lookup radar position in the vehicle frame
  auto tf_it = sensor_tf_cache_.find(receivedLocationsPtr->header.frame_id);
  // Check if it's already in the cache
  if (tf_it == sensor_tf_cache_.end())
  {
    // tf::StampedTransform stampedTransform;
    geometry_msgs::msg::TransformStamped stampedTransform;
    try
    {
      stampedTransform = buffer.lookupTransform(tracking_frame_, receivedLocationsPtr->header.frame_id, receivedLocationsPtr->header.stamp, rclcpp::Duration(0.001)); //tf2::TimePointZero);
      tf2::Stamped<tf2::Transform> stampTransform_tf2_;
      tf2::convert(stampedTransform, stampTransform_tf2_);

      sensor_tf_cache_[receivedLocationsPtr->header.frame_id] = stampTransform_tf2_;
    }
    catch (tf2::TransformException &e)
    {
      // ROS_WARN_THROTTLE(1,
      //                   "Could not determine Radar sensor position for sensor: %s, skipping this packet (error msg is "
      //                   "throttled to 1 sec)",
      //                   receivedLocationsPtr->header.frame_id.c_str());
      auto &clk = *this->get_clock();
      RCLCPP_WARN_THROTTLE(this->get_logger(), clk, 1,
                           "Could not determine Radar sensor position for sensor: %s, skipping this packet (error msg is "
                           "throttled to 1 sec)",
                           receivedLocationsPtr->header.frame_id.c_str());
      return;
    }
  }
#endif

#if TIMER_SWITCH_ON
  //todo ================================================================================= liang
  radar_object_tracking_t_in = this->now();
  RCLCPP_INFO_STREAM(this->get_logger(), "[header_in]:"
                                             << receivedLocationsPtr->header.stamp.sec
                                             << "::"
                                             << receivedLocationsPtr->header.stamp.nanosec);
  RCLCPP_INFO_STREAM(this->get_logger(), "[t_in]:"
                                             << radar_object_tracking_t_in.seconds()
                                             << "::"
                                             << radar_object_tracking_t_in.nanoseconds());

  for (int i = 0; i < receivedLocationsPtr->attributes.size(); i++)
  {
    std::string latency_name = receivedLocationsPtr->attributes.at(i).name;
    msg_timer_tmp.name = latency_name + ":" + std::to_string(receivedLocationsPtr->attributes.at(i).nanoseconds);
    msg_timer_inherit.push_back(msg_timer_tmp);
  }
  //todo =================================================================================
#endif

  // auto &clk_setDt = *this->get_clock();

  // First empty the previous location list
  m_radarTrackerMatlabWrapper.clearMeasurementData();

#if TF_SWITCH_ON //!   !!!!!!!!!
  // Transport all data to the tracker
  m_radarTrackerMatlabWrapper.setSensorTransform(sensor_tf_cache_[receivedLocationsPtr->header.frame_id]);
#endif

  m_radarTrackerMatlabWrapper.setTimeStampAndComputeDt(receivedLocationsPtr->header.stamp /*, clk_setDt*/);

  // rclcpp::Time m_t_last_measurement_ = clk_setDt.now();

  transportLocationsToMatlabTracker(receivedLocationsPtr);
  transportFOVToMatlabTracker(receivedLocationsPtr); // must be called after m_radarTrackerMatlabWrapper.setSensorTransform()

  // get current ego motion data
  if (egoMotionPtr)
  {
    auto const velocity = egoMotionPtr->twist.twist.linear.x;
    auto const acceleration = egoMotionPtr->accel.accel.linear.x;
    auto const yaw_rate = egoMotionPtr->twist.twist.angular.z;
    auto const curvature = egoMotionPtr->curvature;

    m_radarTrackerMatlabWrapper.setEgoData(velocity, acceleration, yaw_rate, curvature, 0.0);
  }

  // execute one tracking cycle
  m_radarTrackerMatlabWrapper.runTrackingCycle();

  //! filter objects by map filter @todo: move to separate node
  // //! auto const relevant_object_indicies = filterObjectsByMap(receivedLocationsPtr->header.stamp);
  // auto const relevant_object_indicies = indicesOfValidObjects();

  // publish object list for planning / fusion / ...
  publishObjects(/*relevant_object_indicies,*/ receivedLocationsPtr->header);

  // also publish as point cloud for visualization
  publishObjectsPointCloud(receivedLocationsPtr->header);
}

#if PHI_SWITCH_ON
// bool CShuttleToMatlabTrackingBridge::project_to_ignore_phi(
//   radar_gen5_msgs::msg::LocationInterface::ConstPtr receivedLocationsPtr,
//   double dMeas,
//   double dVarMeas,
//   double vMeas,
//   double vVarMeas,
//   double elevation_angle,
//   double elevation_angle_variance,
//   double pos_vel[4]
//   )
//   {
//     // tf2_ros::Buffer tf_buffer_;
//     // tf2_ros::TransformListener tf_listener_{tf_buffer_};
//     // tf2::BufferCore tf_buffer_;
//     geometry_msgs::msg::TransformStamped transform;
//     // std::cout<<" 000_phi: "<<std::endl;
//     double test{0};
//     // using TimePoint = std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds>;
//     // static const TimePoint duration = TimePoint(duration_1ms);
//     try
//     {

//       // tf_buffer_.canTransform("radar_02_01", "layered_map_enu", tf2::TimePointZero);
//       transform = buffer.lookupTransform("layered_map_enu",receivedLocationsPtr->header.frame_id,receivedLocationsPtr->header.stamp, rclcpp::Duration(0.001)); //use sensor frame as target, to get height relative to layed map  /*tf2::TimePointZero,*/

//       // geometry_msgs::msg::Vector3 relative_pos;
//       float relative_height;
//       relative_height = transform.transform.translation.z;

//       position_z = relative_height; //lnl

//       auto angle_transfer = std::sqrt(std::pow(dMeas,2) - std::pow(relative_height,2))/dMeas;
      
//       // std::cout<<" before transform: "<<dMeas<<" "<<dVarMeas<<" "<<vMeas<<" "<<vVarMeas<<std::endl;
//       // std::cout<<" angle_transfer: "<<angle_transfer<<" elevation: "<<std::cos(elevation_angle)<<" elevation_variance: "<< elevation_angle_variance<<std::endl;
//       pos_vel[0] = dMeas*angle_transfer;
//       pos_vel[1] = dVarMeas*std::pow(angle_transfer,2);
//       pos_vel[2] = vMeas/angle_transfer;
//       pos_vel[3] = vVarMeas/std::pow(angle_transfer,2);
//       // std::cout<<" after transform: "<<pos_vel[0]<<" "<<pos_vel[1]<<" "<<pos_vel[2]<<" "<<pos_vel[3]<<std::endl;

//       return true;
//     }
//     catch(const std::exception& e)
//     {
//       std::cerr << e.what() << '\n';
//     }    

//   }


bool CShuttleToMatlabTrackingBridge::proj_to_use_phi(
  radar_gen5_msgs::msg::LocationInterface::ConstPtr receivedLocationsPtr,
  double dMeas,
  double dVarMeas,
  double vMeas,
  double vVarMeas,
  double elevation_angle,
  double elevation_angle_variance,
  double pos_vel[4]
  )
  {
    // tf2_ros::Buffer tf_buffer_;
    // tf2_ros::TransformListener tf_listener_{tf_buffer_};
    // tf2::BufferCore tf_buffer_;
    geometry_msgs::msg::TransformStamped transform;
    std::cout<<" 000_phi: "<<std::endl;
    // double test{0};
    // using TimePoint = std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds>;
    // static const TimePoint duration = TimePoint(duration_1ms);
    // check whether we have a valid elevation angle
    const bool bElevationValid = !(std::isnan(elevation_angle) || std::isinf(elevation_angle)) && (std::abs(elevation_angle) < 10.0e20);

    try
    {

      // tf_buffer_.canTransform("radar_02_01", "layered_map_enu", tf2::TimePointZero);
      // transform = buffer.lookupTransform("layered_map_enu",receivedLocationsPtr->header.frame_id,receivedLocationsPtr->header.stamp, rclcpp::Duration(0.001)); //use sensor frame as target, to get height relative to layed map  /*tf2::TimePointZero,*/

      // geometry_msgs::msg::Vector3 relative_pos;
      float relative_height;
      // relative_height = transform.transform.translation.z;

      position_z = 6.5f; //lnl
      relative_height = 6.5f;

      // auto angle_transfer = std::sqrt(std::pow(dMeas,2) - std::pow(relative_height,2))/dMeas;
      
      // std::cout<<" before transform: "<<dMeas<<" "<<dVarMeas<<" "<<vMeas<<" "<<vVarMeas<<std::endl;
      // std::cout<<" angle_transfer: "<<angle_transfer<<" elevation: "<<std::cos(elevation_angle)<<" elevation_variance: "<< elevation_angle_variance<<std::endl;
      // pos_vel[0] = dMeas*angle_transfer;
      // pos_vel[1] = dVarMeas*std::pow(angle_transfer,2);
      // pos_vel[2] = vMeas/angle_transfer;
      // pos_vel[3] = vVarMeas/std::pow(angle_transfer,2);
      // std::cout<<" after transform: "<<pos_vel[0]<<" "<<pos_vel[1]<<" "<<pos_vel[2]<<" "<<pos_vel[3]<<std::endl;

      // set angle and z-value, use elevation angle if it exists
      const float point_z = bElevationValid ? dMeas * std::sin(elevation_angle) : 0.f;



      // std::cout<<"00_phi_debug: "<<point_z<<" threshold: "<<-std::abs(relative_height)<<std::endl;
      if ((point_z >= -std::abs(relative_height) - 5) && (point_z <= -std::abs(relative_height) + 5))
      {
        pos_vel[0] = dMeas * cos(elevation_angle); //(bElevationValid ? cos( elevation ) : 1.f);
        pos_vel[1] = dVarMeas * std::pow(cos(elevation_angle), 2);
        pos_vel[2] = vMeas / cos(elevation_angle);
        pos_vel[3] = vVarMeas / std::pow(cos(elevation_angle), 2);

        return true;
      }
      else
      {
        
        // std::cout<<"01_phi_debug: "<<"discard point"<<std::endl;
        return false; 
      }

    }
    catch(const std::exception& e)
    {
      std::cerr << e.what() << '\n';
    }    

  }




#endif

void CShuttleToMatlabTrackingBridge::transportLocationsToMatlabTracker(
    radar_gen5_msgs::msg::LocationInterface::ConstPtr receivedLocationsPtr)
{
  // Verify number of received locations can be handled by the Matlab tracker
  if (static_cast<int>(receivedLocationsPtr->raw_locations.size()) > NUM_RADAR_LOCATIONS)
  {
    RCLCPP_WARN(get_logger(), "Received %d radar locations, but Matlab tracker can handle only %d. Cropping to the latter.",
                (int)receivedLocationsPtr->raw_locations.size(), NUM_RADAR_LOCATIONS);
  }

  // Copy location data from Radar detection into the internal Matlab structs for further proceeding
  for (int locIdx = 0; locIdx < std::min(NUM_RADAR_LOCATIONS, (int)receivedLocationsPtr->raw_locations.size());
       ++locIdx)
  {
    /**
     * \brief extended measured status
     *        Bit field, Bit 0: measured, Bit 1: Two target estimator azimuth,
     *                   Bit 2: Two target estimator elevation, Bit3: Location standing
     */
    bool bMeasured = (receivedLocationsPtr->raw_locations[locIdx].measurement_status &
                      radar_gen5_msgs::msg::Location::MEASUREMENT_STATUS_BITMASK_MEASURED) != 0;
  //lnl
  double dr = receivedLocationsPtr->raw_locations[locIdx].radial_distance;
  double dr_var = receivedLocationsPtr->raw_locations[locIdx].radial_distance_variance;
  double vr =  receivedLocationsPtr->raw_locations[locIdx].radial_velocity;
  double vr_var = receivedLocationsPtr->raw_locations[locIdx].radial_velocity_variance;

#if PHI_SWITCH_ON

    double pos_vel[4];
    // bool loc_succeed = project_to_ignore_phi(receivedLocationsPtr,
    //                       receivedLocationsPtr->raw_locations[locIdx].radial_distance,
    //                       receivedLocationsPtr->raw_locations[locIdx].radial_distance_variance,
    //                       receivedLocationsPtr->raw_locations[locIdx].radial_velocity,
    //                       receivedLocationsPtr->raw_locations[locIdx].radial_velocity_variance,
    //                       receivedLocationsPtr->raw_locations[locIdx].elevation_angle,
    //                       receivedLocationsPtr->raw_locations[locIdx].elevation_angle_variance,
    //                       pos_vel  
    // );
    bool loc_succeed = proj_to_use_phi(receivedLocationsPtr,
                          receivedLocationsPtr->raw_locations[locIdx].radial_distance,
                          receivedLocationsPtr->raw_locations[locIdx].radial_distance_variance,
                          receivedLocationsPtr->raw_locations[locIdx].radial_velocity,
                          receivedLocationsPtr->raw_locations[locIdx].radial_velocity_variance,
                          receivedLocationsPtr->raw_locations[locIdx].elevation_angle,
                          receivedLocationsPtr->raw_locations[locIdx].elevation_angle_variance,
                          pos_vel  

    );

  if (loc_succeed)
  {
    dr = pos_vel[0];
    dr_var = pos_vel[1];
    vr = pos_vel[2];
    vr_var = pos_vel[3];
  }
  else{
    continue;
  }

#endif 
    //lnl 
    m_radarTrackerMatlabWrapper.setLocationData(locIdx, bMeasured,
                                                dr,
                                                dr_var,
                                                vr,
                                                vr_var,
                                                receivedLocationsPtr->raw_locations[locIdx].rcs,           // dbRCS,
                                                receivedLocationsPtr->raw_locations[locIdx].azimuth_angle, // alpAng
                                                receivedLocationsPtr->raw_locations[locIdx].azimuth_angle_variance);
  }
}

void CShuttleToMatlabTrackingBridge::transportFOVToMatlabTracker(
    radar_gen5_msgs::msg::LocationInterface::ConstPtr receivedLocationsPtr)
{
  // cache
  std::size_t l = receivedLocationsPtr->sensing_state.field_of_view.fov_azimuth_angles.size();

  // iterate through the azimuth angles and ranges
  for (std::size_t i = 0; i < l; ++i)
  {
    // independent index in case flipping is necessary
    std::size_t j = i;

    //lnl, temporary solution
    // double temporary = (double)(5*i)-(double)(60.0) ;
    // std::cout<<"00_list:"<<temporary<<std::endl;

    // convert degrees to radians
    double curr_fov_rad = receivedLocationsPtr->sensing_state.field_of_view.fov_azimuth_angles[i] * M_PI / 180.0;
    // double curr_fov_rad = (receivedLocationsPtr->sensing_state.field_of_view.fov_azimuth_angles[i] + temporary)* M_PI / 180.0; //lnl, fov issue in CANFD

    // if the sensor is upside down, the Field of View (FOV) must be flipped
    // this must be called after m_radarTrackerMatlabWrapper.setSensorTransform()
    if (isMountedUpsideDown())
    {
      curr_fov_rad = -curr_fov_rad;

      // reverse the index
      j = l - 1 - i;
    }
    m_radarTrackerMatlabWrapper.setFovData(
        curr_fov_rad, receivedLocationsPtr->sensing_state.field_of_view.maximum_fov_range_azimuth[j], j);
  }
}

bool CShuttleToMatlabTrackingBridge::isMountedUpsideDown() const
{
  // this must be called after m_radarTrackerMatlabWrapper.setSensorTransform()
  double sensor_mounting_angle = m_radarTrackerMatlabWrapper.getSensorMountingAng();

  return (fmod(abs(sensor_mounting_angle - M_PI), (2 * M_PI))) < (M_PI / 2);
}

signed long int CShuttleToMatlabTrackingBridge::getNextHandleID()
{
  signed long int next_handle_id = i_next_object_id;
  if (i_next_object_id < obj_id_max)
  {
    i_next_object_id = i_next_object_id + 1;
  }
  else
  {
    i_next_object_id = 1;
  }
  return next_handle_id;
}

// Copy matlab object list to output object list
void CShuttleToMatlabTrackingBridge::publishObjects(/*pcl::IndicesPtr relevant_object_indices,*/
                                                    const std_msgs::msg::Header &header)
{
  //! perception_kit_msgs::ObjectsPtr pObjectsMsg = boost::make_shared<perception_kit_msgs::Objects>();
  perception_kit_msgs::msg::Objects::SharedPtr pObjectsMsg = std::make_shared<perception_kit_msgs::msg::Objects>();
  // perception_kit_msgs::msg::Objects pObjectsMsg;

  // fill header
  //! pObjectsMsg->header.seq = header.seq;
  pObjectsMsg->header.stamp = header.stamp;
  pObjectsMsg->header.frame_id = target_frame_;

  // fill obstacles
  for (size_t obj_idx = 0; obj_idx < NUM_RADAR_OBJECTS; ++obj_idx)
  {
    // auto const relevant = std::find(std::begin(*relevant_object_indices), std::end(*relevant_object_indices),
    //                                 obj_idx) != std::end(*relevant_object_indices);
    // if (relevant)
    // {
    //! ROS_ASSERT(m_radarTrackerMatlabWrapper.getObjValid(obj_idx));
    //! lnl, using assert directly would lead to node stop working once no object exist
    // assert(m_radarTrackerMatlabWrapper.getObjValid(obj_idx));
    if (m_radarTrackerMatlabWrapper.getObjValid(obj_idx))
    {
      
      double px = 0;
      double py = 0;
      double vx = 0;
      double vy = 0;
      m_radarTrackerMatlabWrapper.getObjPos(obj_idx, px , py);
      m_radarTrackerMatlabWrapper.getObjVel(obj_idx, vx, vy);

      // std::cout << " 000_obj id: " << i_handle_index2id[obj_idx] << " 00_m_obj_list_ast output delta: " << m_radarTrackerMatlabWrapper.getObjtestdelta(obj_idx) 
      // <<" pexist: "<<m_radarTrackerMatlabWrapper.getObjPExist(obj_idx)
      // <<" Psi: "<<m_radarTrackerMatlabWrapper.getObjPsi(obj_idx)
      // <<" Psi_dt: "<<m_radarTrackerMatlabWrapper.getObjPsiDt(obj_idx)
      // <<" Hist: "<<m_radarTrackerMatlabWrapper.getObjHist(obj_idx)
      // <<" Posi, px: "<<px<<" Posi, py: "<<py
      // <<" velocity, vx: "<<vx<<" velocity, vy: "<<vy
      // <<" moving: "<<m_radarTrackerMatlabWrapper.getObjmoving(obj_idx)
      // <<" Pgr_reflex: "<<m_radarTrackerMatlabWrapper.getObjPgr_reflex(obj_idx)
      // <<" age: "<<m_radarTrackerMatlabWrapper.getObjage(obj_idx)
      // <<" RCS_filt: "<<m_radarTrackerMatlabWrapper.getObjRCS_filt(obj_idx)
      // << std::endl;  //lnl
      perception_kit_msgs::msg::Object curr_object;

      // handle_id generated from matrix entry
      // use i_handle_index2id to store the handle IDs in this class (outside matlab tracker)
      // Check of getObjHist() is necessary to see if new object at entry of a previous object
      if ((i_handle_index2id[obj_idx] == 0) || (!m_radarTrackerMatlabWrapper.getObjHist(obj_idx)))
      {
        // new object
        i_handle_index2id[obj_idx] = getNextHandleID();
      }

      // header
      curr_object.header = pObjectsMsg->header;

      // existence
      curr_object.id = i_handle_index2id[obj_idx];
      curr_object.existence_probability = m_radarTrackerMatlabWrapper.getObjPExist(obj_idx);

      // shape
      curr_object.width = m_radarTrackerMatlabWrapper.getObjWidth(obj_idx);
      curr_object.width_variance = 0.0f;
      curr_object.length = m_radarTrackerMatlabWrapper.getObjLength(obj_idx);
      curr_object.length_variance = 0.0f;
      // height is an assumed value, since the tracker is biased towards pedestrians/vehicles
      curr_object.height = 1.5f;
      // variance is correspondingly large to represent the uncertainty in the value
      curr_object.height_variance = 1e3f;

      // shape offset
      curr_object.x_offset = 0.0f;

      // classification is currently not done
      perception_kit_msgs::msg::Classification unknown;
      unknown.obj_class = "unknown";
      unknown.confidence = 1.0;
      curr_object.classification.push_back(unknown);

      // dynamics
      m_radarTrackerMatlabWrapper.getObjPos(obj_idx, curr_object.position.x, curr_object.position.y);

      if (!std::isfinite(curr_object.position.x) || !std::isfinite(curr_object.position.y))
      {
        //! ROS_ERROR_STREAM_THROTTLE(60, "Position of published radar track is not finite! x = " << curr_object.position.x << ", y = " << curr_object.position.y);
        auto &clk = *this->get_clock();
        RCLCPP_ERROR_STREAM_THROTTLE(this->get_logger(), clk, 60, "Position of published radar track is not finite! x = " << curr_object.position.x << ", y = " << curr_object.position.y);

        continue;
      }

      m_radarTrackerMatlabWrapper.getObjVel(obj_idx, curr_object.velocity.x, curr_object.velocity.y);
      m_radarTrackerMatlabWrapper.getObjAcc(obj_idx, curr_object.acceleration.x, curr_object.acceleration.y);

      curr_object.yaw = m_radarTrackerMatlabWrapper.getObjPsi(obj_idx);

      // get diagonal elements of covariance mtx
      cov3d covariance_3d = m_radarTrackerMatlabWrapper.getObj3DCovariance(obj_idx);
      for (std::size_t cov_entry_ind = 0; cov_entry_ind < 9 * 9; ++cov_entry_ind)
      {
        curr_object.covariance[cov_entry_ind] = covariance_3d[cov_entry_ind];
      }

      // rotate to ouput frame id
      perception_kit_msgs::msg::Object curr_object_output_frame;

      // RCLCPP_INFO_STREAM(get_logger(), "000_curr_object.position.x:" << curr_object.position.x << "curr_object.position.y:" << curr_object.position.y);

#if !TF_SWITCH_ON
      curr_object.position.z = -position_z + curr_object.height/2;
      curr_object_output_frame = curr_object;
      curr_object_output_frame.header.frame_id = target_frame_;
      pObjectsMsg->objects.push_back(curr_object_output_frame);
#endif

//rm tf, lnl
#if TF_SWITCH_ON
      if (rotateObjectToFrameID(curr_object, tracking_frame_, target_frame_, curr_object_output_frame))
      {
        pObjectsMsg->objects.push_back(curr_object_output_frame);
      }
#endif
    }
    else
    {
      i_handle_index2id[obj_idx] = 0;
    }
  }

#if TIMER_SWITCH_ON
  //todo ================================================================================= liang
  radar_object_tracking_t_out = this->now();
  RCLCPP_INFO_STREAM(this->get_logger(), "[t_out]:"
                                             << radar_object_tracking_t_out.seconds()
                                             << "::"
                                             << radar_object_tracking_t_out.nanoseconds());

  rclcpp::Duration radar_object_tracking_dt = radar_object_tracking_t_out - radar_object_tracking_t_in;
  RCLCPP_INFO_STREAM(this->get_logger(), "[t_delta]:"
                                             << radar_object_tracking_dt.seconds()
                                             << "::"
                                             << radar_object_tracking_dt.nanoseconds());
#endif

#if 0
  // ======================== radar_manager_ros: ========================
  t_readin.name = "[radar_manager_ros]_tcp_sent";
  // t_readin.seconds = radar_object_tracking_t_in.seconds();
  // t_readin.nanoseconds = radar_object_tracking_t_in.nanoseconds();
  pObjectsMsg->latency.push_back(t_readin);

  t_readin.name = "[radar_manager_ros]_t_in";
  // t_readin.seconds = radar_object_tracking_t_in.seconds();
  // t_readin.nanoseconds = radar_object_tracking_t_in.nanoseconds();
  pObjectsMsg->latency.push_back(t_readin);

  t_readin.name = "[radar_manager_ros]_t_out";
  // t_readin.seconds = radar_object_tracking_t_in.seconds();
  // t_readin.nanoseconds = radar_object_tracking_t_in.nanoseconds();
  pObjectsMsg->latency.push_back(t_readin);

  t_readin.name = "[radar_manager_ros]_dt";
  // t_readin.seconds = radar_object_tracking_t_in.seconds();
  // t_readin.nanoseconds = radar_object_tracking_t_in.nanoseconds();
  pObjectsMsg->latency.push_back(t_readin);

  // ======================== radar_locations_decoder_gen5_ros: ========================
  t_readin.name = "[radar_locations_decoder_gen5_ros]_t_orig";
  // t_readin.seconds = radar_object_tracking_t_in.seconds();
  // t_readin.nanoseconds = radar_object_tracking_t_in.nanoseconds();
  pObjectsMsg->latency.push_back(t_readin);

  t_readin.name = "[radar_locations_decoder_gen5_ros]_t_in";
  // t_readin.seconds = radar_object_tracking_t_in.seconds();
  // t_readin.nanoseconds = radar_object_tracking_t_in.nanoseconds();
  pObjectsMsg->latency.push_back(t_readin);

  t_readin.name = "[radar_locations_decoder_gen5_ros]_t_out";
  // t_readin.seconds = radar_object_tracking_t_in.seconds();
  // t_readin.nanoseconds = radar_object_tracking_t_in.nanoseconds();
  pObjectsMsg->latency.push_back(t_readin);

  t_readin.name = "[radar_locations_decoder_gen5_ros]_dt";
  // t_readin.seconds = radar_object_tracking_t_in.seconds();
  // t_readin.nanoseconds = radar_object_tracking_t_in.nanoseconds();
  pObjectsMsg->latency.push_back(t_readin);
#endif

#if TIMER_SWITCH_ON
  // ======================== radar_object_tracking: ========================
  tracker_in.name = "[radar_object_tracking]_t_in :" + std::to_string(radar_object_tracking_t_in.nanoseconds());
  tracker_out.name = "[radar_object_tracking]_t_out :" + std::to_string(radar_object_tracking_t_out.nanoseconds());
  tracker_dt.name = "[radar_object_tracking]_dt :" + std::to_string(radar_object_tracking_dt.nanoseconds());
  // tracker_dt.value[0] = radar_object_tracking_dt.nanoseconds() / 1000000;

  for (int i = 0; i < pObjectsMsg->objects.size(); i++)
  {
    pObjectsMsg->objects[i].attributes.insert(pObjectsMsg->objects[i].attributes.end(), msg_timer_inherit.begin(), msg_timer_inherit.end());
    pObjectsMsg->objects[i].attributes.push_back(tracker_in);
    pObjectsMsg->objects[i].attributes.push_back(tracker_out);
    pObjectsMsg->objects[i].attributes.push_back(tracker_dt);
  };
  std::vector<perception_kit_msgs::msg::Attribute>().swap(msg_timer_inherit);

  //todo =================================================================================
#endif
  // m_publisherObjects.publish(pObjectsMsg);
  m_publisherObjects->publish(*pObjectsMsg);
}


#if TF_SWITCH_ON
bool CShuttleToMatlabTrackingBridge::rotateObjectToFrameID(const perception_kit_msgs::msg::Object &object_in,
                                                           const std::string &old_frame_id,
                                                           const std::string &new_frame_id,
                                                           perception_kit_msgs::msg::Object &object_out)
{
  // Lookup tf to newFrameID
  //! ros::Time timestamp = object_in.header.stamp;
  rclcpp::Time timestamp = object_in.header.stamp;

  // if (!m_TransformListener.waitForTransform(new_frame_id, old_frame_id, timestamp, ros::Duration(0.5)))
  // {
  //   ROS_WARN_STREAM("Timeout waiting for current transform. Using the latest transform.");
  //   // latest transform is used
  //   timestamp = ros::Time(0);
  // }
  // tf::StampedTransform map_transform;
  // try
  // {
  //   m_TransformListener.lookupTransform(new_frame_id, old_frame_id, timestamp, map_transform);
  // }
  // catch (...)
  // {
  //   ROS_ERROR_STREAM_THROTTLE(1.0, "Could not determine output transform to " << new_frame_id.c_str() << ", skipping "
  //                                                                                                        "this packet");
  //   return false;
  // }

  geometry_msgs::msg::TransformStamped map_transform_geo_;

  try
  {
    map_transform_geo_ = buffer.lookupTransform(new_frame_id, old_frame_id, timestamp, rclcpp::Duration(0.5)); //tf2::TimePointZero);
  }
  catch (tf2::TransformException &e)
  {

    RCLCPP_WARN_STREAM(this->get_logger(), "Timeout waiting for current transform. Using the latest transform.");
  }

  tf2::Stamped<tf2::Transform> map_transform;
  try
  {
    tf2::convert(map_transform_geo_, map_transform);
  }
  catch (...)
  {
    // ROS_ERROR_STREAM_THROTTLE(1.0, "Could not determine output transform to " << new_frame_id.c_str() << ", skipping "
    //                                                                                                      "this packet");
    // lnl
    auto &clk = *this->get_clock();
    RCLCPP_ERROR_STREAM_THROTTLE(this->get_logger(), clk, 1.0, "Could not determine output transform to " << new_frame_id.c_str() << ", skipping this packet");
    // RCLCPP_ERROR_STREAM(this->get_logger(), "Could not determine output transform to " << new_frame_id.c_str() << ", skipping ");

    return false;
  }

  object_out = object_in; // take over all attributes like p_exist, class, etc

  //! tf::Quaternion q_rotation = map_transform.getRotation();
  //! tf::Matrix3x3 m_rotation(q_rotation);
  tf2::Quaternion q_rotation = map_transform.getRotation();
  tf2::Matrix3x3 m_rotation(q_rotation);

  // apply transforms to pos, vel, acc
  matlab_tracking_bridge::internal::transformVector(object_in.position, map_transform, object_out.position);
  object_out.position.z = 0;
  matlab_tracking_bridge::internal::transformVector(object_in.velocity, m_rotation, object_out.velocity);
  object_out.velocity.z = 0;
  matlab_tracking_bridge::internal::transformVector(object_in.acceleration, m_rotation, object_out.acceleration);
  object_out.acceleration.z = 0;

  // correct yaw angle
  //! object_out.yaw = object_in.yaw + tf::getYaw(q_rotation);
  object_out.yaw = object_in.yaw + tf2::getYaw(q_rotation);
  matlab_tracking_bridge::internal::normalizeAngle(object_out.yaw);

  // correct covariances (only x-y covariance here because z is unknown)
  matlab_tracking_bridge::internal::transformObjectCovariance(object_in, m_rotation, 0, 1, // position x and position y
                                                              9, object_out);
  matlab_tracking_bridge::internal::transformObjectCovariance(object_in, m_rotation, 3, 4, // velocity x and velocity y
                                                              9, object_out);
  matlab_tracking_bridge::internal::transformObjectCovariance(object_in, m_rotation, 6,
                                                              7, // acceleration x and acceleration y
                                                              9, object_out);

  return true;
}
#endif

// Copy matlab object list to output object list
void CShuttleToMatlabTrackingBridge::publishObjectsPointCloud(const std_msgs::msg::Header &header)
{
  // // locations as PCL point cloud message
  // boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> pPointCloud =
  //     boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();

  // The tracker uses vehicle coordinates for objects
  // pPointCloud->header.frame_id = tracking_frame_;

  // pPointCloud->height = 1;
  // pcl_conversions::toPCL(header.stamp, pPointCloud->header.stamp);

  // for (std::size_t obj_idx = 0; obj_idx < NUM_RADAR_OBJECTS; ++obj_idx)
  // {
  //   if (m_radarTrackerMatlabWrapper.getObjValid(obj_idx))
  //   {
  //     pcl::PointXYZI point;

  //     double px, py;
  //     m_radarTrackerMatlabWrapper.getObjPos(obj_idx, px, py);
  //     point.x = static_cast<float>(px);
  //     point.y = static_cast<float>(py);
  //     point.z = static_cast<float>(m_radarTrackerMatlabWrapper.getObjPExist(obj_idx));

  //     // set intensity as object index
  //     point.intensity = obj_idx;

  //     // store current point in the PCL cloud
  //     pPointCloud->push_back(point);
  //   }
  // }

  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::PointCloud<pcl::PointXYZI>::Ptr pPointCloud(new pcl::PointCloud<pcl::PointXYZI>);
  pPointCloud->header.frame_id = tracking_frame_;
  pPointCloud->height = 1;

  for (std::size_t obj_idx = 0; obj_idx < NUM_RADAR_OBJECTS; ++obj_idx)
  {
    if (m_radarTrackerMatlabWrapper.getObjValid(obj_idx))
    {
      pcl::PointXYZI point;

      double px, py;
      m_radarTrackerMatlabWrapper.getObjPos(obj_idx, px, py);
      point.x = static_cast<float>(px);
      point.y = static_cast<float>(py);
      point.z = static_cast<float>(m_radarTrackerMatlabWrapper.getObjPExist(obj_idx));

      // set intensity as object index
      point.intensity = obj_idx;

      // store current point in the PCL cloud
      pPointCloud->push_back(point);
    }
  }

  pcl::toROSMsg(*pPointCloud, cloud_msg);

  m_publisherObjectsPointCloud->publish(cloud_msg);
}

//TODO void CShuttleToMatlabTrackingBridge::spin()  // not sure !!!
// {
//   ros::spin();
//   // rclcpp::spin(std::make_shared<CShuttleToMatlabTrackingBridge>());
// }

// pcl::IndicesPtr CShuttleToMatlabTrackingBridge::filterObjectsByMap(ros::Time const& timestamp_of_objects) const
// {
//   if (map_client_.getOperationMode() == ::lidar_object_tracking::MapClientBase::OperationMode::NONE)
//   {
//     return indicesOfValidObjects();
//   }

//   pcl::PointCloud<pcl::PointXYZ> objects_positions_tracking_frame(NUM_RADAR_OBJECTS, 1);
//   for (std::size_t obj_idx = 0; obj_idx < NUM_RADAR_OBJECTS; ++obj_idx)
//   {
//     if (m_radarTrackerMatlabWrapper.getObjValid(obj_idx))
//     {
//       double x, y;
//       m_radarTrackerMatlabWrapper.getObjPos(obj_idx, x, y);
//       objects_positions_tracking_frame.at(obj_idx).x = static_cast<float>(x);
//       objects_positions_tracking_frame.at(obj_idx).y = static_cast<float>(y);
//       objects_positions_tracking_frame.at(obj_idx).z = 0.0f;
//     }
//     else
//     {
//       objects_positions_tracking_frame.at(obj_idx).x = std::numeric_limits<float>::quiet_NaN();
//       objects_positions_tracking_frame.at(obj_idx).y = std::numeric_limits<float>::quiet_NaN();
//       objects_positions_tracking_frame.at(obj_idx).z = std::numeric_limits<float>::quiet_NaN();
//     }
//   }

//   tf::StampedTransform tracking_to_map_transform;
//   try
//   {
//     // correct would be to use the timestamp of the objects here. However, this would require an extrapolation of the
//     // tf to the future.
//     (void)timestamp_of_objects;
//     m_TransformListener.lookupTransform(map_frame_, tracking_frame_, ros::Time(0), tracking_to_map_transform);
//   }
//   catch (...)
//   {
//     ROS_ERROR_STREAM_THROTTLE(1.0, "Could not get a transform from " << tracking_frame_ << " to " << map_frame_);
//     return boost::make_shared<std::vector<int>>();
//   }

//   pcl::PointCloud<pcl::PointXYZ> objects_positions_map_frame;
//   pcl_ros::transformPointCloud(objects_positions_tracking_frame, objects_positions_map_frame,
//                                tracking_to_map_transform);

//   auto layered_map_classifier = map_client_.getLayeredMapClassifier();
//   return layered_map_classifier->classifyPoints(::layered_map_classifier::LayeredMapClassifier::Lane,
//                                                 objects_positions_map_frame);
// }
