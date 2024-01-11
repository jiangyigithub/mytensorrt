#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <algorithm>

#include "perception_kit_object_transform/object_transform_component.h"
#include "perception_kit_object_transform/object_transform.h"

#include <geometry_msgs/msg/vector3.hpp>

using std::placeholders::_1;

namespace perception_kit
{
  namespace internal
  {
    void throwOnWrongFrameId(/*rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr nli,*/ perception_kit_msgs::msg::Object const &object, std::string const &expected_frame)
    {
      if (object.header.frame_id != expected_frame)
      {
        //RCLCPP_FATAL_STREAM(nli->get_logger(), "Object " << object.id << " is in frame " << object.header.frame_id
        //                                                  << "which is not the configured input frame.");
        //ROS_FATAL_STREAM("Object " << object.id << " is in frame " << object.header.frame_id << " which is not the "
        //                                                                                        "configured input frame.");
        throw std::invalid_argument("invalid object frame id");
      }
    }

    //not sure if this is necessary, get_parameter(name) will throw an error on its own
    //also get_parameter(name, param) will return false if the parameter does not exist.
    void throwIfParamNotExists(rclcpp::node_interfaces::NodeParametersInterface::SharedPtr npi,
                               rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr nli, std::string const &name)
    {
      //In ROS2 parameters always have to be declared first (this does not mean they are set)
      npi->declare_parameter(name);

      if (!npi->has_parameter(name))
      {
        RCLCPP_FATAL_STREAM(nli->get_logger(), "Could not read parameter " << name);
        throw std::invalid_argument("could not read parameter frame");
      }
    }

    void readMandatoryParameter(rclcpp::node_interfaces::NodeParametersInterface::SharedPtr npi,
                                rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr nli, std::string const &name, bool &param)
    {
      throwIfParamNotExists(npi, nli, name);

      // read input frame parameter or throw exception
      rclcpp::Parameter parameter = npi->get_parameter(name);
      param = parameter.as_bool();
    }
    void readMandatoryParameter(rclcpp::node_interfaces::NodeParametersInterface::SharedPtr npi,
                                rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr nli, std::string const &name, std::string &param)
    {
      throwIfParamNotExists(npi, nli, name);

      rclcpp::Parameter parameter = npi->get_parameter(name);
      param = parameter.as_string();
      if (param.empty())
      {
        RCLCPP_FATAL_STREAM(nli->get_logger(), name << "is empty");
        throw std::invalid_argument("read empty parameter");
      }
    }
  } // namespace internal

  namespace object_transform
  {
    perception_kit_msgs::msg::Motion::SharedPtr ObjectTransform::lookupMotion(rclcpp::Time const &target_time) const //const rclcpp::Time&
    {
      // the target_frame_motion_history_ has decreasing timestamps ts(front) > ts(last)
      // hence, the lower bound element is the last one having a time stamp smaller than the requested target
      auto const lower_bound =
          std::lower_bound(target_frame_motion_history_.begin(), target_frame_motion_history_.end(), target_time,
                           [](perception_kit_msgs::msg::Motion::SharedPtr const &motion, rclcpp::Time const &target) {
                             return target < motion->header.stamp;
                           }); // true if is should advance

      if (lower_bound == std::end(target_frame_motion_history_))
      {
        if (target_frame_motion_history_.size())
        {
          //RCLCPP_DEBUG_STREAM(this->get_logger(), ("List is : " << target_frame_motion_history_.front()->header.stamp << " "
          //                                                     << target_frame_motion_history_.back()->header.stamp << " big."));
          //ROS_DEBUG_STREAM("List is : " << target_frame_motion_history_.front()->header.stamp << " "
          //                              << target_frame_motion_history_.back()->header.stamp << " big.");
          //RCLCPP_DEBUG_STREAM(this->get_logger(), "Requested stamp: " << target_time);
          //ROS_DEBUG_STREAM("Requested stamp: " << target_time);
        }

        throw std::runtime_error("could not lookup motion");
      }
      //RCLCPP_DEBUG_STREAM(this->get_logger(), "Found motion with time diff of " << (target_time - lower_bound->get()->header.stamp).toSec());
      //ROS_DEBUG_STREAM("Found motion with time diff of " << (target_time - lower_bound->get()->header.stamp).toSec());

      if (target_time - lower_bound->get()->header.stamp > rclcpp::Duration(0.2))
      {
        //RCLCPP_ERROR_STREAM(this->get_logger(), "Found motion with time diff of " << (target_time - lower_bound->get()->header.stamp).toSec()
        //                                                                          << ". This is too much.");
        //ROS_ERROR_STREAM("Found motion with time diff of " << (target_time - lower_bound->get()->header.stamp).toSec()
        //                                                   << ". This is too much.");
        throw std::runtime_error("Motion time diff too far off");
      }
      return target_frame_motion_history_.front();
    }

    void ObjectTransform::transformObjectsToTargetFrame(const perception_kit_msgs::msg::Objects::SharedPtr msg) const
    {
      perception_kit_msgs::msg::Objects out;
      out.header.frame_id = target_frame_;
      out.header.stamp = msg->header.stamp;

      RCLCPP_DEBUG(this->get_logger(), "Received object list");
      for (auto const &object : msg->objects)
      {
        internal::throwOnWrongFrameId(/*this->get_node_logging_interface,*/ object, input_frame_);

        try
        {
          auto const transform =
              tf_buffer_.lookupTransform(target_frame_, input_frame_, object.header.stamp, rclcpp::Duration(0.001));

          geometry_msgs::msg::Vector3 target_frame_velocity;
          geometry_msgs::msg::Vector3 target_frame_accel;

          /* Added for Yaw transform */
          // Transform alpha with sensor orientation
          tf2::Stamped<tf2::Transform> transform_tf2_;
          tf2::convert(transform, transform_tf2_);

          double alpAng(object.yaw);
          tf2::Quaternion qLocationSensorCoordinates;
          qLocationSensorCoordinates.setRPY(0, 0, alpAng);

          // First apply the location angle, then transform this one into new coordinates.
          // The order of application is right-to-left, see http://wiki.ros.org/tf2/Tutorials/Quaternions#Applying_a_quaternion_rotation
          tf2::Quaternion qLocationNewCoordinates = transform_tf2_.getRotation() * qLocationSensorCoordinates;

          tf2::Matrix3x3 mRotation(qLocationNewCoordinates);
          double roll, pitch, yaw;
          mRotation.getRPY(roll, pitch, yaw);

          /***********************/

          if (needs_motion_)
          {
            auto const motion = lookupMotion(object.header.stamp);

            target_frame_velocity.x = motion->twist.twist.linear.x;
            target_frame_velocity.y = motion->twist.twist.linear.y;
            target_frame_velocity.z = motion->twist.twist.linear.z;

            target_frame_accel.x = motion->accel.accel.linear.x;
            target_frame_accel.y = motion->accel.accel.linear.y;
            target_frame_accel.z = motion->accel.accel.linear.z;
          }

          auto transformed_object = perception_kit::object_transform::transformObject(
              object, transform, target_frame_velocity, target_frame_accel);

          transformed_object.width = object.width;
          transformed_object.width_variance = object.width_variance;
          transformed_object.length = object.length;
          transformed_object.length_variance = object.length_variance;
          transformed_object.height = object.height;
          transformed_object.height_variance = object.height_variance;

          transformed_object.x_offset = object.x_offset;

          transformed_object.classification = object.classification;

          transformed_object.attributes = object.attributes;

          transformed_object.existence_probability = object.existence_probability;

          transformed_object.id = object.id;

      	  /* Added for Yaw transform */
          if (isnan(yaw))
          {
            if(std::abs(transformed_object.velocity.y)>1.f){
              transformed_object.yaw = atan2(transformed_object.velocity.y, transformed_object.velocity.x);
              yaw_buffer = transformed_object.yaw;
            }
            else{
              transformed_object.yaw = yaw_buffer;
            }

            transformed_object.yaw = (transformed_object.yaw < 0) ? transformed_object.yaw + (M_PI * 2) : transformed_object.yaw;
          }
          else
          {
            transformed_object.yaw = (yaw < 0) ? yaw + (M_PI * 2) : yaw;
          }
//---compensate to camera position--
#if 1
          // if(input_frame_.find("camera")!=std::string::npos)
          // {
          //   auto sign = (object.velocity.z*transformed_object.position.x == 0)? sign : object.velocity.z*transformed_object.position.y/std::abs(object.velocity.z*transformed_object.position.y);

          //   transformed_object.position.x = +transformed_object.length*std::cos(transformed_object.yaw);
          //   transformed_object.position.y = +transformed_object.length*std::sin(transformed_object.yaw);
          // }

#endif
          //-------------

          out.objects.push_back(transformed_object);
        }
        catch (...)
        {
          RCLCPP_DEBUG_STREAM(this->get_logger(), "Transformation failed: (" << target_frame_ << "," << input_frame_ << ").");
          //ROS_DEBUG_STREAM("Transformation failed: (" << target_frame_ << "," << input_frame_ << ").");
        }
      }

      transformed_object_publisher_->publish(out);
    }

    void ObjectTransform::queueMotion(const perception_kit_msgs::msg::Motion::SharedPtr motion)
    {
      RCLCPP_DEBUG_STREAM(this->get_logger(), "Received target frame motion "); //<< motion->header.seq);
      //ROS_DEBUG_STREAM("Received target frame motion " << motion->header.seq);
      target_frame_motion_history_.emplace(std::begin(target_frame_motion_history_), motion);

      // motion->header.stamp - tf_buffer_.getCacheLength() can get negative when running in simulation, as there, the
      // ros:Time starts at zero. Take 0 as minimum then
      //auto const oldest_timestamp = motion->header.stamp.toSec() > tf_buffer_.getCacheLength().toSec() ?
      auto const oldest_timestamp = rclcpp::Time(motion->header.stamp) > rclcpp::Time(0) + tf_buffer_.getCacheLength() ? rclcpp::Time(motion->header.stamp) - tf_buffer_.getCacheLength() : rclcpp::Time(0);

      auto const last_element_to_keep = std::find_if(
          target_frame_motion_history_.rbegin(), target_frame_motion_history_.rend(),
          [&oldest_timestamp](perception_kit_msgs::msg::Motion::SharedPtr const &t) { return rclcpp::Time(t->header.stamp) > oldest_timestamp; });

      target_frame_motion_history_.erase(last_element_to_keep.base(), std::end(target_frame_motion_history_));

      //RCLCPP_DEBUG_STREAM_THROTTLE(this->get_logger(), this->get_clock(), 1, "NumElements in Motion List " << target_frame_motion_history_.size());
      //ROS_DEBUG_STREAM_THROTTLE(1, "NumElements in Motion List " << target_frame_motion_history_.size());
      if (target_frame_motion_history_.size())
      {
        //RCLCPP_DEBUG_STREAM_THROTTLE(this->get_logger(), this->get_clock(), 1, "List is : " << (target_frame_motion_history_.front()->header.stamp -
        //                                                                    target_frame_motion_history_.back()->header.stamp)
        //                                                                      .toSec()
        //                                                                 << " seconds long.");
        //ROS_DEBUG_STREAM_THROTTLE(1, "List is : " << (target_frame_motion_history_.front()->header.stamp -
        //                                              target_frame_motion_history_.back()->header.stamp)
        //                                                 .toSec()
        //                                          << " seconds long.");
      }
    }

    ObjectTransform::ObjectTransform(const rclcpp::NodeOptions &options)
        : Node("perception_kit_object_transform", options), tf_buffer_(this->get_clock())
    //void ObjectTransform::onInit(const rclcpp::NodeOptions & options)
    {
      std::cout << "Starting component object transform" << std::endl;
      internal::readMandatoryParameter({this->get_node_parameters_interface()}, this->get_node_logging_interface(), "input/frame", input_frame_);
      internal::readMandatoryParameter(this->get_node_parameters_interface(), this->get_node_logging_interface(), "output/frame", target_frame_);

      internal::readMandatoryParameter(this->get_node_parameters_interface(), this->get_node_logging_interface(), "output/target_frame_needs_motion", needs_motion_);
      if (needs_motion_)
      {
        std::string topic_name_motion;
        internal::readMandatoryParameter(this->get_node_parameters_interface(), this->get_node_logging_interface(), "motion/topic", topic_name_motion);

        subscriber_motion_ = this->create_subscription<perception_kit_msgs::msg::Motion>(topic_name_motion, rclcpp::SystemDefaultsQoS(),
                                                                                         bind(&ObjectTransform::queueMotion, this, _1));
      }

      // subscribe to object list
      std::string object_list_input_topic;
      internal::readMandatoryParameter(this->get_node_parameters_interface(), this->get_node_logging_interface(), "input/topic", object_list_input_topic);
      objects_subscriber_ = this->create_subscription<perception_kit_msgs::msg::Objects>(object_list_input_topic, rclcpp::SystemDefaultsQoS(),
                                                                                         bind(&ObjectTransform::transformObjectsToTargetFrame, this, _1));

      // create output publisher
      std::string object_list_output_topic;
      internal::readMandatoryParameter(this->get_node_parameters_interface(), this->get_node_logging_interface(), "output/topic", object_list_output_topic);

      transformed_object_publisher_ = this->create_publisher<perception_kit_msgs::msg::Objects>(object_list_output_topic, 10);
    }
  } // namespace object_transform
} // namespace perception_kit

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(perception_kit::object_transform::ObjectTransform)