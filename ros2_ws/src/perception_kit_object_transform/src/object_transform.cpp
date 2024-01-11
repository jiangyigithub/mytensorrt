#include "perception_kit_object_transform/object_transform.h"

#include "rclcpp/rclcpp.hpp"
//#include <tf/transform_listener.h>
#include "perception_kit_msgs/msg/objects.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <stdint.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>

#define DIAGONAL(x) x* perception_kit_msgs::msg::Object::NUM_COVARIANCE_ELEMENTS + x

namespace tf2 {
/** \brief Apply a geometry_msgs TransformStamped to an geometry_msgs Pose type.
 * This function is a specialization of the doTransform template defined in tf2/convert.h.
 * \param t_in The pose to transform, as a timestamped Pose3 message with covariance.
 * \param t_out The transformed pose, as a timestamped Pose3 message with covariance.
 * \param transform The timestamped transform to apply, as a TransformStamped message.
 *
 * FIXME: copied from foxy version of tf2_geometry_msgs package
 */
inline
void doTransform(const geometry_msgs::msg::PoseWithCovarianceStamped& t_in, geometry_msgs::msg::PoseWithCovarianceStamped& t_out, const geometry_msgs::msg::TransformStamped& transform)
  {
    KDL::Vector v(t_in.pose.pose.position.x, t_in.pose.pose.position.y, t_in.pose.pose.position.z);
    KDL::Rotation r = KDL::Rotation::Quaternion(t_in.pose.pose.orientation.x, t_in.pose.pose.orientation.y, t_in.pose.pose.orientation.z, t_in.pose.pose.orientation.w);

    KDL::Frame v_out = gmTransformToKDL(transform) * KDL::Frame(r, v);
    t_out.pose.pose.position.x = v_out.p[0];
    t_out.pose.pose.position.y = v_out.p[1];
    t_out.pose.pose.position.z = v_out.p[2];
    v_out.M.GetQuaternion(t_out.pose.pose.orientation.x, t_out.pose.pose.orientation.y, t_out.pose.pose.orientation.z, t_out.pose.pose.orientation.w);
    t_out.header.stamp = transform.header.stamp;
    t_out.header.frame_id = transform.header.frame_id;
    t_out.pose.covariance = t_in.pose.covariance;
  }
}

namespace perception_kit
{
namespace internal
{
template <typename InputType, typename OutputType>
void rotateInternal(InputType const& input_x, InputType const& input_y, InputType const& input_z,
                    tf2::Quaternion const& rotation, OutputType& output_x, OutputType& output_y, OutputType& output_z,
                    bool is_covariance)
{
  tf2::Transform const t(rotation);
  tf2::Vector3 const in(static_cast<tf2Scalar>(input_x), static_cast<tf2Scalar>(input_y), static_cast<tf2Scalar>(input_z));
  auto const out = t * in;
  if (is_covariance)
  {
    output_x = std::abs(static_cast<OutputType>(out.getX()));
    output_y = std::abs(static_cast<OutputType>(out.getY()));
    output_z = std::abs(static_cast<OutputType>(out.getZ()));
  }
  else
  {
    output_x = static_cast<OutputType>(out.getX());
    output_y = static_cast<OutputType>(out.getY());
    output_z = static_cast<OutputType>(out.getZ());
  }
}

template <typename InputType, typename OutputType>
void rotate(InputType const& input_x, InputType const& input_y, InputType const& input_z,
            tf2::Quaternion const& rotation, OutputType& output_x, OutputType& output_y, OutputType& output_z)
{
  rotateInternal(input_x, input_y, input_z, rotation, output_x, output_y, output_z, false);
}

template <typename InputType, typename OutputType>
void rotateCovariance(InputType const& input_x, InputType const& input_y, InputType const& input_z,
                      tf2::Quaternion const& rotation, OutputType& output_x, OutputType& output_y, OutputType& output_z)
{
  rotateInternal(input_x, input_y, input_z, rotation, output_x, output_y, output_z, true);
}

void rotateAccelerationCovariance(perception_kit_msgs::msg::Object const& input_object, tf2::Quaternion const& rotation,
                                  perception_kit_msgs::msg::Object& transformed_object)
{
  rotateCovariance(input_object.covariance[DIAGONAL(perception_kit_msgs::msg::Object::ELEMENT_ACCELERATION_X)],
                   input_object.covariance[DIAGONAL(perception_kit_msgs::msg::Object::ELEMENT_ACCELERATION_Y)],
                   input_object.covariance[DIAGONAL(perception_kit_msgs::msg::Object::ELEMENT_ACCELERATION_Z)], rotation,
                   transformed_object.covariance[DIAGONAL(perception_kit_msgs::msg::Object::ELEMENT_ACCELERATION_X)],
                   transformed_object.covariance[DIAGONAL(perception_kit_msgs::msg::Object::ELEMENT_ACCELERATION_Y)],
                   transformed_object.covariance[DIAGONAL(perception_kit_msgs::msg::Object::ELEMENT_ACCELERATION_Z)]);
}

void rotateAcceleration(perception_kit_msgs::msg::Object const& input_object, tf2::Quaternion const& rotation,
                        perception_kit_msgs::msg::Object& transformed_object)
{
  rotate(input_object.acceleration.x, input_object.acceleration.y, input_object.acceleration.z, rotation,
         transformed_object.acceleration.x, transformed_object.acceleration.y, transformed_object.acceleration.z);
}

void rotateAccelerationWithCovariance(perception_kit_msgs::msg::Object const& input_object, tf2::Quaternion const& rotation,
                                      perception_kit_msgs::msg::Object& transformed_object)
{
  rotateAcceleration(input_object, rotation, transformed_object);
  rotateAccelerationCovariance(input_object, rotation, transformed_object);
}

void rotateVelocity(perception_kit_msgs::msg::Object const& input_object, tf2::Quaternion const& rotation,
                    perception_kit_msgs::msg::Object& transformed_object)
{
  rotate(input_object.velocity.x, input_object.velocity.y, input_object.velocity.z, rotation,
         transformed_object.velocity.x, transformed_object.velocity.y, transformed_object.velocity.z);
}

void rotateVelocityCovariance(perception_kit_msgs::msg::Object const& input_object, tf2::Quaternion const& rotation,
                              perception_kit_msgs::msg::Object& transformed_object)
{
  rotateCovariance(input_object.covariance[DIAGONAL(perception_kit_msgs::msg::Object::ELEMENT_VELOCITY_X)],
                   input_object.covariance[DIAGONAL(perception_kit_msgs::msg::Object::ELEMENT_VELOCITY_Y)],
                   input_object.covariance[DIAGONAL(perception_kit_msgs::msg::Object::ELEMENT_VELOCITY_Z)], rotation,
                   transformed_object.covariance[DIAGONAL(perception_kit_msgs::msg::Object::ELEMENT_VELOCITY_X)],
                   transformed_object.covariance[DIAGONAL(perception_kit_msgs::msg::Object::ELEMENT_VELOCITY_Y)],
                   transformed_object.covariance[DIAGONAL(perception_kit_msgs::msg::Object::ELEMENT_VELOCITY_Z)]);
}

void rotateVelocityWithCovariance(perception_kit_msgs::msg::Object const& input_object, tf2::Quaternion const& rotation,
                                  perception_kit_msgs::msg::Object& transformed_object)
{
  rotateVelocity(input_object, rotation, transformed_object);
  rotateVelocityCovariance(input_object, rotation, transformed_object);
}

void compensateVelocityWithTargetFrameVelocity(perception_kit_msgs::msg::Object const& input_object,
                                               geometry_msgs::msg::Vector3 const& velocity,
                                               perception_kit_msgs::msg::Object& transformed_object)
{
  transformed_object.velocity.x -= velocity.x;
  transformed_object.velocity.y -= velocity.y;
  transformed_object.velocity.z -= velocity.z;
}

void compensateAccelerationWithTargetFrameAcceleration(perception_kit_msgs::msg::Object const& input_object,
                                                       geometry_msgs::msg::Vector3 const& acceleration,
                                                       perception_kit_msgs::msg::Object& transformed_object)
{
  transformed_object.acceleration.x -= acceleration.x;
  transformed_object.acceleration.y -= acceleration.y;
  transformed_object.acceleration.z -= acceleration.z;
}

geometry_msgs::msg::PoseWithCovarianceStamped toPoseWithCovarianceStamped(perception_kit_msgs::msg::Object const& input_object)
{
  geometry_msgs::msg::PoseWithCovarianceStamped in;
  in.header = input_object.header;
  in.pose.pose.position.x = input_object.position.x;
  in.pose.pose.position.y = input_object.position.y;
  in.pose.pose.position.z = input_object.position.z;

  tf2::Quaternion source_rotation;
  source_rotation.setRPY(0, 0, input_object.yaw);
  in.pose.pose.orientation = tf2::toMsg(source_rotation);

  // the upper left 3x3 is the same as the upper left 3x3
  auto const perkit_object_covariance_num_cols = 11;
  auto const geometry_msgs_covariance_num_cols = 6;
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
    {
      auto const index_geometry = i + (j * geometry_msgs_covariance_num_cols);
      auto const index_perkit = i + (j * perkit_object_covariance_num_cols);

      in.pose.covariance[index_geometry] = input_object.covariance[index_perkit];
    }

  auto const yaw_index_geometry = 5;  // x,y,z,roll,pitch,yaw
  in.pose.covariance[yaw_index_geometry + (yaw_index_geometry * geometry_msgs_covariance_num_cols)] =
      input_object.covariance[DIAGONAL(perception_kit_msgs::msg::Object::ELEMENT_YAW)];

  return in;
}

void toPerkitObject(geometry_msgs::msg::PoseWithCovarianceStamped const& in, perception_kit_msgs::msg::Object& out)
{
  out.position.x = in.pose.pose.position.x;
  out.position.y = in.pose.pose.position.y;
  out.position.z = in.pose.pose.position.z;

  tf2::Quaternion in_rotation;
  tf2::fromMsg(in.pose.pose.orientation, in_rotation);
  out.yaw = tf2::getYaw(in_rotation);

  // the upper left 3x3 is the same as the upper left 3x3
  auto const perkit_object_covariance_num_cols = 11;
  auto const geometry_msgs_covariance_num_cols = 6;
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
    {
      auto const index_geometry = i + (j * geometry_msgs_covariance_num_cols);
      auto const index_perkit = i + (j * perkit_object_covariance_num_cols);

      out.covariance[index_perkit] = in.pose.covariance[index_geometry];
    }

  auto const yaw_index_geometry = 5;  // x,y,z,roll,pitch,yaw
  out.covariance[DIAGONAL(perception_kit_msgs::msg::Object::ELEMENT_YAW)] =
      in.pose.covariance[yaw_index_geometry + (yaw_index_geometry * geometry_msgs_covariance_num_cols)];
}

void copyTransformInvariantValues(perception_kit_msgs::msg::Object const& input_object,
                                  perception_kit_msgs::msg::Object& output_object)
{
  output_object.covariance[DIAGONAL(perception_kit_msgs::msg::Object::ELEMENT_YAW)] =
      input_object.covariance[DIAGONAL(perception_kit_msgs::msg::Object::ELEMENT_YAW)];
  output_object.covariance[DIAGONAL(perception_kit_msgs::msg::Object::ELEMENT_YAW_RATE)] =
      input_object.covariance[DIAGONAL(perception_kit_msgs::msg::Object::ELEMENT_YAW_RATE)];

  output_object.attributes = input_object.attributes;
  output_object.classification = input_object.classification;
  output_object.existence_probability = input_object.existence_probability;
  output_object.height = input_object.height;
  output_object.height_variance = input_object.height_variance;
  output_object.id = input_object.id;
  output_object.length = input_object.length;
  output_object.length_variance = input_object.length_variance;
  output_object.width = input_object.width;
  output_object.width_variance = input_object.width_variance;
  output_object.x_offset = input_object.x_offset;
  output_object.yaw_rate = input_object.yaw_rate;
}
}

namespace object_transform
{
perception_kit_msgs::msg::Object transformObject(perception_kit_msgs::msg::Object const& input_object,
                                            geometry_msgs::msg::TransformStamped const& transform,
                                            geometry_msgs::msg::Vector3 target_frame_velocity,
                                            geometry_msgs::msg::Vector3 target_frame_acceleration)
{
  perception_kit_msgs::msg::Object ret;
  ret.header = input_object.header;
  ret.header.frame_id = transform.header.frame_id;

  geometry_msgs::msg::PoseWithCovarianceStamped in = internal::toPoseWithCovarianceStamped(input_object);

  geometry_msgs::msg::PoseWithCovarianceStamped out;
  //tf2::doTransform(in, out, transform);
  tf2::doTransform(in, out, transform);	// FIXME: work-around for function not yet available in eloquent

  internal::toPerkitObject(out, ret);

  tf2::Quaternion rotation;
  tf2::fromMsg(transform.transform.rotation, rotation);
  internal::rotateVelocityWithCovariance(input_object, rotation, ret);
  internal::compensateVelocityWithTargetFrameVelocity(ret, target_frame_velocity, ret);

  internal::rotateAccelerationWithCovariance(input_object, rotation, ret);
  internal::compensateAccelerationWithTargetFrameAcceleration(ret, target_frame_acceleration, ret);

  internal::copyTransformInvariantValues(input_object, ret);

  return ret;
}
}
}
