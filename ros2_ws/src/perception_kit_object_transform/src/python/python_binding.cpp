#include <rclcpp/serialization.hpp>

#include <perception_kit_msgs/msg/motion.hpp>
#include <perception_kit_msgs/msg/object.hpp>

#include <perception_kit_object_transform/object_transform.h>

#include <boost/python.hpp>

namespace perception_kit
{
namespace object_transform_python_binding
{
namespace bp = boost::python;

/* Read a ROS message from a serialized string.
  */
template <typename M>
M from_python(const std::string& str_msg)
{
  size_t serial_size = str_msg.size();
  // ROS1:
  // boost::shared_array<uint8_t> buffer(new uint8_t[serial_size]);
  // for (size_t i = 0; i < serial_size; ++i)
  // {
  //   buffer[i] = str_msg[i];
  // }
  // ros::serialization::IStream stream(buffer.get(), serial_size);
  // M msg;
  // ros::serialization::Serializer<M>::read(stream, msg);

  rclcpp::SerializedMessage serialized_msg;
  for (size_t i = 0; i < serial_size; ++i) {
    //No setter available?
    serialized_msg->set_rcl_serialized_message().buffer[i] = str_msg.pop();
  }
  M msg;
  auto serializer = rclcpp::Serialization<M>();
  serializer.deserialize_message(serialized_msg.get(), &msg);

  return msg;
}

/* Write a ROS message into a serialized string.
*/
template <typename M>
std::string to_python(const M& msg)
{
  // ROS1:
  // size_t serial_size = ros::serialization::serializationLength(msg);
  // boost::shared_array<uint8_t> buffer(new uint8_t[serial_size]);
  // ros::serialization::OStream stream(buffer.get(), serial_size);
  // ros::serialization::serialize(stream, msg);
  // str_msg.reserve(serial_size);
  // for (size_t i = 0; i < serial_size; ++i)
  // {
  //   str_msg.push_back(buffer[i]);
  // }

  auto message_header_length = 8u;
  auto message_payload_length = static_cast<size_t>(msg->data.size());
  rclcpp::SerializedMessage serialized_msg;
  serialized_msg.reserve(message_header_length + message_payload_length);
  static rclcpp::Serialization<M> serializer;
  serializer.serialize_message(string_msg.get(), &serialized_msg);
  std::string str_msg;
  for (size_t i = 0; i < serialized_msg.size(); ++i) {
    str_msg.push_back(serialized_msg.get_rcl_serialized_message().buffer[i]));
  }

  return str_msg;
}

std::string transformObjectPythonWrapper(std::string const& input_object, std::string const& transform,
                                         std::string const& target_frame_velocity,
                                         std::string const& target_frame_acceleration)
{
  auto const i = from_python<perception_kit_msgs::msg::Object>(input_object);
  auto const t = from_python<geometry_msgs::msg::TransformStamped>(transform);
  auto const v = from_python<geometry_msgs::msg::Vector3>(target_frame_velocity);
  auto const a = from_python<geometry_msgs::msg::Vector3>(target_frame_acceleration);

  auto const o = perception_kit::object_transform::transformObject(i, t, v, a);
  return to_python<perception_kit_msgs::msg::Object>(o);
}
}
}

BOOST_PYTHON_MODULE(libperception_kit_object_transform_python)
{
  using namespace boost::python;
  def("transformObjectWrapper", &perception_kit::object_transform_python_binding::transformObjectPythonWrapper);
}
