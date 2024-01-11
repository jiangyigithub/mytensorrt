//#include "rclcpp/rclcpp.hpp"
#include <visualization_msgs/msg/marker.hpp>
#include <perception_kit_msgs/msg/objects.hpp>
#include <QtGui/QColor>
#include <cctype>
#include <map>
#include <iostream>

namespace perception_kit_visu
{
namespace internal
{
perception_kit_msgs::msg::Classification
maxConfidenceClassID(const perception_kit_msgs::msg::Object::_classification_type& confidences)
{
  auto const maximum =
      std::max_element(confidences.begin(), confidences.end(),
                       [](perception_kit_msgs::msg::Classification const& a,
                          perception_kit_msgs::msg::Classification const& b) {
                         return a.confidence < b.confidence;
                       });
  if (maximum == confidences.cend())
  {
    return perception_kit_msgs::msg::Classification();
  }
  return *maximum;
}

void setColor(visualization_msgs::msg::Marker& marker, const QColor& color)
{
  marker.color.r = color.redF();
  marker.color.g = color.greenF();
  marker.color.b = color.blueF();
  marker.color.a = color.alphaF();
}

std::string normalizeClassName(const std::string& obj_class)
{
  std::string result = obj_class;
  std::transform(result.begin(), result.end(), result.begin(), [](unsigned char c) { return std::tolower(c); });
  for (int i = 0; i < (int) result.size(); ++i)
  {
    auto const character = static_cast<unsigned char>(result[i]);
    if (!isalnum(character))
    {
      result[i] = '_';
    }
  }

  std::map<std::string, std::string> const synonyms = { { "vehicle", "car" },
                                                        { "person", "pedestrian" },
                                                        { "bike", "bicycle" } };
  auto const iter = synonyms.find(result);
  return iter == synonyms.cend() ? result : iter->second;
}

double getPositiveValueForVisualization(const std::string& field, double input)
{
  // Note: Substitute value is larger than eps since otherwise, the visualized points would be too small, e.g. if input
  // refers to object width. Eps is chosen small on purpose, as inputs of 0.1 e.g. are perfectly valid and shall be
  // visualized as such
  double const eps = 0.001;
  double const substitute_value = 1;

  // Note: This check also handles NaN, which a simple std::max() does not
  if ((input > eps) && std::isfinite(input))
    return input;
  else
  {
    //ROS_WARN_STREAM_THROTTLE(1.0, "Invalid input field value " << input << " in field " << field << ". Using "
    //                                                           << substitute_value << " instead");
    // Note: RCLCPP_WARN_STREAM_THROTTLE needs this to be a rclcpp::Node, do we want it?
    std::cout << "Invalid input field value " << input << " in field " << field << ". Using "
                                              << substitute_value << " instead" << std::endl;
    return substitute_value;
  }
}
}
}
