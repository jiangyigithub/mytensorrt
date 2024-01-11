#include "perception_kit_visu/dynamic_objects_display.h"

#include "perception_kit_visu/utilities.h"

#define TINYCOLORMAP_WITH_QT5
#include "perception_kit_visu/tinycolormap.h"

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <tf2/transform_datatypes.h>
#include <sstream>
#include <iomanip>
#include <Eigen/Dense>
#include <QApplication>
#include <QColor>

using namespace std;
using namespace perception_kit_msgs::msg;

namespace perception_kit_visu
{
namespace markers
{
/** @brief Shows object as a 3D box, color shows classification type */
visualization_msgs::msg::Marker createBoxMarker(const perception_kit_msgs::msg::Object& object, const QColor& color)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = object.header.frame_id;
  marker.header.stamp = object.header.stamp;
  marker.type = visualization_msgs::msg::Marker::CUBE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.position = object.position;

  // Set orientation
  double yaw_angle{0.0};
  if (std::isnan(object.yaw)) {
    std::cout << "Warning: Can't visualize data object with NaN in yaw angle. ";
    if (!iszero(object.velocity.x)) {
      yaw_angle = atan2(object.velocity.y, object.velocity.x);
      std::cout << "Replaced by angle of velocity vector in xy plane." << std::endl;
    }
    else {
      yaw_angle = 0.0;
      std::cout << "Replaced by '0.0'" << std::endl;
    }
  } else
    yaw_angle = object.yaw;

  tf2::Quaternion q;
  q.setRPY(0, 0, yaw_angle);  // Create this quaternion from roll/pitch/yaw (in radians)
  marker.pose.orientation.x = q.getX();
  marker.pose.orientation.y = q.getY();
  marker.pose.orientation.z = q.getZ();
  marker.pose.orientation.w = q.getW();

  marker.scale.x = internal::getPositiveValueForVisualization("object.length", object.length);
  marker.scale.y = internal::getPositiveValueForVisualization("object.width", object.width);
  marker.scale.z = internal::getPositiveValueForVisualization("object.height", object.height);

  internal::setColor(marker, color);
  return marker;
}

/** @brief Shows object classification (color), identifier and velocity of the object using a text marker above the
 * object */
visualization_msgs::msg::Marker createTextMarker(const perception_kit_msgs::msg::Object& object, const QColor& color,
                                                 const std::string& text, const rviz_common::properties::FloatProperty* marker_size)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = object.header.frame_id;
  marker.header.stamp = object.header.stamp;
  marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  marker.action = visualization_msgs::msg::Marker::ADD;

  marker.pose.position = object.position;
  marker.pose.position.z += 0.5 * internal::getPositiveValueForVisualization("object.height", object.height);
  marker.pose.position.z += 1.0;  // float above the object

  marker.scale.z = marker_size->getFloat();  //lnl
  marker.text = text;
  internal::setColor(marker, color);
  return marker;
}

/** @brief Shows covariances of the x,y position as sphere from the center of the object */
visualization_msgs::msg::Marker createPositionCovarianceMarker(const perception_kit_msgs::msg::Object& object,
                                                               const QColor& color)
{
  visualization_msgs::msg::Marker marker;

  // calulate eigenvectors and eigenvalue to set the position and orientatation
  Eigen::Matrix2d C;
  static const size_t px_covariance_index = Object::ELEMENT_POSITION_X;
  static const size_t py_covariance_index = Object::ELEMENT_POSITION_Y;
  static const size_t convariance_num_columns = Object::NUM_COVARIANCE_ELEMENTS;
  C << object.covariance[px_covariance_index * convariance_num_columns + px_covariance_index],
      object.covariance[px_covariance_index * convariance_num_columns + py_covariance_index],
      object.covariance[py_covariance_index * convariance_num_columns + px_covariance_index],
      object.covariance[py_covariance_index * convariance_num_columns + py_covariance_index];

  // if covariance on diagonal lines are equal 0 --> this is probably input from DL LH5, where covariances are not
  // given! Do not create markers!
  if (C(0) == 0.0 || C(3) == 0.0)
  {
    return marker;
  }

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eigensolver(C);
  auto eigenvalues = eigensolver.eigenvalues();
  auto eigenvectors = eigensolver.eigenvectors();
  Eigen::MatrixXd::Index max_index;
  Eigen::MatrixXd::Index min_index;
  eigenvalues.col(0).maxCoeff(&max_index);
  eigenvalues.col(0).minCoeff(&min_index);
  Eigen::Vector2d eigenVecMax;
  eigenVecMax << eigenvectors.coeff(max_index * 2), eigenvectors.coeff(max_index * 2 + 1);


  // Set orientation
  tf2::Quaternion q;
  auto yaw = std::atan2(eigenVecMax[1], eigenVecMax[0]);
  q.setRPY(0, 0, yaw);  // Create this quaternion from eigenvector of the max eigenvalue
  marker.pose.orientation.x = q.getX();
  marker.pose.orientation.y = q.getY();
  marker.pose.orientation.z = q.getZ();
  marker.pose.orientation.w = q.getW();
  
  marker.header.frame_id = object.header.frame_id;
  marker.header.stamp = object.header.stamp;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.position = object.position;

  marker.scale.x =
      internal::getPositiveValueForVisualization("object.covariance.eigenvalue", 2 * sqrt(eigenvalues[max_index]));
  marker.scale.y =
      internal::getPositiveValueForVisualization("object.covariance.eigenvalue", 2 * sqrt(eigenvalues[min_index]));
  marker.scale.z = 0.1;  // height

  internal::setColor(marker, color);

  return marker;
}

/** @brief Shows velocity of the object using arrow from the center of the object */
visualization_msgs::msg::Marker createVelocityMarker(const perception_kit_msgs::msg::Object& object, const QColor& color)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = object.header.frame_id;
  marker.header.stamp = object.header.stamp;
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.points.push_back(object.position);
  geometry_msgs::msg::Point vel;
  vel.x = object.position.x + object.velocity.x / 2;  // for better visualization take 1/2nd
  vel.y = object.position.y + object.velocity.y / 2;
  vel.z = object.position.z + object.velocity.z / 2;
  marker.points.push_back(vel);
  marker.scale.x = 0.2;
  marker.scale.y = 0.4;
  internal::setColor(marker, color);
  return marker;
}

/** @brief Shows yaw angle using arrow from the center of the object */
visualization_msgs::msg::Marker createYawAngleMarker(const perception_kit_msgs::msg::Object& object, const QColor& color)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = object.header.frame_id;
  marker.header.stamp = object.header.stamp;
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.position = object.position;
  
  // Set orientation
  tf2::Quaternion q;
  q.setRPY(0, 0, object.yaw);  // Create this quaternion from roll/pitch/yaw (in radians)
  marker.pose.orientation.x = q.getX();
  marker.pose.orientation.y = q.getY();
  marker.pose.orientation.z = q.getZ();
  marker.pose.orientation.w = q.getW();
  
  marker.scale.x = 2.0;  // arrow length
  marker.scale.y = 0.2;  // arrow width
  // marker.scale.y = object.yaw_variance + 0.01;  // arrow width showing yaw variance
  marker.scale.z = 0.2;  // arrow height
  internal::setColor(marker, color);

  return marker;
}

/** @brief Draws a point in the center of the box */
visualization_msgs::msg::Marker createCenterPositionMarker(const perception_kit_msgs::msg::Object& object, const QColor& color)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = object.header.frame_id;
  marker.header.stamp = object.header.stamp;
  marker.type = visualization_msgs::msg::Marker::POINTS;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.4;
  marker.scale.y = 0.4;

  geometry_msgs::msg::Point p = object.position;
  p.z = object.position.z -
        0.5 * internal::getPositiveValueForVisualization("object.height", object.height);  // attach to object bottom
  marker.points.push_back(p);

  internal::setColor(marker, color);
  return marker;
}

/** @brief Draws a track of historic object positions */
visualization_msgs::msg::Marker
createTrackMarker(const perception_kit_msgs::msg::Object& object, const QColor& color,
                  std::unordered_map<perception_kit_msgs::msg::Object::_id_type, std::vector<geometry_msgs::msg::Point>>& tracks)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = object.header.frame_id;
  marker.header.stamp = object.header.stamp;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.points = tracks.find(object.id)->second;
  marker.scale.x = 0.04;  // line width
  internal::setColor(marker, color);
  return marker;
}
} // namespace markers


std::string DynamicObjectsDisplay::createText(const perception_kit_msgs::msg::Object& object) const
{
  vector<string> lines;

  if (show_text_id_->getBool())
  {
    std::stringstream stream;
    stream << "Object " << object.id;
    stream << " (" << std::fixed << std::setprecision(1) << double(100.0 * object.existence_probability) << "%)";
    lines.push_back(stream.str());
  }

  if (show_text_classification_->getBool())
  {
    std::stringstream stream;
    auto const maximum = internal::maxConfidenceClassID(object.classification);
    stream << maximum.obj_class << " (" << round(100.0 * maximum.confidence) << "%)";
    lines.push_back(stream.str());
  }

  if (show_text_velocity_->getBool())
  {
    std::stringstream stream;
    auto const speed_kmh = 3.6 * hypot(object.velocity.x, object.velocity.y);
    stream << std::fixed << std::setprecision(1) << speed_kmh << " km/h";
    lines.push_back(stream.str());
  }

  for (auto const& active_attribute : show_attribute_text_)
  {
    if (active_attribute.second->getBool())
    {
      for (auto const& attribute : object.attributes)
      {
        if (attribute.name == active_attribute.first)
        {
          std::stringstream stream;
          stream << attribute.name << ": ";
          for (size_t i = 0, n = attribute.value.size(); i < n; ++i)
          {
            stream << std::fixed << std::setprecision(2) << attribute.value[i];
            if (i + 1 < n)
            {
              stream << ", ";
            }
          }
          lines.push_back(stream.str());
        }
      }
    }
  }

  std::stringstream stream;
  if (!lines.empty())
  {
    stream << lines[0];
  }
  for (size_t i = 1, n = lines.size(); i < n; ++i)
  {
    stream << '\n' << lines[i];
  }
  return stream.str();
}

QColor DynamicObjectsDisplay::assignColor(const perception_kit_msgs::msg::Object& object,
                                          PropertyConfig::Ptr const& property_config)
{
  switch (ColorMode(property_config->color_mode->getOptionInt()))
  {
    case FlatColor:
      return property_config->color_flat->getColor();
    case ByTrack:
      return trackColor(object);
    case ByAttribute:
      return attributeColor(object, property_config->color_attribute->getStdString(),
                            property_config->color_colormap->getOptionInt(), property_config->color_invert->getBool());
    case ByClass:
      return assignClassColor(object);
  }

  assert(false);  // should not be reached
  return box_properties_->color_flat->getColor();
}

QColor DynamicObjectsDisplay::assignClassColor(const perception_kit_msgs::msg::Object& object)
{
  auto const maximum = internal::maxConfidenceClassID(object.classification);
  auto const obj_class = maximum.obj_class;
  auto iter = cityscapes_colors_.find(internal::normalizeClassName(obj_class));
  if (iter == cityscapes_colors_.cend())
  {
    iter = cityscapes_colors_.begin();
    std::hash<std::string> hash;
    size_t const index = hash(obj_class) % cityscapes_colors_.size();
    advance(iter, index);
  }

  auto color = iter->second;
  if (class_colors_.find(obj_class) == class_colors_.cend())
  {
    auto const name = QString::fromStdString(obj_class);
    class_colors_[obj_class] = new rviz_common::properties::ColorProperty(name, color, name, color_legend_);
    class_colors_[obj_class]->setReadOnly(true);
  }
  return color;
}

QColor DynamicObjectsDisplay::trackColor(const perception_kit_msgs::msg::Object& object) const
{
  assert(!twenty_colors_.empty());
  size_t const index = object.id % twenty_colors_.size();
  auto color = twenty_colors_[index];
  return color;
}

QColor DynamicObjectsDisplay::attributeColor(const perception_kit_msgs::msg::Object& object, std::string name, int colormap,
                                             bool invert) const
{
  QColor color;

  for (const auto& attribute : object.attributes)
  {
    assert(attribute.value.size() > 0);
    if (attribute.name == name)
    {
      const float val = invert ? (1. - attribute.value[0]) : attribute.value[0];
      color = tinycolormap::GetColor(val, static_cast<tinycolormap::ColormapType>(colormap)).ConvertToQColor();
      return color;
    }
  }

  // Attribute cannot be found. Set to white color.
  color.setRgb(255, 255, 255);
  return color;
}

float DynamicObjectsDisplay::assignAlpha(const perception_kit_msgs::msg::Object& object,
                                         PropertyConfig::Ptr const& property_config) const
{
  switch (AlphaMode(property_config->alpha_mode->getOptionInt()))
  {
    case FlatAlpha:
      return property_config->alpha_flat->getFloat();
    case ByAttributeAlpha:
      return attributeAlpha(object, property_config->alpha_attribute->getStdString(),
                            property_config->alpha_invert->getBool());
  }

  assert(false);  // should not be reached
  return box_properties_->alpha_flat->getFloat();
}

float DynamicObjectsDisplay::attributeAlpha(const perception_kit_msgs::msg::Object& object, std::string name,
                                            bool invert) const
{
  float alpha = 1.f;

  for (const auto& attribute : object.attributes)
  {
    assert(attribute.value.size() > 0);
    if (attribute.name == name)
    {
      alpha = invert ? (1.f - attribute.value[0]) : attribute.value[0];
    }
  }

  return alpha;
}

DynamicObjectsDisplay::DynamicObjectsDisplay() : 
    RTDClass(),
    marker_common_(std::make_unique<MarkerCommon>(this))
{
}

void DynamicObjectsDisplay::processMessage(perception_kit_msgs::msg::Objects::ConstSharedPtr msgPtr)
{
  objects_ = msgPtr;
  recreateMarkers();
  updateAvailableAttributeNames();
}

void DynamicObjectsDisplay::load(const rviz_common::Config & config)
{
  RTDClass::load(config);

  // Todo: Not sure if this is needed at all, since MarkerCommon doesnt modify anything 
  // in the configuration. However, it gets called in MarkerArrayDisplay and doesn't harm here
  marker_common_->load(config);
}

void DynamicObjectsDisplay::update(float wall_dt, float ros_dt)
{
  RTDClass::update(wall_dt, ros_dt);
  marker_common_->update(wall_dt, ros_dt);
}

void DynamicObjectsDisplay::reset()
{
  RTDClass::reset();
  marker_common_->clearMarkers();
}

void DynamicObjectsDisplay::handlePropertyChange()
{
  for (auto& property : { text_properties_, box_properties_, track_properties_, general_properties_ })
  {
    property->color_flat->setHidden(property->color_mode->getOptionInt() != FlatColor);
    property->color_attribute->setHidden(property->color_mode->getOptionInt() != ByAttribute);
    property->color_invert->setHidden(property->color_mode->getOptionInt() != ByAttribute);
    property->color_colormap->setHidden(property->color_mode->getOptionInt() != ByAttribute);
    property->alpha_flat->setHidden(property->alpha_mode->getOptionInt() != FlatAlpha);
    property->alpha_attribute->setHidden(property->alpha_mode->getOptionInt() != ByAttributeAlpha);
    property->alpha_invert->setHidden(property->alpha_mode->getOptionInt() != ByAttributeAlpha);
  }

  marker_common_->clearMarkers();
  recreateMarkers();
}

void DynamicObjectsDisplay::recreateMarkers()
{
  if (!objects_)
  {
    return;
  }

  float existence_threshold = existence_threshold_->getFloat();

  auto markers = std::make_shared<visualization_msgs::msg::MarkerArray>();

  // delete all previous markers
  auto markersD = std::make_shared<visualization_msgs::msg::MarkerArray>();
  visualization_msgs::msg::Marker markerD{};
  markerD.header.frame_id = "map";
  markerD.action = visualization_msgs::msg::Marker::DELETEALL;
  markersD->markers.push_back(markerD);
  marker_common_->addMessage(markersD);

  // Create new markers for all current objects
  for (auto const& object : objects_->objects)
  {
    if (object.existence_probability >= existence_threshold)
    {
      // create markers
      if (box_properties_->show->getBool())
      {
        auto box_color = assignColor(object, box_properties_);
        float const box_alpha = assignAlpha(object, box_properties_);
        box_color.setAlphaF(box_alpha);
        markers->markers.push_back(markers::createBoxMarker(object, box_color));
      }

      if (text_properties_->show->getBool())
      {
        auto text_color = assignColor(object, text_properties_);
        float const text_alpha = assignAlpha(object, text_properties_);
        text_color.setAlphaF(text_alpha);
        markers->markers.push_back(markers::createTextMarker(object, text_color, createText(object),marker_text_size_));
      }

      if (general_properties_->show->getBool())
      {
        auto color = assignColor(object, general_properties_);
        float const alpha = assignAlpha(object, general_properties_);
        color.setAlphaF(alpha);
        if (show_center_->getBool())
        {
          markers->markers.push_back(markers::createCenterPositionMarker(object, color));
        }
        if (show_position_uncertainty_->getBool())
        {
          markers->markers.push_back(markers::createPositionCovarianceMarker(object, color));
        }
        if (show_velocity_->getBool())
        {
          markers->markers.push_back(markers::createVelocityMarker(object, color));
        }
        if (show_yaw_->getBool())
        {
          markers->markers.push_back(markers::createYawAngleMarker(object, color));
        }
      }

      // Bookkeeping of object track
      auto const has_track = tracks_fused_.count(object.id) > 0;
      auto position_of_the_track = object.position;
      position_of_the_track.z -=
          0.5 * internal::getPositiveValueForVisualization("object.height", object.height);  // attach to object bottom

      // Shift point from the center point to the tracked point
      auto const y_offset = 0.0f;
      double const dx = cos(object.yaw) * object.x_offset - sin(object.yaw) * y_offset;
      double const dy = sin(object.yaw) * object.x_offset + cos(object.yaw) * y_offset;
      position_of_the_track.x += dx;
      position_of_the_track.y += dy;
      tracks_fused_[object.id].push_back(position_of_the_track);
      if (has_track && track_properties_->show->getBool())
      {
        auto track_color = assignColor(object, track_properties_);
        float const track_alpha = assignAlpha(object, track_properties_);
        track_color.setAlphaF(track_alpha);
        markers->markers.push_back(markers::createTrackMarker(object, track_color, tracks_fused_));
      }
    }
  }

  size_t next_id = 0;
  for (auto& marker : markers->markers)
  {
    marker.id = ++next_id;
  }

  // incomingMarkerArray(markers);
  marker_common_->addMessage(markers);
}



void DynamicObjectsDisplay::onInitialize()
{
  RTDClass::onInitialize();
  marker_common_->initialize(context_, scene_node_);

  existence_threshold_ = new rviz_common::properties::FloatProperty(
      "Min. Existence", 0.0f, "Minimum existence filter: Set minimum probability of shown objects (0.0-1.0)", this);
  existence_threshold_->setMin(0.0);
  existence_threshold_->setMax(1.0);
  connect(existence_threshold_, SIGNAL(changed()), this, SLOT(handlePropertyChange()));

  general_properties_ = std::make_shared<PropertyConfig>("Object Properties", this, "Show additional properties",
                                                    QColor(255, 250, 200), FlatColor, 1.0, "");
  show_center_ = new rviz_common::properties::BoolProperty("Center Point", false, "Show object center", general_properties_->show);
  connect(show_center_, SIGNAL(changed()), this, SLOT(handlePropertyChange()));
  show_yaw_ = new rviz_common::properties::BoolProperty("Yaw Angle", false, "Show yaw angle arrow", general_properties_->show);
  connect(show_yaw_, SIGNAL(changed()), this, SLOT(handlePropertyChange()));
  show_velocity_ = new rviz_common::properties::BoolProperty("Velocity", true, "Show velocity arrow", general_properties_->show);
  connect(show_velocity_, SIGNAL(changed()), this, SLOT(handlePropertyChange()));
  show_position_uncertainty_ =
      new rviz_common::properties::BoolProperty("Position Uncertainty", false, "Show position uncertainty", general_properties_->show);
  connect(show_position_uncertainty_, SIGNAL(changed()), this, SLOT(handlePropertyChange()));
  general_properties_->show->expand();

  box_properties_ =
      std::make_shared<PropertyConfig>("Bounding Box", this, "Show bounding box", QColor(0, 128, 128), ByClass, 0.8, "");
  box_properties_->show->expand();

  track_properties_ = std::make_shared<PropertyConfig>("Track", this, "Show track", QColor(210, 245, 60), ByClass, 1.0, "");
  track_properties_->show->expand();

  twenty_colors_ = { QColor(230, 25, 75),      // red
                     QColor(60, 180, 75),      // green
                     QColor(255, 225, 25),     // yellow
                     QColor(0, 130, 200),      // blue
                     QColor(245, 130, 48),     // orange
                     QColor(255, 250, 200),    // beige
                     QColor(70, 240, 240),     // cyan
                     QColor(230, 190, 255),    // lavender
                     QColor(240, 50, 230),     // magenta
                     QColor(210, 245, 60),     // lime
                     QColor(170, 110, 40),     // brown
                     QColor(145, 30, 180),     // purple
                     QColor(250, 190, 190),    // pink
                     QColor(0, 128, 128),      // teal
                     QColor(128, 0, 0),        // maroon
                     QColor(128, 128, 0),      // olive
                     QColor(255, 215, 180),    // apricot
                     QColor(0, 0, 128),        // navy
                     QColor(128, 128, 128),    // grey
                     QColor(255, 255, 255),    // white
                     QColor(0, 0, 0),          // black
                     QColor(170, 255, 195) };  // mint

  cityscapes_colors_ = { { "ego_vehicle", QColor(81, 0, 81) },    { "road", QColor(128, 64, 128) },
                         { "sidewalk", QColor(244, 35, 232) },    { "parking", QColor(250, 170, 160) },
                         { "rail_track", QColor(230, 150, 140) }, { "building", QColor(70, 70, 70) },
                         { "wall", QColor(102, 102, 156) },       { "fence", QColor(190, 153, 153) },
                         { "guard_rail", QColor(180, 165, 180) }, { "bridge", QColor(150, 100, 100) },
                         { "tunnel", QColor(150, 120, 90) },      { "pole", QColor(153, 153, 153) },
                         { "polegroup", QColor(153, 153, 153) },  { "traffic_light", QColor(250, 170, 30) },
                         { "traffic_sign", QColor(220, 220, 0) }, { "vegetation", QColor(107, 142, 35) },
                         { "terrain", QColor(152, 251, 152) },    { "sky", QColor(70, 130, 180) },
                         { "person", QColor(220, 20, 60) },       { "pedestrian", QColor(220, 20, 60) },
                         { "rider", QColor(255, 0, 0) },          { "car", QColor(0, 0, 142) },
                         { "vehicle", QColor(0, 0, 142) },        { "truck", QColor(0, 0, 70) },
                         { "bus", QColor(0, 60, 100) },           { "caravan", QColor(0, 0, 90) },
                         { "trailer", QColor(0, 0, 110) },        { "train", QColor(0, 80, 100) },
                         { "motorcycle", QColor(0, 0, 230) },     { "bicycle", QColor(119, 11, 32) },
                         { "license_plate", QColor(0, 0, 142) } };

  text_properties_ = std::make_shared<PropertyConfig>("Text", this, "Show textual description above objects",
                                                 QColor(Qt::white), FlatColor, 1.0, "");
  
  //lnl
  marker_text_size_ = new rviz_common::properties::FloatProperty(
      "Size (0-10)", 0.5f, "text size marker scale z limited to (0.0-10.0), 0.5 by default", text_properties_->show);
  marker_text_size_->setMin(0.0);
  marker_text_size_->setMax(10.0);
  connect(marker_text_size_, SIGNAL(changed()), this, SLOT(handlePropertyChange()));

  show_text_id_ =
      new rviz_common::properties::BoolProperty("ID", false, "Show object ID and existence probability", text_properties_->show);
  show_text_classification_ = new rviz_common::properties::BoolProperty(
      "Classification", false, "Show object type classification and confidence", text_properties_->show);
  show_text_velocity_ =
      new rviz_common::properties::BoolProperty("Velocity", false, "Show estimated object velocity", text_properties_->show);
  text_attributes_ =
      new rviz_common::properties::Property("Attributes", QVariant(), "Show specified object attributes", text_properties_->show);
  text_properties_->show->expand();

  color_legend_ = new rviz_common::properties::Property("Color Legend", QVariant(), "Color by object class", this);

  expand();
  handlePropertyChange();
}

DynamicObjectsDisplay::PropertyConfig::PropertyConfig(const QString& name, rviz_common::properties::Property* parent,
                                                      const QString& description, const QColor& flat_color,
                                                      ColorMode mode, float alpha_value, const QString& attribute_name)
{
  show = new rviz_common::properties::BoolProperty(name, true, description, parent, SLOT(handlePropertyChange()));
  show->setDisableChildrenIfFalse(true);

  alpha_mode = new rviz_common::properties::EnumProperty("Alpha Mode", "Flat Alpha",
                                      "Alpha mode: Alpha value by an attribute or fixed value.", show);
  alpha_mode->addOption("By Attribute", ByAttributeAlpha);
  alpha_mode->addOption("Flat Alpha", FlatAlpha);
  connect(alpha_mode, SIGNAL(changed()), parent, SLOT(handlePropertyChange()));

  alpha_flat = new rviz_common::properties::FloatProperty("Alpha", alpha_value, "Opacity value", show);
  alpha_flat->setMin(0.0f);
  alpha_flat->setMax(1.0f);
  connect(alpha_flat, SIGNAL(changed()), parent, SLOT(handlePropertyChange()));

  alpha_attribute = new rviz_common::properties::EditableEnumProperty("Alpha Attribute", attribute_name,
                                                   "Select the attribute to use for alpha by attribute", show);
  connect(alpha_attribute, SIGNAL(changed()), parent, SLOT(handlePropertyChange()));

  alpha_invert = new rviz_common::properties::BoolProperty("Alpha Invert", false, "Invert alpha given by attribute.", show);
  connect(alpha_invert, SIGNAL(changed()), parent, SLOT(handlePropertyChange()));

  if (mode == ByClass)
  {
    color_mode = new rviz_common::properties::EnumProperty(
        "Color Mode", "By Object Class",
        "Coloring mode: Color by object class, by track ID, an attribute or use the same color always.", show);
  }
  else if (mode == ByTrack)
  {
    color_mode = new rviz_common::properties::EnumProperty("Color Mode", "By Track ID", "Coloring mode: Color by object class, by track "
                                                                     "ID, an attribute or use the same color always.",
                                        show);
  }
  else if (mode == ByAttribute)
  {
    color_mode = new rviz_common::properties::EnumProperty("Color Mode", "By Attribute", "Coloring mode: Color by object class, by track "
                                                                      "ID, an attribute or use the same color always.",
                                        show);
  }
  else
  {
    color_mode = new rviz_common::properties::EnumProperty("Color Mode", "Flat Color", "Coloring mode: Color by object class, by track "
                                                                    "ID, an attribute or use the same color always.",
                                        show);
  }
  color_mode->addOption("By Object Class", ByClass);
  color_mode->addOption("By Track ID", ByTrack);
  color_mode->addOption("By Attribute", ByAttribute);
  color_mode->addOption("Flat Color", FlatColor);
  connect(color_mode, SIGNAL(changed()), parent, SLOT(handlePropertyChange()));

  color_attribute = new rviz_common::properties::EditableEnumProperty("Color Attribute", attribute_name,
                                                   "Select the attribute to use for coloring by attribute", show);
  connect(color_attribute, SIGNAL(changed()), parent, SLOT(handlePropertyChange()));

  color_flat = new rviz_common::properties::ColorProperty("Flat Color", flat_color, "Color (flat color mode)", show);
  connect(color_flat, SIGNAL(changed()), parent, SLOT(handlePropertyChange()));

  color_invert = new rviz_common::properties::BoolProperty("Color Invert", false, "Invert color given by attribute.", show);
  connect(color_invert, SIGNAL(changed()), parent, SLOT(handlePropertyChange()));

  using CMType = tinycolormap::ColormapType;
  color_colormap = new rviz_common::properties::EnumProperty("Colormap", "Jet", "Coloringmap: Colormap to be used.", show);
  color_colormap->addOption("Jet", CMType::Jet);
  color_colormap->addOption("Parula", CMType::Parula);
  color_colormap->addOption("Heat", CMType::Heat);
  color_colormap->addOption("Hot", CMType::Hot);
  color_colormap->addOption("Gray", CMType::Gray);
  color_colormap->addOption("Magma", CMType::Magma);
  color_colormap->addOption("Inferno", CMType::Inferno);
  color_colormap->addOption("Plasma", CMType::Plasma);
  color_colormap->addOption("Viridis", CMType::Viridis);
  color_colormap->addOption("Cividis", CMType::Cividis);
  color_colormap->addOption("Github", CMType::Github);
  connect(color_colormap, SIGNAL(changed()), parent, SLOT(handlePropertyChange()));
}

void DynamicObjectsDisplay::updateAvailableAttributeNames()
{
  // Search for all attribute names
  std::vector<std::string> attribute_names;
  for (auto const& single_object : objects_->objects)
  {
    for (auto const& attribute : single_object.attributes)
    {
      auto i = std::lower_bound(attribute_names.begin(), attribute_names.end(), attribute.name);
      if (i == attribute_names.end() || attribute.name < *i)
      {
        attribute_names.insert(i, attribute.name);
      }
    }
  }

  if (attribute_names != available_attribute_names_)
  {
    for (auto& property : { text_properties_, box_properties_, track_properties_, general_properties_ })
    {
      property->color_attribute->clearOptions();
      property->alpha_attribute->clearOptions();
      for (const auto& name : attribute_names)
      {
        if (name.empty())
        {
          continue;
        }
        property->color_attribute->addOptionStd(name);
        property->alpha_attribute->addOptionStd(name);
      }
    }

    for (auto const& attribute : attribute_names)
    {
      if (show_attribute_text_.count(attribute) == 0)
      {
        show_attribute_text_[attribute] = new rviz_common::properties::BoolProperty(QString::fromStdString(attribute), false,
                                                                 "Show attribute values as text", text_attributes_);
      }
    }

    available_attribute_names_ = attribute_names;
  }
}


} // namespace perception_kit_visu



#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(perception_kit_visu::DynamicObjectsDisplay, rviz_common::Display) //rviz::
//PLUGINLIB_EXPORT_CLASS(perception_kit_visu::DynamicObjectsDisplay, rviz_common::Tool)