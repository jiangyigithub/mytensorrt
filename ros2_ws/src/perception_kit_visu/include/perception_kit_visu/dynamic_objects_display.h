#pragma once

#ifndef Q_MOC_RUN  // see https://bugreports.qt.io/browse/QTBUG-22829
#include <rviz_common/display.hpp>
#include <rviz_default_plugins/displays/marker_array/marker_array_display.hpp>
#include <rviz_default_plugins/displays/marker/marker_display.hpp>
#include <rviz_default_plugins/displays/marker/marker_common.hpp>
#include <rviz_common/message_filter_display.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <perception_kit_msgs/msg/objects.hpp>
#include <unordered_map>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/ros_topic_property.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/enum_property.hpp>
#endif

namespace perception_kit_visu
{
using rviz_default_plugins::displays::MarkerCommon;

// From https://github.com/ros2/rviz/blob/ros2/docs/migration_guide.md:
// If the display extends the MessageFilterDisplay, switch to extending RosTopicDisplay. Message filters have not yet been ported.
class DynamicObjectsDisplay : public rviz_common::RosTopicDisplay<perception_kit_msgs::msg::Objects>
{
  Q_OBJECT
public:
  DynamicObjectsDisplay();


protected:
  virtual void onInitialize() override;


private:
  enum ColorMode
  {
    FlatColor = 0,
    ByClass = 1,
    ByTrack = 2,
    ByAttribute = 3
  };

  enum AlphaMode
  {
    FlatAlpha = 0,
    ByAttributeAlpha = 1
  };

  struct PropertyConfig
  {
    using Ptr = std::shared_ptr<PropertyConfig>;
    PropertyConfig(const QString& name, rviz_common::properties::Property* parent, const QString& description, const QColor& flat_color,
                   ColorMode color_mode, float alpha, const QString& attribute_name);

    rviz_common::properties::BoolProperty* show;
    rviz_common::properties::EnumProperty* alpha_mode;
    rviz_common::properties::FloatProperty* alpha_flat;
    rviz_common::properties::BoolProperty* alpha_invert;
    rviz_common::properties::EditableEnumProperty* alpha_attribute;
    rviz_common::properties::EnumProperty* color_mode;
    rviz_common::properties::ColorProperty* color_flat;
    rviz_common::properties::EditableEnumProperty* color_attribute;
    rviz_common::properties::EnumProperty* color_colormap;
    rviz_common::properties::BoolProperty* color_invert;
  };

  QColor assignColor(const perception_kit_msgs::msg::Object& object, PropertyConfig::Ptr const& property_config);
  QColor assignClassColor(const perception_kit_msgs::msg::Object& object);
  QColor trackColor(const perception_kit_msgs::msg::Object& object) const;
  QColor attributeColor(const perception_kit_msgs::msg::Object& object, std::string name, int colormap, bool invert) const;
  float assignAlpha(const perception_kit_msgs::msg::Object& object, PropertyConfig::Ptr const& property_config) const;
  float attributeAlpha(const perception_kit_msgs::msg::Object& object, std::string name, bool invert) const;

  std::string createText(const perception_kit_msgs::msg::Object& object) const;

  void recreateMarkers();
  virtual void processMessage(perception_kit_msgs::msg::Objects::ConstSharedPtr msgPtr) override;

  void updateAvailableAttributeNames();

  // pointer of incoming objects for simpliefied access in internal functions
  perception_kit_msgs::msg::Objects::ConstSharedPtr objects_;

  std::unordered_map<perception_kit_msgs::msg::Object::_id_type, std::vector<geometry_msgs::msg::Point> > tracks_fused_;

  rviz_common::properties::FloatProperty* existence_threshold_;
  rviz_common::properties::RosTopicProperty* marker_topic_;

  rviz_common::properties::FloatProperty* marker_text_size_; //lnl

  rviz_common::properties::Property* color_legend_;
  std::map<std::string, rviz_common::properties::ColorProperty*> class_colors_;
  std::vector<QColor> twenty_colors_;
  std::map<std::string, QColor> cityscapes_colors_;

  PropertyConfig::Ptr general_properties_;
  rviz_common::properties::BoolProperty* show_center_;
  rviz_common::properties::BoolProperty* show_yaw_;
  rviz_common::properties::BoolProperty* show_velocity_;
  rviz_common::properties::BoolProperty* show_position_uncertainty_;

  PropertyConfig::Ptr box_properties_;
  PropertyConfig::Ptr track_properties_;

  PropertyConfig::Ptr text_properties_;
  rviz_common::properties::BoolProperty* show_text_id_;
  rviz_common::properties::BoolProperty* show_text_classification_;
  rviz_common::properties::BoolProperty* show_text_velocity_;
  rviz_common::properties::Property* text_attributes_;
  std::map<std::string, rviz_common::properties::BoolProperty*> show_attribute_text_;

  std::vector<std::string> available_attribute_names_;


  // Helper to render the markers. Copied including implementation rviz_default_plugins::MarkerArrayDisplay
  std::unique_ptr<MarkerCommon> marker_common_;

  void load(const rviz_common::Config & config) override;
  void update(float wall_dt, float ros_dt) override;
  void reset() override;


private Q_SLOTS:
  void handlePropertyChange();
};

}
