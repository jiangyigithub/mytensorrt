#pragma once

#include "objects_processors/objects_processors.h"
#include "rclcpp/rclcpp.hpp"

namespace objects_processors
{
  class MapFilter : public SingleObjectProcessor
  {
  public:
    explicit MapFilter(double const &x_max, double const &x_min, double const &y_max, double const &y_min);
    // explicit MapFilter();

    bool process(PerceptionKitObject &object, std::string const &sensor_modality) const override;

  private:
    double const x_max_, x_min_, y_max_, y_min_;
  };

} // namespace objects_processors