#include "objects_prediction/constant_velocity.h"
#include <iomanip>
#include "track_to_track_fusion/global.h"

static const rclcpp::Logger LOGGER = rclcpp::get_logger("objects_prediction/constant_velocity.cpp");

namespace objects_prediction
{
  void ConstantVelocityPredictor::predict(track_to_track_fusion::NonCopyableObjects &objects,
                                          RosTime const &requested_prediction_time) const
  {
    objects.header.stamp = requested_prediction_time;
    for (auto &object : objects.objects)
    {
      auto const prediction_duration = getSecondsFromRosTime(requested_prediction_time - object.header.stamp);

      // @todo: Check if the time is far off ...

      //! liang
      if (DEBUG_MODE_)
      {
        RCLCPP_INFO_STREAM(LOGGER, "\n--------------------------------------------------------------- [" << object.id << "] ---------------------------------------------------\n"
                                                                                                         << std::fixed << std::setprecision(14)
                                                                                                         << " t_sensor[" << object.header.stamp.sec << "." << object.header.stamp.nanosec
                                                                                                         << "], t_now[" << requested_prediction_time.seconds()
                                                                                                         << "] || dx[" << object.velocity.x * prediction_duration
                                                                                                         << "] = vx[" << object.velocity.x
                                                                                                         << "] * dt[" << prediction_duration << "]"
                                                                                                         << "\n--------------------------------------------------------------------------------------------------------------------------");
      }

      object.header.stamp = requested_prediction_time;
      object.position.x += object.velocity.x * prediction_duration;
      object.position.y += object.velocity.y * prediction_duration;
      object.position.z += object.velocity.z * prediction_duration;
    }
  }
} // namespace objects_prediction