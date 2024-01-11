#include "track_to_track_fusion/trace_storage.h"
#include <set>
#include <iostream>
#include "track_to_track_fusion/global.h"

static const rclcpp::Logger LOGGER = rclcpp::get_logger("trace_storage.cpp");

namespace track_to_track_fusion
{
  namespace internal
  {
    bool traceWasAlreadySeen(ObjectWithTrace::Trace const &trace_previous_cycle,
                             ObjectWithTrace::Trace const &trace_this_cycle)
    {
      // True cases:
      // a) everything is the same
      // b) one or more sensors vanished, but the ids of the remaining sensors are identical
      // c) one or more sensors are new, but the ids of the previous ones are the same

      if (trace_this_cycle.size() == 0 || trace_previous_cycle.size() == 0)
        throw std::runtime_error("Traces must not be empty");

      std::set<std::string> sensor_modalities;
      for (auto const &trace : {trace_previous_cycle, trace_this_cycle})
      {
        std::transform(trace.begin(), trace.end(), std::inserter(sensor_modalities, sensor_modalities.begin()),
                       [](ObjectWithTrace::Trace::value_type const &pair) { return pair.first; });
      }

      auto const disjunct = sensor_modalities.size() == trace_this_cycle.size() + trace_previous_cycle.size();

      auto ret = !disjunct;
      for (auto const &sensor_modality : sensor_modalities)
      {
        if (!ret)
          break;

        ret &= trace_previous_cycle.count(sensor_modality) == 0 || trace_this_cycle.count(sensor_modality) == 0 ||
               trace_this_cycle.at(sensor_modality) == trace_previous_cycle.at(sensor_modality);
      }

#if 0
      if (ret)
      {
        RCLCPP_INFO_STREAM(LOGGER, "[found same trace]: ");
        for (auto const &p : trace_previous_cycle)
        {
          RCLCPP_INFO_STREAM(LOGGER, "Prev: " << p.first << ":" << p.second);
        }
        for (auto const &p : trace_this_cycle)
        {
          RCLCPP_INFO_STREAM(LOGGER, "This: " << p.first << ":" << p.second);
        }
      }
      else
      {
        RCLCPP_INFO_STREAM(LOGGER, "[new trace]: ");
        for (auto const &p : trace_previous_cycle)
        {
          RCLCPP_INFO_STREAM(LOGGER, "Prev: " << p.first << ":" << p.second);
        }
        for (auto const &p : trace_this_cycle)
        {
          RCLCPP_INFO_STREAM(LOGGER, "This: " << p.first << ":" << p.second);
        }
      }
#endif

      return ret;
    }

  } // namespace internal

  PerceptionKitObject::_id_type TraceStorage::getIdFromTrace(ObjectWithTrace::Trace const &trace_this_cycle)
  {
    static PerceptionKitObject::_id_type id{0};          //! 初始化 id=0
    static std::map<int, ObjectWithTrace::Trace> traces; //! 初始化 traces = <id, <'camera_01_01', 167>>

    if (DEBUG_MODE_)
    {
      RCLCPP_INFO_STREAM(LOGGER, "--------------- new trace: size[" << trace_this_cycle.size() << "] ---------------");
      for (auto const &p : trace_this_cycle)
      {
        RCLCPP_INFO_STREAM(LOGGER, "" << p.first << ":" << p.second);
      }

      for (auto trace : traces) //! 历史 traces
      {
        RCLCPP_INFO_STREAM(LOGGER, "--------------- history trace: id[" << trace.first << "/" << traces.size() << "], size[" << trace.second.size() << "] ---------------");
        for (auto const &p : trace.second)
        {
          RCLCPP_INFO_STREAM(LOGGER, "" << p.first << ":" << p.second);
        }

        if (internal::traceWasAlreadySeen(trace.second, trace_this_cycle))
        {
          //todo liang print
          RCLCPP_INFO_STREAM(LOGGER, "[found same trace!]");

          traces.at(trace.first) = trace_this_cycle; //! traces(历史id) = trace_this_cycle
          return trace.first;                        //! 历史 traces出现过,则返回历史id
        }
        else
        {
          //todo liang print
          RCLCPP_INFO_STREAM(LOGGER, "[not match!]");
        }
      }
    }
    else
    {
      for (auto const &trace : traces) //! 历史 traces
      {
        if (internal::traceWasAlreadySeen(trace.second, trace_this_cycle))
        {
          traces.at(trace.first) = trace_this_cycle; //! traces(历史id) = trace_this_cycle
          return trace.first;                        //! 历史 traces出现过,则返回历史id
        }
      }
    }

    // @todo: cleanup old traces

    //! 历史traces没有出现过, 添加新的id
    ++id;
    traces[id] = trace_this_cycle;
    return id;
  }

} // namespace track_to_track_fusion