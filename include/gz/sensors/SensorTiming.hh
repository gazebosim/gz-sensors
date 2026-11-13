/*
 * Copyright (C) 2026 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
#ifndef GZ_SENSORS_SENSORTIMING_HH_
#define GZ_SENSORS_SENSORTIMING_HH_

#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

#include <gz/sensors/config.hh>
#include <gz/sensors/Export.hh>

namespace gz
{
  namespace sensors
  {
    // Inline bracket to help doxygen filtering.
    inline namespace GZ_SENSORS_VERSION_NAMESPACE {
    //
    class Sensor;

    /// \brief Whether a sensor had subscribers, the last time that the code
    /// that updates it checked in an interval. See SensorHasConnections.
    enum class SensorSubscribers
    {
      /// \brief Not checked in the interval
      UNKNOWN,

      /// \brief Nothing subscribed
      NONE,

      /// \brief Something subscribed
      SOME
    };

    /// \brief Wall time spent generating data for one sensor, and whether it
    /// had a reason to generate it.
    struct SensorTiming
    {
      /// \brief Name of the sensor
      std::string name;

      /// \brief Name of the parent of the sensor, see Sensor::Parent
      std::string parent;

      /// \brief Topic of the sensor
      std::string topic;

      /// \brief Number of updates
      uint64_t updates{0};

      /// \brief Total time of those updates
      std::chrono::nanoseconds total{0};

      /// \brief Part of the total that was spent blocked on the GPU. Always
      /// zero for sensors that do not render.
      std::chrono::nanoseconds gpuWait{0};

      /// \brief True for a sensor that renders, a RenderingSensor
      bool rendering{false};

      /// \brief Number of the updates that rendered. A rendering sensor that
      /// updates without rendering only publishes information about its
      /// data, like a camera that only has camera_info subscribers.
      uint64_t renders{0};

      /// \brief Whether the sensor had subscribers
      SensorSubscribers subscribers{SensorSubscribers::UNKNOWN};
    };

    /// \brief Check if a sensor has subscribers, see Sensor::HasConnections.
    /// While the timing is being collected, the answer is recorded as well, so
    /// that it can tell the sensors that generate no data because nothing
    /// subscribes to them. The code that decides whether to update a sensor
    /// calls this instead of Sensor::HasConnections, from the thread that
    /// updates the sensor.
    /// \param[in] _sensor The sensor
    /// \return True if something subscribes to the sensor
    GZ_SENSORS_VISIBLE
    bool SensorHasConnections(const Sensor &_sensor);

    /// \brief Enable or disable collecting per sensor timing. It is disabled by
    /// default, in which case Sensor::Update only pays for one atomic read.
    ///
    /// The collection is process wide because the sensors of a simulation are
    /// usually owned by several independent systems.
    /// \param[in] _enabled True to collect
    GZ_SENSORS_VISIBLE
    void SetSensorTimingEnabled(bool _enabled);

    /// \brief Check if per sensor timing is being collected.
    /// \return True if collecting
    GZ_SENSORS_VISIBLE
    bool SensorTimingEnabled();

    /// \brief Get the timing collected since the previous call, and start
    /// collecting a new interval.
    /// \return One entry for each sensor that was updated, or whose
    /// subscribers were checked with SensorHasConnections, in the interval
    GZ_SENSORS_VISIBLE
    std::vector<SensorTiming> TakeSensorTimings();
    }
  }
}
#endif
