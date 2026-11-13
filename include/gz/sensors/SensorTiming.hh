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
    /// \brief Wall time spent generating data for one sensor.
    struct SensorTiming
    {
      /// \brief Name of the sensor
      std::string name;

      /// \brief Name of the parent of the sensor, see Sensor::Parent
      std::string parent;

      /// \brief Topic of the sensor
      std::string topic;

      /// \brief Number of updates that generated data
      uint64_t updates{0};

      /// \brief Total time of those updates
      std::chrono::nanoseconds total{0};

      /// \brief Part of the total that was spent blocked on the GPU. Always
      /// zero for sensors that do not render.
      std::chrono::nanoseconds gpuWait{0};
    };

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
    /// \return One entry for each sensor that generated data in the interval
    GZ_SENSORS_VISIBLE
    std::vector<SensorTiming> TakeSensorTimings();
    }
  }
}
#endif
