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
#ifndef GZ_SENSORS_SENSORTIMINGPRIVATE_HH_
#define GZ_SENSORS_SENSORTIMINGPRIVATE_HH_

#include <chrono>

#include "gz/sensors/SensorTiming.hh"

namespace gz
{
  namespace sensors
  {
    // Inline bracket to help doxygen filtering.
    inline namespace GZ_SENSORS_VERSION_NAMESPACE {
    //
    class Sensor;

    /// \brief Add one update of a sensor to the current interval.
    /// \param[in] _sensor Sensor that was updated
    /// \param[in] _total Wall time of the update
    /// \param[in] _gpuWait Part of it spent blocked on the GPU
    void RecordSensorTiming(const Sensor &_sensor,
        std::chrono::nanoseconds _total, std::chrono::nanoseconds _gpuWait);
    }
  }
}
#endif
