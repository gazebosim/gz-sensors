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
#include <cstdint>

#include "gz/sensors/Export.hh"
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
    /// \param[in] _rendered True if the update rendered
    void RecordSensorTiming(const Sensor &_sensor,
        std::chrono::nanoseconds _total, std::chrono::nanoseconds _gpuWait,
        bool _rendered);

    /// \brief Get the number of renders of rendering sensors that the calling
    /// thread has done so far. Used to tell if an update rendered.
    /// \return The count
    GZ_SENSORS_VISIBLE
    uint64_t ThreadRenderCount();

    /// \brief Count one render of a rendering sensor on the calling thread.
    /// RenderingSensor calls it, which lives in another library.
    GZ_SENSORS_VISIBLE
    void CountThreadRender();

    /// \brief Mark a sensor as a rendering sensor for as long as it exists.
    /// RenderingSensor calls it from its constructor.
    /// \param[in] _sensor The sensor
    GZ_SENSORS_VISIBLE
    void AddRenderingSensor(const Sensor &_sensor);

    /// \brief Forget a rendering sensor that is being destroyed.
    /// \param[in] _sensor The sensor
    GZ_SENSORS_VISIBLE
    void RemoveRenderingSensor(const Sensor &_sensor);
    }
  }
}
#endif
