/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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
#ifndef IGNITION_SENSORS_SENSORPLUGIN_HH_
#define IGNITION_SENSORS_SENSORPLUGIN_HH_

#include <ignition/sensors/config.hh>
#include <ignition/sensors/Export.hh>

namespace ignition
{
  namespace sensors
  {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_SENSORS_VERSION_NAMESPACE {
    //
    /// \brief Base sensor plugin interface
    class IGNITION_SENSORS_VISIBLE SensorInterface
    {
      /// \brief Instantiate new sensor
      /// \return New sensor
      public: virtual Sensor *New() const = 0;
    };

    /// \brief Templated class for instantiating sensors of the specified type
    /// \tparam Type of sensor being instantiated.
    template<typename SensorType>
    class IGNITION_SENSORS_VISIBLE SensorPlugin:
      public SensorInterface
    {
      // Documentation inherited
      public: Sensor *New() const override
              {
                return new SensorType();
              };
    };
    }
  }
}
#endif
