/*
 * Copyright (C) 2017 Open Source Robotics Foundation
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

#include <ignition/sensors/Manager.hh>
#include <ignition/sensors/Sensor.hh>

namespace ignition
{
  namespace sensors
  {
    /// \brief a class for generating/manipulating sensor data
    class SensorPlugin
    {
      /// \brief Virtual destructor
      public: virtual ~SensorPlugin() = default;

      /// \brief Initialze plugin with a manager and sensor object
      public: virtual void Init(Manager *_mgr, Sensor *_sensor) = 0;

      /// \brief Load plugin from sdformat
      /// \param[in] _sdf SDF at <plugin> tag inside of <sensor>
      /// \return true if loading was successful
      public: virtual bool Load(sdf::ElementPtr _sdf) = 0;

      /// \brief Force the sensor to generate data
      /// \param[in] _now The current time
      public: virtual void Update(const common::Time &_now) = 0;
    };
  }
}

#endif
