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
#ifndef IGNITION_SENSORS_SENSOR_HH_
#define IGNITION_SENSORS_SENSOR_HH_

namespace ignition
{
  namespace sensors
  {
    /// \brief A string used to identify a sensor
    using SensorId = std::size_t;

    /// \brief a base sensor class
    class Sensor
    {
      /// \brief constructor
      public: Sensor();

      /// \brief destructor
      public: virtual ~Sensor();

      // TODO how to tell if it needs ignition physics?
      // TODO how to tell if it needs ignition rendering?
      // TODO What kind of data is it outputting?
      // TODO What is it's ID?
      // TODO What is it's name?
    };
  }
}

#endif
