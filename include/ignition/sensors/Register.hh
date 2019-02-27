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

#ifndef IGNITION_SENSORS_REGISTER_HH_
#define IGNITION_SENSORS_REGISTER_HH_

#include <ignition/plugin/Register.hh>


#include <ignition/sensors/config.hh>
#include "ignition/sensors/SensorPlugin.hh"
#include "ignition/sensors/SensorFactory.hh"

/*namespace ignition
{
  namespace sensors
  {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_SENSORS_VERSION_NAMESPACE {
    }
  }
}
*/
/// \brief Sensor registration macro
#define IGN_SENSORS_REGISTER_SENSOR(classname) \
IGNITION_ADD_PLUGIN(\
   ignition::sensors::IGNITION_SENSORS_VERSION_NAMESPACE::SensorTypePlugin<classname>, \
   ignition::sensors::IGNITION_SENSORS_VERSION_NAMESPACE::SensorPlugin)

#endif
