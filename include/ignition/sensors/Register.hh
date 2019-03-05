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

#include "ignition/sensors/SensorFactory.hh"
#include "ignition/sensors/SensorPlugin.hh"

/// \brief Sensor plugin registration macro
///
/// Use this macro to register sensor as plugins that are to be dynamically
// loaded
#define IGN_SENSORS_REGISTER_SENSOR_PLUGIN(classname) \
IGNITION_ADD_PLUGIN(\
   ignition::sensors::SensorPlugin<classname>, \
   ignition::sensors::SensorInterface)
#endif

/// \brief Static sensor registration macro
///
/// Use this macro to register sensors with the sensor factory.
#define IGN_SENSORS_REGISTER_STATIC_SENSOR(name, classname) \
class classname##Factory : public ignition::sensors::SensorInterface { \
public: \
    classname##Factory() \
    { \
        ignition::sensors::SensorFactory::RegisterSensor(name, this); \
    } \
    Sensor *New() const override { \
        return new classname(); \
    } \
}; \
static classname##Factory global_##classname##Factory;


