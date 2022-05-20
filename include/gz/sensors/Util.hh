/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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
#ifndef GZ_SENSORS_UTIL_HH_
#define GZ_SENSORS_UTIL_HH_

#include <string>

#include <sdf/Element.hh>
#include <sdf/Sensor.hh>

#include "gz/sensors/config.hh"
#include "gz/sensors/Export.hh"

namespace gz
{
namespace sensors
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SENSORS_VERSION_NAMESPACE {
//
/// \brief Get the name of a sensor's custom type from SDF.
///
/// Given an SDF tag as follows:
///
///     <sensor name="sensor_name" type="custom" ignition:type="sensor_type">
///
/// This function returns `sensor_type`.
///
/// It will return an empty string if the element is malformed. For example,
/// if it misses the `ignition:type` attribute or is not of `type="custom"`.
///
/// \param[in] _sdf Sensor SDF object.
/// \return _sensorType Name of sensor type.
std::string GZ_SENSORS_VISIBLE customType(const sdf::Sensor &_sdf); // NOLINT

/// \brief Get the name of a sensor's custom type from SDF.
///
/// Given an SDF tag as follows:
///
///     <sensor name="sensor_name" type="custom" ignition:type="sensor_type">
///
/// This function returns `sensor_type`.
///
/// It will return an empty string if the element is malformed. For example,
/// if it misses the `ignition:type` attribute or is not of `type="custom"`.
///
/// \param[in] _sdf Sensor SDF object.
/// \return _sensorType Name of sensor type.
std::string GZ_SENSORS_VISIBLE customType(sdf::ElementPtr _sdf); // NOLINT
}
}
}

#endif
