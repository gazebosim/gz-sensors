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
#ifndef IGNITION_SENSORS_CAMERAUTILS_HH_
#define IGNITION_SENSORS_CAMERAUTILS_HH_

/// \brief This file contains utilities useful for working with camera sensors

#include <memory>

#include <ignition/sensors/ign_sensors_export.h>
#include <ignition/msgs.hh>
#include <sdf/sdf.hh>

namespace ignition
{
  namespace sensors
  {
    // TODO Should this be a constructor on a camera sensor instead?
    /// \brief Build an SDFormat Element for a Camera Sensor
    /// \param[in] _name Name of the camera sensor
    /// \param[in] _topic ignition-transport topic the camera should publish to
    /// \param[in] _hz Rate in Hz that the camera should generate images
    /// \param[in] _width Width of the image in pixels
    /// \param[in] _height Height of the image in pixels
    /// \param[in] _hfov Horizontal field of view in radians
    /// \param[in] _near Near clip plane distance
    /// \param[in] _far clip plane distance
    /// \return An SDF Element pointer to the camera config
    sdf::ElementPtr IGN_SENSORS_EXPORT CameraConfig(const std::string &_name,
        const std::string &_topic, double _hz, std::size_t _width,
        std::size_t _height, double _hfov, double _near, double _far);
  }
}

#endif
