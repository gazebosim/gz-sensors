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

#ifndef IGNITION_SENSORS_IMAGENOISE_HH_
#define IGNITION_SENSORS_IMAGENOISE_HH_

#include <string>

#include <sdf/sdf.hh>

#include "ignition/sensors/config.hh"
#include "ignition/sensors/SensorTypes.hh"
#include "ignition/sensors/rendering/Export.hh"

namespace ignition
{
  namespace sensors
  {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_SENSORS_VERSION_NAMESPACE {
    // Forward declarations
    class NoisePrivate;

    /// \class NoiseFactory Noise.hh ignition/sensors/Noise.hh
    /// \brief Use this noise manager for creating and loading noise models.
    class IGNITION_SENSORS_RENDERING_VISIBLE ImageNoiseFactory
    {
      /// \brief Load a noise model based on the input sdf parameters and
      /// sensor type.
      /// \param[in] _sdf Noise sdf parameters.
      /// \param[in] _sensorType Type of sensor. This is currently used to
      /// distinguish between image and non image sensors in order to create
      /// the appropriate noise model.
      /// \return Pointer to the noise model created.
      public: static NoisePtr NewNoiseModel(sdf::ElementPtr _sdf,
          const std::string &_sensorType = "");

      /// \brief Load a noise model based on the input sdf parameters and
      /// sensor type.
      /// \param[in] _sdf Noise sdf parameters.
      /// \param[in] _sensorType Type of sensor. This is currently used to
      /// distinguish between image and non image sensors in order to create
      /// the appropriate noise model.
      /// \return Pointer to the noise model created.
      public: static NoisePtr NewNoiseModel(const sdf::Noise &_sdf,
                  const std::string &_sensorType = "");
    };
    }
  }
}
#endif
