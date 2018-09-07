/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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
#ifndef IGNITION_SENSORS_SENSORTYPES_HH_
#define IGNITION_SENSORS_SENSORTYPES_HH_

#include <vector>
#include <memory>

#include <ignition/common/EnumIface.hh>
#include <ignition/sensors/config.hh>
#include <ignition/sensors/Export.hh>

/// \file
/// \ingroup ignition_sensors
/// \brief Forward declarations and typedefs for sensors
namespace ignition
{
  /// \ingroup ignition_sensors
  /// \brief Sensors namespace
  namespace sensors
  {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_SENSORS_VERSION_NAMESPACE {
    // Forward declarations.
    class AltimeterSensor;
    class CameraSensor;
    class GaussianNoiseModel;
    class ImageGaussianNoiseModel;
    class Noise;
    class Sensor;

    /// \def SensorPtr
    /// \brief Shared pointer to Sensor
    typedef std::shared_ptr<Sensor> SensorPtr;

    /// \def CameraSensorPtr
    /// \brief Shared pointer to CameraSensor
    typedef std::shared_ptr<CameraSensor> CameraSensorPtr;

    /// \def NoisePtr
    /// \brief Shared pointer to Noise
    typedef std::shared_ptr<Noise> NoisePtr;

    /// \def GaussianNoisePtr
    /// \brief Shared pointer to Noise
    typedef std::shared_ptr<GaussianNoiseModel> GaussianNoiseModelPtr;

    /// \brief Shared pointer to Noise
    typedef std::shared_ptr<ImageGaussianNoiseModel>
        ImageGaussianNoiseModelPtr;

    /// \def Sensor_V
    /// \brief Vector of Sensor shared pointers
    typedef std::vector<SensorPtr> Sensor_V;

    /// \def CameraSensor_V
    /// \brief Vector of CameraSensor shared pointers
    typedef std::vector<CameraSensorPtr> CameraSensor_V;

    /// \def SensorNoiseType
    /// \brief Eumeration of all sensor noise types
    enum SensorNoiseType
    {
      /// \internal
      /// \brief Indicator used to create an iterator over the enum. Do not
      /// use this.
      SENSOR_NOISE_TYPE_BEGIN = 0,

      /// \brief Noise streams for the Camera sensor
      /// \sa CameraSensor
      NO_NOISE = SENSOR_NOISE_TYPE_BEGIN,

      /// \brief Noise streams for the Camera sensor
      /// \sa CameraSensor
      CAMERA_NOISE = 1,

      /// \internal
      /// \brief Indicator used to create an iterator over the enum. Do not
      /// use this.
      SENSOR_NOISE_TYPE_END
    };
    /// \}

    /// \brief SensorCategory is used to categorize sensors. This is used to
    /// put sensors into different threads.
    enum SensorCategory
    {
      // IMAGE must be the first element, and it must start with 0. Do not
      // change this! See SensorManager::sensorContainers for reference.
      /// \brief Image based sensor class. This type requires the rendering
      /// engine.
      IMAGE = 0,

      /// \brief Ray based sensor class.
      RAY = 1,

      /// \brief A type of sensor is not a RAY or IMAGE sensor.
      OTHER = 2,

      /// \brief Number of Sensor Categories
      CATEGORY_COUNT = 3
    };
    }
  }
}
#endif
