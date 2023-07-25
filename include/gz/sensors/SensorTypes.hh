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
#ifndef GZ_SENSORS_SENSORTYPES_HH_
#define GZ_SENSORS_SENSORTYPES_HH_

#include <vector>
#include <memory>

#include <gz/common/EnumIface.hh>
#include <gz/sensors/config.hh>
#include <gz/sensors/Export.hh>

/// \file
/// \ingroup gz_sensors
/// \brief Forward declarations and typedefs for sensors
namespace gz
{
  /// \ingroup gz_sensors
  /// \brief Sensors namespace
  namespace sensors
  {
    // Inline bracket to help doxygen filtering.
    inline namespace GZ_SENSORS_VERSION_NAMESPACE {
    // Forward declarations.
    class AltimeterSensor;
    class CameraSensor;
    class GpuLidarSensor;
    class GaussianNoiseModel;
    class ImageGaussianNoiseModel;
    class Noise;
    class Distortion;
    class BrownDistortionModel;
    class ImageBrownDistortionModel;
    class Sensor;

    /// \def SensorPtr
    /// \brief Shared pointer to Sensor
    typedef std::shared_ptr<Sensor> SensorPtr;

    /// \def CameraSensorPtr
    /// \brief Shared pointer to CameraSensor
    typedef std::shared_ptr<CameraSensor> CameraSensorPtr;

    /// \def GpuLidarSensorPtr
    /// \brief Shared pointer to GpuLidarSensor
    typedef std::shared_ptr<GpuLidarSensor> GpuLidarSensorPtr;

    /// \def NoisePtr
    /// \brief Shared pointer to Noise
    typedef std::shared_ptr<Noise> NoisePtr;

    /// \def GaussianNoiseModelPtr
    /// \brief Shared pointer to Noise
    typedef std::shared_ptr<GaussianNoiseModel> GaussianNoiseModelPtr;

    /// \brief Shared pointer to Noise
    typedef std::shared_ptr<ImageGaussianNoiseModel>
        ImageGaussianNoiseModelPtr;

    /// \def DistortionPtr
    /// \brief Shared pointer to Distortion
    typedef std::shared_ptr<Distortion> DistortionPtr;

    /// \def DistortionPtr
    /// \brief Shared pointer to Distortion
    typedef std::shared_ptr<BrownDistortionModel> BrownDistortionModelPtr;

    /// \def DistortionPtr
    /// \brief Shared pointer to Distortion
    typedef std::shared_ptr<ImageBrownDistortionModel>
        ImageBrownDistortionModelPtr;

    /// \def Sensor_V
    /// \brief Vector of Sensor shared pointers
    typedef std::vector<SensorPtr> Sensor_V;

    /// \def CameraSensor_V
    /// \brief Vector of CameraSensor shared pointers
    typedef std::vector<CameraSensorPtr> CameraSensor_V;

    /// \def GpuLidarSensor_V
    /// \brief Vector of GpuLidarSensor shared pointers
    typedef std::vector<GpuLidarSensorPtr> GpuLidarSensor_V;

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

      /// \brief Magnetometer body-frame X axis noise in Tesla
      /// \sa MagnetometerSensor
      MAGNETOMETER_X_NOISE_TESLA = 2,

      /// \brief Magnetometer body-frame Y axis noise in Tesla
      /// \sa MagnetometerSensor
      MAGNETOMETER_Y_NOISE_TESLA = 3,

      /// \brief Magnetometer body-frame Z axis noise in Tesla
      /// \sa MagnetometerSensor
      MAGNETOMETER_Z_NOISE_TESLA = 4,

      /// \brief Vertical noise stream for the altimeter sensor
      /// \sa AltimeterSensor
      ALTIMETER_VERTICAL_POSITION_NOISE_METERS = 5,

      /// \brief Velocity noise streams for the altimeter sensor
      /// \sa AltimeterSensor
      ALTIMETER_VERTICAL_VELOCITY_NOISE_METERS_PER_S = 6,

      /// \brief Air Pressure noise streams for the air pressure sensor
      /// \sa AirPressureSensor
      AIR_PRESSURE_NOISE_PASCALS = 7,

      /// \brief Accelerometer body-frame X axis noise in m/s^2
      /// \sa ImuSensor
      ACCELEROMETER_X_NOISE_M_S_S = 8,

      /// \brief Accelerometer body-frame Y axis noise in m/s^2
      /// \sa ImuSensor
      ACCELEROMETER_Y_NOISE_M_S_S = 9,

      /// \brief Accelerometer body-frame Z axis noise in m/s^2
      /// \sa ImuSensor
      ACCELEROMETER_Z_NOISE_M_S_S = 10,

      /// \brief Gyroscope body-frame X axis noise in m/s^2
      /// \sa ImuSensor
      GYROSCOPE_X_NOISE_RAD_S = 11,

      /// \brief Gyroscope body-frame X axis noise in m/s^2
      /// \sa ImuSensor
      GYROSCOPE_Y_NOISE_RAD_S = 12,

      /// \brief Gyroscope body-frame X axis noise in m/s^2
      /// \sa ImuSensor
      GYROSCOPE_Z_NOISE_RAD_S = 13,

      /// \brief Noise streams for the Lidar sensor
      /// \sa Lidar
      LIDAR_NOISE = 14,

      /// \brief Force body-frame X axis noise in N
      /// \sa ForceTorqueSensor
      FORCE_X_NOISE_N = 15,

      /// \brief Force body-frame Y axis noise in N
      /// \sa ForceTorqueSensor
      FORCE_Y_NOISE_N = 16,

      /// \brief Force body-frame Z axis noise in N
      /// \sa ForceTorqueSensor
      FORCE_Z_NOISE_N = 17,

      /// \brief Torque body-frame X axis noise in Nm
      /// \sa ForceTorqueSensor
      TORQUE_X_NOISE_N_M = 18,

      /// \brief Torque body-frame Y axis noise in Nm
      /// \sa ForceTorqueSensor
      TORQUE_Y_NOISE_N_M = 19,

      /// \brief Torque body-frame Z axis noise in Nm
      /// \sa ForceTorqueSensor
      TORQUE_Z_NOISE_N_M = 20,

      /// \brief Noise streams for the NavSat position sensor
      /// \sa NavSat
      NAVSAT_HORIZONTAL_POSITION_NOISE = 21,

      /// \brief Noise streams for the NavSat position sensor
      /// \sa NavSat
      NAVSAT_VERTICAL_POSITION_NOISE = 22,

      /// \brief Noise streams for the NavSat velocity sensor
      /// \sa NavSat
      NAVSAT_HORIZONTAL_VELOCITY_NOISE = 23,

      /// \brief Noise streams for the NavSat velocity sensor
      /// \sa NavSat
      NAVSAT_VERTICAL_VELOCITY_NOISE = 24,

      /// \brief Air speed noise streams for the air speed sensor
      /// \sa AirSpeedSensor
      AIR_SPEED_NOISE_PASCALS = 25,

      /// \internal
      /// \brief Indicator used to create an iterator over the enum. Do not
      /// use this.
      SENSOR_NOISE_TYPE_END
    };


    /// \def SensorDistortionType
    /// \brief Eumeration of all sensor noise types
    enum SensorDistortionType
    {
      /// \internal
      /// \brief Indicator used to create an iterator over the enum. Do not
      /// use this.
      SENSOR_DISTORTION_TYPE_BEGIN = 0,

      /// \brief Noise streams for the Camera sensor
      /// \sa CameraSensor
      NO_DISTORTION = SENSOR_DISTORTION_TYPE_BEGIN,

      /// \brief Noise streams for the Camera sensor
      /// \sa CameraSensor
      CAMERA_DISTORTION = 1,

      /// \internal
      /// \brief Indicator used to create an iterator over the enum. Do not
      /// use this.
      SENSOR_DISTORTION_TYPE_END
    };

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
