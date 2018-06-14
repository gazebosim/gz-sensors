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
#ifndef IGNITION_SENSORS_GPULIDARSENSOR_HH_
#define IGNITION_SENSORS_GPULIDARSENSOR_HH_

#include <memory>

#include <ignition/common/Event.hh>
#include <ignition/sensors/ign_sensors_gpu_lidar_export.hh>
#include <ignition/sensors/Sensor.hh>
#include <ignition/msgs.hh>

namespace ignition
{
  namespace sensors
  {
    /// \brief forward declarations
    class GpuLidarSensorPrivate;

    /// \brief GpuLidar Sensor Class
    ///
    ///   This class creates laser scans using the GPU. It's measures the range
    ///   from the origin of the center to points on the visual geometry in the
    ///   scene.
    ///
    ///   It offers both an ignition-transport interface and a direct C++ API
    ///   to access the image data. The API works by setting a callback to be
    ///   called with image data.
    class IGN_SENSORS_GPU_LIDAR_EXPORT GpuLidarSensor : public Sensor
    {
      /// \brief constructor
      public: GpuLidarSensor();

      /// \brief destructor
      public: virtual ~GpuLidarSensor();

      /// \brief Force the sensor to generate data
      /// \param[in] _now The current time
      /// \return true if the update was successfull
      public: virtual bool Update(const common::Time &_now) override;

      /// \brief Load the sensor with SDF parameters.
      /// \param[in] _sdf SDF Sensor parameters.
      /// \return true if loading was successful
      public: virtual bool Load(sdf::ElementPtr _sdf) override;

      /// \brief Initialize values in the sensor
      /// \return True on success
      public: virtual bool Init() override;

      /// \brief Get Cos Vert field-of-view
      /// \return 2 * atan(tan(this->vfov/2) / cos(this->hfov/2))
      public: double CosVertFOV() const;

     /// \brief Get Cos Horz field-of-view
      /// \return 2 * atan(tan(this->hfov/2) / cos(this->vfov/2))
      public: double CosHorzFOV() const;

       /// \brief Get (horizontal_max_angle + horizontal_min_angle) * 0.5
      /// \return (horizontal_max_angle + horizontal_min_angle) * 0.5
      public: double HorzHalfAngle() const;

     /// \brief Get (vertical_max_angle + vertical_min_angle) * 0.5
      /// \return (vertical_max_angle + vertical_min_angle) * 0.5
      public: double VertHalfAngle() const;

      /// \brief Returns a pointer to the internally kept rendering::GpuLaser
      /// \return Pointer to GpuLaser
      public: rendering::GpuLaserPtr LaserCamera() const;

       /// \brief Gets the camera count
      /// \return Number of cameras
      public: unsigned int CameraCount() const;

     /// \brief Connect to the new laser frame event.
      /// \param[in] _subscriber Event callback.
      public: event::ConnectionPtr ConnectNewLaserFrame(
        std::function<void(const float *, unsigned int, unsigned int,
        unsigned int, const std::string &)> _subscriber);

      /// \brief Data pointer for private data
      /// \internal
      private: std::unique_ptr<GpuLidarSensorPrivate> dataPtr;
    };
  }
}

#endif

