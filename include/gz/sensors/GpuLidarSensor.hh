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
#ifndef IGNITION_SENSORS_GPULIDARSENSOR_HH_
#define IGNITION_SENSORS_GPULIDARSENSOR_HH_

#include <memory>
#include <string>

#include <sdf/sdf.hh>

#include <ignition/common/SuppressWarning.hh>

// TODO(louise) Remove these pragmas once ign-rendering is disabling the
// warnings
#ifdef _WIN32
#pragma warning(push)
#pragma warning(disable: 4251)
#endif
#include <ignition/rendering/GpuRays.hh>
#ifdef _WIN32
#pragma warning(pop)
#endif

#include "ignition/sensors/gpu_lidar/Export.hh"
#include "ignition/sensors/RenderingEvents.hh"
#include "ignition/sensors/Lidar.hh"

namespace ignition
{
  namespace sensors
  {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_SENSORS_VERSION_NAMESPACE {
    //
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
    class IGNITION_SENSORS_GPU_LIDAR_VISIBLE GpuLidarSensor : public Lidar
    {
      /// \brief constructor
      public: GpuLidarSensor();

      /// \brief destructor
      public: virtual ~GpuLidarSensor();

      /// \brief Force the sensor to generate data
      /// \param[in] _now The current time
      /// \return true if the update was successfull
      public: virtual bool Update(const common::Time &_now) override;

      /// \brief Initialize values in the sensor
      /// \return True on success
      public: virtual bool Init() override;

      /// \brief Load the sensor based on data from an sdf::Sensor object.
      /// \param[in] _sdf SDF Sensor parameters.
      /// \return true if loading was successful
      public: virtual bool Load(const sdf::Sensor &_sdf) override;

      /// \brief Load sensor sata from SDF
      /// \param[in] _sdf SDF used
      /// \return True on success
      public: virtual bool Load(sdf::ElementPtr _sdf) override;

      /// \brief Create Lidar sensor
      public: virtual bool CreateLidar() override;

      /// \brief Gets if sensor is horizontal
      /// \return True if horizontal, false if not
      public: bool IsHorizontal() const;

      /// \brief Makes possible to change sensor scene
      /// \param[in] _scene used with the sensor
      public: void SetScene(ignition::rendering::ScenePtr _scene) override;

      /// \brief Remove sensor from scene
      /// \param[in] _scene used with the sensor
      public: void RemoveGpuRays(ignition::rendering::ScenePtr _scene);

      /// \brief Get Gpu Rays object used in the sensor
      /// \return Pointer to ignition::rendering::GpuRays
      public: ignition::rendering::GpuRaysPtr GpuRays() const;

      /// \brief Return the ratio of horizontal ray count to vertical ray
      /// count.
      ///
      /// A ray count is the number of simulated rays. Whereas a range count
      /// is the total number of data points returned. When range count
      /// != ray count, then values are interpolated between rays.
      public: double RayCountRatio() const;

      /// \brief Get the horizontal field of view of the laser sensor.
      /// \return The horizontal field of view of the laser sensor.
      public: ignition::math::Angle HFOV() const;

      /// \brief Get the vertical field-of-view.
      /// \return Vertical field of view.
      public: ignition::math::Angle VFOV() const;

      /// \brief Connect function pointer to internal GpuRays callback
      /// \return ignition::common::Connection pointer
      public: virtual ignition::common::ConnectionPtr ConnectNewLidarFrame(
          std::function<void(const float *_scan, unsigned int _width,
                  unsigned int _heighti, unsigned int _channels,
                  const std::string &/*_format*/)> _subscriber) override;

      IGN_COMMON_WARN_IGNORE__DLL_INTERFACE_MISSING
      /// \brief Data pointer for private data
      /// \internal
      private: std::unique_ptr<GpuLidarSensorPrivate> dataPtr;
      IGN_COMMON_WARN_RESUME__DLL_INTERFACE_MISSING
    };
    }
  }
}

#endif
