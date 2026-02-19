/*
 * Copyright (C) 2026 Open Source Robotics Foundation
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
#ifndef GZ_SENSORS_CPULIDARSENSOR_HH_
#define GZ_SENSORS_CPULIDARSENSOR_HH_

#include <memory>
#include <utility>
#include <vector>

#include <sdf/sdf.hh>

#include <gz/math/Angle.hh>
#include <gz/math/Vector3.hh>
#include <gz/utils/SuppressWarning.hh>

#include <gz/sensors/config.hh>
#include <gz/sensors/cpu_lidar/Export.hh>

#include "gz/sensors/Sensor.hh"

namespace gz
{
  namespace sensors
  {
    inline namespace GZ_SENSORS_VERSION_NAMESPACE {

    class CpuLidarSensorPrivate;

    /// \brief CPU-based lidar sensor that performs ray casting without
    /// depending on gz-rendering.
    class GZ_SENSORS_CPU_LIDAR_VISIBLE CpuLidarSensor : public Sensor
    {
      /// \brief Result of a single ray cast
      public: struct RayResult
      {
        /// \brief Hit point in entity frame
        gz::math::Vector3d point;

        /// \brief Fraction along the ray [0, 1]. NaN if no hit.
        double fraction;

        /// \brief Normal at hit point in entity frame
        gz::math::Vector3d normal;

        /// \brief Intensity value
        double intensity = 0.0;
      };
      /// \brief constructor
      public: CpuLidarSensor();

      /// \brief destructor
      public: virtual ~CpuLidarSensor();

      /// \brief Load the sensor based on data from an sdf::Sensor object.
      /// \param[in] _sdf SDF Sensor parameters.
      /// \return true if loading was successful
      public: virtual bool Load(const sdf::Sensor &_sdf) override;

      /// \brief Load the sensor with SDF parameters.
      /// \param[in] _sdf SDF Sensor parameters.
      /// \return true if loading was successful
      public: virtual bool Load(sdf::ElementPtr _sdf) override;

      using Sensor::Update;

      /// \brief Update the sensor and generate data
      /// \param[in] _now The current time
      /// \return true if the update was successful
      public: virtual bool Update(
        const std::chrono::steady_clock::duration &_now) override;

      /// \brief Check if there are any subscribers
      /// \return True if there are subscribers, false otherwise
      public: virtual bool HasConnections() const override;

      /// \brief Get the minimum horizontal angle
      public: gz::math::Angle AngleMin() const;

      /// \brief Get the maximum horizontal angle
      public: gz::math::Angle AngleMax() const;

      /// \brief Get the minimum vertical angle
      public: gz::math::Angle VerticalAngleMin() const;

      /// \brief Get the maximum vertical angle
      public: gz::math::Angle VerticalAngleMax() const;

      /// \brief Get the minimum range
      public: double RangeMin() const;

      /// \brief Get the maximum range
      public: double RangeMax() const;

      /// \brief Get the horizontal ray count
      public: unsigned int RayCount() const;

      /// \brief Get the vertical ray count
      public: unsigned int VerticalRayCount() const;

      /// \brief Generate rays from the lidar configuration.
      /// \return Vector of (start, end) pairs in entity frame
      public: std::vector<std::pair<gz::math::Vector3d, gz::math::Vector3d>>
        GenerateRays() const;

      /// \brief Set the raycast results from the physics engine.
      /// \param[in] _results Vector of results, one per ray.
      public: void SetRaycastResults(
        const std::vector<RayResult> &_results);

      /// \brief Get all range values.
      /// \param[out] _ranges Vector to fill with range data.
      public: void Ranges(std::vector<double> &_ranges) const;

      GZ_UTILS_WARN_IGNORE__DLL_INTERFACE_MISSING
      /// \brief Pre-computed unit vectors for each ray.
      private: std::vector<gz::math::Vector3d> unitVectors;

      /// \brief Data pointer for private data
      /// \internal
      private: std::unique_ptr<CpuLidarSensorPrivate> dataPtr;
      GZ_UTILS_WARN_RESUME__DLL_INTERFACE_MISSING
    };
    }
  }
}

#endif
