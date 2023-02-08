/*
 * Copyright (C) 2023 Open Source Robotics Foundation
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
#ifndef GZ_SENSORS_AIRSPEEDSENSOR_HH_
#define GZ_SENSORS_AIRSPEEDSENSOR_HH_

#include <memory>

#include <sdf/sdf.hh>

#include <gz/utils/SuppressWarning.hh>

#include <gz/sensors/config.hh>
#include <gz/sensors/air_speed/Export.hh>

#include "gz/sensors/Sensor.hh"

namespace gz
{
  namespace sensors
  {
    // Inline bracket to help doxygen filtering.
    inline namespace GZ_SENSORS_VERSION_NAMESPACE {
    //
    /// \brief forward declarations
    class AirSpeedSensorPrivate;

    /// \brief AirSpeed Sensor Class
    ///
    /// A sensor that reports air speed through differential pressure readings.
    class AirSpeedSensor :
      public Sensor
    {
      /// \brief constructor
      public: AirSpeedSensor();

      /// \brief destructor
      public: virtual ~AirSpeedSensor();

      /// \brief Load the sensor based on data from an sdf::Sensor object.
      /// \param[in] _sdf SDF Sensor parameters.
      /// \return true if loading was successful
      public: virtual bool Load(const sdf::Sensor &_sdf) override;

      /// \brief Load the sensor with SDF parameters.
      /// \param[in] _sdf SDF Sensor parameters.
      /// \return true if loading was successful
      public: virtual bool Load(sdf::ElementPtr _sdf) override;

      /// \brief Initialize values in the sensor
      /// \return True on success
      public: virtual bool Init() override;

      /// \brief Get the current velocity.
      /// \return Current velocity of the sensor.
      public: gz::math::Vector3d Velocity() const;

      /// \brief Update the velocity of the sensor
      public: void SetVelocity(const gz::math::Vector3d &_vel);

      using Sensor::Update;

      /// \brief Update the sensor and generate data
      /// \param[in] _now The current time
      /// \return true if the update was successfull
      public: virtual bool Update(
        const std::chrono::steady_clock::duration &_now) override;

      /// \brief Check if there are any subscribers
      /// \return True if there are subscribers, false otherwise
      public: virtual bool HasConnections() const override;

      GZ_UTILS_WARN_IGNORE__DLL_INTERFACE_MISSING
      /// \brief Data pointer for private data
      /// \internal
      private: std::unique_ptr<AirSpeedSensorPrivate> dataPtr;
      GZ_UTILS_WARN_RESUME__DLL_INTERFACE_MISSING
    };
    }
  }
}

#endif
