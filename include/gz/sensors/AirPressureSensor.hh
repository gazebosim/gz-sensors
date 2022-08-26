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
#ifndef GZ_SENSORS_AIRPRESSURESENSOR_HH_
#define GZ_SENSORS_AIRPRESSURESENSOR_HH_

#include <memory>

#include <sdf/sdf.hh>

#include <gz/common/SuppressWarning.hh>
#include <gz/common/Time.hh>

#include <gz/sensors/config.hh>
#include <gz/sensors/air_pressure/Export.hh>

#include "gz/sensors/Sensor.hh"

namespace ignition
{
  namespace sensors
  {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_SENSORS_VERSION_NAMESPACE {
    //
    /// \brief forward declarations
    class AirPressureSensorPrivate;

    /// \brief AirPressure Sensor Class
    ///
    /// A sensor that reports air pressure readings.
    class IGNITION_SENSORS_AIR_PRESSURE_VISIBLE AirPressureSensor :
      public Sensor
    {
      /// \brief constructor
      public: AirPressureSensor();

      /// \brief destructor
      public: virtual ~AirPressureSensor();

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

      /// \brief Update the sensor and generate data
      /// \param[in] _now The current time
      /// \return true if the update was successfull
      public: virtual bool Update(const common::Time &_now) override;

      /// \brief Set the reference altitude.
      /// \param[in] _ref Verical reference position in meters
      public: void SetReferenceAltitude(double _reference);

      /// \brief Get the vertical reference altitude.
      /// \return Verical reference position in meters
      public: double ReferenceAltitude() const;

      IGN_COMMON_WARN_IGNORE__DLL_INTERFACE_MISSING
      /// \brief Data pointer for private data
      /// \internal
      private: std::unique_ptr<AirPressureSensorPrivate> dataPtr;
      IGN_COMMON_WARN_RESUME__DLL_INTERFACE_MISSING
    };
    }
  }
}

#endif
