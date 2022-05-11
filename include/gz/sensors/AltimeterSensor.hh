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
#ifndef GZ_SENSORS_ALTIMETERSENSOR_HH_
#define GZ_SENSORS_ALTIMETERSENSOR_HH_

#include <memory>

#include <sdf/sdf.hh>

#include <gz/utils/SuppressWarning.hh>

#include <gz/sensors/config.hh>
#include <gz/sensors/altimeter/Export.hh>

#include "gz/sensors/Sensor.hh"

namespace ignition
{
  namespace sensors
  {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_SENSORS_VERSION_NAMESPACE {
    //
    /// \brief forward declarations
    class AltimeterSensorPrivate;

    /// \brief Altimeter Sensor Class
    ///
    /// An altimeter sensor that reports vertical position and velocity
    /// readings over ign transport
    class IGNITION_SENSORS_ALTIMETER_VISIBLE AltimeterSensor : public Sensor
    {
      /// \brief constructor
      public: AltimeterSensor();

      /// \brief destructor
      public: virtual ~AltimeterSensor();

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

      using Sensor::Update;

      /// \brief Update the sensor and generate data
      /// \param[in] _now The current time
      /// \return true if the update was successfull
      public: virtual bool Update(
        const std::chrono::steady_clock::duration &_now) override;

      /// \brief Set the vertical reference position of the altimeter
      /// \param[in] _ref Verical reference position in meters
      public: void SetVerticalReference(double _reference);

      /// \brief Get the vertical reference position of the altimeter
      /// \return Verical reference position in meters
      public: double VerticalReference() const;

      /// \brief Set the current z position of the altimeter
      /// \param[in] _pos Z position in meters
      public: void SetPosition(double _pos);

      /// \brief Get the vertical position of the altimeter relative to the
      /// reference position
      /// \return Vertical position relative to referene position
      public: double VerticalPosition() const;

      /// \brief Set the vertical velocity of the altimeter
      /// \param[in] _vel Vertical velocity in meters per second
      public: void SetVerticalVelocity(double _vel);

      /// \brief Get the vertical velocity of the altimeter
      /// \return Vertical velocity in meters per second
      public: double VerticalVelocity() const;

      IGN_UTILS_WARN_IGNORE__DLL_INTERFACE_MISSING
      /// \brief Data pointer for private data
      /// \internal
      private: std::unique_ptr<AltimeterSensorPrivate> dataPtr;
      IGN_UTILS_WARN_RESUME__DLL_INTERFACE_MISSING
    };
    }
  }
}

#endif
