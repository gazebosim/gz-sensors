/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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
#ifndef IGNITION_SENSORS_NAVSAT_HH_
#define IGNITION_SENSORS_NAVSAT_HH_

#include <memory>

#include <ignition/common/SuppressWarning.hh>
#include <sdf/Sensor.hh>

#include "gz/sensors/config.hh"
#include "gz/sensors/navsat/Export.hh"

#include "ignition/sensors/Sensor.hh"

namespace ignition
{
  namespace sensors
  {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_SENSORS_VERSION_NAMESPACE {
    //
    /// \brief forward declarations
    class NavSatPrivate;

    /// \brief NavSat Sensor Class
    ///
    /// A sensor that reports position and velocity readings over
    /// Ignition Transport using spherical coordinates (latitude / longitude).
    ///
    /// By default, it publishes `ignition::msgs::NavSat` messages on the
    /// `/.../navsat` topic.
    ///
    /// This sensor assumes the world is using the East-North-Up (ENU) frame.
    class IGNITION_SENSORS_NAVSAT_VISIBLE NavSatSensor : public Sensor
    {
      /// \brief Constructor
      public: NavSatSensor();

      /// \brief Destructor
      public: virtual ~NavSatSensor();

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
      public: virtual bool Update(
        const std::chrono::steady_clock::duration &_now) override;

      /// \brief Set the latitude of the NavSat
      /// \param[in] _latitude Latitude of NavSat
      public: void SetLatitude(const math::Angle &_latitude);

      /// \brief Get the latitude of the NavSat, wrapped between +/- 180
      /// degrees.
      /// \return Latitude angle.
      public: const math::Angle &Latitude() const;

      /// \brief Set the longitude of the NavSat
      /// \param[in] _longitude Longitude of NavSat
      public: void SetLongitude(const math::Angle &_longitude);

      /// \brief Get the longitude of the NavSat, wrapped between +/- 180
      /// degrees.
      /// \return Longitude angle.
      public: const math::Angle &Longitude() const;

      /// \brief Set the altitude of the NavSat
      /// \param[in] _altitude altitude of NavSat in meters
      public: void SetAltitude(double _altitude);

      /// \brief Get NavSat altitude above sea level
      /// \return Altitude in meters
      public: double Altitude() const;

      /// \brief Set the velocity of the NavSat in ENU world frame.
      /// \param[in] _vel NavSat in meters per second.
      public: void SetVelocity(const math::Vector3d &_vel);

      /// \brief Get the velocity of the NavSat sensor in the ENU world frame.
      /// \return Velocity in meters per second
      public: const math::Vector3d &Velocity() const;

      /// \brief Check if there are any subscribers
      /// \return True if there are subscribers, false otherwise
      /// \todo(iche033) Make this function virtual on Garden
      public: bool HasConnections() const;

      /// \brief Easy short hand for setting the position of the sensor.
      /// \param[in] _latitude Latitude angle.
      /// \param[in] _longitude Longitude angle.
      /// \param[in] _altitude Altitude in meters; defaults to zero.
      public: void SetPosition(const math::Angle &_latitude,
          const math::Angle &_longitude, double _altitude = 0.0);

      IGN_COMMON_WARN_IGNORE__DLL_INTERFACE_MISSING
      /// \brief Data pointer for private data
      /// \internal
      private: std::unique_ptr<NavSatPrivate> dataPtr;
      IGN_COMMON_WARN_RESUME__DLL_INTERFACE_MISSING
    };
    }
  }
}

#endif
