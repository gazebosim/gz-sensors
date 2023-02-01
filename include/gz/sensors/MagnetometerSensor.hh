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
#ifndef GZ_SENSORS_MAGNETOMETERSENSOR_HH_
#define GZ_SENSORS_MAGNETOMETERSENSOR_HH_

#include <memory>

#include <sdf/sdf.hh>

#include <gz/common/SuppressWarning.hh>
#include <gz/math/Pose3.hh>

#include <gz/sensors/config.hh>
#include <gz/sensors/magnetometer/Export.hh>

#include "gz/sensors/Sensor.hh"

namespace ignition
{
  namespace sensors
  {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_SENSORS_VERSION_NAMESPACE {
    //
    /// \brief forward declarations
    class MagnetometerSensorPrivate;

    /// \brief Magnetometer Sensor Class
    ///
    /// A magnetometer reports the magnetic field vector
    class IGNITION_SENSORS_MAGNETOMETER_VISIBLE MagnetometerSensor
        : public Sensor
    {
      /// \brief constructor
      public: MagnetometerSensor();

      /// \brief destructor
      public: virtual ~MagnetometerSensor();

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

      /// \brief Set the world pose of the sensor
      /// \param[in] _pose Pose in world frame
      public: void SetWorldPose(const math::Pose3d _pose);

      /// \brief Get the world pose of the sensor
      /// \return Pose in world frame.
      public: math::Pose3d WorldPose() const;

      /// \brief Set the magnetic field vector in world frame
      /// \param[in] _field Magnetic field vector in world frame.
      public: void SetWorldMagneticField(const math::Vector3d &_field);

      /// \brief Get the magnetic field vector in world frame
      /// \return Magnetic field vector in world frame
      public: math::Vector3d WorldMagneticField() const;

      /// \brief Get the magnetic field vector in body frame
      /// \return Magnetic field vector in body frame
      public: math::Vector3d MagneticField() const;

      /// \brief Check if there are any subscribers
      /// \return True if there are subscribers, false otherwise
      /// \todo(iche033) Make this function virtual on Garden
      public: bool HasConnections() const;

      IGN_COMMON_WARN_IGNORE__DLL_INTERFACE_MISSING
      /// \brief Data pointer for private data
      /// \internal
      private: std::unique_ptr<MagnetometerSensorPrivate> dataPtr;
      IGN_COMMON_WARN_RESUME__DLL_INTERFACE_MISSING
    };
    }
  }
}

#endif
