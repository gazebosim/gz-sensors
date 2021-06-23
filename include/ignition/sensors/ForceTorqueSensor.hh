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

#ifndef IGNITION_SENSORS_FORCETORQUESENSOR_HH_
#define IGNITION_SENSORS_FORCETORQUESENSOR_HH_

#include <memory>

#include <sdf/sdf.hh>

#include <ignition/common/SuppressWarning.hh>
#include <ignition/common/Time.hh>
#include <ignition/math/Pose3.hh>

#include <ignition/sensors/config.hh>

#include <ignition/sensors/force_torque/Export.hh>

#include "ignition/sensors/Sensor.hh"

namespace ignition
{
  namespace sensors
  {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_SENSORS_VERSION_NAMESPACE {
    //
    /// \brief forward declarations
    class ForceTorqueSensorPrivate;

    /// \brief Force Torque Sensor Class
    ///
    /// A force-torque Sensor that reports force and torque applied on a joint.

    class IGNITION_SENSORS_FORCE_TORQUE_VISIBLE ForceTorqueSensor : public Sensor
    {
      /// \brief constructor
      public: ForceTorqueSensor();

      /// \brief destructor
      public: virtual ~ForceTorqueSensor();

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
      public: virtual bool IGN_DEPRECATED(4) Update(
        const ignition::common::Time &_now) override;

      /// \brief Update the sensor and generate data
      /// \param[in] _now The current time
      /// \return true if the update was successfull
      public: virtual bool Update(
        const std::chrono::steady_clock::duration &_now) override;

      /// \brief Get the current joint force.
      /// \return The latested measured force.
      public: math::Vector3d Force() const;

      /// \brief Set the force vector.
      /// \param[in] _force force vector in newton.
      public: void SetForce(const math::Vector3d &_force);

      /// \brief Get the current joint torque.
      /// \return The latested measured torque.
      public: math::Vector3d Torque() const;

      /// \brief Set the torque vector.
      /// \param[in] _torque torque vector in newton.
      public: void SetTorque(const math::Vector3d &_torque);
    

      IGN_COMMON_WARN_IGNORE__DLL_INTERFACE_MISSING
      /// \brief Data pointer for private data
      /// \internal
      private: std::unique_ptr<ForceTorqueSensorPrivate> dataPtr;
      IGN_COMMON_WARN_RESUME__DLL_INTERFACE_MISSING
    };
    }
  }
}

#endif
