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
#ifndef IGNITION_SENSORS_IMUSENSOR_HH_
#define IGNITION_SENSORS_IMUSENSOR_HH_

#include <memory>

#include <sdf/sdf.hh>

#include <ignition/common/SuppressWarning.hh>
#include <ignition/common/Time.hh>
#include <ignition/math/Pose3.hh>

#include <ignition/sensors/config.hh>
#include <ignition/sensors/imu/Export.hh>

#include "ignition/sensors/Sensor.hh"

namespace ignition
{
  namespace sensors
  {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_SENSORS_VERSION_NAMESPACE {
    //
    /// \brief forward declarations
    class ImuSensorPrivate;

    /// \brief Imu Sensor Class
    ///
    /// An imu sensor that reports linear acceleration, angular velocity, and
    /// orientation
    class IGNITION_SENSORS_IMU_VISIBLE ImuSensor : public Sensor
    {
      /// \brief constructor
      public: ImuSensor();

      /// \brief destructor
      public: virtual ~ImuSensor();

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

      /// \brief Set the angular velocity of the imu
      /// \param[in] _angularVel Angular velocity of the imu in body frame
      /// expressed in radians per second
      public: void SetAngularVelocity(const math::Vector3d &_angularVel);

      /// \brief Get the angular velocity of the imu
      /// \return Angular velocity of the imu in body frame, expressed in
      /// radians per second.
      public: math::Vector3d AngularVelocity() const;

      /// \brief Set the linear acceleration of the imu
      /// \param[in] _linearAcc Linear accceleration of the imu in body frame
      /// expressed in meters per second squared.
      public: void SetLinearAcceleration(const math::Vector3d &_linearAcc);

      /// \brief Get the linear acceleration of the imu
      /// \return Linear acceleration of the imu in local frame, expressed in
      /// meters per second squared.
      public: math::Vector3d LinearAcceleration() const;

      /// \brief Set the world pose of the imu
      /// \param[in] _pose Pose in world frame
      public: void SetWorldPose(const math::Pose3d _pose);

      /// \brief Get the world pose of the imu
      /// \return Pose in world frame.
      public: math::Pose3d WorldPose() const;

      /// \brief Set the orientation reference, i.e. initial imu
      /// orientation. Imu orientation data generated will be relative to this
      /// reference frame.
      /// \param[in] _orientation Reference orientation
      public: void SetOrientationReference(
          const math::Quaterniond &_orient);

      /// \brief Get the world orienation reference of the imu
      /// \return Orientation reference in world frame
      public: math::Quaterniond OrientationReference() const;

      /// \brief Get the orienation of the imu with respect to reference frame
      /// \return Orientation in reference frame
      public: math::Quaterniond Orientation() const;

      /// \brief Set the gravity vector
      /// \param[in] _gravity gravity vector in meters per second squared.
      public: void SetGravity(const math::Vector3d &_gravity);

      /// \brief Get the gravity vector
      /// \return Gravity vectory in meters per second squared.
      public: math::Vector3d Gravity() const;

      IGN_COMMON_WARN_IGNORE__DLL_INTERFACE_MISSING
      /// \brief Data pointer for private data
      /// \internal
      private: std::unique_ptr<ImuSensorPrivate> dataPtr;
      IGN_COMMON_WARN_RESUME__DLL_INTERFACE_MISSING
    };
    }
  }
}

#endif
