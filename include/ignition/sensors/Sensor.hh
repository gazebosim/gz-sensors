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
#ifndef IGNITION_SENSORS_SENSOR_HH_
#define IGNITION_SENSORS_SENSOR_HH_

#include <memory>

#include <ignition/common/Time.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/sensors/ign_sensors_export.h>
#include <sdf/sdf.hh>

namespace ignition
{
  namespace sensors
  {
    /// \brief A string used to identify a sensor
    using SensorId = std::size_t;
    const SensorId NO_SENSOR = 0;

    /// \brief forwar declarations
    class SensorPrivate;
    class Manager;

    /// \brief a base sensor class
    /// \description This class common tasks like timing updates the sensor and
    ///              parsing the `<sensor>` block in sdformat, and loading
    ///              plugins to do the actual work
    class IGN_SENSORS_EXPORT Sensor
    {
      friend ignition::sensors::Manager;

      /// \brief constructor
      protected: Sensor();

      /// \brief destructor
      public: ~Sensor();

      /// \brief Initialize values in the sensor
      public: void Init(ignition::sensors::Manager *_mgr, SensorId _id);

      /// \brief Load the sensor with SDF parameters.
      /// \param[in] _sdf SDF Sensor parameters.
      /// \return true if loading was successful
      public: bool Load(sdf::ElementPtr _sdf);

      /// \brief Return the next time the sensor will generate data
      public: common::Time NextUpdateTime() const;

      /// \brief Update the sensor.
      /// \param[in] _now The current time
      /// \param[in] _force Force the update to happen even if it's not time
      /// \remarks if forced the NextUpdateTime() will be unchanged
      public: void Update(const common::Time &_now, const bool _force);

      /// \brief Get the update rate of the sensor.
      /// \return _hz update rate of sensor.
      public: double UpdateRate() const;

      /// \brief Set the update rate of the sensor.
      /// \param[in] _hz update rate of sensor.
      public: void SetUpdateRate(const double _hz);

      /// \brief Get the current pose.
      /// \return Current pose of the sensor.
      public: const ignition::math::Pose3d &Pose() const;

      /// \brief Update the pose of the sensor
      public: const void SetPose(const ignition::math::Pose3d &_pose);

      /// \brief Get name.
      /// \return Name of sensor.
      public: const std::string &Name() const;

      /// \brief Get topic
      /// \return Topic sensor publishes data to
      public: const std::string &Topic() const;

      /// \brief Get the sensor's ID.
      /// \return The sensor's ID.
      public: SensorId Id() const;

      /// \brief Get the sensor manager
      public: ignition::sensors::Manager *Manager() const;

      /// \internal
      /// \brief Data pointer for private data
      private: std::shared_ptr<SensorPrivate> dataPtr;
    };
  }
}

#endif
