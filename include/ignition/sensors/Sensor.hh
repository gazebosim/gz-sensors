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
#include <sdf/sdf.hh>

namespace ignition
{
  namespace sensors
  {
    /// \brief A string used to identify a sensor
    using SensorId = std::size_t;
    const SensorId NO_ID = 0;

    /// \brief forwar declaration
    class SensorPrivate;

    /// \brief a base sensor class
    class Sensor
    {
      /// \brief constructor
      public: Sensor();

      /// \brief destructor
      public: virtual ~Sensor();

      /// \brief Initialize values in the sensor
      public: void Init(Sensor *_parent, SensorId _id);

      /// \brief Load the sensor with SDF parameters.
      /// \param[in] _sdf SDF Sensor parameters.
      public: virtual void Load(sdf::ElementPtr _sdf);

      /// \brief Force the sensor to generate data
      /// \param[in] _now The current time
      public: virtual void Update(const common::Time &_now);

      /// \brief Get the parent sensor (The one which loaded us)
      public: Sensor *Parent() const;

      /// \brief Return the next time the sensor will generate data
      public: common::Time NextUpdateTime() const;

      /// \brief Update the sensor.
      /// \param[in] _now The current time
      /// \param[in] _force Force the update to happen even if it's not time
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

      /// \brief Get the sensor's ID.
      /// \return The sensor's ID.
      public: SensorId Id() const;

      /// \internal
      /// \brief Data pointer for private data
      private: std::unique_ptr<SensorPrivate> dataPtr;
    };
  }
}

#endif
