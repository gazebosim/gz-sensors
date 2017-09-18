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
#include <string>

#include <ignition/common/Time.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/sensors/ign_sensors_export.hh>
#include <ignition/sensors/PluginIface.hh>
#include <sdf/sdf.hh>

namespace ignition
{
  namespace sensors
  {
    /// \brief A string used to identify a sensor
    using SensorId = std::size_t;
    const SensorId NO_SENSOR = 0;

    /// \brief forward declarations
    class SensorPrivate;
    class SensorPlugin;

    /// \brief a base sensor class
    /// \description This class is capable of loading any sensor plugin at the
    ///              cost of not offering direct c++ access to the sensor data.
    ///              It finds and loads the appropriate sensor plugin according
    ///              to the given sdformat element pointer. An ignition
    ///              transport subscription must be used to get the sensor data.
    ///              If direct access is required and you know the type of the
    ///              sensor then one of the derived sensor classes should be
    ///              used instead.
    class IGN_SENSORS_EXPORT Sensor
    {
      /// \brief constructor
      protected: Sensor();

      /// \brief destructor
      public: virtual ~Sensor();

      public: void SetIface(std::shared_ptr<PluginIface> _iface);

      public: std::shared_ptr<PluginIface> Iface() const;

      /// \brief Load the sensor with SDF parameters.
      /// \param[in] _sdf SDF <sensor> or <plugin> inside of <sensor>
      /// \return true if loading was successful
      public: virtual bool Load(sdf::ElementPtr _sdf);

      /// \brief Initialize values in the sensor
      public: virtual bool Init();

      /// \brief Force the sensor to generate data
      /// \param[in] _now The current time
      /// \return true if the update was successfull
      public: virtual bool Update(const common::Time &_now) = 0;

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

      /// \brief Get the SDF used to load this sensor.
      /// \return Pointer to an SDF element that contains initialization
      /// information for this sensor.
      public: sdf::ElementPtr SDF() const;

      /// \internal
      /// \brief Data pointer for private data
      private: std::shared_ptr<SensorPrivate> dataPtr;
    };
  }
}

#endif
