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
#ifndef IGNITION_SENSORS_MANAGER_HH_
#define IGNITION_SENSORS_MANAGER_HH_

#include <memory>
#include <ignition/common/Time.hh>
#include <ignition/sensors/Sensor.hh>
#include <ignition/sensors/ign_sensors_export.h>
#include <sdf/sdf.hh>


namespace ignition
{
  namespace rendering
  {
    // Forward declarations
    class Manager;
  }

  namespace physics
  {
    // Forward declarations
    class Manager;
  }

  namespace sensors
  {
    // Forward declarations
    class ManagerPrivate;

    /// \brief Responsible for running sensors
    class IGN_SENSORS_EXPORT Manager
    {
      /// \brief constructor
      public: Manager();

      /// \brief destructor
      public: virtual ~Manager();

      /// \brief Initialize the sensor library without rendering or physics.
      /// \return True if successfully initialized, false if not
      public: bool Init();

      /// \brief Initialize the sensor library with rendering.
      /// \return True if successfully initialized, false if not
      public: bool Init(ignition::rendering::Manager &_rendering);

      /// \brief Set or change the ignition-rendering instance used
      public: void SetRendering(ignition::rendering::Manager &_rendering);

      /// \brief Create a sensor from SDF
      /// \param[in] _sdf pointer to the sdf element
      public: SensorId LoadSensor(sdf::ElementPtr &_sdf);

      /// \brief Remove a sensor by ID
      /// \param[in] _sensorId ID of the sensor to remove
      public: void Remove(const SensorId _id);

      /// \brief Remove a sensor by name
      /// \param[in] _name ID of the sensor to remove
      public: void Remove(const std::string &_name);

      /// \brief Run the sensor generation one step.
      /// \param _time: The current simulated time
      /// \param _force: If true, all sensors are forced to update. Otherwise
      ///        a sensor will update based on it's Hz rate.
      public: void RunOnce(const ignition::common::Time &_time,
                  bool _force = false);

      /// \brief Get a sensor using by name
      ///
      /// The given name should follow the standard URI naming scheme.
      /// \param[in] _name URI name of the sensor.
      /// If the name is not unique a nullptr pointer is returned.
      /// \return Pointer to the sensor, nullptr if the sensor could not be
      /// found.
      public: SensorId Sensor(const std::string &_name);

      /// \brief private data pointer
      private: std::unique_ptr<ManagerPrivate> dataPtr;
    };
  }
}

#endif
