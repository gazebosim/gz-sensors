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
#include <string>
#include <type_traits>
#include <vector>
#include <sdf/sdf.hh>
#include <ignition/common/Time.hh>
#include <ignition/rendering/Scene.hh>
#include <ignition/sensors/Sensor.hh>
#include <ignition/sensors/ign_sensors_export.hh>

namespace ignition
{
  namespace physics
  {
    // Forward declarations
    class Manager;
  }

  namespace sensors
  {
    // Forward declarations
    class ManagerPrivate;

    /// \brief Loads and runs sensors
    ///
    ///   This class is responsible for loading and running sensors, and
    ///   providing sensors with common environments to generat data from. For
    ///   example, sensors that need information about the visible world would
    ///   access the common igntion::rendering::ScenePtr. A user of this class
    ///   is expected to popluate that scene and give it to the manager.
    ///
    ///   The primary interface through which to load a sensor is LoadSensor().
    ///   This takes an sdf element pointer that should be configured with
    ///   everything the sensor will need. Custom sensors configuration must
    ///   be in the <plugin> tag of the sdf::Element. The manager will
    ///   dynamically load the sensor library and update it.
    /// \remarks This class is not thread safe.
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
      public: bool Init(ignition::rendering::ScenePtr _rendering);

      /// \brief Set or change the ignition-rendering instance used
      public: void SetRenderingScene(ignition::rendering::ScenePtr _rendering);

      /// \brief Get the rendering manager instance
      public: ignition::rendering::ScenePtr RenderingScene() const;

      /// \brief Create a sensor from SDF
      ///
      ///   This creates sensors by looking at the given sdf element.
      ///   Sensors created with this API offer an ignition-transport interface.
      ///   If you need a direct C++ interface to the data, you must get the
      ///   sensor pointer and cast to the correct type.
      ///
      ///   A <sensor> tag may have multiple <plugin> tags. A SensorId will be
      ///   returned for each plugin that is described in SDF.
      ///   If there are no <plugin> tags then one of the plugins shipped with
      ///   this library will be loaded. For example, a <sensor> tag with
      ///   <camera> but no <plugin> will load a CameraSensor from
      ///   ignition-sensors-camera.
      /// \sa Sensor()
      /// \param[in] _sdf pointer to the sdf element
      public: std::vector<ignition::sensors::SensorId> LoadSensor(
          sdf::ElementPtr &_sdf);

      /// \brief Get an instance of a loaded sensor by sensor id
      public: std::shared_ptr<ignition::sensors::Sensor> Sensor(
          ignition::sensors::SensorId);

      /// \brief Remove a sensor by ID
      /// \param[in] _sensorId ID of the sensor to remove
      /// \return True if the sensor exists and removed.
      public: bool Remove(const ignition::sensors::SensorId _id);

      /// \brief Run the sensor generation one step.
      /// \param _time: The current simulated time
      /// \param _force: If true, all sensors are forced to update. Otherwise
      ///        a sensor will update based on it's Hz rate.
      public: void RunOnce(const ignition::common::Time &_time,
                  bool _force = false);

      /// \brief Adds colon delimited paths sensor plugins may be
      public: void AddPluginPaths(const std::string &_path);

      /// \brief Get a sensor id by name
      ///
      /// The given name should follow the standard URI naming scheme.
      /// \param[in] _name URI name of the sensor.
      /// \return Pointer to the sensor, nullptr if the sensor could not be
      ///   found.
      public: ignition::sensors::SensorId SensorId(const std::string &_name);

      /// \brief private data pointer
      private: std::unique_ptr<ManagerPrivate> dataPtr;
    };
  }
}

#endif
