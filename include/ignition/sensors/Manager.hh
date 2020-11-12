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
#include <ignition/common/Console.hh>
#include <ignition/sensors/config.hh>
#include <ignition/sensors/Export.hh>
#include <ignition/sensors/Sensor.hh>

namespace ignition
{
  namespace sensors
  {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_SENSORS_VERSION_NAMESPACE {
    // Forward declarations
    class ManagerPrivate;

    /// \brief Loads and runs sensors
    ///
    ///   This class is responsible for loading and running sensors, and
    ///   providing sensors with common environments to generat data from.
    ///
    ///   The primary interface through which to load a sensor is LoadSensor().
    ///   This takes an sdf element pointer that should be configured with
    ///   everything the sensor will need. Custom sensors configuration must
    ///   be in the <plugin> tag of the sdf::Element. The manager will
    ///   dynamically load the sensor library and update it.
    /// \remarks This class is not thread safe.
    class IGNITION_SENSORS_VISIBLE Manager
    {
      /// \brief constructor
      public: Manager();

      /// \brief destructor
      public: virtual ~Manager();

      /// \brief Initialize the sensor library without rendering or physics.
      /// \return True if successfully initialized, false if not
      public: bool Init();

      /// \brief Create a sensor from SDF with a known sensor type.
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
      /// \return A pointer to the created sensor. nullptr returned on
      /// error.
      public: template<typename T>
              T *CreateSensor(sdf::Sensor _sdf)
              {
                ignition::sensors::SensorId id = this->CreateSensor(_sdf);

                if (id != NO_SENSOR)
                {
                  T *result = dynamic_cast<T*>(this->Sensor(id));

                  if (!result)
                    ignerr << "SDF sensor type does not match template type\n";

                  return result;
                }

                ignerr << "Failed to create sensor of type["
                       << _sdf.TypeStr() << "]\n";
                return nullptr;
              }

      /// \brief Create a sensor from SDF with a known sensor type.
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
      /// \return A pointer to the created sensor. nullptr returned on
      /// error.
      public: template<typename T>
              T *CreateSensor(sdf::ElementPtr _sdf)
              {
                ignition::sensors::SensorId id = this->CreateSensor(_sdf);

                if (id != NO_SENSOR)
                {
                  T *result = dynamic_cast<T*>(this->Sensor(id));

                  if (nullptr == result)
                  {
                    ignerr << "Failed to create sensor [" << id << "] of type ["
                           << _sdf->Get<std::string>("type")
                           << "]. SDF sensor type does not match template type."
                           << std::endl;
                  }

                  return result;
                }

                ignerr << "Failed to create sensor of type ["
                       << _sdf->Get<std::string>("type") << "]\n";
                return nullptr;
              }

      /// \brief Create a sensor from SDF without a known sensor type.
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
      /// \return A sensor id that refers to the created sensor. NO_SENSOR
      /// is returned on erro.
      public: ignition::sensors::SensorId CreateSensor(sdf::ElementPtr _sdf);

      /// \brief Create a sensor from SDF without a known sensor type.
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
      /// \param[in] _sdf SDF sensor DOM object
      /// \return A sensor id that refers to the created sensor. NO_SENSOR
      /// is returned on erro.
      public: ignition::sensors::SensorId CreateSensor(const sdf::Sensor &_sdf);


      /// \brief Get an instance of a loaded sensor by sensor id
      /// \param[in] _id Idenitifier of the sensor.
      /// \return Pointer to the sensor, nullptr on error.
      public: ignition::sensors::Sensor *Sensor(
                  ignition::sensors::SensorId _id);

      /// \brief Remove a sensor by ID
      /// \param[in] _sensorId ID of the sensor to remove
      /// \return True if the sensor exists and removed.
      public: bool Remove(const ignition::sensors::SensorId _id);

      /// \brief Run the sensor generation one step.
      /// \param _time: The current simulated time
      /// \param _force: If true, all sensors are forced to update. Otherwise
      ///        a sensor will update based on it's Hz rate.
      public: void IGN_DEPRECATED(4) RunOnce(
        const ignition::common::Time &_time, bool _force = false);

      /// \brief Run the sensor generation one step.
      /// \param _time: The current simulated time
      /// \param _force: If true, all sensors are forced to update. Otherwise
      ///        a sensor will update based on it's Hz rate.
      public: void RunOnce(const std::chrono::steady_clock::duration &_time,
                  bool _force = false);

      /// \brief Adds colon delimited paths sensor plugins may be
      public: void AddPluginPaths(const std::string &_path);

      /// \brief load a plugin and return a shared_ptr
      /// \param[in] _filename Sensor plugin file to load.
      /// \return Pointer to the new sensor, nullptr on error.
      private: ignition::sensors::SensorId LoadSensorPlugin(
                   const std::string &_filename, sdf::ElementPtr _sdf);

      /// \brief private data pointer
      private: std::unique_ptr<ManagerPrivate> dataPtr;
    };
    }
  }
}

#endif
