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
#ifndef GZ_SENSORS_MANAGER_HH_
#define GZ_SENSORS_MANAGER_HH_

#include <memory>
#include <string>
#include <utility>
#include <type_traits>
#include <vector>
#include <sdf/sdf.hh>
#include <gz/utils/SuppressWarning.hh>
#include <gz/common/Console.hh>
#include <gz/sensors/config.hh>
#include <gz/sensors/Export.hh>
#include <gz/sensors/Sensor.hh>
#include <gz/sensors/SensorFactory.hh>

namespace gz
{
  namespace sensors
  {
    // Inline bracket to help doxygen filtering.
    inline namespace GZ_SENSORS_VERSION_NAMESPACE {
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
    class GZ_SENSORS_VISIBLE Manager
    {
      /// \brief constructor
      public: Manager();

      /// \brief destructor
      public: virtual ~Manager();

      /// \brief Initialize the sensor library without rendering or physics.
      /// \return True if successfully initialized, false if not
      public: bool Init();

      /// \brief Create a sensor from an SDF ovject with a known sensor type.
      /// \sa Sensor()
      /// \param[in] _sdf An SDF element or DOM object.
      /// \tparam SensorType Sensor type
      /// \tparam SdfType It may be an `sdf::ElementPtr` containing a sensor or
      /// an `sdf::Sensor`.
      /// \return A pointer to the created sensor. Null returned on
      /// error. The Manager keeps ownership of the pointer's lifetime.
      public: template<typename SensorType, typename SdfType>
              SensorType *CreateSensor(SdfType _sdf)
              {
                SensorFactory sensorFactory;
                auto sensor = sensorFactory.CreateSensor<SensorType>(_sdf);
                if (nullptr == sensor)
                {
                  ignerr << "Failed to create sensor." << std::endl;
                  return nullptr;
                }
                auto result = sensor.get();
                if (NO_SENSOR == this->AddSensor(std::move(sensor)))
                {
                  ignerr << "Failed to add sensor." << std::endl;
                  return nullptr;
                }
                return result;
              }

      /// \brief Create a sensor from SDF without a known sensor type.
      /// \sa Sensor()
      /// \param[in] _sdf pointer to the sdf element
      /// \return A sensor id that refers to the created sensor. NO_SENSOR
      /// is returned on erro.
      /// \deprecated Sensor registration is deprecated, so it's necessary to
      /// provide the specific sensor type to create it. Use the templated
      /// `CreateSensor` function.
      public: gz::sensors::SensorId GZ_DEPRECATED(6) CreateSensor(
          sdf::ElementPtr _sdf);

      /// \brief Create a sensor from SDF without a known sensor type.
      /// \sa Sensor()
      /// \param[in] _sdf SDF sensor DOM object
      /// \return A sensor id that refers to the created sensor. NO_SENSOR
      /// is returned on erro.
      /// \deprecated Sensor registration is deprecated, so it's necessary to
      /// provide the specific sensor type to create it. Use the templated
      /// `CreateSensor` function.
      public: gz::sensors::SensorId GZ_DEPRECATED(6) CreateSensor(
          const sdf::Sensor &_sdf);

      /// \brief Add a sensor for this manager to manage.
      /// \sa Sensor()
      /// \param[in] _sensor Pointer to the sensor
      /// \return A sensor id that refers to the created sensor. NO_SENSOR
      /// is returned on error.
      public: SensorId AddSensor(std::unique_ptr<Sensor> _sensor);

      /// \brief Get an instance of a loaded sensor by sensor id
      /// \param[in] _id Idenitifier of the sensor.
      /// \return Pointer to the sensor, nullptr on error.
      public: gz::sensors::Sensor *Sensor(
                  gz::sensors::SensorId _id);

      /// \brief Remove a sensor by ID
      /// \param[in] _sensorId ID of the sensor to remove
      /// \return True if the sensor exists and removed.
      public: bool Remove(const gz::sensors::SensorId _id);

      /// \brief Run the sensor generation one step.
      /// \param _time: The current simulated time
      /// \param _force: If true, all sensors are forced to update. Otherwise
      ///        a sensor will update based on it's Hz rate.
      public: void RunOnce(const std::chrono::steady_clock::duration &_time,
                  bool _force = false);

      /// \brief Adds colon delimited paths sensor plugins may be
      public: void GZ_DEPRECATED(6) AddPluginPaths(const std::string &_path);

      IGN_UTILS_WARN_IGNORE__DLL_INTERFACE_MISSING
      /// \brief private data pointer
      private: std::unique_ptr<ManagerPrivate> dataPtr;
      IGN_UTILS_WARN_RESUME__DLL_INTERFACE_MISSING
    };
    }
  }
}

#endif
