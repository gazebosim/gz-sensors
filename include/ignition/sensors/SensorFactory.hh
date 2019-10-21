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
#ifndef IGNITION_SENSORS_SENSORFACTORY_HH_
#define IGNITION_SENSORS_SENSORFACTORY_HH_

#include <memory>
#include <string>
#include <type_traits>
#include <sdf/sdf.hh>

#include <ignition/common/Console.hh>

#include <ignition/sensors/config.hh>
#include <ignition/sensors/Export.hh>

#include "ignition/sensors/Sensor.hh"
#include "ignition/sensors/SensorPlugin.hh"

namespace ignition
{
  namespace sensors
  {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_SENSORS_VERSION_NAMESPACE {
    // forward declaration
    class SensorFactoryPrivate;

    /// \def Sensor
    /// \brief Prototype for sensor factory functions
    typedef Sensor *(*SensorFactoryFn) ();

    /// \brief A factory class for creating sensors
    /// This class wll load a sensor plugin based on the given sensor type and
    ///  instantiates a sensor object
    ///
    class IGNITION_SENSORS_VISIBLE SensorFactory
    {
      /// \brief Constructor
      public: SensorFactory();

      /// \brief Destructor
      public: ~SensorFactory();

      /// \brief Create a sensor from a SDF DOM object with a known sensor type.
      ///
      ///   This creates sensors by looking at the given SDF DOM object.
      ///   Sensors created with this API offer an ignition-transport interface.
      ///   If you need a direct C++ interface to the data, you must get the
      ///   sensor pointer and cast to the correct type.
      ///
      /// \sa Sensor()
      /// \param[in] _sdf SDF Sensor DOM object.
      /// \return A pointer to the created sensor. nullptr returned on
      /// error.
      public: template<typename T>
              std::unique_ptr<T> CreateSensor(const sdf::Sensor &_sdf)
              {
                auto sensor = SensorFactory::CreateSensor(_sdf);

                if (sensor)
                {
                  std::unique_ptr<T> result(
                      dynamic_cast<T *>(sensor.release()));

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
      /// \sa Sensor()
      /// \param[in] _sdf pointer to the sdf element
      /// \return A pointer to the created sensor. nullptr returned on
      /// error.
      public: template<typename T>
              std::unique_ptr<T> CreateSensor(sdf::ElementPtr _sdf)
              {
                auto sensor = SensorFactory::CreateSensor(_sdf);

                if (sensor)
                {
                  std::unique_ptr<T> result(
                      dynamic_cast<T *>(sensor.release()));

                  if (!result)
                    ignerr << "SDF sensor type does not match template type\n";

                  return result;
                }

                ignerr << "Failed to create sensor of type["
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
      /// \sa Sensor()
      /// \param[in] _sdf pointer to the sdf element
      /// \return A sensor id that refers to the created sensor. Null is
      /// is returned on error.
      public: std::unique_ptr<Sensor> CreateSensor(sdf::ElementPtr _sdf);

      /// \brief Create a sensor from an SDF Sensor DOM object without a known
      /// sensor type.
      ///
      ///   This creates sensors by looking at the given SDF Sensor DOM
      ///   object.
      ///   Sensors created with this API offer an ignition-transport interface.
      ///   If you need a direct C++ interface to the data, you must get the
      ///   sensor pointer and cast to the correct type.
      ///
      /// \sa Sensor()
      /// \param[in] _sdf SDF Sensor DOM object.
      /// \return A sensor id that refers to the created sensor. Null is
      /// is returned on error.
      public: std::unique_ptr<Sensor> CreateSensor(const sdf::Sensor &_sdf);

      /// \brief Add additional path to search for sensor plugins
      /// \param[in] _path Search path
      public: void AddPluginPaths(const std::string &_path);

      /// \brief Register a sensor class (called by sensor registration function).
      /// \param[in] _sensorType Name of sensor type to register.
      /// \param[in] _factoryfn Function handle for registration.
      public: static void RegisterSensor(const std::string &_sensorType,
                                         //SensorFactoryFn _factoryfn);
                                         SensorInterface *_interface);

      /// \brief load a plugin and return a pointer
      /// \param[in] _filename Sensor plugin file to load.
      /// \return Pointer to the new sensor, nullptr on error.
      private: std::shared_ptr<SensorInterface> LoadSensorPlugin(
          const std::string &_filename);

      /// \brief A list of registered sensor types
      // private: static std::map<std::string, SensorFactoryFn> sensorMap;
      private: static std::map<std::string, SensorInterface *> sensorMap;

      /// \brief private data pointer
      private: std::unique_ptr<SensorFactoryPrivate> dataPtr;
    };
    }
  }
}

#endif
