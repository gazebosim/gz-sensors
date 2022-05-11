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
#ifndef GZ_SENSORS_SENSORFACTORY_HH_
#define GZ_SENSORS_SENSORFACTORY_HH_

#include <memory>
#include <string>
#include <type_traits>
#include <sdf/sdf.hh>

#include <gz/common/Console.hh>
#include <gz/utils/SuppressWarning.hh>

#include <gz/sensors/config.hh>
#include <gz/sensors/Export.hh>

#include "gz/sensors/Sensor.hh"

namespace ignition
{
  namespace sensors
  {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_SENSORS_VERSION_NAMESPACE {
    // forward declaration
    class SensorFactoryPrivate;

    /// \brief Base sensor plugin interface
    /// \deprecated Sensor plugins are deprecated. Instantiate sensor objects
    /// instead.
    class IGNITION_SENSORS_VISIBLE SensorPlugin
    {
      /// \brief Instantiate new sensor
      /// \return New sensor
      public: virtual Sensor IGN_DEPRECATED(6) * New() = 0;
    };

    /// \brief Templated class for instantiating sensors of the specified type
    /// \tparam Type of sensor being instantiated.
    /// \deprecated Sensor plugins are deprecated. Instantiate sensor objects
    /// instead.
    template<class SensorType>
    class SensorTypePlugin : public SensorPlugin
    {
      // Documentation inherited
      public: SensorType IGN_DEPRECATED(6) * New() override
              {
                return new SensorType();
              };
    };

    /// \brief A factory class for creating sensors
    /// This class instantiates sensor objects based on the sensor type and
    /// makes sure they're initialized correctly.
    // After removing plugin functionality, the sensor factory class doesn't
    // hold any internal state. Consider converting the functionality in this
    // class to helper functions.
    class IGNITION_SENSORS_VISIBLE SensorFactory
    {
      /// \brief Constructor
      public: SensorFactory();

      /// \brief Destructor
      public: ~SensorFactory();

      /// \brief Create a sensor from a SDF DOM object with a known sensor type.
      /// \sa Sensor()
      /// \param[in] _sdf SDF Sensor DOM object.
      /// \tparam SensorType Sensor type
      /// \return A pointer to the created sensor. Null returned on error.
      public: template<typename SensorType>
              std::unique_ptr<SensorType> CreateSensor(const sdf::Sensor &_sdf)
              {
                auto sensor = std::make_unique<SensorType>();

                if (nullptr == sensor)
                {
                  ignerr << "Failed to create sensor [" << _sdf.Name()
                         << "] of type[" << _sdf.TypeStr() << "]" << std::endl;
                  return nullptr;
                }

                if (!sensor->Load(_sdf))
                {
                  ignerr << "Failed to load sensor [" << _sdf.Name()
                         << "] of type[" << _sdf.TypeStr() << "]" << std::endl;
                  return nullptr;
                }

                if (!sensor->Init())
                {
                  ignerr << "Failed to initialize sensor [" << _sdf.Name()
                         << "] of type[" << _sdf.TypeStr() << "]" << std::endl;
                  return nullptr;
                }

                return sensor;
              }

      /// \brief Create a sensor from an SDF element with a known sensor type.
      /// \sa Sensor()
      /// \param[in] _sdf pointer to the sdf element
      /// \tparam SensorType Sensor type
      /// \return A pointer to the created sensor. Null returned on
      /// error.
      public: template<typename SensorType>
              std::unique_ptr<SensorType> CreateSensor(sdf::ElementPtr _sdf)
              {
                if (nullptr == _sdf)
                {
                  ignerr << "Failed to create sensor, received null SDF "
                         << "pointer." << std::endl;
                  return nullptr;
                }

                auto sensor = std::make_unique<SensorType>();

                auto type = _sdf->Get<std::string>("type");
                auto name = _sdf->Get<std::string>("name");

                if (nullptr == sensor)
                {
                  ignerr << "Failed to create sensor [" << name
                         << "] of type[" << type << "]" << std::endl;
                  return nullptr;
                }

                if (!sensor->Load(_sdf))
                {
                  ignerr << "Failed to load sensor [" << name
                         << "] of type[" << type << "]" << std::endl;
                  return nullptr;
                }

                if (!sensor->Init())
                {
                  ignerr << "Failed to initialize sensor [" << name
                         << "] of type[" << type << "]" << std::endl;
                  return nullptr;
                }

                return sensor;
              }

      /// \brief Create a sensor from SDF without a known sensor type.
      /// \sa Sensor()
      /// \param[in] _sdf pointer to the sdf element
      /// \return Null, as the function is deprecated.
      /// \deprecated Sensor registration is deprecated, so it's necessary to
      /// provide the specific sensor type to create it. Use the templated
      /// `CreateSensor` function.
      public: std::unique_ptr<Sensor> IGN_DEPRECATED(6) CreateSensor(
          sdf::ElementPtr _sdf);

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
      /// \deprecated Sensor registration is deprecated, so it's necessary to
      /// provide the specific sensor type to create it. Use the templated
      /// `CreateSensor` function.
      public: std::unique_ptr<Sensor> IGN_DEPRECATED(6) CreateSensor(
          const sdf::Sensor &_sdf);

      /// \brief Add additional path to search for sensor plugins
      /// \param[in] _path Search path
      /// \deprecated Sensor plugins aren't supported anymore.
      public: void IGN_DEPRECATED(6) AddPluginPaths(const std::string &_path);

      IGN_UTILS_WARN_IGNORE__DLL_INTERFACE_MISSING
      /// \brief private data pointer
      private: std::unique_ptr<SensorFactoryPrivate> dataPtr;
      IGN_UTILS_WARN_RESUME__DLL_INTERFACE_MISSING
    };
    }
  }
}

#endif
