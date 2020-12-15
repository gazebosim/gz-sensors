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

#include <ignition/common/PluginLoader.hh>
#include <ignition/common/SystemPaths.hh>
#include <ignition/common/Console.hh>
#include "ignition/sensors/config.hh"

#include "ignition/sensors/SensorFactory.hh"

/// \brief Private data class for SensorFactory
class ignition::sensors::SensorFactoryPrivate
{
  /// \brief Constructor
  public: SensorFactoryPrivate();

  /// \brief A map of loaded sensor plugins and their type.
  public: std::map<std::string, std::shared_ptr<SensorPlugin>> sensorPlugins;

  /// \brief Stores paths to search for on file system
  public: ignition::common::SystemPaths systemPaths;

  /// \brief For loading plugins
  public: ignition::common::PluginLoader pluginLoader;
};

using namespace ignition;
using namespace sensors;

//////////////////////////////////////////////////
SensorFactoryPrivate::SensorFactoryPrivate()
{
  this->systemPaths.AddPluginPaths(IGN_SENSORS_PLUGIN_PATH);
}

//////////////////////////////////////////////////
void SensorFactory::AddPluginPaths(const std::string &_path)
{
  this->dataPtr->systemPaths.AddPluginPaths(_path);
}

//////////////////////////////////////////////////
SensorFactory::SensorFactory() : dataPtr(new SensorFactoryPrivate)
{
}

//////////////////////////////////////////////////
SensorFactory::~SensorFactory()
{
}

//////////////////////////////////////////////////
std::shared_ptr<SensorPlugin> SensorFactory::LoadSensorPlugin(
    const std::string &_filename)
{
  std::string fullPath =
      this->dataPtr->systemPaths.FindSharedLibrary(_filename);
  if (fullPath.empty())
  {
    ignerr << "Unable to find sensor plugin path for [" << _filename << "]\n";
    return std::shared_ptr<SensorPlugin>();
  }

  auto pluginNames = this->dataPtr->pluginLoader.LoadLibrary(fullPath);
  if (pluginNames.empty())
  {
    ignerr << "Unable to load sensor plugin file for [" << fullPath << "]\n";
    return std::shared_ptr<SensorPlugin>();
  }

  // Assume the first plugin is the one we're interested in
  std::string pluginName = *(pluginNames.begin());

  common::PluginPtr pluginPtr =
      this->dataPtr->pluginLoader.Instantiate(pluginName);

  auto sensorPlugin = pluginPtr->QueryInterfaceSharedPtr<
      ignition::sensors::SensorPlugin>();
  return sensorPlugin;
}

/////////////////////////////////////////////////
std::unique_ptr<Sensor> SensorFactory::CreateSensor(const sdf::Sensor &_sdf)
{
  std::unique_ptr<Sensor> result;
  std::shared_ptr<SensorPlugin> sensorPlugin;
  std::string type = _sdf.TypeStr();
  std::string fullPath = IGN_SENSORS_PLUGIN_NAME(type);

  auto it = this->dataPtr->sensorPlugins.find(type);
  if (it != this->dataPtr->sensorPlugins.end())
    sensorPlugin = it->second;
  else
  {
    sensorPlugin = this->LoadSensorPlugin(fullPath);
    if (!sensorPlugin)
    {
      ignerr << "Unable to instantiate sensor plugin for [" << fullPath
        << "]\n";
      return nullptr;
    }

    this->dataPtr->sensorPlugins[type] = sensorPlugin;
  }

  auto sensor = sensorPlugin->New();
  if (!sensor)
  {
    ignerr << "Unable to instantiate sensor for [" << fullPath << "]\n";
    return nullptr;
  }

  if (!sensor->Load(_sdf))
  {
    ignerr << "Sensor::Load failed for plugin [" << fullPath << "]\n";
    return nullptr;
  }

  if (!sensor->Init())
  {
    ignerr << "Sensor::Init failed for plugin [" << fullPath << "]\n";
    return nullptr;
  }

  result.reset(sensor);
  return result;
}

/////////////////////////////////////////////////
std::unique_ptr<Sensor> SensorFactory::CreateSensor(sdf::ElementPtr _sdf)
{
  std::unique_ptr<Sensor> result;
  if (_sdf)
  {
    if (_sdf->GetName() == "sensor")
    {
      std::shared_ptr<SensorPlugin> sensorPlugin;
      std::string type = _sdf->Get<std::string>("type");
      std::string fullPath = IGN_SENSORS_PLUGIN_NAME(type);
      auto it = this->dataPtr->sensorPlugins.find(type);
      if (it != this->dataPtr->sensorPlugins.end())
        sensorPlugin = it->second;
      else
      {
        sensorPlugin = this->LoadSensorPlugin(fullPath);
        if (!sensorPlugin)
        {
          ignerr << "Unable to instantiate sensor plugin for [" << fullPath
                 << "]\n";
          return nullptr;
        }

        this->dataPtr->sensorPlugins[type] = sensorPlugin;
      }

      auto sensor = sensorPlugin->New();
      if (!sensor)
      {
        ignerr << "Unable to instantiate sensor for [" << fullPath << "]\n";
        return nullptr;
      }

      if (!sensor->Load(_sdf))
      {
        ignerr << "Sensor::Load failed for plugin [" << fullPath << "]\n";
        return nullptr;
      }

      if (!sensor->Init())
      {
        ignerr << "Sensor::Init failed for plugin [" << fullPath << "]\n";
        return nullptr;
      }
      result.reset(sensor);
      return result;
    }
    else
    {
      ignerr << "Provided SDF is not a <sensor> element.\n";
    }
  }

  return nullptr;
}
