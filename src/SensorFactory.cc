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

#include <dlfcn.h>

#include <ignition/common/SystemPaths.hh>
#include <ignition/common/Console.hh>
#include <ignition/plugin/Loader.hh>
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
    const std::string &_type)
{
  auto filename = IGN_SENSORS_PLUGIN_NAME(_type);

  std::string fullPath =
      this->dataPtr->systemPaths.FindSharedLibrary(filename);
  if (fullPath.empty())
  {
    ignerr << "Unable to find sensor plugin path for [" << filename << "]\n";
    return std::shared_ptr<SensorPlugin>();
  }

  plugin::Loader pluginLoader;
  pluginLoader.SetFlags(RTLD_LAZY | RTLD_GLOBAL);
  auto pluginNames = pluginLoader.LoadLib(fullPath);

  if (pluginNames.empty())
  {
    ignerr << "Failed to load plugin [" << filename <<
              "] : couldn't load library on path [" << fullPath <<
              "]." << std::endl;
    return std::shared_ptr<SensorPlugin>();
  }

  std::string pluginName;
  bool capitalize{true};

  // snake_case to CamelCase
  for (const auto &c : _type)
  {
    if (c == '_')
    {
      capitalize = true;
      continue;
    }

    if (!std::isalpha(c))
      continue;

    if (!capitalize)
    {
      pluginName += c;
    }
    else
    {
      pluginName += std::toupper(c);
      capitalize = false;
    }
  }

  auto ns = "ignition::sensors::v"
      + std::to_string(IGNITION_SENSORS_MAJOR_VERSION) + "::";

  pluginName = ns + "SensorTypePlugin<" + ns + pluginName + "Sensor>";

  auto plugin = pluginLoader.Instantiate(pluginName);
  if (!plugin)
  {
    std::stringstream error;
    error << "Failed to instantiate plugin [" << pluginName << "] from ["
           << fullPath << "]. Available plugins:" << std::endl;
    for (auto name : pluginNames)
    {
      error << "- " << name << std::endl;
    }
    ignerr << error.str();

    return nullptr;
  }

  auto sensorPlugin =
      plugin->QueryInterfaceSharedPtr<ignition::sensors::SensorPlugin>();

  if (!sensorPlugin)
  {
    ignerr << "Failed to query interface from [" << pluginName << "]"
           << std::endl;
    return nullptr;
  }

  return sensorPlugin;
}

/////////////////////////////////////////////////
std::unique_ptr<Sensor> SensorFactory::CreateSensor(const sdf::Sensor &_sdf)
{
  std::unique_ptr<Sensor> result;
  std::shared_ptr<SensorPlugin> sensorPlugin;
  std::string type = _sdf.TypeStr();

  auto it = this->dataPtr->sensorPlugins.find(type);
  if (it != this->dataPtr->sensorPlugins.end())
    sensorPlugin = it->second;
  else
  {
    sensorPlugin = this->LoadSensorPlugin(type);
    if (!sensorPlugin)
    {
      ignerr << "Unable to instantiate sensor plugin for [" << type
        << "]\n";
      return nullptr;
    }

    this->dataPtr->sensorPlugins[type] = sensorPlugin;
  }

  auto sensor = sensorPlugin->New();
  if (!sensor)
  {
    ignerr << "Unable to instantiate sensor for [" << type << "]\n";
    return nullptr;
  }

  if (!sensor->Load(_sdf))
  {
    ignerr << "Sensor::Load failed for plugin [" << type << "]\n";
    return nullptr;
  }

  if (!sensor->Init())
  {
    ignerr << "Sensor::Init failed for plugin [" << type << "]\n";
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
      auto it = this->dataPtr->sensorPlugins.find(type);
      if (it != this->dataPtr->sensorPlugins.end())
        sensorPlugin = it->second;
      else
      {
        sensorPlugin = this->LoadSensorPlugin(type);
        if (!sensorPlugin)
        {
          ignerr << "Unable to instantiate sensor plugin for [" << type
                 << "]\n";
          return nullptr;
        }

        this->dataPtr->sensorPlugins[type] = sensorPlugin;
      }

      auto sensor = sensorPlugin->New();
      if (!sensor)
      {
        ignerr << "Unable to instantiate sensor for [" << type << "]\n";
        return nullptr;
      }

      if (!sensor->Load(_sdf))
      {
        ignerr << "Sensor::Load failed for plugin [" << type << "]\n";
        return nullptr;
      }

      if (!sensor->Init())
      {
        ignerr << "Sensor::Init failed for plugin [" << type << "]\n";
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
