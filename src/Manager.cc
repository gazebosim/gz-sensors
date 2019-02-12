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

#include "ignition/sensors/Manager.hh"
#include <unordered_map>
#include <ignition/common/PluginLoader.hh>
#include <ignition/common/Plugin.hh>
#include <ignition/common/SystemPaths.hh>
#include <ignition/common/Console.hh>
#include "ignition/sensors/Events.hh"
#include "ignition/sensors/config.hh"


using namespace ignition::sensors;

class ignition::sensors::ManagerPrivate
{
  /// \brief constructor
  public: ManagerPrivate();

  /// \brief destructor
  public: ~ManagerPrivate();

  /// \brief Loaded sensors.
  public: std::map<SensorId, std::shared_ptr<Sensor>> sensors;

  /// \brief Ignition Rendering manager
  public: ignition::rendering::ScenePtr renderingScene;

  /// \brief Instance used to find stuff on the file system
  public: ignition::common::SystemPaths systemPaths;

  /// \brief Instance used to load plugins
  public: ignition::common::PluginLoader pluginLoader;
};

//////////////////////////////////////////////////
ManagerPrivate::ManagerPrivate()
{
}

//////////////////////////////////////////////////
ManagerPrivate::~ManagerPrivate()
{
}

//////////////////////////////////////////////////
Manager::Manager() :
  dataPtr(new ManagerPrivate)
{
  // Search for plugins in directory where libraries were installed
  this->dataPtr->systemPaths.AddPluginPaths(IGN_SENSORS_PLUGIN_PATH);
}

//////////////////////////////////////////////////
Manager::~Manager()
{
}

//////////////////////////////////////////////////
bool Manager::Init()
{
  return true;
}

//////////////////////////////////////////////////
bool Manager::Init(ignition::rendering::ScenePtr _rendering)
{
  if (!_rendering)
  {
    ignerr << "Null ScenePtr cannot initialize a sensor manager.\n";
    return false;
  }

  bool success = this->Init();
  if (success)
  {
    this->SetRenderingScene(_rendering);
  }
  return success;
}

//////////////////////////////////////////////////
void Manager::SetRenderingScene(ignition::rendering::ScenePtr _rendering)
{
  this->dataPtr->renderingScene = _rendering;
  Events::sceneEvent(this->dataPtr->renderingScene);
}

//////////////////////////////////////////////////
ignition::rendering::ScenePtr Manager::RenderingScene() const
{
  return this->dataPtr->renderingScene;
}

//////////////////////////////////////////////////
ignition::sensors::Sensor *Manager::Sensor(
    ignition::sensors::SensorId _id)
{
  auto iter = this->dataPtr->sensors.find(_id);
  return iter != this->dataPtr->sensors.end() ? iter->second.get() : nullptr;
}

//////////////////////////////////////////////////
void Manager::AddPluginPaths(const std::string &_paths)
{
  this->dataPtr->systemPaths.AddPluginPaths(_paths);
}

//////////////////////////////////////////////////
bool Manager::Remove(const ignition::sensors::SensorId _id)
{
  return this->dataPtr->sensors.erase(_id) > 0;
}

//////////////////////////////////////////////////
void Manager::RunOnce(const ignition::common::Time &_time, bool _force)
{
  for (auto &s : this->dataPtr->sensors)
  {
    s.second->Update(_time, _force);
  }
}

//////////////////////////////////////////////////
ignition::sensors::SensorId Manager::LoadSensorPlugin(
    const std::string &_filename, sdf::ElementPtr _sdf)
{
  std::string fullPath =
    this->dataPtr->systemPaths.FindSharedLibrary(_filename);
  if (fullPath.empty())
  {
    ignerr << "Unable to find sensor plugin path for [" << _filename << "]\n";
    return NO_SENSOR;
  }

  auto pluginNames = this->dataPtr->pluginLoader.LoadLibrary(fullPath);
  if (pluginNames.empty())
  {
    ignerr << "Unable to load sensor plugin file for [" << fullPath << "]\n";
    return NO_SENSOR;
  }

  // Assume the first plugin is the one we're interested in
  // \todo(sloretz) fix Manager API to handle libraries with multiple plugins
  std::string pluginName = *(pluginNames.begin());

  common::PluginPtr pluginPtr =
    this->dataPtr->pluginLoader.Instantiate(pluginName);
  auto sensor = pluginPtr->QueryInterfaceSharedPtr<ignition::sensors::Sensor>();
  if (!sensor)
  {
    ignerr << "Unable to instantiate sensor plugin for [" << fullPath << "]\n";
    return NO_SENSOR;
  }

  // Set the rendering scene
  sensor->SetScene(this->dataPtr->renderingScene);

  if (!sensor->Load(_sdf))
  {
    ignerr << "Sensor::Load failed for plugin [" << fullPath << "]\n";
    return NO_SENSOR;
  }

  if (!sensor->Init())
  {
    ignerr << "Sensor::Init failed for plugin [" << fullPath << "]\n";
    return NO_SENSOR;
  }

  ignition::sensors::SensorId id = sensor->Id();
  this->dataPtr->sensors.insert(
      std::make_pair(id, sensor));

  // Shared pointer so others can access plugins
  return id;
}

/////////////////////////////////////////////////
ignition::sensors::SensorId Manager::CreateSensor(sdf::ElementPtr _sdf)
{
  if (_sdf)
  {
    if (_sdf->GetName() == "sensor")
    {
      std::string type = _sdf->Get<std::string>("type");
      return this->LoadSensorPlugin(IGN_SENSORS_PLUGIN_NAME(type), _sdf);
    }
    /// \todo: Add in plugin support when SDF is updated.
    else
    {
      ignerr << "Provided SDF is not a <sensor> element.\n";
      return NO_SENSOR;
    }
  }

  return NO_SENSOR;
}
