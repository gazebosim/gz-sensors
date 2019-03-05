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

#include <unordered_map>
#include <ignition/common/SystemPaths.hh>
#include <ignition/common/Console.hh>

#include "ignition/sensors/config.hh"
// #include "ignition/sensors/Events.hh"
#include "ignition/sensors/Manager.hh"
#include "ignition/sensors/SensorFactory.hh"

using namespace ignition::sensors;

class ignition::sensors::ManagerPrivate
{
  /// \brief constructor
  public: ManagerPrivate();

  /// \brief destructor
  public: ~ManagerPrivate();

  /// \brief Loaded sensors.
  public: std::map<SensorId, std::unique_ptr<Sensor>> sensors;

  /// \brief Sensor factory for creating sensors from plugins;
  public: SensorFactory sensorFactory;
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
//bool Manager::Init(ignition::rendering::ScenePtr _rendering)
//{
//  if (!_rendering)
//  {
//    ignerr << "Null ScenePtr cannot initialize a sensor manager.\n";
//    return false;
//  }
//
//  bool success = this->Init();
//  if (success)
//  {
//    this->SetRenderingScene(_rendering);
//  }
//  return success;
//}

//////////////////////////////////////////////////
//void Manager::SetRenderingScene(ignition::rendering::ScenePtr _rendering)
//{
//  this->dataPtr->renderingScene = _rendering;
//  Events::sceneEvent(this->dataPtr->renderingScene);
//}

//////////////////////////////////////////////////
//ignition::rendering::ScenePtr Manager::RenderingScene() const
//{
//  return this->dataPtr->renderingScene;
//}

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
  this->dataPtr->sensorFactory.AddPluginPaths(_paths);
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

/////////////////////////////////////////////////
ignition::sensors::SensorId Manager::CreateSensor(sdf::ElementPtr _sdf)
{
  auto sensor = this->dataPtr->sensorFactory.CreateSensor(_sdf);
  if (!sensor)
    return NO_SENSOR;

  // Set the rendering scene
  // sensor->SetScene(this->dataPtr->renderingScene);

  SensorId id = sensor->Id();
  this->dataPtr->sensors[id] = std::move(sensor);
  return id;
}
