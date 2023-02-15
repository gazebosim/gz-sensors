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

#include "gz/sensors/Manager.hh"
#include <memory>
#include <unordered_map>
#include <gz/common/PluginLoader.hh>
#include <gz/common/Plugin.hh>
#include <gz/common/Profiler.hh>
#include <gz/common/SystemPaths.hh>
#include <gz/common/Console.hh>

#include "gz/sensors/config.hh"
#include "gz/sensors/SensorFactory.hh"

using namespace gz::sensors;

class gz::sensors::ManagerPrivate
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
Sensor *Manager::Sensor(
    SensorId _id)
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
bool Manager::Remove(const SensorId _id)
{
  return this->dataPtr->sensors.erase(_id) > 0;
}

//////////////////////////////////////////////////
void Manager::RunOnce(const gz::common::Time &_time, bool _force)
{
  IGN_PROFILE("SensorManager::RunOnce");
  for (auto &s : this->dataPtr->sensors)
  {
    s.second->Update(_time, _force);
  }
}

/////////////////////////////////////////////////
SensorId Manager::CreateSensor(const sdf::Sensor &_sdf)
{
  auto sensor = this->dataPtr->sensorFactory.CreateSensor(_sdf);
  if (!sensor)
    return NO_SENSOR;

  SensorId id = sensor->Id();
  this->dataPtr->sensors[id] = std::move(sensor);
  return id;
}

/////////////////////////////////////////////////
SensorId Manager::CreateSensor(sdf::ElementPtr _sdf)
{
  auto sensor = this->dataPtr->sensorFactory.CreateSensor(_sdf);
  if (!sensor)
    return NO_SENSOR;

  SensorId id = sensor->Id();
  this->dataPtr->sensors[id] = std::move(sensor);
  return id;
}
