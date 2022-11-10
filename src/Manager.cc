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
#include <memory>
#include <unordered_map>
#include <ignition/common/Profiler.hh>
#include <ignition/common/SystemPaths.hh>
#include <ignition/common/Console.hh>

#include "ignition/sensors/config.hh"
#include "ignition/sensors/SensorFactory.hh"

using namespace ignition::sensors;

class ignition::sensors::ManagerPrivate
{
  /// \brief Loaded sensors.
  public: std::map<SensorId, std::unique_ptr<Sensor>> sensors;
};

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
void Manager::AddPluginPaths(const std::string &)
{
  ignwarn << "Trying to add plugin paths, but Ignition Sensors doesn't support"
          << " plugins anymore." << std::endl;
}

//////////////////////////////////////////////////
bool Manager::Remove(const SensorId _id)
{
  return this->dataPtr->sensors.erase(_id) > 0;
}

//////////////////////////////////////////////////
void Manager::RunOnce(
  const std::chrono::steady_clock::duration &_time, bool _force)
{
  IGN_PROFILE("SensorManager::RunOnce");
  for (auto &s : this->dataPtr->sensors)
  {
    s.second->Update(_time, _force);
  }
}

/////////////////////////////////////////////////
SensorId Manager::AddSensor(
  std::unique_ptr<sensors::Sensor> _sensor)
{
  if (!_sensor)
    return NO_SENSOR;
  SensorId id = _sensor->Id();
  this->dataPtr->sensors[id] = std::move(_sensor);
  return id;
}

/////////////////////////////////////////////////
SensorId Manager::CreateSensor(const sdf::Sensor &)
{
  ignwarn << "Trying to create sensor without providing sensor type. Ignition"
          << " Sensor doesn't support sensor registration anymore. Use the"
          << " templated `CreateSensor` function instead." << std::endl;
  return NO_SENSOR;
}

/////////////////////////////////////////////////
SensorId Manager::CreateSensor(sdf::ElementPtr)
{
  ignwarn << "Trying to create sensor without providing sensor type. Ignition"
          << " Sensor doesn't support sensor registration anymore. Use the"
          << " templated `CreateSensor` function instead." << std::endl;
  return NO_SENSOR;
}
