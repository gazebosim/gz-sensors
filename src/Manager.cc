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
#include <gz/common/Profiler.hh>
#include <gz/common/SystemPaths.hh>
#include <gz/common/Console.hh>

#include "gz/sensors/config.hh"
#include "gz/sensors/SensorFactory.hh"

using namespace gz::sensors;

class gz::sensors::ManagerPrivate
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
  this->dataPtr->sensors.clear();
}

//////////////////////////////////////////////////
bool Manager::Init()
{
  return true;
}

//////////////////////////////////////////////////
gz::sensors::Sensor *Manager::Sensor(
    gz::sensors::SensorId _id)
{
  auto iter = this->dataPtr->sensors.find(_id);
  return iter != this->dataPtr->sensors.end() ? iter->second.get() : nullptr;
}

//////////////////////////////////////////////////
void Manager::AddPluginPaths(const std::string &)
{
  gzwarn << "Trying to add plugin paths, but Gazebo Sensors doesn't support"
          << " plugins anymore." << std::endl;
}

//////////////////////////////////////////////////
bool Manager::Remove(const gz::sensors::SensorId _id)
{
  return this->dataPtr->sensors.erase(_id) > 0;
}

//////////////////////////////////////////////////
void Manager::RunOnce(
  const std::chrono::steady_clock::duration &_time, bool _force)
{
  GZ_PROFILE("SensorManager::RunOnce");
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
gz::sensors::SensorId Manager::CreateSensor(const sdf::Sensor &)
{
  gzwarn << "Trying to create sensor without providing sensor type. Gazebo"
          << " Sensors doesn't support sensor registration anymore. Use the"
          << " templated `CreateSensor` function instead." << std::endl;
  return NO_SENSOR;
}

/////////////////////////////////////////////////
gz::sensors::SensorId Manager::CreateSensor(sdf::ElementPtr)
{
  gzwarn << "Trying to create sensor without providing sensor type. Gazebo"
          << " Sensors doesn't support sensor registration anymore. Use the"
          << " templated `CreateSensor` function instead." << std::endl;
  return NO_SENSOR;
}
