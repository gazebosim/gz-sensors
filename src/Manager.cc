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
#include <vector>
#include <gz/common/Profiler.hh>
#include <gz/common/SystemPaths.hh>
#include <gz/common/Console.hh>

#include "gz/sensors/config.hh"
#include "gz/sensors/SensorFactory.hh"

using namespace gz::sensors;

class gz::sensors::Manager::Implementation
{
  /// \brief Loaded sensors.
  public: std::map<SensorId, std::unique_ptr<gz::sensors::Sensor>> sensors;
};

//////////////////////////////////////////////////
Manager::Manager() :
  dataPtr(gz::utils::MakeUniqueImpl<Implementation>())
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
bool Manager::Remove(const gz::sensors::SensorId _id)
{
  return this->dataPtr->sensors.erase(_id) > 0;
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

//////////////////////////////////////////////////
void Manager::RunOnce(
  const std::chrono::steady_clock::duration &_time, bool _force)
{
  GZ_PROFILE("SensorManager::RunOnce");
  auto &sensors = this->dataPtr->sensors;

  // Render ahead: consecutive sensors that can render ahead all render and
  // issue their GPU readbacks first, then each Update() waits for its own
  // readback and publishes, so the readbacks' latencies overlap. Any other
  // sensor runs after the pending run is complete, so GPU commands (and
  // random draws while rendering) keep the serial order.
  // Update() may run callbacks that remove sensors, so sensors are looked up
  // by id again after each call that may run them.
  std::vector<SensorId> ahead;
  auto complete = [&]()
  {
    for (SensorId id : ahead)
    {
      auto it = sensors.find(id);
      if (it != sensors.end())
        it->second->Update(_time, _force);
    }
    ahead.clear();
  };
  SensorId id = 0;
  for (auto it = sensors.begin();; it = sensors.upper_bound(id))
  {
    if (it == sensors.end())
    {
      if (ahead.empty())
        break;
      // Complete the last run; its callbacks may have added sensors after
      // it, which the serial loop would visit too.
      complete();
      continue;
    }
    id = it->first;
    if (it->second->RenderAheadIfDue(_time, _force))
    {
      ahead.push_back(id);
    }
    else
    {
      complete();
      it = sensors.find(id);
      if (it != sensors.end())
        it->second->Update(_time, _force);
    }
  }
}
