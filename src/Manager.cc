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

#include <ignition/common/SystemPaths.hh>
#include <ignition/sensors/Manager.hh>

#include <atomic>


using namespace ignition::sensors;


class ignition::sensors::ManagerPrivate
{
  /// \brief constructor
  public: ManagerPrivate();

  /// \brief destructor
  public: ~ManagerPrivate();

  // TODO use a map so sensors can be removed without changing their id
  /// \brief loaded sensors (index + 1 is sensor id)
  public: std::vector<ignition::sensors::Sensor> sensors;

  /// \brief Ignition Rendering manager
  public: ignition::rendering::Manager *renderingManager;

  /// \brief Instance used to find stuff on the file system
  public: ignition::common::SystemPaths systemPaths;
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
  this->dataPtr = std::make_unique<ManagerPrivate>();
  return true;
}

//////////////////////////////////////////////////
bool Manager::Init(ignition::rendering::Manager &_rendering)
{
  bool success = this->Init();
  if (success)
  {
    this->SetRendering(_rendering);
  }
  return success;
}

//////////////////////////////////////////////////
void Manager::SetRendering(ignition::rendering::Manager &_rendering)
{
  this->dataPtr->renderingManager = &_rendering;
}

//////////////////////////////////////////////////
ignition::rendering::Manager &Manager::RenderingManager() const
{
  return *(this->dataPtr->renderingManager);
}

//////////////////////////////////////////////////
SensorId Manager::LoadSensor(sdf::ElementPtr &_sdf)
{
  ignition::sensors::Sensor sensor;
  SensorId id = this->dataPtr->sensors.size() + 1;
  sensor.Init(this, id);
  // TODO check if load succeeded
  if (sensor.Load(_sdf))
  {
    // TODO does this class need to be thread safe?
    this->dataPtr->sensors.push_back(sensor);
  }
  else
    id = NO_SENSOR;
  return id;
}

//////////////////////////////////////////////////
void Manager::AddPluginPaths(const std::string &_paths)
{
  this->dataPtr->systemPaths.AddPluginPaths(_paths);
}

//////////////////////////////////////////////////
std::string Manager::FindPlugin(const std::string &_name)
{
  return this->dataPtr->systemPaths.FindSharedLibrary(_name);
}

//////////////////////////////////////////////////
void Manager::Remove(const SensorId _id)
{
  // TODO remove sensor
}

//////////////////////////////////////////////////
void Manager::Remove(const std::string &_name)
{
  // TODO remove sensor by name
}

//////////////////////////////////////////////////
void Manager::RunOnce(const ignition::common::Time &_time, bool _force)
{
  for (auto &s : this->dataPtr->sensors)
  {
    s.Update(_time, _force);
  }
}

//////////////////////////////////////////////////
SensorId Manager::Sensor(const std::string &_name)
{
  // TODO find sensor id given sensor name
}
