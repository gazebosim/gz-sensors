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

#include <atomic>

#include <ignition/sensors/Manager.hh>

namespace ignsen = ignition::sensors;
using namespace ignition;
using namespace ignsen;

class ignsen::ManagerPrivate
{
  /// \brief constructor
  public: ManagerPrivate();

  /// \brief destructor
  public: ~ManagerPrivate();
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
void SetRendering(ignition::rendering::Manager &_rendering)
{
}

//////////////////////////////////////////////////
SensorId Manager::LoadSensor(const std::string &_filename,
            const std::string &_sensorName,
            const std::string &_parentName)
{
  // TODO plugin loader
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
void Manager::RunOnce(bool _force)
{
  // TODO Run all sensors once
}

//////////////////////////////////////////////////
void Manager::Stop()
{
  // Stop running all threads
}

//////////////////////////////////////////////////
SensorId Sensor(const std::string &_name)
{
  // TODO find sensor id given sensor name
}
