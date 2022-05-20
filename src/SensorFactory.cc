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

#include <gz/common/Console.hh>

#include "gz/sensors/SensorFactory.hh"

/// \brief Private data class for SensorFactory
class gz::sensors::SensorFactoryPrivate
{
};

using namespace gz;
using namespace sensors;

//////////////////////////////////////////////////
void SensorFactory::AddPluginPaths(const std::string &)
{
  ignwarn << "Trying to add plugin paths, but Gazebo Sensors doesn't support"
          << " plugins anymore." << std::endl;
}

//////////////////////////////////////////////////
SensorFactory::SensorFactory() : dataPtr(new SensorFactoryPrivate)
{
}

//////////////////////////////////////////////////
SensorFactory::~SensorFactory()
{
}

/////////////////////////////////////////////////
std::unique_ptr<Sensor> SensorFactory::CreateSensor(const sdf::Sensor &)
{
  ignwarn << "Trying to create sensor without providing sensor type. Ignition"
          << " Sensor doesn't support sensor registration anymore. Use the"
          << " templated `CreateSensor` function instead." << std::endl;
  return nullptr;
}

/////////////////////////////////////////////////
std::unique_ptr<Sensor> SensorFactory::CreateSensor(sdf::ElementPtr)
{
  ignwarn << "Trying to create sensor without providing sensor type. Ignition"
          << " Sensor doesn't support sensor registration anymore. Use the"
          << " templated `CreateSensor` function instead." << std::endl;
  return nullptr;
}
