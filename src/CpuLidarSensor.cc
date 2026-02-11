/*
 * Copyright (C) 2025 Open Source Robotics Foundation
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

#include "gz/sensors/CpuLidarSensor.hh"
#include "gz/sensors/SensorFactory.hh"

using namespace gz;
using namespace sensors;

/// \brief Private data for CpuLidarSensor
class gz::sensors::CpuLidarSensorPrivate
{
  /// \brief true if Load() has been called and was successful
  public: bool initialized = false;
};

//////////////////////////////////////////////////
CpuLidarSensor::CpuLidarSensor()
  : dataPtr(new CpuLidarSensorPrivate())
{
}

//////////////////////////////////////////////////
CpuLidarSensor::~CpuLidarSensor()
{
}

//////////////////////////////////////////////////
bool CpuLidarSensor::Load(const sdf::Sensor &_sdf)
{
  if (!Sensor::Load(_sdf))
    return false;

  if (_sdf.Type() != sdf::SensorType::LIDAR)
  {
    gzerr << "Attempting to load a CpuLidar sensor, but received "
      << "a " << _sdf.TypeStr() << std::endl;
    return false;
  }

  this->dataPtr->initialized = true;
  return true;
}

//////////////////////////////////////////////////
bool CpuLidarSensor::Load(sdf::ElementPtr _sdf)
{
  sdf::Sensor sdfSensor;
  sdfSensor.Load(_sdf);
  return this->Load(sdfSensor);
}

//////////////////////////////////////////////////
bool CpuLidarSensor::Update(
    const std::chrono::steady_clock::duration &/*_now*/)
{
  if (!this->dataPtr->initialized)
  {
    gzerr << "Not initialized, update ignored.\n";
    return false;
  }

  return false;
}

//////////////////////////////////////////////////
bool CpuLidarSensor::HasConnections() const
{
  return false;
}
