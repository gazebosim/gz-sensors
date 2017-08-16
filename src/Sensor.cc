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

#include <ignition/sensors/Sensor.hh>

using namespace ignition::sensors;


class ignition::sensors::SensorPrivate
{
  /// \brief id given to sensor when constructed
  public: SensorId id;

  /// \brief name given to sensor when loaded
  public: std::string name;

  /// \brief Pose of the sensor
  public: ignition::math::Pose3d pose;
};


//////////////////////////////////////////////////
Sensor::Sensor(SensorId _id) :
  dataPtr(new SensorPrivate)
{
  this->dataPtr->id =_id;
}

//////////////////////////////////////////////////
Sensor::~Sensor()
{
}

//////////////////////////////////////////////////
void Sensor::Load(sdf::ElementPtr _sdf)
{
  // TODO load basic sensor parameters
}

//////////////////////////////////////////////////
const std::string &Sensor::Name() const
{
  return this->dataPtr->name;
}

//////////////////////////////////////////////////
const ignition::math::Pose3d &Sensor::Pose() const
{
  return this->dataPtr->pose;
}

//////////////////////////////////////////////////
const void Sensor::SetPose(const ignition::math::Pose3d &_pose)
{
  this->dataPtr->pose = _pose;
}

//////////////////////////////////////////////////
SensorId Sensor::Id() const
{
  return this->dataPtr->id;
}
