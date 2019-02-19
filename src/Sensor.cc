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

#include "ignition/sensors/Sensor.hh"
#include <vector>
#include <ignition/sensors/Manager.hh>
#include <ignition/common/Console.hh>

using namespace ignition::sensors;


class ignition::sensors::SensorPrivate
{
  /// \brief Populates fields from a <sensor> element
  public: bool PopulateFromSDF(sdf::ElementPtr _sdf);

  /// \brief id given to sensor when constructed
  public: SensorId id;

  /// \brief Counter used to generate unique sensor identifiers.
  public: static SensorId idCounter;

  /// \brief name given to sensor when loaded
  public: std::string name;

  /// \brief name given to the sensor parent link
  public: std::string parent;

  /// \brief topic to send sensor data
  public: std::string topic;

  /// \brief Pose of the sensor
  public: ignition::math::Pose3d pose;

  /// \brief How many times the sensor will generate data per second
  public: double updateRate = 1;

  /// \brief What sim time should this sensor update at
  public: ignition::common::Time nextUpdateTime;

  public: sdf::ElementPtr sdf = nullptr;
};

SensorId SensorPrivate::idCounter = 0;

//////////////////////////////////////////////////
bool SensorPrivate::PopulateFromSDF(sdf::ElementPtr _sdf)
{
  // All SDF code gets auto converted to latest version. This code is
  // written assuming sdformat 1.6 is the latest

  // \todo(nkoenig) what to do with <always_on>? SDFormat docs seem
  // to say if true
  //  then the update_rate will be obeyed. Gazebo seems to use it as if
  //  true means enable the sensor at startup.

  // \todo(nkoenig) what to do with <visualize>? ign-sensor data is meant to
  // create
  // sensor data. Whether or not that data should be visualized seems to
  // be outside the scope of this library

  // \todo(nkoenig) how to use frame?

  if (!_sdf)
  {
    ignerr << "null _sdf\n";
    return false;
  }

  if (std::string("plugin") == _sdf->GetName())
    _sdf = _sdf->GetParent();

  if (std::string("sensor") != _sdf->GetName())
  {
    ignerr << "SDF is not a sensor.\n";
    _sdf->PrintValues("  ");
    return false;
  }

  this->name = _sdf->Get<std::string>("name");

  if (_sdf->HasElement("topic"))
  {
    this->topic = _sdf->Get<std::string>("topic");
  }

  if (_sdf->HasElement("pose"))
  {
    this->pose = _sdf->Get<ignition::math::Pose3d>("pose");
  }

  if (_sdf->HasElement("update_rate"))
  {
    this->updateRate = (_sdf->Get<double>("update_rate"));
  }
  return true;
}

//////////////////////////////////////////////////
Sensor::Sensor() :
  dataPtr(new SensorPrivate)
{
  this->dataPtr->id = (++this->dataPtr->idCounter);
}

//////////////////////////////////////////////////
bool Sensor::Init()
{
  return true;
}

//////////////////////////////////////////////////
Sensor::~Sensor()
{
}

//////////////////////////////////////////////////
bool Sensor::Load(sdf::ElementPtr _sdf)
{
  if (!this->dataPtr->sdf)
  {
    this->dataPtr->sdf = _sdf->Clone();
  }
  else
    this->dataPtr->sdf->Copy(_sdf);
  return this->dataPtr->PopulateFromSDF(_sdf);
}

//////////////////////////////////////////////////
sdf::ElementPtr Sensor::SDF() const
{
  return this->dataPtr->sdf;
}

//////////////////////////////////////////////////
SensorId Sensor::Id() const
{
  return this->dataPtr->id;
}

//////////////////////////////////////////////////
std::string Sensor::Name() const
{
  return this->dataPtr->name;
}

//////////////////////////////////////////////////
std::string Sensor::Topic() const
{
  return this->dataPtr->topic;
}

//////////////////////////////////////////////////
ignition::math::Pose3d Sensor::Pose() const
{
  return this->dataPtr->pose;
}

//////////////////////////////////////////////////
std::string Sensor::Parent() const
{
  return this->dataPtr->parent;
}

//////////////////////////////////////////////////
void Sensor::SetParent(const std::string &_parent)
{
  this->dataPtr->parent = _parent;
}

//////////////////////////////////////////////////
void Sensor::SetPose(const ignition::math::Pose3d &_pose)
{
  this->dataPtr->pose = _pose;
}

//////////////////////////////////////////////////
double Sensor::UpdateRate() const
{
  return this->dataPtr->updateRate;
}

//////////////////////////////////////////////////
void Sensor::SetUpdateRate(const double _hz)
{
  this->dataPtr->updateRate = _hz;
}

//////////////////////////////////////////////////
bool Sensor::Update(const ignition::common::Time &_now,
                  const bool _force)
{
  bool result = false;

  // Check if it's time to update
  if (_now < this->dataPtr->nextUpdateTime && !_force)
    return result;

  // Make the update happen
  result = this->Update(_now);

  if (!_force)
  {
    // Update the time the plugin should be loaded
    ignition::common::Time delta(1.0 / this->dataPtr->updateRate);
    this->dataPtr->nextUpdateTime += delta;
  }

  return result;
}

//////////////////////////////////////////////////
ignition::common::Time Sensor::NextUpdateTime() const
{
  return this->dataPtr->nextUpdateTime;
}

/////////////////////////////////////////////////
void Sensor::SetScene(ignition::rendering::ScenePtr)
{
}
