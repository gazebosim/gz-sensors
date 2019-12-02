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
#include <ignition/common/Profiler.hh>

using namespace ignition::sensors;


class ignition::sensors::SensorPrivate
{
  /// \brief Populates fields from a <sensor> DOM
  public: bool PopulateFromSDF(const sdf::Sensor &_sdf);

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
  public: double updateRate = 0.0;

  /// \brief What sim time should this sensor update at
  public: ignition::common::Time nextUpdateTime;

  /// \brief SDF element with sensor information.
  public: sdf::ElementPtr sdf = nullptr;

  /// \brief SDF Sensor DOM object.
  public: sdf::Sensor sdfSensor;
};

SensorId SensorPrivate::idCounter = 0;

//////////////////////////////////////////////////
bool SensorPrivate::PopulateFromSDF(const sdf::Sensor &_sdf)
{
  this->sdfSensor = _sdf;

  // All SDF code gets auto converted to latest version. This code is
  // written assuming sdformat 1.7 is the latest

  // \todo(nkoenig) what to do with <always_on>? SDFormat docs seem
  // to say if true
  //  then the update_rate will be obeyed. Gazebo seems to use it as if
  //  true means enable the sensor at startup.

  // \todo(nkoenig) what to do with <visualize>? ign-sensor data is meant to
  // create
  // sensor data. Whether or not that data should be visualized seems to
  // be outside the scope of this library

  // \todo(nkoenig) how to use frame?
  this->name = _sdf.Name();
  this->topic = _sdf.Topic();

  // Try resolving the pose first, and only use the raw pose if that fails
  auto semPose = _sdf.SemanticPose();
  sdf::Errors errors = semPose.Resolve(this->pose);
  if (!errors.empty())
  {
    this->pose = _sdf.RawPose();
  }

  this->updateRate = _sdf.UpdateRate();
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
bool Sensor::Load(const sdf::Sensor &_sdf)
{
  return this->dataPtr->PopulateFromSDF(_sdf);
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

  sdf::Sensor sdfSensor;
  sdfSensor.Load(_sdf);
  return this->dataPtr->PopulateFromSDF(sdfSensor);
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
  if (_hz < 0)
  {
    this->dataPtr->updateRate = 0;
  }
  else
  {
    this->dataPtr->updateRate = _hz;
  }
}

//////////////////////////////////////////////////
bool Sensor::Update(const ignition::common::Time &_now,
                  const bool _force)
{
  IGN_PROFILE("Sensor::Update");
  bool result = false;

  // Check if it's time to update
  if (_now < this->dataPtr->nextUpdateTime && !_force &&
      this->dataPtr->updateRate > 0)
  {
    return result;
  }

  // Make the update happen
  result = this->Update(_now);

  if (!_force && this->dataPtr->updateRate > 0.0)
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
