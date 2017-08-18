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

using namespace ignition::sensors;


struct PluginDescription
{
  /// \brief Name of a plugin to load
  public: std::string pluginName;

  /// \brief Library name of a plugin to load
  public: std::string pluginFileName;

  /// \brief element pointer for plugin config data
  public: sdf::ElementPtr pluginElement;
};


class ignition::sensors::SensorPrivate
{
  /// \brief Populates fields from a <sensor> element
  public: void PopulateFromSDF(sdf::ElementPtr _sdf);

  /// \brief a Parent sensor from which to get additional info
  /// \remarks Having a parent is what allows a sensor to bootstrap itself
  ///          with the right plugins when loading from SDF
  public: Sensor *parent;

  /// \brief id given to sensor when constructed
  public: SensorId id;

  /// \brief name given to sensor when loaded
  public: std::string name;

  /// \brief topic to send sensor data
  public: std::string topic;

  /// \brief Pose of the sensor
  public: ignition::math::Pose3d pose;

  /// \brief How many times the sensor will generate data per second
  public: double updateRate;

  /// \brief What sim time should this sensor update at
  public: ignition::common::Time nextUpdateTime;

  /// \brief descriptions of plugins used by this sensor
  public: std::vector<PluginDescription> pluginDescriptions;

  /// \brief instances of plugins used by this sensor
  public: std::vector<std::unique_ptr<Sensor>> plugins;
};

//////////////////////////////////////////////////
void SensorPrivate::PopulateFromSDF(sdf::ElementPtr _sdf)
{
  // All SDF code gets auto converted to latest version. This code is
  // written assuming sdformat 1.6 is the latest

  // TODO what to do with <always_on>? SDFormat docs seem to say if true
  //  then the update_rate will be obeyed. Gazebo seems to use it as if
  //  true means enable the sensor at startup.

  // TODO what to do with <visualize>? ign-sensor data is meant to create
  // sensor data. Whether or not that data should be visualized seems to
  // be outside the scope of this library

  // TODO how to use frame?

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

  // Get info about plugins
  if (_sdf->HasElement("plugin"))
  {
    sdf::ElementPtr pluginElem = _sdf->GetElement("plugin");
    while (pluginElem)
    {
      PluginDescription desc;
      desc.pluginName = pluginElem->Get<std::string>("name");
      desc.pluginFileName = pluginElem->Get<std::string>("filename");
      desc.pluginElement = pluginElem;
      this->pluginDescriptions.push_back(desc);
      pluginElem = pluginElem->GetNextElement("plugin");
    }
  }

  // Load built-in plugins for sensors which are defined by SDFormat
  std::vector<std::pair<std::string, std::string>> builtinPlugins = {
    {"camera", "ignition-sensors-camera"},
    {"altimeter", "ignition-sensors-altimeter"},
    {"contact", "ignition-sensors-contact"},
    {"gps", "ignition-sensors-gps"},
    {"imu", "ignition-sensors-imu"},
    {"logical_camera", "ignition-sensors-logical-camera"},
    {"magnetometer", "ignition-sensors-magnetometer"},
    {"ray", "ignition-sensors-ray"},
    {"sonar", "ignition-sensors-sonar"},
    {"transceiver", "ignition-sensors-transceiver"},
    {"force_torque", "ignition-sensors-force_torque"},
  };

  for (auto builtin : builtinPlugins)
  {
    if (_sdf->HasElement(builtin.first))
    {
      sdf::ElementPtr pluginElem = _sdf->GetElement(builtin.first);
      PluginDescription desc;
      desc.pluginName = "__builtin__";
      desc.pluginFileName = builtin.second;
      desc.pluginElement = pluginElem;
      this->pluginDescriptions.push_back(desc);
    }
  }
}

//////////////////////////////////////////////////
Sensor::Sensor() :
  dataPtr(new SensorPrivate)
{
}

//////////////////////////////////////////////////
void Sensor::Init(Sensor *_parent, SensorId _id)
{
  this->dataPtr->parent = _parent;
  this->dataPtr->id = _id;
}

//////////////////////////////////////////////////
Sensor::~Sensor()
{
}

//////////////////////////////////////////////////
void Sensor::Load(sdf::ElementPtr _sdf)
{
  this->dataPtr->PopulateFromSDF(_sdf);

  // TODO Load plugins
}

//////////////////////////////////////////////////
Sensor *Sensor::Parent() const
{
  return this->dataPtr->parent;
}

//////////////////////////////////////////////////
SensorId Sensor::Id() const
{
  return this->dataPtr->id;
}

//////////////////////////////////////////////////
const std::string &Sensor::Name() const
{
  return this->dataPtr->name;
}

//////////////////////////////////////////////////
const ignition::math::Pose3d &Sensor::Pose() const
{
  // Parent controls the pose
  if (this->dataPtr->parent)
    return this->dataPtr->parent->Pose();
  return this->dataPtr->pose;
}

//////////////////////////////////////////////////
const void Sensor::SetPose(const ignition::math::Pose3d &_pose)
{
  // Parent controls the pose
  if (this->dataPtr->parent)
    this->dataPtr->parent->SetPose(_pose);
  else
    this->dataPtr->pose = _pose;
}

//////////////////////////////////////////////////
double Sensor::UpdateRate() const
{
  // Parent controls the update rate
  if (this->dataPtr->parent)
    return this->dataPtr->parent->UpdateRate();
  return this->dataPtr->updateRate;
}

//////////////////////////////////////////////////
void Sensor::SetUpdateRate(const double _hz)
{
  // Parent controls the update rate
  if (this->dataPtr->parent)
    return this->dataPtr->parent->SetUpdateRate(_hz);
  else
    this->dataPtr->updateRate = _hz;
}

//////////////////////////////////////////////////
void Sensor::Update(const ignition::common::Time &_now,
                  const bool _force)
{
  // Check if it's time to update
  if (_now < this->dataPtr->nextUpdateTime && !_force)
    return;

  // update all plugins
  for (auto &pluginInst : this->dataPtr->plugins)
  {
    pluginInst->Update(_now);
  }

  // update self (useful when running a derived plugin standalone)
  this->Update(_now);

  if (!_force)
  {
    // Update the time the plugin should be loaded
    ignition::common::Time delta(1.0 / this->dataPtr->updateRate);
    this->dataPtr->nextUpdateTime += delta;
  }
}

//////////////////////////////////////////////////
void Sensor::Update(const ignition::common::Time &_now)
{
  // Overridden by derived classes
}

//////////////////////////////////////////////////
ignition::common::Time Sensor::NextUpdateTime() const
{
  // parent controls the next update time
  if (this->dataPtr->parent)
    return this->dataPtr->parent->NextUpdateTime();
  return this->dataPtr->nextUpdateTime;
}
