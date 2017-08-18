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

  /// \brief plugins this sensor uses
  std::vector<PluginDescription> plugins;
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
      this->plugins.push_back(desc);
      pluginElem = pluginElem->GetNextElement("plugin");
    }
  }

  // Add built in plugins to handle sensors defined in SDF 1.6
  if (_sdf->HasElement("camera"))
  {
    sdf::ElementPtr pluginElem = _sdf->GetElement("camera");
    PluginDescription desc;
    desc.pluginName = pluginElem->Get<std::string>("name");
    desc.pluginFileName = "ignition-sensors-camera";
    desc.pluginElement = pluginElem;
    this->plugins.push_back(desc);
  }
  if (_sdf->HasElement("altimeter"))
  {
    sdf::ElementPtr pluginElem = _sdf->GetElement("altimeter");
    PluginDescription desc;
    desc.pluginName = this->name;
    desc.pluginFileName = "ignition-sensors-altimeter";
    desc.pluginElement = pluginElem;
    this->plugins.push_back(desc);
  }
  if (_sdf->HasElement("contact"))
  {
    sdf::ElementPtr pluginElem = _sdf->GetElement("contact");
    PluginDescription desc;
    desc.pluginName = this->name;
    desc.pluginFileName = "ignition-sensors-contact";
    desc.pluginElement = pluginElem;
    this->plugins.push_back(desc);
  }
  if (_sdf->HasElement("gps"))
  {
    sdf::ElementPtr pluginElem = _sdf->GetElement("gps");
    PluginDescription desc;
    desc.pluginName = this->name;
    desc.pluginFileName = "ignition-sensors-gps";
    desc.pluginElement = pluginElem;
    this->plugins.push_back(desc);
  }
  if (_sdf->HasElement("imu"))
  {
    sdf::ElementPtr pluginElem = _sdf->GetElement("imu");
    PluginDescription desc;
    desc.pluginName = this->name;
    desc.pluginFileName = "ignition-sensors-gps";
    desc.pluginElement = pluginElem;
    this->plugins.push_back(desc);
  }
  if (_sdf->HasElement("logical_camera"))
  {
    sdf::ElementPtr pluginElem = _sdf->GetElement("logical_camera");
    PluginDescription desc;
    desc.pluginName = this->name;
    desc.pluginFileName = "ignition-sensors-logical-camera";
    desc.pluginElement = pluginElem;
    this->plugins.push_back(desc);
  }
  if (_sdf->HasElement("magnetometer"))
  {
    sdf::ElementPtr pluginElem = _sdf->GetElement("magnetometer");
    PluginDescription desc;
    desc.pluginName = this->name;
    desc.pluginFileName = "ignition-sensors-magnetometer";
    desc.pluginElement = pluginElem;
    this->plugins.push_back(desc);
  }
  if (_sdf->HasElement("ray"))
  {
    sdf::ElementPtr pluginElem = _sdf->GetElement("ray");
    PluginDescription desc;
    desc.pluginName = this->name;
    desc.pluginFileName = "ignition-sensors-ray";
    desc.pluginElement = pluginElem;
    this->plugins.push_back(desc);
  }
  if (_sdf->HasElement("sonar"))
  {
    sdf::ElementPtr pluginElem = _sdf->GetElement("sonar");
    PluginDescription desc;
    desc.pluginName = this->name;
    desc.pluginFileName = "ignition-sensors-sonar";
    desc.pluginElement = pluginElem;
    this->plugins.push_back(desc);
  }
  if (_sdf->HasElement("transceiver"))
  {
    sdf::ElementPtr pluginElem = _sdf->GetElement("tranceiver");
    PluginDescription desc;
    desc.pluginName = this->name;
    desc.pluginFileName = "ignition-sensors-transceiver";
    desc.pluginElement = pluginElem;
    this->plugins.push_back(desc);
  }
  if (_sdf->HasElement("force_torque"))
  {
    sdf::ElementPtr pluginElem = _sdf->GetElement("force_torque");
    PluginDescription desc;
    desc.pluginName = this->name;
    desc.pluginFileName = "ignition-sensors-force-torque";
    desc.pluginElement = pluginElem;
    this->plugins.push_back(desc);
  }
}

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
  this->dataPtr->PopulateFromSDF(_sdf);

  // TODO Load plugins
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
  return this->dataPtr->pose;
}

//////////////////////////////////////////////////
const void Sensor::SetPose(const ignition::math::Pose3d &_pose)
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
void Sensor::Update(const ignition::common::Time &_now,
                  const bool _force)
{
  // TODO Check if it's time to update

  // TODO update all plugins

  // update self (useful when running a derived plugin standalone)
  this->Update(_now);
}

//////////////////////////////////////////////////
void Sensor::Update(const ignition::common::Time &_now)
{
  // Overridden by derived classes
}

//////////////////////////////////////////////////
ignition::common::Time Sensor::NextUpdateTime() const
{
  // TODO return the next time the sensor will be updated
}
