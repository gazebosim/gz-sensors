/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#include <ignition/msgs/double.pb.h>

#include <ignition/common/Console.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/sensors/Noise.hh>
#include <ignition/sensors/SensorFactory.hh>

#include "CustomSensor.hh"

using namespace custom;

//////////////////////////////////////////////////
bool CustomSensor::Load(sdf::ElementPtr _sdf)
{
  if (!_sdf)
  {
    ignerr << "Null SDF pointer." << std::endl;
    return false;
  }

  if (_sdf->GetName() != "sensor")
  {
    ignerr << "SDF element is not a sensor." << std::endl;
    return false;
  }

  std::string type = _sdf->Get<std::string>("type");
  if (type != "custom_sensor")
  {
    ignerr << "Trying to load [custom_sensor], got [" << type << "] instead."
           << std::endl;
    return false;
  }

  // Load common sensor params
  ignition::sensors::Sensor::Load(_sdf);

  // Load noise
  if (_sdf->HasElement("noise"))
  {
    sdf::Noise noiseSdf;
    noiseSdf.Load(_sdf->GetElement("noise"));
    this->noise = ignition::sensors::NoiseFactory::NewNoiseModel(noiseSdf);
  }

  // Advertise topic where data will be published
  this->pub = this->node.Advertise<ignition::msgs::Double>(this->Topic());

  return true;
}

//////////////////////////////////////////////////
bool CustomSensor::Update(const std::chrono::steady_clock::duration &_now)
{
  ignition::msgs::Double msg;
  *msg.mutable_header()->mutable_stamp() = ignition::msgs::Convert(_now);
  auto frame = msg.mutable_header()->add_data();
  frame->set_key("frame_id");
  frame->add_value(this->Name());

  this->data = this->noise->Apply(this->data);

  msg.set_data(this->data);

  this->AddSequence(msg.mutable_header());
  this->pub.Publish(msg);

  return true;
}

IGNITION_ADD_PLUGIN(
    ignition::sensors::SensorTypePlugin<custom::CustomSensor>,
    ignition::sensors::SensorPlugin)
IGNITION_ADD_PLUGIN_ALIAS(
    ignition::sensors::SensorTypePlugin<custom::CustomSensor>,
    "custom_sensor")
