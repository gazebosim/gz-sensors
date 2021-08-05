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
#include <ignition/sensors/Noise.hh>
#include <ignition/sensors/SensorFactory.hh>

#include "DoubleSensor.hh"

using namespace custom;

//////////////////////////////////////////////////
bool DoubleSensor::Load(const sdf::Sensor &_sdf)
{
  if (_sdf.Type() != sdf::SensorType::CUSTOM)
  {
    ignerr << "Trying to load custom sensor, but got type ["
           << static_cast<int>(_sdf.Type()) << "] instead." << std::endl;
    return false;
  }

  auto elem = _sdf.Element();
  if (nullptr == elem)
  {
    ignerr << "SDF missing element pointer." << std::endl;
    return false;
  }

  if (!elem->HasAttribute("ignition:type"))
  {
    ignerr << "Custom sensor missing `ignition::type` attribute. "
           << "Not loading." << std::endl;
    return false;
  }

  auto customType = elem->Get<std::string>("ignition:type");
  if (customType != "double")
  {
    ignerr << "Trying to load sensor of type [double], but got sensor of type ["
           << customType << "]." << std::endl;
    return false;
  }

  // Load common sensor params
  ignition::sensors::Sensor::Load(_sdf);

  // Advertise topic where data will be published
  this->pub = this->node.Advertise<ignition::msgs::Double>(this->Topic());

  if (!elem->HasElement("ignition:double"))
  {
    igndbg << "No custom configuration for [" << this->Topic() << "]"
           << std::endl;
    return true;
  }

  auto customElem = elem->GetElement("ignition:double");

  if (!customElem->HasElement("noise"))
  {
    igndbg << "No noise for [" << this->Topic() << "]" << std::endl;
    return true;
  }

  sdf::Noise noiseSdf;
  noiseSdf.Load(customElem->GetElement("noise"));
  this->noise = ignition::sensors::NoiseFactory::NewNoiseModel(noiseSdf);
  if (nullptr == this->noise)
  {
    ignerr << "Failed to load noise." << std::endl;
    return false;
  }

  return true;
}

//////////////////////////////////////////////////
bool DoubleSensor::Update(const std::chrono::steady_clock::duration &_now)
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

