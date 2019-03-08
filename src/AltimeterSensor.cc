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

#include <ignition/transport/Node.hh>

#include "ignition/sensors/SensorFactory.hh"
#include "ignition/sensors/AltimeterSensor.hh"

using namespace ignition;
using namespace sensors;

/// \brief Private data for AltimeterSensor
class ignition::sensors::AltimeterSensorPrivate
{
  /// \brief node to create publisher
  public: transport::Node node;

  /// \brief publisher to publish logical camera messages.
  public: transport::Node::Publisher pub;

  /// \brief true if Load() has been called and was successful
  public: bool initialized = false;

  /// \brief Vertical position in meters
  public: double verticalPosition = 0.0;

  /// \brief Vertical velocity in meters per second
  public: double verticalVelocity = 0.0;

  /// \brief Vertical reference, i.e. initial sensor position
  public: double verticalReference = 0.0;
};

//////////////////////////////////////////////////
AltimeterSensor::AltimeterSensor()
  : dataPtr(new AltimeterSensorPrivate())
{
}

//////////////////////////////////////////////////
AltimeterSensor::~AltimeterSensor()
{
}

//////////////////////////////////////////////////
bool AltimeterSensor::Init()
{
  return this->Sensor::Init();
}

//////////////////////////////////////////////////
bool AltimeterSensor::Load(sdf::ElementPtr _sdf)
{
  if (!Sensor::Load(_sdf))
    return false;

  std::string topic = this->Topic();
  if (topic.empty())
    topic = "/altimeter";

  this->dataPtr->pub =
      this->dataPtr->node.Advertise<ignition::msgs::Altimeter>(topic);

  if (!this->dataPtr->pub)
    return false;

  this->dataPtr->initialized = true;
  return true;
}

//////////////////////////////////////////////////
bool AltimeterSensor::Update(const ignition::common::Time &_now)
{
  if (!this->dataPtr->initialized)
  {
    ignerr << "Not initialized, update ignored.\n";
    return false;
  }

  msgs::Altimeter msg;
  msg.mutable_header()->mutable_stamp()->set_sec(_now.sec);
  msg.mutable_header()->mutable_stamp()->set_nsec(_now.nsec);
  msg.set_vertical_position(this->dataPtr->verticalPosition);
  msg.set_vertical_velocity(this->dataPtr->verticalVelocity);
  msg.set_vertical_reference(this->dataPtr->verticalReference);

  // publish
  this->dataPtr->pub.Publish(msg);

  return true;
}

//////////////////////////////////////////////////
void AltimeterSensor::SetVerticalReference(double _reference)
{
  this->dataPtr->verticalReference = _reference;
}

//////////////////////////////////////////////////
double AltimeterSensor::VerticalReference() const
{
  return this->dataPtr->verticalReference;
}

//////////////////////////////////////////////////
void AltimeterSensor::SetPosition(double _pos)
{
  this->dataPtr->verticalPosition = _pos - this->dataPtr->verticalReference;
}

//////////////////////////////////////////////////
double AltimeterSensor::VerticalPosition() const
{
  return this->dataPtr->verticalPosition;
}

//////////////////////////////////////////////////
void AltimeterSensor::SetVerticalVelocity(double _vel)
{
  this->dataPtr->verticalVelocity = _vel;
}

//////////////////////////////////////////////////
double AltimeterSensor::VerticalVelocity() const
{
  return this->dataPtr->verticalVelocity;
}

IGN_SENSORS_REGISTER_SENSOR(AltimeterSensor)
