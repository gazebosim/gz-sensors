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

#include <ignition/common/Profiler.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>

#include "ignition/sensors/AltimeterSensor.hh"
#include "ignition/sensors/Noise.hh"
#include "ignition/sensors/SensorFactory.hh"
#include "ignition/sensors/SensorTypes.hh"

using namespace ignition;
using namespace sensors;

/// \brief Private data for AltimeterSensor
class ignition::sensors::AltimeterSensorPrivate
{
  /// \brief node to create publisher
  public: transport::Node node;

  /// \brief publisher to publish altimeter messages.
  public: transport::Node::Publisher pub;

  /// \brief true if Load() has been called and was successful
  public: bool initialized = false;

  /// \brief Vertical position in meters
  public: double verticalPosition = 0.0;

  /// \brief Vertical velocity in meters per second
  public: double verticalVelocity = 0.0;

  /// \brief Vertical reference, i.e. initial sensor position
  public: double verticalReference = 0.0;

  /// \brief Noise added to sensor data
  public: std::map<SensorNoiseType, NoisePtr> noises;
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
bool AltimeterSensor::Load(const sdf::Sensor &_sdf)
{
  if (!Sensor::Load(_sdf))
    return false;

  if (_sdf.Type() != sdf::SensorType::ALTIMETER)
  {
    ignerr << "Attempting to a load an Altimeter sensor, but received "
      << "a " << _sdf.TypeStr() << std::endl;
    return false;
  }

  if (_sdf.AltimeterSensor() == nullptr)
  {
    ignerr << "Attempting to a load an Altimeter sensor, but received "
      << "a null sensor." << std::endl;
    return false;
  }

  if (this->Topic().empty())
    this->SetTopic("/altimeter");

  this->dataPtr->pub =
      this->dataPtr->node.Advertise<ignition::msgs::Altimeter>(this->Topic());

  if (!this->dataPtr->pub)
  {
    ignerr << "Unable to create publisher on topic[" << this->Topic() << "].\n";
    return false;
  }

  // Load the noise parameters
  if (_sdf.AltimeterSensor()->VerticalPositionNoise().Type()
      != sdf::NoiseType::NONE)
  {
    this->dataPtr->noises[ALTIMETER_VERTICAL_POSITION_NOISE_METERS] =
      NoiseFactory::NewNoiseModel(
          _sdf.AltimeterSensor()->VerticalPositionNoise());
  }

  if (_sdf.AltimeterSensor()->VerticalVelocityNoise().Type()
      != sdf::NoiseType::NONE)
  {
    this->dataPtr->noises[ALTIMETER_VERTICAL_VELOCITY_NOISE_METERS_PER_S] =
      NoiseFactory::NewNoiseModel(
          _sdf.AltimeterSensor()->VerticalVelocityNoise());
  }

  this->dataPtr->initialized = true;
  return true;
}

//////////////////////////////////////////////////
bool AltimeterSensor::Load(sdf::ElementPtr _sdf)
{
  sdf::Sensor sdfSensor;
  sdfSensor.Load(_sdf);
  return this->Load(sdfSensor);
}

//////////////////////////////////////////////////
bool AltimeterSensor::Update(const ignition::common::Time &_now)
{
  IGN_PROFILE("AltimeterSensor::Update");
  if (!this->dataPtr->initialized)
  {
    ignerr << "Not initialized, update ignored.\n";
    return false;
  }

  msgs::Altimeter msg;
  msg.mutable_header()->mutable_stamp()->set_sec(_now.sec);
  msg.mutable_header()->mutable_stamp()->set_nsec(_now.nsec);
  auto frame = msg.mutable_header()->add_data();
  frame->set_key("frame_id");
  frame->add_value(this->Name());

  // Apply altimeter vertical position noise
  if (this->dataPtr->noises.find(ALTIMETER_VERTICAL_POSITION_NOISE_METERS) !=
      this->dataPtr->noises.end())
  {
    this->dataPtr->verticalPosition =
      this->dataPtr->noises[ALTIMETER_VERTICAL_POSITION_NOISE_METERS]->Apply(
          this->dataPtr->verticalPosition);
  }

  // Apply altimeter vertical velocity noise
  if (this->dataPtr->noises.find(
        ALTIMETER_VERTICAL_VELOCITY_NOISE_METERS_PER_S) !=
      this->dataPtr->noises.end())
  {
    this->dataPtr->verticalVelocity =
      this->dataPtr->noises[
      ALTIMETER_VERTICAL_VELOCITY_NOISE_METERS_PER_S]->Apply(
          this->dataPtr->verticalVelocity);
  }

  msg.set_vertical_position(this->dataPtr->verticalPosition);
  msg.set_vertical_velocity(this->dataPtr->verticalVelocity);
  msg.set_vertical_reference(this->dataPtr->verticalReference);

  // publish
  this->AddSequence(msg.mutable_header());
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
