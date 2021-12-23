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

#include <ignition/common/Profiler.hh>
#include <ignition/math/Angle.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/transport/Node.hh>

#include "ignition/sensors/NavSatSensor.hh"
#include "ignition/sensors/Noise.hh"
#include "ignition/sensors/SensorFactory.hh"
#include "ignition/sensors/SensorTypes.hh"

using namespace ignition;
using namespace sensors;

/// \brief Private data for NavSat
class ignition::sensors::NavSatPrivate
{
  /// \brief Node to create publisher
  public: transport::Node node;

  /// \brief To publish NavSat messages.
  public: transport::Node::Publisher pub;

  /// \brief True if Load() has been called and was successful
  public: bool initialized = false;

  /// \brief Latitude angle
  public: math::Angle latitude;

  /// \brief Longitude angle
  public: math::Angle longitude;

  /// \brief Altitude
  public: double altitude = 0.0;

  /// \brief Velocity in ENU frame.
  public: math::Vector3d velocity;

  /// \brief Noise added to sensor data
  public: std::map<SensorNoiseType, NoisePtr> noises;
};

//////////////////////////////////////////////////
NavSatSensor::NavSatSensor()
  : dataPtr(std::make_unique<NavSatPrivate>())
{
}

//////////////////////////////////////////////////
NavSatSensor::~NavSatSensor() = default;

//////////////////////////////////////////////////
bool NavSatSensor::Init()
{
  return this->Sensor::Init();
}

//////////////////////////////////////////////////
bool NavSatSensor::Load(const sdf::Sensor &_sdf)
{
  if (!Sensor::Load(_sdf))
    return false;

  if (_sdf.Type() != sdf::SensorType::NAVSAT)
  {
    ignerr << "Attempting to a load an NAVSAT sensor, but received "
      << "a " << _sdf.TypeStr() << std::endl;
    return false;
  }

  if (_sdf.NavSatSensor() == nullptr)
  {
    ignerr << "Attempting to a load an NAVSAT sensor, but received "
      << "a null sensor." << std::endl;
    return false;
  }

  if (this->Topic().empty())
    this->SetTopic("/navsat");

  this->dataPtr->pub =
      this->dataPtr->node.Advertise<msgs::NavSat>(this->Topic());

  if (!this->dataPtr->pub)
  {
    ignerr << "Unable to create publisher on topic [" << this->Topic() << "].\n";
    return false;
  }

  // Load the noise parameters
  if (_sdf.NavSatSensor()->HorizontalPositionNoise().Type()
      != sdf::NoiseType::NONE)
  {
    this->dataPtr->noises[NAVSAT_HORIZONTAL_POSITION_NOISE] =
      NoiseFactory::NewNoiseModel(
        _sdf.NavSatSensor()->HorizontalPositionNoise());
  }
  if (_sdf.NavSatSensor()->VerticalPositionNoise().Type()
      != sdf::NoiseType::NONE)
  {
    this->dataPtr->noises[NAVSAT_VERTICAL_POSITION_NOISE] =
      NoiseFactory::NewNoiseModel(
        _sdf.NavSatSensor()->VerticalPositionNoise());
  }
  if (_sdf.NavSatSensor()->HorizontalVelocityNoise().Type()
      != sdf::NoiseType::NONE)
  {
    this->dataPtr->noises[NAVSAT_HORIZONTAL_VELOCITY_NOISE] =
      NoiseFactory::NewNoiseModel(
        _sdf.NavSatSensor()->HorizontalVelocityNoise());
  }
  if (_sdf.NavSatSensor()->VerticalVelocityNoise().Type()
      != sdf::NoiseType::NONE)
  {
    this->dataPtr->noises[NAVSAT_VERTICAL_VELOCITY_NOISE] =
      NoiseFactory::NewNoiseModel(
        _sdf.NavSatSensor()->VerticalVelocityNoise());
  }

  this->dataPtr->initialized = true;
  return true;
}

//////////////////////////////////////////////////
bool NavSatSensor::Load(sdf::ElementPtr _sdf)
{
  sdf::Sensor sdfSensor;
  sdfSensor.Load(_sdf);
  return this->Load(sdfSensor);
}

//////////////////////////////////////////////////
bool NavSatSensor::Update(const std::chrono::steady_clock::duration &_now)
{
  IGN_PROFILE("NavSatSensor::Update");
  if (!this->dataPtr->initialized)
  {
    ignerr << "Not initialized, update ignored.\n";
    return false;
  }

  msgs::NavSat msg;
  *msg.mutable_header()->mutable_stamp() = msgs::Convert(_now);
  msg.set_frame_id(this->Name());

  // Apply noise
  if (this->dataPtr->noises.find(NAVSAT_HORIZONTAL_POSITION_NOISE) !=
      this->dataPtr->noises.end())
  {
    this->SetLatitude(IGN_DTOR(
      this->dataPtr->noises[NAVSAT_HORIZONTAL_POSITION_NOISE]->Apply(
        this->Latitude().Degree())));

    this->SetLongitude(IGN_DTOR(
      this->dataPtr->noises[NAVSAT_HORIZONTAL_POSITION_NOISE]->Apply(
        this->Longitude().Degree())));
  }

  if (this->dataPtr->noises.find(NAVSAT_VERTICAL_POSITION_NOISE) !=
      this->dataPtr->noises.end())
  {
    this->SetAltitude(
      this->dataPtr->noises[NAVSAT_VERTICAL_POSITION_NOISE]->Apply(
        this->Altitude()));
  }

  auto applyNoise = [&](SensorNoiseType noiseType, double &value)
  {
    if (this->dataPtr->noises.find(noiseType) != this->dataPtr->noises.end()){
      value = this->dataPtr->noises[noiseType]->Apply(value);
    }
  };

  applyNoise(NAVSAT_HORIZONTAL_VELOCITY_NOISE, this->dataPtr->velocity.X());
  applyNoise(NAVSAT_HORIZONTAL_VELOCITY_NOISE, this->dataPtr->velocity.Y());
  applyNoise(NAVSAT_VERTICAL_VELOCITY_NOISE, this->dataPtr->velocity.Z());

  // normalise so that it is within +/- 180
  this->dataPtr->latitude.Normalize();
  this->dataPtr->longitude.Normalize();

  msg.set_latitude_deg(this->dataPtr->latitude.Degree());
  msg.set_longitude_deg(this->dataPtr->longitude.Degree());
  msg.set_altitude(this->dataPtr->altitude);
  msg.set_velocity_east(this->dataPtr->velocity.X());
  msg.set_velocity_north(this->dataPtr->velocity.Y());
  msg.set_velocity_up(this->dataPtr->velocity.Z());

  // publish
  this->AddSequence(msg.mutable_header());
  this->dataPtr->pub.Publish(msg);

  return true;
}

//////////////////////////////////////////////////
void NavSatSensor::SetLatitude(const math::Angle &_latitude)
{
  this->dataPtr->latitude = _latitude;
}

//////////////////////////////////////////////////
const math::Angle &NavSatSensor::Latitude() const
{
  return this->dataPtr->latitude;
}

//////////////////////////////////////////////////
void NavSatSensor::SetAltitude(double _altitude)
{
  this->dataPtr->altitude = _altitude;
}

//////////////////////////////////////////////////
double NavSatSensor::Altitude() const
{
  return this->dataPtr->altitude;
}

//////////////////////////////////////////////////
void NavSatSensor::SetLongitude(const math::Angle &_longitude)
{
  this->dataPtr->longitude = _longitude;
}

//////////////////////////////////////////////////
const math::Angle &NavSatSensor::Longitude() const
{
  return this->dataPtr->longitude;
}

//////////////////////////////////////////////////
void NavSatSensor::SetVelocity(const math::Vector3d &_vel)
{
  this->dataPtr->velocity = _vel;
}

//////////////////////////////////////////////////
const math::Vector3d &NavSatSensor::Velocity() const
{
  return this->dataPtr->velocity;
}

//////////////////////////////////////////////////
void NavSatSensor::SetPosition(const math::Angle &_latitude,
    const math::Angle &_longitude, double _altitude)
{
  this->SetLatitude(_latitude);
  this->SetLongitude(_longitude);
  this->SetAltitude(_altitude);
}

