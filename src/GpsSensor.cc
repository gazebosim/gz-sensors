/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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
#include <ignition/transport/Node.hh>
#include <ignition/math/Angle.hh>
#include <ignition/math/Vector3.hh>

#include "ignition/sensors/GpsSensor.hh"
#include "ignition/sensors/Noise.hh"
#include "ignition/sensors/SensorFactory.hh"
#include "ignition/sensors/SensorTypes.hh"

using namespace ignition;
using namespace sensors;

/// \brief Private data for Gps
class ignition::sensors::GpsPrivate
{
  /// \brief node to create publisher
  public: transport::Node node;

  /// \brief publisher to publish gps messages.
  public: transport::Node::Publisher pub;

  /// \brief true if Load() has been called and was successful
  public: bool initialized = false;

  /// \brief latitude angle
  public: ignition::math::Angle latitude;

  /// \brief longitude angle
  public: ignition::math::Angle longitude;

  /// \brief altitude
  public: double altitude = 0.0;

  /// \brief velocity
  public: ignition::math::Vector3d velocity;

  /// \brief Noise added to sensor data
  public: std::map<SensorNoiseType, NoisePtr> noises;
};

//////////////////////////////////////////////////
GpsSensor::GpsSensor()
  : dataPtr(new GpsPrivate())
{
}

//////////////////////////////////////////////////
GpsSensor::~GpsSensor()
{
}

//////////////////////////////////////////////////
bool GpsSensor::Init()
{
  return this->Sensor::Init();
}

//////////////////////////////////////////////////
bool GpsSensor::Load(const sdf::Sensor &_sdf)
{
  if (!Sensor::Load(_sdf))
    return false;

  if (_sdf.Type() != sdf::SensorType::GPS)
  {
    ignerr << "Attempting to a load an GPS sensor, but received "
      << "a " << _sdf.TypeStr() << std::endl;
    return false;
  }

  if (_sdf.GpsSensor() == nullptr)
  {
    ignerr << "Attempting to a load an GPS sensor, but received "
      << "a null sensor." << std::endl;
    return false;
  }

  if (this->Topic().empty())
    this->SetTopic("/gps");

  this->dataPtr->pub =
      this->dataPtr->node.Advertise<ignition::msgs::GPS>(this->Topic());

  if (!this->dataPtr->pub)
  {
    ignerr << "Unable to create publisher on topic[" << this->Topic() << "].\n";
    return false;
  }

  // Load the noise parameters

  if (_sdf.GpsSensor()->HorizontalPositionNoise().Type()
      != sdf::NoiseType::NONE)
  {
    this->dataPtr->noises[GPS_HORIZONTAL_POSITION_NOISE] =
      NoiseFactory::NewNoiseModel(
        _sdf.GpsSensor()->HorizontalPositionNoise());
  }
  if (_sdf.GpsSensor()->VerticalPositionNoise().Type()
      != sdf::NoiseType::NONE)
  {
    this->dataPtr->noises[GPS_VERTICAL_POSITION_NOISE] =
      NoiseFactory::NewNoiseModel(
        _sdf.GpsSensor()->VerticalPositionNoise());
  }
  if (_sdf.GpsSensor()->HorizontalVelocityNoise().Type()
      != sdf::NoiseType::NONE)
  {
    this->dataPtr->noises[GPS_HORIZONTAL_VELOCITY_NOISE] =
      NoiseFactory::NewNoiseModel(
        _sdf.GpsSensor()->HorizontalVelocityNoise());
  }
  if (_sdf.GpsSensor()->VerticalVelocityNoise().Type()
      != sdf::NoiseType::NONE)
  {
    this->dataPtr->noises[GPS_VERTICAL_VELOCITY_NOISE] =
      NoiseFactory::NewNoiseModel(
        _sdf.GpsSensor()->VerticalVelocityNoise());
  }

  this->dataPtr->initialized = true;
  return true;
}

//////////////////////////////////////////////////
bool GpsSensor::Load(sdf::ElementPtr _sdf)
{
  sdf::Sensor sdfSensor;
  sdfSensor.Load(_sdf);
  return this->Load(sdfSensor);
}

//////////////////////////////////////////////////
bool GpsSensor::Update(
  const ignition::common::Time &_now)
{
  return this->Update(math::secNsecToDuration(_now.sec, _now.nsec));
}

//////////////////////////////////////////////////
bool GpsSensor::Update(const std::chrono::steady_clock::duration &_now)
{
  IGN_PROFILE("GpsSensor::Update");
  if (!this->dataPtr->initialized)
  {
    ignerr << "Not initialized, update ignored.\n";
    return false;
  }

  msgs::GPS msg;

  *msg.mutable_header()->mutable_stamp() = msgs::Convert(_now);
  auto frame = msg.mutable_header()->add_data();
  frame->set_key("frame_id");
  frame->add_value(this->Name());

  // Apply gps horizontal position noise
  if (this->dataPtr->noises.find(GPS_HORIZONTAL_POSITION_NOISE) !=
      this->dataPtr->noises.end())
  {
    this->SetLatitude(
      this->dataPtr->noises[GPS_HORIZONTAL_POSITION_NOISE]->Apply(
        this->Latitude()));

    this->SetLongitude(
      this->dataPtr->noises[GPS_HORIZONTAL_POSITION_NOISE]->Apply(
        this->Longitude()));
  }

  if (this->dataPtr->noises.find(GPS_VERTICAL_POSITION_NOISE) !=
      this->dataPtr->noises.end())
  {
    this->SetAltitude(
      this->dataPtr->noises[GPS_VERTICAL_POSITION_NOISE]->Apply(
        this->Altitude()));
  }

  // taken from ImuSensor.cc - convenience method
  auto applyNoise = [&](SensorNoiseType noiseType, double &value)
  {
    if (this->dataPtr->noises.find(noiseType) != this->dataPtr->noises.end()){
      value = this->dataPtr->noises[noiseType]->Apply(value);
    }
  };

  applyNoise(GPS_HORIZONTAL_VELOCITY_NOISE, this->dataPtr->velocity.X());
  applyNoise(GPS_HORIZONTAL_VELOCITY_NOISE, this->dataPtr->velocity.Y());
  applyNoise(GPS_VERTICAL_VELOCITY_NOISE, this->dataPtr->velocity.Z());

  // normalise so that it is within +/- 180
  this->dataPtr->latitude.Normalize();
  this->dataPtr->longitude.Normalize();


  msg.set_latitude_deg(this->dataPtr->latitude.Degree());
  msg.set_longitude_deg(this->dataPtr->longitude.Degree());
  msg.set_altitude(this->dataPtr->altitude);
  msg.set_velocity_east(this->dataPtr->velocity.X());
  msg.set_velocity_north(this->dataPtr->velocity.X());
  msg.set_velocity_up(this->dataPtr->velocity.X());

  // publish
  this->AddSequence(msg.mutable_header());
  this->dataPtr->pub.Publish(msg);

  return true;
}

void   GpsSensor::SetLatitude(double _latitude)
{
  this->dataPtr->latitude.Degree(_latitude);
}

double GpsSensor::Latitude() const
{
  return this->dataPtr->latitude.Degree();
}

void   GpsSensor::SetAltitude(double _altitude)
{
  this->dataPtr->altitude = _altitude;
}

double GpsSensor::Altitude() const
{
  return this->dataPtr->altitude;
}

void   GpsSensor::SetLongitude(double _longitude)
{
  this->dataPtr->longitude.Degree(_longitude);
}

double GpsSensor::Longitude() const
{
  return this->dataPtr->longitude.Degree();
}

void   GpsSensor::SetVelocity(ignition::math::Vector3d &_vel)
{
  this->dataPtr->velocity = _vel;
}

ignition::math::Vector3d GpsSensor::Velocity() const
{
  return this->dataPtr->velocity;
}

void   GpsSensor::SetPosition(double _latitude, double _longitude,
 double _altitude)
{
  this->SetLongitude(_longitude);
  this->SetLatitude(_latitude);
  this->SetAltitude(_altitude);
}

IGN_SENSORS_REGISTER_SENSOR(GpsSensor)
