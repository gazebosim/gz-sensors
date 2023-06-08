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

#include <unordered_map>

#ifdef _WIN32
#pragma warning(push)
#pragma warning(disable: 4005)
#pragma warning(disable: 4251)
#endif
#include <gz/msgs/navsat.pb.h>
#ifdef _WIN32
#pragma warning(pop)
#endif

#include <gz/common/Profiler.hh>
#include <gz/math/Angle.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/Utility.hh>
#include <gz/transport/Node.hh>

#include "gz/sensors/NavSatSensor.hh"
#include "gz/sensors/Noise.hh"
#include "gz/sensors/SensorFactory.hh"
#include "gz/sensors/SensorTypes.hh"

using namespace gz;
using namespace sensors;

/// \brief Private data for NavSat
class gz::sensors::NavSatPrivate
{
  /// \brief Node to create publisher
  public: transport::Node node;

  /// \brief To publish NavSat messages.
  public: transport::Node::Publisher pub;

  /// \brief True if Load() has been called and was successful
  public: bool loaded = false;

  /// \brief Latitude angle
  public: math::Angle latitude;

  /// \brief Longitude angle
  public: math::Angle longitude;

  /// \brief Altitude
  public: double altitude = 0.0;

  /// \brief Velocity in ENU frame.
  public: math::Vector3d velocity;

  /// \brief Noise added to sensor data
  public: std::unordered_map<SensorNoiseType, NoisePtr> noises;
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
    gzerr << "Attempting to a load an NAVSAT sensor, but received "
      << "a " << _sdf.TypeStr() << std::endl;
    return false;
  }

  if (_sdf.NavSatSensor() == nullptr)
  {
    gzerr << "Attempting to a load an NAVSAT sensor, but received "
      << "a null sensor." << std::endl;
    return false;
  }

  if (this->Topic().empty())
    this->SetTopic("/navsat");

  this->dataPtr->pub =
      this->dataPtr->node.Advertise<msgs::NavSat>(this->Topic());

  if (!this->dataPtr->pub)
  {
    gzerr << "Unable to create publisher on topic [" << this->Topic()
           << "]." << std::endl;
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

  this->dataPtr->loaded = true;
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
  GZ_PROFILE("NavSatSensor::Update");
  if (!this->dataPtr->loaded)
  {
    gzerr << "Not loaded, update ignored.\n";
    return false;
  }

  msgs::NavSat msg;
  *msg.mutable_header()->mutable_stamp() = msgs::Convert(_now);
  msg.set_frame_id(this->FrameId());

  // Apply noise
  auto iter = this->dataPtr->noises.find(NAVSAT_HORIZONTAL_POSITION_NOISE);
  if (iter != this->dataPtr->noises.end())
  {
    this->SetLatitude(GZ_DTOR(iter->second->Apply(this->Latitude().Degree())));
    this->SetLongitude(GZ_DTOR(iter->second->Apply(
        this->Longitude().Degree())));
  }
  iter = this->dataPtr->noises.find(NAVSAT_VERTICAL_POSITION_NOISE);
  if (iter != this->dataPtr->noises.end())
  {
    this->SetAltitude(iter->second->Apply(this->Altitude()));
  }
  iter = this->dataPtr->noises.find(NAVSAT_HORIZONTAL_VELOCITY_NOISE);
  if (iter != this->dataPtr->noises.end())
  {
    this->dataPtr->velocity.X(iter->second->Apply(this->dataPtr->velocity.X()));
    this->dataPtr->velocity.Y(iter->second->Apply(this->dataPtr->velocity.Y()));
  }
  iter = this->dataPtr->noises.find(NAVSAT_VERTICAL_VELOCITY_NOISE);
  if (iter != this->dataPtr->noises.end())
  {
    this->dataPtr->velocity.Z(iter->second->Apply(this->dataPtr->velocity.Z()));
  }

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

//////////////////////////////////////////////////
bool NavSatSensor::HasConnections() const
{
  return this->dataPtr->pub && this->dataPtr->pub.HasConnections();
}

