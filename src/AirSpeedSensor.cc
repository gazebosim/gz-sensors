/*
 * Copyright (C) 2023 Open Source Robotics Foundation
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

#if defined(_MSC_VER)
  #pragma warning(push)
  #pragma warning(disable: 4005)
  #pragma warning(disable: 4251)
#endif
#include <gz/msgs/air_speed_sensor.pb.h>
#if defined(_MSC_VER)
  #pragma warning(pop)
#endif
#include <gz/msgs/Utility.hh>

#include <gz/common/Profiler.hh>
#include <gz/transport/Node.hh>

#include "gz/sensors/GaussianNoiseModel.hh"
#include "gz/sensors/Noise.hh"
#include "gz/sensors/SensorTypes.hh"
#include "gz/sensors/SensorFactory.hh"
#include "gz/sensors/AirSpeedSensor.hh"

using namespace gz;
using namespace sensors;

static constexpr auto kDefaultHomeAltAmsl = 0.0f; // altitude AMSL see level [m]

// international standard atmosphere (troposphere model - valid up to 11km).
// temperature at MSL [K] (15 [C])
static constexpr auto kTemperaturMsl = 288.15f;
// pressure at MSL [Pa]
static constexpr auto kPressureMsl = 101325.0f;
// reduction in temperature with altitude for troposphere [K/m]
static constexpr auto kLapseRate = 0.0065f;
// air density at MSL [kg/m^3]
static constexpr auto kAirDensityMsl = 1.225f;

/// \brief Private data for AirSpeedSensor
class gz::sensors::AirSpeedSensorPrivate
{
  /// \brief node to create publisher
  public: transport::Node node;

  /// \brief publisher to publish air speed messages.
  public: transport::Node::Publisher pub;

  /// \brief true if Load() has been called and was successful
  public: bool initialized = false;

  /// \brief Pressure in pascals.
  public: double pressure = 0.0;

  /// \brief Velocity of the air coming from the sensor
  public: gz::math::Vector3d vel;

  /// \brief Noise added to sensor data
  public: std::map<SensorNoiseType, NoisePtr> noises;
};

//////////////////////////////////////////////////
AirSpeedSensor::AirSpeedSensor()
  : dataPtr(new AirSpeedSensorPrivate())
{
}

//////////////////////////////////////////////////
AirSpeedSensor::~AirSpeedSensor()
{
}

//////////////////////////////////////////////////
bool AirSpeedSensor::Init()
{
  return this->Sensor::Init();
}

//////////////////////////////////////////////////
bool AirSpeedSensor::Load(const sdf::Sensor &_sdf)
{
  if (!Sensor::Load(_sdf))
    return false;

  if (_sdf.Type() != sdf::SensorType::AIR_SPEED)
  {
    gzerr << "Attempting to a load an AirSpeed sensor, but received "
      << "a " << _sdf.TypeStr() << std::endl;
    return false;
  }

  if (_sdf.AirSpeedSensor() == nullptr)
  {
    gzerr << "Attempting to a load an AirSpeed sensor, but received "
      << "a null sensor." << std::endl;
    return false;
  }

  if (this->Topic().empty())
    this->SetTopic("/air_speed");

  this->dataPtr->pub =
      this->dataPtr->node.Advertise<msgs::AirSpeedSensor>(
      this->Topic());

  if (!this->dataPtr->pub)
  {
    gzerr << "Unable to create publisher on topic[" << this->Topic() << "].\n";
    return false;
  }

  gzdbg << "Air speed for [" << this->Name() << "] advertised on ["
         << this->Topic() << "]" << std::endl;

  // Load the noise parameters
  if (_sdf.AirSpeedSensor()->PressureNoise().Type() != sdf::NoiseType::NONE)
  {
    this->dataPtr->noises[AIR_SPEED_NOISE_PASCALS] =
      NoiseFactory::NewNoiseModel(_sdf.AirSpeedSensor()->PressureNoise());
  }

  this->dataPtr->initialized = true;
  return true;
}

//////////////////////////////////////////////////
bool AirSpeedSensor::Load(sdf::ElementPtr _sdf)
{
  sdf::Sensor sdfSensor;
  sdfSensor.Load(_sdf);
  return this->Load(sdfSensor);
}

//////////////////////////////////////////////////
bool AirSpeedSensor::Update(
  const std::chrono::steady_clock::duration &_now)
{
  GZ_PROFILE("AirSpeedSensor::Update");
  if (!this->dataPtr->initialized)
  {
    gzerr << "Not initialized, update ignored.\n";
    return false;
  }

  msgs::AirSpeedSensor msg;
  *msg.mutable_header()->mutable_stamp() = msgs::Convert(_now);
  auto frame = msg.mutable_header()->add_data();
  frame->set_key("frame_id");
  frame->add_value(this->FrameId());

  // compute the air density at the local altitude / temperature
  // Z-component from ENU
  const float alt_rel = static_cast<float>(this->Pose().Pos().Z());
  const float alt_amsl = kDefaultHomeAltAmsl + alt_rel;
  const float temperature_local = kTemperaturMsl - kLapseRate * alt_amsl;
  const float density_ratio = powf(kTemperaturMsl / temperature_local , 4.256f);
  const float air_density = kAirDensityMsl / density_ratio;

  math::Vector3d wind_vel_{0, 0, 0};
  math::Quaterniond veh_q_world_to_body = this->Pose().Rot();

  // calculate differential pressure + noise in hPa
  math::Vector3d air_vel_in_body_ = this->dataPtr->vel -
    veh_q_world_to_body.RotateVectorReverse(wind_vel_);
  float diff_pressure = math::sgn(air_vel_in_body_.X()) * 0.005f * air_density
    * air_vel_in_body_.X() * air_vel_in_body_.X();

  // Apply pressure noise
  if (this->dataPtr->noises.find(AIR_SPEED_NOISE_PASCALS) !=
      this->dataPtr->noises.end())
  {
    diff_pressure =
      this->dataPtr->noises[AIR_SPEED_NOISE_PASCALS]->Apply(
          diff_pressure);
    msg.mutable_pressure_noise()->set_type(msgs::SensorNoise::GAUSSIAN);
  }

  msg.set_diff_pressure(diff_pressure * 100.0f);
  msg.set_temperature(temperature_local);

  // publish
  this->AddSequence(msg.mutable_header());
  this->dataPtr->pub.Publish(msg);

  return true;
}

//////////////////////////////////////////////////
gz::math::Vector3d AirSpeedSensor::Velocity() const
{
  return this->dataPtr->vel;
}

//////////////////////////////////////////////////
void AirSpeedSensor::SetVelocity(const gz::math::Vector3d &_vel)
{
  this->dataPtr->vel = _vel;
}

//////////////////////////////////////////////////
bool AirSpeedSensor::HasConnections() const
{
  return this->dataPtr->pub && this->dataPtr->pub.HasConnections();
}
