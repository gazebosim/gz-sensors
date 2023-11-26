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
#include <gz/msgs/air_flow_sensor.pb.h>
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
#include "gz/sensors/AirFlowSensor.hh"

using namespace gz;
using namespace sensors;

/// \brief Private data for AirFlowSensor
class gz::sensors::AirFlowSensorPrivate
{
  /// \brief node to create publisher
  public: transport::Node node;

  /// \brief publisher to publish air speed messages.
  public: transport::Node::Publisher pub;

  /// \brief true if Load() has been called and was successful
  public: bool initialized = false;

  /// \brief Pressure in pascals.
  public: double pressure = 0.0;

  /// \brief Resolution of: [deg]
  public: double direction_resolution = 0.0;

  /// \brief Pressure in pascals.
  public: double speed_resolution = 0.0;

  /// \brief Velocity of the air coming from the sensor
  public: gz::math::Vector3d vel;

  /// \brief Velocity of the wind
  public: gz::math::Vector3d wind_vel;

  /// \brief Noise added to speed measurement
  public: std::map<SensorNoiseType, NoisePtr> speed_noises;
  
  /// \brief Noise added to directional measurement
  public: std::map<SensorNoiseType, NoisePtr> dir_noises;
};

//////////////////////////////////////////////////
AirFlowSensor::AirFlowSensor()
  : dataPtr(new AirFlowSensorPrivate())
{
}

//////////////////////////////////////////////////
AirFlowSensor::~AirFlowSensor()
{
}

//////////////////////////////////////////////////
bool AirFlowSensor::Init()
{
  return this->Sensor::Init();
}

//////////////////////////////////////////////////
bool AirFlowSensor::Load(const sdf::Sensor &_sdf)
{
  if (!Sensor::Load(_sdf))
    return false;

  if (_sdf.Type() != sdf::SensorType::air_flow)
  {
    gzerr << "Attempting to a load an AirFlow sensor, but received "
      << "a " << _sdf.TypeStr() << std::endl;
    return false;
  }

  if (_sdf.AirFlowSensor() == nullptr)
  {
    gzerr << "Attempting to a load an AirFlow sensor, but received "
      << "a null sensor." << std::endl;
    return false;
  }

  if (this->Topic().empty())
    this->SetTopic("/air_flow");

  this->dataPtr->pub =
      this->dataPtr->node.Advertise<msgs::AirFlowSensor>(
      this->Topic());

  if (!this->dataPtr->pub)
  {
    gzerr << "Unable to create publisher on topic[" << this->Topic() << "].\n";
    return false;
  }

  gzdbg << "Airflow for [" << this->Name() << "] advertised on ["
         << this->Topic() << "]" << std::endl;

  // Load the noise parameters
  if (_sdf.AirFlowSensor()->SpeedNoise().Type() != sdf::NoiseType::NONE)
  {
    this->dataPtr->speed_noises[AIR_FLOW_SPEED_NOISE_PASCALS] =
      NoiseFactory::NewNoiseModel(_sdf.AirFlowSensor()->SpeedNoise());
  }

  if (_sdf.AirFlowSensor()->DirectionNoise().Type() != sdf::NoiseType::NONE)
  {
    this->dataPtr->dir_noises[AIR_FLOW_DIR_NOISE_PASCALS] =
      NoiseFactory::NewNoiseModel(_sdf.AirFlowSensor()->DirectionNoise());
  }

  this->dataPtr->initialized = true;
  return true;
}

//////////////////////////////////////////////////
bool AirFlowSensor::Load(sdf::ElementPtr _sdf)
{
  sdf::Sensor sdfSensor;
  sdfSensor.Load(_sdf);
  return this->Load(sdfSensor);
}

//////////////////////////////////////////////////
bool AirFlowSensor::Update(
  const std::chrono::steady_clock::duration &_now)
{
  GZ_PROFILE("AirFlowSensor::Update");
  if (!this->dataPtr->initialized)
  {
    gzerr << "Not initialized, update ignored.\n";
    return false;
  }

  msgs::AirFlowSensor msg;
  *msg.mutable_header()->mutable_stamp() = msgs::Convert(_now);
  auto frame = msg.mutable_header()->add_data();
  frame->set_key("frame_id");
  frame->add_value(this->FrameId());

  math::Vector3d wind_vel_ = this-dataPtr->wind_vel;
  math::Quaterniond veh_q_world_to_body = this->Pose().Rot();

  // calculate differential pressure + noise in hPa
  math::Vector3d air_vel_in_body_ = this->dataPtr->vel -
    veh_q_world_to_body.RotateVectorReverse(wind_vel_);

  double airflow_direction_in_xy_plane = atan2f(air_vel_in_body_.Y(),
                                              air_vel_in_body_.X());

  // Apply noise
  if (this->dataPtr->dir_noises.find(air_flow_dir_NOISE_PASCALS) !=
      this->dataPtr->dir_noises.end())
  {
    airflow_direction_in_xy_plane =
      this->dataPtr->dir_noises[air_flow_dir_NOISE_PASCALS]->Apply(
          airflow_direction_in_xy_plane);
    msg.mutable_direction_noise()->set_type(msgs::SensorNoise::GAUSSIAN);
  }

  // apply resolution to sensor measurement


  air_vel_in_body_.Z() = 0;
  double airflow_speed =  air_vel_in_body.Length();

  // Apply noise
  if (this->dataPtr->speed_noises.find(air_flow_speed_NOISE_PASCALS) !=
      this->dataPtr->speed_noises.end())
  {
    airflow_speed =
      this->dataPtr->speed_noises[air_flow_speed_NOISE_PASCALS]->Apply(
          airflow_direction_in_xy_plane);
    msg.mutable_speed_noise()->set_type(msgs::SensorNoise::GAUSSIAN);
  }

  // apply resolution to sensor measurement



  msg.set_xy_speed(airflow_direction_in_xy_plane);
  msg.set_xy_direction(airflow_speed);

  // publish
  this->AddSequence(msg.mutable_header());
  this->dataPtr->pub.Publish(msg);

  return true;
}

//////////////////////////////////////////////////
gz::math::Vector3d AirFlowSensor::Velocity() const
{
  return this->dataPtr->vel;
}

//////////////////////////////////////////////////
void AirFlowSensor::SetVelocity(const gz::math::Vector3d &_vel)
{
  this->dataPtr->vel = _vel;
}

void AirFlowSensor::SetWindVelocity(const gz::math::Vector3d &_vel)
{
  this->dataPtr->wind_vel = _vel;
}

//////////////////////////////////////////////////
bool AirFlowSensor::HasConnections() const
{
  return this->dataPtr->pub && this->dataPtr->pub.HasConnections();
}
