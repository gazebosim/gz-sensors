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

#ifdef _WIN32
#pragma warning(push)
#pragma warning(disable: 4005)
#pragma warning(disable: 4251)
#endif
#include <ignition/msgs/wrench.pb.h>
#ifdef _WIN32
#pragma warning(pop)
#endif

#include <ignition/common/Profiler.hh>
#include <ignition/transport/Node.hh>

#include "ignition/sensors/ForceTorqueSensor.hh"
#include "ignition/sensors/Noise.hh"
#include "ignition/sensors/SensorFactory.hh"
#include "ignition/sensors/SensorTypes.hh"

using namespace ignition;
using namespace sensors;

/// \brief Private data for ForceTorqueSensor
class ignition::sensors::ForceTorqueSensorPrivate
{
  /// \brief node to create publisher
  public: transport::Node node;

  /// \brief publisher to publish Wrench messages.
  public: transport::Node::Publisher pub;

  /// \brief true if Load() has been called and was successful
  public: bool initialized = false;

  /// \brief Noise free force
  public: ignition::math::Vector3d force;

  /// \brief Noise free torque
  public: ignition::math::Vector3d torque;
  
  /// \brief Frame in which we return the measured force torque info.
  public: sdf::ForceTorqueFrame measureFrame;

  /// \brief Direction in which we return the measured force torque info.
  public: sdf::ForceTorqueMeasureDirection measureDirection;

  /// \brief Rotation matrix than transforms a vector expressed in child
      ///        orientation in a vector expressed in joint orientation.
      ///        Necessary is the measure is specified in joint frame.
  public: ignition::math::Matrix3d rotationSensorChild;

  /// \brief Flag for if time has been initialized
  public: bool timeInitialized = false;

  /// \brief Previous update time step.
  public: std::chrono::steady_clock::duration prevStep
    {std::chrono::steady_clock::duration::zero()};

  /// \brief Noise added to sensor data
  public: std::map<SensorNoiseType, NoisePtr> noises;
};

//////////////////////////////////////////////////
ForceTorqueSensor::ForceTorqueSensor()
  : dataPtr(new ForceTorqueSensorPrivate())
{
}

//////////////////////////////////////////////////
ForceTorqueSensor::~ForceTorqueSensor()
{
}

//////////////////////////////////////////////////
bool ForceTorqueSensor::Init()
{
  return this->Sensor::Init();
}

//////////////////////////////////////////////////
bool ForceTorqueSensor::Load(const sdf::Sensor &_sdf)
{
  if (!Sensor::Load(_sdf))
    return false;

  // Check if this is the right type
  if (_sdf.Type() != sdf::SensorType::FORCE_TORQUE)
  {
    ignerr << "Attempting to a load a Force Torque sensor, but received "
      << "a " << _sdf.TypeStr() << std::endl;
  }

  if (_sdf.ForceTorqueSensor() == nullptr)
  {
    ignerr << "Attempting to a load a Force Torque sensor, but received "
      << "a null sensor." << std::endl;
    return false;
  }

  this->dataPtr->measureFrame = _sdf.ForceTorqueSensor()->Frame();
  this->dataPtr->measureDirection = _sdf.ForceTorqueSensor()->MeasureDirection();

  if (this->Topic().empty())
    this->SetTopic("/forcetorque");

  this->dataPtr->pub =
      this->dataPtr->node.Advertise<ignition::msgs::Wrench>(this->Topic());

  if (!this->dataPtr->pub)
  {
    ignerr << "Unable to create publisher on topic[" << this->Topic() << "].\n";
    return false;
  }

  const std::map<SensorNoiseType, sdf::Noise> noises = {
    {FORCE_X_NOISE_N, _sdf.ForceTorqueSensor()->ForceXNoise()},
    {FORCE_Y_NOISE_N, _sdf.ForceTorqueSensor()->ForceYNoise()},
    {FORCE_Z_NOISE_N, _sdf.ForceTorqueSensor()->ForceZNoise()},
    {TORQUE_X_NOISE_N_M, _sdf.ForceTorqueSensor()->TorqueXNoise()},
    {TORQUE_Y_NOISE_N_M, _sdf.ForceTorqueSensor()->TorqueYNoise()},
    {TORQUE_Z_NOISE_N_M, _sdf.ForceTorqueSensor()->TorqueZNoise()},
  };

  for (const auto & [noiseType, noiseSdf] : noises)
  {
    if (noiseSdf.Type() != sdf::NoiseType::NONE)
    {
      this->dataPtr->noises[noiseType] = NoiseFactory::NewNoiseModel(noiseSdf);
    }
  }

  this->dataPtr->initialized = true;
  return true;
}

//////////////////////////////////////////////////
bool ForceTorqueSensor::Load(sdf::ElementPtr _sdf)
{
  sdf::Sensor sdfSensor;
  sdfSensor.Load(_sdf);
  return this->Load(sdfSensor);
}

//////////////////////////////////////////////////
bool ForceTorqueSensor::Update(
  const ignition::common::Time &_now)
{
  return this->Update(math::secNsecToDuration(_now.sec, _now.nsec));
}

//////////////////////////////////////////////////
bool ForceTorqueSensor::Update(const std::chrono::steady_clock::duration &_now)
{
  IGN_PROFILE("ForceTorqueSensor::Update");
  if (!this->dataPtr->initialized)
  {
    ignerr << "Not initialized, update ignored.\n";
    return false;
  }

  // If time has gone backwards, reinitialize.
  if (_now < this->dataPtr->prevStep)
  {
    this->dataPtr->timeInitialized = false;
  }

  // Only compute dt if time is initialized and increasing.
  double dt;
  if (this->dataPtr->timeInitialized)
  {
    auto delay = std::chrono::duration_cast<std::chrono::duration<float>>(
        _now - this->dataPtr->prevStep);
    dt = delay.count();
  }
  else
  {
    dt = 0.0;
  }

  // Convenience method to apply noise to a channel, if present.
  auto applyNoise = [&](SensorNoiseType noiseType, double & value)
  {
    if (this->dataPtr->noises.find(noiseType) != this->dataPtr->noises.end()) {
      value = this->dataPtr->noises[noiseType]->Apply(value, dt);
    }
  };

  applyNoise(FORCE_X_NOISE_N, this->dataPtr->force.X());
  applyNoise(FORCE_Y_NOISE_N, this->dataPtr->force.Y());
  applyNoise(FORCE_Z_NOISE_N, this->dataPtr->force.Z());
  applyNoise(TORQUE_X_NOISE_N_M, this->dataPtr->torque.X());
  applyNoise(TORQUE_Y_NOISE_N_M, this->dataPtr->torque.Y());
  applyNoise(TORQUE_Z_NOISE_N_M, this->dataPtr->torque.Z());

  msgs::Wrench msg;
  *msg.mutable_header()->mutable_stamp() = msgs::Convert(_now);
  auto frame = msg.mutable_header()->add_data();
  frame->set_key("frame_id");
  frame->add_value(this->Name());

  // Get the force and torque in the appropriate frame.
  ignition::math::Vector3d measuredForce;
  ignition::math::Vector3d measuredTorque;

  if (this->dataPtr->measureFrame == sdf::ForceTorqueFrame::PARENT)
  {
    if (this->dataPtr->measureDirection == sdf::ForceTorqueMeasureDirection::PARENT_TO_CHILD)
    {
      measuredForce = this->dataPtr->force;
      measuredTorque = this->dataPtr->torque;
    }
    else
    {
      measuredForce = -1*this->dataPtr->force;
      measuredTorque = -1*this->dataPtr->torque;
    }
  }
  else if (this->dataPtr->measureFrame == sdf::ForceTorqueFrame::CHILD)
  {
    if (this->dataPtr->measureDirection == sdf::ForceTorqueMeasureDirection::CHILD_TO_PARENT)
    {
      measuredForce = this->dataPtr->force;
      measuredTorque = this->dataPtr->torque;
      ignerr << "Warning: ForceTorqueSensor::Update() " << std::endl;
    }
    else
    {
      measuredForce = -1*this->dataPtr->force;
      measuredTorque = -1*this->dataPtr->torque;
    }
  }
  else
  {
    ignerr << "measureFrame must be PARENT_LINK, CHILD_LINK or SENSOR\n";

    if (this->dataPtr->measureDirection == sdf::ForceTorqueMeasureDirection::CHILD_TO_PARENT)
    {
      measuredForce = this->dataPtr->rotationSensorChild *
        this->dataPtr->force;
      measuredTorque = this->dataPtr->rotationSensorChild *
        this->dataPtr->torque;
    }
    else
    {
      measuredForce = this->dataPtr->rotationSensorChild *
        (-1*this->dataPtr->force);
      measuredTorque = this->dataPtr->rotationSensorChild *
        (-1*this->dataPtr->torque);
    }
  }

  msgs::Set(msg.mutable_force(), measuredForce);
  msgs::Set(msg.mutable_torque(), measuredTorque);

  // publish
  this->AddSequence(msg.mutable_header());
  this->dataPtr->pub.Publish(msg);
  this->dataPtr->prevStep = _now;
  this->dataPtr->timeInitialized = true;
  return true;
}

//////////////////////////////////////////////////
math::Vector3d ForceTorqueSensor::Force() const
{
  return this->dataPtr->force;
}

//////////////////////////////////////////////////
void ForceTorqueSensor::SetForce(const math::Vector3d &_force)
{
  this->dataPtr->force = _force;
}

//////////////////////////////////////////////////
math::Vector3d ForceTorqueSensor::Torque() const
{
  return this->dataPtr->torque;
}

//////////////////////////////////////////////////
void ForceTorqueSensor::SetTorque(const math::Vector3d &_torque)
{
  this->dataPtr->torque = _torque;
}


IGN_SENSORS_REGISTER_SENSOR(ForceTorqueSensor)
