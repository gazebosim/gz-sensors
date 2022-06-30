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

#include <gz/msgs/wrench.pb.h>
#include <gz/msgs/Utility.hh>

#include <gz/common/Profiler.hh>
#include <gz/transport/Node.hh>

#include "gz/sensors/ForceTorqueSensor.hh"
#include "gz/sensors/Noise.hh"
#include "gz/sensors/SensorFactory.hh"
#include "gz/sensors/SensorTypes.hh"

using namespace gz;
using namespace sensors;

/// \brief Private data for ForceTorqueSensor
class gz::sensors::ForceTorqueSensorPrivate
{
  /// \brief node to create publisher
  public: transport::Node node;

  /// \brief publisher to publish Wrench messages.
  public: transport::Node::Publisher pub;

  /// \brief true if Load() has been called and was successful
  public: bool initialized = false;

  /// \brief Noise free force as set by SetForce
  public: gz::math::Vector3d force{0, 0, 0};

  /// \brief Noise free torque as set by SetTorque
  public: gz::math::Vector3d torque{0, 0, 0};

  /// \brief Frame in which we return the measured force torque info.
  public: sdf::ForceTorqueFrame measureFrame;

  /// \brief Direction in which we return the measured force torque info.
  public: sdf::ForceTorqueMeasureDirection measureDirection;

  /// \brief Rotation matrix that transforms a vector expressed in the parent
  ///  frame to a vector expressed in the sensor frame.
  ///  \note We store the rotation as a 3x3 matrix because matrix-vector
  ///  product is than quaternion-vector when there are a lot of vectors and the
  ///  rotation is not changing frequently.
  public: gz::math::Matrix3d rotationParentInSensor{
              gz::math::Matrix3d::Identity};

  /// \brief Rotation matrix that transforms a vector expressed in the child
  /// frame to a vector expressed in the sensor frame.
  ///  \note We store the rotation as a 3x3 matrix because matrix-vector
  ///  product is than quaternion-vector when there are a lot of vectors and the
  ///  rotation is not changing frequently.
  public: gz::math::Matrix3d rotationChildInSensor{
              gz::math::Matrix3d::Identity};

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
  : dataPtr(std::make_unique<ForceTorqueSensorPrivate>())
{
}

//////////////////////////////////////////////////
ForceTorqueSensor::~ForceTorqueSensor() = default;

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
    gzerr << "Attempting to a load a Force Torque sensor, but received "
      << "a " << _sdf.TypeStr() << std::endl;
  }

  if (_sdf.ForceTorqueSensor() == nullptr)
  {
    gzerr << "Attempting to a load a Force Torque sensor, but received "
      << "a null sensor." << std::endl;
    return false;
  }

  this->dataPtr->measureFrame = _sdf.ForceTorqueSensor()->Frame();
  this->dataPtr->measureDirection =
      _sdf.ForceTorqueSensor()->MeasureDirection();

  if (this->Topic().empty())
    this->SetTopic("/forcetorque");

  this->dataPtr->pub =
      this->dataPtr->node.Advertise<gz::msgs::Wrench>(this->Topic());

  if (!this->dataPtr->pub)
  {
    gzerr << "Unable to create publisher on topic[" << this->Topic() << "].\n";
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
bool ForceTorqueSensor::Update(const std::chrono::steady_clock::duration &_now)
{
  GZ_PROFILE("ForceTorqueSensor::Update");
  if (!this->dataPtr->initialized)
  {
    gzerr << "Not initialized, update ignored.\n";
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
  // Get the force and torque in the appropriate frame.
  gz::math::Vector3d measuredForce;
  gz::math::Vector3d measuredTorque;

  if (this->dataPtr->measureFrame == sdf::ForceTorqueFrame::PARENT)
  {
    measuredForce =
        this->dataPtr->rotationParentInSensor.Inverse() * this->dataPtr->force;
    measuredTorque =
        this->dataPtr->rotationParentInSensor.Inverse() * this->dataPtr->torque;
  }
  else if (this->dataPtr->measureFrame == sdf::ForceTorqueFrame::CHILD)
  {
    measuredForce =
        this->dataPtr->rotationChildInSensor.Inverse() * this->dataPtr->force;
    measuredTorque =
        this->dataPtr->rotationChildInSensor.Inverse() * this->dataPtr->torque;
  }
  else if (this->dataPtr->measureFrame == sdf::ForceTorqueFrame::SENSOR)
  {
    measuredForce = this->dataPtr->force;
    measuredTorque = this->dataPtr->torque;
  }
  else
  {
    gzerr << "measureFrame must be PARENT_LINK, CHILD_LINK or SENSOR\n";
  }

  if (this->dataPtr->measureDirection ==
      sdf::ForceTorqueMeasureDirection::CHILD_TO_PARENT)
  {
    measuredForce *= -1;
    measuredTorque *= -1;
  }

  // Convenience method to apply noise to a channel, if present.
  auto applyNoise = [&](SensorNoiseType noiseType, double &value)
  {
    if (this->dataPtr->noises.find(noiseType) != this->dataPtr->noises.end())
    {
      value = this->dataPtr->noises[noiseType]->Apply(value, dt);
    }
  };

  applyNoise(FORCE_X_NOISE_N, measuredForce.X());
  applyNoise(FORCE_Y_NOISE_N, measuredForce.Y());
  applyNoise(FORCE_Z_NOISE_N, measuredForce.Z());
  applyNoise(TORQUE_X_NOISE_N_M, measuredTorque.X());
  applyNoise(TORQUE_Y_NOISE_N_M, measuredTorque.Y());
  applyNoise(TORQUE_Z_NOISE_N_M, measuredTorque.Z());

  msgs::Wrench msg;
  *msg.mutable_header()->mutable_stamp() = msgs::Convert(_now);
  auto frame = msg.mutable_header()->add_data();
  frame->set_key("frame_id");
  frame->add_value(this->FrameId());

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

//////////////////////////////////////////////////
math::Quaterniond ForceTorqueSensor::RotationParentInSensor() const
{
  return math::Quaterniond(this->dataPtr->rotationParentInSensor);
}

//////////////////////////////////////////////////
void ForceTorqueSensor::SetRotationParentInSensor(
    const math::Quaterniond &_rotParentInSensor)
{
  this->dataPtr->rotationParentInSensor = _rotParentInSensor;
}

//////////////////////////////////////////////////
void ForceTorqueSensor::SetRotationChildInSensor(
    const math::Quaterniond &_rotChildInSensor)
{
  this->dataPtr->rotationChildInSensor = _rotChildInSensor;
}

//////////////////////////////////////////////////
math::Quaterniond ForceTorqueSensor::RotationChildInSensor() const
{
  return math::Quaterniond(this->dataPtr->rotationChildInSensor);
}

//////////////////////////////////////////////////
bool ForceTorqueSensor::HasConnections() const
{
  return this->dataPtr->pub && this->dataPtr->pub.HasConnections();
}
