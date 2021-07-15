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

  /// \brief Which orientation we support for returning sensor measure
      public: enum MeasureFrame
      {
        PARENT_LINK,
        CHILD_LINK,
        SENSOR
      };
  
  /// \brief Frame in which we return the measured force torque info.
      public: MeasureFrame measureFrame;

  /// \brief Direction of the measure
      ///        True if the measured force torque is the one applied
      ///        by the parent on the child, false otherwise
      public: bool parentToChild;

  /// \brief Rotation matrix than transforms a vector expressed in child
      ///        orientation in a vector expressed in joint orientation.
      ///        Necessary is the measure is specified in joint frame.
      public: ignition::math::Matrix3d rotationSensorChild;


  /// \brief Flag for if time has been initialized
  public: bool timeInitialized = false;

  /// \brief Previous update time step.
  public: std::chrono::steady_clock::duration prevStep
    {std::chrono::steady_clock::duration::zero()};

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

  if (this->Topic().empty())
    this->SetTopic("/forcetorque");

  this->dataPtr->pub =
      this->dataPtr->node.Advertise<ignition::msgs::Wrench>(this->Topic());

  if (!this->dataPtr->pub)
  {
    ignerr << "Unable to create publisher on topic[" << this->Topic() << "].\n";
    return false;
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

  msgs::Wrench msg;
  *msg.mutable_header()->mutable_stamp() = msgs::Convert(_now);
  auto frame = msg.mutable_header()->add_data();
  frame->set_key("frame_id");
  frame->add_value(this->Name());

  // Get the force and torque in the appropriate frame.
  ignition::math::Vector3d measuredForce;
  ignition::math::Vector3d measuredTorque;

  if (this->dataPtr->measureFrame == ForceTorqueSensorPrivate::PARENT_LINK)
  {
    if (this->dataPtr->parentToChild)
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
  else if (this->dataPtr->measureFrame == ForceTorqueSensorPrivate::CHILD_LINK)
  {
    if (!this->dataPtr->parentToChild)
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
  else
  {
    ignerr << "measureFrame must be PARENT_LINK, CHILD_LINK or SENSOR\n";

    if (!this->dataPtr->parentToChild)
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
