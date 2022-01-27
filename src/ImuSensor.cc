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
#ifdef _WIN32
#pragma warning(push)
#pragma warning(disable: 4005)
#pragma warning(disable: 4251)
#endif
#include <ignition/msgs/imu.pb.h>
#ifdef _WIN32
#pragma warning(pop)
#endif

#include <ignition/common/Profiler.hh>
#include <ignition/transport/Node.hh>

#include "ignition/sensors/ImuSensor.hh"
#include "ignition/sensors/Noise.hh"
#include "ignition/sensors/SensorFactory.hh"
#include "ignition/sensors/SensorTypes.hh"

using namespace ignition;
using namespace sensors;

/// \brief Private data for ImuSensor
class ignition::sensors::ImuSensorPrivate
{
  /// \brief node to create publisher
  public: transport::Node node;

  /// \brief publisher to publish imu messages.
  public: transport::Node::Publisher pub;

  /// \brief true if Load() has been called and was successful
  public: bool initialized = false;

  /// \brief Noise free linear acceleration
  public: ignition::math::Vector3d linearAcc;

  /// \brief Noise free angular velocity.
  public: ignition::math::Vector3d angularVel;

  /// \brief transform to Imu orientation reference frame.
  public: ignition::math::Quaterniond orientationReference;

  /// \brief transform to Imu frame from Imu reference frame.
  public: ignition::math::Quaterniond orientation;

  /// \brief True to publish orientation data.
  public: bool orientationEnabled = true;

  /// \brief store gravity vector to be added to the IMU output.
  public: ignition::math::Vector3d gravity;

  /// \brief World pose of the imu sensor
  public: ignition::math::Pose3d worldPose;

  /// \brief Flag for if time has been initialized
  public: bool timeInitialized = false;

  /// \brief Orientation of world frame relative to a specified frame
  public: ignition::math::Quaterniond worldRelativeOrientation;

  /// \brief Frame relative-to which the worldRelativeOrientation
  //  is defined
  public: WorldFrameEnumType worldFrameRelativeTo = WorldFrameEnumType::ENU;

  /// \brief Frame relative-to which the sensor orientation is reported
  public: WorldFrameEnumType sensorOrientationRelativeTo;

  /// \brief Frame realtive to which custom_rpy is specified
  public: std::string CustomRpyParentFrame;

  /// \brief Quaternion for to store custom_rpy
  public: ignition::math::Quaterniond CustomRpyQuaternion;

  /// \brief Previous update time step.
  public: std::chrono::steady_clock::duration prevStep
    {std::chrono::steady_clock::duration::zero()};

  /// \brief Noise added to sensor data
  public: std::map<SensorNoiseType, NoisePtr> noises;
};

//////////////////////////////////////////////////
ImuSensor::ImuSensor()
  : dataPtr(new ImuSensorPrivate())
{
}

//////////////////////////////////////////////////
ImuSensor::~ImuSensor()
{
}

//////////////////////////////////////////////////
bool ImuSensor::Init()
{
  return this->Sensor::Init();
}

//////////////////////////////////////////////////
bool ImuSensor::Load(const sdf::Sensor &_sdf)
{
  if (!Sensor::Load(_sdf))
    return false;

  // Check if this is the right type
  if (_sdf.Type() != sdf::SensorType::IMU)
  {
    ignerr << "Attempting to a load an IMU sensor, but received "
      << "a " << _sdf.TypeStr() << std::endl;
  }

  if (_sdf.ImuSensor() == nullptr)
  {
    ignerr << "Attempting to a load an IMU sensor, but received "
      << "a null sensor." << std::endl;
    return false;
  }

  if (this->Topic().empty())
    this->SetTopic("/imu");

  this->dataPtr->pub =
      this->dataPtr->node.Advertise<ignition::msgs::IMU>(this->Topic());

  if (!this->dataPtr->pub)
  {
    ignerr << "Unable to create publisher on topic[" << this->Topic() << "].\n";
    return false;
  }

  igndbg << "IMU data for [" << this->Name() << "] advertised on ["
         << this->Topic() << "]" << std::endl;

  const std::map<SensorNoiseType, sdf::Noise> noises = {
    {ACCELEROMETER_X_NOISE_M_S_S, _sdf.ImuSensor()->LinearAccelerationXNoise()},
    {ACCELEROMETER_Y_NOISE_M_S_S, _sdf.ImuSensor()->LinearAccelerationYNoise()},
    {ACCELEROMETER_Z_NOISE_M_S_S, _sdf.ImuSensor()->LinearAccelerationZNoise()},
    {GYROSCOPE_X_NOISE_RAD_S, _sdf.ImuSensor()->AngularVelocityXNoise()},
    {GYROSCOPE_Y_NOISE_RAD_S, _sdf.ImuSensor()->AngularVelocityYNoise()},
    {GYROSCOPE_Z_NOISE_RAD_S, _sdf.ImuSensor()->AngularVelocityZNoise()},
  };

  for (const auto & [noiseType, noiseSdf] : noises)
  {
    if (noiseSdf.Type() != sdf::NoiseType::NONE)
    {
      this->dataPtr->noises[noiseType] = NoiseFactory::NewNoiseModel(noiseSdf);
    }
  }

  std::string localization = _sdf.ImuSensor()->Localization();

  if (localization == "ENU")
  {
    this->dataPtr->sensorOrientationRelativeTo = WorldFrameEnumType::ENU;
  } else if (localization == "NED") {
    this->dataPtr->sensorOrientationRelativeTo = WorldFrameEnumType::NED;
    std::cout << "add: " << &(this->dataPtr->sensorOrientationRelativeTo) << std::endl;
  } else if (localization == "NWU") {
    this->dataPtr->sensorOrientationRelativeTo = WorldFrameEnumType::NWU;
  } else if (localization == "CUSTOM") {
    this->dataPtr->sensorOrientationRelativeTo = WorldFrameEnumType::CUSTOM;
  } else {
    this->dataPtr->sensorOrientationRelativeTo = WorldFrameEnumType::NONE;
  }

  this->dataPtr->CustomRpyParentFrame = _sdf.ImuSensor()->CustomRpyParentFrame();
  this->dataPtr->CustomRpyQuaternion = ignition::math::Quaterniond(
		                         _sdf.ImuSensor()->CustomRpy());

  this->dataPtr->initialized = true;
  return true;
}

//////////////////////////////////////////////////
bool ImuSensor::Load(sdf::ElementPtr _sdf)
{
  sdf::Sensor sdfSensor;
  sdfSensor.Load(_sdf);
  return this->Load(sdfSensor);
}

//////////////////////////////////////////////////
bool ImuSensor::Update(const std::chrono::steady_clock::duration &_now)
{
  IGN_PROFILE("ImuSensor::Update");
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

  // Add contribution from gravity
  // Skip if gravity is not enabled?
  this->dataPtr->linearAcc -=
      this->dataPtr->worldPose.Rot().Inverse().RotateVector(
      this->dataPtr->gravity);

  // Convenience method to apply noise to a channel, if present.
  auto applyNoise = [&](SensorNoiseType noiseType, double & value)
  {
    if (this->dataPtr->noises.find(noiseType) != this->dataPtr->noises.end()) {
      value = this->dataPtr->noises[noiseType]->Apply(value, dt);
    }
  };

  applyNoise(ACCELEROMETER_X_NOISE_M_S_S, this->dataPtr->linearAcc.X());
  applyNoise(ACCELEROMETER_Y_NOISE_M_S_S, this->dataPtr->linearAcc.Y());
  applyNoise(ACCELEROMETER_Z_NOISE_M_S_S, this->dataPtr->linearAcc.Z());
  applyNoise(GYROSCOPE_X_NOISE_RAD_S, this->dataPtr->angularVel.X());
  applyNoise(GYROSCOPE_Y_NOISE_RAD_S, this->dataPtr->angularVel.Y());
  applyNoise(GYROSCOPE_Z_NOISE_RAD_S, this->dataPtr->angularVel.Z());

  msgs::IMU msg;
  *msg.mutable_header()->mutable_stamp() = msgs::Convert(_now);
  msg.set_entity_name(this->Name());
  auto frame = msg.mutable_header()->add_data();
  frame->set_key("frame_id");
  frame->add_value(this->Name());

  if (this->dataPtr->orientationEnabled)
  {
    // Set the IMU orientation
    // imu orientation with respect to reference frame
    this->dataPtr->orientation =
        this->dataPtr->orientationReference.Inverse() *
        this->dataPtr->worldPose.Rot();

    msgs::Set(msg.mutable_orientation(), this->dataPtr->orientation);
  }
  msgs::Set(msg.mutable_angular_velocity(), this->dataPtr->angularVel);
  msgs::Set(msg.mutable_linear_acceleration(), this->dataPtr->linearAcc);

  // publish
  this->AddSequence(msg.mutable_header());
  this->dataPtr->pub.Publish(msg);
  this->dataPtr->prevStep = _now;
  this->dataPtr->timeInitialized = true;
  return true;
}

//////////////////////////////////////////////////
void ImuSensor::SetAngularVelocity(const math::Vector3d &_angularVel)
{
  this->dataPtr->angularVel = _angularVel;
}

//////////////////////////////////////////////////
math::Vector3d ImuSensor::AngularVelocity() const
{
  return this->dataPtr->angularVel;
}

//////////////////////////////////////////////////
void ImuSensor::SetLinearAcceleration(const math::Vector3d &_linearAcc)
{
  this->dataPtr->linearAcc = _linearAcc;
}

//////////////////////////////////////////////////
math::Vector3d ImuSensor::LinearAcceleration() const
{
  return this->dataPtr->linearAcc;
}

//////////////////////////////////////////////////
void ImuSensor::SetWorldPose(const math::Pose3d _pose)
{
  this->dataPtr->worldPose = _pose;
}

//////////////////////////////////////////////////
math::Pose3d ImuSensor::WorldPose() const
{
  return this->dataPtr->worldPose;
}

//////////////////////////////////////////////////
void ImuSensor::SetWorldFrameOrientation(
  const math::Quaterniond &_rot, WorldFrameEnumType _relativeTo)
{
  std::cout << "Debug 1" << std::endl;
  std::cout << "add: " << &(this->dataPtr->sensorOrientationRelativeTo) << std::endl;
  // Set orientation reference frame if custom_rpy was supplied
  if (this->dataPtr->sensorOrientationRelativeTo == WorldFrameEnumType::CUSTOM)
  {
    if (this->dataPtr->CustomRpyParentFrame == "")
    {
      this->SetOrientationReference(this->dataPtr->CustomRpyQuaternion);
    }
    else
    {
      ignwarn << "custom_rpy parent frame must be set to empty "
                "string. Setting it to any other frame is not "
                "supported yet." << std::endl;
    }
    return;
  }
  
  this->dataPtr->worldRelativeOrientation = _rot;
  this->dataPtr->worldFrameRelativeTo = _relativeTo;

  // Table to hold frame transformations
  static const std::map<WorldFrameEnumType,
    std::map<WorldFrameEnumType, ignition::math::Quaterniond>>
      transformTable =
    {
      {WorldFrameEnumType::ENU,
        {
          {WorldFrameEnumType::ENU, ignition::math::Quaterniond(0, 0, 0)},
          {WorldFrameEnumType::NED, ignition::math::Quaterniond(
            IGN_PI, 0, IGN_PI/2)},
          {WorldFrameEnumType::NWU, ignition::math::Quaterniond(
            0, 0, IGN_PI/2)},
        }
      },
      {WorldFrameEnumType::NED,
        {
          {WorldFrameEnumType::ENU, ignition::math::Quaterniond(
            IGN_PI, 0, IGN_PI/2).Inverse()},
          {WorldFrameEnumType::NED, ignition::math::Quaterniond(0, 0, 0)},
          {WorldFrameEnumType::NWU, ignition::math::Quaterniond(
            -IGN_PI, 0, 0)},
        }
      },
      {WorldFrameEnumType::NWU,
        {
          {WorldFrameEnumType::ENU, ignition::math::Quaterniond(
            0, 0, -IGN_PI/2)},
          {WorldFrameEnumType::NED, ignition::math::Quaterniond(IGN_PI, 0, 0)},
          {WorldFrameEnumType::NWU, ignition::math::Quaterniond(0, 0, 0)},
        }
      }
    };

  if (this->dataPtr->sensorOrientationRelativeTo != WorldFrameEnumType::NONE &&
      this->dataPtr->sensorOrientationRelativeTo != WorldFrameEnumType::CUSTOM)
  {
    // A valid named localization tag is supplied in the sdf
    auto tranformation =
      transformTable.at(this->dataPtr->worldFrameRelativeTo).at
      (this->dataPtr->sensorOrientationRelativeTo);
    this->SetOrientationReference(this->dataPtr->worldRelativeOrientation *
      tranformation);
  }
}

//////////////////////////////////////////////////
void ImuSensor::SetOrientationReference(const math::Quaterniond &_orient)
{
  this->dataPtr->orientationReference = _orient;
}

//////////////////////////////////////////////////
math::Quaterniond ImuSensor::OrientationReference() const
{
  return this->dataPtr->orientationReference;
}

//////////////////////////////////////////////////
void ImuSensor::SetOrientationEnabled(bool _enabled)
{
  this->dataPtr->orientationEnabled = _enabled;
}

//////////////////////////////////////////////////
bool ImuSensor::OrientationEnabled() const
{
  return this->dataPtr->orientationEnabled;
}

//////////////////////////////////////////////////
void ImuSensor::SetGravity(const math::Vector3d &_gravity)
{
  this->dataPtr->gravity = _gravity;
}

//////////////////////////////////////////////////
math::Vector3d ImuSensor::Gravity() const
{
  return this->dataPtr->gravity;
}

//////////////////////////////////////////////////
math::Quaterniond ImuSensor::Orientation() const
{
  return this->dataPtr->orientation;
}

