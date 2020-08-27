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

#include <ignition/msgs/magnetometer.pb.h>

#include <ignition/common/Profiler.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/sensors/Noise.hh>
#include <ignition/sensors/SensorFactory.hh>
#include <ignition/sensors/SensorTypes.hh>
#include <ignition/sensors/MagnetometerSensor.hh>
#include <ignition/transport/Node.hh>
#include <sdf/Magnetometer.hh>

using namespace ignition;
using namespace sensors;

/// \brief Private data for MagnetometerSensor
class ignition::sensors::MagnetometerSensorPrivate
{
  /// \brief node to create publisher
  public: transport::Node node;

  /// \brief publisher to publish magnetometer messages.
  public: transport::Node::Publisher pub;

  /// \brief true if Load() has been called and was successful
  public: bool initialized = false;

  /// \brief The latest field reading from the sensor, based on the world
  /// field and the sensor's current pose.
  public: ignition::math::Vector3d localField;

  /// \brief Store world magnetic field vector. We assume it is uniform
  /// everywhere in the world, and that it doesn't change during the simulation.
  public: ignition::math::Vector3d worldField;

  /// \brief World pose of the magnetometer
  public: ignition::math::Pose3d worldPose;

  /// \brief Noise added to sensor data
  public: std::map<SensorNoiseType, NoisePtr> noises;
};

//////////////////////////////////////////////////
MagnetometerSensor::MagnetometerSensor()
  : dataPtr(new MagnetometerSensorPrivate())
{
}

//////////////////////////////////////////////////
MagnetometerSensor::~MagnetometerSensor()
{
}

//////////////////////////////////////////////////
bool MagnetometerSensor::Init()
{
  return this->Sensor::Init();
}

//////////////////////////////////////////////////
bool MagnetometerSensor::Load(const sdf::Sensor &_sdf)
{
  if (!Sensor::Load(_sdf))
    return false;

  if (_sdf.Type() != sdf::SensorType::MAGNETOMETER)
  {
    ignerr << "Attempting to a load a Magnetometer sensor, but received "
      << "a " << _sdf.TypeStr() << std::endl;
    return false;
  }

  if (_sdf.MagnetometerSensor() == nullptr)
  {
    ignerr << "Attempting to a load a Magnetometer sensor, but received "
      << "a null sensor." << std::endl;
    return false;
  }

  std::string topic = this->Topic();
  if (topic.empty())
    topic = "/magnetometer";

  this->dataPtr->pub =
      this->dataPtr->node.Advertise<ignition::msgs::Magnetometer>(topic);

  if (!this->dataPtr->pub)
  {
    ignerr << "Unable to create publisher on topic[" << topic << "].\n";
    return false;
  }

  // Load the noise parameters
  if (_sdf.MagnetometerSensor()->XNoise().Type() != sdf::NoiseType::NONE)
  {
    this->dataPtr->noises[MAGNETOMETER_X_NOISE_TESLA] =
      NoiseFactory::NewNoiseModel(_sdf.MagnetometerSensor()->XNoise());
  }

  if (_sdf.MagnetometerSensor()->YNoise().Type() != sdf::NoiseType::NONE)
  {
    this->dataPtr->noises[MAGNETOMETER_Y_NOISE_TESLA] =
      NoiseFactory::NewNoiseModel(_sdf.MagnetometerSensor()->YNoise());
  }

  if (_sdf.MagnetometerSensor()->ZNoise().Type() != sdf::NoiseType::NONE)
  {
    this->dataPtr->noises[MAGNETOMETER_Z_NOISE_TESLA] =
      NoiseFactory::NewNoiseModel(_sdf.MagnetometerSensor()->ZNoise());
  }

  this->dataPtr->initialized = true;
  return true;
}

//////////////////////////////////////////////////
bool MagnetometerSensor::Load(sdf::ElementPtr _sdf)
{
  sdf::Sensor sdfSensor;
  sdfSensor.Load(_sdf);
  return this->Load(sdfSensor);
}

//////////////////////////////////////////////////
bool MagnetometerSensor::Update(const ignition::common::Time &_now)
{
  IGN_PROFILE("MagnetometerSensor::Update");
  if (!this->dataPtr->initialized)
  {
    ignerr << "Not initialized, update ignored.\n";
    return false;
  }

  // compute magnetic field in body frame
  this->dataPtr->localField =
      this->dataPtr->worldPose.Rot().Inverse().RotateVector(
      this->dataPtr->worldField);

  msgs::Magnetometer msg;
  msg.mutable_header()->mutable_stamp()->set_sec(_now.sec);
  msg.mutable_header()->mutable_stamp()->set_nsec(_now.nsec);
  auto frame = msg.mutable_header()->add_data();
  frame->set_key("frame_id");
  frame->add_value(this->Name());

  // Apply magnetometer noise after converting to body frame
  if (this->dataPtr->noises.find(MAGNETOMETER_X_NOISE_TESLA) !=
      this->dataPtr->noises.end())
  {
    this->dataPtr->localField.X(
        this->dataPtr->noises[MAGNETOMETER_X_NOISE_TESLA]->Apply(
          this->dataPtr->localField.X()));
  }

  if (this->dataPtr->noises.find(MAGNETOMETER_Y_NOISE_TESLA) !=
      this->dataPtr->noises.end())
  {
    this->dataPtr->localField.Y(
        this->dataPtr->noises[MAGNETOMETER_Y_NOISE_TESLA]->Apply(
          this->dataPtr->localField.Y()));
  }

  if (this->dataPtr->noises.find(MAGNETOMETER_Z_NOISE_TESLA) !=
      this->dataPtr->noises.end())
  {
    this->dataPtr->localField.Z(
        this->dataPtr->noises[MAGNETOMETER_Z_NOISE_TESLA]->Apply(
          this->dataPtr->localField.Z()));
  }

  msgs::Set(msg.mutable_field_tesla(), this->dataPtr->localField);

  // publish
  this->AddSequence(msg.mutable_header());
  this->dataPtr->pub.Publish(msg);

  return true;
}

//////////////////////////////////////////////////
void MagnetometerSensor::SetWorldPose(const math::Pose3d _pose)
{
  this->dataPtr->worldPose = _pose;
}

//////////////////////////////////////////////////
math::Pose3d MagnetometerSensor::WorldPose() const
{
  return this->dataPtr->worldPose;
}

//////////////////////////////////////////////////
void MagnetometerSensor::SetWorldMagneticField(const math::Vector3d &_field)
{
  this->dataPtr->worldField = _field;
}

//////////////////////////////////////////////////
math::Vector3d MagnetometerSensor::WorldMagneticField() const
{
  return this->dataPtr->worldField;
}

//////////////////////////////////////////////////
math::Vector3d MagnetometerSensor::MagneticField() const
{
  return this->dataPtr->localField;
}

IGN_SENSORS_REGISTER_SENSOR(MagnetometerSensor)
