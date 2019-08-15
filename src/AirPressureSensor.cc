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

#include <ignition/msgs/fluid_pressure.pb.h>
#include <ignition/common/Profiler.hh>
#include <ignition/transport/Node.hh>

#include "ignition/sensors/GaussianNoiseModel.hh"
#include "ignition/sensors/Noise.hh"
#include "ignition/sensors/SensorTypes.hh"
#include "ignition/sensors/SensorFactory.hh"
#include "ignition/sensors/AirPressureSensor.hh"

using namespace ignition;
using namespace sensors;

// Constants. These constants from from RotorS:
// https://github.com/ethz-asl/rotors_simulator/blob/master/rotors_gazebo_plugins/include/rotors_gazebo_plugins/gazebo_pressure_plugin.h
static constexpr double kGasConstantNmPerKmolKelvin = 8314.32;
static constexpr double kMeanMolecularAirWeightKgPerKmol = 28.9644;
static constexpr double kGravityMagnitude = 9.80665;
static constexpr double kEarthRadiusMeters = 6356766.0;
static constexpr double kPressureOneAtmospherePascals = 101325.0;
static constexpr double kSeaLevelTempKelvin = 288.15;
static constexpr double kTempLapseKelvinPerMeter = 0.0065;
static constexpr double kAirConstantDimensionless = kGravityMagnitude *
    kMeanMolecularAirWeightKgPerKmol /
        (kGasConstantNmPerKmolKelvin * -kTempLapseKelvinPerMeter);

/// \brief Private data for AirPressureSensor
class ignition::sensors::AirPressureSensorPrivate
{
  /// \brief node to create publisher
  public: transport::Node node;

  /// \brief publisher to publish air pressure messages.
  public: transport::Node::Publisher pub;

  /// \brief true if Load() has been called and was successful
  public: bool initialized = false;

  /// \brief Pressure in pascals.
  public: double pressure = 0.0;

  /// \brief Altitude reference, i.e. initial sensor position
  public: double referenceAltitude = 0.0;

  /// \brief Noise added to sensor data
  public: std::map<SensorNoiseType, NoisePtr> noises;
};

//////////////////////////////////////////////////
AirPressureSensor::AirPressureSensor()
  : dataPtr(new AirPressureSensorPrivate())
{
}

//////////////////////////////////////////////////
AirPressureSensor::~AirPressureSensor()
{
}

//////////////////////////////////////////////////
bool AirPressureSensor::Init()
{
  return this->Sensor::Init();
}

//////////////////////////////////////////////////
bool AirPressureSensor::Load(const sdf::Sensor &_sdf)
{
  if (!Sensor::Load(_sdf))
    return false;

  if (_sdf.Type() != sdf::SensorType::AIR_PRESSURE)
  {
    ignerr << "Attempting to a load an AirPressure sensor, but received "
      << "a " << _sdf.TypeStr() << std::endl;
    return false;
  }

  if (_sdf.AirPressureSensor() == nullptr)
  {
    ignerr << "Attempting to a load an AirPressure sensor, but received "
      << "a null sensor." << std::endl;
    return false;
  }

  std::string topic = this->Topic();
  if (topic.empty())
    topic = "/air_pressure";

  this->dataPtr->pub =
      this->dataPtr->node.Advertise<ignition::msgs::FluidPressure>(topic);

  if (!this->dataPtr->pub)
  {
    ignerr << "Unabled to create publisher on topic[" << topic << "].\n";
    return false;
  }

  // Load the noise parameters
  if (_sdf.AirPressureSensor()->PressureNoise().Type() != sdf::NoiseType::NONE)
  {
    this->dataPtr->noises[AIR_PRESSURE_NOISE_PASCALS] =
      NoiseFactory::NewNoiseModel(_sdf.AirPressureSensor()->PressureNoise());
  }

  this->dataPtr->initialized = true;
  return true;
}

//////////////////////////////////////////////////
bool AirPressureSensor::Load(sdf::ElementPtr _sdf)
{
  sdf::Sensor sdfSensor;
  sdfSensor.Load(_sdf);
  return this->Load(sdfSensor);
}

//////////////////////////////////////////////////
bool AirPressureSensor::Update(const ignition::common::Time &_now)
{
  IGN_PROFILE("AirPressureSensor::Update");
  if (!this->dataPtr->initialized)
  {
    ignerr << "Not initialized, update ignored.\n";
    return false;
  }

  msgs::FluidPressure msg;
  msg.mutable_header()->mutable_stamp()->set_sec(_now.sec);
  msg.mutable_header()->mutable_stamp()->set_nsec(_now.nsec);
  auto frame = msg.mutable_header()->add_data();
  frame->set_key("frame_id");
  frame->add_value(this->Name());

  // This block of code comes from RotorS:
  // https://github.com/ethz-asl/rotors_simulator/blob/master/rotors_gazebo_plugins/src/gazebo_pressure_plugin.cpp
  {
    // Get the current height.
    double height = this->dataPtr->referenceAltitude + this->Pose().Pos().Z();

    // Compute the geopotential height.
    double geoHeight = kEarthRadiusMeters * height /
      (kEarthRadiusMeters + height);

    // Compute the temperature at the current altitude in Kelvin.
    double tempAtHeight =
      kSeaLevelTempKelvin - kTempLapseKelvinPerMeter * geoHeight;

    // Compute the current air pressure.
    this->dataPtr->pressure =
      kPressureOneAtmospherePascals * exp(kAirConstantDimensionless *
          log(kSeaLevelTempKelvin / tempAtHeight));
  }

  // Apply pressure noise
  if (this->dataPtr->noises.find(AIR_PRESSURE_NOISE_PASCALS) !=
      this->dataPtr->noises.end())
  {
    this->dataPtr->pressure =
      this->dataPtr->noises[AIR_PRESSURE_NOISE_PASCALS]->Apply(
          this->dataPtr->pressure);

    if (this->dataPtr->noises[AIR_PRESSURE_NOISE_PASCALS]->Type() ==
        NoiseType::GAUSSIAN)
    {
     GaussianNoiseModelPtr gaussian =
       std::dynamic_pointer_cast<sensors::GaussianNoiseModel>(
           this->dataPtr->noises[AIR_PRESSURE_NOISE_PASCALS]);
      msg.set_variance(sqrt(gaussian->StdDev()));
    }
  }

  msg.set_pressure(this->dataPtr->pressure);

  // publish
  this->dataPtr->pub.Publish(msg);

  return true;
}

//////////////////////////////////////////////////
void AirPressureSensor::SetReferenceAltitude(double _reference)
{
  this->dataPtr->referenceAltitude = _reference;
}

//////////////////////////////////////////////////
double AirPressureSensor::ReferenceAltitude() const
{
  return this->dataPtr->referenceAltitude;
}

IGN_SENSORS_REGISTER_SENSOR(AirPressureSensor)
