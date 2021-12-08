/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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
  // Ensure that Winsock2.h is included before Windows.h, which can get
  // pulled in by anybody (e.g., Boost).
  #include <Winsock2.h>
#endif

#include <functional>

#include <ignition/common/Console.hh>

#include "ignition/sensors/GaussianNoiseModel.hh"
#include "ignition/sensors/Noise.hh"

using namespace ignition;
using namespace sensors;

class ignition::sensors::NoisePrivate
{
  /// \brief Which type of noise we're applying
  public: NoiseType type = NoiseType::NONE;

  /// \brief Noise sdf element.
  public: sdf::Noise noiseDom;

  /// \brief Callback function for applying custom noise to sensor data.
  public: std::function<double(double, double)> customNoiseCallback;
};

//////////////////////////////////////////////////
NoisePtr NoiseFactory::NewNoiseModel(const sdf::Noise &_sdf,
    const std::string &_sensorType)
{
  sdf::NoiseType noiseType = _sdf.Type();

  NoisePtr noise;

  // Check for 'gaussian' noise. The 'gaussian_quantized' type is kept for
  // backward compatibility.
  if (noiseType == sdf::NoiseType::GAUSSIAN ||
      noiseType == sdf::NoiseType::GAUSSIAN_QUANTIZED)
  {
    if (_sensorType == "camera" || _sensorType == "depth" ||
        _sensorType == "multicamera" || _sensorType == "wide_angle_camera" ||
        _sensorType == "thermal_camera" || _sensorType == "rgbd_camera")
    {
      ignerr << "Image noise requested. "
             << "Please use ImageNoiseFactory::NoiseModel instead"
             << std::endl;
      return noise;
    }
    else
      noise.reset(new GaussianNoiseModel());

    IGN_ASSERT(noise->Type() == NoiseType::GAUSSIAN,
        "Noise type should be 'gaussian'");
  }
  else if (noiseType == sdf::NoiseType::NONE)
  {
    // Return empty noise if 'none' or 'custom' is specified.
    // if 'custom', the type will be set once the user calls the
    // SetCustomNoiseCallback function.
    noise.reset(new Noise(NoiseType::NONE));
    IGN_ASSERT(noise->Type() == NoiseType::NONE,
        "Noise type should be 'none'");
  }
  else
  {
    ignerr << "Unrecognized noise type" << std::endl;
    return NoisePtr();
  }
  noise->Load(_sdf);

  return noise;
}

//////////////////////////////////////////////////
NoisePtr NoiseFactory::NewNoiseModel(sdf::ElementPtr _sdf,
    const std::string &_sensorType)
{
  IGN_ASSERT(_sdf != nullptr, "noise sdf is null");
  IGN_ASSERT(_sdf->GetName() == "noise", "Not a noise SDF element");
  sdf::Noise noiseDom;
  noiseDom.Load(_sdf);

  return NewNoiseModel(noiseDom, _sensorType);
}

//////////////////////////////////////////////////
Noise::Noise(NoiseType _type)
  : dataPtr(new NoisePrivate())
{
  this->dataPtr->type = _type;
}

//////////////////////////////////////////////////
Noise::~Noise()
{
  delete this->dataPtr;
  this->dataPtr = nullptr;
}

//////////////////////////////////////////////////
void Noise::Load(const sdf::Noise &_sdf)
{
  this->dataPtr->noiseDom = _sdf;
}

//////////////////////////////////////////////////
double Noise::Apply(double _in, double _dt)
{
  if (this->dataPtr->type == NoiseType::NONE)
    return _in;
  else if (this->dataPtr->type == NoiseType::CUSTOM)
  {
    if (this->dataPtr->customNoiseCallback)
      return this->dataPtr->customNoiseCallback(_in, _dt);
    else
    {
      ignerr << "Custom noise callback function not set!"
          << " Please call SetCustomNoiseCallback within a sensor plugin."
          << std::endl;
      return _in;
    }
  }

  return this->ApplyImpl(_in, _dt);
}

//////////////////////////////////////////////////
double Noise::ApplyImpl(double _in, double /*_dt*/)
{
  return _in;
}

//////////////////////////////////////////////////
NoiseType Noise::Type() const
{
  return this->dataPtr->type;
}

//////////////////////////////////////////////////
void Noise::SetCustomNoiseCallback(std::function<double(double, double)> _cb)
{
  this->dataPtr->type = NoiseType::CUSTOM;
  this->dataPtr->customNoiseCallback = _cb;
}

//////////////////////////////////////////////////
void Noise::Print(std::ostream &_out) const
{
  _out << "Noise with type[" << static_cast<int>(this->dataPtr->type) << "] "
    << "does not have an overloaded Print function. "
    << "No more information is available.";
}
