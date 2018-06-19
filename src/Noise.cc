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
#include <ignition/sensors/GaussianNoiseModel.hh>
#include <ignition/sensors/Noise.hh>

using namespace ignition;
using namespace sensors;

//////////////////////////////////////////////////
NoisePtr NoiseFactory::NewNoiseModel(sdf::ElementPtr _sdf,
    const std::string &_sensorType)
{
  IGN_ASSERT(_sdf != nullptr, "noise sdf is null");
  IGN_ASSERT(_sdf->GetName() == "noise", "Not a noise SDF element");

  std::string typeString = _sdf->Get<std::string>("type");

  NoisePtr noise;

  // Check for 'gaussian' noise. The 'gaussian_quantized' type is kept for
  // backward compatibility.
  if (typeString == "gaussian" ||
      typeString == "gaussian_quantized")
  {
    if (_sensorType == "camera" || _sensorType == "depth" ||
      _sensorType == "multicamera" || _sensorType == "wideanglecamera")
    {
      // TODO(jchoclin): We need to implement Ogre Compositor Instance in
      // ign-rendering
      // noise.reset(new ImageGaussianNoiseModel());
    }
    else
      noise.reset(new GaussianNoiseModel());

    IGN_ASSERT(noise->GetNoiseType() == Noise::GAUSSIAN,
        "Noise type should be 'gaussian'");
  }
  else if (typeString == "none" || typeString == "custom")
  {
    // Return empty noise if 'none' or 'custom' is specified.
    // if 'custom', the type will be set once the user calls the
    // SetCustomNoiseCallback function.
    noise.reset(new Noise(Noise::NONE));
    IGN_ASSERT(noise->GetNoiseType() == Noise::NONE,
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
Noise::Noise(NoiseType _type)
  : type(_type)
{
}

//////////////////////////////////////////////////
Noise::~Noise()
{
}

//////////////////////////////////////////////////
void Noise::Load(sdf::ElementPtr _sdf)
{
  this->sdf = _sdf;
  IGN_ASSERT(this->sdf != nullptr, "this->sdf is null");
}

//////////////////////////////////////////////////
void Noise::SetCamera(rendering::CameraPtr /*_camera*/)
{
  ignerr << "Ignoring SetCamera: Not attached to an image sensor" << std::endl;
}

//////////////////////////////////////////////////
double Noise::Apply(double _in)
{
  if (this->type == NONE)
    return _in;
  else if (this->type == CUSTOM)
  {
    if (this->customNoiseCallback)
      return this->customNoiseCallback(_in);
    else
    {
      ignerr << "Custom noise callback function not set!"
          << " Please call SetCustomNoiseCallback within a sensor plugin."
          << std::endl;
      return _in;
    }
  }
  else
    return this->ApplyImpl(_in);
}

//////////////////////////////////////////////////
double Noise::ApplyImpl(double _in)
{
  return _in;
}

//////////////////////////////////////////////////
Noise::NoiseType Noise::GetNoiseType() const
{
  return this->type;
}

//////////////////////////////////////////////////
void Noise::SetCustomNoiseCallback(std::function<double(double)> _cb)
{
  this->type = CUSTOM;
  this->customNoiseCallback = _cb;
}

//////////////////////////////////////////////////
void Noise::Fini()
{
  this->customNoiseCallback = nullptr;
}

//////////////////////////////////////////////////
void Noise::Print(std::ostream &_out) const
{
  _out << "Noise with type[" << this->type << "] "
    << "does not have an overloaded Print function. "
    << "No more information is available.";
}
