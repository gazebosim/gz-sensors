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
  // Ensure that Winsock2.h is included before Windows.h, which can get
  // pulled in by anybody (e.g., Boost).
  #include <Winsock2.h>
#endif

#include "ignition/common/Console.hh"

#include "ignition/sensors/ImageNoise.hh"
#include "ignition/sensors/ImageGaussianNoiseModel.hh"

using namespace gz;
using namespace sensors;

//////////////////////////////////////////////////
NoisePtr ImageNoiseFactory::NewNoiseModel(const sdf::Noise &_sdf,
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
        _sensorType == "thermal_camera" ||  _sensorType == "rgbd_camera")
    {
      noise.reset(new ImageGaussianNoiseModel());
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
NoisePtr ImageNoiseFactory::NewNoiseModel(sdf::ElementPtr _sdf,
    const std::string &_sensorType)
{
  IGN_ASSERT(_sdf != nullptr, "noise sdf is null");
  IGN_ASSERT(_sdf->GetName() == "noise", "Not a noise SDF element");
  sdf::Noise noiseDom;
  noiseDom.Load(_sdf);

  return NewNoiseModel(noiseDom, _sensorType);
}
