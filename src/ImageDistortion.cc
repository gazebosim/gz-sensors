/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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

#include "ignition/sensors/ImageDistortion.hh"
#include "ignition/sensors/ImageBrownDistortionModel.hh"

using namespace gz;
using namespace sensors;

//////////////////////////////////////////////////
DistortionPtr ImageDistortionFactory::NewDistortionModel(
    const sdf::Camera &_sdf,
    const std::string &_sensorType)
{
  DistortionType distortionType = DistortionType::BROWN;

  DistortionPtr distortion;

  // Check for 'brown' distortion.
  if (distortionType == DistortionType::BROWN)
  {
    if (_sensorType == "camera")
    {
      distortion.reset(new ImageBrownDistortionModel());
    }
    else
      distortion.reset(new BrownDistortionModel());

    IGN_ASSERT(distortion->Type() == DistortionType::BROWN,
        "Distortion type should be 'brown'");
  }
  else if (distortionType == DistortionType::NONE)
  {
    // Return empty distortion if 'none' or 'custom' is specified.
    // if 'custom', the type will be set once the user calls the
    // SetCustomDistortionCallback function.
    distortion.reset(new Distortion(DistortionType::NONE));
    IGN_ASSERT(distortion->Type() == DistortionType::NONE,
        "Distortion type should be 'none'");
  }
  else
  {
    ignerr << "Unrecognized distortion type" << std::endl;
    return DistortionPtr();
  }
  distortion->Load(_sdf);

  return distortion;
}

//////////////////////////////////////////////////
DistortionPtr ImageDistortionFactory::NewDistortionModel(sdf::ElementPtr _sdf,
    const std::string &_sensorType)
{
  // TODO(WilliamLewww): create a distortion SDF to support different
  // distortion models
  IGN_ASSERT(_sdf != nullptr, "camera sdf is null");
  IGN_ASSERT(_sdf->GetName() == "camera", "Not a camera SDF element");
  sdf::Camera cameraDom;
  cameraDom.Load(_sdf);

  return NewDistortionModel(cameraDom, _sensorType);
}
