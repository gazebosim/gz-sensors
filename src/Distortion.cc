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

#include <functional>

#include <gz/common/Console.hh>

#include "gz/sensors/BrownDistortionModel.hh"
#include "gz/sensors/Distortion.hh"

using namespace gz;
using namespace sensors;

class gz::sensors::Distortion::Implementation
{
  /// \brief Which type of distortion we're applying
  public: DistortionType type = DistortionType::NONE;

  /// \brief Camera sdf element.
  public: sdf::Camera cameraDom;
};

//////////////////////////////////////////////////
DistortionPtr DistortionFactory::NewDistortionModel(const sdf::Camera &_sdf,
    const std::string &_sensorType)
{
  // TODO(WilliamLewww): support different distortion models
  DistortionType distortionType = DistortionType::BROWN;

  DistortionPtr distortion;

  // Check for 'brown' distortion.
  if (distortionType == DistortionType::BROWN)
  {
    if (_sensorType == "camera")
    {
      gzerr << "Image distortion requested. "
             << "Please use ImageDistortionFactory::DistortionModel instead"
             << std::endl;
      return distortion;
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
    gzerr << "Unrecognized distortion type" << std::endl;
    return DistortionPtr();
  }
  distortion->Load(_sdf);

  return distortion;
}

//////////////////////////////////////////////////
DistortionPtr DistortionFactory::NewDistortionModel(sdf::ElementPtr _sdf,
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

//////////////////////////////////////////////////
Distortion::Distortion(DistortionType _type)
  : dataPtr(utils::MakeImpl<Implementation>())
{
  this->dataPtr->type = _type;
}

//////////////////////////////////////////////////
Distortion::~Distortion()
{
}

//////////////////////////////////////////////////
void Distortion::Load(const sdf::Camera &_sdf)
{
  this->dataPtr->cameraDom = _sdf;
}

//////////////////////////////////////////////////
DistortionType Distortion::Type() const
{
  return this->dataPtr->type;
}

//////////////////////////////////////////////////
void Distortion::Print(std::ostream &_out) const
{
  _out << "Distortion with type[" << static_cast<int>(this->dataPtr->type)
      << "] "
      << "does not have an overloaded Print function. "
      << "No more information is available.";
}
