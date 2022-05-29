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

#include "gz/sensors/BrownDistortionModel.hh"

#include <gz/common/Console.hh>
#include <gz/math/Helpers.hh>
#include <gz/math/Rand.hh>

using namespace gz;
using namespace sensors;

class gz::sensors::BrownDistortionModel::Implementation
{
  /// \brief The radial distortion coefficient k1.
  public: double k1 = 0.0;

  /// \brief The radial distortion coefficient k2.
  public: double k2 = 0.0;

  /// \brief The radial distortion coefficient k3.
  public: double k3 = 0.0;

  /// \brief The radial distortion coefficient p1.
  public: double p1 = 0.0;

  /// \brief The radial distortion coefficient p2.
  public: double p2 = 0.0;

  /// \brief The distortion center.
  public: math::Vector2d lensCenter = {0.5, 0.5};
};

//////////////////////////////////////////////////
BrownDistortionModel::BrownDistortionModel()
  : Distortion(DistortionType::BROWN),
    dataPtr(utils::MakeImpl<Implementation>())
{
}

//////////////////////////////////////////////////
BrownDistortionModel::~BrownDistortionModel()
{
}

//////////////////////////////////////////////////
void BrownDistortionModel::Load(const sdf::Camera &_sdf)
{
  Distortion::Load(_sdf);

  this->dataPtr->k1 = _sdf.DistortionK1();
  this->dataPtr->k2 = _sdf.DistortionK2();
  this->dataPtr->k3 = _sdf.DistortionK3();
  this->dataPtr->p1 = _sdf.DistortionP1();
  this->dataPtr->p2 = _sdf.DistortionP2();
  this->dataPtr->lensCenter = _sdf.DistortionCenter();
}

//////////////////////////////////////////////////
double BrownDistortionModel::K1() const
{
  return this->dataPtr->k1;
}

//////////////////////////////////////////////////
double BrownDistortionModel::K2() const
{
  return this->dataPtr->k2;
}

//////////////////////////////////////////////////
double BrownDistortionModel::K3() const
{
  return this->dataPtr->k3;
}

//////////////////////////////////////////////////
double BrownDistortionModel::P1() const
{
  return this->dataPtr->p1;
}

//////////////////////////////////////////////////
double BrownDistortionModel::P2() const
{
  return this->dataPtr->p2;
}

//////////////////////////////////////////////////
math::Vector2d BrownDistortionModel::Center() const
{
  return this->dataPtr->lensCenter;
}

//////////////////////////////////////////////////
void BrownDistortionModel::Print(std::ostream &_out) const
{
  _out << "Distortion, k1[" << this->dataPtr->k1 << "], "
      << "k2[" << this->dataPtr->k2 << "] "
      << "k3[" << this->dataPtr->k2 << "] "
      << "p1[" << this->dataPtr->p1 << "] "
      << "p2[" << this->dataPtr->p2 << "] "
      << "lensCenter[" << this->dataPtr->lensCenter << "]";
}
