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

#include <ignition/common/Console.hh>

// TODO(WilliamLewww): Remove these pragmas once ign-rendering is disabling the
// warnings
#ifdef _WIN32
#pragma warning(push)
#pragma warning(disable: 4251)
#endif
#include <ignition/rendering/DistortionPass.hh>
#include <ignition/rendering/RenderPass.hh>
#include <ignition/rendering/RenderEngine.hh>
#include <ignition/rendering/RenderPassSystem.hh>
#ifdef _WIN32
#pragma warning(pop)
#endif

#include "ignition/sensors/ImageBrownDistortionModel.hh"

using namespace ignition;
using namespace sensors;

class ignition::sensors::ImageBrownDistortionModelPrivate
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

  /// \brief The distortion pass.
  public: rendering::DistortionPassPtr distortionPass;
};

//////////////////////////////////////////////////
ImageBrownDistortionModel::ImageBrownDistortionModel()
  : BrownDistortionModel(), dataPtr(new ImageBrownDistortionModelPrivate())
{
}

//////////////////////////////////////////////////
ImageBrownDistortionModel::~ImageBrownDistortionModel()
{
  delete this->dataPtr;
  this->dataPtr = nullptr;
}

//////////////////////////////////////////////////
void ImageBrownDistortionModel::Load(const sdf::Camera &_sdf)
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
void ImageBrownDistortionModel::SetCamera(rendering::CameraPtr _camera)
{
  if (!_camera)
  {
    ignerr << "Unable to apply distortion, camera is null\n";
    return;
  }

  rendering::RenderEngine *engine = _camera->Scene()->Engine();
  rendering::RenderPassSystemPtr rpSystem = engine->RenderPassSystem();
  if (rpSystem)
  {
    // add distortion pass
    rendering::RenderPassPtr distortionPass =
      rpSystem->Create<rendering::DistortionPass>();
    this->dataPtr->distortionPass =
        std::dynamic_pointer_cast<rendering::DistortionPass>(distortionPass);
    this->dataPtr->distortionPass->SetK1(this->dataPtr->k1);
    this->dataPtr->distortionPass->SetK2(this->dataPtr->k2);
    this->dataPtr->distortionPass->SetK3(this->dataPtr->k3);
    this->dataPtr->distortionPass->SetP1(this->dataPtr->p1);
    this->dataPtr->distortionPass->SetP2(this->dataPtr->p2);
    this->dataPtr->distortionPass->SetCenter(this->dataPtr->lensCenter);
    this->dataPtr->distortionPass->SetEnabled(true);
    _camera->AddRenderPass(this->dataPtr->distortionPass);
  }
}

//////////////////////////////////////////////////
void ImageBrownDistortionModel::Print(std::ostream &_out) const
{
  _out << "Image distortion, k1[" << this->dataPtr->k1 << "], "
    << "k2[" << this->dataPtr->k2 << "] "
    << "k3[" << this->dataPtr->k2 << "] "
    << "p1[" << this->dataPtr->p1 << "] "
    << "p2[" << this->dataPtr->p2 << "] "
    << "lensCenter[" << this->dataPtr->lensCenter << "]";
}
