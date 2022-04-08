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

#include <ignition/common/Console.hh>

// TODO(louise) Remove these pragmas once ign-rendering is disabling the
// warnings
#ifdef _WIN32
#pragma warning(push)
#pragma warning(disable: 4251)
#endif
#include <ignition/rendering/GaussianNoisePass.hh>
#include <ignition/rendering/RenderPass.hh>
#include <ignition/rendering/RenderEngine.hh>
#include <ignition/rendering/RenderPassSystem.hh>
#ifdef _WIN32
#pragma warning(pop)
#endif

#include "ignition/sensors/ImageGaussianNoiseModel.hh"

using namespace ignition;
using namespace sensors;

class ignition::sensors::ImageGaussianNoiseModelPrivate
{
  /// \brief If type starts with GAUSSIAN, the mean of the distribution
  /// from which we sample when adding noise.
  public: double mean = 0.0;

  /// \brief If type starts with GAUSSIAN, the standard deviation of the
  /// distribution from which we sample when adding noise.
  public: double stdDev = 0.0;

  /// \brief Gaussian noise pass.
  public: rendering::GaussianNoisePassPtr gaussianNoisePass;
};

//////////////////////////////////////////////////
ImageGaussianNoiseModel::ImageGaussianNoiseModel()
  : GaussianNoiseModel(), dataPtr(new ImageGaussianNoiseModelPrivate())
{
}

//////////////////////////////////////////////////
ImageGaussianNoiseModel::~ImageGaussianNoiseModel()
{
  delete this->dataPtr;
  this->dataPtr = nullptr;
}

//////////////////////////////////////////////////
void ImageGaussianNoiseModel::Load(const sdf::Noise &_sdf)
{
  Noise::Load(_sdf);

  this->dataPtr->mean = _sdf.Mean();
  this->dataPtr->stdDev = _sdf.StdDev();
}

//////////////////////////////////////////////////
void ImageGaussianNoiseModel::SetCamera(rendering::CameraPtr _camera)
{
  if (!_camera)
  {
    ignerr << "Unable to apply gaussian noise, camera is null\n";
    return;
  }

  rendering::RenderEngine *engine = _camera->Scene()->Engine();
  rendering::RenderPassSystemPtr rpSystem = engine->RenderPassSystem();
  if (rpSystem)
  {
    // add gaussian noise pass
    rendering::RenderPassPtr noisePass =
      rpSystem->Create<rendering::GaussianNoisePass>();
    if (!noisePass)
    {
      ignwarn << "ImageGaussianNoiseModel is not supported in "
             << engine->Name() << std::endl;
      return;
    }
    this->dataPtr->gaussianNoisePass =
        std::dynamic_pointer_cast<rendering::GaussianNoisePass>(noisePass);
    this->dataPtr->gaussianNoisePass->SetMean(this->dataPtr->mean);
    this->dataPtr->gaussianNoisePass->SetStdDev(this->dataPtr->stdDev);
    this->dataPtr->gaussianNoisePass->SetEnabled(true);
    _camera->AddRenderPass(this->dataPtr->gaussianNoisePass);
  }
}

//////////////////////////////////////////////////
void ImageGaussianNoiseModel::Print(std::ostream &_out) const
{
  _out << "Image Gaussian noise, mean[" << this->dataPtr->mean << "], "
    << "stdDev[" << this->dataPtr->stdDev << "] ";
}
