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

#include "ignition/sensors/GaussianNoiseModel.hh"
#include <ignition/math/Helpers.hh>
#include <ignition/math/Rand.hh>

#include "ignition/common/Console.hh"
#include "ignition/rendering/GaussianNoisePass.hh"
#include "ignition/rendering/RenderPass.hh"
#include "ignition/rendering/RenderEngine.hh"
#include "ignition/rendering/RenderPassSystem.hh"


using namespace ignition;
using namespace sensors;

class ignition::sensors::GaussianNoiseModelPrivate
{
  /// \brief If type starts with GAUSSIAN, the mean of the distribution
  /// from which we sample when adding noise.
  public: double mean = 0.0;

  /// \brief If type starts with GAUSSIAN, the standard deviation of the
  /// distribution from which we sample when adding noise.
  public: double stdDev = 0.0;

  /// \brief If type starts with GAUSSIAN, the bias we'll add.
  public: double bias = 0.0;

  /// \brief If type==GAUSSIAN_QUANTIZED, the precision to which
  /// the output signal is rounded.
  public: double precision = 0.0;

  /// \brief True if the type is GAUSSIAN_QUANTIZED
  public: bool quantized = false;
};

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
GaussianNoiseModel::GaussianNoiseModel()
  : Noise(NoiseType::GAUSSIAN), dataPtr(new GaussianNoiseModelPrivate())
{
}

//////////////////////////////////////////////////
GaussianNoiseModel::~GaussianNoiseModel()
{
  delete this->dataPtr;
  this->dataPtr = nullptr;
}

//////////////////////////////////////////////////
void GaussianNoiseModel::Load(const sdf::Noise &_sdf)
{
  Noise::Load(_sdf);
  std::ostringstream out;

  this->dataPtr->mean = _sdf.Mean();
  this->dataPtr->stdDev = _sdf.StdDev();

  // Sample the bias
  double biasMean = 0;
  double biasStdDev = 0;
  biasMean = _sdf.BiasMean();
  biasStdDev = _sdf.BiasStdDev();
  this->dataPtr->bias = ignition::math::Rand::DblNormal(biasMean, biasStdDev);

  // With equal probability, we pick a negative bias (by convention,
  // rateBiasMean should be positive, though it would work fine if
  // negative).
  if (ignition::math::Rand::DblUniform() < 0.5)
    this->dataPtr->bias = -this->dataPtr->bias;

  this->Print(out);

  this->dataPtr->precision = _sdf.Precision();
  if (this->dataPtr->precision < 0)
    ignerr << "Noise precision cannot be less than 0" << std::endl;
  else if (!ignition::math::equal(this->dataPtr->precision, 0.0, 1e-6))
    this->dataPtr->quantized = true;
}

//////////////////////////////////////////////////
double GaussianNoiseModel::ApplyImpl(double _in)
{
  // Add independent (uncorrelated) Gaussian noise to each input value.
  double whiteNoise = ignition::math::Rand::DblNormal(
      this->dataPtr->mean, this->dataPtr->stdDev);
  double output = _in + this->dataPtr->bias + whiteNoise;

  if (this->dataPtr->quantized)
  {
    // Apply this->dataPtr->precision
    if (!ignition::math::equal(this->dataPtr->precision, 0.0, 1e-6))
    {
      output = std::round(output / this->dataPtr->precision) *
        this->dataPtr->precision;
    }
  }
  return output;
}

//////////////////////////////////////////////////
double GaussianNoiseModel::Mean() const
{
  return this->dataPtr->mean;
}

//////////////////////////////////////////////////
double GaussianNoiseModel::StdDev() const
{
  return this->dataPtr->stdDev;
}

//////////////////////////////////////////////////
double GaussianNoiseModel::Bias() const
{
  return this->dataPtr->bias;
}

//////////////////////////////////////////////////
void GaussianNoiseModel::Print(std::ostream &_out) const
{
  _out << "Gaussian noise, mean[" << this->dataPtr->mean << "], "
    << "stdDev[" << this->dataPtr->stdDev << "] "
    << "bias[" << this->dataPtr->bias << "] "
    << "precision[" << this->dataPtr->precision << "] "
    << "quantized[" << this->dataPtr->quantized << "]";
}

//////////////////////////////////////////////////
ImageGaussianNoiseModel::ImageGaussianNoiseModel()
  : GaussianNoiseModel()
{
}

//////////////////////////////////////////////////
ImageGaussianNoiseModel::~ImageGaussianNoiseModel()
{
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
