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
#include "ignition/rendering/Camera.hh"

/* TODO: We need to implement Ogre Compositor Instance in ign-rendering
namespace ignition
{
  // We'll create an instance of this class for each camera, to be used to
  // inject random values on each render call.
  class GaussianNoiseCompositorListener
    : public Ogre::CompositorInstance::Listener
  {
    /// \brief Constructor, setting mean and standard deviation.
    public: GaussianNoiseCompositorListener(double _mean, double _stddev):
        mean(_mean), stddev(_stddev) {}

    /// \brief Callback that OGRE will invoke for us on each render call
    /// \param[in] _passID OGRE material pass ID.
    /// \param[in] _mat Pointer to OGRE material.
    public: virtual void notifyMaterialRender(unsigned int _passId,
                                              Ogre::MaterialPtr &_mat)
    {
      IGN_ASSERT(!_mat.isNull(), "Null OGRE material");
      // modify material here (wont alter the base material!), called for
      // every drawn geometry instance (i.e. compositor render_quad)

      // Sample three values within the range [0,1.0] and set them for use in
      // the fragment shader, which will interpret them as offsets from (0,0)
      // to use when computing pseudo-random values.
      Ogre::Vector3 offsets(ignition::math::Rand::DblUniform(0.0, 1.0),
                            ignition::math::Rand::DblUniform(0.0, 1.0),
                            ignition::math::Rand::DblUniform(0.0, 1.0));
      // These calls are setting parameters that are declared in two places:
      // 1. media/materials/scripts/gazebo.material, in
      //    fragment_program Gazebo/GaussianCameraNoiseFS
      // 2. media/materials/scripts/camera_noise_gaussian_fs.glsl
      Ogre::Technique *technique = _mat->getTechnique(0);
      IGN_ASSERT(technique, "Null OGRE material technique");
      Ogre::Pass *pass = technique->getPass(_passId);
      IGN_ASSERT(pass, "Null OGRE material pass");
      Ogre::GpuProgramParametersSharedPtr params =
          pass->getFragmentProgramParameters();
      IGN_ASSERT(!params.isNull(), "Null OGRE material GPU parameters");

      params->setNamedConstant("offsets", offsets);
      params->setNamedConstant("mean", static_cast<Ogre::Real>(this->dataPtr->mean));
      params->setNamedConstant("stddev", static_cast<Ogre::Real>(this->stddev));
    }

    /// \brief Mean that we'll pass down to the GLSL fragment shader.
    private: double mean;
    /// \brief Standard deviation that we'll pass down to the GLSL fragment
    /// shader.
    private: double stddev;
  };
}*/

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


/* TODO: We need to implement Ogre Compositor Instance in ign-rendering

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
void ImageGaussianNoiseModel::Load(sdf::ElementPtr _sdf)
{
  GaussianNoiseModel::Load(_sdf);
}

//////////////////////////////////////////////////
void ImageGaussianNoiseModel::SetCamera(rendering::CameraPtr _camera)
{

  IGN_ASSERT(_camera, "Unable to apply gaussian noise, camera is null");

  this->gaussianNoiseCompositorListener.reset(new
        GaussianNoiseCompositorListener(this->dataPtr->mean, this->dataPtr->stdDev));

  this->gaussianNoiseInstance =
    Ogre::CompositorManager::getSingleton().addCompositor(
      _camera->OgreViewport(), "CameraNoise/Gaussian");
  this->gaussianNoiseInstance->setEnabled(true);
  this->gaussianNoiseInstance->addListener(
    this->gaussianNoiseCompositorListener.get());
}

//////////////////////////////////////////////////
void ImageGaussianNoiseModel::Print(std::ostream &_out) const
{
  _out << "Image Gaussian noise, mean[" << this->dataPtr->mean << "], "
    << "stdDev[" << this->dataPtr->stdDev << "] "
    << "bias[" << this->dataPtr->bias << "] "
    << "precision[" << this->dataPtr->precision << "] "
    << "quantized[" << this->dataPtr->quantized << "]";
}
*/
