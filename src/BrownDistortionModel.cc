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

#include "ignition/sensors/BrownDistortionModel.hh"
#include <ignition/math/Helpers.hh>
#include <ignition/math/Rand.hh>

#include "ignition/common/Console.hh"

using namespace ignition;
using namespace sensors;

class ignition::sensors::BrownDistortionModelPrivate
{
  /// \brief If type starts with BROWN, the mean of the distribution
  /// from which we sample when adding noise.
  public: double k1 = 0.0;

  /// \brief If type starts with BROWN, the standard deviation of the
  /// distribution from which we sample when adding noise.
  public: double k2 = 0.0;

  /// \brief If type starts with BROWN, the bias we'll add.
  public: double k3 = 0.0;

  /// \brief If type starts with BROWN, the standard deviation of the
  /// distribution from which the dynamic bias will be driven.
  public: double p1 = 0.0;

  /// \brief If type starts with BROWN, the correlation time of the
  /// process from which the dynamic bias will be driven.
  public: double p2 = 0.0;

  /// \brief If type==BROWN_QUANTIZED, the precision to which
  /// the output signal is rounded.
  public: math::Vector2d lensCenter = {0.5, 0.5};
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
  this->dataPtr->dynamicBiasStdDev = _sdf.DynamicBiasStdDev();
  this->dataPtr->dynamicBiasCorrTime = _sdf.DynamicBiasCorrelationTime();

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
double GaussianNoiseModel::ApplyImpl(double _in, double _dt)
{
  // Generate independent (uncorrelated) Gaussian noise to each input value.
  double whiteNoise = ignition::math::Rand::DblNormal(
      this->dataPtr->mean, this->dataPtr->stdDev);

  // Generate varying (correlated) bias to each input value.
  // This implementation is based on the one available in Rotors:
  // https://github.com/ethz-asl/rotors_simulator/blob/master/rotors_gazebo_plugins/src/gazebo_imu_plugin.cpp
  //
  // More information about the parameters and their derivation:
  //
  // https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model
  //
  // This can only be generated in the case that _dt > 0.0

  if (this->dataPtr->dynamicBiasStdDev > 0 &&
     this->dataPtr->dynamicBiasCorrTime > 0 &&
     _dt > 0)
  {
    double sigma_b = this->dataPtr->dynamicBiasStdDev;
    double tau = this->dataPtr->dynamicBiasCorrTime;

    double sigma_b_d = sqrt(-sigma_b * sigma_b *
        tau / 2 * expm1(-2 * _dt / tau));
    double phi_d = exp(-_dt / tau);
    this->dataPtr->bias = phi_d * this->dataPtr->bias +
      ignition::math::Rand::DblNormal(0, sigma_b_d);
  }

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
