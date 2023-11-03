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

#include "gz/sensors/GaussianNoiseModel.hh"
#include <gz/math/Helpers.hh>
#include <gz/math/Rand.hh>

#include "gz/common/Console.hh"

using namespace gz;
using namespace sensors;

class gz::sensors::GaussianNoiseModelPrivate
{
  /// \brief If type starts with GAUSSIAN, the mean of the distribution
  /// from which we sample when adding noise.
  public: double mean = 0.0;

  /// \brief If type starts with GAUSSIAN, the standard deviation of the
  /// distribution from which we sample when adding noise.
  public: double stdDev = 0.0;

  /// \brief If type starts with GAUSSIAN, the bias we'll add.
  public: double bias = 0.0;

  /// \brief If type starts with GAUSSIAN, the standard deviation of the
  /// distribution from which the dynamic bias will be driven.
  public: double dynamicBiasStdDev = 0.0;

  /// \brief If type starts with GAUSSIAN, the correlation time of the
  /// process from which the dynamic bias will be driven.
  public: double dynamicBiasCorrTime = 0.0;

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
  this->dataPtr->dynamicBiasStdDev = _sdf.DynamicBiasStdDev();
  this->dataPtr->dynamicBiasCorrTime = _sdf.DynamicBiasCorrelationTime();

  // Sample the bias
  double biasMean = 0;
  double biasStdDev = 0;
  biasMean = _sdf.BiasMean();
  biasStdDev = _sdf.BiasStdDev();
  if (biasStdDev > 0.0)
    this->dataPtr->bias = math::Rand::DblNormal(biasMean, biasStdDev);
  else
    this->dataPtr->bias = biasMean;

  // With equal probability, we pick a negative bias (by convention,
  // rateBiasMean should be positive, though it would work fine if
  // negative).
  if (math::Rand::DblUniform() < 0.5)
    this->dataPtr->bias = -this->dataPtr->bias;

  this->Print(out);

  this->dataPtr->precision = _sdf.Precision();
  if (this->dataPtr->precision < 0)
    gzerr << "Noise precision cannot be less than 0" << std::endl;
  else if (!math::equal(this->dataPtr->precision, 0.0, 1e-6))
    this->dataPtr->quantized = true;
}

//////////////////////////////////////////////////
double GaussianNoiseModel::ApplyImpl(double _in, double _dt)
{
  // Generate independent (uncorrelated) Gaussian noise to each input value.
  double whiteNoise = this->dataPtr->stdDev > 0.0 ?
      math::Rand::DblNormal(this->dataPtr->mean, this->dataPtr->stdDev) :
      this->dataPtr->mean;

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
      math::Rand::DblNormal(0, sigma_b_d);
  }

  double output = _in + this->dataPtr->bias + whiteNoise;

  if (this->dataPtr->quantized)
  {
    // Apply this->dataPtr->precision
    if (!math::equal(this->dataPtr->precision, 0.0, 1e-6))
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
