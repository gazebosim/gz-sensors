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

#include <iostream>
#include <vector>

#include <sdf/sdf.hh>

#include <gz/sensors/Noise.hh>
#include <gz/sensors/GaussianNoiseModel.hh>
#include <gz/sensors/SensorFactory.hh>

static constexpr double kSampleFrequency = 100.0;
// 16-bit ADC
static double kSamplePrecision = 1.0/pow(2.0, 16.0);
// Generate 6 hours of data.
static constexpr double kNumSamples = 6 * 3600 * kSampleFrequency;

// Default values for use with ADIS16448 IMU
// These values come from the Rotors default values:
// https://github.com/ethz-asl/rotors_simulator/blob/513bb92da0c1a0c968bdc679dffc8fe7d77de918/rotors_gazebo_plugins/include/rotors_gazebo_plugins/gazebo_imu_plugin.h#L40
static constexpr double kDefaultAdisGyroscopeNoiseDensity =
    2.0 * 35.0 / 3600.0 / 180.0 * IGN_PI;
static constexpr double kDefaultAdisGyroscopeRandomWalk =
    2.0 * 4.0 / 3600.0 / 180.0 * IGN_PI;
static constexpr double kDefaultAdisGyroscopeBiasCorrelationTime =
    1.0e+3;
static constexpr double kDefaultAdisGyroscopeTurnOnBiasSigma =
    0.5 / 180.0 * IGN_PI;
static constexpr double kDefaultAdisAccelerometerNoiseDensity =
    2.0 * 2.0e-3;
static constexpr double kDefaultAdisAccelerometerRandomWalk =
    2.0 * 3.0e-3;
static constexpr double kDefaultAdisAccelerometerBiasCorrelationTime =
    300.0;
static constexpr double kDefaultAdisAccelerometerTurnOnBiasSigma =
    20.0e-3 * 9.8;


// Convenience structure to represent noise parameters.
struct NoiseParameters {
  // Sample Frequency (Hz)
  double sampleFreq = 0;
  // Sensor noise gaussian distribution mean
  double mean = 0;
  // Sensor noise gaussian distribution standard deviation
  double stdDev = 0;
  // Static sensor bias mean
  double biasMean = 0;
  // Static sensor bias standard deviation
  double biasStdDev = 0;
  // Dynamic sensor bias standard deviation
  double dynamicBiasStdDev = 0;
  // Dynamic sensor bias correlation time
  double dynamicBiasCorrelationTime = 0;
  // Quantization precision
  double precision = 0;
};

// Generate a set of noise parameters for accelerometer channel
NoiseParameters accelerometerParameters(double _sampleFreq, double _precision)
{
  const double dt = 1.0/_sampleFreq;
  NoiseParameters accelParams;
  accelParams.sampleFreq = _sampleFreq;
  accelParams.stdDev = kDefaultAdisAccelerometerNoiseDensity * 1.0/sqrt(dt);
  accelParams.biasStdDev = kDefaultAdisAccelerometerTurnOnBiasSigma;
  accelParams.dynamicBiasStdDev = kDefaultAdisAccelerometerRandomWalk;
  accelParams.dynamicBiasCorrelationTime = kDefaultAdisAccelerometerBiasCorrelationTime;
  accelParams.precision = _precision;
  return accelParams;
}

// Generate a set of noise parameters for gyroscope channel
NoiseParameters gyroscopeParameters(double _sampleFreq, double _precision)
{
  const double dt = 1.0/_sampleFreq;
  NoiseParameters gyroParams;
  gyroParams.sampleFreq = _sampleFreq;
  gyroParams.stdDev = kDefaultAdisGyroscopeNoiseDensity * 1.0/sqrt(dt);
  gyroParams.biasStdDev = kDefaultAdisGyroscopeTurnOnBiasSigma;
  gyroParams.dynamicBiasStdDev = kDefaultAdisGyroscopeRandomWalk;
  gyroParams.dynamicBiasCorrelationTime = kDefaultAdisGyroscopeBiasCorrelationTime;
  gyroParams.precision = _precision;
  return gyroParams;
}

std::vector<double>
generateSamples(size_t _nSamples, const NoiseParameters& _params)
{
  std::vector<double> samples;

  sdf::Noise noise;
  noise.SetType(sdf::NoiseType::GAUSSIAN);

  noise.SetMean(_params.mean);
  noise.SetStdDev(_params.stdDev);
  noise.SetBiasMean(_params.biasMean);
  noise.SetBiasStdDev(_params.biasStdDev);
  noise.SetDynamicBiasStdDev(_params.dynamicBiasStdDev);
  noise.SetDynamicBiasCorrelationTime(_params.dynamicBiasCorrelationTime);

  const double dt = 1.0 / _params.sampleFreq;
  auto model = ignition::sensors::NoiseFactory::NewNoiseModel(noise);
  for (size_t ii = 0; ii < _nSamples ; ++ii) {
    samples.push_back(model->Apply(0.0, dt));
  }
  return samples;
}

void
writeSamples(const std::string& _fname, const std::vector<double>& _samples)
{
  std::ofstream out(_fname, std::ofstream::binary);
  out.write(reinterpret_cast<const char*>(_samples.data()), _samples.size() * sizeof(double));
}


int main()
{
  auto accelParams = accelerometerParameters(kSampleFrequency, kSamplePrecision);
  auto accelSamples = generateSamples(kNumSamples, accelParams);

  auto gyroParams = gyroscopeParameters(kSampleFrequency, kSamplePrecision);
  auto gyroSamples = generateSamples(kNumSamples, gyroParams);

  writeSamples("accel_samples.bin", accelSamples);
  writeSamples("gyro_samples.bin", gyroSamples);

  return 0;
}
