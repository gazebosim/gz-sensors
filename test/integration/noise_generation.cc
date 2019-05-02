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

#include <gtest/gtest.h>

#include <memory>

#include <sdf/sdf.hh>

#include <ignition/sensors/Noise.hh>
#include <ignition/sensors/GaussianNoiseModel.hh>
#include <ignition/sensors/SensorFactory.hh>

#include "test_config.h"  // NOLINT(build/include)
#include "TransportTestTools.hh"

static constexpr double kDefaultAdisGyroscopeNoiseDensity =
    2.0 * 35.0 / 3600.0 / 180.0 * M_PI;
static constexpr double kDefaultAdisGyroscopeRandomWalk =
    2.0 * 4.0 / 3600.0 / 180.0 * M_PI;
static constexpr double kDefaultAdisGyroscopeBiasCorrelationTime =
    1.0e+3;
static constexpr double kDefaultAdisGyroscopeTurnOnBiasSigma =
    0.5 / 180.0 * M_PI;

class NoiseGenerationTest: public testing::Test
{
};


/////////////////////////////////////////////////
TEST_F(NoiseGenerationTest, SensorReadings)
{
  const double f = 100.0;
  const double dt = 1/f;

  sdf::Noise noise;
  noise.SetType(sdf::NoiseType::GAUSSIAN);
  noise.SetMean(0.0);
  noise.SetStdDev(kDefaultAdisGyroscopeNoiseDensity * sqrt(f));
  noise.SetBiasMean(0.0);
  noise.SetBiasStdDev(kDefaultAdisGyroscopeTurnOnBiasSigma);
  noise.SetDynamicBiasStdDev(kDefaultAdisGyroscopeRandomWalk * sqrt(f));
  noise.SetDynamicBiasCorrelationTime(kDefaultAdisGyroscopeBiasCorrelationTime);
  noise.SetPrecision(1.0/1024.0);

  auto model = ignition::sensors::NoiseFactory::NewNoiseModel(noise);

  auto gmodel = std::dynamic_pointer_cast<ignition::sensors::GaussianNoiseModel>(model);

  for (size_t ii = 0; ii < 100000; ++ii) {
    std::cout << gmodel->Apply(0.0, dt) << ", " << gmodel->Bias() << std::endl;
  }

}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

