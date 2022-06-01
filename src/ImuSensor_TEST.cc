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
#include <sdf/sdf.hh>

#include <gz/math/Helpers.hh>
#ifdef _WIN32
#pragma warning(push)
#pragma warning(disable: 4005)
#pragma warning(disable: 4251)
#endif
#include <gz/msgs.hh>
#ifdef _WIN32
#pragma warning(pop)
#endif

#include <gz/common/Console.hh>
#include <gz/sensors/Export.hh>
#include <gz/sensors/ImuSensor.hh>
#include <gz/sensors/Manager.hh>

using namespace gz;

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

// Generate a set of noise parameters for zero noise
NoiseParameters noNoiseParameters(double _sampleFreq, double _precision)
{
  NoiseParameters params;
  params.sampleFreq = _sampleFreq;
  params.mean = 0.0;
  params.stdDev = 0.0;
  params.biasMean = 0.0;
  params.biasStdDev = 0.0;
  params.dynamicBiasStdDev = 0.0;
  params.dynamicBiasCorrelationTime = 0.0;
  params.precision = _precision;
  return params;
}

// Generate a set of noise parameters for accelerometer channel
NoiseParameters accelerometerParameters(double _sampleFreq, double _precision)
{
  const double dt = 1.0/_sampleFreq;
  NoiseParameters accelParams;
  accelParams.sampleFreq = _sampleFreq;
  accelParams.stdDev = kDefaultAdisAccelerometerNoiseDensity * 1.0/sqrt(dt);
  accelParams.biasStdDev = kDefaultAdisAccelerometerTurnOnBiasSigma;
  accelParams.dynamicBiasStdDev = kDefaultAdisAccelerometerRandomWalk;
  accelParams.dynamicBiasCorrelationTime =
    kDefaultAdisAccelerometerBiasCorrelationTime;
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
  gyroParams.dynamicBiasCorrelationTime =
    kDefaultAdisGyroscopeBiasCorrelationTime;
  gyroParams.precision = _precision;
  return gyroParams;
}

sdf::ElementPtr ImuSensorToSDF(const std::string &name, double update_rate,
    const std::string &topic, const NoiseParameters& accelNoise,
    const NoiseParameters& gyroNoise, bool always_on, bool visualize)
{
  auto noiseToSDF = [](NoiseParameters params) {
    std::ostringstream stream;
    for (const auto & channel : { "x", "y", "z" }) {
      stream
        << "          <" << channel << ">"
        << "            <noise type='gaussian'>"
        << "              <mean>" << params.mean << "</mean>"
        << "              <stddev>" << params.stdDev << "</stddev>"
        << "              <bias_mean>" << params.biasMean << "</bias_mean>"
        << "              <bias_stddev>"
        << params.biasStdDev
        << "</bias_stddev>"
        << "              <dynamic_bias_stddev>"
        << params.dynamicBiasStdDev
        << "</dynamic_bias_stddev>"
        << "              <dynamic_bias_correlation_time>"
        << params.dynamicBiasCorrelationTime
        << "</dynamic_bias_correlation_time>"
        << "              <precision>" << params.precision << "</precision>"
        << "            </noise>"
        << "          </" << channel << ">";
    }
    return stream.str();
  };

  std::ostringstream stream;
  stream
    << "<?xml version='1.0'?>"
    << "<sdf version='1.6'>"
    << " <model name='m1'>"
    << "  <link name='link1'>"
    << "    <sensor name='" << name << "' type='imu'>"
    << "      <topic>" << topic << "</topic>"
    << "      <update_rate>"<< update_rate <<"</update_rate>"
    << "      <imu>"
    << "        <angular_velocity>"
    << noiseToSDF(gyroNoise)
    << "        </angular_velocity>"
    << "        <linear_acceleration>"
    << noiseToSDF(accelNoise)
    << "        </linear_acceleration>"
    << "      </imu>"
    << "      <always_on>"<< always_on <<"</always_on>"
    << "      <visualize>" << visualize << "</visualize>"
    << "    </sensor>"
    << "  </link>"
    << " </model>"
    << "</sdf>";

  sdf::SDFPtr sdfParsed(new sdf::SDF());
  sdf::init(sdfParsed);
  if (!sdf::readString(stream.str(), sdfParsed))
    return sdf::ElementPtr();

  return sdfParsed->Root()->GetElement("model")->GetElement("link")
    ->GetElement("sensor");
}

/// \brief Test IMU sensor
class ImuSensor_TEST : public ::testing::Test
{
  // Documentation inherited
  protected: void SetUp() override
  {
    gz::common::Console::SetVerbosity(3);
  }
};

//////////////////////////////////////////////////
TEST(ImuSensor_TEST, CreateImuSensor)
{
  // Create a sensor manager
  gz::sensors::Manager mgr;

  const std::string name = "TestImu";
  const std::string topic = "/ignition/sensors/test/imu";
  const double update_rate = 100;
  const auto accelNoise = accelerometerParameters(update_rate, 0.0);
  const auto gyroNoise = gyroscopeParameters(update_rate, 0.0);
  const bool always_on = 1;
  const bool visualize = 1;

  sdf::ElementPtr imuSDF = ImuSensorToSDF(name, update_rate, topic,
    accelNoise, gyroNoise, always_on, visualize);

  // Create an ImuSensor
  auto sensor = mgr.CreateSensor<gz::sensors::ImuSensor>(imuSDF);

  // Make sure the above dynamic cast worked.
  EXPECT_TRUE(sensor != nullptr);
}

//////////////////////////////////////////////////
TEST(ImuSensor_TEST, ComputeNoise)
{
  // Create a sensor manager
  gz::sensors::Manager mgr;

  sdf::ElementPtr imuSDF, imuSDF_truth;

  {
    const std::string name = "TestImu_Truth";
    const std::string topic = "/ignition/sensors/test/imu_truth";
    const double update_rate = 100;
    const auto accelNoise = noNoiseParameters(update_rate, 0.0);
    const auto gyroNoise = noNoiseParameters(update_rate, 0.0);
    const bool always_on = 1;
    const bool visualize = 1;

    imuSDF_truth = ImuSensorToSDF(name, update_rate, topic,
      accelNoise, gyroNoise, always_on, visualize);
  }

  {
    const std::string name = "TestImu_Truth";
    const std::string topic = "/ignition/sensors/test/imu_truth";
    const double update_rate = 100;
    auto accelNoise = accelerometerParameters(update_rate, 0.0);
    auto gyroNoise = gyroscopeParameters(update_rate, 0.0);
    // Small bias to make sure that the results won't equal zero.
    accelNoise.biasMean = 0.01;
    gyroNoise.biasMean = 0.01;
    const bool always_on = 1;
    const bool visualize = 1;

    imuSDF = ImuSensorToSDF(name, update_rate, topic,
      accelNoise, gyroNoise, always_on, visualize);
  }

  // Create an ImuSensor
  auto sensor_truth = mgr.CreateSensor<gz::sensors::ImuSensor>(
      imuSDF_truth);
  auto sensor = mgr.CreateSensor<gz::sensors::ImuSensor>(imuSDF);

  // Make sure the above dynamic cast worked.
  ASSERT_NE(nullptr, sensor);
  ASSERT_NE(nullptr, sensor_truth);

  sensor->SetAngularVelocity(math::Vector3d::Zero);
  sensor->SetLinearAcceleration(math::Vector3d::Zero);
  sensor->SetWorldPose(math::Pose3d::Zero);
  // Note no gravity.
  sensor->SetGravity(math::Vector3d::Zero);

  sensor_truth->SetLinearAcceleration(math::Vector3d::Zero);
  sensor_truth->SetAngularVelocity(math::Vector3d::Zero);
  sensor_truth->SetWorldPose(math::Pose3d::Zero);
  // Note no gravity.
  sensor_truth->SetGravity(math::Vector3d::Zero);

  sensor->Update(std::chrono::steady_clock::duration(
    std::chrono::nanoseconds(10000000)));
  sensor_truth->Update(std::chrono::steady_clock::duration(
    std::chrono::nanoseconds(10000000)));

  // Since this IMU has no noise, measurements should equal the inputs.
  EXPECT_DOUBLE_EQ(sensor_truth->AngularVelocity().X(), 0.0);
  EXPECT_DOUBLE_EQ(sensor_truth->AngularVelocity().Y(), 0.0);
  EXPECT_DOUBLE_EQ(sensor_truth->AngularVelocity().Z(), 0.0);
  EXPECT_DOUBLE_EQ(sensor_truth->LinearAcceleration().X(), 0.0);
  EXPECT_DOUBLE_EQ(sensor_truth->LinearAcceleration().Y(), 0.0);
  EXPECT_DOUBLE_EQ(sensor_truth->LinearAcceleration().Z(), 0.0);

  // Since this IMU has noise, we don't expect it to equal truth.
  //
  // \TODO(mjcarroll): It's _very_ unlikely any of these numbers
  // will be exactly zero, but there are probably better ways
  // of verifying the noise is correctly added statistically.
  EXPECT_GT(sensor->AngularVelocity().SquaredLength(), 0.0);
  EXPECT_GT(sensor->LinearAcceleration().SquaredLength(), 0.0);

  sensor->Update(std::chrono::steady_clock::duration(
    std::chrono::nanoseconds(20000000)));
  sensor_truth->Update(std::chrono::steady_clock::duration(
    std::chrono::nanoseconds(20000000)));

  // Since this IMU has no noise, measurements should equal the inputs.
  EXPECT_DOUBLE_EQ(sensor_truth->AngularVelocity().X(), 0.0);
  EXPECT_DOUBLE_EQ(sensor_truth->AngularVelocity().Y(), 0.0);
  EXPECT_DOUBLE_EQ(sensor_truth->AngularVelocity().Z(), 0.0);
  EXPECT_DOUBLE_EQ(sensor_truth->LinearAcceleration().X(), 0.0);
  EXPECT_DOUBLE_EQ(sensor_truth->LinearAcceleration().Y(), 0.0);
  EXPECT_DOUBLE_EQ(sensor_truth->LinearAcceleration().Z(), 0.0);

  EXPECT_GT(sensor->AngularVelocity().SquaredLength(), 0.0);
  EXPECT_GT(sensor->LinearAcceleration().SquaredLength(), 0.0);
}

//////////////////////////////////////////////////
TEST(ImuSensor_TEST, Orientation)
{
  // Create a sensor manager
  gz::sensors::Manager mgr;

  sdf::ElementPtr imuSDF;

  {
    const std::string name = "TestImu_Truth";
    const std::string topic = "/ignition/sensors/test/imu_truth";
    const double updateRate = 100;
    const auto accelNoise = noNoiseParameters(updateRate, 0.0);
    const auto gyroNoise = noNoiseParameters(updateRate, 0.0);
    const bool always_on = 1;
    const bool visualize = 1;

    imuSDF = ImuSensorToSDF(name, updateRate, topic,
      accelNoise, gyroNoise, always_on, visualize);
  }

  // Create an ImuSensor
  auto sensor = mgr.CreateSensor<gz::sensors::ImuSensor>(
      imuSDF);

  // Make sure the above dynamic cast worked.
  ASSERT_NE(nullptr, sensor);

  math::Quaterniond orientRef;
  math::Quaterniond orientValue(math::Vector3d(IGN_PI/2.0, 0, IGN_PI));
  math::Pose3d pose(math::Vector3d(0, 1, 2), orientValue);

  sensor->SetOrientationReference(orientRef);
  sensor->SetWorldPose(pose);

  sensor->Update(std::chrono::steady_clock::duration(
    std::chrono::nanoseconds(10000000)));

  // Check orientation
  EXPECT_TRUE(sensor->OrientationEnabled());
  EXPECT_EQ(orientRef, sensor->OrientationReference());
  EXPECT_EQ(orientValue, sensor->Orientation());

  // update pose and check orientation
  math::Quaterniond newOrientValue(math::Vector3d(IGN_PI, IGN_PI/2, IGN_PI));
  math::Pose3d newPose(math::Vector3d(0, 1, 1), newOrientValue);
  sensor->SetWorldPose(newPose);

  sensor->Update(std::chrono::steady_clock::duration(
    std::chrono::nanoseconds(20000000)));

  EXPECT_TRUE(sensor->OrientationEnabled());
  EXPECT_EQ(orientRef, sensor->OrientationReference());
  EXPECT_EQ(newOrientValue, sensor->Orientation());

  // disable orientation and check
  sensor->SetOrientationEnabled(false);
  EXPECT_FALSE(sensor->OrientationEnabled());
  EXPECT_EQ(orientRef, sensor->OrientationReference());
  // orientation remains the same after disabling orientation
  EXPECT_EQ(newOrientValue, sensor->Orientation());

  // update world pose with orientation disabled and verify that orientation
  // does not change
  math::Quaterniond newOrientValue2(math::Vector3d(IGN_PI/2, IGN_PI/2, IGN_PI));
  math::Pose3d newPose2(math::Vector3d(1, 1, 0), newOrientValue2);
  sensor->SetWorldPose(newPose2);
  sensor->Update(std::chrono::steady_clock::duration(
    std::chrono::nanoseconds(20000000)));

  sensor->SetOrientationEnabled(false);
  EXPECT_FALSE(sensor->OrientationEnabled());
  EXPECT_EQ(orientRef, sensor->OrientationReference());
  // orientation should still be the previous value because it is not being
  // updated.
  EXPECT_EQ(newOrientValue, sensor->Orientation());

}

//////////////////////////////////////////////////
TEST(ImuSensor_TEST, OrientationReference)
{
  // Create a sensor manager
  gz::sensors::Manager mgr;

  sdf::ElementPtr imuSDF;

  std::ostringstream stream;
  stream
    << "<?xml version='1.0'?>"
    << "<sdf version='1.6'>"
    << " <model name='m1'>"
    << "  <link name='link1'>"
    << "    <sensor name='imu_sensor' type='imu'>"
    << "      <topic>imu_test</topic>"
    << "      <update_rate>1</update_rate>"
    << "      <imu>"
    << "      <orientation_reference_frame>"
    << "      <localization>CUSTOM</localization>"
    << "      <custom_rpy parent_frame='world'>1.570795 0 0</custom_rpy>"
    << "      </orientation_reference_frame>"
    << "      </imu>"
    << "      <always_on>1</always_on>"
    << "      <visualize>true</visualize>"
    << "    </sensor>"
    << "  </link>"
    << " </model>"
    << "</sdf>";

  sdf::SDFPtr sdfParsed(new sdf::SDF());
  sdf::init(sdfParsed);
  if (!sdf::readString(stream.str(), sdfParsed))
  {
    imuSDF = sdf::ElementPtr();
  }
  else
  {
    imuSDF = sdfParsed->Root()->GetElement("model")->GetElement("link")
      ->GetElement("sensor");
  }

  // Create an ImuSensor
  auto sensor = mgr.CreateSensor<gz::sensors::ImuSensor>(
      imuSDF);
  sensor->SetWorldFrameOrientation(math::Quaterniond(0, 0, 0),
    sensors::WorldFrameEnumType::ENU);

  // Make sure the above dynamic cast worked.
  ASSERT_NE(nullptr, sensor);

  sensor->Update(std::chrono::steady_clock::duration(
    std::chrono::nanoseconds(10000000)));

  math::Quaterniond orientValue(math::Vector3d(-1.570795, 0, 0));
  EXPECT_EQ(orientValue, sensor->Orientation());

}

//////////////////////////////////////////////////
TEST(ImuSensor_TEST, CustomRpyParentFrame)
{
  // Create a sensor manager
  gz::sensors::Manager mgr;

  sdf::ElementPtr imuSDF;

  std::ostringstream stream;
  stream
    << "<?xml version='1.0'?>"
    << "<sdf version='1.6'>"
    << " <model name='m1'>"
    << "  <link name='link1'>"
    << "    <sensor name='imu_sensor' type='imu'>"
    << "      <topic>imu_test</topic>"
    << "      <update_rate>1</update_rate>"
    << "      <imu>"
    << "      <orientation_reference_frame>"
    << "      <localization>CUSTOM</localization>"
    << "      <custom_rpy parent_frame='some_frame'>"
    << "      1.570795 0 0"
    << "      </custom_rpy>"
    << "      </orientation_reference_frame>"
    << "      </imu>"
    << "      <always_on>1</always_on>"
    << "      <visualize>true</visualize>"
    << "    </sensor>"
    << "  </link>"
    << " </model>"
    << "</sdf>";

  sdf::SDFPtr sdfParsed(new sdf::SDF());
  sdf::init(sdfParsed);
  if (!sdf::readString(stream.str(), sdfParsed))
  {
    imuSDF = sdf::ElementPtr();
  }
  else
  {
    imuSDF = sdfParsed->Root()->GetElement("model")->GetElement("link")
      ->GetElement("sensor");
  }

  // Create an ImuSensor
  auto sensor = mgr.CreateSensor<gz::sensors::ImuSensor>(
      imuSDF);

  sensor->SetWorldFrameOrientation(math::Quaterniond(0, 0, 0),
    sensors::WorldFrameEnumType::ENU);

  // Make sure the above dynamic cast worked.
  ASSERT_NE(nullptr, sensor);

  sensor->Update(std::chrono::steady_clock::duration(
    std::chrono::nanoseconds(10000000)));

  // Since parent_frame is not set to 'world' in the sdf,
  // custom_rpy will be rejected and reference orientation will
  // not be set.
  math::Quaterniond orientValue(math::Vector3d(-1.570795, 0, 0));
  math::Quaterniond defultOrientValue(math::Vector3d(0, 0, 0));
  ASSERT_NE(orientValue, sensor->Orientation());
  EXPECT_EQ(defultOrientValue, sensor->Orientation());
}

sdf::ElementPtr sensorWithLocalization(
  std::string _orientationLocalization)
{
  std::ostringstream stream;
  stream
    << "<?xml version='1.0'?>"
    << "<sdf version='1.6'>"
    << " <model name='m1'>"
    << "  <link name='link1'>"
    << "    <sensor name='imu_sensor' type='imu'>"
    << "      <topic>imu_test</topic>"
    << "      <update_rate>1</update_rate>"
    << "      <imu>"
    << "      <orientation_reference_frame>"
    << "      <localization>"+_orientationLocalization+"</localization>"
    << "      </orientation_reference_frame>"
    << "      </imu>"
    << "      <always_on>1</always_on>"
    << "      <visualize>true</visualize>"
    << "    </sensor>"
    << "  </link>"
    << " </model>"
    << "</sdf>";

  sdf::SDFPtr sdfParsed(new sdf::SDF());
  sdf::init(sdfParsed);
  if (!sdf::readString(stream.str(), sdfParsed))
  {
    return sdf::ElementPtr();
  }
  else
  {
    return sdfParsed->Root()->GetElement("model")->GetElement("link")
      ->GetElement("sensor");
  }
}

//////////////////////////////////////////////////
TEST(ImuSensor_TEST, NamedFrameOrientationReference)
{
  // Create a sensor manager
  gz::sensors::Manager mgr;

  math::Quaterniond orientValue;

  // A. Localization tag is set to ENU
  // ---------------------------------
  auto sensorENU = mgr.CreateSensor<gz::sensors::ImuSensor>(
      sensorWithLocalization("ENU"));
  ASSERT_NE(nullptr, sensorENU);

  // Case A.1 : Localization ref: ENU, World : ENU
  // Sensor aligns with ENU, we're measuring wrt ENU
  {
    const math::Quaterniond worldFrameOrientation(0, 0, 0);
    const sensors::WorldFrameEnumType worldRelativeTo =
      sensors::WorldFrameEnumType::ENU;
    const math::Quaterniond expectedSensorOrientation(0, 0, 0);

    sensorENU->SetWorldFrameOrientation(worldFrameOrientation,
      worldRelativeTo);
    sensorENU->Update(std::chrono::steady_clock::duration(
      std::chrono::nanoseconds(10000000)));
    EXPECT_EQ(expectedSensorOrientation, sensorENU->Orientation());
  }

  // Case A.2 Localization ref: ENU, World : ENU + rotation offset
  {
    const math::Quaterniond worldFrameOrientation(0, 0, IGN_PI/4);
    const sensors::WorldFrameEnumType worldRelativeTo =
      sensors::WorldFrameEnumType::ENU;
    const math::Quaterniond expectedSensorOrientation(0, 0, -IGN_PI/4);

    sensorENU->SetWorldFrameOrientation(worldFrameOrientation,
      worldRelativeTo);
    sensorENU->Update(std::chrono::steady_clock::duration(
      std::chrono::nanoseconds(10000000)));
    EXPECT_EQ(expectedSensorOrientation, sensorENU->Orientation());
  }

  // Case A.3 Localization ref: ENU, World : NWU
  // Sensor aligns with NWU, we're measuring wrt ENU
  {
    const math::Quaterniond worldFrameOrientation(0, 0, 0);
    const sensors::WorldFrameEnumType worldRelativeTo =
      sensors::WorldFrameEnumType::NWU;
    const math::Quaterniond expectedSensorOrientation(0, 0, IGN_PI/2);

    sensorENU->SetWorldFrameOrientation(worldFrameOrientation,
      worldRelativeTo);
    sensorENU->Update(std::chrono::steady_clock::duration(
      std::chrono::nanoseconds(10000000)));
    EXPECT_EQ(expectedSensorOrientation, sensorENU->Orientation());
  }

  // Case A.4 Localization ref: ENU, World : NED
  // Sensor aligns with NED, we're measuring wrt ENU
  {
    const math::Quaterniond worldFrameOrientation(0, 0, 0);
    const sensors::WorldFrameEnumType worldRelativeTo =
      sensors::WorldFrameEnumType::NED;
    const math::Quaterniond expectedSensorOrientation(IGN_PI, 0, IGN_PI/2);

    sensorENU->SetWorldFrameOrientation(worldFrameOrientation,
      worldRelativeTo);
    sensorENU->Update(std::chrono::steady_clock::duration(
      std::chrono::nanoseconds(10000000)));
    EXPECT_EQ(expectedSensorOrientation, sensorENU->Orientation());
  }

  // B. Localization tag is set to NWU
  // ---------------------------------
  auto sensorNWU = mgr.CreateSensor<gz::sensors::ImuSensor>(
      sensorWithLocalization("NWU"));
  ASSERT_NE(nullptr, sensorNWU);

  // Case B.1 : Localization ref: NWU, World : NWU
  // Sensor aligns with NWU, we're measuring wrt NWU
  {
    const math::Quaterniond worldFrameOrientation(0, 0, 0);
    const sensors::WorldFrameEnumType worldRelativeTo =
      sensors::WorldFrameEnumType::NWU;
    const math::Quaterniond expectedSensorOrientation(0, 0, 0);

    sensorNWU->SetWorldFrameOrientation(worldFrameOrientation,
      worldRelativeTo);
    sensorNWU->Update(std::chrono::steady_clock::duration(
      std::chrono::nanoseconds(10000000)));
    EXPECT_EQ(expectedSensorOrientation, sensorNWU->Orientation());
  }

  // Case B.2 : Localization ref: NWU, World : NWU + rotation offset
  {
    const math::Quaterniond worldFrameOrientation(0, 0, IGN_PI/4);
    const sensors::WorldFrameEnumType worldRelativeTo =
      sensors::WorldFrameEnumType::NWU;
    const math::Quaterniond expectedSensorOrientation(0, 0, -IGN_PI/4);

    sensorNWU->SetWorldFrameOrientation(worldFrameOrientation,
      worldRelativeTo);
    sensorNWU->Update(std::chrono::steady_clock::duration(
      std::chrono::nanoseconds(10000000)));
    EXPECT_EQ(expectedSensorOrientation, sensorNWU->Orientation());
  }

  // Case B.3 : Localization ref: NWU, World : ENU
  // Sensor aligns with ENU, we're measuring wrt NWU
  {
    const math::Quaterniond worldFrameOrientation(0, 0, 0);
    const sensors::WorldFrameEnumType worldRelativeTo =
      sensors::WorldFrameEnumType::ENU;
    const math::Quaterniond expectedSensorOrientation(0, 0, -IGN_PI/2);

    sensorNWU->SetWorldFrameOrientation(worldFrameOrientation,
      worldRelativeTo);
    sensorNWU->Update(std::chrono::steady_clock::duration(
      std::chrono::nanoseconds(10000000)));
    EXPECT_EQ(expectedSensorOrientation, sensorNWU->Orientation());
  }

  // Case B.4 : Localization ref: NWU, World : NED
  // Sensor aligns with NED, we're measuring wrt NWU
  {
    const math::Quaterniond worldFrameOrientation(0, 0, 0);
    const sensors::WorldFrameEnumType worldRelativeTo =
      sensors::WorldFrameEnumType::NED;
    const math::Quaterniond expectedSensorOrientation(IGN_PI, 0, 0);

    sensorNWU->SetWorldFrameOrientation(worldFrameOrientation,
      worldRelativeTo);
    sensorNWU->Update(std::chrono::steady_clock::duration(
      std::chrono::nanoseconds(10000000)));
    EXPECT_EQ(expectedSensorOrientation, sensorNWU->Orientation());
  }

  // C. Localization tag is set to NED
  // ---------------------------------
  auto sensorNED = mgr.CreateSensor<gz::sensors::ImuSensor>(
      sensorWithLocalization("NED"));
  ASSERT_NE(nullptr, sensorNED);

  // Case C.1 : Localization ref: NED, World : NED
  // Sensor aligns with NED, we're measuring wrt NED
  {
    const math::Quaterniond worldFrameOrientation(0, 0, 0);
    const sensors::WorldFrameEnumType worldRelativeTo =
      sensors::WorldFrameEnumType::NED;
    const math::Quaterniond expectedSensorOrientation(0, 0, 0);

    sensorNED->SetWorldFrameOrientation(worldFrameOrientation,
      worldRelativeTo);
    sensorNED->Update(std::chrono::steady_clock::duration(
      std::chrono::nanoseconds(10000000)));
    EXPECT_EQ(expectedSensorOrientation, sensorNED->Orientation());
  }

  // Case C.2 : Localization ref: NED, World : NED + rotation offset
  {
    const math::Quaterniond worldFrameOrientation(0, 0, IGN_PI/4);
    const sensors::WorldFrameEnumType worldRelativeTo =
      sensors::WorldFrameEnumType::NED;
    const math::Quaterniond expectedSensorOrientation(0, 0, -IGN_PI/4);

    sensorNED->SetWorldFrameOrientation(worldFrameOrientation,
      worldRelativeTo);
    sensorNED->Update(std::chrono::steady_clock::duration(
      std::chrono::nanoseconds(10000000)));
    EXPECT_EQ(expectedSensorOrientation, sensorNED->Orientation());
  }

  // Case C.3 : Localization ref: NED, World : NWU
  // Sensor aligns with NWU, we're measuring wrt NED
  {
    const math::Quaterniond worldFrameOrientation(0, 0, 0);
    const sensors::WorldFrameEnumType worldRelativeTo =
      sensors::WorldFrameEnumType::NWU;
    const math::Quaterniond expectedSensorOrientation(-IGN_PI, 0, 0);

    sensorNED->SetWorldFrameOrientation(worldFrameOrientation,
      worldRelativeTo);
    sensorNED->Update(std::chrono::steady_clock::duration(
      std::chrono::nanoseconds(10000000)));
    EXPECT_EQ(expectedSensorOrientation, sensorNED->Orientation());
  }

  // Case C.4 : Localization ref: NED, World : ENU
  // Sensor aligns with ENU, we're measuring wrt NED
  {
    const math::Quaterniond worldFrameOrientation(0, 0, 0);
    const sensors::WorldFrameEnumType worldRelativeTo =
      sensors::WorldFrameEnumType::ENU;
    const math::Quaterniond expectedSensorOrientation(-IGN_PI, 0, IGN_PI/2);

    sensorNED->SetWorldFrameOrientation(worldFrameOrientation,
      worldRelativeTo);
    sensorNED->Update(std::chrono::steady_clock::duration(
      std::chrono::nanoseconds(10000000)));
    EXPECT_EQ(expectedSensorOrientation, sensorNED->Orientation());
  }
}

//////////////////////////////////////////////////
TEST(ImuSensor_TEST, LocalizationTagInvalid)
{
  // Create a sensor manager
  gz::sensors::Manager mgr;

  sdf::ElementPtr imuSDF;

  std::ostringstream stream;
  stream
    << "<?xml version='1.0'?>"
    << "<sdf version='1.6'>"
    << " <model name='m1'>"
    << "  <link name='link1'>"
    << "    <sensor name='imu_sensor' type='imu'>"
    << "      <topic>imu_test</topic>"
    << "      <update_rate>1</update_rate>"
    << "      <imu>"
    << "      <orientation_reference_frame>"
    << "      <localization>UNRECOGNIZED</localization>"
    << "      </orientation_reference_frame>"
    << "      </imu>"
    << "      <always_on>1</always_on>"
    << "      <visualize>true</visualize>"
    << "    </sensor>"
    << "  </link>"
    << " </model>"
    << "</sdf>";

  sdf::SDFPtr sdfParsed(new sdf::SDF());
  sdf::init(sdfParsed);
  if (!sdf::readString(stream.str(), sdfParsed))
  {
    imuSDF = sdf::ElementPtr();
  }
  else
  {
    imuSDF = sdfParsed->Root()->GetElement("model")->GetElement("link")
      ->GetElement("sensor");
  }

  // Create an ImuSensor
  auto sensor = mgr.CreateSensor<gz::sensors::ImuSensor>(
      imuSDF);
  sensor->SetWorldFrameOrientation(math::Quaterniond(0, 0, 0),
    sensors::WorldFrameEnumType::ENU);

  // Make sure the above dynamic cast worked.
  ASSERT_NE(nullptr, sensor);

  sensor->Update(std::chrono::steady_clock::duration(
    std::chrono::nanoseconds(10000000)));

  math::Quaterniond orientValue(math::Vector3d(0, 0, 0));
  EXPECT_EQ(orientValue, sensor->Orientation());
}

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
