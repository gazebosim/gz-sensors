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
#include <gtest/gtest.h>
#include <sdf/sdf.hh>

#include <ignition/math/Angle.hh>
#include <ignition/math/Helpers.hh>
#ifdef _WIN32
#pragma warning(push)
#pragma warning(disable: 4005)
#pragma warning(disable: 4251)
#endif
#include <ignition/msgs.hh>
#ifdef _WIN32
#pragma warning(pop)
#endif

#include <ignition/sensors/Export.hh>
#include <ignition/sensors/Manager.hh>
#include <ignition/sensors/Lidar.hh>

sdf::ElementPtr LidarToSDF(const std::string &name, double update_rate,
    const std::string &topic, double horz_samples, double horz_resolution,
    double horz_min_angle, double horz_max_angle, double vert_samples,
    double vert_resolution, double vert_min_angle, double vert_max_angle,
    double range_resolution, double range_min, double range_max,
    bool always_on, bool visualize)
{
  std::ostringstream stream;
  stream
    << "<?xml version='1.0'?>"
    << "<sdf version='1.6'>"
    << " <model name='m1'>"
    << "  <link name='link1'>"
    << "    <sensor name='" << name << "' type='lidar'>"
    << "      <topic>" << topic << "</topic>"
    << "      <update_rate>"<< update_rate <<"</update_rate>"
    << "      <ray>"
    << "        <scan>"
    << "          <horizontal>"
    << "            <samples>" << horz_samples << "</samples>"
    << "            <resolution>" << horz_resolution << "</resolution>"
    << "            <min_angle>" << horz_min_angle << "</min_angle>"
    << "            <max_angle>" << horz_max_angle << "</max_angle>"
    << "          </horizontal>"
    << "          <vertical>"
    << "            <samples>" << vert_samples << "</samples>"
    << "            <resolution>" << vert_resolution << "</resolution>"
    << "            <min_angle>" << vert_min_angle << "</min_angle>"
    << "            <max_angle>" << vert_max_angle << "</max_angle>"
    << "          </vertical>"
    << "        </scan>"
    << "        <range>"
    << "          <min>" << range_min << "</min>"
    << "          <max>" << range_max << "</max>"
    << "          <resolution>" << range_resolution << "</resolution>"
    << "        </range>"
    << "      </ray>"
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

void OnNewLaserFrame(int *_scanCounter, float *_scanDest,
                  const float *_scan,
                  unsigned int _width, unsigned int _height,
                  unsigned int _depth,
                  const std::string &/*_format*/)
{
  memcpy(_scanDest, _scan, _width * _height * _depth);
  *_scanCounter += 1;
}

/// \brief Test lidar sensor
class Lidar_TEST : public ::testing::Test
{
  // Documentation inherited
  protected: void SetUp() override
  {
    ignition::common::Console::SetVerbosity(4);
  }
};

/////////////////////////////////////////////////
/// \brief Test Creation of a Lidar sensor
TEST(Lidar_TEST, CreateLaser)
{
  // Create a sensor manager
  ignition::sensors::Manager mgr;

  // Create SDF describing a camera sensor
  const std::string name = "TestLidar";
  const std::string topic = "/ignition/sensors/test/lidar";
  const double update_rate = 30;
  const double horz_samples = 640;
  const double horz_resolution = 1;
  const double horz_min_angle = -1.396263;
  const double horz_max_angle = 1.396263;
  const double vert_resolution = 1;
  const double vert_samples = 1;
  const double vert_min_angle = 0;
  const double vert_max_angle = 0;
  const double range_resolution = 0.01;
  const double range_min = 0.08;
  const double range_max = 10.0;
  const bool always_on = 1;
  const bool visualize = 1;

  sdf::ElementPtr lidarSDF = LidarToSDF(name, update_rate, topic,
    horz_samples, horz_resolution, horz_min_angle, horz_max_angle,
    vert_samples, vert_resolution, vert_min_angle, vert_max_angle,
    range_resolution, range_min, range_max, always_on, visualize);

  // Create a CameraSensor
  ignition::sensors::Lidar *sensor = mgr.CreateSensor<ignition::sensors::Lidar>(
      lidarSDF);

  EXPECT_FALSE(sensor->CreateLidar());

  // Make sure the above dynamic cast worked.
  ASSERT_NE(nullptr, sensor);

  double angleRes = (sensor->AngleMax() - sensor->AngleMin()).Radian() /
                    sensor->RayCount();
  EXPECT_EQ(sensor->AngleMin(), ignition::math::Angle(-1.396263));
  EXPECT_EQ(sensor->AngleMax(), ignition::math::Angle(1.396263));
  EXPECT_NEAR(sensor->RangeMin(), 0.08, 1e-6);
  EXPECT_NEAR(sensor->RangeMax(), 10.0, 1e-6);
  EXPECT_NEAR(sensor->AngleResolution(), angleRes, 1e-3);
  EXPECT_NEAR(sensor->RangeResolution(), 0.01, 1e-3);
  EXPECT_EQ(sensor->RayCount(), 640u);
  EXPECT_EQ(sensor->RangeCount(), 640u);

  EXPECT_EQ(sensor->VerticalRayCount(), 1u);
  EXPECT_EQ(sensor->VerticalRangeCount(), 1u);
  EXPECT_DOUBLE_EQ(sensor->VerticalAngleMin().Radian(), 0.0);
  EXPECT_DOUBLE_EQ(sensor->VerticalAngleMax().Radian(), 0.0);

  EXPECT_TRUE(sensor->IsActive());
}

TEST(Lidar_TEST, CreateLaserFailures)
{
  sdf::Sensor sdfSensor;
  sdfSensor.SetType(sdf::SensorType::CAMERA);
  sdf::Lidar sdfLidarSensor;

  ignition::sensors::Lidar sensor;
  EXPECT_FALSE(sensor.Load(sdfSensor));

  EXPECT_DOUBLE_EQ(0.0, sensor.Range(-1));
  EXPECT_DOUBLE_EQ(0.0, sensor.Range(0));

  EXPECT_FALSE(sensor.IsHorizontal());
  EXPECT_DOUBLE_EQ(640, sensor.RangeCountRatio());
  sensor.SetAngleMax(0.707);
  sensor.SetAngleMin(-0.707);
  EXPECT_DOUBLE_EQ(-0.707, sensor.AngleMin().Radian());
  EXPECT_DOUBLE_EQ(0.707, sensor.AngleMax().Radian());

  sensor.SetVerticalAngleMax(0.707);
  sensor.SetVerticalAngleMin(-0.707);
  EXPECT_DOUBLE_EQ(-0.707, sensor.VerticalAngleMin().Radian());
  EXPECT_DOUBLE_EQ(0.707, sensor.VerticalAngleMax().Radian());

  sensor.ApplyNoise();

  sdfSensor.SetType(sdf::SensorType::LIDAR);
  sdfSensor.SetLidarSensor(sdfLidarSensor);

  ignition::sensors::Lidar sensor2;

  EXPECT_TRUE(sensor2.Load(sdfSensor));
  EXPECT_FALSE(sensor2.Load(sdfSensor));

  ignition::sensors::Lidar sensor3;

  sdfLidarSensor.SetHorizontalScanSamples(0);
  sdfSensor.SetLidarSensor(sdfLidarSensor);
  EXPECT_TRUE(sensor3.Load(sdfSensor));

  sdfLidarSensor.SetHorizontalScanSamples(20);

  sdf::Noise noise;
  noise.SetType(sdf::NoiseType::GAUSSIAN);
  noise.SetMean(1.2);
  noise.SetStdDev(2.3);
  noise.SetBiasMean(4.5);
  noise.SetBiasStdDev(6.7);
  noise.SetPrecision(8.9);

  sdfLidarSensor.SetLidarNoise(noise);
  sdfSensor.SetLidarSensor(sdfLidarSensor);
  ignition::sensors::Lidar sensor4;
  EXPECT_TRUE(sensor4.Load(sdfSensor));

  noise.SetType(sdf::NoiseType::GAUSSIAN_QUANTIZED);
  sdfLidarSensor.SetLidarNoise(noise);
  sdfSensor.SetLidarSensor(sdfLidarSensor);
  ignition::sensors::Lidar sensor5;
  EXPECT_TRUE(sensor5.Load(sdfSensor));

  sensor.Update(ignition::common::Time(0.1));
}

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
