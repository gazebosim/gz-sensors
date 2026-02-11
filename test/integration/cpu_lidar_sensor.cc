/*
 * Copyright (C) 2025 Open Source Robotics Foundation
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

#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include <sdf/sdf.hh>

#include <gz/common/Console.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/sensors/CpuLidarSensor.hh>
#include <gz/sensors/SensorFactory.hh>

#include "test_config.hh"  // NOLINT(build/include)

/// \brief Helper function to create a cpu lidar sdf element
sdf::ElementPtr CpuLidarToSdf(const std::string &_name,
    const gz::math::Pose3d &_pose, const double _updateRate,
    const std::string &_topic, const double _horzSamples,
    const double _horzMinAngle, const double _horzMaxAngle,
    const double _vertSamples, const double _vertMinAngle,
    const double _vertMaxAngle, const double _rangeMin,
    const double _rangeMax, const bool _alwaysOn,
    const bool _visualize)
{
  std::ostringstream stream;
  stream
    << "<?xml version='1.0'?>"
    << "<sdf version='1.6'>"
    << " <model name='m1'>"
    << "  <link name='link1'>"
    << "    <sensor name='" << _name << "' type='lidar'>"
    << "      <pose>" << _pose << "</pose>"
    << "      <topic>" << _topic << "</topic>"
    << "      <update_rate>" << _updateRate << "</update_rate>"
    << "      <ray>"
    << "        <scan>"
    << "          <horizontal>"
    << "            <samples>" << _horzSamples << "</samples>"
    << "            <resolution>1</resolution>"
    << "            <min_angle>" << _horzMinAngle << "</min_angle>"
    << "            <max_angle>" << _horzMaxAngle << "</max_angle>"
    << "          </horizontal>"
    << "          <vertical>"
    << "            <samples>" << _vertSamples << "</samples>"
    << "            <resolution>1</resolution>"
    << "            <min_angle>" << _vertMinAngle << "</min_angle>"
    << "            <max_angle>" << _vertMaxAngle << "</max_angle>"
    << "          </vertical>"
    << "        </scan>"
    << "        <range>"
    << "          <min>" << _rangeMin << "</min>"
    << "          <max>" << _rangeMax << "</max>"
    << "          <resolution>0.01</resolution>"
    << "        </range>"
    << "      </ray>"
    << "      <alwaysOn>" << _alwaysOn << "</alwaysOn>"
    << "      <visualize>" << _visualize << "</visualize>"
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

/// \brief Test CPU Lidar sensor
class CpuLidarSensorTest : public testing::Test
{
  protected: void SetUp() override
  {
    gz::common::Console::SetVerbosity(4);
  }
};

/////////////////////////////////////////////////
TEST_F(CpuLidarSensorTest, CreateSensor)
{
  const std::string name = "TestCpuLidar";
  const std::string topic = "/gz/sensors/test/cpu_lidar";
  const double updateRate = 10;

  gz::math::Pose3d sensorPose(gz::math::Vector3d(0.25, 0.0, 0.5),
      gz::math::Quaterniond::Identity);
  auto sdf = CpuLidarToSdf(name, sensorPose, updateRate, topic,
      640, -1.396, 1.396, 1, 0, 0, 0.08, 10.0, true, false);
  ASSERT_NE(nullptr, sdf);

  gz::sensors::SensorFactory sf;
  auto sensor = sf.CreateSensor<gz::sensors::CpuLidarSensor>(sdf);
  ASSERT_NE(nullptr, sensor);

  EXPECT_EQ(name, sensor->Name());
  EXPECT_EQ(topic, sensor->Topic());
  EXPECT_DOUBLE_EQ(updateRate, sensor->UpdateRate());
}

/////////////////////////////////////////////////
TEST_F(CpuLidarSensorTest, LidarConfig)
{
  gz::math::Pose3d sensorPose(gz::math::Vector3d::Zero,
      gz::math::Quaterniond::Identity);
  auto sdf = CpuLidarToSdf("test_config", sensorPose, 10,
      "/test/config",
      640, -1.396, 1.396,
      16, -0.26, 0.26,
      0.08, 10.0, true, false);
  ASSERT_NE(nullptr, sdf);

  gz::sensors::SensorFactory sf;
  auto sensor = sf.CreateSensor<gz::sensors::CpuLidarSensor>(sdf);
  ASSERT_NE(nullptr, sensor);

  EXPECT_EQ(640u, sensor->RayCount());
  EXPECT_EQ(16u, sensor->VerticalRayCount());
  EXPECT_NEAR(-1.396, sensor->AngleMin().Radian(), 1e-6);
  EXPECT_NEAR(1.396, sensor->AngleMax().Radian(), 1e-6);
  EXPECT_NEAR(-0.26, sensor->VerticalAngleMin().Radian(), 1e-6);
  EXPECT_NEAR(0.26, sensor->VerticalAngleMax().Radian(), 1e-6);
  EXPECT_DOUBLE_EQ(0.08, sensor->RangeMin());
  EXPECT_DOUBLE_EQ(10.0, sensor->RangeMax());
}

/////////////////////////////////////////////////
TEST_F(CpuLidarSensorTest, GenerateRays)
{
  // 3 horizontal rays over 180° FOV, 1 vertical layer, range [0.1, 5.0]
  gz::math::Pose3d sensorPose(gz::math::Vector3d::Zero,
      gz::math::Quaterniond::Identity);
  auto sdf = CpuLidarToSdf("test_rays", sensorPose, 10,
      "/test/rays",
      3, -M_PI / 2, M_PI / 2,
      1, 0, 0,
      0.1, 5.0, true, false);
  ASSERT_NE(nullptr, sdf);

  gz::sensors::SensorFactory sf;
  auto sensor = sf.CreateSensor<gz::sensors::CpuLidarSensor>(sdf);
  ASSERT_NE(nullptr, sensor);

  auto rays = sensor->GenerateRays();
  ASSERT_EQ(3u, rays.size());

  // Each ray should start at range_min and end at range_max distance
  for (const auto &ray : rays) {
    EXPECT_NEAR(0.1, ray.first.Length(), 1e-6);
    EXPECT_NEAR(5.0, ray.second.Length(), 1e-6);
  }

  // Ray 0: azimuth = -π/2 → direction (0, -1, 0)
  EXPECT_NEAR(0.0, rays[0].first.X(), 1e-4);
  EXPECT_NEAR(-0.1, rays[0].first.Y(), 1e-4);
  EXPECT_NEAR(0.0, rays[0].second.X(), 1e-4);
  EXPECT_NEAR(-5.0, rays[0].second.Y(), 1e-4);

  // Ray 1: azimuth = 0 → direction (1, 0, 0)
  EXPECT_NEAR(0.1, rays[1].first.X(), 1e-6);
  EXPECT_NEAR(0.0, rays[1].first.Y(), 1e-6);
  EXPECT_NEAR(5.0, rays[1].second.X(), 1e-6);
  EXPECT_NEAR(0.0, rays[1].second.Y(), 1e-6);

  // Ray 2: azimuth = π/2 → direction (0, 1, 0)
  EXPECT_NEAR(0.0, rays[2].first.X(), 1e-4);
  EXPECT_NEAR(0.1, rays[2].first.Y(), 1e-4);
  EXPECT_NEAR(0.0, rays[2].second.X(), 1e-4);
  EXPECT_NEAR(5.0, rays[2].second.Y(), 1e-4);
}

/////////////////////////////////////////////////
TEST_F(CpuLidarSensorTest, GenerateRaysMultiLayer)
{
  gz::math::Pose3d sensorPose(gz::math::Vector3d::Zero,
      gz::math::Quaterniond::Identity);
  auto sdf = CpuLidarToSdf("test_multi", sensorPose, 10,
      "/test/multi",
      4, -M_PI / 4, M_PI / 4,
      3, -0.2, 0.2,
      0.5, 10.0, true, false);
  ASSERT_NE(nullptr, sdf);

  gz::sensors::SensorFactory sf;
  auto sensor = sf.CreateSensor<gz::sensors::CpuLidarSensor>(sdf);
  ASSERT_NE(nullptr, sensor);

  auto rays = sensor->GenerateRays();
  // 4 horizontal × 3 vertical = 12 rays
  ASSERT_EQ(12u, rays.size());

  for (const auto &ray : rays) {
    EXPECT_NEAR(0.5, ray.first.Length(), 1e-6);
    EXPECT_NEAR(10.0, ray.second.Length(), 1e-6);
  }
}
