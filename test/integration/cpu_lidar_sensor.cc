/*
 * Copyright (C) 2026 Open Source Robotics Foundation
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
#include <cstddef>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <sdf/sdf.hh>

#include <gz/msgs/laserscan.pb.h>
#include <gz/msgs/pointcloud_packed.pb.h>

#include <gz/common/Console.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/sensors/CpuLidarSensor.hh>
#include <gz/sensors/SensorFactory.hh>
#include <gz/transport/Node.hh>

#include "test_config.hh"  // NOLINT(build/include)
#include "TransportTestTools.hh"

/// \brief Helper function to create a cpu lidar sdf element
sdf::ElementPtr CpuLidarToSdf(const std::string &_name,
    const gz::math::Pose3d &_pose, const double _updateRate,
    const std::string &_topic, const double _horzSamples,
    const double _horzMinAngle, const double _horzMaxAngle,
    const double _vertSamples, const double _vertMinAngle,
    const double _vertMaxAngle, const double _rangeMin,
    const double _rangeMax, const bool _alwaysOn,
    const bool _visualize,
    const double _horzResolution = 1.0,
    const double _vertResolution = 1.0)
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
    << "            <resolution>" << _horzResolution << "</resolution>"
    << "            <min_angle>" << _horzMinAngle << "</min_angle>"
    << "            <max_angle>" << _horzMaxAngle << "</max_angle>"
    << "          </horizontal>"
    << "          <vertical>"
    << "            <samples>" << _vertSamples << "</samples>"
    << "            <resolution>" << _vertResolution << "</resolution>"
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

/////////////////////////////////////////////////
TEST_F(CpuLidarSensorTest, SetRaycastResults)
{
  // 3 horizontal rays, range [0.1, 5.0]
  gz::math::Pose3d sensorPose(gz::math::Vector3d::Zero,
      gz::math::Quaterniond::Identity);
  auto sdf = CpuLidarToSdf("test_results", sensorPose, 10,
      "/test/results",
      3, -M_PI / 4, M_PI / 4,
      1, 0, 0,
      0.1, 5.0, true, false);
  ASSERT_NE(nullptr, sdf);

  gz::sensors::SensorFactory sf;
  auto sensor = sf.CreateSensor<gz::sensors::CpuLidarSensor>(sdf);
  ASSERT_NE(nullptr, sensor);

  auto rays = sensor->GenerateRays();
  ASSERT_EQ(3u, rays.size());

  // Build results: hit at fraction 0.5, no hit, hit at fraction 0.02
  std::vector<gz::sensors::CpuLidarSensor::RayResult> results(3);

  // Ray 0: hit at fraction 0.5 → distance along ray = 0.5 * (5.0 - 0.1) + 0.1
  // Actually fraction is along the full ray from start to end:
  // range = range_min + fraction * (range_max - range_min)
  results[0].fraction = 0.5;
  results[0].point = rays[0].first + 0.5 * (rays[0].second - rays[0].first);

  // Ray 1: no hit
  results[1].fraction = std::numeric_limits<double>::quiet_NaN();
  results[1].point = {std::numeric_limits<double>::quiet_NaN(),
                      std::numeric_limits<double>::quiet_NaN(),
                      std::numeric_limits<double>::quiet_NaN()};

  // Ray 2: hit very close → fraction 0.0 (at start = range_min)
  results[2].fraction = 0.0;
  results[2].point = rays[2].first;

  sensor->SetRaycastResults(results);

  std::vector<double> ranges;
  sensor->Ranges(ranges);

  ASSERT_EQ(3u, ranges.size());
  // Ray 0: hit at midpoint between range_min and range_max
  EXPECT_NEAR(2.55, ranges[0], 1e-4);
  // Ray 1: no hit → +inf (REP-117)
  EXPECT_TRUE(std::isinf(ranges[1]));
  // Ray 2: hit at range_min
  EXPECT_NEAR(0.1, ranges[2], 1e-4);
}

/////////////////////////////////////////////////
TEST_F(CpuLidarSensorTest, PublishLaserScan)
{
  const std::string topic = "/test/cpu_lidar/scan";
  gz::math::Pose3d sensorPose(gz::math::Vector3d::Zero,
      gz::math::Quaterniond::Identity);
  auto sdf = CpuLidarToSdf("test_pub", sensorPose, 30, topic,
      5, -0.5, 0.5,
      1, 0, 0,
      0.1, 10.0, true, false);
  ASSERT_NE(nullptr, sdf);

  gz::sensors::SensorFactory sf;
  auto sensor = sf.CreateSensor<gz::sensors::CpuLidarSensor>(sdf);
  ASSERT_NE(nullptr, sensor);

  // Subscribe to LaserScan
  gz::transport::Node node;
  std::vector<gz::msgs::LaserScan> received;
  std::mutex mtx;
  node.Subscribe(topic,
    std::function<void(const gz::msgs::LaserScan &)>(
      [&](const gz::msgs::LaserScan &_msg) {
        std::lock_guard<std::mutex> lock(mtx);
        received.push_back(_msg);
      }));

  EXPECT_TRUE(sensor->HasConnections());

  // Build results: all hit at fraction 0.5
  std::vector<gz::sensors::CpuLidarSensor::RayResult> results(5);
  auto rays = sensor->GenerateRays();
  for (size_t i = 0; i < 5; ++i)
  {
    results[i].fraction = 0.5;
    results[i].point = rays[i].first +
      0.5 * (rays[i].second - rays[i].first);
  }
  sensor->SetRaycastResults(results);

  auto now = std::chrono::steady_clock::duration(
    std::chrono::milliseconds(100));
  EXPECT_TRUE(sensor->Update(now));

  // Wait for message
  for (int i = 0; i < 100 && received.empty(); ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

  std::lock_guard<std::mutex> lock(mtx);
  ASSERT_GE(received.size(), 1u);
  auto &msg = received.back();
  EXPECT_EQ(5, msg.count());
  EXPECT_NEAR(-0.5, msg.angle_min(), 1e-4);
  EXPECT_NEAR(0.5, msg.angle_max(), 1e-4);
  EXPECT_DOUBLE_EQ(0.1, msg.range_min());
  EXPECT_DOUBLE_EQ(10.0, msg.range_max());
  EXPECT_EQ(5, msg.ranges_size());
  for (int i = 0; i < msg.ranges_size(); ++i)
  {
    EXPECT_NEAR(5.05, msg.ranges(i), 1e-2);
  }
}

/////////////////////////////////////////////////
TEST_F(CpuLidarSensorTest, NoiseApplied)
{
  std::ostringstream stream;
  stream
    << "<?xml version='1.0'?>"
    << "<sdf version='1.6'>"
    << " <model name='m1'>"
    << "  <link name='link1'>"
    << "    <sensor name='noisy_lidar' type='lidar'>"
    << "      <topic>/test/cpu_lidar/noise</topic>"
    << "      <update_rate>30</update_rate>"
    << "      <ray>"
    << "        <scan>"
    << "          <horizontal>"
    << "            <samples>10</samples>"
    << "            <resolution>1</resolution>"
    << "            <min_angle>-0.5</min_angle>"
    << "            <max_angle>0.5</max_angle>"
    << "          </horizontal>"
    << "          <vertical>"
    << "            <samples>1</samples>"
    << "            <resolution>1</resolution>"
    << "            <min_angle>0</min_angle>"
    << "            <max_angle>0</max_angle>"
    << "          </vertical>"
    << "        </scan>"
    << "        <range>"
    << "          <min>0.1</min>"
    << "          <max>10.0</max>"
    << "          <resolution>0.01</resolution>"
    << "        </range>"
    << "        <noise>"
    << "          <type>gaussian</type>"
    << "          <mean>0.0</mean>"
    << "          <stddev>0.5</stddev>"
    << "        </noise>"
    << "      </ray>"
    << "      <always_on>1</always_on>"
    << "      <visualize>false</visualize>"
    << "    </sensor>"
    << "  </link>"
    << " </model>"
    << "</sdf>";

  sdf::SDFPtr sdfParsed(new sdf::SDF());
  sdf::init(sdfParsed);
  ASSERT_TRUE(sdf::readString(stream.str(), sdfParsed));

  auto sdfElem = sdfParsed->Root()->GetElement("model")->GetElement("link")
    ->GetElement("sensor");

  gz::sensors::SensorFactory sf;
  auto sensor = sf.CreateSensor<gz::sensors::CpuLidarSensor>(sdfElem);
  ASSERT_NE(nullptr, sensor);

  auto rays = sensor->GenerateRays();
  std::vector<gz::sensors::CpuLidarSensor::RayResult> results(10);
  for (size_t i = 0; i < 10; ++i)
  {
    results[i].fraction = 0.5;
    results[i].point = rays[i].first +
      0.5 * (rays[i].second - rays[i].first);
  }
  sensor->SetRaycastResults(results);

  std::vector<double> ranges;
  sensor->Ranges(ranges);
  ASSERT_EQ(10u, ranges.size());

  int diffCount = 0;
  for (size_t i = 0; i < ranges.size(); ++i)
  {
    if (std::abs(ranges[i] - 5.05) > 1e-6)
      ++diffCount;
    EXPECT_GT(ranges[i], 0.0);
    EXPECT_LT(ranges[i], 15.0);
  }
  EXPECT_GT(diffCount, 0) << "Noise should perturb at least some ranges";
}

/////////////////////////////////////////////////
TEST_F(CpuLidarSensorTest, PublishPointCloud)
{
  const std::string topic = "/test/cpu_lidar/pc";
  gz::math::Pose3d sensorPose(gz::math::Vector3d::Zero,
      gz::math::Quaterniond::Identity);
  auto sdf = CpuLidarToSdf("test_pc", sensorPose, 30, topic,
      5, -0.5, 0.5,
      1, 0, 0,
      0.1, 10.0, true, false);
  ASSERT_NE(nullptr, sdf);

  gz::sensors::SensorFactory sf;
  auto sensor = sf.CreateSensor<gz::sensors::CpuLidarSensor>(sdf);
  ASSERT_NE(nullptr, sensor);

  gz::transport::Node node;
  std::vector<gz::msgs::PointCloudPacked> received;
  std::mutex mtx;
  node.Subscribe(topic + "/points",
    std::function<void(const gz::msgs::PointCloudPacked &)>(
      [&](const gz::msgs::PointCloudPacked &_msg) {
        std::lock_guard<std::mutex> lock(mtx);
        received.push_back(_msg);
      }));

  auto rays = sensor->GenerateRays();
  std::vector<gz::sensors::CpuLidarSensor::RayResult> results(5);
  for (size_t i = 0; i < 5; ++i)
  {
    results[i].fraction = 0.5;
    results[i].point = rays[i].first +
      0.5 * (rays[i].second - rays[i].first);
  }
  sensor->SetRaycastResults(results);

  auto now = std::chrono::steady_clock::duration(
    std::chrono::milliseconds(100));
  EXPECT_TRUE(sensor->Update(now));

  for (int i = 0; i < 100 && received.empty(); ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

  std::lock_guard<std::mutex> lock(mtx);
  ASSERT_GE(received.size(), 1u);
  auto &msg = received.back();
  EXPECT_EQ(5u, msg.width());
  EXPECT_EQ(1u, msg.height());
  EXPECT_TRUE(msg.is_dense());
  EXPECT_GT(msg.data().size(), 0u);
}

/////////////////////////////////////////////////
TEST_F(CpuLidarSensorTest, PublishLaserScanIntensity)
{
  const std::string topic = "/test/cpu_lidar/scan_intensity";
  gz::math::Pose3d sensorPose(gz::math::Vector3d::Zero,
      gz::math::Quaterniond::Identity);
  auto sdf = CpuLidarToSdf("test_scan_intensity", sensorPose, 30, topic,
      3, -0.2, 0.2,
      1, 0, 0,
      0.1, 10.0, true, false);
  ASSERT_NE(nullptr, sdf);

  gz::sensors::SensorFactory sf;
  auto sensor = sf.CreateSensor<gz::sensors::CpuLidarSensor>(sdf);
  ASSERT_NE(nullptr, sensor);

  gz::transport::Node node;
  std::vector<gz::msgs::LaserScan> received;
  std::mutex mtx;
  node.Subscribe(topic,
    std::function<void(const gz::msgs::LaserScan &)>(
      [&](const gz::msgs::LaserScan &_msg) {
        std::lock_guard<std::mutex> lock(mtx);
        received.push_back(_msg);
      }));

  auto rays = sensor->GenerateRays();
  std::vector<gz::sensors::CpuLidarSensor::RayResult> results(3);
  for (size_t i = 0; i < 3; ++i)
  {
    results[i].fraction = 0.5;
    results[i].point = rays[i].first +
      0.5 * (rays[i].second - rays[i].first);
  }
  results[0].intensity = 0.1;
  results[1].intensity = 0.6;
  results[2].intensity = 1.0;
  sensor->SetRaycastResults(results);

  auto now = std::chrono::steady_clock::duration(
    std::chrono::milliseconds(100));
  EXPECT_TRUE(sensor->Update(now));

  for (int i = 0; i < 100 && received.empty(); ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

  std::lock_guard<std::mutex> lock(mtx);
  ASSERT_GE(received.size(), 1u);
  auto &msg = received.back();
  ASSERT_EQ(3, msg.intensities_size());
  EXPECT_NEAR(0.1, msg.intensities(0), 1e-6);
  EXPECT_NEAR(0.6, msg.intensities(1), 1e-6);
  EXPECT_NEAR(1.0, msg.intensities(2), 1e-6);
}

/////////////////////////////////////////////////
TEST_F(CpuLidarSensorTest, PublishPointCloudIntensity)
{
  const std::string topic = "/test/cpu_lidar/pc_intensity";
  gz::math::Pose3d sensorPose(gz::math::Vector3d::Zero,
      gz::math::Quaterniond::Identity);
  auto sdf = CpuLidarToSdf("test_pc_intensity", sensorPose, 30, topic,
      3, -0.2, 0.2,
      1, 0, 0,
      0.1, 10.0, true, false);
  ASSERT_NE(nullptr, sdf);

  gz::sensors::SensorFactory sf;
  auto sensor = sf.CreateSensor<gz::sensors::CpuLidarSensor>(sdf);
  ASSERT_NE(nullptr, sensor);

  gz::transport::Node node;
  std::vector<gz::msgs::PointCloudPacked> received;
  std::mutex mtx;
  node.Subscribe(topic + "/points",
    std::function<void(const gz::msgs::PointCloudPacked &)>(
      [&](const gz::msgs::PointCloudPacked &_msg) {
        std::lock_guard<std::mutex> lock(mtx);
        received.push_back(_msg);
      }));

  auto rays = sensor->GenerateRays();
  std::vector<gz::sensors::CpuLidarSensor::RayResult> results(3);
  for (size_t i = 0; i < 3; ++i)
  {
    results[i].fraction = 0.5;
    results[i].point = rays[i].first +
      0.5 * (rays[i].second - rays[i].first);
  }
  results[0].intensity = 0.2;
  results[1].intensity = 0.7;
  results[2].intensity = 1.2;
  sensor->SetRaycastResults(results);

  auto now = std::chrono::steady_clock::duration(
    std::chrono::milliseconds(100));
  EXPECT_TRUE(sensor->Update(now));

  for (int i = 0; i < 100 && received.empty(); ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

  std::lock_guard<std::mutex> lock(mtx);
  ASSERT_GE(received.size(), 1u);
  auto &msg = received.back();

  int intensityOffset = -1;
  for (int i = 0; i < msg.field_size(); ++i)
  {
    if (msg.field(i).name() == "intensity")
    {
      intensityOffset = static_cast<int>(msg.field(i).offset());
      break;
    }
  }
  ASSERT_GE(intensityOffset, 0);
  ASSERT_EQ(3u, msg.width());

  std::vector<float> publishedIntensities;
  publishedIntensities.reserve(msg.width());
  for (uint32_t i = 0; i < msg.width(); ++i)
  {
    const char *base = msg.data().data() + i * msg.point_step();
    float intensity = *reinterpret_cast<const float *>(base + intensityOffset);
    publishedIntensities.push_back(intensity);
  }

  EXPECT_NEAR(0.2f, publishedIntensities[0], 1e-6);
  EXPECT_NEAR(0.7f, publishedIntensities[1], 1e-6);
  EXPECT_NEAR(1.2f, publishedIntensities[2], 1e-6);
}

/////////////////////////////////////////////////
TEST_F(CpuLidarSensorTest, NoiseClamping)
{
  std::ostringstream stream;
  stream
    << "<?xml version='1.0'?>"
    << "<sdf version='1.6'>"
    << " <model name='m1'>"
    << "  <link name='link1'>"
    << "    <sensor name='clamped_lidar' type='lidar'>"
    << "      <topic>/test/cpu_lidar/clamping</topic>"
    << "      <update_rate>10</update_rate>"
    << "      <ray>"
    << "        <scan>"
    << "          <horizontal>"
    << "            <samples>1</samples>"
    << "            <resolution>1</resolution>"
    << "            <min_angle>0</min_angle>"
    << "            <max_angle>0</max_angle>"
    << "          </horizontal>"
    << "          <vertical>"
    << "            <samples>1</samples>"
    << "            <resolution>1</resolution>"
    << "            <min_angle>0</min_angle>"
    << "            <max_angle>0</max_angle>"
    << "          </vertical>"
    << "        </scan>"
    << "        <range>"
    << "          <min>0.1</min>"
    << "          <max>10.0</max>"
    << "          <resolution>0.01</resolution>"
    << "        </range>"
    << "        <noise>"
    << "          <type>gaussian</type>"
    << "          <mean>5.0</mean>"
    << "          <stddev>0.0</stddev>"
    << "        </noise>"
    << "      </ray>"
    << "      <always_on>1</always_on>"
    << "    </sensor>"
    << "  </link>"
    << " </model>"
    << "</sdf>";

  sdf::SDFPtr sdfParsed(new sdf::SDF());
  sdf::init(sdfParsed);
  ASSERT_TRUE(sdf::readString(stream.str(), sdfParsed));

  auto sdfElem = sdfParsed->Root()->GetElement("model")->GetElement("link")
    ->GetElement("sensor");

  gz::sensors::SensorFactory sf;
  auto sensor = sf.CreateSensor<gz::sensors::CpuLidarSensor>(sdfElem);
  ASSERT_NE(nullptr, sensor);

  auto rays = sensor->GenerateRays();
  ASSERT_EQ(1u, rays.size());

  // Supply a raycast result of 8.0.
  // range = range_min + fraction * (range_max - range_min)
  // 8.0 = 0.1 + fraction * (10.0 - 0.1)
  // 7.9 = fraction * 9.9
  // fraction = 7.9 / 9.9
  double rangeMin = 0.1;
  double rangeMax = 10.0;
  double targetRange = 8.0;
  double fraction = (targetRange - rangeMin) / (rangeMax - rangeMin);

  std::vector<gz::sensors::CpuLidarSensor::RayResult> results(1);
  results[0].fraction = fraction;
  results[0].point = rays[0].first +
    fraction * (rays[0].second - rays[0].first);
  sensor->SetRaycastResults(results);

  std::vector<double> ranges;
  sensor->Ranges(ranges);
  ASSERT_EQ(1u, ranges.size());

  // With mean 5.0, raw range 8.0 would become 13.0, but should be clamped
  // to max_range (10.0).
  EXPECT_NEAR(10.0, ranges[0], 1e-6);
}

/////////////////////////////////////////////////
TEST_F(CpuLidarSensorTest, DefaultTopic)
{
  gz::math::Pose3d sensorPose(gz::math::Vector3d::Zero,
      gz::math::Quaterniond::Identity);

  // Empty topic → falls back to /cpu_lidar
  auto sdf = CpuLidarToSdf("test_default_topic", sensorPose, 10,
      "", 3, -0.5, 0.5, 1, 0, 0, 0.08, 10.0, true, false);
  ASSERT_NE(nullptr, sdf);

  gz::sensors::SensorFactory sf;
  auto sensor = sf.CreateSensor<gz::sensors::CpuLidarSensor>(sdf);
  ASSERT_NE(nullptr, sensor);
  EXPECT_EQ("/cpu_lidar", sensor->Topic());

  // Custom topic is used as-is
  auto sdf2 = CpuLidarToSdf("test_custom_topic", sensorPose, 10,
      "/my/custom/topic", 3, -0.5, 0.5, 1, 0, 0, 0.08, 10.0, true, false);
  ASSERT_NE(nullptr, sdf2);

  auto sensor2 = sf.CreateSensor<gz::sensors::CpuLidarSensor>(sdf2);
  ASSERT_NE(nullptr, sensor2);
  EXPECT_EQ("/my/custom/topic", sensor2->Topic());
}

/////////////////////////////////////////////////
TEST_F(CpuLidarSensorTest, HasConnectionsFalse)
{
  gz::math::Pose3d sensorPose(gz::math::Vector3d::Zero,
      gz::math::Quaterniond::Identity);
  auto sdf = CpuLidarToSdf("test_no_conn", sensorPose, 30,
      "/test/cpu_lidar/no_conn",
      3, -0.5, 0.5, 1, 0, 0, 0.1, 10.0, true, false);
  ASSERT_NE(nullptr, sdf);

  gz::sensors::SensorFactory sf;
  auto sensor = sf.CreateSensor<gz::sensors::CpuLidarSensor>(sdf);
  ASSERT_NE(nullptr, sensor);

  // No subscribers → HasConnections is false
  EXPECT_FALSE(sensor->HasConnections());

  // Feed some results and try to update
  auto rays = sensor->GenerateRays();
  std::vector<gz::sensors::CpuLidarSensor::RayResult> results(3);
  for (size_t i = 0; i < 3; ++i)
  {
    results[i].fraction = 0.5;
    results[i].point = rays[i].first +
      0.5 * (rays[i].second - rays[i].first);
  }
  sensor->SetRaycastResults(results);

  // Update should return false when no connections
  auto now = std::chrono::steady_clock::duration(
    std::chrono::milliseconds(100));
  EXPECT_FALSE(sensor->Update(now));
}

/////////////////////////////////////////////////
TEST_F(CpuLidarSensorTest, NonUnitResolution)
{
  // Verify that the sensor loads and generates the correct number of rays
  // when horizontal/vertical resolution is not 1. Resolution is a render
  // oversampling hint; the published ray count is always equal to <samples>.
  gz::math::Pose3d sensorPose(gz::math::Vector3d::Zero,
      gz::math::Quaterniond::Identity);
  auto sdf = CpuLidarToSdf("test_resolution", sensorPose, 10,
      "/test/resolution",
      8, -M_PI / 4, M_PI / 4,
      4, -0.1, 0.1,
      0.1, 10.0, true, false,
      /*_horzResolution=*/2.0, /*_vertResolution=*/0.5);
  ASSERT_NE(nullptr, sdf);

  gz::sensors::SensorFactory sf;
  auto sensor = sf.CreateSensor<gz::sensors::CpuLidarSensor>(sdf);
  ASSERT_NE(nullptr, sensor);

  // RayCount / VerticalRayCount reflect <samples>, not <samples> * <resolution>
  EXPECT_EQ(8u, sensor->RayCount());
  EXPECT_EQ(4u, sensor->VerticalRayCount());

  auto rays = sensor->GenerateRays();
  ASSERT_EQ(32u, rays.size());  // 8 horizontal × 4 vertical

  for (const auto &ray : rays)
  {
    EXPECT_NEAR(0.1, ray.first.Length(), 1e-6);
    EXPECT_NEAR(10.0, ray.second.Length(), 1e-6);
  }
}
