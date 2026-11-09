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

#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include <gz/msgs/laserscan.pb.h>
#include <gz/transport/Node.hh>

#include <gz/math/Angle.hh>
#include <gz/math/Helpers.hh>

#include <gz/sensors/Export.hh>
#include <gz/sensors/Manager.hh>
#include <gz/sensors/Lidar.hh>

sdf::ElementPtr LidarToSDF(const std::string &name, double update_rate,
    const std::string &topic, double horz_samples, double horz_resolution,
    double horz_min_angle, double horz_max_angle, double vert_samples,
    double vert_resolution, double vert_min_angle, double vert_max_angle,
    double range_resolution, double range_min, double range_max,
    uint32_t visibility_mask, bool always_on, bool visualize)
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
    << "        <visibility_mask>" << visibility_mask << "</visibility_mask>"
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
    gz::common::Console::SetVerbosity(4);
  }
};

/////////////////////////////////////////////////
/// \brief Test Creation of a Lidar sensor
TEST(Lidar_TEST, CreateLaser)
{
  // Create a sensor manager
  gz::sensors::Manager mgr;

  // Create SDF describing a camera sensor
  const std::string name = "TestLidar";
  const std::string topic = "/gz/sensors/test/lidar";
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
  const uint32_t visibility_mask = 1234u;
  const bool always_on = 1;
  const bool visualize = 1;

  sdf::ElementPtr lidarSDF = LidarToSDF(name, update_rate, topic,
    horz_samples, horz_resolution, horz_min_angle, horz_max_angle,
    vert_samples, vert_resolution, vert_min_angle, vert_max_angle,
    range_resolution, range_min, range_max, visibility_mask, always_on,
    visualize);

  // Create a CameraSensor
  gz::sensors::Lidar *sensor = mgr.CreateSensor<gz::sensors::Lidar>(
      lidarSDF);

  EXPECT_FALSE(sensor->CreateLidar());

  // Make sure the above dynamic cast worked.
  ASSERT_NE(nullptr, sensor);

  double angleRes = (sensor->AngleMax() - sensor->AngleMin()).Radian() /
                    sensor->RayCount();
  EXPECT_EQ(sensor->AngleMin(), gz::math::Angle(-1.396263));
  EXPECT_EQ(sensor->AngleMax(), gz::math::Angle(1.396263));
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
  EXPECT_EQ(1234u, sensor->VisibilityMask());

  EXPECT_TRUE(sensor->IsActive());
}

TEST(Lidar_TEST, CreateLaserFailures)
{
  sdf::Sensor sdfSensor;
  sdfSensor.SetName("camera");
  sdfSensor.SetType(sdf::SensorType::CAMERA);
  sdf::Lidar sdfLidarSensor;

  gz::sensors::Lidar sensor;
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

  gz::sensors::Lidar sensor2;

  EXPECT_TRUE(sensor2.Load(sdfSensor));
  EXPECT_FALSE(sensor2.Load(sdfSensor));

  gz::sensors::Lidar sensor3;

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
  gz::sensors::Lidar sensor4;
  EXPECT_TRUE(sensor4.Load(sdfSensor));

  noise.SetType(sdf::NoiseType::GAUSSIAN_QUANTIZED);
  sdfLidarSensor.SetLidarNoise(noise);
  sdfSensor.SetLidarSensor(sdfLidarSensor);
  gz::sensors::Lidar sensor5;
  EXPECT_TRUE(sensor5.Load(sdfSensor));

  sensor.Update(std::chrono::steady_clock::duration(
    std::chrono::milliseconds(100)));
}

/////////////////////////////////////////////////
/// \brief With more ranges than rays (resolution > 1), the published scan
/// has one range per ray, the first ones of the buffer. PublishLidarScan
/// used to write RangeCount() x VerticalRangeCount() entries into a message
/// sized for the rays, past its end (AddressSanitizer: heap-buffer-overflow).
TEST(Lidar_TEST, PublishMoreRangesThanRays)
{
  sdf::Sensor sdfSensor;
  sdfSensor.SetName("more_ranges");
  sdfSensor.SetType(sdf::SensorType::LIDAR);
  sdfSensor.SetTopic("/lidar_more_ranges");
  sdf::Lidar sdfLidar;
  sdfLidar.SetHorizontalScanSamples(4);
  sdfLidar.SetHorizontalScanResolution(4);
  sdfLidar.SetVerticalScanSamples(1);
  sdfLidar.SetRangeMin(0.5);
  sdfLidar.SetRangeMax(10.0);
  sdfSensor.SetLidarSensor(sdfLidar);

  gz::sensors::Lidar sensor;
  ASSERT_TRUE(sensor.Load(sdfSensor));
  ASSERT_EQ(4u, sensor.RayCount());
  ASSERT_EQ(16u, sensor.RangeCount());
  // A buffer with every range, so that only the message could overflow.
  sensor.laserBuffer = new float[16 * 3];
  for (int i = 0; i < 16; ++i)
  {
    sensor.laserBuffer[i * 3] = 1.0f + i;
    sensor.laserBuffer[i * 3 + 1] = 101.0f + i;
    sensor.laserBuffer[i * 3 + 2] = 0.0f;
  }

  // Owned with the subscriber callback, which may still run after a failed
  // assertion has left this scope.
  struct Received
  {
    std::mutex mutex;
    std::vector<gz::msgs::LaserScan> scans;
  };
  auto received = std::make_shared<Received>();
  std::mutex &mutex = received->mutex;
  std::vector<gz::msgs::LaserScan> &scans = received->scans;
  std::function<void(const gz::msgs::LaserScan &)> cb =
      [received](const gz::msgs::LaserScan &_msg)
      {
        std::lock_guard<std::mutex> lock(received->mutex);
        received->scans.push_back(_msg);
      };
  gz::transport::Node node;
  ASSERT_TRUE(node.Subscribe("/lidar_more_ranges", cb));
  for (int i = 0; i < 200 && !sensor.HasConnections(); ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  ASSERT_TRUE(sensor.HasConnections());

  EXPECT_TRUE(sensor.PublishLidarScan(std::chrono::seconds(1)));
  for (int i = 0; i < 200; ++i)
  {
    {
      std::lock_guard<std::mutex> lock(mutex);
      if (!scans.empty())
        break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  std::lock_guard<std::mutex> lock(mutex);
  ASSERT_EQ(1u, scans.size());
  ASSERT_EQ(4, scans[0].ranges_size());
  ASSERT_EQ(4, scans[0].intensities_size());
  for (int i = 0; i < 4; ++i)
  {
    EXPECT_DOUBLE_EQ(1.0 + i, scans[0].ranges(i));
    EXPECT_DOUBLE_EQ(101.0 + i, scans[0].intensities(i));
  }
}

/////////////////////////////////////////////////
/// \brief With more ranges than rays in both directions, the message rows
/// are the buffer's rows, RangeCount() wide as the message's count says,
/// and the message holds the first RayCount() x VerticalRayCount() ranges:
/// with 4 x 2 rays at horizontal resolution 2, the first buffer row (8
/// ranges), not 4 ranges of each row.
TEST(Lidar_TEST, PublishMoreRangesThanRaysVertical)
{
  sdf::Sensor sdfSensor;
  sdfSensor.SetName("more_ranges_vertical");
  sdfSensor.SetType(sdf::SensorType::LIDAR);
  sdfSensor.SetTopic("/lidar_more_ranges_vertical");
  sdf::Lidar sdfLidar;
  sdfLidar.SetHorizontalScanSamples(4);
  sdfLidar.SetHorizontalScanResolution(2);
  sdfLidar.SetVerticalScanSamples(2);
  sdfLidar.SetRangeMin(0.5);
  sdfLidar.SetRangeMax(100.0);
  sdfSensor.SetLidarSensor(sdfLidar);

  gz::sensors::Lidar sensor;
  ASSERT_TRUE(sensor.Load(sdfSensor));
  ASSERT_EQ(8u, sensor.RangeCount());
  ASSERT_EQ(2u, sensor.VerticalRangeCount());
  // Range 10 * row + column + 1, intensity 100 more.
  sensor.laserBuffer = new float[16 * 3];
  for (int row = 0; row < 2; ++row)
  {
    for (int col = 0; col < 8; ++col)
    {
      const int i = row * 8 + col;
      sensor.laserBuffer[i * 3] = 10.0f * row + col + 1.0f;
      sensor.laserBuffer[i * 3 + 1] = 110.0f * row + col + 101.0f;
      sensor.laserBuffer[i * 3 + 2] = 0.0f;
    }
  }

  // Owned with the subscriber callback, which may still run after a failed
  // assertion has left this scope.
  struct Received
  {
    std::mutex mutex;
    std::vector<gz::msgs::LaserScan> scans;
  };
  auto received = std::make_shared<Received>();
  std::function<void(const gz::msgs::LaserScan &)> cb =
      [received](const gz::msgs::LaserScan &_msg)
      {
        std::lock_guard<std::mutex> lock(received->mutex);
        received->scans.push_back(_msg);
      };
  gz::transport::Node node;
  ASSERT_TRUE(node.Subscribe("/lidar_more_ranges_vertical", cb));
  for (int i = 0; i < 200 && !sensor.HasConnections(); ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  ASSERT_TRUE(sensor.HasConnections());

  EXPECT_TRUE(sensor.PublishLidarScan(std::chrono::seconds(1)));
  for (int i = 0; i < 200; ++i)
  {
    {
      std::lock_guard<std::mutex> lock(received->mutex);
      if (!received->scans.empty())
        break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  std::lock_guard<std::mutex> lock(received->mutex);
  ASSERT_EQ(1u, received->scans.size());
  const gz::msgs::LaserScan &scan = received->scans[0];
  EXPECT_EQ(8u, scan.count());
  EXPECT_EQ(2u, scan.vertical_count());
  ASSERT_EQ(8, scan.ranges_size());
  ASSERT_EQ(8, scan.intensities_size());
  for (int col = 0; col < 8; ++col)
  {
    EXPECT_DOUBLE_EQ(col + 1.0, scan.ranges(col));
    EXPECT_DOUBLE_EQ(col + 101.0, scan.intensities(col));
  }
}
