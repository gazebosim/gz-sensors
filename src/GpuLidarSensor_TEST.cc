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
#include <ignition/msgs.hh>
#include <ignition/rendering.hh>
#include <ignition/sensors/Export.hh>
#include <ignition/sensors/Manager.hh>

#include <ignition/sensors/LidarSensor.hh>

sdf::ElementPtr LidarToSDF(std::string name, double update_rate, std::string topic,
    double horz_samples, double horz_resolution, double horz_min_angle, double horz_max_angle,
    double vert_samples, double vert_resolution, double vert_min_angle, double vert_max_angle,
    double range_resolution, double range_min, double range_max, bool always_on, bool visualize)
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

/////////////////////////////////////////////////
/// \brief Test Creation of a Ray sensor
TEST(LidarSensor_TEST, CreateLaser)
{
  // Setup ign-rendering with a scene
  auto *engine = ignition::rendering::engine("ogre");
  EXPECT_TRUE(engine != nullptr);

  ignition::rendering::ScenePtr scene = engine->CreateScene("scene");

  // Create a sensor manager
  ignition::sensors::Manager mgr;
  mgr.SetRenderingScene(scene);

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
  auto sensor = mgr.CreateSensor<ignition::sensors::LidarSensor>(
      lidarSDF);

  // Make sure the above dynamic cast worked.
  EXPECT_TRUE(sensor != nullptr);

  double angleRes = (sensor->AngleMax() - sensor->AngleMin()).Radian() /
                    sensor->RayCount();
  EXPECT_EQ(sensor->AngleMin(), ignition::math::Angle(-1.396263));
  EXPECT_EQ(sensor->AngleMax(), ignition::math::Angle(1.396263));
  EXPECT_NEAR(sensor->RangeMin(), 0.08, 1e-6);
  EXPECT_NEAR(sensor->RangeMax(), 10.0, 1e-6);
  EXPECT_NEAR(sensor->AngleResolution(), angleRes, 1e-3);
  EXPECT_NEAR(sensor->RangeResolution(), 0.01, 1e-3);
  EXPECT_EQ(sensor->RayCount(), 640);
  EXPECT_EQ(sensor->RangeCount(), 640);

  EXPECT_EQ(sensor->VerticalRayCount(), 1);
  EXPECT_EQ(sensor->VerticalRangeCount(), 1);
  EXPECT_EQ(sensor->VerticalAngleMin(), 0.0);
  EXPECT_EQ(sensor->VerticalAngleMax(), 0.0);

  EXPECT_TRUE(sensor->IsActive());

  // Set a callback on the lidar sensor to get a scan
//  ignition::common::ConnectionPtr connection =
//    sensor->ConnectNewLaserFrame(&OnNewLaserFrame);

  // listen to new laser frames
  float *scan = new float[sensor->RayCount()
      * sensor->VerticalRayCount() * 3];
  int scanCount = 0;
  ignition::common::ConnectionPtr connection =
    sensor->ConnectNewLaserFrame(
        std::bind(&::OnNewLaserFrame, &scanCount, scan,
          std::placeholders::_1, std::placeholders::_2,
          std::placeholders::_3, std::placeholders::_4,
          std::placeholders::_5));

  /*
  // listen to new laser frames
  float *scan = new float[sensor->RayCount()
      * sensor->VerticalRayCount() * 3];
  int scanCount = 0;

  // wait for a few laser scans
  int i = 0;
  while (scanCount < 10 && i < 300)
  {
    common::Time::MSleep(10);
    i++;
  }
  EXPECT_LT(i, 300);

  // Get all the range values
  std::vector<double> ranges;
  sensor->Ranges(ranges);
  EXPECT_EQ(ranges.size(), static_cast<size_t>(640));

  // Check that all the range values
  for (unsigned int i = 0; i < ranges.size(); ++i)
  {
    EXPECT_DOUBLE_EQ(ranges[i], ignition::math::INF_D);
    EXPECT_DOUBLE_EQ(sensor->Range(i), ranges[i]);
    EXPECT_NEAR(sensor->Retro(i), 0, 1e-6);
    EXPECT_EQ(sensor->Fiducial(i), -1);
  }

  delete [] scan;
  */
}

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
